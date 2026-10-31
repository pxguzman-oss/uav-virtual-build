"""Mass properties: per-part mass, CG, inertia; assembly totals; frame conversions;
JSBSim / Gazebo-SDF export.

Internal units: mm, g  ->  inertia in g*mm^2.  Output in SI (kg, m, kg*m^2).
Per-part mass models
  solid : uniform density through the closed mesh volume (boxes, motors, tubes)
  shell : thin shell of `wall_mm` on the mesh surface (+ optional infill fraction of
          the remaining enclosed volume) -- printed skins
  point : point mass, no own inertia
"""
from __future__ import annotations

import numpy as np
import trimesh

G_MM2_TO_KG_M2 = 1e-9  # g*mm^2 -> kg*m^2

# structural (x aft, y right, z up) -> body FRD (x fwd, y right, z down)
R_STRUCT_TO_FRD = np.diag([-1.0, 1.0, -1.0])
# structural -> FLU (x fwd, y left, z up): Gazebo / ROS link frame
R_STRUCT_TO_FLU = np.diag([-1.0, -1.0, 1.0])


def shell_props(mesh: trimesh.Trimesh, areal_density: float):
    """Thin-shell mass, CoM and inertia about CoM. areal_density in g/mm^2.
    Exact for a zero-thickness surface: per triangle  int(r r^T dA) = A/12 (sum v v^T + s s^T)."""
    tri = mesh.triangles                      # (n,3,3)
    A = mesh.area_faces                      # (n,)
    s = tri.sum(axis=1)                      # (n,3)
    m = areal_density * A.sum()
    com = (areal_density * (A[:, None] * s / 3.0).sum(0)) / m
    vv = np.einsum("nij,nik->njk", tri, tri)  # sum over vertices v v^T
    ss = np.einsum("ni,nj->nij", s, s)
    C = areal_density * ((A / 12.0)[:, None, None] * (vv + ss)).sum(0)   # second moment about origin
    C_cm = C - m * np.outer(com, com)
    I = np.trace(C_cm) * np.eye(3) - C_cm
    return m, com, I


def solid_props(mesh: trimesh.Trimesh, density: float):
    """density g/mm^3. Uses trimesh's exact polyhedral integrals."""
    mp = trimesh.Trimesh(mesh.vertices, mesh.faces, process=False)
    mp.density = density
    return float(mp.mass), np.asarray(mp.center_mass), np.asarray(mp.moment_inertia)


def part_raw(part, materials: dict):
    """Returns (mass_g, com_mm, I_cm_gmm2, mass_was_given)."""
    given = part.mass is not None
    if part.mass_model == "point" or part.mesh is None:
        return float(part.mass or 0.0), np.asarray(part.point, float), np.zeros((3, 3)), True
    mesh = part.mesh
    if part.mass_model == "shell":
        rho = materials[part.material] / 1000.0          # g/cm^3 -> g/mm^3
        t = part.wall_mm
        m_s, c_s, I_s = shell_props(mesh, rho * t)
        parts = [(m_s, c_s, I_s)]
        if part.infill:
            v_in = max(mesh.volume - mesh.area * t, 0.0)
            m_i, c_i, I_i = solid_props(mesh, part.infill * rho)
            k = v_in / mesh.volume                        # approx: remove shell volume
            parts.append((m_i * k, c_i, I_i * k))
        m, c, I = combine(parts)
    else:
        if given:
            dens = part.mass / mesh.volume
        elif part.material:
            dens = materials[part.material] / 1000.0
        else:
            raise ValueError(f"{part.name}: no mass and no material")
        m, c, I = solid_props(mesh, dens)
    if given:   # keep the mass distribution, honour the stated mass
        k = part.mass / m if m > 0 else 0.0
        m, I = m * k, I * k
    return m, c, I, given


def combine(items):
    """items: iterable of (m, com, I_about_own_com) -> (M, CG, I_about_CG)."""
    items = [(m, np.asarray(c, float), np.asarray(I, float)) for m, c, I in items if m > 0]
    M = sum(m for m, _, _ in items)
    cg = sum(m * c for m, c, _ in items) / M
    I = np.zeros((3, 3))
    for m, c, Ii in items:
        d = c - cg
        I += Ii + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return M, cg, I


def compute(parts, plan: dict):
    materials = plan["materials"]
    groups = plan.get("groups", {})
    calibrate = plan.get("options", {}).get("calibrate_masses", True)
    rows = []
    for p in parts:
        if p.role == "prop" and not p.mass:
            continue
        m, c, I, given = part_raw(p, materials)
        rows.append(dict(name=p.name, group=p.group, raw_mass=m, mass=m, com=c, I=I, given=given))
    # group calibration
    calib = {}
    for g, gd in groups.items():
        tgt = gd.get("total_mass_g")
        if tgt is None:
            continue
        members = [r for r in rows if r["group"] == g]
        fixed = sum(r["mass"] for r in members if r["given"])
        free = [r for r in members if not r["given"]]
        raw = sum(r["mass"] for r in free)
        calib[g] = dict(target_g=tgt, raw_estimate_g=round(raw + fixed, 1))
        if calibrate and free and raw > 0:
            k = (tgt - fixed) / raw
            for r in free:
                r["mass"] *= k
                r["I"] = r["I"] * k
            calib[g]["scale"] = round(k, 3)
    M, cg, I = combine((r["mass"], r["com"], r["I"]) for r in rows)
    return dict(rows=rows, mass_g=M, cg_mm=cg, I_cg_gmm2=I, calibration=calib)


def mac_geometry(info: dict, key="wing_R"):
    s = info["surfaces"][key]
    cr, ct, b = s["root_chord"], s["tip_chord"], s["span"]
    lam = ct / cr
    mac = 2 / 3 * cr * (1 + lam + lam**2) / (1 + lam)
    y_mac = b / 3 * (1 + 2 * lam) / (1 + lam)
    x_le_mac = s["tan_le"] * y_mac
    return dict(mac_mm=mac, y_mac_mm=y_mac, x_le_mac_mm=x_le_mac)


def summarize(res: dict, plan: dict, info: dict | None = None) -> dict:
    I_s = res["I_cg_gmm2"] * G_MM2_TO_KG_M2
    I_frd = R_STRUCT_TO_FRD @ I_s @ R_STRUCT_TO_FRD.T
    I_flu = R_STRUCT_TO_FLU @ I_s @ R_STRUCT_TO_FLU.T
    cg = res["cg_mm"]
    out = dict(
        mass_kg=res["mass_g"] / 1000.0,
        cg_struct_mm=cg.tolist(),
        frames=dict(struct="x aft, y right, z up; origin wing-root LE",
                    body_frd="x fwd, y right, z down (ArduPilot/aero body axes), about CG"),
        # flight-dynamics convention: Ixx.. positive moments, Ixz = +int(x z dm) in body axes
        body_axes_about_cg=dict(Ixx=I_frd[0, 0], Iyy=I_frd[1, 1], Izz=I_frd[2, 2],
                                Ixy=-I_frd[0, 1], Ixz=-I_frd[0, 2], Iyz=-I_frd[1, 2]),
        inertia_tensor_frd=I_frd.tolist(),
        inertia_tensor_flu=I_flu.tolist(),
        calibration=res["calibration"],
    )
    ref = plan.get("reference", {})
    if "cg_target_x" in ref:
        out["cg_target_x_mm"] = ref["cg_target_x"]
        out["cg_error_x_mm"] = cg[0] - ref["cg_target_x"]
        out["cg_ok"] = abs(out["cg_error_x_mm"]) <= ref.get("cg_tolerance", 5)
    if info and "wing_R" in info.get("surfaces", {}):
        mg = mac_geometry(info)
        out["mac"] = mg
        out["cg_pct_mac"] = 100 * (cg[0] - mg["x_le_mac_mm"]) / mg["mac_mm"]
        # how far to move the battery to hit the target
        if "cg_target_x" in ref:
            bat = [r for r in res["rows"] if "battery" in r["name"]]
            if bat:
                out["battery_shift_to_target_mm"] = -out["cg_error_x_mm"] * res["mass_g"] / bat[0]["mass"]
            x_nose = plan.get("airframe", {}).get("fuselage", {}).get("x_nose")
            if x_nose is not None and out["cg_error_x_mm"] > 0:
                xb = x_nose + 30.0   # ballast just behind the nose cap
                out["nose_ballast_g"] = res["mass_g"] * out["cg_error_x_mm"] / (ref["cg_target_x"] - xb)
                out["nose_ballast_x_mm"] = xb
    out["parts"] = [dict(name=r["name"], group=r["group"], mass_g=round(r["mass"], 2),
                         raw_mass_g=round(r["raw_mass"], 2), cg_mm=np.round(r["com"], 1).tolist()) for r in res["rows"]]
    return out


def jsbsim_xml(summary: dict) -> str:
    """<mass_balance> block. Verified against JSBSim src/models/FGMassBalance.cpp (ReadInertiaMatrix):
    with negated_crossproduct_inertia="false" JSBSim builds the BODY-axis tensor as
    J = [[ixx, ixy, -ixz], [ixy, iyy, iyz], [-ixz, iyz, izz]], i.e. ixz = +int(x z dm) in body
    axes (= our Ixz) while ixy/iyz are tensor entries (= -int(x y dm), -int(y z dm)).
    The CG <location> is in the structural frame (x aft, y right, z up), metres."""
    b = summary["body_axes_about_cg"]
    ixy_s, iyz_s = -b["Ixy"], -b["Iyz"]
    cg = np.asarray(summary["cg_struct_mm"]) / 1000.0
    return f"""<mass_balance negated_crossproduct_inertia="false">
  <ixx unit="KG*M2"> {b['Ixx']:.6f} </ixx>
  <iyy unit="KG*M2"> {b['Iyy']:.6f} </iyy>
  <izz unit="KG*M2"> {b['Izz']:.6f} </izz>
  <ixy unit="KG*M2"> {ixy_s:.6f} </ixy>
  <ixz unit="KG*M2"> {b['Ixz']:.6f} </ixz>
  <iyz unit="KG*M2"> {iyz_s:.6f} </iyz>
  <emptywt unit="KG"> {summary['mass_kg']:.4f} </emptywt>
  <location name="CG" unit="M">
    <x> {cg[0]:.4f} </x> <y> {cg[1]:.4f} </y> <z> {cg[2]:.4f} </z>
  </location>
</mass_balance>
"""


def sdf_inertial(summary: dict) -> str:
    """Gazebo SDF <inertial> in a FLU link frame whose origin is the wing-root LE.
    SDF ixy/ixz/iyz are tensor entries (= -int(xy dm))."""
    I = np.asarray(summary["inertia_tensor_flu"])
    cg = R_STRUCT_TO_FLU @ (np.asarray(summary["cg_struct_mm"]) / 1000.0)
    return f"""<inertial>
  <pose>{cg[0]:.4f} {cg[1]:.4f} {cg[2]:.4f} 0 0 0</pose>
  <mass>{summary['mass_kg']:.4f}</mass>
  <inertia>
    <ixx>{I[0,0]:.6f}</ixx> <ixy>{I[0,1]:.6f}</ixy> <ixz>{I[0,2]:.6f}</ixz>
    <iyy>{I[1,1]:.6f}</iyy> <iyz>{I[1,2]:.6f}</iyz> <izz>{I[2,2]:.6f}</izz>
  </inertia>
</inertial>
"""
