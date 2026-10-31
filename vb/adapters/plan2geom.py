"""Adapter: master build plan (SI, design frame) -> geometry-stage plan (mm, g; geomkit format).

Mapping
  airframe.lifting_surfaces[]  -> airframe.surfaces[] (wing: symmetric; v_tail: two dihedral panels;
                                  htail/canard: symmetric; fin: single vertical panel) + printed shell settings
  ...spars[]                   -> tubes[] (centre spars that start at y=0 on a symmetric surface become one
                                  straight tube through the fuselage; others are attached along the panel)
  airframe.booms[]             -> tubes[] (square or round, root..tip, mirrored)
  airframe.fuselage            -> airframe.fuselage (stations, superellipse exponent, printed shell, bays)
  components[]                 -> components[] (box / cylinder / point); structure_printed -> printed_mass_g
                                  targets of the shells; structure_tube -> masses of the linked tubes
  propulsion.units[].positions -> props[] (rotating discs for clearance checks and renders)
"""
from __future__ import annotations

import math

import numpy as np

from .. import plan as P

GROUP = {"structure_printed": "printed", "structure_tube": "carbon", "energy": "power", "propulsion": "propulsion",
         "servo": "servos", "avionics": "avionics", "payload": "payload", "wiring": "misc", "hardware": "misc",
         "ballast": "misc", "other": "misc"}
COLORS = {"printed": [0.86, 0.86, 0.83], "carbon": [0.12, 0.12, 0.13], "avionics": [0.10, 0.55, 0.20],
          "power": [0.85, 0.35, 0.10], "propulsion": [0.70, 0.70, 0.75], "servos": [0.15, 0.30, 0.80],
          "misc": [0.55, 0.45, 0.35], "payload": [0.55, 0.20, 0.65], "vtol": [0.80, 0.10, 0.10]}
MM = 1000.0


def _surface(ls, plan):
    kind = ls.get("kind", "wing")
    sym = ls.get("symmetric", True)
    span = ls["span"]
    panel = span / 2 if kind in ("wing", "htail", "canard") and sym else span
    skin = ls.get("skin") or {}
    mat = skin.get("material") or next((k for k, m in plan["materials"].items() if m["kind"] == "printed_polymer"), None)
    d = dict(name=ls["name"], kind=kind, airfoil=ls["airfoil"].lower(), root_chord=ls["root_chord"] * MM,
             tip_chord=ls["tip_chord"] * MM, panel_span=panel * MM,
             root_le=[ls.get("x_le_root", 0.0) * MM, 0.0, ls.get("z_le_root", 0.0) * MM],
             dihedral_deg=90.0 if kind == "fin" else ls.get("dihedral_deg", 0.0),
             twist_tip_deg=ls.get("twist_tip_deg", 0.0), symmetric=(kind != "fin") and sym,
             sections_per_side=ls.get("sections_per_side", 8 if kind == "wing" else 4),
             printed=dict(material=mat, wall_mm=skin.get("thickness", 0.00045) * MM, infill=skin.get("infill", 0.0)))
    if ls.get("sweep_le_deg") is not None and ls.get("sweep_c4_deg") is None:
        d["sweep_le_deg"] = ls["sweep_le_deg"]
    else:
        d["sweep_quarter_chord_deg"] = ls.get("sweep_c4_deg", 0.0) or 0.0
    return d


def _lin_camber_z(ls, x_c):
    """approximate camber-line height (fraction of chord) for straight centre tubes: use the airfoil."""
    try:
        import sys
        from pathlib import Path
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lib" / "geom"))
        from geomkit import geometry as G
        af = G.resample_airfoil(G.load_airfoil(ls["airfoil"].lower()), 100)
        i = int(np.argmin(af[:, 0]))
        up, lo = af[: i + 1][::-1], af[i:]
        return 0.5 * (np.interp(x_c, up[:, 0], up[:, 1]) + np.interp(x_c, lo[:, 0], lo[:, 1]))
    except Exception:
        return 0.0


def build(plan, render_title=None):
    af = plan["airframe"]
    mats = {k: m["density"] / 1000.0 for k, m in plan["materials"].items()}
    surfaces = [_surface(ls, plan) for ls in af["lifting_surfaces"]]
    w = P.main_wing(plan)
    mg = P.mac_geometry(w)
    var = plan["meta"].get("variant", "base")
    g = dict(name=P.slug(plan["meta"]["name"]), title=plan["meta"]["name"] + ("" if var == "base" else f" ({var})"),
             units=dict(length="mm", mass="g"),
             options=dict(vtol=False, calibrate_masses=False), materials=mats,
             groups={k: dict(color=v) for k, v in COLORS.items()})
    cg_t = P.val(plan.get("analysis", {}).get("cg_target_x"))
    tol = plan.get("analysis", {}).get("cg_tolerance_mac", 0.03) * mg["mac"]
    g["reference"] = dict(span=w["span"] * MM, wing_area_dm2=mg["area"] * 100, mac=mg["mac"] * MM,
                          cg_target_x=None if cg_t is None else cg_t * MM, cg_tolerance=round(tol * MM, 1))
    if cg_t is None:
        g["reference"].pop("cg_target_x")
    fus = af.get("fuselage") or {}
    gaf = dict(surfaces=surfaces)
    if fus.get("stations"):
        sk = fus.get("skin") or {}
        mat = sk.get("material") or surfaces[0]["printed"]["material"]
        gaf["fuselage"] = dict(x_nose=fus["stations"][0][0] * MM, x_tail=fus["stations"][-1][0] * MM,
                               stations=[[s[0] * MM, s[1] * MM, s[2] * MM, s[3] * MM] for s in fus["stations"]],
                               exponent=fus.get("exponent", 3.0),
                               printed=dict(material=mat, wall_mm=sk.get("thickness", 0.00045) * MM, infill=sk.get("infill", 0.0)))
    gaf["bays"] = {k: dict(center=[v * MM for v in b["center"]], size=[v * MM for v in b["size"]])
                   for k, b in (fus.get("bays") or {}).items()}
    g["airframe"] = gaf

    # ---- tubes from spars and booms
    tubes = []
    for ls in af["lifting_surfaces"]:
        kind = ls.get("kind", "wing")
        sym = ls.get("symmetric", True) and kind != "fin"
        for sp in ls.get("spars", []):
            t = sp["tube"]
            base = dict(od=t["od"] * MM, wall=(t["od"] - t["id"]) / 2 * MM, material=t.get("material"),
                        spar=sp["name"], shape=t.get("shape", "round"))
            xc = sp.get("x_c", 0.25)
            if sym and sp["y_start"] <= 1e-6 and kind in ("wing", "htail", "canard"):
                # straight tube through the centre section
                x = ls.get("x_le_root", 0.0) + xc * ls["root_chord"]
                z = ls.get("z_le_root", 0.0) + _lin_camber_z(ls, xc) * ls["root_chord"]
                tubes.append(dict(base, name=f"{ls['name']}_{sp['name']}", p0=[x * MM, -sp["y_end"] * MM, z * MM],
                                  p1=[x * MM, sp["y_end"] * MM, z * MM]))
            else:
                sides = ("R", "L") if sym else (None,)
                for tag in sides:
                    srf = f"{ls['name']}_{tag}" if tag else ls["name"]
                    tubes.append(dict(base, name=f"{ls['name']}_{sp['name']}" + (f"_{tag}" if tag else ""), attach=srf,
                                      s0=max(sp["y_start"], 0.005) * MM, s1=sp["y_end"] * MM, chord_frac=xc))
    for b in af.get("booms", []):
        t = b["tube"]
        r0, r1 = np.asarray(b["root"], float), np.asarray(b["tip"], float)
        ax = (r1 - r0) / np.linalg.norm(r1 - r0)
        p0, p1 = r0 - 0.015 * ax, r1 + 0.015 * ax           # 15 mm overhang past the motor centres
        mirror = abs(r0[1]) > 1e-6
        for sgn, tag in ((1, ""), (-1, "_mirror")) if mirror else ((1, ""),):
            nm = b["name"] if sgn > 0 else P.mirror_name(b["name"])
            q0, q1 = p0 * [1, sgn, 1], p1 * [1, sgn, 1]
            tubes.append(dict(name=nm, od=t["od"] * MM, wall=(t["od"] - t["id"]) / 2 * MM, shape=t.get("shape", "round"),
                              material=t.get("material"), p0=(q0 * MM).tolist(), p1=(q1 * MM).tolist(), boom=b["name"],
                              group="vtol" if plan["meta"].get("variant") not in (None, "base") else "carbon"))

    # ---- tube masses from linked components (plan is the source; geometry splits by volume)
    def vol(t):
        L = np.linalg.norm(np.subtract(t["p1"], t["p0"])) if "p0" in t else (t["s1"] - t["s0"])
        od, wall = t["od"], t["wall"]
        a = od ** 2 - (od - 2 * wall) ** 2 if t.get("shape") == "square" else math.pi / 4 * (od ** 2 - (od - 2 * wall) ** 2)
        return a * L
    linked = {}
    boom_names = {b["name"] for b in af.get("booms", [])}
    for c in plan["components"]:
        if c.get("category") != "structure_tube":
            continue
        keys = c.get("geom_spar") or (c["name"] if c["name"] in boom_names else None)
        if not keys:
            continue
        keys = [keys] if isinstance(keys, str) else list(keys)
        members = [t for t in tubes if t.get("spar") in keys or t.get("boom") in keys]
        if not members:
            continue
        total_g = P.val(c["mass"]) * 1000 * (2 if c.get("mirror_y") else 1)
        vs = sum(vol(t) for t in members)
        for t in members:
            t["mass"] = round(float(total_g * vol(t) / vs), 3)
            linked[t["name"]] = c["name"]
    for t in tubes:
        t["material"] = t.get("material") if t.get("material") in mats else next(iter(mats))
    g["tubes"] = tubes

    # ---- components
    comps, pmass, external, pointmass = [], {}, [], []
    for c in P.expanded_components(plan):
        cat = c.get("category", "other")
        m_g = c["mass"] * 1000
        if cat == "structure_printed" and c.get("geom_part"):
            gp = c["geom_part"]
            if c["name"].endswith("_L") or c["name"].endswith("_mirror"):
                gp = gp[:-2] + "_L" if gp.endswith("_R") else gp
            pmass[gp] = m_g
            continue
        if cat == "structure_tube" and (c.get("geom_spar") or c["name"] in boom_names
                                        or any(P.mirror_name(bn) == c["name"] for bn in boom_names)):
            continue
        grp = GROUP.get(cat, "misc")
        if c.get("variant_only") or (plan["meta"].get("variant") not in (None, "base") and c["name"].startswith("lift")):
            grp = "vtol"
        pos = [v * MM for v in c["position"]]
        e = dict(name=c["name"], group=grp, mass=round(m_g, 3), center=pos, label=c.get("label", c["name"]))
        dims = c.get("dims")
        if c.get("geom") is False or not dims:
            e["shape"] = "point"
        elif c.get("shape") == "cylinder":
            axis = np.asarray(c.get("axis", [0, 0, 1]), float)
            k = int(np.argmax(np.abs(axis)))
            e.update(shape="cylinder", length=dims[k] * MM, d=max(d for i, d in enumerate(dims) if i != k) * MM,
                     axis=axis.tolist())
        else:
            e.update(shape="box", size=[d * MM for d in dims])
        if c.get("bay"):
            e["bay"] = c["bay"]
        if c.get("external"):
            external.append(c["name"])
        if e["shape"] == "point":
            pointmass.append(c["name"])
        comps.append(e)
    g["components"] = comps
    g["printed_mass_g"] = pmass

    # ---- rotating discs
    props = []
    for u in plan["propulsion"]["units"]:
        ax = np.asarray(u.get("thrust_axis", [-1, 0, 0]), float)
        for i, pos in enumerate(u.get("positions", [])):
            props.append(dict(name=f"prop_{u['name']}{i + 1 if u['count'] > 1 else ''}", diameter=u["prop"]["diameter"] * MM,
                              center=[v * MM for v in pos], axis=ax.tolist(), label=u["prop"].get("model")))
    g["props"] = props
    g["external_parts"] = external
    g["point_masses"] = pointmass
    g["linked_tubes"] = linked
    return g
