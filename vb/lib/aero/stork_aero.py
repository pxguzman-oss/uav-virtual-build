#!/usr/bin/env python
"""
Low-order aerodynamic analysis of the Flightory "Stork" (fixed-wing Phase 0 or VTOL variant) from a
PARAMETRIC definition (the geometry stage's stork.yaml), producing an aero dataset JSON for the flight sim.

Tools (all run here, see REPORT.md):
  * AeroSandbox 4.2 AeroBuildup (NeuralFoil section polars + fuselage/tail empirical buildup):
      - CL, CD, Cm vs alpha (-20..+30 deg, post-stall via ASB 360-deg model), CY/Cl/Cn vs beta,
      - drag breakdown, CLmax / stall, L/D, all stability derivatives (finite differences).
  * AVL 3.40 (built from github.com/mayerrobert/avl): VLM stability + rate + CONTROL derivatives
      for every physical surface (aileron_L/R, flap_L/R, ruddervator_L/R; + = trailing edge down).
  * AeroSandbox VLM: independent cross-check of CLa / Cma / Cnb (no viscous effects).
  * Extra parasite items not modelled by AeroBuildup (VTOL booms are modelled as bodies; stopped lift
    props + lift motors, excrescences/roughness for printed skin) as CD*A increments.

Usage:
  /home/claude/venvs/aero/bin/python stork_aero.py --yaml /home/claude/research/geom/stork.yaml \
      [--vtol] [--mass 1.775] [--out out/stork_p0] [--avl tools/bin/avl]
Outputs (in --out):
  aero_dataset.json  (validates against aero_dataset.schema.json)
  sim_aero_block.json (drop-in 'aero' + 'surfaces[].derivatives_per_rad' for sim/aircraft_model.schema.json)
  stork.avl, avl_*.st, plots *.png, log
"""
import argparse, copy, datetime, json, math, os, re, subprocess, sys, time
import numpy as np
import yaml
import aerosandbox as asb
import aerosandbox.numpy as anp

HERE = os.path.dirname(os.path.abspath(__file__))
DEG = math.pi / 180

# ----------------------------------------------------------------------------------------------
# Defaults for items the geometry YAML does not (yet) define. All marked as ASSUMED in the output.
# ----------------------------------------------------------------------------------------------
DEFAULTS = {
    "controls": {
        # spanwise extents as y [m] from centreline (per side); hinge as x/c
        "flap":    {"y0": 0.060, "y1": 0.365, "hinge_xc": 0.75, "max_deg": 30},
        "aileron": {"y0": 0.375, "y1": 0.770, "hinge_xc": 0.75, "max_deg": 20},
        "ruddervator": {"s0": 0.0, "s1": 1.0, "hinge_xc": 0.70, "max_deg": 25},  # s = fraction of panel span
    },
    "parasite": {
        # Additional drag areas [m^2] not captured by AeroBuildup
        "excrescence_factor": 1.10,      # +10 % on viscous CD for printed-skin roughness, seams, hinge gaps, servo horns
        "pusher_prop_stopped_or_windmill_CDA": 0.0,  # powered: 0 (prop thrust is net); glide: ~3e-4
        "vtol_lift_motor_CD": 0.8,       # bluff cylinder, motor frontal area = d * length
        "vtol_stopped_prop_CD": 1.2,     # flat plate on frontal projection of a stopped blade
        "vtol_prop_blade_mean_chord_frac": 0.09,   # mean chord / D for 7" props
        "vtol_prop_pitch_deg_at_0p75R": 15.0,
        "vtol_prop_parking": "crosswise_worst",     # 'aligned' (both blades fore-aft) or 'crosswise_worst'
    },
}


# ================================== geometry =========================================================
def load_params(path, vtol=False):
    p = yaml.safe_load(open(path))
    mm = 1e-3
    a = p["airframe"]
    w = a["wing"]
    g = {
        "name": p.get("name", "stork") + ("_vtol" if vtol else "_p0"),
        "source_yaml": os.path.abspath(path),
        "vtol": vtol,
        "wing": dict(airfoil=w["airfoil"], span=w["span"] * mm, cr=w["root_chord"] * mm, ct=w["tip_chord"] * mm,
                     sweep_c4_deg=w.get("sweep_quarter_chord_deg", 0.0), dihedral_deg=w.get("dihedral_deg", 0.0),
                     twist_tip_deg=w.get("twist_tip_deg", 0.0), z_root=w.get("z_root", 0.0) * mm),
        "fuse": dict(stations=[[s[0] * mm, s[1] * mm, s[2] * mm, s[3] * mm] for s in a["fuselage"]["stations"]],
                     exponent=a["fuselage"].get("exponent", 2.0)),
        "vtail": dict(airfoil=a["vtail"]["airfoil"], x_le=a["vtail"]["x_root_le"] * mm, z=a["vtail"]["z_root"] * mm,
                      cr=a["vtail"]["root_chord"] * mm, ct=a["vtail"]["tip_chord"] * mm,
                      panel_span=a["vtail"]["panel_span"] * mm, dihedral_deg=a["vtail"]["dihedral_deg"],
                      sweep_le_deg=a["vtail"]["sweep_le_deg"]),
        "ref": dict(S=p["reference"]["wing_area_dm2"] * 0.01, b=p["reference"]["span"] * mm,
                    c=p["reference"]["mac"] * mm, x_cg=p["reference"]["cg_target_x"] * mm),
        "controls": copy.deepcopy(DEFAULTS["controls"]),
        "parasite": copy.deepcopy(DEFAULTS["parasite"]),
        "assumed": ["control-surface extents/hinges (DEFAULTS.controls)", "parasite extras (DEFAULTS.parasite)",
                    "moment reference z = 0 (fuselage axis) unless --zref given"],
    }
    if vtol and "vtol" in p:
        v = p["vtol"]
        g["vtol_items"] = dict(boom_side=v["boom"]["outer"] * mm, boom_len=v["boom"]["length"] * mm,
                               boom_y=v["boom"]["y"] * mm, boom_z=v["boom"]["z"] * mm,
                               boom_xc=v["boom"]["x_center"] * mm,
                               motor_d=v["lift_motor"]["d"] * mm, motor_len=v["lift_motor"]["length"] * mm,
                               prop_d=v["prop_diameter"] * mm, n_rotors=4)
    return g


def wing_section_y(g):
    """Spanwise stations (per side) incl. control-surface breaks."""
    c = g["controls"]
    ys = {0.0, g["wing"]["span"] / 2}
    for k in ("flap", "aileron"):
        if k in c:
            ys |= {c[k]["y0"], c[k]["y1"]}
    return sorted(ys)


def wing_station(g, y):
    w = g["wing"]
    s = w["span"] / 2
    eta = abs(y) / s
    chord = w["cr"] + (w["ct"] - w["cr"]) * eta
    x_c4 = 0.25 * w["cr"] + abs(y) * math.tan(w["sweep_c4_deg"] * DEG)
    xle = x_c4 - 0.25 * chord
    zle = w["z_root"] + abs(y) * math.tan(w["dihedral_deg"] * DEG)
    twist = w["twist_tip_deg"] * eta
    return xle, zle, chord, twist


def vtail_station(g, s_frac, side=+1):
    v = g["vtail"]
    s = v["panel_span"] * s_frac
    chord = v["cr"] + (v["ct"] - v["cr"]) * s_frac
    xle = v["x_le"] + s * math.tan(v["sweep_le_deg"] * DEG)
    y = side * s * math.cos(v["dihedral_deg"] * DEG)
    z = v["z"] + s * math.sin(v["dihedral_deg"] * DEG)
    return xle, y, z, chord


def build_asb(g, deflections=None):
    """AeroSandbox Airplane. deflections: dict name->deg for asb control surfaces (used by VLM only)."""
    d = deflections or {}
    af = asb.Airfoil(g["wing"]["airfoil"])
    taf = asb.Airfoil(g["vtail"]["airfoil"])
    c = g["controls"]
    ys = wing_section_y(g)
    wings = []
    xsecs = []
    for y in ys:
        xle, zle, ch, tw = wing_station(g, y)
        xsecs.append(asb.WingXSec(xyz_le=[xle, y, zle], chord=ch, twist=tw, airfoil=af))
    wings.append(asb.Wing(name="Wing", symmetric=True, xsecs=xsecs))
    # V-tail panels: model right panel symmetric (ASB mirrors across XZ plane -> V)
    vx = []
    for s in (0.0, 1.0):
        xle, y, z, ch = vtail_station(g, s, +1)
        vx.append(asb.WingXSec(xyz_le=[xle, y, z], chord=ch, airfoil=taf))
    wings.append(asb.Wing(name="V-tail", symmetric=True, xsecs=vx))
    fus = []
    st = g["fuse"]["stations"]
    fus.append(asb.Fuselage(name="Fuselage", xsecs=[
        asb.FuselageXSec(xyz_c=[x, 0, zc], width=max(wd, 1e-3), height=max(h, 1e-3), shape=g["fuse"]["exponent"])
        for x, wd, h, zc in st]))
    if g.get("vtol") and "vtol_items" in g:
        v = g["vtol_items"]
        r = v["boom_side"] / 2 * 1.13  # equal-area circle for a square tube
        for side in (-1, 1):
            x0 = v["boom_xc"] - v["boom_len"] / 2
            x1 = v["boom_xc"] + v["boom_len"] / 2
            fus.append(asb.Fuselage(name=f"Boom_{'L' if side < 0 else 'R'}", xsecs=[
                asb.FuselageXSec(xyz_c=[x, side * v["boom_y"], v["boom_z"]], radius=rr)
                for x, rr in ((x0, 0.2 * r), (x0 + 0.01, r), (x1 - 0.01, r), (x1, 0.2 * r))]))
    return asb.Airplane(name=g["name"], xyz_ref=[g["ref"]["x_cg"], 0, g.get("zref", 0.0)], wings=wings,
                        fuselages=fus, s_ref=g["ref"]["S"], c_ref=g["ref"]["c"], b_ref=g["ref"]["b"])


def extra_parasite_CDA(g):
    """Drag areas [m^2] not modelled in AeroBuildup. Returns dict item->CDA."""
    pp = g["parasite"]
    out = {}
    if pp["pusher_prop_stopped_or_windmill_CDA"]:
        out["pusher_prop"] = pp["pusher_prop_stopped_or_windmill_CDA"]
    if g.get("vtol") and "vtol_items" in g:
        v = g["vtol_items"]
        n = v["n_rotors"]
        out["vtol_lift_motors"] = n * pp["vtol_lift_motor_CD"] * v["motor_d"] * v["motor_len"]
        R = v["prop_d"] / 2
        cbar = pp["vtol_prop_blade_mean_chord_frac"] * v["prop_d"]
        if pp["vtol_prop_parking"] == "aligned":
            # blades fore-aft: frontal area ~ blade thickness (12 % c) x 0.5 R near hub + hub
            frontal = 2 * 0.12 * cbar * 0.5 * R + 0.012 * 0.010
        else:
            # one blade pair crosswise: projected = 2 * R * cbar * sin(pitch) (+ hub)
            frontal = 2 * R * cbar * math.sin(pp["vtol_prop_pitch_deg_at_0p75R"] * DEG) + 0.012 * 0.010
        out["vtol_stopped_props"] = n * pp["vtol_stopped_prop_CD"] * frontal
    return out


# ================================== AeroBuildup ======================================================
def run_buildup(ap, g, V, alphas, beta=0.0, stab=False):
    t0 = time.time()
    op = asb.OperatingPoint(velocity=V, alpha=np.asarray(alphas, float), beta=beta)
    ab = asb.AeroBuildup(ap, op, xyz_ref=ap.xyz_ref)
    r = ab.run_with_stability_derivatives() if stab else ab.run()
    r = {k: (np.atleast_1d(np.asarray(v, float)) if not isinstance(v, list) else v) for k, v in r.items()}
    r["_wall_s"] = time.time() - t0
    return r


def viscous_corrections(g, r, V):
    """Apply excrescence factor to profile drag and add extra CDA items."""
    S = g["ref"]["S"]
    q = 0.5 * 1.225 * V ** 2
    CDp = np.asarray(r["D_profile"]) / (q * S)
    CDi = np.asarray(r["D_induced"]) / (q * S)
    extra = sum(extra_parasite_CDA(g).values()) / S
    CD = CDi + CDp * g["parasite"]["excrescence_factor"] + extra
    return CD, CDp, CDi, extra


# ================================== AVL ==============================================================
def write_avl(g, path, afdir):
    """AVL input. Left/right wing and left/right V-tail panels are separate surfaces so each physical control
    surface gets its own control variable; sections ordered by increasing y => positive deflection = TE down."""
    os.makedirs(afdir, exist_ok=True)
    for name in (g["wing"]["airfoil"], g["vtail"]["airfoil"]):
        af = asb.Airfoil(name).repanel(80)
        with open(os.path.join(afdir, f"{name}.dat"), "w") as f:
            f.write(name + "\n" + "".join(f" {x:.6f} {y:.6f}\n" for x, y in af.coordinates))
    S, c, b = g["ref"]["S"], g["ref"]["c"], g["ref"]["b"]
    L = [f"{g['name']}", "0.0", "0 0 0.0", f"{S:.5f} {c:.5f} {b:.5f}",
         f"{g['ref']['x_cg']:.5f} 0.0 {g.get('zref', 0.0):.5f}", "0.0", ""]
    ctl = g["controls"]
    ys = wing_section_y(g)
    controls_order = []

    def ctrl_lines(name, xh):
        return ["CONTROL", f"{name} 1.0 {xh:.3f} 0.0 0.0 0.0 1.0"]

    for side, tag in ((+1, "R"), (-1, "L")):
        L += ["#" + "=" * 60, "SURFACE", f"Wing_{tag}", "10 1.0",
              "COMPONENT", "1", ""]
        yy = ys if side > 0 else list(reversed(ys))
        for i, y in enumerate(yy):
            xle, zle, ch, tw = wing_station(g, y)
            # vortices to next section: ~1 per 25 mm, min 1 (equal spacing within a segment)
            ns = max(1, int(round(abs(yy[i + 1] - y) / 0.025))) if i + 1 < len(yy) else 1
            L += ["SECTION", f"{xle:.5f} {side * y:.5f} {zle:.5f} {ch:.5f} {tw:.3f} {ns} 0.0", "AFILE",
                  os.path.relpath(os.path.join(afdir, f"{g['wing']['airfoil']}.dat"), os.path.dirname(os.path.abspath(path)))]
            for cname in ("flap", "aileron"):
                if cname not in ctl:
                    continue
                cc = ctl[cname]
                if cc["y0"] - 1e-9 <= y <= cc["y1"] + 1e-9:
                    L += ctrl_lines(f"{cname}_{tag}", cc["hinge_xc"])
            L.append("")
    for side, tag in ((+1, "R"), (-1, "L")):
        L += ["#" + "=" * 60, "SURFACE", f"Vtail_{tag}", "8 1.0 12 1.0", "COMPONENT", "2", ""]
        ss = (0.0, 1.0) if side > 0 else (1.0, 0.0)
        for s in ss:
            xle, y, z, ch = vtail_station(g, s, side)
            L += ["SECTION", f"{xle:.5f} {y:.5f} {z:.5f} {ch:.5f} 0.0", "NACA", g["vtail"]["airfoil"].lower().replace("naca", "")]
            L += ctrl_lines(f"rv_{tag}", ctl["ruddervator"]["hinge_xc"])
            L.append("")
    # Fuselage as AVL BODY (slender-body source line), side-view outline file
    st = g["fuse"]["stations"]
    bfile = os.path.join(afdir, "fuselage_body.dat")
    xs = np.array([s[0] for s in st]); wd = np.array([s[1] for s in st]); h = np.array([s[2] for s in st]); zc = np.array([s[3] for s in st])
    deq = np.sqrt(wd * h)  # equivalent diameter
    xn = np.linspace(xs.min(), xs.max(), 41)
    dd = np.interp(xn, xs, deq)
    with open(bfile, "w") as f:
        f.write("fuselage\n")
        for x, d_ in zip(xn[::-1], dd[::-1]):
            f.write(f" {x:.5f} {d_ / 2:.5f}\n")
        for x, d_ in zip(xn[1:], dd[1:]):
            f.write(f" {x:.5f} {-d_ / 2:.5f}\n")
    L += ["#" + "=" * 60, "BODY", "Fuselage", "24 1.0", "BFIL", os.path.relpath(bfile, os.path.dirname(os.path.abspath(path))), ""]
    open(path, "w").write("\n".join(L) + "\n")
    names = [f"flap_R", "aileron_R", "flap_L", "aileron_L", "rv_R", "rv_L"]
    return names


def avl_control_order(path):
    """Control variables are numbered in order of first appearance in the file."""
    seen = []
    for line in open(path):
        m = re.match(r"^(\w+)\s+1\.0\s+[\d.]+\s+0\.0 0\.0 0\.0", line.strip())
        if m and m.group(1) not in seen:
            seen.append(m.group(1))
    return seen


def run_avl(avl_exe, avl_file, alphas, workdir, tag="st"):
    """Run AVL at several alpha; return list of parsed ST outputs."""
    # AVL (Fortran) truncates file names at 80 characters: run in workdir with RELATIVE names
    cmds = [f"load {os.path.relpath(avl_file, workdir)}", "oper"]
    outs = []
    for a in alphas:
        fn = os.path.join(workdir, f"avl_{tag}_a{a:+05.1f}.st")
        if os.path.exists(fn):
            os.remove(fn)
        cmds += [f"a a {a}", "x", "st", os.path.basename(fn)]
        outs.append(fn)
    cmds += ["", "quit"]
    t0 = time.time()
    p = subprocess.run([avl_exe], input="\n".join(cmds) + "\n", text=True, capture_output=True, timeout=600, cwd=workdir)
    dt = time.time() - t0
    open(os.path.join(workdir, f"avl_{tag}.log"), "w").write(p.stdout + p.stderr)
    res = [parse_avl_st(fn) for fn in outs]
    return res, dt


def parse_avl_st(fn):
    txt = open(fn).read()
    d = {}
    for m in re.finditer(r"([A-Za-z][A-Za-z0-9_']*(?: ?d\d\d)?)\s*=\s*(-?\d+\.\d+(?:E[-+]?\d+)?)", txt):
        k = m.group(1).replace("'", "p").replace(" ", "")
        if k not in d:
            d[k] = float(m.group(2))
    # control-variable mapping lines e.g. "flap_R   d01"
    d["_controls"] = {}
    for line in txt.splitlines():
        pairs = re.findall(r"(\w+)\s+(d\d\d)(?=\s|$)", line)
        if pairs and "=" not in line:
            d["_controls"].update({dn: nm for nm, dn in pairs})
    return d


# ================================== main analysis =====================================================
def analyse(g, mass, avl_exe, outdir, V_cruise=14.0, speeds=(10.0, 14.0, 20.0)):
    os.makedirs(outdir, exist_ok=True)
    log = {}
    ap = build_asb(g)
    S, b, c = g["ref"]["S"], g["ref"]["b"], g["ref"]["c"]
    rho = 1.225
    W = mass * 9.81

    # ---- 1. AeroBuildup alpha sweeps at several speeds (Re effect) -------------------------------
    alphas = np.concatenate([np.arange(-20, -10, 2.0), np.arange(-10, 20.01, 0.5), np.arange(22, 30.01, 2.0)])
    tables = {}
    for V in speeds:
        r = run_buildup(ap, g, V, alphas)
        CD, CDp, CDi, extra = viscous_corrections(g, r, V)
        tables[V] = dict(alpha_deg=alphas.tolist(), CL=r["CL"].tolist(), CD=CD.tolist(), Cm=r["Cm"].tolist(),
                         CD_profile=(CDp * g["parasite"]["excrescence_factor"]).tolist(), CD_induced=CDi.tolist(),
                         CD_extra=float(extra), wall_s=r["_wall_s"])
        log[f"buildup_V{V}_s"] = r["_wall_s"]
    # ---- 2. beta sweep at cruise alpha ----------------------------------------------------------
    T = tables[V_cruise]
    al = np.array(T["alpha_deg"]); CLa_arr = np.array(T["CL"])
    CL_cruise = W / (0.5 * rho * V_cruise ** 2 * S)
    lin = (al > -4) & (al < 8)
    a_cruise = float(np.interp(CL_cruise, CLa_arr[lin], al[lin]))
    betas = np.arange(-20, 20.01, 2.0)
    rb = [run_buildup(ap, g, V_cruise, [a_cruise], beta=bb) for bb in betas]
    beta_tab = dict(alpha_deg=a_cruise, beta_deg=betas.tolist(),
                    CY=[float(x["CY"][0]) for x in rb], Cl=[float(x["Cl"][0]) for x in rb],
                    Cn=[float(x["Cn"][0]) for x in rb],
                    CD=[float(viscous_corrections(g, x, V_cruise)[0][0]) for x in rb])
    # ---- 3. AeroBuildup stability derivatives at cruise -----------------------------------------
    rs = run_buildup(ap, g, V_cruise, [a_cruise], stab=True)
    ab_der = {k: float(np.asarray(rs[k]).ravel()[0]) for k in
              ["CLa", "Cma", "CYb", "Clb", "Cnb", "CLp", "Clp", "Cnp", "CYp", "CLq", "Cmq", "CYr", "Clr", "Cnr", "x_np"]}
    log["buildup_stab_s"] = rs["_wall_s"]
    # drag breakdown at cruise
    r1 = run_buildup(ap, g, V_cruise, [a_cruise])
    q = 0.5 * rho * V_cruise ** 2
    br = {}
    for comp in r1["wing_aero_components"]:
        pass
    names_w = [w.name for w in ap.wings]; names_f = [f.name for f in ap.fuselages]
    for n, comp in zip(names_w, r1["wing_aero_components"]):
        br[n] = float(np.asarray(comp.D).ravel()[0] / (q * S))
    for n, comp in zip(names_f, r1["fuselage_aero_components"]):
        br[n] = float(np.asarray(comp.D).ravel()[0] / (q * S))
    CDtot, CDp1, CDi1, extra = viscous_corrections(g, r1, V_cruise)
    br["excrescence(+%.0f%% of profile)" % ((g["parasite"]["excrescence_factor"] - 1) * 100)] = float(CDp1[0] * (g["parasite"]["excrescence_factor"] - 1))
    for k, v in extra_parasite_CDA(g).items():
        br[k] = v / S
    breakdown = dict(V=V_cruise, alpha_deg=a_cruise, CL=float(r1["CL"][0]), CD_total=float(CDtot[0]),
                     CD_induced=float(CDi1[0]), components_CD=br)
    # CD0 estimate: fit CD = CD0 + k (CL-CLmin)^2 over linear range
    m = (al > -4) & (al < 7)
    CDarr = np.array(T["CD"])
    pf = np.polyfit(CLa_arr[m], CDarr[m], 2)
    k_ = pf[0]; CLmind = -pf[1] / (2 * pf[0]); CD0 = pf[2] - pf[1] ** 2 / (4 * pf[0])
    e_eff = 1 / (math.pi * (b ** 2 / S) * k_)

    # ---- 4. AVL: stability + control derivatives -------------------------------------------------
    avl_file = os.path.join(outdir, "stork.avl")
    write_avl(g, avl_file, os.path.join(outdir, "avl_airfoils"))
    avl_alphas = [-4.0, 0.0, round(a_cruise, 2), 4.0, 8.0]
    avl_res, t_avl = run_avl(avl_exe, avl_file, avl_alphas, outdir)
    log["avl_s"] = t_avl
    ac = avl_res[avl_alphas.index(round(a_cruise, 2))]
    cmap = ac["_controls"]  # d01->name
    avl_der = {}
    for k_avl, k_std in [("CLa", "CLa"), ("Cma", "Cma"), ("CYb", "CYb"), ("Clb", "Clb"), ("Cnb", "Cnb"),
                         ("CLq", "CLq"), ("Cmq", "Cmq"), ("Clp", "Clp"), ("Cnp", "Cnp"), ("CYp", "CYp"),
                         ("Clr", "Clr"), ("Cnr", "Cnr"), ("CYr", "CYr"), ("Xnp", "x_np"), ("e", "e_inviscid")]:
        if k_avl in ac:
            avl_der[k_std] = ac[k_avl]
    avl_ctrl = {}
    for dnn, name in cmap.items():
        dd = {}
        for coef, key in (("CL", "CL"), ("CY", "CY"), ("Cl", "Cl"), ("Cm", "Cm"), ("Cn", "Cn"), ("CDff", "CDff")):
            kk = f"{coef}{dnn}"
            if kk in ac:
                dd[key] = ac[kk] / DEG  # AVL control derivatives are per degree -> per rad
        avl_ctrl[name] = dd
    avl_alpha_tab = dict(alpha_deg=avl_alphas, CL=[r.get("CLtot") for r in avl_res], Cm=[r.get("Cmtot") for r in avl_res],
                         CDind=[r.get("CDind") for r in avl_res])
    # ---- 4b. Cm correction: AeroBuildup has no wing->tail downwash and no fuselage/tail interference, so its
    # Cm_alpha is too stable (x_np too far aft). Replace the LINEAR part of each Cm table with AVL's (which
    # includes downwash and the fuselage BODY) and keep AeroBuildup's nonlinear (stall) shape:
    #   Cm(a) = Cm_AB(a) + [Cm0_AVL - Cm0_AB] + [Cma_AVL - Cma_AB] * clip(a, -10, 15 deg)
    aa = np.array(avl_alphas) * DEG
    pa = np.polyfit(aa, np.array(avl_alpha_tab["Cm"], float), 1)
    cm_corr = {}
    for V, tb in tables.items():
        al_ = np.array(tb["alpha_deg"]); cm = np.array(tb["Cm"])
        mm_ = (al_ >= -4) & (al_ <= 8)
        pb_ = np.polyfit(al_[mm_] * DEG, cm[mm_], 1)
        corr = (pa[1] - pb_[1]) + (pa[0] - pb_[0]) * np.clip(al_, -10, 15) * DEG
        tb["Cm_aerobuildup_raw"] = cm.tolist()
        tb["Cm"] = (cm + corr).tolist()
        cm_corr[V] = dict(Cm0_AB=float(pb_[1]), Cma_AB=float(pb_[0]), Cm0_AVL=float(pa[1]), Cma_AVL=float(pa[0]))
    log["cm_correction"] = cm_corr

    # ---- 5. ASB VLM cross-check -----------------------------------------------------------------
    t0 = time.time()
    vlm = asb.VortexLatticeMethod(ap, asb.OperatingPoint(velocity=V_cruise, alpha=a_cruise), xyz_ref=ap.xyz_ref,
                                  spanwise_resolution=8, chordwise_resolution=8)
    try:
        rv = vlm.run_with_stability_derivatives(alpha=True, beta=True, p=False, q=False, r=False)
        vlm_der = {k: float(np.asarray(rv[k]).ravel()[0]) for k in ("CL", "CLa", "Cma", "CYb", "Clb", "Cnb", "x_np") if k in rv}
    except Exception as e:  # noqa
        vlm_der = {"error": repr(e)}
    log["asb_vlm_s"] = time.time() - t0

    # ---- 6. Performance numbers -----------------------------------------------------------------
    def trimmed_CL_table(V):
        """Correct CL for tail trim load using AVL symmetric ruddervator derivatives."""
        TT = tables[V]
        Cm_de = avl_ctrl.get("rv_R", {}).get("Cm", 0) + avl_ctrl.get("rv_L", {}).get("Cm", 0)
        CL_de = avl_ctrl.get("rv_R", {}).get("CL", 0) + avl_ctrl.get("rv_L", {}).get("CL", 0)
        Cm = np.array(TT["Cm"]); CL = np.array(TT["CL"])
        de = -Cm / Cm_de if Cm_de else np.zeros_like(Cm)
        return CL + CL_de * de, de
    perf = {}
    for V in speeds:
        TT = tables[V]
        al_ = np.array(TT["alpha_deg"]); CL = np.array(TT["CL"]); CD_ = np.array(TT["CD"])
        rng = (al_ >= -5) & (al_ <= 20)
        i = int(np.argmax(np.where(rng, CL, -9)))
        CLt, de = trimmed_CL_table(V)
        it = int(np.argmax(np.where(rng, CLt, -9)))
        LD = CL / CD_
        j = int(np.argmax(np.where(rng & (CL > 0), LD, -9)))
        perf[V] = dict(CLmax_untrimmed=float(CL[i]), alpha_CLmax_deg=float(al_[i]),
                       CLmax_trimmed=float(CLt[it]), trim_def_at_CLmax_deg=float(de[it] / DEG),
                       LD_max=float(LD[j]), CL_at_LDmax=float(CL[j]), alpha_LDmax_deg=float(al_[j]),
                       Re_mac=rho * V * c / 1.789e-5)
    CLmax_lo = perf[min(speeds)]["CLmax_trimmed"]
    V_stall = math.sqrt(2 * W / (rho * S * CLmax_lo))
    # power required vs speed (aerodynamic power D*V) using the speed-closest table
    Vs = np.arange(8, 25.1, 1.0)
    preq = []
    for V in Vs:
        Vt = min(speeds, key=lambda x: abs(x - V))
        TT = tables[Vt]; al_ = np.array(TT["alpha_deg"]); CL = np.array(TT["CL"]); CD_ = np.array(TT["CD"])
        CLreq = W / (0.5 * rho * V ** 2 * S)
        rng = (al_ >= -6) & (al_ <= perf[Vt]["alpha_CLmax_deg"])
        if CLreq > CL[rng].max():
            preq.append(None); continue
        CDv = float(np.interp(CLreq, CL[rng], CD_[rng]))
        preq.append(dict(V=float(V), CL=CLreq, CD=CDv, LD=CLreq / CDv, P_aero_W=0.5 * rho * V ** 3 * S * CDv))
    preq = [x for x in preq if x]
    return dict(tables=tables, beta_table=beta_tab, a_cruise=a_cruise, CL_cruise=CL_cruise, ab_der=ab_der,
                avl_der=avl_der, avl_ctrl=avl_ctrl, avl_alpha_tab=avl_alpha_tab, vlm_der=vlm_der,
                breakdown=breakdown, polar_fit=dict(CD0=CD0, CL_minD=CLmind, k=k_, e_eff=e_eff), perf=perf,
                V_stall=V_stall, power=preq, log=log, avl_controls_map=cmap)


def make_dataset(g, mass, res, args):
    S, b, c = g["ref"]["S"], g["ref"]["b"], g["ref"]["c"]
    ab, av, ctrl = res["ab_der"], res["avl_der"], res["avl_ctrl"]
    T = res["tables"][args.v_cruise]
    # Recommended derivative set: AVL for lifting-surface & rate terms (better V-tail/wing interaction),
    # AeroBuildup for fuselage-sensitive static terms is blended: CLa (viscous) from buildup slope, Cma from AVL
    # (AVL includes fuselage BODY), lateral static from AVL.
    al = np.array(T["alpha_deg"]); CL = np.array(T["CL"]); Cm = np.array(T["Cm"])
    m = (al >= -2) & (al <= 6)
    CLa_tab = np.polyfit(al[m] * DEG, CL[m], 1)
    Cma_tab = np.polyfit(al[m] * DEG, Cm[m], 1)
    perf_lo = res["perf"][min(args.speeds)]
    deriv = dict(
        CL0=float(CLa_tab[1]), CLa=float(CLa_tab[0]), CLq=av.get("CLq", ab["CLq"]),
        CLmax=perf_lo["CLmax_untrimmed"], alpha_stall_deg=perf_lo["alpha_CLmax_deg"], CL_post_stall_drop=0.3,
        CD0=res["polar_fit"]["CD0"], oswald=res["polar_fit"]["e_eff"], CD_beta2=None,
        Cm0=float(Cma_tab[1]), Cma=float(Cma_tab[0]), Cmq=av.get("Cmq", ab["Cmq"]), Cmadot=None,
        CYb=av.get("CYb", ab["CYb"]), CYp=av.get("CYp", ab["CYp"]), CYr=av.get("CYr", ab["CYr"]),
        Clb=av.get("Clb", ab["Clb"]), Clp=av.get("Clp", ab["Clp"]), Clr=av.get("Clr", ab["Clr"]),
        Cnb=av.get("Cnb", ab["Cnb"]), Cnp=av.get("Cnp", ab["Cnp"]), Cnr=av.get("Cnr", ab["Cnr"]),
    )
    # Cmadot: tail downwash lag estimate  Cmadot ~ Cmq_tail * deps/dalpha, take deps/da = 0.4, tail share ~ 85 % of Cmq
    deriv["Cmadot"] = 0.4 * 0.85 * deriv["Cmq"]
    # CD_beta2 from beta sweep: CD(beta) = CD(0) + k*beta^2
    bt = res["beta_table"]; bb = np.array(bt["beta_deg"]) * DEG; cdb = np.array(bt["CD"])
    deriv["CD_beta2"] = float(np.polyfit(bb ** 2, cdb, 1)[0])
    # surfaces (per physical surface, + TE down)
    name_map = {"aileron_L": ("aileron_left", "aileron"), "aileron_R": ("aileron_right", "aileron"),
                "flap_L": ("flap_left", "flap"), "flap_R": ("flap_right", "flap"),
                "rv_L": ("ruddervator_left", "ruddervator"), "rv_R": ("ruddervator_right", "ruddervator")}
    surfaces = []
    for k, (nm, kind) in name_map.items():
        if kind not in g["controls"]:
            continue
        d = ctrl.get(k, {})
        # CD_abs: profile-drag increment of a plain flap ~ use AVL CDff slope magnitude + viscous estimate
        # viscous: dCD/d|delta| ~ 0.9 * (Sflap_wet/S) * ... use NeuralFoil-geomflap-like estimate 0.04*Sflapped/S per rad
        surfaces.append(dict(name=nm, kind=kind, max_deflection_deg=g["controls"][kind]["max_deg"],
                             derivatives_per_rad=dict(CL=d.get("CL", 0.0), CY=d.get("CY", 0.0), Cl=d.get("Cl", 0.0),
                                                      Cm=d.get("Cm", 0.0), Cn=d.get("Cn", 0.0),
                                                      CD_abs=abs(d.get("CDff", 0.0)) + 0.02)))
    ds = {
        "schema": "aero_dataset.schema.json", "schema_version": "1.0",
        "name": g["name"],
        "created": datetime.datetime.now().isoformat(timespec="seconds"),
        "provenance": {
            "geometry": g["source_yaml"],
            "assumptions": g["assumed"],
            "tools": {"AeroSandbox": asb.__version__, "NeuralFoil": __import__("neuralfoil").__version__,
                      "AVL": "3.40 (github.com/mayerrobert/avl, gfortran 13)",
                      "method": "Static/longitudinal tables: ASB AeroBuildup (NeuralFoil section polars, model_size='small') "
                                "+ excrescence factor + extra CDA items. Lateral/rate/control derivatives: AVL VLM (wing+V-tail+fuselage BODY). "
                                "Cmadot: estimate 0.34*Cmq."},
            "wall_time_s": res["log"],
        },
        "reference": {"S_m2": S, "b_m": b, "c_m": c, "moment_ref_m": [g["ref"]["x_cg"], 0.0, g.get("zref", 0.0)],
                      "frame": "structural: origin wing-root LE on centreline, +X aft, +Y right, +Z up; coefficients in stability axes, standard signs",
                      "mass_kg": mass},
        "conditions": {"V_cruise_mps": args.v_cruise, "alpha_cruise_deg": res["a_cruise"], "CL_cruise": res["CL_cruise"],
                       "Re_mac_cruise": 1.225 * args.v_cruise * c / 1.789e-5, "rho": 1.225},
        "alpha_tables": {str(V): {"V_mps": V, "Re_mac": 1.225 * V * c / 1.789e-5, **{k: v for k, v in tb.items() if k != "wall_s"}}
                         for V, tb in res["tables"].items()},
        "beta_table": res["beta_table"],
        "derivatives": deriv,
        "derivatives_by_source": {"aerobuildup": res["ab_der"], "avl": res["avl_der"], "asb_vlm": res["vlm_der"]},
        "surfaces": surfaces,
        "drag_breakdown_cruise": res["breakdown"],
        "polar_fit": res["polar_fit"],
        "performance": {"by_speed": {str(k): v for k, v in res["perf"].items()}, "V_stall_mps_trimmed_flaps0": res["V_stall"],
                        "power_required_aero": res["power"]},
        "avl_alpha_table": res["avl_alpha_tab"],
    }
    # sim export (sim/schema/aircraft_model.schema.json 'aero' block + surfaces derivatives)
    sim = {"aero": {"moment_ref_m": ds["reference"]["moment_ref_m"], "reynolds_ref": ds["conditions"]["Re_mac_cruise"],
                    "alpha_tables": {k: res["tables"][args.v_cruise][k] for k in ("alpha_deg", "CL", "CD", "Cm")},
                    "derivatives": deriv},
           "surfaces_derivatives_per_rad": {s["name"]: s["derivatives_per_rad"] for s in surfaces},
           "provenance": "aero stage: research/aero/stork_aero.py"}
    return ds, sim


def plots(ds, outdir):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 2, figsize=(12, 9))
    for k, t in ds["alpha_tables"].items():
        lab = f"V={t['V_mps']:.0f} m/s Re={t['Re_mac'] / 1e3:.0f}k"
        ax[0, 0].plot(t["alpha_deg"], t["CL"], label=lab)
        ax[0, 1].plot(t["CD"], t["CL"], label=lab)
        ax[1, 0].plot(t["alpha_deg"], t["Cm"], label=lab)
        ax[1, 1].plot(t["alpha_deg"], np.array(t["CL"]) / np.array(t["CD"]), label=lab)
    a = ds["avl_alpha_table"]
    ax[0, 0].plot(a["alpha_deg"], a["CL"], "ko", label="AVL (inviscid)")
    ax[1, 0].plot(a["alpha_deg"], a["Cm"], "ko", label="AVL")
    ax[0, 0].set(xlabel="alpha [deg]", ylabel="CL", xlim=(-20, 30)); ax[0, 1].set(xlabel="CD", ylabel="CL", xlim=(0, 0.2))
    ax[1, 0].set(xlabel="alpha [deg]", ylabel="Cm (about CG ref)", xlim=(-20, 30)); ax[1, 1].set(xlabel="alpha [deg]", ylabel="L/D", xlim=(-5, 20), ylim=(-5, 20))
    for x in ax.ravel(): x.grid(alpha=.3); x.legend(fontsize=7)
    fig.suptitle(f"{ds['name']}: AeroBuildup (+excrescence, extras) vs AVL")
    fig.tight_layout(); fig.savefig(os.path.join(outdir, "aero_tables.png"), dpi=100)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--yaml", default="/home/claude/research/geom/stork.yaml")
    ap.add_argument("--vtol", action="store_true")
    ap.add_argument("--mass", type=float, default=None, help="kg (default 1.775 fixed wing, 2.6 VTOL)")
    ap.add_argument("--avl", default=os.path.join(HERE, "tools", "bin", "avl"))
    ap.add_argument("--out", default=None)
    ap.add_argument("--v_cruise", type=float, default=14.0)
    ap.add_argument("--speeds", default="10,14,20")
    ap.add_argument("--prop_parking", default=None, choices=[None, "aligned", "crosswise_worst"])
    ap.add_argument("--zref", type=float, default=0.0)
    args = ap.parse_args()
    args.speeds = tuple(float(x) for x in args.speeds.split(","))
    if args.v_cruise not in args.speeds:
        args.speeds = tuple(sorted(args.speeds + (args.v_cruise,)))
    g = load_params(args.yaml, vtol=args.vtol)
    g["zref"] = args.zref
    if args.prop_parking:
        g["parasite"]["vtol_prop_parking"] = args.prop_parking
    mass = args.mass or (2.6 if args.vtol else 1.775)
    out = os.path.abspath(args.out or os.path.join(HERE, "out", g["name"]))
    os.makedirs(out, exist_ok=True)
    t0 = time.time()
    res = analyse(g, mass, args.avl, out, V_cruise=args.v_cruise, speeds=args.speeds)
    ds, sim = make_dataset(g, mass, res, args)
    ds["provenance"]["wall_time_s"]["total"] = time.time() - t0
    json.dump(ds, open(os.path.join(out, "aero_dataset.json"), "w"), indent=1)
    json.dump(sim, open(os.path.join(out, "sim_aero_block.json"), "w"), indent=1)
    try:
        import jsonschema
        jsonschema.validate(ds, json.load(open(os.path.join(HERE, "aero_dataset.schema.json"))))
        print("schema validation: OK")
    except FileNotFoundError:
        pass
    plots(ds, out)
    d = ds["derivatives"]
    print(f"== {g['name']}  mass {mass} kg  total {time.time() - t0:.1f} s  timings {res['log']}")
    print(f"cruise V={args.v_cruise} alpha={res['a_cruise']:.2f} CL={res['CL_cruise']:.3f}")
    print("polar fit:", {k: round(v, 4) for k, v in res["polar_fit"].items()})
    print("perf:", json.dumps({k: {kk: round(vv, 3) for kk, vv in v.items()} for k, v in res["perf"].items()}))
    print(f"V_stall (trimmed, flaps 0, lowest-speed table) = {res['V_stall']:.2f} m/s")
    print("drag breakdown:", json.dumps(res["breakdown"], indent=0))
    print("derivatives (recommended):", json.dumps({k: (round(v, 4) if isinstance(v, float) else v) for k, v in d.items()}))
    print("AeroBuildup:", {k: round(v, 4) for k, v in res["ab_der"].items()})
    print("AVL:", {k: round(v, 4) for k, v in res["avl_der"].items()})
    print("ASB VLM:", res["vlm_der"])
    print("controls (per rad):", json.dumps({k: {kk: round(vv, 4) for kk, vv in v.items()} for k, v in res["avl_ctrl"].items()}))
    for p in res["power"]:
        if p["V"] in (10, 12, 13, 14, 15, 16, 18, 20):
            print(f"  V={p['V']:4.1f} CL={p['CL']:.3f} CD={p['CD']:.4f} L/D={p['LD']:.1f} P_aero={p['P_aero_W']:.1f} W")


if __name__ == "__main__":
    main()
