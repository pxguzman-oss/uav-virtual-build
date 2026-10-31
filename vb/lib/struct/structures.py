"""Structures & build-risk stage (prototype, beam theory).

    python structures.py [buildplan.yaml] [--variant vtol] [--out out/]

Checks (each produces a margin and PASS/WARN/FAIL):
  1. Load envelope: V-n (manoeuvre, Pratt gust, stall ceiling), manoeuvring speed VA,
     belly-landing inertial load factor, VTOL max-thrust and hard-landing cases.
  2. Wing spar: Schrenk lift - inertia relief -> shear/moment; EI-proportional share
     between the carbon tubes; stress at ultimate (limit*FoS) vs knocked-down allowable;
     tip deflection; upper-skin wrinkling onset (serviceability).
  3. Torsion/aeroelastic: closed-skin GJ, divergence and aileron-reversal speed,
     bending/torsion frequencies (Rayleigh), separation from prop 1P/2P.
  4. Servo torque: hinge moment from Ch_delta, Ch_alpha (plain flap) at VNE vs stall torque.
  5. VTOL booms: clamp stress, thrust-axis tilt, cantilever + tip-mass frequency vs RPM band,
     wing-torsion frequency with lift pods attached.
  6. Thermal: printed-material Tg vs hottest environment case (+solar).
  7. Firmware limits implied by structure (pitch-rate limit, max airspeed).
Reads out/propulsion.json (rpm band, max lift thrust) when present.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from vb import plan as bp  # noqa: E402

G = 9.80665
RHO = 1.225
results = []


def gate(stage, check, value, limit, kind=">=", severity="FAIL", unit="", note=""):
    ok = value >= limit if kind == ">=" else value <= limit
    st = "PASS" if ok else severity
    results.append(dict(stage=stage, check=check, value=value, limit=limit, kind=kind,
                        status=st, unit=unit, note=note))
    print(f"   [{st:4s}] {check:58s} {value:9.3g} {kind} {limit:<8.3g}{unit} {note}")
    return ok


def tube_props(t, materials):
    od, idd = t["od"], t["id"]
    mat = materials[t["material"]]
    if t.get("shape", "round") == "square":
        I = (od ** 4 - idd ** 4) / 12.0
        A = od ** 2 - idd ** 2
        J = 2 * ((od + idd) / 2) ** 4 * ((od - idd) / 2) / ((od + idd) / 2)   # thin-wall square: t*a^3
    else:
        I = math.pi / 64 * (od ** 4 - idd ** 4)
        A = math.pi / 4 * (od ** 2 - idd ** 2)
        J = 2 * I
    E = bp.val(mat["E"]); Gm = bp.val(mat.get("G"), E / 2.6)
    allow = bp.val(mat.get("strength_flex", mat.get("strength_tension"))) * mat.get("knockdown", 1.0)
    return dict(I=I, A=A, J=J, E=E, G=Gm, c=od / 2, EI=E * I, GJ=Gm * J,
                m_per_m=A * mat["density"], allow=allow)


# ---------------------------------------------------------------------------
def run(plan, variant, out_dir, prop=None, M_auw=None, aero=None):
    """plan: variant-resolved plan dict; prop: propulsion-stage data (hover table, cruise rpm); M_auw [kg];
    aero: aero_lowfi data -> the ONE lift model (trimmed nominal CLmax and CLa) used by every stage (review C1/M2)."""
    results.clear()
    diag = {}
    mats = plan["materials"]
    env = plan["envelope"]
    mp = bp.mass_properties(plan)
    prop = prop or {}
    M_auw = M_auw or mp["mass"]
    W = M_auw * G
    wing = plan["airframe"]["lifting_surfaces"][0]
    b, S, cr, ct = wing["span"], wing["area"], wing["root_chord"], wing["tip_chord"]
    s = b / 2
    AR = b * b / S
    if aero and aero.get("CLmax_trimmed"):
        clmax = float(aero["CLmax_trimmed"])                 # nominal trimmed CLmax of the lift model (aero stage)
        clmax_src = "aero_lowfi lift model (nominal, trimmed)"
    else:
        clmax = bp.val(wing.get("cl_max"), 1.1)
        clmax_src = "plan wing cl_max (aero stage missing)"
    fos = env.get("fos", 1.5)
    print(f"=== STRUCTURES: {plan['meta']['name']} [{variant or 'base'}]  AUW {M_auw:.3f} kg, "
          f"W/S {W/S:.1f} N/m^2, AR {AR:.1f}, FoS {fos} ===")

    # ---------------- 1. load envelope ------------------------------------------
    if aero and aero.get("CLa"):
        a3d = float(aero["CLa"])                          # lift-curve slope /rad of the lift model (AVL-anchored)
    else:
        a3d = 2 * math.pi * AR / (AR + 2)               # Helmbold-type estimate when the aero stage is missing
    mac = (2 / 3) * cr * (1 + ct / cr + (ct / cr) ** 2) / (1 + ct / cr)
    mu = 2 * (W / S) / (RHO * mac * a3d * G)
    Kg = 0.88 * mu / (5.3 + mu)

    def n_stall(V):
        return 0.5 * RHO * V * V * S * clmax / W

    def n_gust(V, U):
        return 1 + RHO * V * a3d * U * Kg / (2 * W / S)

    Vs = math.sqrt(2 * W / (RHO * S * clmax))
    VA = Vs * math.sqrt(env["n_limit_pos"])
    gust = env.get("gust_u_de", {})
    cases = {
        "manoeuvre n_limit": env["n_limit_pos"],
        "gust @Vc (stall-capped)": min(n_gust(env["v_cruise"], gust.get("at_v_cruise", 7.5)), n_stall(env["v_cruise"])),
        "gust @VNE (stall-capped)": min(n_gust(env["v_ne"], gust.get("at_v_ne", 5.0)), n_stall(env["v_ne"])),
        "full pull-up @VNE (stall-capped)": n_stall(env["v_ne"]),
    }
    land = plan["airframe"].get("landing", {})
    n_land = 1 + land.get("sink_rate", 1.5) ** 2 / (2 * G * land.get("crush_stroke", 0.01))
    print(f" 1. envelope: Vs {Vs:.1f} m/s, VA(n={env['n_limit_pos']}) {VA:.1f} m/s, gust mass ratio mu {mu:.1f}, Kg {Kg:.2f}")
    for k, v in cases.items():
        print(f"      {k:36s} n = {v:5.2f}")
    print(f"      belly/hard landing (sink {land.get('sink_rate')} m/s, stroke {land.get('crush_stroke')*1000:.0f} mm): n = {n_land:.1f}")
    n_design = max(cases["manoeuvre n_limit"], cases["gust @VNE (stall-capped)"], cases["gust @Vc (stall-capped)"])
    diag["vn"] = dict(Vs=Vs, VA=VA, a3d=a3d, Kg=Kg, mu=mu, clmax=clmax, clmax_source=clmax_src, WS=W / S, n_land=n_land, cases=cases,
                      V=np.linspace(0, env["v_ne"], 60).tolist(), n_stall=[n_stall(v) for v in np.linspace(0, env["v_ne"], 60)],
                      n_gust_c=[n_gust(v, gust.get("at_v_cruise", 7.5)) for v in np.linspace(0, env["v_ne"], 60)],
                      n_gust_ne=[n_gust(v, gust.get("at_v_ne", 5.0)) for v in np.linspace(0, env["v_ne"], 60)])
    gate("envelope", "gust load factor @VNE <= n_limit (else lower VNE)", cases["gust @VNE (stall-capped)"],
         env["n_limit_pos"], "<=", "WARN")
    q_max = (env["n_limit_pos"] - 1) * G / env["v_ne"]
    print(f"      -> firmware: limit pitch rate to {math.degrees(q_max):.0f} deg/s at VNE "
          f"(n = 1 + V q / g); keep ARSPD_FBW_MAX <= {env['v_ne']*0.88:.0f} m/s; manual-mode full elevator above "
          f"VA={VA:.1f} m/s can exceed n_limit")

    # ---------------- 2. wing spar ---------------------------------------------------
    y = np.linspace(0, s, 401)
    dy = y[1] - y[0]
    c = cr + (ct - cr) * y / s
    planform = c / np.trapezoid(c, y)
    ellip = np.sqrt(np.clip(1 - (y / s) ** 2, 0, None)); ellip /= np.trapezoid(ellip, y)
    lift_shape = 0.5 * (planform + ellip)            # Schrenk, integrates to 1 over the half-span

    spars = wing.get("spars", [])
    sp_props = [dict(sp, **tube_props(sp["tube"], mats)) for sp in spars]
    EI_tubes = np.zeros_like(y)
    m_tubes = np.zeros_like(y)
    for sp in sp_props:
        on = (y >= sp["y_start"]) & (y <= sp["y_end"])
        EI_tubes += on * sp["EI"]
        m_tubes += on * sp["m_per_m"]
    # wing printed mass (distributed ~ chord) + point masses outboard of the fuselage
    comps = bp.expanded_components(plan)
    fus_half = plan["airframe"]["fuselage"].get("max_width", 0.1) / 2
    wing_print = sum(cc["mass"] for cc in comps if cc["name"].startswith("wing_print") and cc["position"][1] > 0)
    m_dist = m_tubes + wing_print * planform
    points = [(cc["position"][1], cc["mass"], cc["name"]) for cc in comps
              if cc["position"][1] > fus_half and not cc["name"].startswith(("wing_print", "spar_"))
              and cc.get("shape") != "distributed" and cc["position"][0] < 0.45]   # exclude tail items
    m_wing_half = np.trapezoid(m_dist, y) + sum(p[1] for p in points)
    print(f" 2. wing half: structural+mounted mass {m_wing_half*1000:.0f} g "
          f"(point masses: {', '.join(f'{p[2]} {p[1]*1000:.0f} g @y={p[0]:.2f}' for p in points)})")

    def moment(n_lift, n_inertia, extra_points=()):
        """Bending moment M(y) (positive = tip up) for the half wing.
        lift = n_lift * W/2 * lift_shape; inertia = -n_inertia * g * (m_dist + point masses)."""
        w = n_lift * W / 2 * lift_shape - n_inertia * G * m_dist
        Vsh = np.zeros_like(y); Mb = np.zeros_like(y)
        # integrate from tip inward
        Vsh[:-1] = np.cumsum((w[:-1] + w[1:])[::-1] / 2 * dy)[::-1]
        pl = [(py, -n_inertia * G * pm) for py, pm, _ in points] + list(extra_points)
        for py, F in pl:
            Vsh += F * (y < py)
        Mb[:-1] = np.cumsum((Vsh[:-1] + Vsh[1:])[::-1] / 2 * dy)[::-1]
        return Vsh, Mb

    load_cases = {f"{k}": (v, v, ()) for k, v in cases.items() if "pull-up" not in k}
    load_cases["belly landing (no lift, inertia)"] = (0.0, n_land, ())
    if prop.get("vtol", {}).get("hover_table") and plan["airframe"].get("booms"):
        rows = prop["vtol"]["hover_table"]
        T_max = max(r["wot_thrust_g"] for r in rows) / 1000 * G       # per motor
        y_boom = plan["airframe"]["booms"][0]["root"][1]
        n_q = 4 * T_max / W
        load_cases[f"VTOL all-motors WOT (T/W {n_q:.2f})"] = (0.0, n_q, ((y_boom, 2 * T_max),))
    worst = {}
    print("    load case                               n     M_root[Nm]  sigma_max/allow (ultimate, per tube)")
    for name, (nl, ni, ext) in load_cases.items():
        Vsh, Mb = moment(nl, ni, ext)
        line = []
        for sp in sp_props:
            on = (y >= sp["y_start"]) & (y <= sp["y_end"])
            share = np.where(on, sp["EI"] / np.maximum(EI_tubes, 1e-12), 0)
            sig = np.abs(Mb) * share * sp["c"] / sp["I"] * fos
            k = int(np.argmax(sig))
            ratio = sig[k] / sp["allow"]
            line.append(f"{sp['name']} {ratio:.2f}@y={y[k]:.2f}")
            if ratio > worst.get(sp["name"], (0,))[0]:
                worst[sp["name"]] = (ratio, name, y[k], sig[k] / fos / 1e6)
        print(f"    {name:38s} {max(nl, ni):5.2f}  {Mb[0]:8.2f}    " + "; ".join(line))
    _, Mb_d = moment(n_design, n_design)
    ratios = {}
    for sp in sp_props:
        on = (y >= sp["y_start"]) & (y <= sp["y_end"])
        share = np.where(on, sp["EI"] / np.maximum(EI_tubes, 1e-12), 0)
        ratios[sp["name"]] = (np.abs(Mb_d) * share * sp["c"] / sp["I"] * fos / sp["allow"]).tolist()
    diag["spar"] = dict(y=y.tolist(), M_design=Mb_d.tolist(), n_design=n_design, stress_ratio_ult=ratios,
                        EI_tubes=EI_tubes.tolist())
    for nm, (ratio, case, yy, sig_lim) in worst.items():
        gate("wing spar", f"spar {nm}: margin of safety (ultimate) [{case}]", 1 / ratio - 1, 0.0,
             note=f"(limit stress {sig_lim:.0f} MPa @y={yy:.2f} m)")

    # deflection (tubes only = lower-bound stiffness) under n_design
    _, Mb = moment(n_design, n_design)
    kappa = Mb / EI_tubes
    slope = np.concatenate([[0], np.cumsum((kappa[:-1] + kappa[1:]) / 2 * dy)])
    defl = np.concatenate([[0], np.cumsum((slope[:-1] + slope[1:]) / 2 * dy)])
    _, M1 = moment(1, 1)
    k1 = M1 / EI_tubes
    d1 = np.concatenate([[0], np.cumsum((np.concatenate([[0], np.cumsum((k1[:-1] + k1[1:]) / 2 * dy)])[:-1]
                                          + np.concatenate([[0], np.cumsum((k1[:-1] + k1[1:]) / 2 * dy)])[1:]) / 2 * dy)])
    diag["spar"]["deflection_design"] = defl.tolist()
    print(f"    tip deflection (tubes only): 1 g {d1[-1]*1000:.0f} mm, n={n_design:.1f} {defl[-1]*1000:.0f} mm "
          f"({defl[-1]/s*100:.0f} % semi-span)")
    gate("wing spar", "tip deflection at n_design / semi-span", defl[-1] / s, 0.15, "<=", "WARN",
         note="(>15 % -> aileron binding, skin cracking at rib/tube joints)")

    # skin contribution & wrinkling (serviceability)
    skin = wing.get("skin", {})
    smat = mats.get(skin.get("material", ""), {})
    Es = bp.val(smat.get("E"), 1.4e9); ts = skin.get("thickness", 0.45e-3)
    h = wing.get("thickness_ratio", 0.1) * c
    EI_skin = Es * 2 * (0.6 * c) * ts * (h / 2) ** 2                 # two caps, 60 % chord effective
    _, Mb1 = moment(1, 1)
    eps_skin = Mb1 / (EI_tubes + EI_skin) * h / 2
    b_panel = 0.020                                                      # unsupported skin between infill walls (est.)
    sig_cr = 3.6 * Es * (ts / b_panel) ** 2
    n_wrinkle = sig_cr / np.max(Es * np.abs(eps_skin))
    print(f"    skin EI at root {EI_skin[0]:.1f} N m^2 vs tubes {EI_tubes[0]:.1f} N m^2 -> skin is a real load path; "
          f"panel buckling {sig_cr/1e6:.2f} MPa")
    gate("wing skin", "upper-skin wrinkle onset load factor (serviceability)", n_wrinkle, env["n_limit_pos"], ">=", "WARN",
         note="(b_panel=20 mm est.; wrinkles at lower n are elastic but crack paint/hinges)")

    # stiffness steps at internal tube ends (printed skin/ribs see a strain concentration there)
    for sp in sp_props:
        ye = sp["y_end"]
        if ye < s - 0.02:
            i0 = np.searchsorted(y, ye) - 1
            before, after = (EI_tubes + EI_skin)[i0], (EI_tubes + EI_skin)[i0 + 2]
            _, Mn = moment(n_design, n_design)
            eps_after = abs(Mn[i0 + 2]) / after * h[i0 + 2] / 2 * fos
            gate("wing spar", f"EI step at end of {sp['name']} (y={ye:.2f}) before/after", before / after, 3.0, "<=", "WARN",
                 note=f"(skin strain just outboard {eps_after*100:.2f} % ult. vs LW-PLA ~1.4 % -> stagger ends / printed doubler)")
            smat_allow = bp.val(smat.get("strength_tension"), 2e7) * smat.get("knockdown", 0.5)
            gate("wing skin", f"skin stress just outboard of {sp['name']} end: MS (ultimate)",
                 smat_allow / (Es * eps_after) - 1, 0.0, ">=", "FAIL")

    # ---------------- 3. torsion / aeroelastic --------------------------------------------
    Gs = Es / (2 * 1.35)
    A_enc = 0.68 * wing.get("thickness_ratio", 0.1) * c * c
    perim = 2.04 * c
    GJ = Gs * 4 * A_enc ** 2 * ts / perim
    GJ_mean = float(np.mean(GJ))
    e_ac = 0.10 * c.mean()                       # AC (0.25c) to elastic axis (~0.35c)
    a2d = 2 * math.pi * 0.9
    qD = (math.pi / (2 * s)) ** 2 * GJ_mean / (e_ac * c.mean() * a2d)
    VD = math.sqrt(2 * qD / RHO)
    cs = [cs_ for cs_ in wing.get("control_surfaces", []) if cs_["function"] in ("aileron", "elevon", "flaperon")]
    if cs:
        E_f = cs[0]["chord_fraction"]
        th = math.acos(2 * E_f - 1)
        cl_d = 2 * (math.pi - th + math.sin(th))
        cm_d = -0.5 * math.sin(th) * (1 - math.cos(th))
        qR = (math.pi / (2 * s)) ** 2 * GJ_mean * cl_d / (c.mean() ** 2 * a2d * -cm_d)
        VR = math.sqrt(2 * qR / RHO)
    else:
        VR = float("inf")
    print(f" 3. torsion: skin GJ {GJ[0]:.0f} (root) .. {GJ[-1]:.0f} (tip) N m^2; divergence {VD:.0f} m/s, "
          f"aileron reversal {VR:.0f} m/s (strip theory, skin only)")
    gate("aeroelastic", "divergence speed / VNE", VD / env["v_ne"], 1.5, ">=", "FAIL")
    gate("aeroelastic", "aileron reversal speed / VNE", VR / env["v_ne"], 1.2, ">=", "FAIL")
    # frequencies (Rayleigh, static deflection shape, tubes + skin)
    EI_tot = EI_tubes + EI_skin
    k1t = M1 / EI_tot
    sl = np.concatenate([[0], np.cumsum((k1t[:-1] + k1t[1:]) / 2 * dy)])
    wdef = np.concatenate([[0], np.cumsum((sl[:-1] + sl[1:]) / 2 * dy)])
    m_all = m_dist.copy()
    pt_terms = sum(pm * np.interp(py, y, wdef) ** 2 for py, pm, _ in points)
    pt_work = sum(pm * np.interp(py, y, wdef) for py, pm, _ in points)
    # Rayleigh with gravity-load shape: omega^2 = g * sum(m w) / sum(m w^2)
    om_b = math.sqrt(G * (np.trapezoid(m_all * wdef, y) + pt_work) / (np.trapezoid(m_all * wdef ** 2, y) + pt_terms))
    Ia = m_dist * (0.25 * c) ** 2                      # torsional inertia per span (r_g = 0.25 c)
    I_pts = 0.0
    if plan["airframe"].get("booms"):
        x_ea = 0.35 * cr
        for cc in comps:
            if cc["name"].startswith("lift_pod") and cc["position"][1] > 0:
                I_pts += cc["mass"] * (cc["position"][0] - x_ea) ** 2
    om_t = (math.pi / (2 * s)) * math.sqrt(GJ_mean / np.mean(Ia))
    fb, ft = om_b / 2 / math.pi, om_t / 2 / math.pi
    print(f"    1st bending {fb:.1f} Hz, 1st torsion (clean) {ft:.1f} Hz, ratio {ft/fb:.1f}")
    gate("aeroelastic", "torsion/bending frequency ratio (flutter screening)", ft / fb, 2.0, ">=", "WARN")
    if I_pts > 0:
        y_b = plan["airframe"]["booms"][0]["root"][1]
        k_th = GJ_mean / y_b
        f_tp = math.sqrt(k_th / (I_pts + np.trapezoid(Ia[y <= y_b], y[y <= y_b]))) / 2 / math.pi
        print(f"    wing torsion WITH lift pods (I_pods {I_pts*1e4:.1f} kg cm^2/side, k {k_th:.0f} Nm/rad): {f_tp:.1f} Hz")
        gate("aeroelastic", "wing torsion freq with lift pods [Hz] (>= 3x Q-mode attitude BW ~5 Hz)", f_tp, 15.0, ">=", "WARN",
             note="(also: pods ahead of EA lower flutter speed)")
    cru = prop.get("cruise_unit", {}).get("cruise", {})
    if cru:
        f1p = cru["rpm"] / 60
        sep = min(abs(f1p - fb) / fb, abs(f1p - ft) / ft)
        gate("aeroelastic", "cruise prop 1P separation from wing modes", sep, 0.3, ">=", "WARN",
             note=f"(1P {f1p:.0f} Hz)")

    # ---------------- 4. servo torque ---------------------------------------------------------
    print(" 4. servos (hinge moment at VNE, max deflection; Ch_delta plain flap, Ch_alpha with alpha=4 deg):")
    servos = {cc["name"]: cc for cc in comps}
    for surf in plan["airframe"]["lifting_surfaces"]:
        cr_s, ct_s, span_s = surf["root_chord"], surf["tip_chord"], (surf["span"] / 2 if surf.get("kind") == "wing" else surf["span"])
        for cs_ in surf.get("control_surfaces", []):
            ym = 0.5 * (cs_["y_start"] + cs_["y_end"])
            cl_ = cr_s + (ct_s - cr_s) * ym / span_s
            cf = cs_["chord_fraction"] * cl_
            L = cs_["y_end"] - cs_["y_start"]
            d = math.radians(cs_["max_defl_deg"])
            Ch_d = -0.55 - 0.6 * cs_["chord_fraction"]        # approx. 2D plain flap, unsealed gap (/rad)
            Ch_a = -0.35
            q = 0.5 * RHO * env["v_ne"] ** 2
            H = q * (cf * L) * cf * abs(Ch_d * d + Ch_a * math.radians(4) * np.sign(Ch_d * d))
            ratio = cs_.get("servo_arm", 0.01) / cs_.get("horn_arm", 0.01)
            T_req = H * ratio * 1.25                           # 25 % linkage friction / binding
            # empirical cross-check (modelairplanenews-style): oz-in = L*C^2*V^2*defl_deg/430000 (in, mph)
            H_emp = (L / 0.0254) * (cf / 0.0254) ** 2 * (env["v_ne"] * 2.23694) ** 2 * cs_["max_defl_deg"] / 430000 * 0.0070616
            sv = servos.get(cs_["servo"]) or servos.get(cs_["servo"] + "_R") or next(
                (v for k, v in servos.items() if k.startswith(cs_["servo"])), None)
            stall = sv.get("torque_stall_Nm", 0.15) if sv else 0.15
            print(f"    {cs_['name']:12s} cf {cf*1000:.0f} mm x {L*1000:.0f} mm, H {H/G*100:.2f} kg.cm "
                  f"(empirical {H_emp/G*100:.2f}), servo req {T_req/G*100:.2f} kg.cm vs stall {stall/G*100:.2f} kg.cm")
            gate("servos", f"{cs_['name']} servo stall torque / required (VNE, full defl)", stall / T_req, 2.0, ">=", "FAIL",
                 note="(>=2: servo holds at <=50 % stall, no blow-back)")

    # ---------------- 5. VTOL booms ------------------------------------------------------------
    for boom in plan["airframe"].get("booms", []):
        tp = tube_props(boom["tube"], mats)
        sup = np.array(boom.get("supports", [boom["root"]]))
        ends = [np.array(boom["root"]), np.array(boom["tip"])]
        pods = [cc for cc in comps if cc["name"] in boom.get("tip_mass_components", [])]
        rows = prop.get("vtol", {}).get("hover_table", [])
        T_max = max((r["wot_thrust_g"] for r in rows), default=1200) / 1000 * G
        band = prop.get("vtol", {}).get("rpm_band", [9000, 16000])
        hover_rpms = [r["rpm"] for r in rows] or [11000]
        print(f" 5. boom {boom['name']}: {boom['tube'].get('shape','round')} {boom['tube']['od']*1000:.0f}x"
              f"{boom['tube']['id']*1000:.0f} mm, EI {tp['EI']:.0f} N m^2, T_max/motor {T_max:.1f} N, "
              f"lift RPM band {band[0]:.0f}-{band[1]:.0f}")
        for end in ends:
            L = float(np.min(np.linalg.norm(sup[:, :2] - end[:2], axis=1)))
            pod = min(pods, key=lambda p_: np.linalg.norm(np.array(p_["position"][:2]) - end[:2])) if pods else None
            m_tip = pod["mass"] if pod else 0.07
            M_thr = T_max * L
            M_land = n_land * m_tip * G * L
            sig = max(M_thr, M_land) * fos * tp["c"] / tp["I"]
            tilt = math.degrees(T_max / 2 * L ** 2 / (2 * tp["EI"]))   # at hover-ish thrust (T_max/2)
            k = 3 * tp["EI"] / L ** 3
            f1 = math.sqrt(k / (m_tip + 0.24 * tp["m_per_m"] * L)) / 2 / math.pi
            f1p_lo, f1p_hi = min(hover_rpms) / 60, band[1] / 60
            tag = f"{'front' if end[0] < 0.1 else 'rear'} L={L*1000:.0f} mm"
            print(f"    {tag}: M_thrust {M_thr:.2f} Nm, M_land {M_land:.2f} Nm, sigma_ult {sig/1e6:.0f} MPa, "
                  f"tilt @T_max/2 {tilt:.2f} deg, f1 {f1:.0f} Hz vs hover 1P {f1p_lo:.0f}-{f1p_hi:.0f} Hz, 2P {2*f1p_lo:.0f}+ Hz")
            gate("booms", f"boom {tag} margin of safety (ultimate)", tp["allow"] / sig - 1, 0.0)
            gate("booms", f"boom {tag} thrust-axis tilt at hover [deg]", tilt, 0.5, "<=", "WARN")
            gate("booms", f"boom {tag} f1 below hover 1P (f1/1P_min)", f1 / f1p_lo, 0.7, "<=", "WARN",
                 note="(if f1 sits inside the 1P band -> resonance; add notch / stiffen)")

    # ---------------- 6. thermal --------------------------------------------------------------
    hot = max(plan.get("environment_cases", [{}]), key=lambda e: e.get("temperature_C", 15) + e.get("solar_heating_dT_C", 0))
    T_skin = hot.get("temperature_C", 15) + hot.get("solar_heating_dT_C", 0)
    print(f" 6. thermal: hottest case '{hot.get('name')}' skin ~{T_skin:.0f} C")
    for k, m in mats.items():
        if m["kind"] == "printed_polymer" and "tg_C" in m:
            gate("thermal", f"{k}: Tg - 10 C margin vs skin temp [C]", m["tg_C"] - 10, T_skin, ">=", "FAIL",
                 note="(creep/warp of skins & spar sockets; use LW-ASA/ASA/PETG or white paint + shade)")

    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
    fn = out / f"structures_{variant or 'base'}.json"
    out_d = dict(plan=plan["meta"]["name"], variant=variant, auw=M_auw, VA=VA, Vs=Vs, load_factors=cases, n_land=n_land,
                 n_design=n_design, f_bend=fb, f_tors=ft, V_div=VD, V_rev=VR, q_max_rad_s=q_max, gates=list(results), diag=diag)
    fn.write_text(json.dumps(out_d, indent=2, default=float))
    n_fail = sum(r["status"] == "FAIL" for r in results); n_warn = sum(r["status"] == "WARN" for r in results)
    print(f"\n   summary: {len(results)} checks, {n_fail} FAIL, {n_warn} WARN -> {fn}")
    return out_d


if __name__ == "__main__":
    argv = sys.argv[1:]
    args = [a for i, a in enumerate(argv) if not a.startswith("--") and (i == 0 or not argv[i - 1].startswith("--"))]
    plan = args[0] if args else str(Path(__file__).resolve().parent / "stork_phase0.buildplan.yaml")
    var = sys.argv[sys.argv.index("--variant") + 1] if "--variant" in sys.argv else None
    out = sys.argv[sys.argv.index("--out") + 1] if "--out" in sys.argv else str(Path(__file__).resolve().parent / "out")
    run(plan, var, out)
