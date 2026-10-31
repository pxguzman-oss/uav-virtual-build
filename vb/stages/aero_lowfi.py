"""Stage 6a - low-order aerodynamics: AeroSandbox AeroBuildup (NeuralFoil) + AVL 3.40 + ASB VLM
-> aero_dataset.json (tables, derivatives, per-surface control derivatives) + sim_aero_block.json,
then stability/trim/authority gates and linear modes about the mass-stage CG."""
import json
import math
import types

import numpy as np

from .. import env, plan as P
from .. import liftmodel as LM
from ..adapters import geom2aero
from ..figures import style, save, PALETTE, INK2
from ..flight_dyn import modes

DEG = math.pi / 180


def _flap_k2(airfoil, hinge_xc, Re):
    """Section profile-drag increment per rad^2 of a plain flap: NeuralFoil on the DEFLECTED geometry,
    mean of +-4/+-8 deg at alpha 0 (symmetric part), fitted dCd = k2 * delta^2."""
    import aerosandbox as asb
    af = asb.Airfoil(airfoil)

    def cd(a):
        return float(np.ravel(a.get_aero_from_neuralfoil(alpha=0.0, Re=Re, model_size="large")["CD"])[0])
    c0 = cd(af)
    ks = []
    for d in (4.0, 8.0):
        dcd = 0.5 * (cd(af.add_control_surface(deflection=d, hinge_point_x=hinge_xc)) +
                     cd(af.add_control_surface(deflection=-d, hinge_point_x=hinge_xc))) - c0
        ks.append(dcd / (d * DEG) ** 2)
    return float(np.mean(ks))


def control_drag(plan, ds, res, g):
    """Control-surface drag (review M1): CD = CD_lin*delta (signed AVL CDff, induced) + CD_d2*delta^2 (profile,
    NeuralFoil deflected section x strip area / S_ref). Replaces the old CD_abs*|delta| (+0.02/rad) term that
    added ~0.0044 CD at the cruise ruddervator trim."""
    S = g["ref"]["S"]
    vc = float(plan["envelope"]["v_cruise"])
    w = P.main_wing(plan)
    vt = next(s for s in plan["airframe"]["lifting_surfaces"] if s.get("kind") in ("v_tail", "vtail"))
    nu = 1.789e-5 / 1.225
    out = {}
    for srf in ds["surfaces"]:
        kind = srf["kind"]
        side = "R" if "right" in srf["name"] else "L"
        avl_name = {"aileron": f"aileron_{side}", "flap": f"flap_{side}", "ruddervator": f"rv_{side}"}[kind]
        cdff = res["avl_ctrl"].get(avl_name, {}).get("CDff", 0.0)
        if kind in ("aileron", "flap"):
            cs = g["controls"][kind]
            y0, y1 = cs["y0"], cs["y1"]
            s2 = w["span"] / 2
            ch = lambda y: w["root_chord"] + (w["tip_chord"] - w["root_chord"]) * y / s2
            strip = (ch(y0) + ch(y1)) / 2 * (y1 - y0)
            k2 = _flap_k2(w["airfoil"].lower(), cs["hinge_xc"], vc * (ch(y0) + ch(y1)) / 2 / nu)
        else:
            cs = g["controls"]["ruddervator"]
            sp = vt["span"]
            ch = lambda f: vt["root_chord"] + (vt["tip_chord"] - vt["root_chord"]) * f
            strip = (ch(cs["s0"]) + ch(cs["s1"])) / 2 * (cs["s1"] - cs["s0"]) * sp
            k2 = _flap_k2(vt["airfoil"].lower(), cs["hinge_xc"], vc * (ch(cs["s0"]) + ch(cs["s1"])) / 2 / nu)
        d = srf["derivatives_per_rad"]
        d["CD_abs"] = 0.0
        # first-order (AVL CDff) term NOT applied: in trimmed flight the tail-lift change is carried by the CL/CD(alpha)
        # tables (alpha rises to compensate a download), and a signed linear term would make trim REDUCE drag
        d["CD_lin"] = 0.0
        d["CD_ff_avl_info"] = float(cdff)
        d["CD_d2"] = float(k2 * strip / S)
        out[srf["name"]] = dict(CD_d2=d["CD_d2"], k2_section=k2, strip_area_m2=strip, CD_ff_avl=float(cdff))
    ds["control_drag"] = dict(method="CD_d2 * delta^2 (NeuralFoil deflected section x strip area / S_ref); AVL CDff recorded, not applied",
                              surfaces=out)
    return out


def run(ctx):
    plan = ctx.plan
    mass = ctx.data("mass")
    g, sa = geom2aero.build(plan, mass["cg_m"], mass["mass_kg"])
    avl = env.avl()
    if not avl:
        raise RuntimeError("AVL not found (run setup/install.sh or set VB_AVL)")
    env_ = plan["envelope"]
    an = plan.get("analysis", {}).get("aero", {}) or {}
    vc = float(env_["v_cruise"])
    speeds = tuple(sorted(set(float(s) for s in an.get("speeds", [10, 14, 20])) | {vc}))
    out = ctx.dir
    w0 = P.main_wing(plan)
    clmax_node = w0.get("cl_max")
    clmax_plan = P.val(clmax_node)
    clmax_u = LM.uncertainty_of(clmax_node)
    res = sa.analyse(g, mass["mass_kg"], avl, str(out), V_cruise=vc, speeds=speeds, clmax_plan=clmax_plan, clmax_u=clmax_u)
    args = types.SimpleNamespace(v_cruise=vc, speeds=speeds)
    ds, sim = sa.make_dataset(g, mass["mass_kg"], res, args)
    ds["provenance"]["geometry"] = f"build plan {ctx.plan_path.name} ({ctx.variant})"
    control_drag(plan, ds, res, g)
    json.dump(ds, open(out / "aero_dataset.json", "w"), indent=1)
    json.dump(sim, open(out / "sim_aero_block.json", "w"), indent=1)
    try:
        sa.plots(ds, str(out))
        ctx.add_file(out / "aero_tables.png", "AeroBuildup tables at 3 speeds vs AVL", "figure")
    except Exception as e:
        ctx.note(f"aero plot failed: {e}")
    d = ds["derivatives"]
    w = P.main_wing(plan)
    S, b, c = g["ref"]["S"], g["ref"]["b"], g["ref"]["c"]
    m = mass["mass_kg"]
    lm = ds["lift_model"]
    Vs = lm["V_stall_nominal"]
    Vs_p = lm["V_stall_pessimistic"]
    cl_drv = ["/airframe/lifting_surfaces/wing/cl_max", "/components (mass)"]
    vs_note = (f"Vs nominal {Vs:.2f} / pessimistic {Vs_p:.2f} m/s (CLmax trimmed {lm['CLmax_trimmed_nominal']:.3f} / "
               f"{lm['CLmax_trimmed_pessimistic']:.3f}, -{100*lm['uncertainty_rel']:.0f} %)")

    # ---- gates -------------------------------------------------------------------------------
    # lift-model credibility: AeroBuildup vs the AVL anchor on the same geometry (info; CFD checks the model)
    xc = next((r for r in lm["crosscheck"] if r["alpha_deg"] == 4.0), lm["crosscheck"][0])
    ctx.gate("lift model: AeroBuildup raw CL / AVL CL at alpha 4 deg (same geometry) - raw buildup NOT used for the linear lift",
             xc["aerobuildup_raw"] / xc["avl"], None, "info", "INFO",
             note=f"AeroBuildup {xc['aerobuildup_raw']:.3f}, AVL {xc['avl']:.3f}, model {xc['model']:.3f}; CFD stage checks the model")
    ctx.gate("lift model: 3-D CLmax (untrimmed) used by every stage", lm["CLmax_untrimmed_nominal"], None, "info", "INFO",
             note=f"source: {lm['clmax_source']}; candidates " + ", ".join(f"{k.split(' (')[0]} {v:.3f}" for k, v in lm["clmax_sources"].items()),
             drivers=["/airframe/lifting_surfaces/wing/cl_max"])
    if clmax_plan:
        model_only = min(v for k, v in lm["clmax_sources"].items() if not k.startswith("plan"))
        ctx.gate("plan wing cl_max vs model CLmax: abs(ratio-1) within the stated uncertainty", abs(clmax_plan / model_only - 1),
                 lm["uncertainty_rel"], "<=", "WARN", note=f"plan {clmax_plan:.3f} vs model {model_only:.3f}",
                 drivers=["/airframe/lifting_surfaces/wing/cl_max"])
    launch = plan["airframe"].get("launch", {})
    if launch.get("method") == "hand" and launch.get("launch_speed"):
        ctx.gate("PESSIMISTIC stall speed (trimmed) <= hand-launch speed / 1.2 [m/s]", Vs_p, launch["launch_speed"] / 1.2, "<=", "FAIL", "m/s",
                 note=f"launch {launch['launch_speed']} m/s; {vs_note}; 1.2 = team criterion (launch at >= 1.2 Vs)",
                 drivers=cl_drv + ["/airframe/launch/launch_speed"])
    if env_.get("v_stall_target"):
        ctx.gate("nominal stall speed vs plan target x 1.05 [m/s]", Vs, env_["v_stall_target"] * 1.05, "<=", "WARN", "m/s",
                 note=vs_note, drivers=cl_drv)
    sm_deriv = -d["Cma"] / d["CLa"]
    xnp = res["avl_der"].get("x_np")
    sm_avl = (xnp - mass["cg_m"][0]) / c if xnp is not None else None
    lo, hi = plan.get("analysis", {}).get("static_margin_range", [0.05, 0.15])
    ctx.gate("Cm_alpha < 0 (static pitch stability)", d["Cma"], 0.0, "<=", "FAIL", "/rad")
    ctx.gate("static margin (AVL neutral point vs CG) [% MAC]", None if sm_avl is None else 100 * sm_avl, [100 * lo, 100 * hi], "range",
             "WARN", "%", note=f"from Cma/CLa: {100*sm_deriv:.1f} %", drivers=["/components (CG)", "/airframe/lifting_surfaces/vtail"])
    ctx.gate("Cn_beta > 0 (weathercock stability)", d["Cnb"], 0.0, ">=", "FAIL", "/rad", drivers=["/airframe/lifting_surfaces/vtail"])
    ctx.gate("Cl_beta < 0 (dihedral effect)", d["Clb"], 0.0, "<=", "FAIL", "/rad")
    # trim + authority
    surf = {s["name"]: s["derivatives_per_rad"] for s in ds["surfaces"]}
    rv = [s for s in ds["surfaces"] if s["kind"] == "ruddervator"]
    ail = [s for s in ds["surfaces"] if s["kind"] == "aileron"]
    Cm_de = sum(s["derivatives_per_rad"]["Cm"] for s in rv)
    Cn_dr = abs(sum(s["derivatives_per_rad"]["Cn"] * (1 if "left" in s["name"] else -1) for s in rv))
    Cl_da = abs(sum(s["derivatives_per_rad"]["Cl"] * (1 if "left" in s["name"] else -1) for s in ail))
    rv_max = rv[0]["max_deflection_deg"] if rv else 25
    a_max = ail[0]["max_deflection_deg"] if ail else 20
    perf_lo = ds["performance"]["by_speed"][str(min(speeds))]
    trim_stall = abs(perf_lo["trim_def_at_CLmax_deg"])
    ctx.gate("elevator (both ruddervators) trim at CLmax / max deflection", trim_stall / rv_max, 0.8, "<=", "WARN",
             note=f"{trim_stall:.1f} deg of {rv_max} deg (forward CG needs more)")
    CLc = m * 9.80665 / (0.5 * 1.225 * vc ** 2 * S)
    a_c = res["a_cruise"] * DEG
    Cm_c = d["Cm0"] + d["Cma"] * a_c
    de_c = -Cm_c / Cm_de / DEG if Cm_de else None
    ctx.gate("cruise trim deflection [deg]", None if de_c is None else abs(de_c), 0.4 * rv_max, "<=", "WARN", "deg",
             note="large cruise trim = drag + less authority (check CG / tail incidence)")
    pb2v = Cl_da * a_max * DEG / abs(d["Clp"]) if d["Clp"] else None
    ctx.gate("roll authority pb/2V at full aileron", pb2v, 0.07, ">=", "WARN", note="MIL-F-8785C class I level 1 >= 0.07")
    # V-tail volume coefficients (projected)
    vt = next(s for s in plan["airframe"]["lifting_surfaces"] if s.get("kind") in ("v_tail", "vtail"))
    Svt = 2 * vt["span"] * (vt["root_chord"] + vt["tip_chord"]) / 2
    gam = vt.get("dihedral_deg", 35) * DEG
    lt = vt.get("x_le_root", 0.44) + 0.25 * (vt["root_chord"] + vt["tip_chord"]) / 2 + 0.5 * vt["span"] * math.tan(
        vt.get("sweep_le_deg", 0) * DEG) - mass["cg_m"][0]
    Vh = Svt * math.cos(gam) ** 2 * lt / (S * c)
    Vv = Svt * math.sin(gam) ** 2 * lt / (S * b)
    ctx.gate("V-tail horizontal volume coefficient (projected)", Vh, 0.30, ">=", "WARN", drivers=["/airframe/lifting_surfaces/vtail"])
    ctx.gate("V-tail vertical volume coefficient (projected)", Vv, 0.020, ">=", "WARN", drivers=["/airframe/lifting_surfaces/vtail"])
    # linear modes at cruise
    I = mass["inertia_frd"]
    mo = modes(dict(d, CD0=d["CD0"], oswald=d["oswald"]), m, I["Ixx"], I["Iyy"], I["Izz"], I.get("Ixz", 0.0), S, b, c, vc)
    sp, ph, dr, sr, rl = (mo.get(k, {}) for k in ("short_period", "phugoid", "dutch_roll", "spiral", "roll"))
    ctx.gate("short-period damping ratio", sp.get("zeta"), 0.35, ">=", "WARN")
    ctx.gate("Dutch-roll damping ratio", dr.get("zeta"), 0.08, ">=", "WARN" if dr.get("zeta", 0) > 0 else "FAIL",
             note=f"wn {dr.get('wn', 0):.2f} rad/s", drivers=["/airframe/lifting_surfaces/vtail"])
    if sr:
        if sr["stable"]:
            ctx.gate("spiral mode", True, True, "bool", "WARN", note=f"stable, time to half {sr.get('time_to_half_s', 0):.0f} s")
        else:
            ctx.gate("spiral mode time to double [s] (autopilot copes if >= 12 s)", sr["time_to_double_s"], 12.0, ">=",
                     "WARN" if sr["time_to_double_s"] >= 4 else "FAIL", "s")
    if rl:
        ctx.gate("roll-mode time constant [s]", rl.get("tau_s"), 1.0, "<=", "WARN", "s")
    if ph:
        ctx.gate("phugoid damping ratio", ph.get("zeta"), 0.0, ">=", "WARN", note="TECS handles a lightly damped phugoid")

    # ---- figure: polars + drag breakdown ----------------------------------------------------------
    plt = style()
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.9))
    for i, (k, t) in enumerate(sorted(ds["alpha_tables"].items(), key=lambda kv: float(kv[0]))):
        al, CL, CD = np.array(t["alpha_deg"]), np.array(t["CL"]), np.array(t["CD"])
        sel = (al >= -8) & (al <= 20)
        ax[0].plot(al[sel], CL[sel], color=PALETTE[i % 8], label=f"model {t['V_mps']:.0f} m/s")
        if "CL_aerobuildup_raw" in t and abs(t["V_mps"] - vc) < 1e-6:
            ax[0].plot(al[sel], np.array(t["CL_aerobuildup_raw"])[sel], color=PALETTE[i % 8], lw=0.9, ls=":",
                       label=f"AeroBuildup raw {t['V_mps']:.0f} m/s")
        ax[1].plot(CD[sel], CL[sel], color=PALETTE[i % 8])
    a = ds["avl_alpha_table"]
    ax[0].plot(a["alpha_deg"], a["CL"], "o", color=INK2, ms=4, label="AVL (anchor)")
    ax[0].axhline(lm["CLmax_untrimmed_nominal"], color=PALETTE[7], ls="--", lw=1)
    ax[0].axhline(lm["CLmax_untrimmed_pessimistic"], color=PALETTE[7], ls=":", lw=1)
    ax[0].text(-7.5, lm["CLmax_untrimmed_nominal"] + 0.02, "CLmax nominal", color=PALETTE[7], fontsize=7.5)
    ax[0].text(-7.5, lm["CLmax_untrimmed_pessimistic"] - 0.09, "pessimistic", color=PALETTE[7], fontsize=7.5)
    ax[0].set(xlabel="alpha [deg]", ylabel="CL", title="Lift model (AVL-anchored, CLmax capped)")
    ax[1].set(xlabel="CD", ylabel="CL", title="Drag polar", xlim=(0, 0.15))
    ax[0].legend(loc="lower right")
    br = ds["drag_breakdown_cruise"]["components_CD"]
    br = dict(br, induced=ds["drag_breakdown_cruise"]["CD_induced"])
    keys = sorted(br, key=lambda k: br[k])
    ax[2].barh([k.replace("(+10% of profile)", "") for k in keys], [br[k] for k in keys], color=PALETTE[0], height=0.6)
    for i, k in enumerate(keys):
        ax[2].text(br[k] + 0.0003, i, f"{br[k]:.4f}", va="center", color=INK2, fontsize=8)
    ax[2].set(xlabel="CD contribution", title=f"Drag breakdown at {vc:g} m/s (CD {ds['drag_breakdown_cruise']['CD_total']:.4f})")
    ax[2].set_xlim(0, max(br.values()) * 1.35)
    ax[2].yaxis.grid(False)
    from matplotlib.ticker import MaxNLocator
    ax[2].xaxis.set_major_locator(MaxNLocator(4))
    save(fig, out / "aero_summary.png")
    ctx.add_file(out / "aero_summary.png", "Lift curve, drag polar and cruise drag breakdown", "figure")
    # power required
    pw = ds["performance"]["power_required_aero"]
    if pw:
        fig, ax = plt.subplots(figsize=(6, 3.4))
        ax.plot([p["V"] for p in pw], [p["P_aero_W"] for p in pw], "o-", color=PALETTE[0], label="aerodynamic power D*V")
        ax.axvspan(Vs, Vs_p, color=PALETTE[7], alpha=0.12, lw=0)
        ax.axvline(Vs, color=PALETTE[7], ls=":", lw=1); ax.text(Vs, ax.get_ylim()[1] * 0.92, " Vs nom.", color=PALETTE[7], ha="right")
        ax.text(Vs_p, ax.get_ylim()[1] * 0.80, " Vs pess.", color=PALETTE[7])
        ax.axvline(vc, color=INK2, ls="--", lw=1); ax.text(vc, ax.get_ylim()[1] * 0.92, " cruise", color=INK2)
        ax.set(xlabel="airspeed [m/s]", ylabel="power [W]", title="Aerodynamic power required (level, ISA SL)")
        save(fig, out / "power_required.png")
        ctx.add_file(out / "power_required.png", "Aerodynamic power required vs airspeed", "figure")

    data = dict(dataset="aero_dataset.json", sim_block="sim_aero_block.json", derivatives=d, V_stall=Vs, V_stall_pess=Vs_p,
                lift_model=lm, CLmax_untrimmed=lm["CLmax_untrimmed_nominal"], CLmax_untrimmed_pess=lm["CLmax_untrimmed_pessimistic"],
                CLmax_trimmed=lm["CLmax_trimmed_nominal"], CLmax_trimmed_pess=lm["CLmax_trimmed_pessimistic"], CLa=d["CLa"],
                CD0=ds["polar_fit"]["CD0"], e=ds["polar_fit"]["e_eff"], k=ds["polar_fit"]["k"], CL_minD=ds["polar_fit"]["CL_minD"],
                alpha_cruise_deg=res["a_cruise"], CL_cruise=CLc, LD_cruise=ds["drag_breakdown_cruise"]["CL"] / ds["drag_breakdown_cruise"]["CD_total"],
                CD_cruise=ds["drag_breakdown_cruise"]["CD_total"], static_margin_avl=sm_avl, static_margin_deriv=sm_deriv, x_np=xnp,
                Cm_de=Cm_de, Cn_dr=Cn_dr, Cl_da=Cl_da, pb2V=pb2v, Vh=Vh, Vv=Vv, trim_cruise_deg=de_c,
                modes={k: v for k, v in mo.items() if not k.startswith("A_")}, drag_breakdown=ds["drag_breakdown_cruise"],
                perf=ds["performance"]["by_speed"], power_required=pw, S=S, b=b, c=c, moment_ref=[g["ref"]["x_cg"], 0, g["zref"]],
                timings=res["log"])
    summ = dict(V_stall_nominal=round(Vs, 2), V_stall_pessimistic=round(Vs_p, 2),
                CLmax_trimmed_nom_pess=[round(lm["CLmax_trimmed_nominal"], 3), round(lm["CLmax_trimmed_pessimistic"], 3)],
                CLmax_source=lm["clmax_source"].split(" (")[0], CLa_per_rad=round(d["CLa"], 3), CD0=round(data["CD0"], 4), LD_cruise=round(data["LD_cruise"], 1),
                static_margin_pct=None if sm_avl is None else round(100 * sm_avl, 1), Cnb=round(d["Cnb"], 4),
                dutch_roll_zeta=round(dr.get("zeta", float("nan")), 3), spiral=sr)
    return dict(summary=summ, data=data)
