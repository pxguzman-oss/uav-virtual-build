"""Stage 6a - low-order aerodynamics: AeroSandbox AeroBuildup (NeuralFoil) + AVL 3.40 + ASB VLM
-> aero_dataset.json (tables, derivatives, per-surface control derivatives) + sim_aero_block.json,
then stability/trim/authority gates and linear modes about the mass-stage CG."""
import json
import math
import types

import numpy as np

from .. import env, plan as P
from ..adapters import geom2aero
from ..figures import style, save, PALETTE, INK2
from ..flight_dyn import modes

DEG = math.pi / 180


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
    res = sa.analyse(g, mass["mass_kg"], avl, str(out), V_cruise=vc, speeds=speeds)
    args = types.SimpleNamespace(v_cruise=vc, speeds=speeds)
    ds, sim = sa.make_dataset(g, mass["mass_kg"], res, args)
    ds["provenance"]["geometry"] = f"build plan {ctx.plan_path.name} ({ctx.variant})"
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
    Vs = ds["performance"]["V_stall_mps_trimmed_flaps0"]

    # ---- gates -------------------------------------------------------------------------------
    launch = plan["airframe"].get("launch", {})
    if launch.get("method") == "hand" and launch.get("launch_speed"):
        ctx.gate("stall speed (trimmed) <= hand-launch speed / 1.2 [m/s]", Vs, launch["launch_speed"] / 1.2, "<=", "FAIL", "m/s",
                 note=f"launch {launch['launch_speed']} m/s", drivers=["/airframe/lifting_surfaces/wing/cl_max", "/components (mass)"])
    if env_.get("v_stall_target"):
        ctx.gate("stall speed vs plan target [m/s]", Vs, env_["v_stall_target"] * 1.05, "<=", "WARN", "m/s")
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
        ax[0].plot(al[sel], CL[sel], color=PALETTE[i % 8], label=f"{t['V_mps']:.0f} m/s")
        ax[1].plot(CD[sel], CL[sel], color=PALETTE[i % 8])
    a = ds["avl_alpha_table"]
    ax[0].plot(a["alpha_deg"], a["CL"], "o", color=INK2, ms=4, label="AVL (inviscid)")
    ax[0].set(xlabel="alpha [deg]", ylabel="CL", title="Lift curve (AeroBuildup)")
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
        ax.axvline(Vs, color=PALETTE[7], ls=":", lw=1); ax.text(Vs, ax.get_ylim()[1] * 0.92, " Vs", color=PALETTE[7])
        ax.axvline(vc, color=INK2, ls="--", lw=1); ax.text(vc, ax.get_ylim()[1] * 0.92, " cruise", color=INK2)
        ax.set(xlabel="airspeed [m/s]", ylabel="power [W]", title="Aerodynamic power required (level, ISA SL)")
        save(fig, out / "power_required.png")
        ctx.add_file(out / "power_required.png", "Aerodynamic power required vs airspeed", "figure")

    data = dict(dataset="aero_dataset.json", sim_block="sim_aero_block.json", derivatives=d, V_stall=Vs,
                CD0=ds["polar_fit"]["CD0"], e=ds["polar_fit"]["e_eff"], k=ds["polar_fit"]["k"], CL_minD=ds["polar_fit"]["CL_minD"],
                alpha_cruise_deg=res["a_cruise"], CL_cruise=CLc, LD_cruise=ds["drag_breakdown_cruise"]["CL"] / ds["drag_breakdown_cruise"]["CD_total"],
                CD_cruise=ds["drag_breakdown_cruise"]["CD_total"], static_margin_avl=sm_avl, static_margin_deriv=sm_deriv, x_np=xnp,
                Cm_de=Cm_de, Cn_dr=Cn_dr, Cl_da=Cl_da, pb2V=pb2v, Vh=Vh, Vv=Vv, trim_cruise_deg=de_c,
                modes={k: v for k, v in mo.items() if not k.startswith("A_")}, drag_breakdown=ds["drag_breakdown_cruise"],
                perf=ds["performance"]["by_speed"], power_required=pw, S=S, b=b, c=c, moment_ref=[g["ref"]["x_cg"], 0, g["zref"]],
                timings=res["log"])
    summ = dict(V_stall=round(Vs, 2), CD0=round(data["CD0"], 4), LD_cruise=round(data["LD_cruise"], 1),
                static_margin_pct=None if sm_avl is None else round(100 * sm_avl, 1), Cnb=round(d["Cnb"], 4),
                dutch_roll_zeta=round(dr.get("zeta", float("nan")), 3), spiral=sr)
    return dict(summary=summ, data=data)
