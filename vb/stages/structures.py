"""Stage 4 - structures & build risk (beam theory, see vb/lib/struct/structures.py): V-n envelope, spar
margins at ultimate, tip deflection, EI steps at tube ends, skin wrinkling, divergence / aileron reversal,
bending/torsion frequencies, servo torque, VTOL booms, Tg vs hottest skin temperature, firmware limits."""
import math
import sys
from pathlib import Path

import numpy as np

from ..figures import style, save, PALETTE, INK2

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lib" / "struct"))

DRIVERS = {"wing spar": ["/materials/cf_pultruded/knockdown", "/materials/cf_pultruded/E", "/airframe/lifting_surfaces/wing/spars"],
           "wing skin": ["/materials/lw_pla_230C/E", "/materials/lw_pla_230C/strength_tension"],
           "aeroelastic": ["/materials/lw_pla_230C/E (skin GJ)", "/airframe/lifting_surfaces/wing/skin"],
           "servos": ["/components/servo_*/torque_stall_Nm", "/airframe/lifting_surfaces/*/control_surfaces (horn/servo arms)"],
           "booms": ["/airframe/booms", "/components/lift_pod_*"], "thermal": ["/environment_cases", "/materials/*/tg_C"]}


def run(ctx):
    import structures as ST
    plan = ctx.plan
    mass = ctx.data("mass")
    prop = ctx.data("propulsion")
    r = ST.run(plan, ctx.variant, str(ctx.dir), prop=prop, M_auw=mass["mass_kg"])
    for g in r["gates"]:
        ctx.gates.append(dict(g, check=f"{g['stage']}: {g['check']}", stage="structures",
                              drivers=DRIVERS.get(g["stage"], [])))
    diag = r["diag"]
    env = plan["envelope"]
    plt = style()
    # V-n diagram
    vn = diag["vn"]
    fig, ax = plt.subplots(figsize=(7, 4))
    V = np.array(vn["V"])
    ax.plot(V, vn["n_stall"], color=PALETTE[0], label="stall boundary (CLmax)")
    ax.plot(V, -0.7 * np.array(vn["n_stall"]), color=PALETTE[0], lw=1, ls="--")
    ax.plot(V, vn["n_gust_c"], color=PALETTE[1], lw=1.2, label=f"gust {env.get('gust_u_de', {}).get('at_v_cruise', 7.5)} m/s")
    ax.plot(V, 2 - np.array(vn["n_gust_c"]), color=PALETTE[1], lw=1.2)
    ax.plot(V, vn["n_gust_ne"], color=PALETTE[3], lw=1.2, label=f"gust {env.get('gust_u_de', {}).get('at_v_ne', 5)} m/s")
    ax.axhline(env["n_limit_pos"], color=PALETTE[7], ls="--", lw=1.2, label=f"n limit +{env['n_limit_pos']}")
    ax.axhline(env.get("n_limit_neg", -1.5), color=PALETTE[7], ls="--", lw=1.2)
    for x, lab in ((vn["Vs"], "Vs"), (vn["VA"], "VA"), (env["v_cruise"], "Vc"), (env["v_ne"], "VNE")):
        ax.axvline(x, color=INK2, ls=":", lw=0.8)
        ax.text(x, ax.get_ylim()[1] if False else env["n_limit_pos"] + 0.9, lab, ha="center", color=INK2)
    ax.set(xlabel="equivalent airspeed [m/s]", ylabel="load factor n", title="V-n diagram (manoeuvre + Pratt gust)",
           ylim=(min(-2.5, env.get("n_limit_neg", -1.5) - 0.8), env["n_limit_pos"] + 1.5))
    ax.legend(loc="lower right", fontsize=7.5)
    save(fig, ctx.dir / "vn_diagram.png")
    ctx.add_file(ctx.dir / "vn_diagram.png", "V-n diagram", "figure")
    # spar moment and stress ratio
    sp = diag["spar"]
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.6))
    y = np.array(sp["y"]) * 1000
    ax[0].plot(y, sp["M_design"], color=PALETTE[0])
    ax[0].set(xlabel="spanwise station y [mm]", ylabel="bending moment [N m]",
              title=f"Half-wing bending moment at n = {sp['n_design']:.2f} (limit)")
    for i, (nm, rr) in enumerate(sp["stress_ratio_ult"].items()):
        rr = np.array(rr)
        m = rr > 0
        ax[1].plot(y[m], rr[m], color=PALETTE[i % 8], label=nm)
    ax[1].axhline(1.0, color=PALETTE[7], ls="--", lw=1.2)
    ax[1].text(y.max() * 0.98, 1.0, "allowable (knocked down)", ha="right", va="bottom", color=PALETTE[7], fontsize=8)
    ax[1].set(xlabel="spanwise station y [mm]", ylabel="stress / allowable at ultimate",
              title=f"Spar utilisation (n x FoS {env.get('fos', 1.5)})")
    ax[1].legend(loc="upper right")
    save(fig, ctx.dir / "spar_utilisation.png")
    ctx.add_file(ctx.dir / "spar_utilisation.png", "Wing bending moment and spar utilisation", "figure")
    fw = dict(PTCH2SRV_RMAX_UP=round(math.degrees(r["q_max_rad_s"])), PTCH2SRV_RMAX_DN=round(math.degrees(r["q_max_rad_s"])),
              AIRSPEED_MAX_cap=round(env["v_ne"] * 0.88, 1), VA=round(r["VA"], 1))
    data = dict(VA=r["VA"], Vs=r["Vs"], n_design=r["n_design"], n_land=r["n_land"], load_factors=r["load_factors"],
                f_bend_Hz=r["f_bend"], f_tors_Hz=r["f_tors"], V_div=r["V_div"], V_rev=r["V_rev"], firmware=fw,
                tip_deflection_mm=sp["deflection_design"][-1] * 1000)
    summ = dict(VA=round(r["VA"], 1), n_design=round(r["n_design"], 2), f_bend_Hz=round(r["f_bend"], 1),
                f_tors_Hz=round(r["f_tors"], 1), V_div=round(r["V_div"]), tip_defl_mm=round(data["tip_deflection_mm"]))
    return dict(summary=summ, data=data)
