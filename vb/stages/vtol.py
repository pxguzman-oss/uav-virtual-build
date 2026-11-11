"""Stage 5b - quadplane (4 lift + 1 pusher) hover, transition and failure checks (analytic, seconds).

What the propulsion stage does NOT look at (it treats the four lift motors as identical and the
aircraft as a point mass):
  * lift-thrust centre vs CG: fore/aft thrust split, the most-loaded motor's hover duty
  * hover thrust margin in the WORST air of the plan (hottest/highest case, low battery)
  * roll / pitch / yaw control authority at hover from the remaining thrust headroom
  * crosswind hover: tilt needed to hold position, weathervane moment vs yaw authority
  * forward transition: time, distance, energy and battery current with lift + pusher together
  * back-transition distance (for the landing approach)
  * single lift-motor failure in hover and at which speed fixed-wing flight takes over
Skipped (status 'skipped') when the plan has no lift unit. Thresholds marked TEAM CRITERION are
defaults under gates.vtol in the plan; they are engineering judgement, not a standard."""
import math
import sys
from pathlib import Path

import numpy as np

from .. import plan as P
from ..figures import style, save, PALETTE, INK2

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lib" / "struct"))
G = 9.80665


def rho_at(T_C=15.0, alt=0.0):
    p = 101325 * (1 - 2.25577e-5 * alt) ** 5.25588
    return p / (287.05 * (T_C + 273.15))


def side_area_moment(plan, x_cg):
    """Side-projected drag area [m2] and its moment arm integral about the CG [m3] (x aft positive)
    for a pure crosswind in hover: fuselage (Cd 1.0 on height x length) + V-tail / fin panels."""
    A, Ax = 0.0, 0.0
    st = np.array(plan["airframe"]["fuselage"].get("stations") or [], float)
    if len(st) >= 2:
        for a, b in zip(st[:-1], st[1:]):
            dA = 0.5 * (a[2] + b[2]) * (b[0] - a[0]) * 1.0
            A += dA
            Ax += dA * (0.5 * (a[0] + b[0]) - x_cg)
    for s in plan["airframe"]["lifting_surfaces"]:
        kind = s.get("kind", "wing")
        if kind == "wing":
            continue
        span, cr, ct = P.val(s["span"]), P.val(s["root_chord"]), P.val(s["tip_chord"])
        dih = math.radians(P.val(s.get("dihedral_deg"), 90.0 if kind in ("fin", "vtail") else 0.0))
        # span of a v_tail in this schema = per panel along the panel; other surfaces = full span
        n_pan = 2 if kind == "v_tail" else 1
        area = n_pan * 0.5 * (cr + ct) * (span if kind == "v_tail" else span)
        dA = 1.2 * area * abs(math.sin(dih)) ** 3          # flat plate, normal-velocity component only
        x = P.val(s.get("x_le_root"), 0.0) + 0.4 * cr
        A += dA
        Ax += dA * (x - x_cg)
    return A, Ax


def run(ctx):
    import propulsion as PR
    plan = ctx.plan
    lifts = P.units(plan, "lift")
    if not lifts:
        return dict(skipped=True, summary=dict(not_applicable="no lift unit in this variant (fixed wing)"))
    cfg = (plan.get("gates") or {}).get("vtol", {})
    mass = ctx.data("mass")
    prop = ctx.data("propulsion")
    aero = ctx.data("aero_lowfi", required=False) or {}
    M, cg = mass["mass_kg"], np.array(mass["cg_m"], float)
    W = M * G
    I = mass["inertia_frd"]
    lift = lifts[0]
    n = lift["count"]
    pos = np.array(lift.get("positions") or [], float)
    if len(pos) != n:
        raise ValueError("lift unit needs one position per motor")
    pack = PR.Pack(plan["energy"])
    lm, lp = PR.Motor(lift["motor"]), PR.Prop(lift["prop"], ctx.plan_path.parent)
    envs = plan.get("environment_cases") or [dict(name="ISA", temperature_C=15, altitude_msl=0)]
    rho0 = rho_at(15, 0)
    thin = min(envs, key=lambda e: rho_at(e.get("temperature_C", 15), e.get("altitude_msl", 0)))
    rho_thin = rho_at(thin.get("temperature_C", 15), thin.get("altitude_msl", 0))
    ldrv = ["/propulsion/units/lift", "/components (positions)", "/analysis/cg_target_x"]

    # ---- 1. thrust centre vs CG -------------------------------------------------------------------
    xc, yc = pos[:, 0].mean(), pos[:, 1].mean()
    front = pos[:, 0] < xc
    xf, xr = pos[front, 0].mean(), pos[~front, 0].mean()
    f_front = (xr - cg[0]) / (xr - xf)                      # share of the weight carried by the front motors
    T_i = np.where(front, W * f_front / front.sum(), W * (1 - f_front) / (~front).sum())
    right = pos[:, 1] > yc
    if abs(cg[1] - yc) > 1e-6 and right.any() and (~right).any():      # lateral offset shifts load between sides too
        yl, yr = pos[~right, 1].mean(), pos[right, 1].mean()
        f_right = (cg[1] - yl) / (yr - yl)
        T_i = T_i * np.where(right, 2 * f_right, 2 * (1 - f_right))
    T_mean = W / n
    off_mm = (cg[0] - xc) * 1000
    ctx.gate("CG ahead(-)/behind(+) of the lift-thrust centre [mm]", round(off_mm, 1),
             [round(-cfg.get("centroid_offset_frac", 0.05) * float(xr - xf) * 1000, 1), round(cfg.get("centroid_offset_frac", 0.05) * float(xr - xf) * 1000, 1)],
             "range", "WARN", "mm",
             note=f"thrust centre x {xc*1000:.0f} mm (motors at {xf*1000:.0f} / {xr*1000:.0f} mm), CG {cg[0]*1000:.1f} mm; "
                  f"front motors carry {100*f_front:.1f} % of the weight. TEAM CRITERION: within 5 % of the fore-aft motor spacing",
             drivers=ldrv)
    ctx.gate("most-loaded lift motor: hover thrust / average", float(T_i.max() / T_mean), cfg.get("max_motor_load_ratio", 1.15), "<=",
             "FAIL", note="an off-centre CG makes one pair work harder: less control headroom, hotter motor/ESC, earlier saturation",
             drivers=ldrv)

    def op_T(T, rho, soc):
        return PR.duty_for_thrust(lift, lm, lp, pack, T, 0.0, rho, soc=soc)

    def wot(rho, soc):
        return PR.operating_point(lift, lm, lp, pack, 1.0, 0.0, rho, soc)

    hov_max = op_T(float(T_i.max()), rho0, 0.5)
    if hov_max is None:
        ctx.gate("most-loaded lift motor can produce its hover thrust", False, True, "bool", "FAIL", drivers=ldrv)
    else:
        ctx.gate("most-loaded lift motor: hover ESC duty", hov_max["duty"], 0.80, "<=", "FAIL",
                 note="TEAM CRITERION (same as the propulsion stage): >= 20 % duty headroom", drivers=ldrv)

    # ---- 2. thrust margin in the worst air of the plan ----------------------------------------------
    rows = []
    for tag, rho, soc in (("ISA sea level, 50 % battery", rho0, 0.5),
                          (f"'{thin['name']}' ({thin.get('temperature_C', 15):g} C, {thin.get('altitude_msl', 0):g} m), 10 % battery", rho_thin, 0.1)):
        w_ = wot(rho, soc)
        Tm = w_["thrust_N"] if w_ else 0.0
        h_ = op_T(T_mean, rho, soc)
        rows.append(dict(case=tag, rho=rho, soc=soc, T_max_N=Tm, TW=n * Tm / W, hover_duty=h_["duty"] if h_ else None,
                         hover_P_W=n * h_["P_elec_per_motor"] if h_ else None, I_motor=h_["I_motor"] if h_ else None,
                         motor_loss_W=h_["motor_loss_W"] if h_ else None, rpm=h_["rpm"] if h_ else None,
                         Q_per_T=(lp.thrust_torque(h_["rpm"], 0.0, rho)[1] / max(h_["thrust_N"], 1e-9)) if h_ else None))
    nom, bad = rows
    ctx.gate(f"lift T/W in the thinnest air of the plan: {bad['case']}", bad["TW"], cfg.get("tw_worst_air_warn", 1.5), ">=",
             "FAIL" if bad["TW"] < cfg.get("tw_worst_air_fail", 1.25) else "WARN",
             note=f"air density {bad['rho']:.3f} kg/m3 vs 1.225 at ISA: thrust scales with density. TEAM CRITERION: >= 1.5 "
                  f"(warn), < 1.25 no-go. ISA value {nom['TW']:.2f}", drivers=["/environment_cases", "/propulsion/units/lift"])
    if bad["hover_duty"] is not None:
        ctx.gate(f"hover ESC duty in {bad['case']}", bad["hover_duty"], 0.85, "<=", "WARN",
                 note="landing with an empty battery on the hottest day is the worst hover of the mission")

    # ---- 3. control authority at hover ----------------------------------------------------------------
    auth = {}
    for r in rows:
        if r["hover_duty"] is None:
            continue
        d = max(min(r["T_max_N"] - float(T_i.max()), float(T_i.min()) * 0.9), 0.0)      # usable +-thrust change per motor
        arm_y = float(np.abs(pos[:, 1] - cg[1]).mean())
        Lx = float(np.abs(pos[:, 0] - cg[0]).sum())
        tau = dict(roll=n * d * arm_y, pitch=d * Lx, yaw=n * d * r["Q_per_T"])
        acc = dict(roll=tau["roll"] / I["Ixx"], pitch=tau["pitch"] / I["Iyy"], yaw=tau["yaw"] / I["Izz"])
        auth[r["case"]] = dict(dT_N=d, torque_Nm=tau, accel_rad_s2=acc)
    a_nom = auth.get(nom["case"])
    a_bad = auth.get(bad["case"])
    lim = dict(roll=cfg.get("roll_accel_min", 4.0), pitch=cfg.get("pitch_accel_min", 4.0), yaw=cfg.get("yaw_accel_min", 0.35))
    for ax in ("roll", "pitch", "yaw"):
        a = (a_bad or a_nom or {}).get("accel_rad_s2", {}).get(ax)
        ctx.gate(f"hover {ax} authority, worst air [rad/s2]", a, lim[ax], ">=", "WARN", "rad/s2",
                 note=("differential lift-motor torque only: quadplanes are weak in yaw; tilting the lift motors 2-4 deg "
                       "(alternating) multiplies yaw authority" if ax == "yaw" else
                       "from the thrust headroom of the lift motors about the actual CG and inertia")
                 + ". TEAM CRITERION" + (f"; ISA value {a_nom['accel_rad_s2'][ax]:.2f}" if a_nom else ""))

    # ---- 4. crosswind hover ---------------------------------------------------------------------------
    A_side, Ax_side = side_area_moment(plan, cg[0])
    w_cases = sorted({float(e.get("wind_gust", e.get("wind_mean", 0)) or 0) for e in envs})
    v_w = max(w_cases) if w_cases else 0.0
    q = 0.5 * rho0 * v_w ** 2
    F_side = q * A_side
    tilt = math.degrees(math.atan2(F_side, W))
    M_wv = abs(q * Ax_side)
    tau_yaw = (a_nom or {}).get("torque_Nm", {}).get("yaw", 0.0)
    v_hold = math.sqrt(tau_yaw / max(0.5 * rho0 * abs(Ax_side), 1e-9)) if tau_yaw else 0.0
    ctx.gate(f"hover tilt to hold position in a {v_w:g} m/s crosswind [deg]", round(tilt, 1), cfg.get("tilt_max_deg", 15.0), "<=", "WARN", "deg",
             note=f"side drag area {A_side*1e4:.0f} cm2 (fuselage side + tail panels, Cd ~1, +-50 %); ArduPilot Q_ANGLE_MAX default 30 deg")
    ctx.gate("crosswind the yaw authority can hold a heading in [m/s] vs worst gust", round(v_hold, 1), v_w, ">=", "WARN", "m/s",
             note=f"weathervane moment {M_wv:.2f} N m at {v_w:g} m/s vs yaw torque {tau_yaw:.2f} N m. If this warns: set "
                  "Q_WVANE_ENABLE so it turns into wind, take off and land nose into wind", drivers=["/environment_cases"])

    # ---- 5. forward transition ------------------------------------------------------------------------
    S = aero.get("S") or P.main_wing(plan).get("area")
    CD0, k, CLm = aero.get("CD0", 0.045), aero.get("k", 0.07), aero.get("CL_minD", 0.0)
    CLmax = aero.get("CLmax_trimmed_pess") or aero.get("CLmax_trimmed") or 1.0
    vs_p = aero.get("V_stall_pess") or math.sqrt(2 * W / (rho0 * S * CLmax))
    params = (plan.get("avionics") or {}).get("params") or {}
    v_tr = float(params.get("AIRSPEED_MIN", 1.2 * vs_p))
    cr = P.units(plan, "cruise")
    tr = None
    if cr:
        cu = cr[0]
        cm, cp_ = PR.Motor(cu["motor"]), PR.Prop(cu["prop"], ctx.plan_path.parent)
        hov = op_T(T_mean, rho_thin, 0.3)
        I_hover = n * hov["I_bus_per_motor"] if hov else 0.0
        P_hover = n * hov["P_elec_per_motor"] if hov else 0.0
        v, t, x, e, dt, i_peak, ok = 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, True
        while v < v_tr and t < 60:
            qd = 0.5 * rho_thin * v * v
            CL = min(CLmax, W / max(qd * S, 1e-9))
            lift_frac = min(1.0, qd * S * CL / W)
            D = qd * S * (1.5 * CD0 + k * (CL - CLm) ** 2)             # pessimistic drag band + lift props turning
            o = PR.operating_point(cu, cm, cp_, pack, 1.0, v, rho_thin, 0.3, other_bus_A=I_hover * (1 - lift_frac))
            if not o or o["thrust_N"] <= D:
                ok = False
                break
            a = (o["thrust_N"] - D) / M
            p_now = o["P_elec_per_motor"] + P_hover * (1 - lift_frac)
            i_peak = max(i_peak, o["I_bus_total"])
            v += a * dt; x += v * dt; t += dt; e += p_now * dt
        tr = dict(v_transition=v_tr, time_s=t if ok else None, distance_m=x if ok else None, energy_Wh=e / 3600 if ok else None,
                  peak_battery_A=i_peak, peak_cell_A=i_peak / pack.P, air=thin["name"])
        allow = float(P.val(((plan.get("analysis") or {}).get("vtol") or {}).get("transition_s"), 10.0))
        ctx.gate(f"forward transition time to {v_tr:.1f} m/s in '{thin['name']}' air [s] vs the mission allowance", None if not ok else round(t, 1),
                 allow, "<=", "WARN", "s", note="pusher at full throttle, lift motors unloading as the wing takes over, 30 % battery, "
                 "pessimistic drag. Longer than the allowance = the mission energy budget is optimistic (raise analysis.vtol.transition_s)")
        ctx.gate("battery current per cell during transition (lift + pusher) / cell rating", i_peak / pack.P / pack.i_cell_max, 1.0, "<=", "FAIL",
                 note=f"{i_peak:.0f} A total, {i_peak/pack.P:.1f} A per cell vs {pack.i_cell_max} A")
        # back-transition: decelerate on drag alone from cruise to the transition speed
        vc = plan["envelope"]["v_cruise"]
        vv = np.linspace(vc, max(v_tr * 0.8, 3.0), 60)
        Dv = [0.5 * rho0 * u * u * S * (CD0 + k * (min(CLmax, W / (0.5 * rho0 * u * u * S)) - CLm) ** 2) for u in vv]
        dist = float(np.trapezoid([M * u / d_ for u, d_ in zip(vv, Dv)][::-1], vv[::-1]))
        tr["back_transition_distance_m"] = dist
        ctx.gate("back-transition: distance to slow from cruise on drag alone [m]", round(dist), None, "info", "INFO", "m",
                 note="plan the landing approach leg at least this long (ArduPilot brakes with the lift motors too; this is the gentle bound)")

    # ---- 6. one lift motor out --------------------------------------------------------------------------
    tw2 = 2 * nom["T_max_N"] / W
    ctx.gate("one lift motor failed in hover: T/W on the two motors that can still balance", tw2, 1.0, ">=", "WARN",
             note="a 4-motor lift system cannot hold attitude AND yaw with one motor out; >= 1.0 means a spinning but arrested descent, "
                  "below 1.0 the aircraft falls. Mitigation: keep hover low and short, or fly away on the wing above "
                  f"{v_tr:.0f} m/s (the fixed-wing mode does not need the lift motors)")

    # ---- figure --------------------------------------------------------------------------------------
    plt = style()
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    ax[0].scatter(pos[:, 1] * 1000, -pos[:, 0] * 1000, s=160, color=PALETTE[0], zorder=3)
    for i, p_ in enumerate(pos):
        ax[0].annotate(f"motor {i+1}: {T_i[i]/G*1000:.0f} g", (p_[1] * 1000, -p_[0] * 1000), xytext=(0, 12 if p_[0] < xc else -18),
                       textcoords="offset points", ha="center", fontsize=8, color=INK2, zorder=4)
    ax[0].scatter([yc * 1000], [-xc * 1000], marker="+", s=200, color=INK2, label="lift-thrust centre", zorder=5)
    ax[0].scatter([cg[1] * 1000], [-cg[0] * 1000], marker="o", s=60, color=PALETTE[3], label="CG", zorder=6)
    ax[0].set(xlabel="y [mm] (right +)", ylabel="-x [mm] (nose up the page)", title="Hover thrust each lift motor must carry (top view)")
    ax[0].axis("equal"); ax[0].margins(0.25); ax[0].legend(loc="center", fontsize=8, bbox_to_anchor=(0.5, 0.25))
    names = ["roll", "pitch", "yaw"]
    xs = np.arange(3)
    for j, (lab, a_) in enumerate(((nom["case"], a_nom), (bad["case"], a_bad))):
        if a_:
            ax[1].bar(xs + (j - 0.5) * 0.36, [a_["accel_rad_s2"][k_] for k_ in names], 0.34, color=PALETTE[j * 2], label=lab[:38])
    for i_, k_ in enumerate(names):
        ax[1].plot([i_ - 0.42, i_ + 0.42], [lim[k_]] * 2, color=INK2, lw=1.2, ls="--")
    ax[1].set_yscale("log"); ax[1].set_xticks(xs, names)
    ax[1].set(ylabel="angular acceleration available [rad/s2]", title="Hover control authority (dashes: team criterion)")
    ax[1].legend(fontsize=7)
    save(fig, ctx.dir / "vtol_hover_authority.png")
    ctx.add_file(ctx.dir / "vtol_hover_authority.png", "Lift-motor loading and hover control authority", "figure")

    hints = {}
    if a_bad or a_nom:
        acc = (a_bad or a_nom)["accel_rad_s2"]
        hints = {"Q_A_ACCEL_R_MAX": int(min(110000, round(0.5 * math.degrees(acc["roll"]) * 100, -2))),
                 "Q_A_ACCEL_P_MAX": int(min(110000, round(0.5 * math.degrees(acc["pitch"]) * 100, -2))),
                 "Q_A_ACCEL_Y_MAX": int(min(27000, round(0.5 * math.degrees(acc["yaw"]) * 100, -2)))}
    data = dict(thrust_centre_m=[float(xc), float(yc)], cg_offset_mm=off_mm, front_share=float(f_front),
                motor_hover_thrust_N=[float(v) for v in T_i], cases=rows, authority=auth, side_area_m2=A_side,
                crosswind=dict(v=v_w, tilt_deg=tilt, weathervane_Nm=M_wv, v_heading_hold=v_hold), transition=tr,
                motor_out_TW2=tw2, ardupilot_hints=hints)
    summ = dict(cg_to_thrust_centre_mm=round(off_mm, 1), front_share_pct=round(100 * f_front, 1),
                TW_isa=round(nom["TW"], 2), TW_worst_air=round(bad["TW"], 2), worst_air=bad["case"],
                roll_pitch_yaw_accel_worst=[round(v, 2) for v in ((a_bad or a_nom or {}).get("accel_rad_s2") or {}).values()],
                crosswind_heading_hold_mps=round(v_hold, 1),
                transition_s=None if not tr or tr["time_s"] is None else round(tr["time_s"], 1),
                transition_distance_m=None if not tr or tr["distance_m"] is None else round(tr["distance_m"]),
                motor_out_TW=round(tw2, 2))
    return dict(summary=summ, data=data)
