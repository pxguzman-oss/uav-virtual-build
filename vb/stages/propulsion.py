"""Stage 5 - propulsion & electrical (eCalc-style motor/prop/ESC/battery operating points).

Cruise unit: static WOT (full / nominal / low pack, sea level and hottest environment), cruise operating
point from the aero drag polar, WOT climb, max level speed, motor thermal, electrical margins, mission
energy (segments x wind) vs usable energy with reserve. Lift units (VTOL): hover table over AUW cases,
T/W, duty, WOT current per cell, cell heating, rpm band (-> structures)."""
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


def run(ctx):
    import propulsion as PR
    plan = ctx.plan
    mass = ctx.data("mass")
    aero = ctx.data("aero_lowfi", required=False)
    M = mass["mass_kg"]
    W = M * G
    env = plan["envelope"]
    envs = plan.get("environment_cases") or [dict(name="ISA", temperature_C=15, altitude_msl=0)]
    hot = max(envs, key=lambda e: e.get("temperature_C", 15))
    rho0 = rho_at(15, 0)
    rho_hot = rho_at(hot.get("temperature_C", 15), hot.get("altitude_msl", 0))
    pack = PR.Pack(plan["energy"])
    w = P.main_wing(plan)
    S = w.get("area") or (w["root_chord"] + w["tip_chord"]) / 2 * w["span"]
    AR = w["span"] ** 2 / S
    gates_cfg = (plan.get("gates") or {}).get("propulsion", {})
    inst = gates_cfg.get("installation_factor", 1.08)

    if aero:
        CD0, k, CLm = aero["CD0"], aero["k"], aero["CL_minD"]
        drag_src = "aero_lowfi polar fit (AeroBuildup + excrescence)"
    else:
        CD0, k, CLm = 0.032, 1 / (math.pi * 0.78 * AR), 0.0
        drag_src = "FALLBACK polar CD0 0.032, e 0.78 (aero stage missing)"
        ctx.note(drag_src)

    P_av = float(plan.get("avionics", {}).get("avionics_power_W", 4.0))
    an_aero = (plan.get("analysis") or {}).get("aero") or {}
    cd0_u = P.val(an_aero.get("cd0_uncertainty"), 0.5)

    def drag(V, rho=rho0, n=1.0, cd0_mult=1.0):
        q = 0.5 * rho * V * V
        CL = n * W / (q * S)
        CD = CD0 * cd0_mult + k * (CL - CLm) ** 2
        return q * S * CD, CL, CD

    cr_units = P.units(plan, "cruise")
    if not cr_units:
        raise ValueError("no propulsion unit with role 'cruise'")
    unit = cr_units[0]
    motor, prop = PR.Motor(unit["motor"]), PR.Prop(unit["prop"], ctx.plan_path.parent)
    res = dict(pack=dict(config=f"{pack.S}S{pack.P}P", capacity_Ah=pack.cap, R_pack_ohm=pack.R, E_nom_Wh=pack.E_nom,
                         usable_Wh=pack.E_nom * pack.usable), drag_source=drag_src, cruise_unit={}, vtol={})
    # ---- static WOT --------------------------------------------------------------------------------
    static = {}
    for label, soc in (("full", 1.0), ("nominal", 0.5), ("low", 0.10)):
        static[label] = PR.operating_point(unit, motor, prop, pack, 1.0, 0.0, rho0, soc)
    wot = static["full"]
    wot_hot_low = PR.operating_point(unit, motor, prop, pack, 1.0, 0.0, rho_hot, 0.10)
    res["cruise_unit"]["static_wot"] = static
    tw = wot["thrust_N"] / W
    tw_worst = wot_hot_low["thrust_N"] / W
    # ---- cruise ------------------------------------------------------------------------------------
    Vc = env["v_cruise"]
    D, CL, CD = drag(Vc)
    T_req = D * inst
    cru = PR.duty_for_thrust(unit, motor, prop, pack, T_req, Vc, rho0, soc=0.5)
    if cru is None:
        raise RuntimeError(f"cruise unit cannot produce {T_req:.2f} N at {Vc} m/s")
    # endurance / range INCLUDING the avionics load (review M1), nominal and pessimistic drag (review M9)
    t_end, E_used = PR.endurance_cruise(unit, motor, prop, pack, T_req, Vc, rho0, p_aux_W=P_av)
    T_req_p = drag(Vc, cd0_mult=1 + cd0_u)[0] * inst
    t_end_p, _ = PR.endurance_cruise(unit, motor, prop, pack, T_req_p, Vc, rho0, p_aux_W=P_av)
    res["cruise_unit"]["cruise"] = dict(cru, V=Vc, T_req_N=T_req, CL=CL, CD=CD, LD=CL / CD, endurance_min=t_end / 60,
                                        range_km=t_end * Vc / 1000, endurance_with_reserve_min=t_end * (1 - pack.reserve) / 60,
                                        P_total_W=cru["P_elec_per_motor"] + P_av, P_avionics_W=P_av)
    res["cruise_unit"]["range_band"] = dict(
        basis="UNCALIBRATED (no flight data): steady model at cruise speed, usable energy, avionics included",
        V=Vc, cd0=CD0, cd0_uncertainty=cd0_u,
        nominal=dict(endurance_min=t_end / 60, range_km=t_end * Vc / 1000),
        pessimistic_drag=dict(endurance_min=t_end_p / 60, range_km=t_end_p * Vc / 1000, cd0=CD0 * (1 + cd0_u)))
    climb = PR.operating_point(unit, motor, prop, pack, 1.0, Vc, rho0, 0.5)
    roc = (climb["thrust_N"] / inst - D) * Vc / W
    res["cruise_unit"]["climb_rate_ms"] = roc
    vmax = None
    for V in np.arange(Vc, 45, 0.25):
        op = PR.operating_point(unit, motor, prop, pack, 1.0, V, rho0, 0.5)
        if op is None or op["thrust_N"] < drag(V)[0] * inst:
            vmax = float(V); break
    res["cruise_unit"]["v_max_level"] = vmax
    # power curve (battery power vs airspeed)
    pcurve = []
    Vs = aero["V_stall"] if aero else 9.0
    for V in np.arange(max(Vs * 1.1, 6), min((vmax or 25), 30) + 0.01, 0.5):
        op = PR.duty_for_thrust(unit, motor, prop, pack, drag(V)[0] * inst, V, rho0, soc=0.5)
        opp = PR.duty_for_thrust(unit, motor, prop, pack, drag(V, cd0_mult=1 + cd0_u)[0] * inst, V, rho0, soc=0.5)
        if op:
            # P_W / Wh_per_km INCLUDE the avionics load, like the SITL battery (review M1)
            pcurve.append(dict(V=float(V), P_prop_W=op["P_elec_per_motor"], P_W=op["P_elec_per_motor"] + P_av, duty=op["duty"],
                               I=op["I_bus_total"], Wh_per_km=(op["P_elec_per_motor"] + P_av) / (V * 3.6),
                               P_W_pess_drag=(opp["P_elec_per_motor"] + P_av) if opp else None))
    res["cruise_unit"]["power_curve"] = pcurve
    # ---- thermal -----------------------------------------------------------------------------------
    m_g = unit["motor"].get("mass", 0.077) * 1000
    C_th, R_th = 0.45 * m_g, unit["motor"].get("r_th_KW", 1.5)
    tau = C_th * R_th
    T_amb = hot.get("temperature_C", 15)
    T_lim = unit["motor"].get("t_limit_C", 100.0)
    dT_allow = T_lim - T_amb

    def t_to_limit(Pw):
        ss = Pw * R_th
        return float("inf") if ss <= dT_allow else -tau * math.log(1 - dT_allow / ss)
    t_wot = t_to_limit(wot["motor_loss_W"])
    res["cruise_unit"]["thermal"] = dict(C_JK=C_th, R_th_KW=R_th, tau_s=tau, T_amb_C=T_amb,
                                         T_cruise_ss_C=T_amb + cru["motor_loss_W"] * R_th, wot_seconds_to_limit=t_wot)
    # ---- gates (cruise unit) -----------------------------------------------------------------------
    Imax, Ibat = wot["I_motor"], wot["I_bus_total"]
    wiring = unit.get("wiring", {})
    mdrv = ["/propulsion/units/cruise/motor/rm", "/propulsion/units/cruise/motor/i0", "/propulsion/units/cruise/prop/ct_scale"]
    ctx.gate("ESC continuous / WOT motor current", unit["esc"]["i_cont"] / Imax, 1.2, ">=", "FAIL",
             note=f"{unit['esc']['i_cont']} A vs {Imax:.1f} A", drivers=mdrv)
    ctx.gate("connector rating / WOT battery current", wiring.get("connector_i_cont", 60) / Ibat, 1.3, ">=", "FAIL",
             note=f"{wiring.get('connector', '?')} {wiring.get('connector_i_cont', 60)} A vs {Ibat:.1f} A")
    amp = PR.AWG_CHASSIS_A.get(wiring.get("awg"))
    if amp:
        ctx.gate(f"AWG{wiring.get('awg')} chassis rating / WOT current (short duty)", amp / Ibat, 1.0, ">=", "WARN")
    ctx.gate("per-cell current at WOT / cell continuous rating", wot["I_cell"] / pack.i_cell_max, 0.5, "<=", "FAIL",
             note=f"{wot['I_cell']:.1f} A/cell vs {pack.i_cell_max} A")
    if motor.i_max:
        ctx.gate("motor max current / WOT current", motor.i_max / Imax, 1.15, ">=", "FAIL", drivers=mdrv)
    ctx.gate("motor loss limit / cruise loss", motor.loss_limit / max(cru["motor_loss_W"], 1e-6), 1.5, ">=", "FAIL")
    ctx.gate(f"WOT time to {T_lim:.0f} C at {T_amb:.0f} C ambient [s] (>= 60 s launch climb)", min(t_wot, 1e4), 60.0, ">=", "WARN", "s",
             note="lumped motor thermal model R_th 1.5 K/W (estimate)")
    ctx.gate("cruise ESC duty (headroom for gusts/climb)", cru["duty"], 0.65, "<=", "FAIL", drivers=mdrv + ["/analysis/aero"])
    if plan["airframe"].get("launch", {}).get("method") == "hand":
        ctx.gate("static T/W (hand launch, full pack, sea level)", tw, 0.6, ">=", "FAIL", drivers=mdrv)
        ctx.gate("static T/W worst case (hot day, 10 % SOC)", tw_worst, 0.5, ">=", "WARN", note=f"rho {rho_hot:.3f}")
    ctx.gate("climb rate at WOT, cruise speed [m/s]", roc, 2.0, ">=", "WARN", "m/s")
    if vmax:
        ctx.gate("max level speed vs VNE (overspeed possible in level flight?)", vmax, env["v_ne"], "<=", "WARN", "m/s",
                 note="if > VNE, cap THR_MAX / AIRSPEED_MAX")

    # ---- VTOL phases for the mission model (review M7) ----------------------------------------------------
    lifts0 = P.units(plan, "lift")
    vprof = None
    if lifts0:
        l0 = lifts0[0]
        lm0, lp0 = PR.Motor(l0["motor"]), PR.Prop(l0["prop"], ctx.plan_path.parent)
        hov0 = PR.duty_for_thrust(l0, lm0, lp0, pack, M * G / l0["count"], 0.0, rho0, soc=0.5)
        vt_cfg = (plan.get("analysis") or {}).get("vtol") or {}
        vprof = dict(P_hover_W=(l0["count"] * hov0["P_elec_per_motor"]) if hov0 else float("nan"),
                     takeoff_alt_m=float(P.val(vt_cfg.get("takeoff_alt"), 30.0)),
                     climb_rate=float(P.val(vt_cfg.get("climb_rate"), 2.5)),        # ArduPilot Q_WP_SPEED_UP default 2.5 m/s
                     descent_rate=float(P.val(vt_cfg.get("descent_rate"), 1.5)),    # Q_WP_SPEED_DN default 1.5 m/s
                     land_final_alt=float(P.val(vt_cfg.get("land_final_alt"), 6.0)),  # Q_LAND_FINAL_ALT default 6 m
                     land_speed=float(P.val(vt_cfg.get("land_speed"), 0.5)),          # Q_LAND_SPEED default 0.5 m/s
                     transition_s=float(P.val(vt_cfg.get("transition_s"), 10.0)),     # team allowance per transition
                     hover_allowance_s=float(P.val(vt_cfg.get("hover_allowance_s"), 30.0)),  # team allowance per landing
                     source="ArduPilot QuadPlane defaults Q_WP_SPEED_UP/DN, Q_LAND_FINAL_ALT, Q_LAND_SPEED; transition and "
                            "hover allowances = team values (analysis.vtol)")

    # ---- mission energy ------------------------------------------------------------------------------
    usable = pack.E_nom * pack.usable
    missions = []
    for mi in plan.get("missions", []):
        envc = next((e for e in envs if e["name"] == mi.get("environment")), envs[0])
        wind = envc.get("wind_mean", 0.0)
        rho_m = rho_at(envc.get("temperature_C", 15), envc.get("altitude_msl", 0))
        rows, E, t_tot, d_tot = [], 0.0, 0.0, 0.0
        cruise_legs = 0
        for sgm in mi["segments"]:
            kind = sgm["kind"]
            V = sgm.get("airspeed", Vc)
            op = PR.duty_for_thrust(unit, motor, prop, pack, drag(V, rho_m)[0] * inst, V, rho_m, soc=0.5)
            Pw = op["P_elec_per_motor"] if op else float("nan")
            if kind in ("cruise", "survey"):
                gs = V - wind if cruise_legs % 2 == 0 else V + wind      # out into wind, back downwind
                cruise_legs += 1
                t = sgm["distance"] / max(gs, 0.5)
                e = Pw * t / 3600
                d_tot += sgm["distance"]
            elif kind == "loiter":
                t = sgm.get("duration", 0)
                e = Pw * t / 3600
            elif kind == "climb":
                h = sgm.get("altitude", 100)
                opc = PR.operating_point(unit, motor, prop, pack, 1.0, V, rho_m, 0.9)
                rate = max((opc["thrust_N"] / inst - drag(V, rho_m)[0]) * V / W, 0.5)
                t = h / rate
                e = opc["P_elec_per_motor"] * t / 3600
            elif kind == "launch" and vprof and plan["airframe"].get("launch", {}).get("method") == "vtol":
                # VTOL takeoff: hover climb to takeoff_alt + forward transition (lift at hover power + pusher WOT)
                t_c = vprof["takeoff_alt_m"] / vprof["climb_rate"]
                t_tr = vprof["transition_s"]
                t = t_c + t_tr
                e = (1.10 * vprof["P_hover_W"] * t_c + (vprof["P_hover_W"] + wot["P_elec_per_motor"]) * t_tr) / 3600
                kind = "launch (VTOL takeoff + transition)"
            elif kind in ("land", "glide") and vprof and plan["airframe"].get("landing", {}).get("method") == "vtol":
                # back-transition + hover allowance + descent at Q_WP_SPEED_DN to Q_LAND_FINAL_ALT, then Q_LAND_SPEED
                t_tr, t_h = vprof["transition_s"], vprof["hover_allowance_s"]
                t_d = (vprof["takeoff_alt_m"] - vprof["land_final_alt"]) / vprof["descent_rate"] + vprof["land_final_alt"] / vprof["land_speed"]
                t = t_tr + t_h + t_d
                e = (vprof["P_hover_W"] * (t_tr + t_h) + 0.95 * vprof["P_hover_W"] * t_d) / 3600
                kind = "land (VTOL: back-transition, hover, descent)"
            elif kind == "launch":
                t, e = 10.0, wot["P_elec_per_motor"] * 10 / 3600
            elif kind in ("land", "glide"):
                t, e = 60.0, 0.3 * Pw * 60 / 3600
            else:
                t, e = sgm.get("duration", 0), Pw * sgm.get("duration", 0) / 3600
            e_av = P_av * t / 3600
            rows.append(dict(kind=kind, time_s=t, energy_prop_Wh=e, energy_avionics_Wh=e_av, energy_Wh=e + e_av,
                             power_W=(e + e_av) * 3600 / max(t, 1e-9)))
            E += e + e_av; t_tot += t
        reserve = mi.get("reserve_fraction", pack.reserve)
        avail = usable * (1 - reserve)
        opn = PR.duty_for_thrust(unit, motor, prop, pack, drag(Vc, rho_m)[0] * inst, Vc, rho_m, soc=0.5)
        opp = PR.duty_for_thrust(unit, motor, prop, pack, drag(Vc, rho_m, cd0_mult=1 + cd0_u)[0] * inst, Vc, rho_m, soc=0.5)
        r_d = (opp["P_elec_per_motor"] / opn["P_elec_per_motor"]) if (opn and opp) else float("nan")
        E_av_m = sum(r["energy_avionics_Wh"] for r in rows)
        E_pess = (E - E_av_m) * r_d + E_av_m
        E_av = sum(r["energy_avionics_Wh"] for r in rows)
        missions.append(dict(name=mi["name"], environment=envc["name"], wind_mps=wind, segments=rows, energy_Wh=E,
                             energy_prop_Wh=E - E_av, energy_avionics_Wh=E_av, rho=rho_m, time_min=t_tot / 60,
                             distance_km=d_tot / 1000, usable_Wh=usable, available_after_reserve_Wh=avail, margin=avail / E - 1,
                             energy_pess_drag_Wh=E_pess, pess_drag_power_ratio=r_d))
        ctx.gate(f"mission '{mi['name']}' energy vs usable after {reserve*100:.0f} % reserve [Wh]", E, avail, "<=", "FAIL", "Wh",
                 note=f"{E:.0f} Wh needed, {avail:.0f} Wh available ({envc['name']}, wind {wind} m/s)",
                 drivers=["/analysis/aero (drag)", "/propulsion/units/cruise", "/energy/usable_fraction"])
        ctx.gate(f"mission '{mi['name']}' energy with PESSIMISTIC drag (CD0 x {1 + cd0_u:.2f}) vs usable after reserve [Wh]", E_pess, avail,
                 "<=", "WARN", "Wh", note=f"uncalibrated drag band until F2/F3; propulsive power x {r_d:.2f}",
                 drivers=["/analysis/aero/cd0_uncertainty", "/analysis/aero (drag)"])
    res["missions"] = missions
    res["vtol_profile"] = vprof
    res["ardupilot_hints"] = {"TRIM_THROTTLE": round(cru["duty"] * 100), "BATT_CAPACITY": round(pack.cap * 1000),
                              "BATT_ARM_VOLT": round(pack.S * 3.5, 1), "BATT_LOW_VOLT": round(pack.S * 3.1, 1),
                              "BATT_CRT_VOLT": round(pack.S * 2.9, 1), "MOT_BAT_VOLT_MAX": round(pack.S * 4.2, 1),
                              "MOT_BAT_VOLT_MIN": round(pack.S * 3.0, 1),
                              "BATT_LOW_MAH": round(pack.cap * 1000 * (1 - pack.usable + pack.reserve * pack.usable)),
                              "BATT_CRT_MAH": round(pack.cap * 1000 * (1 - pack.usable))}

    # takeoff throttle cap: static T/W ~0.65 is enough for a hand launch and limits the climb overspeed
    if plan["airframe"].get("launch", {}).get("method") == "hand" and tw > 0.85:
        opt = PR.duty_for_thrust(unit, motor, prop, pack, 0.65 * W, 0.0, rho0, soc=0.5)
        if opt:
            res["ardupilot_hints"]["TKOFF_THR_MAX"] = int(min(100, max(60, math.ceil(opt["duty"] * 20) * 5)))
    # ---- lift units (VTOL) ----------------------------------------------------------------------------
    lifts = P.units(plan, "lift")
    if lifts:
        lift = lifts[0]
        n = lift["count"]
        lm, lp = PR.Motor(lift["motor"]), PR.Prop(lift["prop"], ctx.plan_path.parent)
        wot_l = PR.operating_point(lift, lm, lp, pack, 1.0, 0.0, rho0, 0.5)
        wot_l_full = PR.operating_point(lift, lm, lp, pack, 1.0, 0.0, rho0, 1.0)
        cases = sorted(set([round(M, 3)] + list(plan.get("analysis", {}).get("vtol_auw_cases_kg") or [])))
        rows = []
        # ArduPilot hover throttle domain (review M8): Q_M_THST_HOVER and QTUN.ThO are THRUST-LINEARISED throttle t;
        # the ESC sees duty d = spin_min + (spin_max - spin_min) * a with t = expo a^2 + (1 - expo) a (no battery
        # voltage compensation unless Q_M_BAT_VOLT_MAX is set). Propulsion solves for d.
        qp = {}
        for f_ in ("plane_defaults.parm", "quadplane_defaults.parm"):
            fp = Path(__file__).resolve().parents[2] / "data" / "ardupilot" / f_
            if fp.exists():
                for line in fp.read_text().splitlines():
                    line = line.split("#")[0].split()
                    if len(line) >= 2:
                        qp[line[0]] = float(line[1])
        qp.update({k: float(v) for k, v in ((plan.get("avionics") or {}).get("params") or {}).items() if k.startswith("Q_M_")})
        expo, smin, smax = qp.get("Q_M_THST_EXPO", 0.65), qp.get("Q_M_SPIN_MIN", 0.15), qp.get("Q_M_SPIN_MAX", 0.95)

        def tho_from_duty(d):
            a = min(max((d - smin) / (smax - smin), 0.0), 1.0)
            return expo * a * a + (1 - expo) * a
        res["vtol"]["throttle_mapping"] = dict(Q_M_THST_EXPO=expo, Q_M_SPIN_MIN=smin, Q_M_SPIN_MAX=smax,
                                               note="ThO = expo a^2 + (1-expo) a, a = (duty - spin_min)/(spin_max - spin_min)")
        for auw in cases:
            Th = auw * G / n
            hov = PR.duty_for_thrust(lift, lm, lp, pack, Th, 0.0, rho0, soc=0.5)
            hov_hot = PR.duty_for_thrust(lift, lm, lp, pack, Th, 0.0, rho_hot, soc=0.1)
            twr = n * wot_l["thrust_N"] / (auw * G)
            if hov is None:
                rows.append(dict(auw_kg=auw, TW_max=twr, status="CANNOT HOVER")); continue
            P_tot = n * hov["P_elec_per_motor"]
            t_hover = pack.cap * pack.usable * pack.ocv(0.5) / P_tot * 60
            q_cell = hov["I_cell"] ** 2 * pack.r_cell
            rows.append(dict(auw_kg=auw, is_plan_auw=abs(auw - M) < 1e-3, thrust_per_motor_g=Th / G * 1000, duty=hov["duty"],
                             duty_hot_low=hov_hot["duty"] if hov_hot else None, rpm=hov["rpm"], I_bat=hov["I_bus_total"],
                             I_cell=hov["I_cell"], I_motor=hov["I_motor"], P_total_W=P_tot, g_per_W=hov["g_per_W"], TW_max=twr,
                             motor_loss_W=hov["motor_loss_W"], wot_I_motor=wot_l_full["I_motor"], wot_I_bat=wot_l_full["I_bus_total"],
                             wot_I_cell=wot_l_full["I_cell"], wot_thrust_g=wot_l["thrust_g"], wot_rpm=wot_l_full["rpm"],
                             hover_minutes=t_hover, Q_M_THST_HOVER=round(tho_from_duty(hov["duty"]), 3), thr_lin=tho_from_duty(hov["duty"]),
                             V_bus=hov["V_bus"], cell_heat_W=q_cell,
                             cell_dT_120s_hover_K=q_cell * 120 / (plan["energy"]["cell"]["mass"] * 1000 * 1.0)))
        res["vtol"]["hover_table"] = rows
        ok_rows = [r for r in rows if "rpm" in r]
        res["vtol"]["rpm_band"] = [min(r["rpm"] for r in ok_rows), max(r["wot_rpm"] for r in ok_rows)] if ok_rows else None
        worst = max(rows, key=lambda r: r["auw_kg"])
        mine = next((r for r in rows if r.get("is_plan_auw")), rows[0])
        ldrv = ["/propulsion/units/lift/motor", "/propulsion/units/lift/prop/ct_scale", "/analysis/vtol_auw_cases_kg"]
        for r, tag in ((mine, "plan AUW"), (worst, "max AUW case")):
            ctx.gate(f"VTOL T/W at {tag} {r['auw_kg']:.2f} kg >= 1.6 x 1.10 (stand-in prop data margin)", r["TW_max"], 1.76, ">=",
                     "FAIL" if r["TW_max"] < 1.3 else "WARN",
                     note="TEAM CRITERION: >= 1.6 for attitude-control headroom (common multirotor practice targets ~2), < 1.3 = no-go; "
                          "+10 % margin required while the lift prop is the APC 7x5E stand-in (until the C2 thrust stand)", drivers=ldrv)
            if "duty" in r:
                ctx.gate(f"VTOL hover throttle at {tag} {r['auw_kg']:.2f} kg, thrust-linearised (Q_M_THST_HOVER / QTUN.ThO domain)",
                         r["thr_lin"], 0.60, "<=", "WARN",
                         note=f"ESC duty {r['duty']:.3f} -> ThO {r['thr_lin']:.3f} (expo {expo}, spin {smin}-{smax}); "
                              "ArduPilot docs 'Setting Hover Throttle': the value is usually between 0.2 and 0.6", drivers=ldrv)
                ctx.gate(f"VTOL hover ESC duty at {tag} {r['auw_kg']:.2f} kg", r["duty"], 0.80, "<=", "FAIL",
                         note="TEAM CRITERION: >= 20 % duty headroom for attitude control and gusts", drivers=ldrv)
        ctx.gate("lift ESC continuous / WOT lift motor current", lift["esc"]["i_cont"] / wot_l_full["I_motor"], 1.2, ">=", "FAIL",
                 note=f"{lift['esc']['i_cont']} A vs {wot_l_full['I_motor']:.1f} A")
        ctx.gate("per-cell current, all lift motors WOT / cell rating", wot_l_full["I_cell"] / pack.i_cell_max, 1.0, "<=", "FAIL",
                 note=f"{wot_l_full['I_cell']:.1f} A/cell vs {pack.i_cell_max} A -> set Q_M_BAT_CURR_MAX")
        if "I_cell" in mine:
            ctx.gate("per-cell current in hover / 0.5 x rating", mine["I_cell"] / pack.i_cell_max, 0.5, "<=", "WARN")
            ctx.gate("cell temperature rise per 120 s hover (adiabatic) [K]", mine["cell_dT_120s_hover_K"], 10.0, "<=", "WARN", "K")
            ctx.gate("lift motor loss in hover vs loss limit", mine["motor_loss_W"], lm.loss_limit, "<=", "WARN", "W")
        conn = lift.get("wiring", {}).get("connector_i_cont")
        res["ardupilot_hints"].update({"Q_M_THST_HOVER": mine.get("Q_M_THST_HOVER", 0.5),
                                       "Q_M_BAT_CURR_MAX": round(min(pack.i_cell_max * pack.P * 0.8, 1e3)),
                                       "INS_HNTCH_FREQ": round(mine.get("rpm", 9000) / 60)})
    # ---- figures ---------------------------------------------------------------------------------------
    plt = style()
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.8))
    duties = np.linspace(0.2, 1.0, 17)
    for i, (Vx, lab) in enumerate(((0.0, "static"), (Vc, f"{Vc:g} m/s"))):
        ops = [PR.operating_point(unit, motor, prop, pack, dd, Vx, rho0, 0.5) for dd in duties]
        ok = [(dd, o) for dd, o in zip(duties, ops) if o]
        ax[0].plot([dd * 100 for dd, _ in ok], [o["thrust_N"] for _, o in ok], color=PALETTE[i], label=f"thrust, {lab}")
        ax[1].plot([dd * 100 for dd, _ in ok], [o["I_bus_total"] for _, o in ok], color=PALETTE[i], label=f"battery current, {lab}")
    ax[0].axhline(T_req, color=INK2, ls=":", lw=1); ax[0].text(99, T_req, "cruise thrust required", color=INK2, va="bottom", ha="right")
    ax[0].axhline(W * 0.6, color=PALETTE[7], ls=":", lw=1); ax[0].text(99, W * 0.6, "T/W 0.6 (hand launch)", color=PALETTE[7], va="bottom", ha="right")
    ax[1].axhline(unit["esc"]["i_cont"], color=PALETTE[7], ls=":", lw=1); ax[1].text(21, unit["esc"]["i_cont"], "ESC continuous", color=PALETTE[7], va="bottom", ha="left")
    ax[1].set_ylim(0, max(unit["esc"]["i_cont"], wot["I_bus_total"]) * 1.15)
    ax[0].set(xlabel="throttle (ESC duty) [%]", ylabel="thrust [N]", title=f"{unit['motor'].get('model', '')} + {unit['prop'].get('model', '')}")
    ax[1].set(xlabel="throttle (ESC duty) [%]", ylabel="current [A]", title=f"Pack {pack.S}S{pack.P}P, 50 % SOC")
    ax[0].legend(loc="upper left"); ax[1].legend(loc="center left")
    if pcurve:
        ax[2].plot([p["V"] for p in pcurve], [p["P_W"] for p in pcurve], "o-", ms=3, color=PALETTE[0], label="battery power")
        ax2 = ax[2].twinx(); ax2.plot([p["V"] for p in pcurve], [p["Wh_per_km"] for p in pcurve], color=PALETTE[1], label="Wh/km")
        ax2.set_ylabel("energy per km [Wh/km]", color=PALETTE[1]); ax2.grid(False)
        ax[2].set(xlabel="airspeed [m/s]", ylabel="battery power, level [W]", title="Level-flight power (aero polar + prop/motor)")
    save(fig, ctx.dir / "propulsion_operating_points.png")
    ctx.add_file(ctx.dir / "propulsion_operating_points.png", "Thrust/current vs throttle and level-flight power", "figure")
    if missions:
        mi = missions[0]
        fig, ax = plt.subplots(figsize=(8, 2.8))
        left = 0
        seen = {}
        for i, r in enumerate(mi["segments"]):
            seen[r["kind"]] = seen.get(r["kind"], 0) + 1
            lab = r["kind"] + (f" {seen[r['kind']]}" if sum(x["kind"] == r["kind"] for x in mi["segments"]) > 1 else "")
            ax.barh([0], [r["energy_Wh"]], left=left, color=PALETTE[i % 8], height=0.5, label=lab)
            left += r["energy_Wh"]
        ax.axvline(mi["available_after_reserve_Wh"], color=PALETTE[7], ls="--", lw=1.4)
        ax.text(mi["available_after_reserve_Wh"], -0.36, "usable after reserve ", color=PALETTE[7], ha="right", va="center")
        ax.axvline(mi["usable_Wh"], color=INK2, ls=":", lw=1.2)
        ax.text(mi["usable_Wh"], -0.36, " usable", color=INK2, va="center")
        ax.set_yticks([]); ax.set_xlabel("energy [Wh]")
        ax.yaxis.grid(False); ax.set_ylim(-0.45, 0.4)
        ax.set_xlim(0, max(left, mi["usable_Wh"]) * 1.15)
        ax.set_title(f"Mission energy: {mi['name']} ({mi['distance_km']:.0f} km, wind {mi['wind_mps']} m/s) = {mi['energy_Wh']:.0f} Wh", loc="left")
        ax.legend(ncol=7, loc="upper center", bbox_to_anchor=(0.5, -0.32), fontsize=8)
        save(fig, ctx.dir / "mission_energy.png")
        ctx.add_file(ctx.dir / "mission_energy.png", "Mission energy budget by segment vs usable energy", "figure")
    if lifts and res["vtol"].get("hover_table"):
        rows = [r for r in res["vtol"]["hover_table"] if "duty" in r]
        fig, ax = plt.subplots(1, 2, figsize=(11, 3.6))
        from matplotlib.patches import Patch
        A_ = [r["auw_kg"] for r in rows]
        ax[0].axhspan(0, 1.3, color=PALETTE[7], alpha=0.10, lw=0); ax[0].axhspan(1.3, 1.76, color=PALETTE[3], alpha=0.14, lw=0)
        ax[0].plot(A_, [r["TW_max"] for r in rows], "o-", color=PALETTE[0], label="T/W (all lift motors WOT)")
        pa = next((r for r in rows if r.get("is_plan_auw")), None)
        if pa:
            ax[0].plot([pa["auw_kg"]], [pa["TW_max"]], "D", color=INK2, ms=8, label=f"plan AUW {pa['auw_kg']:.2f} kg")
        ax[0].set_ylim(0.8, max(2.2, max(r["TW_max"] for r in rows) * 1.1))
        h, l = ax[0].get_legend_handles_labels()
        h += [Patch(color=PALETTE[7], alpha=0.25), Patch(color=PALETTE[3], alpha=0.3)]
        l += ["FAIL < 1.3", "WARN 1.3-1.76 (1.6 + 10 %)"]
        ax[0].legend(h, l, loc="upper right", fontsize=7.5)
        ax[0].set(xlabel="AUW [kg]", ylabel="T/W", title="VTOL thrust margin (team criterion)")
        ax[1].plot(A_, [r["duty"] for r in rows], "o-", color=PALETTE[0], label="ESC duty (FAIL > 0.80)")
        ax[1].plot(A_, [r["thr_lin"] for r in rows], "s--", color=PALETTE[1], label="thrust-linearised ThO (WARN > 0.60)")
        ax[1].axhline(0.80, color=PALETTE[7], ls=":"); ax[1].axhline(0.60, color=PALETTE[3], ls=":")
        if pa:
            ax[1].axvline(pa["auw_kg"], color=INK2, lw=0.8, ls=":"); ax[1].text(pa["auw_kg"], 0.05, " plan AUW", color=INK2, fontsize=8)
        ax[1].set(xlabel="AUW [kg]", ylabel="hover throttle [-]", title="Hover throttle: ESC duty vs ArduPilot ThO domain", ylim=(0, 1.05))
        ax[1].legend(loc="upper left", fontsize=7.5)
        save(fig, ctx.dir / "vtol_hover.png")
        ctx.add_file(ctx.dir / "vtol_hover.png", "VTOL T/W and hover duty vs AUW", "figure")
    summ = dict(static_TW=round(tw, 2), cruise_duty=round(cru["duty"], 3), cruise_power_W_incl_avionics=round(cru["P_elec_per_motor"] + P_av, 1),
                cruise_current_A=round(cru["I_bus_total"], 2),
                endurance_min_UNCALIBRATED=f"{t_end_p / 60:.0f}-{t_end / 60:.0f} (pessimistic drag - nominal)",
                range_km_UNCALIBRATED=f"{t_end_p * Vc / 1000:.0f}-{t_end * Vc / 1000:.0f} (pessimistic drag - nominal)",
                v_max_level=vmax, climb_rate=round(roc, 2), drag_source=drag_src,
                mission_energy_Wh=[round(m["energy_Wh"], 1) for m in missions])
    if lifts and res["vtol"].get("hover_table"):
        mine = next((r for r in res["vtol"]["hover_table"] if r.get("is_plan_auw")), {})
        summ.update(vtol_TW=round(mine.get("TW_max", 0), 2), hover_power_W=round(mine.get("P_total_W", 0)),
                    hover_minutes=round(mine.get("hover_minutes", 0), 1))
    return dict(summary=summ, data=P_clean(res))


def P_clean(o):
    from ..core import clean
    return clean(o)
