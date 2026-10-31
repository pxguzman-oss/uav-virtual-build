"""Propulsion & electrical sizing stage (prototype, eCalc-like).

    python propulsion.py [buildplan.yaml] [--out out/]

Physics (all steady-state):
  prop     T = ct_scale*Ct(J) rho n^2 D^4 ,  P = cp_scale*Cp(J) rho n^3 D^5,  J = V/(nD)
           Ct, Cp tables from APC PER3 performance files (apcprop.com/files/PER3_<prop>.dat)
  motor    3-constant model: E = rpm/Kv, Kt = 60/(2 pi Kv), Q = Kt (I - I0(rpm)),
           I0(rpm) = i0 * sqrt(rpm / (Kv * i0_ref_V))  (iron+bearing loss grows with speed)
  ESC      averaging PWM: V_motor = d * V_bus, I_bus = d * I_motor, conduction loss I^2 R_on
  battery  V_bus = OCV(SOC)*S - I_bus_total * (S r_cell / P + R_wiring); generic NMC OCV curve
  airframe cruise drag from out/aero.json (if the aero stage ran) else a parabolic polar fallback
Outputs: out/propulsion.json (operating points, margins, ArduPilot parameter hints).
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import brentq

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from vb import plan as bp  # noqa: E402

RHO0 = 1.225
G = 9.80665

# ---------------------------------------------------------------------------
# Embedded propeller data (APC PER3 files, simulated by APC; v2022-09-15)
#   10x6E RPM=9000 block: J, Ct, Cp   (Ct0 varies only 0.1122..0.1148 from 5k..12k rpm,
#   so one block is used for all rpm)
# ---------------------------------------------------------------------------
APC_10x6E = np.array([
    [0.0000, 0.1134, 0.0434], [0.0258, 0.1118, 0.0439], [0.0516, 0.1101, 0.0445],
    [0.0774, 0.1082, 0.0450], [0.1032, 0.1062, 0.0455], [0.1290, 0.1039, 0.0458],
    [0.1548, 0.1015, 0.0461], [0.1806, 0.0989, 0.0463], [0.2064, 0.0962, 0.0464],
    [0.2322, 0.0932, 0.0463], [0.2580, 0.0900, 0.0461], [0.2837, 0.0866, 0.0457],
    [0.3095, 0.0830, 0.0451], [0.3353, 0.0791, 0.0444], [0.3611, 0.0751, 0.0435],
    [0.3869, 0.0708, 0.0424], [0.4127, 0.0664, 0.0411], [0.4385, 0.0618, 0.0396],
    [0.4643, 0.0570, 0.0379], [0.4901, 0.0521, 0.0360], [0.5159, 0.0470, 0.0340],
    [0.5417, 0.0419, 0.0317], [0.5675, 0.0367, 0.0293], [0.5933, 0.0315, 0.0268],
    [0.6191, 0.0262, 0.0241], [0.6449, 0.0209, 0.0212], [0.6707, 0.0155, 0.0183],
    [0.6965, 0.0103, 0.0152], [0.7223, 0.0051, 0.0120], [0.7481, 0.0000, 0.0088]])
# 7x5E: only the static row (RPM=10000, V=0) was extracted: Ct 0.1403, Cp 0.0635.
APC_7x5E = np.array([[0.0, 0.1403, 0.0635], [0.05, 0.1403 * 0.985, 0.0635]])  # hover only
PROP_DB = {"apc_10x6e": APC_10x6E, "apc_7x5e": APC_7x5E}

# generic NMC/NCA 21700 open-circuit voltage vs SOC (replace with measured curve)
OCV_SOC = np.array([0.0, 0.05, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00])
OCV_V = np.array([3.00, 3.30, 3.42, 3.53, 3.60, 3.66, 3.72, 3.80, 3.88, 3.97, 4.07, 4.18])

AWG_OHM_PER_M = {10: 0.00328, 12: 0.00521, 14: 0.00829, 16: 0.0132, 18: 0.0210, 20: 0.0333}
AWG_CHASSIS_A = {10: 55, 12: 41, 14: 32, 16: 22, 18: 16, 20: 11}   # chassis-wiring rule of thumb


class Prop:
    """CT/CP(J[, rpm]) from data/props/<key>.json (APC PER3 / UIUC) or the embedded fallback tables."""
    DATA = Path(__file__).resolve().parents[3] / "data" / "props"

    def __init__(self, cfg, base_dir=None):
        self.D = cfg["diameter"]
        ref = cfg.get("data", "")
        key = ref.replace("embedded:", "")
        f = self.DATA / f"{key}.json" if ref.startswith("embedded:") else Path(base_dir or ".") / ref
        if f.exists():
            t = json.loads(f.read_text())
            self.J = np.asarray(t["J"], float)
            CT, CP = np.asarray(t["CT"], float), np.asarray(t["CP"], float)
            self.rpm = np.asarray(t["rpm"], float) if t.get("rpm") and CT.ndim == 2 else None
            self.CT = CT if CT.ndim == 2 else CT[None, :]
            self.CP = CP if CP.ndim == 2 else CP[None, :]
            self.source = t.get("source", str(f))
        else:
            tab = PROP_DB[key]
            self.J, self.CT, self.CP, self.rpm = tab[:, 0], tab[None, :, 1], tab[None, :, 2], None
            self.source = "embedded table"
        self.ks, self.kp = cfg.get("ct_scale", 1.0), cfg.get("cp_scale", 1.0)
        self.name = cfg.get("model", key)

    def coeffs(self, J, rpm=None):
        J = min(max(J, 0.0), self.J[-1])
        ct = [np.interp(J, self.J, row) for row in self.CT]
        cp = [np.interp(J, self.J, row) for row in self.CP]
        if self.rpm is not None and rpm is not None and len(ct) > 1:
            r = min(max(rpm, self.rpm[0]), self.rpm[-1])
            ct, cp = np.interp(r, self.rpm, ct), np.interp(r, self.rpm, cp)
        else:
            ct, cp = ct[-1], cp[-1]
        return self.ks * ct, self.kp * cp

    def thrust_torque(self, rpm, V, rho):
        n = max(rpm, 1e-3) / 60.0
        ct, cp = self.coeffs(V / (n * self.D), rpm)
        T = ct * rho * n * n * self.D ** 4
        P = cp * rho * n ** 3 * self.D ** 5
        return T, P / (2 * math.pi * n), P


class Motor:
    def __init__(self, cfg):
        self.kv = bp.val(cfg["kv"]); self.rm = bp.val(cfg["rm"]); self.i0 = bp.val(cfg["i0"])
        self.i0_ref_V = cfg.get("i0_ref_V", 10.0)
        self.kt = 60.0 / (2 * math.pi * self.kv)
        self.i_max = cfg.get("i_max"); self.loss_limit = cfg.get("loss_limit_W", 30.0)
        self.name = cfg.get("model", "motor")

    def i0_at(self, rpm):
        return self.i0 * math.sqrt(max(rpm, 1.0) / (self.kv * self.i0_ref_V))


class Pack:
    def __init__(self, energy):
        c = energy["cell"]
        self.S, self.P = energy["series"], energy["parallel"]
        self.cap = c["capacity_Ah"] * self.P
        self.r_cell = bp.val(c["r_dc"])
        self.R = self.S * self.r_cell / self.P + energy.get("pack_wiring_r", 0.0)
        self.i_cell_max = c["i_cont"]
        self.usable = energy.get("usable_fraction", 0.85)
        self.reserve = energy.get("reserve_fraction", 0.2)
        self.mass = c["mass"] * self.S * self.P
        self.E_nom = self.cap * c.get("v_nom", 3.6) * self.S

    def ocv(self, soc):
        return self.S * float(np.interp(soc, OCV_SOC, OCV_V))


def operating_point(unit, motor, prop, pack, duty, V, rho, soc=1.0, n_units=None, other_bus_A=0.0):
    """Solve the motor/prop/ESC/battery loop at a given ESC duty for ONE motor of `unit`.
    All `n_units` identical motors are assumed to run at the same duty."""
    n_units = n_units or unit["count"]
    r_on = unit["esc"].get("r_on", 0.004)
    awg = unit.get("wiring", {}).get("awg")
    r_wire = AWG_OHM_PER_M.get(awg, 0.0) * unit.get("wiring", {}).get("length", 0.0) * 2  # +/- leads
    Voc = pack.ocv(soc) - other_bus_A * pack.R
    Rb = pack.R

    def current(rpm):
        E = rpm / motor.kv
        # d*(Voc - n*d*I*Rb) = E + I*(Rm + Ron + Rwire*d^2)
        return max((duty * Voc - E) / (motor.rm + r_on + n_units * duty ** 2 * Rb + duty ** 2 * r_wire), 0.0)

    def resid(rpm):
        I = current(rpm)
        Qm = motor.kt * (I - motor.i0_at(rpm))
        return Qm - prop.thrust_torque(rpm, V, rho)[1]

    rpm_hi = motor.kv * Voc * duty
    if resid(50.0) < 0:
        return None
    rpm = brentq(resid, 50.0, rpm_hi * 0.9999)
    I = current(rpm)
    T, Q, Pshaft = prop.thrust_torque(rpm, V, rho)
    I_bus = duty * I
    V_bus = Voc - (n_units * I_bus) * Rb
    P_in = V_bus * I_bus
    E = rpm / motor.kv
    p_cu = I * I * motor.rm
    p_fe = motor.i0_at(rpm) * E
    return dict(duty=duty, rpm=rpm, V_air=V, thrust_N=T, thrust_g=T / G * 1000, I_motor=I,
                I_bus_per_motor=I_bus, I_bus_total=n_units * I_bus + other_bus_A, V_bus=V_bus,
                V_cell=V_bus / pack.S, P_elec_per_motor=P_in, P_shaft=Pshaft,
                eta_motor=Pshaft / max(I * duty * Voc, 1e-9) if I > 0 else 0,
                eta_total=(T * V / P_in if V > 0 else None), g_per_W=T / G * 1000 / max(P_in, 1e-9),
                motor_loss_W=p_cu + p_fe, I_cell=(n_units * I_bus + other_bus_A) / pack.P,
                J=V / (rpm / 60 * prop.D))


def duty_for_thrust(unit, motor, prop, pack, T_req, V, rho, soc=1.0, **kw):
    f = lambda d: (operating_point(unit, motor, prop, pack, d, V, rho, soc, **kw) or {"thrust_N": 0})["thrust_N"] - T_req
    if f(1.0) < 0:
        return None
    return operating_point(unit, motor, prop, pack, brentq(f, 0.05, 1.0, xtol=1e-4), V, rho, soc, **kw)


def cruise_drag(plan, mass, V, rho, aero=None):
    wing = plan["airframe"]["lifting_surfaces"][0]
    S, b = wing["area"], wing["span"]
    AR = b * b / S
    W = mass * G
    q = 0.5 * rho * V * V
    CL = W / (q * S)
    if aero and "cd0" in aero:
        CD0, e = aero["cd0"], aero["e"]
        src = "out/aero.json"
    else:
        CD0, e = 0.032, 0.78  # fallback: printed skin, pusher, V-tail, exposed GPS/pitot
        src = "fallback polar CD0=0.032, e=0.78 (replace by aero stage)"
    CD = CD0 + CL * CL / (math.pi * e * AR)
    return q * S * CD, CL, CD, CL / CD, src


def endurance_cruise(unit, motor, prop, pack, T_req, V, rho, dt=10.0):
    soc, t, E_used = 1.0, 0.0, 0.0
    soc_min = 1.0 - pack.usable
    while soc > soc_min:
        op = duty_for_thrust(unit, motor, prop, pack, T_req, V, rho, soc=soc)
        if op is None or op["V_cell"] < 3.0:
            break
        dq = op["I_bus_total"] * dt / 3600.0
        soc -= dq / pack.cap
        E_used += op["V_bus"] * op["I_bus_total"] * dt / 3600.0
        t += dt
    return t, E_used


def margin(capacity, demand):
    return capacity / demand - 1.0 if demand > 0 else float("inf")


def run(plan_path, out_dir):
    out_dir = Path(out_dir); out_dir.mkdir(parents=True, exist_ok=True)
    plan = bp.load(plan_path)
    mp = bp.mass_properties(plan)
    aero = None
    if (out_dir / "aero.json").exists():
        aero = json.loads((out_dir / "aero.json").read_text())
    pack = Pack(plan["energy"])
    rho = RHO0
    res = {"plan": plan["meta"]["name"], "auw_kg": mp["mass"], "pack": {
        "config": f"{pack.S}S{pack.P}P", "capacity_Ah": pack.cap, "R_pack_ohm": pack.R,
        "E_nom_Wh": pack.E_nom}, "cruise_unit": {}, "vtol": {}, "gates": []}

    unit = next(u for u in plan["propulsion"]["units"] if u["role"] == "cruise")
    motor, prop = Motor(unit["motor"]), Prop(unit["prop"])
    env = plan["envelope"]

    print(f"=== {plan['meta']['name']}  AUW {mp['mass']:.3f} kg  pack {pack.S}S{pack.P}P "
          f"{pack.cap:.1f} Ah {pack.E_nom:.0f} Wh, R_pack {pack.R*1000:.1f} mOhm ===")
    print(f"cruise unit: {motor.name} (Kv {motor.kv:.0f}, Rm {motor.rm*1000:.0f} mOhm, I0 {motor.i0} A) + {prop.name}")

    # --- static WOT at full / nominal / low battery ---------------------------
    static = {}
    for label, soc in (("full_4.18V", 1.0), ("nominal_3.72V", 0.5), ("low_3.42V", 0.10)):
        op = operating_point(unit, motor, prop, pack, 1.0, 0.0, rho, soc)
        static[label] = op
        print(f"  static WOT {label:14s}: {op['thrust_g']:6.0f} g  {op['rpm']:6.0f} rpm  "
              f"I_bat {op['I_bus_total']:5.1f} A  P {op['P_elec_per_motor']:5.0f} W  "
              f"V_cell {op['V_cell']:.2f}  motor loss {op['motor_loss_W']:.0f} W  {op['g_per_W']:.2f} g/W")
    res["cruise_unit"]["static_wot"] = static
    wot = static["full_4.18V"]
    tw = wot["thrust_N"] / (mp["mass"] * G)
    print(f"  static T/W = {tw:.2f}  (hand launch wants >= 0.5-0.6)")

    # --- cruise ---------------------------------------------------------------
    Vc = env["v_cruise"]
    D, CL, CD, LD, src = cruise_drag(plan, mp["mass"], Vc, rho, aero)
    T_req = D * 1.08  # pusher thrust deduction / installation loss 8 %
    cru = duty_for_thrust(unit, motor, prop, pack, T_req, Vc, rho, soc=0.5)
    t_end, E_used = endurance_cruise(unit, motor, prop, pack, T_req, Vc, rho)
    print(f"  cruise {Vc} m/s: CL {CL:.2f}, L/D {LD:.1f} [{src}] -> T_req {T_req:.2f} N")
    print(f"    duty {cru['duty']*100:.0f} %, {cru['rpm']:.0f} rpm, I_bat {cru['I_bus_total']:.2f} A, "
          f"P_elec {cru['P_elec_per_motor']:.0f} W, eta_prop*motor {cru['eta_total']:.2f}, motor loss {cru['motor_loss_W']:.1f} W")
    print(f"    endurance to {pack.usable*100:.0f} % DoD: {t_end/60:.0f} min, range {t_end*Vc/1000:.0f} km (still air), "
          f"energy used {E_used:.0f} Wh; with {pack.reserve*100:.0f} % reserve: {t_end*(1-pack.reserve)/60:.0f} min")
    res["cruise_unit"]["cruise"] = dict(cru, T_req_N=T_req, CL=CL, CD=CD, LD=LD, drag_source=src,
                                        endurance_min=t_end / 60, range_km=t_end * Vc / 1000,
                                        endurance_with_reserve_min=t_end * (1 - pack.reserve) / 60)

    # --- climb at WOT, V = 1.0 * Vc ---------------------------------------------
    climb = operating_point(unit, motor, prop, pack, 1.0, Vc, rho, 0.5)
    roc = (climb["thrust_N"] / 1.08 - D) * Vc / (mp["mass"] * G)
    print(f"    WOT at {Vc} m/s: T {climb['thrust_N']:.1f} N -> climb rate {roc:.1f} m/s")
    res["cruise_unit"]["climb_rate_ms"] = roc
    # --- level max speed ------------------------------------------------------
    vmax = None
    for V in np.arange(Vc, 40, 0.25):
        op = operating_point(unit, motor, prop, pack, 1.0, V, rho, 0.5)
        Dv = cruise_drag(plan, mp["mass"], V, rho, aero)[0] * 1.08
        if op is None or op["thrust_N"] < Dv:
            vmax = V; break
    print(f"    max level speed ~ {vmax:.1f} m/s (VNE {env['v_ne']} m/s)")
    res["cruise_unit"]["v_max_level"] = vmax

    # --- electrical margins (worst = static WOT on a full pack) ---------------
    Imax = wot["I_motor"]; Ibat = wot["I_bus_total"]
    wiring = unit.get("wiring", {})
    # lumped thermal model of the motor: C = m*cp (cp ~ 0.45 J/gK copper/steel mix),
    # R_th ~ 1.5 K/W for a ~75 g outrunner in pusher airflow (estimate - calibrate with an
    # IR thermometer after the first WOT climb). tau = R_th*C.
    m_g = (unit["motor"].get("mass", 0.077)) * 1000
    C_th, R_th = 0.45 * m_g, unit["motor"].get("r_th_KW", 1.5)
    tau = C_th * R_th
    T_amb = max(e.get("temperature_C", 15) for e in plan.get("environment_cases", [{}]))
    T_lim = unit["motor"].get("t_limit_C", 100.0)   # winding/magnet comfort limit
    dT_allow = T_lim - T_amb
    def t_to_limit(P):
        ss = P * R_th
        return float("inf") if ss <= dT_allow else -tau * math.log(1 - dT_allow / ss)
    t_wot = t_to_limit(wot["motor_loss_W"])
    T_cruise_ss = T_amb + cru["motor_loss_W"] * R_th
    print(f"  motor thermal (C={C_th:.0f} J/K, R_th={R_th} K/W, tau={tau:.0f} s, ambient {T_amb} C): "
          f"cruise steady {T_cruise_ss:.0f} C; WOT reaches {T_lim:.0f} C after {t_wot:.0f} s")
    res["cruise_unit"]["thermal"] = dict(C_JK=C_th, R_th_KW=R_th, tau_s=tau, T_amb_C=T_amb,
                                         T_cruise_ss_C=T_cruise_ss, wot_seconds_to_limit=t_wot)

    Imax = wot["I_motor"]; Ibat = wot["I_bus_total"]
    wiring = unit.get("wiring", {})
    # (name, capacity, demand, required margin, severity if failed)
    checks = [
        ("ESC continuous vs WOT motor current", unit["esc"]["i_cont"], Imax, 0.20, "FAIL"),
        ("Connector vs WOT battery current", wiring.get("connector_i_cont", 60), Ibat, 0.30, "FAIL"),
        ("Wire AWG%s chassis rating vs WOT (short duty)" % wiring.get("awg"), AWG_CHASSIS_A.get(wiring.get("awg"), 0), Ibat, 0.0, "WARN"),
        ("Cell continuous rating vs WOT per-cell current", pack.i_cell_max, wot["I_cell"], 1.0, "FAIL"),
        ("Motor 60 s max current vs WOT", motor.i_max or 1e9, Imax, 0.15, "FAIL"),
        ("Motor loss limit vs CRUISE loss (continuous)", motor.loss_limit, cru["motor_loss_W"], 0.5, "FAIL"),
        ("WOT time-to-%.0fC vs 60 s launch climb" % T_lim, t_wot, 60.0, 0.0, "WARN"),
        ("Cruise duty headroom (duty <= 0.65)", 0.65, cru["duty"], 0.0, "FAIL"),
        ("Endurance (with reserve) vs mission 45 min", t_end * (1 - pack.reserve) / 60, 45.0, 0.0, "FAIL"),
    ]
    print("  electrical / thermal margins (margin = capacity/demand - 1, required >= gate):")
    for name, cap, dem, req, sev in checks:
        m = margin(cap, dem)
        st = "PASS" if m >= req else sev
        res["gates"].append(dict(check=name, capacity=cap, demand=dem, margin=m, required=req, status=st))
        print(f"    [{st}] {name:52s} cap {cap:7.1f}  demand {dem:7.2f}  margin {m*100:+5.0f} % (req {req*100:+.0f} %)")

    res["ardupilot_hints"] = {
        "TRIM_THROTTLE": round(cru["duty"] * 100), "BATT_CAPACITY": round(pack.cap * 1000),
        "BATT_ARM_VOLT": round(pack.S * 3.5, 1), "BATT_LOW_VOLT": round(pack.S * 3.1, 1),
        "BATT_CRT_VOLT": round(pack.S * 2.9, 1), "MOT_BAT_VOLT_MAX": round(pack.S * 4.2, 1),
        "MOT_BAT_VOLT_MIN": round(pack.S * 3.0, 1),
        "note": "Li-ion: voltage-based failsafes must be set lower than LiPo defaults; prefer BATT_LOW_MAH."}

    # --- VTOL variant -------------------------------------------------------------
    if "vtol" in plan.get("variants", {}):
        vplan = bp.load(plan_path, "vtol")
        lift = next(u for u in vplan["propulsion"]["units"] if u["role"] == "lift")
        lm, lp = Motor(lift["motor"]), Prop(lift["prop"])
        vmp = bp.mass_properties(vplan)
        print(f"\n=== VTOL variant: 4x {lm.name} + {lp.name}; built-up AUW from plan {vmp['mass']:.3f} kg ===")
        rows = []
        for S in (4, 6):
            energy = json.loads(json.dumps(vplan["energy"])); energy["series"] = S
            vpack = Pack(energy)
            wot_l = operating_point(lift, lm, lp, vpack, 1.0, 0.0, rho, 0.5)
            for auw in vplan.get("auw_cases_kg", [vmp["mass"]]):
                auw_eff = auw + (vpack.mass - pack.mass)   # 6S adds 2 cells per string
                Th = auw_eff * G / 4
                hov = duty_for_thrust(lift, lm, lp, vpack, Th, 0.0, rho, soc=0.5)
                twr = 4 * wot_l["thrust_N"] / (auw_eff * G)
                if hov is None:
                    print(f"  {S}S AUW {auw_eff:.2f} kg: CANNOT HOVER (T/W {twr:.2f})"); continue
                P_tot = 4 * hov["P_elec_per_motor"]
                t_hover = vpack.cap * vpack.usable * vpack.ocv(0.5) / P_tot * 60
                row = dict(series=S, auw_kg=auw_eff, thrust_per_motor_g=Th / G * 1000, duty=hov["duty"],
                           rpm=hov["rpm"], I_bat=hov["I_bus_total"], I_cell=hov["I_cell"],
                           I_motor=hov["I_motor"], P_total_W=P_tot, g_per_W=hov["g_per_W"],
                           TW_max=twr, motor_loss_W=hov["motor_loss_W"], wot_I_motor=wot_l["I_motor"],
                           wot_I_bat=wot_l["I_bus_total"], wot_thrust_g=wot_l["thrust_g"],
                           wot_rpm=wot_l["rpm"], hover_minutes=t_hover)
                rows.append(row)
                flag = "OK" if (twr >= 1.6 and hov["duty"] <= 0.7) else "MARGINAL" if twr >= 1.3 else "NO-GO"
                if wot_l["I_cell"] > vpack.i_cell_max * 0.9:
                    flag += " (WOT all-4 at cell limit: cap Q_M_THST_MAX / current limit)"
                row["Q_M_THST_HOVER_guess"] = round(1 / twr, 2); row["status"] = flag
                # adiabatic cell heating (closed fuselage): dT = I^2 R t / (m cp), cp ~ 1.0 J/gK (21700 NMC)
                q_cell = hov["I_cell"] ** 2 * vpack.r_cell
                row["cell_heat_W"] = q_cell
                row["cell_dT_120s_hover_K"] = q_cell * 120 / (70 * 1.0)
                row["cell_dT_full_hover_K"] = q_cell * t_hover * 60 / (70 * 1.0)
                print(f"  {S}S AUW {auw_eff:.2f} kg: hover {Th/G*1000:.0f} g/motor duty {hov['duty']*100:.0f} % "
                      f"{hov['rpm']:.0f} rpm | I_bat {hov['I_bus_total']:.0f} A ({hov['I_cell']:.0f} A/cell) "
                      f"P {P_tot:.0f} W {hov['g_per_W']:.1f} g/W | motor loss {hov['motor_loss_W']:.0f} W | "
                      f"T/W {twr:.2f} | WOT {wot_l['I_motor']:.0f} A/motor, {wot_l['I_bus_total']:.0f} A bus | "
                      f"~{t_hover:.1f} min hover -> {flag}; Q_M_THST_HOVER~{1/twr:.2f}")
                print(f"        cell heating {q_cell:.1f} W/cell: +{row['cell_dT_120s_hover_K']:.0f} K per 120 s hover, "
                      f"+{row['cell_dT_full_hover_K']:.0f} K if hovering to empty (adiabatic upper bound; P50B discharge limit 60 C)")
        res["vtol"]["hover_table"] = rows
        res["vtol"]["rpm_band"] = [min(r["rpm"] for r in rows), max(r["wot_rpm"] for r in rows)]

    (out_dir / "propulsion.json").write_text(json.dumps(res, indent=2, default=float))
    print(f"\nwrote {out_dir/'propulsion.json'}")
    return res


if __name__ == "__main__":
    argv = sys.argv[1:]
    args = [a for i, a in enumerate(argv) if not a.startswith("--") and (i == 0 or not argv[i - 1].startswith("--"))]
    plan = args[0] if args else str(Path(__file__).resolve().parent / "stork_phase0.buildplan.yaml")
    out = sys.argv[sys.argv.index("--out") + 1] if "--out" in sys.argv else str(Path(__file__).resolve().parent / "out")
    run(plan, out)
