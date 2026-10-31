"""Stage 8 - ArduPilot SITL test matrix (ArduPlane --model JSON <-> in-process JSBSim bridge with motor/prop/
battery/wind/hand-launch models), flown with exactly the derived.param of the flightmodel stage.

Matrix (fixed wing)
  quick    : smoke (launch, 4-waypoint box, RTL)
  standard : canal mission (L1 15) + L1 sweep subset + stall + hand launch (nominal, weak) + RC-loss failsafe
  full     : standard + full L1 sweep + cruise sweep + battery and GCS failsafes + mission in every environment case
Matrix (variants with lift units = QuadPlane)
  quick: VTOL takeoff/transition/box/VTOL land | standard: + same in std-day wind | full: + every environment case
Gates: launch, mission completion, energy (SITL-calibrated mission energy vs usable), stall/overspeed/load factor,
throttle saturation, path tracking, failsafe action, VTOL hover throttle and landing accuracy."""
import json
import math
import os
import time
import types
from pathlib import Path

import numpy as np

from .. import env, plan as P
from ..figures import style, save, PALETTE, INK2

LIB = Path(__file__).resolve().parents[1] / "lib" / "sim"
TURB = {"none": None, "light": 2, "moderate": 3, "severe": 5}


def clean_parm(src, dst):
    lines = []
    for line in Path(src).read_text().splitlines():
        line = line.split("#")[0].strip()
        if line:
            k, v = line.split()[:2]
            lines.append(f"{k} {v}")
    Path(dst).write_text("\n".join(lines) + "\n")


def free_instance(max_n=10):
    """First SITL instance whose MAVLink TCP port and JSON/control UDP ports are free (parallel runs/sub-agents)."""
    import socket
    for i in range(max_n):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as t:
                t.bind(("127.0.0.1", 5760 + 10 * i))
            for p in (9002 + 10 * i, 9003 + 10 * i, 9012 + 10 * i):
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as u:
                    u.bind(("127.0.0.1", p))
            return i
        except OSError:
            continue
    return 0


def env_spec(e, calm=False):
    """SITL environment of a plan environment case (review C2): mean wind from 270 deg, Dryden turbulence
    (severity from the case, W20 = gust speed), scripted 1-cosine horizontal gusts of (gust - mean) every 20 s
    lasting 3 s, air temperature and home altitude of the case. calm=True: no wind/turbulence/gusts but the same
    air (altitude, temperature), so calm and windy runs differ ONLY in the wind."""
    alt = float(e.get("altitude_msl", 0.0) or 0.0)
    T = e.get("temperature_C", 15.0)
    if calm:
        return dict(wind=None, turb=None, gusts=None, temp_c=T, home_alt=alt, seed=1,
                    env_label=f"calm air (no wind, turbulence or gusts); {e.get('name', 'ISA')} air: {alt:g} m MSL, {T:g} C")
    wm, wg = float(e.get("wind_mean", 0) or 0), float(e.get("wind_gust", 0) or 0)
    tb = TURB.get(e.get("turbulence", "none"))
    amp = max(wg - wm, 0.0)
    spec = dict(wind=f"{wm:g},270" if wm else None, turb=f"{tb},{wg if wg else 6:g}" if tb else None,
                gusts=f"{amp:g},20,3" if amp > 0 else None, temp_c=T, home_alt=alt, seed=7)
    spec["env_label"] = (f"{e.get('name')}: wind {wm:g} m/s from 270 deg, gusts to {wg:g} m/s (1-cos {amp:g} m/s every 20 s), "
                         f"turbulence {e.get('turbulence', 'none')} (Dryden severity {tb}, W20 {wg:g} m/s); {alt:g} m MSL, {T:g} C")
    return spec


def matrix(ctx, is_vtol, fm):
    """(kind, name, kwargs). Every run carries its environment (env_label is printed with each gate).
    quick    : calm air only (stated in the report)
    standard : calm baseline + the plan mission's own environment case (wind + gusts + turbulence) for the
               mission, the nominal and pessimistic-CLmax hand launch and the RC failsafe
    full     : standard + every environment case + full L1 sweep, cruise sweep, battery and GCS failsafes"""
    an = ctx.plan.get("analysis", {}).get("sitl", {}) or {}
    envs = ctx.plan.get("environment_cases") or [dict(name="ISA", altitude_msl=0, temperature_C=15)]
    mi_env = (ctx.plan.get("missions") or [{}])[0].get("environment")
    std = next((e for e in envs if e["name"] == mi_env), envs[0])
    calm = env_spec(std, calm=True)
    wind = env_spec(std)
    acc, dur = an.get("launch_accel", 25), an.get("launch_duration", 0.45)
    fid = ctx.fidelity
    others = [e for e in envs if e is not std]
    T = []
    if is_vtol:
        box = [(200, 0), (200, 150), (0, 150), (-60, 0)] if fid == "quick" else [(400, 0), (400, 300), (0, 300), (-100, 0)]
        T.append(("vtol", "vtol", dict(vtol_box=box, **calm)))
        if fid in ("standard", "full"):
            T.append(("vtol", f"vtol_wind_{std['name']}", dict(vtol_box=box, **wind)))
        if fid == "full":
            for e in others:
                T.append(("vtol", f"vtol_wind_{e['name']}", dict(vtol_box=box, **env_spec(e))))
        return T
    if fid == "quick":
        return [("smoke", "smoke", dict(calm))]
    base_l1 = an.get("l1_base", 15)
    T.append(("mission", f"mission_l1_{base_l1:g}", dict(l1=base_l1, **calm)))
    T.append(("mission", f"mission_wind_{std['name']}", dict(l1=base_l1, **wind)))
    # waypoint-spacing check of the roll weave (review M10): same mission, sparser waypoints
    T.append(("mission", f"mission_l1_{base_l1:g}_wp{an.get('wp_spacing_alt', 75):g}", dict(l1=base_l1, spacing=an.get("wp_spacing_alt", 75.0), **calm)))
    periods = an.get("l1_periods_full" if fid == "full" else "l1_periods_standard", [10, 15])
    for p in periods:
        if p != base_l1:
            T.append(("mission", f"mission_l1_{p:g}", dict(l1=p, **calm)))
    T.append(("stall", "stall", dict(calm)))
    T.append(("launch", "launch_nominal", dict(accel=acc, dur=dur, **calm)))
    T.append(("launch", f"launch_nominal_wind_{std['name']}", dict(accel=acc, dur=dur, **wind)))
    if fm.get("model_name_pess"):
        T.append(("launch", f"launch_pessCLmax_wind_{std['name']}",
                  dict(accel=acc, dur=dur, model=fm["model_name_pess"], gen_dir=str(ctx.updir("flightmodel") / fm["gen_dir_pess"]), **wind)))
    T.append(("launch", "launch_weak", dict(accel=an.get("weak_accel", 15), dur=an.get("weak_duration", 0.40), **calm)))
    T.append(("fs_rc", "fs_rc", dict(calm)))
    T.append(("fs_rc", f"fs_rc_wind_{std['name']}", dict(wind)))
    if fid == "full":
        T.append(("cruise", "cruise_sweep", dict(calm)))
        T.append(("fs_batt", "fs_batt", dict(calm)))
        T.append(("fs_gcs", "fs_gcs", dict(calm)))
        for e in others:
            T.append(("mission", f"mission_wind_{e['name']}", dict(l1=base_l1, **env_spec(e))))
    return T


def run(ctx):
    import sys
    plan = ctx.plan
    fm = ctx.data("flightmodel")
    prop = ctx.data("propulsion", required=False)
    ap = env.arduplane()
    if not ap:
        raise RuntimeError("ArduPlane SITL binary not found (setup/install.sh builds it; or set VB_ARDUPILOT)")
    os.environ["VB_ARDUPLANE"] = ap
    sys.path.insert(0, str(LIB))
    import run_tests as RT
    import analyze as AN
    fdir = ctx.updir("flightmodel")
    gen = fdir / fm["gen_dir"]
    model = fm["model_name"]
    sp = ctx.dir / "sitl_params.parm"
    clean_parm(fdir / "derived.param", sp)
    plumb = Path(__file__).resolve().parents[2] / "data" / "ardupilot" / "sitl_plumbing.parm"
    an = plan.get("analysis", {}).get("sitl", {}) or {}
    res_dir = ctx.dir / "runs"
    res_dir.mkdir(exist_ok=True)
    inst = free_instance()
    ctx.note(f"SITL instance -I {inst} (MAVLink tcp {5760 + 10 * inst}, JSON udp {9002 + 10 * inst})")
    RT.CFG.update(results=str(res_dir), gen_dir=str(gen), model=model, parm_files=[str(sp), str(plumb)], instance=inst,
                  survey_alt=an.get("survey_alt", 60), takeoff_alt=an.get("takeoff_alt", 40),
                  cruise_v=plan["envelope"]["v_cruise"])
    params = fm["params"]
    Vs = params.get("AIRSPEED_STALL", 9)
    S = plan["energy"]["series"]
    is_vtol = fm.get("is_vtol")
    tests = matrix(ctx, is_vtol, fm)
    planned = [(k, n) for k, n, _ in tests]
    sel = os.environ.get("VB_SITL_TESTS")          # sharding / re-running a subset: VB_SITL_TESTS=fs_batt,stall
    if sel:
        want = {x.strip() for x in sel.split(",") if x.strip()}
        tests = [t for t in tests if t[1] in want or t[0] in want]
    ctx.note(f"SITL matrix ({ctx.fidelity}): " + ", ".join(n for _, n, _ in tests))
    results = {}
    for kind, name, kw in tests:
        a = types.SimpleNamespace(name=name, l1=None, wind=None, turb=None, gusts=None, temp_c=None, home_alt=None, seed=1,
                                  env_label=None, model=None, gen_dir=None, spacing=an.get("wp_spacing", 25.0),
                                  periods="", speeds=an.get("cruise_speeds", "10,11,12,13,14,15,16,18,20"), speeds_min=params.get("AIRSPEED_STALL", 9),
                                  settle=25, hold=25, accel=25, dur=0.45, soc=0.22, compensate=False, backend="jsbsim", rate=400,
                                  batt_low_volt=round(S * 3.49, 2),
                                  batt_low_mah=int(params.get("BATT_CAPACITY", 5000)) - 150)
        if kind == "fs_batt":
            a.soc = 0.6
        for k, v in kw.items():
            setattr(a, k, v)
        t0 = time.time()
        print(f"\n##### SITL {name} ({kind}) #####", flush=True)
        try:
            fn = dict(smoke=RT.t_smoke, mission=RT.t_mission, stall=RT.t_stall, launch=RT.t_launch, vtol=RT.t_vtol,
                      cruise=RT.t_cruise).get(kind)
            if kind.startswith("fs_"):
                r = RT.t_failsafe(a, kind[3:])
            else:
                r = fn(a)
        except Exception as e:
            r = {"run_error": f"{type(e).__name__}: {e}"}
        # meta needs the model path for analyze; re-analyse with it
        rd = res_dir / name
        try:
            meta = json.loads((rd / "meta.json").read_text())
            meta["model_json"] = str(Path(a.gen_dir or gen) / f"{a.model or model}.json")
            (rd / "meta.json").write_text(json.dumps(meta, indent=1))
            r = AN.analyze_run(str(rd))
        except Exception as e:
            r = dict(r or {}, analyze_error=str(e))
        r["wall_s_total"] = round(time.time() - t0, 1)
        r["environment"] = a.env_label or "calm (no wind, no turbulence, no gusts)"
        r["model_flown"] = a.model or model
        results[name] = r
        (ctx.dir / "sitl_results.json").write_text(json.dumps(results, indent=1, default=str))
        for f in sorted(rd.glob("*.png")) if rd.exists() else []:
            ctx.add_file(f, f"SITL {name}: {f.stem}", "figure")

    # ---- mandatory evidence: every planned run flown, every required metric logged (review M6) ----------
    REQ = {"mission": ["xt_truth_rms_m", "course_Wh_per_km", "min_airspeed_inflight", "max_cas", "max_nz", "roll_weave_wp_amp_deg"],
           "launch": ["min_cas_after_release", "height_at_10s_m", "crashed"], "stall": ["stalls", "V_stall_1g_mean"],
           "fs_rc": ["first_mode_change_after_trigger"], "fs_gcs": ["first_mode_change_after_trigger"], "fs_batt": ["trigger"],
           "vtol": ["landing_pos_error_m", "hover_power_W", "ap_QTUN_ThO_median", "hover_throttle_lift", "vtol_phase_energy_Wh"], "smoke": ["mode_changes", "max_cas"],
           "cruise": ["table"]}
    for kind, name in planned:
        if name not in results:
            ctx.missing(f"{name}: planned {ctx.fidelity}-fidelity SITL run not flown",
                        note=f"filtered out by VB_SITL_TESTS={sel}" if sel else "no result")
            continue
        r = results[name]
        for k in REQ.get(kind, []):
            if r.get(k) in (None, [], {}):
                ctx.missing(f"{name}: required metric '{k}' not produced", note=str(r.get("run_error") or r.get("analyze_error") or "")[:120])

    # ---- gates ---------------------------------------------------------------------------------------
    env_ = plan["envelope"]
    nlim = env_["n_limit_pos"]
    xt_max = (plan.get("gates") or {}).get("sitl", {}).get("xt_rms_max_m", 10.0)
    Vs_nom = fm.get("V_stall") or Vs
    Vs_pess = fm.get("V_stall_pess") or Vs
    for name, r in results.items():
        envlab = r.get("environment") or "?"
        short = "calm" if envlab.startswith("calm") else "wind+gust+turb"

        def G(check, *ga, _n=name, _s=short, _e=envlab, **gk):
            gk["note"] = (gk.get("note", "") + f" | env: {_e}").strip(" |")
            return ctx.gate(f"{_n} [{_s}]: {check}", *ga, **gk)
        err = r.get("run_error") or r.get("analyze_error")
        G(f"run completed without scripted-step errors", not err, True, "bool",
                 "WARN" if name.startswith(("fs_batt", "launch_weak")) else "FAIL", note=str(err or "")[:150])
        if r.get("min_airspeed_inflight") is not None and not name.startswith(("stall", "launch_weak")) and "vtol" not in name:
            mca = r["min_airspeed_inflight"]
            vs_flown = Vs_pess if "pess" in name else Vs_nom
            G("min airspeed in flight >= pessimistic Vs (AIRSPEED_STALL) [m/s]", mca, Vs_pess, ">=",
              "FAIL" if mca < vs_flown else "WARN", "m/s",
              note=f"model flown: {'PESSIMISTIC' if 'pess' in name else 'nominal'} CLmax (Vs {vs_flown:.2f} m/s); below the flown "
                   f"model's Vs = FAIL, below Vs pessimistic {Vs_pess:.2f} = WARN", drivers=["/airframe/lifting_surfaces/wing/cl_max"])
        if r.get("stall_time_s") is not None and not name.startswith(("stall", "launch_weak")):
            G("unintended stall in wing-borne flight: time above the stall alpha (h > 5 m, lift motors off) [s]", r["stall_time_s"], 0.5, "<=", "FAIL", "s",
              note=f"alpha_stall {r.get('alpha_stall_model_deg')} deg of the model flown ({'PESSIMISTIC' if 'pess' in name else 'nominal'} "
                   "CLmax); a gust or TECS speed undershoot that stalls the aircraft = raise AIRSPEED_MIN/AIRSPEED_CRUISE or fly calmer air",
              drivers=["/airframe/lifting_surfaces/wing/cl_max", "/avionics/params/AIRSPEED_MIN"])
        if r.get("max_cas") is not None:
            G(f"max airspeed <= VNE [m/s]", r["max_cas"], env_["v_ne"], "<=", "FAIL", "m/s")
        if r.get("max_cas") is not None and "vtol" not in name and not name.startswith("stall"):
            G(f"max airspeed <= AIRSPEED_MAX + 2 m/s (climb/descent overspeed)", r["max_cas"],
                     params.get("AIRSPEED_MAX", 22) + 2, "<=", "WARN", "m/s")
        if r.get("max_nz") is not None and not name.startswith("stall"):
            G(f"max load factor <= n_limit", r["max_nz"], nlim, "<=", "FAIL")
        if r.get("max_throttle_saturation_s") is not None and name.startswith(("mission", "smoke")):
            G(f"longest full-throttle stretch [s] (launch climb excluded)", r["max_throttle_saturation_s"],
                     an.get("max_wot_s", 20.0), "<=", "WARN", "s")
        if name.startswith("mission"):
            G(f"course completed and RTL reached", "xt_truth_rms_m" in r, True, "bool", "FAIL")
            if "xt_truth_rms_m" in r:
                G(f"cross-track RMS to the canal centreline [m]", r["xt_truth_rms_m"], xt_max, "<=", "WARN", "m",
                         note=f"max {r.get('xt_truth_max_m')} m; worst bend " + str(max(r.get("xt_truth_by_bend", {}).items(),
                                                                                         key=lambda kv: kv[1]["rms"], default=("-", {}))[0]))
        if name.startswith("mission") and r.get("roll_weave_wp_amp_deg") is not None:
            f_w, f_wp = r.get("roll_weave_freq_Hz"), r.get("wp_passing_freq_Hz")
            G("roll weave at the waypoint-passing frequency (half peak-to-peak) [deg]", r["roll_weave_wp_amp_deg"], 1.5, "<=", "WARN", "deg",
              note=f"TEAM CRITERION (camera footage); f_wp = V/spacing = {f_wp} Hz ({r.get('wp_spacing_m')} m spacing); whole-course "
                   f"weave {r.get('roll_weave_amp_deg')} deg, dominant {f_w} Hz; cause: L1 re-targets at every waypoint of a dense "
                   "polyline with default (untuned) roll gains -> sparser waypoints on straights, longer NAVL1_PERIOD, roll AUTOTUNE",
              drivers=["/analysis/sitl/wp_spacing"])
        if name.startswith("smoke"):
            G("launch, box and RTL completed", any(m[1] == 11 for m in r.get("mode_changes", []) if isinstance(m, (list, tuple))), True,
                     "bool", "FAIL")
        if name.startswith("launch"):
            weak = "weak" in name
            ok = not r.get("crashed", True) and (r.get("height_at_10s_m") or 0) >= 15
            G(f"survives the throw and climbs (>=15 m after 10 s)", ok, True, "bool", "WARN" if weak else "FAIL",
                     note=f"release {r.get('release_tas')} m/s, motor +{r.get('motor_start_delay_s')} s, min CAS {r.get('min_cas_after_release')} m/s, "
                          f"h(10 s) {r.get('height_at_10s_m')} m")
            if not weak and r.get("min_cas_after_release") is not None:
                vs_ref = Vs_pess if "pess" in name else Vs_nom
                G(f"min airspeed after release / Vs of the model flown", r["min_cas_after_release"] / vs_ref, 1.1, ">=", "WARN",
                  note=f"Vs {vs_ref:.2f} m/s ({'pessimistic' if 'pess' in name else 'nominal'} CLmax model)",
                  drivers=["/airframe/lifting_surfaces/wing/cl_max", "/airframe/launch/launch_speed"])
        if name == "stall" and r.get("V_stall_1g_mean"):
            G("SITL Vs(1g) / Vs of the flown model (MODEL SELF-CONSISTENCY - reads back the table, NOT a stall validation)",
              r["V_stall_1g_mean"] / Vs_nom, None, "info", "INFO",
              note=f"SITL {r['V_stall_1g_mean']} m/s vs nominal {Vs_nom:.2f} m/s; the real CLmax/Vs is only known after the F1 stall test")
        if name == "stall" and r.get("stalls"):
            sts = r["stalls"]
            wd = max((x.get("wing_drop_deg") or 0) for x in sts)
            G("wing drop within 5 s of the break, with +-7.5 % left/right CLmax asymmetry [deg]", wd, 30.0, "<=", "WARN", "deg",
              note="TEAM CRITERION 30 deg; strip-theory post-stall roll damping and aileron loss in the model; real departure / "
                   "spin behaviour UNKNOWN -> F1 stall test at >= 100 m is mandatory", drivers=["/airframe/lifting_surfaces/wing"])
            recs = [x.get("recovery_s") for x in sts]
            G("recovers within 10 s of releasing the stick (alpha < break - 2 deg, |roll| < 30 deg, sink < 2 m/s)",
              all(x is not None and x <= 10 for x in recs), True, "bool", "FAIL", note=f"recovery times {recs} s")
            al = [x.get("alt_loss_release_to_recovery_m") for x in sts if x.get("alt_loss_release_to_recovery_m") is not None]
            alb = [x.get("alt_loss_break_to_recovery_m") for x in sts if x.get("alt_loss_break_to_recovery_m") is not None]
            if al:
                G("height lost from stick release to recovery [m]", max(al), 20.0, "<=", "WARN", "m",
                  note=f"TEAM CRITERION 20 m; break -> recovery incl. ~7 s of held full-back stick {max(alb) if alb else '-'} m; practise "
                       f"stalls above {max(50, 2 * (max(alb) if alb else 0)):.0f} m with STALL_PREVENTION=0 only")
            lk = max((x.get("locked_in_s") or 0) for x in sts)
            G("post-stall pitch: time alpha stays > break + 3 deg with the stick held [s] (no nose-down break = locked-in stall)", lk, 3.0,
              "<=", "WARN", "s", note=f"nose drop in 3 s after the break {[x.get('nose_drop_deg') for x in sts]} deg; post-stall Cm "
                                     "pitch-up of AeroBuildup removed (artefact) -> real behaviour UNKNOWN")
        if name.startswith("fs_") and name != "fs_batt":
            fc = r.get("first_mode_change_after_trigger") or {}
            G(f"failsafe switches to RTL", fc.get("mode") == 11, True, "bool", "FAIL",
                     note=f"mode {fc.get('mode')} after {fc.get('dt_s')} s")
        if name == "fs_batt":
            fc = r.get("first_mode_change_after_trigger") or {}
            trig = (r.get("trigger") or {}).get("t")
            before = [m for m in r.get("mode_changes", []) if trig is not None and m[0] <= trig]
            already = bool(before) and before[-1][1] == 11
            G("battery failsafe (BATT_LOW_MAH) -> RTL", fc.get("mode") == 11 or already, True, "bool", "FAIL",
                     note=f"trigger at {trig} s: {(r.get('trigger') or {}).get('text', '')}" + (" (already in RTL)" if already else ""))
        if name.startswith("vtol"):
            ok = r.get("landing_pos_error_m") is not None and not r.get("run_error")
            G(f"VTOL takeoff, transitions both ways, VTOL land, disarm", ok, True, "bool", "FAIL")
            if r.get("landing_pos_error_m") is not None:
                G(f"landing position error [m]", r["landing_pos_error_m"], 2.0, "<=", "WARN", "m")
            hp = (prop or {}).get("vtol", {}).get("hover_table") or []
            mine = next((x for x in hp if x.get("is_plan_auw")), None)
            tmap = (prop or {}).get("vtol", {}).get("throttle_mapping") or {}
            if mine and r.get("hover_power_W") and short == "calm":
                G(f"SITL hover power / propulsion prediction", r["hover_power_W"] / mine["P_total_W"], [0.85, 1.15], "range",
                  "WARN", note=f"{r['hover_power_W']:.0f} W vs {mine['P_total_W']:.0f} W (still air only)")
            # hover throttle, like with like (review M8): ESC duty vs ESC duty at the same bus voltage, ThO vs ThO
            d_sitl, v_sitl = r.get("hover_throttle_lift"), r.get("hover_V_bat")
            if mine and d_sitl and v_sitl and mine.get("V_bus") and short == "calm":
                d_pred = mine["duty"] * mine["V_bus"] / v_sitl
                G("hover ESC duty SITL / propulsion model at the same bus voltage", d_sitl / d_pred, [0.9, 1.1], "range", "WARN",
                  note=f"SITL {d_sitl:.3f} at {v_sitl:.1f} V vs model {mine['duty']:.3f} at {mine['V_bus']:.1f} V -> {d_pred:.3f}")
                if tmap and r.get("ap_QTUN_ThO_median") is not None:
                    a_ = min(max((d_pred - tmap["Q_M_SPIN_MIN"]) / (tmap["Q_M_SPIN_MAX"] - tmap["Q_M_SPIN_MIN"]), 0), 1)
                    t_pred = tmap["Q_M_THST_EXPO"] * a_ * a_ + (1 - tmap["Q_M_THST_EXPO"]) * a_
                    G("QTUN.ThO / model thrust-linearised hover throttle (same domain, same voltage)", r["ap_QTUN_ThO_median"] / t_pred,
                      [0.85, 1.15], "range", "WARN", note=f"ThO {r['ap_QTUN_ThO_median']:.3f} vs model {t_pred:.3f}")
            thr = r.get("ap_QTUN_ThO_median")
            if thr is not None:
                G(f"Q-mode hover throttle QTUN.ThO (thrust-linearised) <= 0.6", thr, 0.60, "<=", "WARN",
                  note="ArduPilot docs 'Setting Hover Throttle': usually between 0.2 and 0.6")
            if d_sitl is not None:
                G("hover ESC duty of the lift motors <= 0.80", d_sitl, 0.80, "<=", "FAIL", note="TEAM CRITERION: >= 20 % duty headroom")
            # VTOL phases energy vs the mission-model budget (review M7)
            mrows = ((prop or {}).get("missions") or [{}])[0].get("segments", [])
            budget = sum(x["energy_Wh"] for x in mrows if "VTOL" in str(x.get("kind")))
            if budget and r.get("vtol_phase_energy_Wh") is not None:
                G("SITL VTOL-phase energy (takeoff, transitions, landing) <= mission-model VTOL budget [Wh]", r["vtol_phase_energy_Wh"], budget,
                  "<=", "WARN", "Wh", note=f"{r.get('vtol_phase_time_s')} s with lift motors on; model budget {budget:.1f} Wh incl. "
                                          f"hover allowance")

    # ---- energy (review M1/M9): compare like with like. Both sides include the avionics load, fly the same air
    # (SITL home at the environment altitude/temperature) and the JSBSim control drag is quadratic. The SITL
    # calibrates the PROPULSIVE power: r = (SITL course power - avionics) / steady propulsive power at the same
    # mean airspeed (a power ratio, so the course geometry relative to the wind does not matter). Mission
    # energy = steady propulsive energy x r + avionics energy (avionics never multiplied).
    energy = None
    P_av = float(plan.get("avionics", {}).get("avionics_power_W", 4.0))
    pc = (prop or {}).get("cruise_unit", {}).get("power_curve", []) if prop else []
    mi = (prop or {}).get("missions", [None])[0] if prop and prop.get("missions") else None

    def p_steady_prop(V):
        return float(np.interp(V, [p["V"] for p in pc], [p.get("P_prop_W", p["P_W"] - P_av) for p in pc]))

    def calib(r):
        V = r.get("course_mean_cas")
        if V is None or r.get("course_mean_power_W") is None or not pc:
            return None
        Pp_sitl = r["course_mean_power_W"] - P_av
        Pp_pred = p_steady_prop(V)
        return dict(V=V, P_sitl_W=r["course_mean_power_W"], P_pred_W=Pp_pred + P_av, P_sitl_prop_W=Pp_sitl,
                    P_pred_prop_W=Pp_pred, ratio_prop=Pp_sitl / Pp_pred, Wh_per_km=r.get("course_Wh_per_km"),
                    environment=r.get("environment"))
    calm_name = next((n for n in results if n.startswith("mission_l1") and "course_mean_power_W" in results[n]), None)
    own_env = (mi or {}).get("environment")
    wind_name = f"mission_wind_{own_env}" if own_env and f"mission_wind_{own_env}" in results else None
    if not is_vtol and ctx.fidelity != "quick":
        if not (calm_name and mi and pc):
            ctx.missing("SITL energy calibration (calm mission run + propulsion mission)", note="no calm mission run with course power")
    if calm_name and mi and pc:
        c_calm = calib(results[calm_name])
        c_wind = calib(results[wind_name]) if wind_name else None
        Ep, Eav = mi.get("energy_prop_Wh", mi["energy_Wh"]), mi.get("energy_avionics_Wh", 0.0)
        E_calm = Ep * c_calm["ratio_prop"] + Eav
        E_wind = Ep * c_wind["ratio_prop"] + Eav if c_wind else None
        energy = dict(mission=mi["name"], environment=own_env, calm=c_calm, wind=c_wind, avionics_W=P_av,
                      mission_energy_steady_Wh=mi["energy_Wh"], mission_energy_steady_pess_drag_Wh=mi.get("energy_pess_drag_Wh"),
                      mission_energy_sitl_calm_Wh=E_calm, mission_energy_sitl_env_Wh=E_wind,
                      available_Wh=mi["available_after_reserve_Wh"],
                      # legacy keys (report / calibration table)
                      sitl_Wh_per_km=c_calm["Wh_per_km"], predicted_Wh_per_km=(c_calm["P_pred_W"]) / (3.6 * c_calm["V"]),
                      ratio=c_calm["P_sitl_W"] / c_calm["P_pred_W"])
        ctx.gate("[calm] SITL / steady-state propulsive power at the same airspeed (model consistency)", c_calm["ratio_prop"], [0.8, 1.3],
                 "range", "WARN",
                 note=f"{c_calm['P_sitl_prop_W']:.1f} W vs {c_calm['P_pred_prop_W']:.1f} W at {c_calm['V']:.1f} m/s CAS (+{P_av:g} W avionics on both "
                      f"sides); residual = turns, TECS/throttle activity, trim drag | env: {c_calm['environment']}")
        if c_wind:
            ctx.gate(f"[wind+gust+turb] SITL / steady propulsive power in the mission environment", c_wind["ratio_prop"], [0.8, 1.5], "range",
                     "WARN", note=f"{c_wind['P_sitl_prop_W']:.1f} W vs {c_wind['P_pred_prop_W']:.1f} W at {c_wind['V']:.1f} m/s | env: {c_wind['environment']}")
        Egate = E_wind if E_wind is not None else E_calm
        ctx.gate(f"mission '{mi['name']}' energy, SITL-calibrated in its own environment ({'wind+gust+turb' if E_wind is not None else 'CALM - env run missing'}) "
                 f"<= usable after reserve [Wh]", Egate, mi["available_after_reserve_Wh"], "<=", "FAIL", "Wh",
                 note=f"steady {mi['energy_Wh']:.1f} Wh (pessimistic drag {mi.get('energy_pess_drag_Wh', float('nan')):.1f}), SITL calm "
                      f"{E_calm:.1f}, SITL env {('%.1f' % E_wind) if E_wind is not None else '-'} Wh; UNCALIBRATED until F3",
                 drivers=["/analysis/aero (drag)", "/analysis/aero/cd0_uncertainty", "/propulsion/units/cruise"])
        for n, r in results.items():
            if n.startswith("mission_wind") and n != wind_name and "course_mean_power_W" in r:
                cw = calib(r)
                Ew = Ep * cw["ratio_prop"] + Eav
                ctx.gate(f"{n} [wind+gust+turb]: mission energy if flown in this environment case (SITL-scaled) [Wh]", Ew,
                         mi["available_after_reserve_Wh"], "<=", "WARN", "Wh",
                         note=f"other environment case -> operating limit | env: {r.get('environment')}")
        # figure: mission energy band
        plt = style()
        fig, ax = plt.subplots(figsize=(8, 2.9))
        bars = [("steady model, nominal drag", mi["energy_Wh"]), ("steady model, pessimistic drag", mi.get("energy_pess_drag_Wh")),
                ("SITL-calibrated, calm", E_calm), (f"SITL-calibrated, {own_env} wind+gust+turb", E_wind)]
        bars = [(k, v) for k, v in bars if v is not None]
        ax.barh(range(len(bars)), [v for _, v in bars], color=[PALETTE[0], PALETTE[3], PALETTE[1], PALETTE[2]][:len(bars)], height=0.55)
        for i, (k, v) in enumerate(bars):
            ax.text(v + 1, i, f"{v:.0f} Wh", va="center", color=INK2, fontsize=8.5)
        ax.set_yticks(range(len(bars))); ax.set_yticklabels([k for k, _ in bars])
        ax.axvline(mi["available_after_reserve_Wh"], color=PALETTE[7], ls="--", lw=1.4)
        ax.text(mi["available_after_reserve_Wh"], -0.45, "usable after reserve ", color=PALETTE[7], fontsize=8.5, ha="right", va="center")
        ax.set_xlim(0, max(max(v for _, v in bars), mi["available_after_reserve_Wh"]) * 1.12)
        ax.set_ylim(len(bars) - 0.5, -0.75)
        ax.set_xlabel("mission energy [Wh] (UNCALIBRATED until F3 cruise legs)")
        ax.yaxis.grid(False)
        ax.set_title(f"Mission energy band: {mi['name']}", loc="left", fontsize=10)
        save(fig, ctx.dir / "mission_energy_band.png")
        ctx.add_file(ctx.dir / "mission_energy_band.png", "Mission energy: steady (nominal/pessimistic drag) vs SITL-calibrated (calm / mission env)", "figure")
    # L1 sweep summary
    l1 = None
    l1runs = [str(res_dir / n) for n in results if n.startswith("mission_l1") and "_wp" not in n and "xt_truth_rms_m" in results[n]]
    # mission-design finding (review M10): roll weave vs waypoint spacing, same L1
    base_l1 = an.get("l1_base", 15)
    a_, b_ = results.get(f"mission_l1_{base_l1:g}"), next((results[n] for n in results if n.startswith(f"mission_l1_{base_l1:g}_wp")), None)
    if a_ and b_ and a_.get("roll_weave_wp_amp_deg") is not None and b_.get("roll_weave_wp_amp_deg") is not None:
        ctx.gate("mission design: waypoint spacing effect (calm, same NAVL1_PERIOD) - waypoint-rate weave sparse / dense",
                 b_["roll_weave_wp_amp_deg"] / max(a_["roll_weave_wp_amp_deg"], 1e-3), None, "info", "INFO",
                 note=f"{a_.get('wp_spacing_m'):g} m: waypoint-rate weave {a_['roll_weave_wp_amp_deg']} deg, whole-course {a_['roll_weave_amp_deg']} "
                      f"deg, XT RMS {a_.get('xt_truth_rms_m')} m; {b_.get('wp_spacing_m'):g} m: {b_['roll_weave_wp_amp_deg']} deg, whole-course "
                      f"{b_['roll_weave_amp_deg']} deg, XT RMS {b_.get('xt_truth_rms_m')} m. Both weaves track the waypoint rate: sparse points only "
                      "help on straights (bends become long chords) -> one waypoint per straight leg, ~25 m points only in bends, tune "
                      "WP_RADIUS / NAVL1_PERIOD and run roll AUTOTUNE")
    if len(l1runs) >= 2:
        try:
            l1 = AN.l1_summary(l1runs, str(ctx.dir / "l1_sweep.png"))
            ctx.add_file(ctx.dir / "l1_sweep.png", "Path tracking vs NAVL1_PERIOD by bend radius", "figure")
            best = min(l1, key=lambda r: r["xt_rms"])
            ctx.note(f"best NAVL1_PERIOD in sweep: {best['NAVL1_PERIOD']} (XT RMS {best['xt_rms']} m)")
        except Exception as e:
            ctx.note(f"L1 summary failed: {e}")
    walls = {n: r.get("wall_s_total") for n, r in results.items()}
    data = dict(results=results, energy=energy, l1_sweep=l1, matrix=[n for _, n, _ in tests], walls=walls,
                params_file="sitl_params.parm")
    summ = dict(runs=len(results), errors=sum(bool(r.get("run_error")) for r in results.values()),
                wall_min=round(sum(v or 0 for v in walls.values()) / 60, 1),
                propulsive_power_ratio_calm=energy and round(energy["calm"]["ratio_prop"], 3),
                mission_energy_Wh_sitl_env=energy and energy.get("mission_energy_sitl_env_Wh") and round(energy["mission_energy_sitl_env_Wh"], 1))
    return dict(summary=summ, data=data)
