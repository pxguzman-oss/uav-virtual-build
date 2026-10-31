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
    REQ = {"mission": ["xt_truth_rms_m", "course_Wh_per_km", "min_airspeed_inflight", "max_cas", "max_nz"],
           "launch": ["min_cas_after_release", "height_at_10s_m", "max_cas"], "stall": ["stalls", "V_stall_1g_mean"],
           "fs_rc": ["first_mode_change_after_trigger"], "fs_gcs": ["first_mode_change_after_trigger"], "fs_batt": ["trigger"],
           "vtol": ["landing_pos_error_m", "hover_power_W", "ap_QTUN_ThO_median"], "smoke": ["mode_changes", "max_cas"],
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
            G("min airspeed in flight >= pessimistic Vs (AIRSPEED_STALL) [m/s]", mca, Vs_pess, ">=",
              "FAIL" if mca < Vs_nom else "WARN", "m/s",
              note=f"Vs nominal {Vs_nom:.2f} (model flown) / pessimistic {Vs_pess:.2f} m/s; below nominal = FAIL",
              drivers=["/airframe/lifting_surfaces/wing/cl_max"])
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
            G("SITL Vs(1g) vs aero stage abs(ratio-1)", abs(r["V_stall_1g_mean"] / Vs - 1), 0.10, "<=", "WARN",
                     note=f"SITL {r['V_stall_1g_mean']} m/s vs {Vs} m/s; alt loss {max((s['alt_loss_m'] for s in r.get('stalls', [])), default=0)} m")
        if name == "stall" and r.get("stalls"):
            st0 = r["stalls"][0]
            G("(FBWA, idle, full back stick): altitude lost / max roll", st0["alt_loss_m"], None, "info", "INFO", "m",
                     note=f"max alpha {st0['max_alpha_deg']} deg, roll {st0['max_abs_roll_deg']} deg; keep STALL_PREVENTION=1 and "
                          f"practise stalls above {max(30, 1.5 * st0['alt_loss_m']):.0f} m")
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
            if mine and r.get("hover_power_W") and "wind" not in name:
                G(f"SITL hover power / propulsion prediction", r["hover_power_W"] / mine["P_total_W"], [0.85, 1.15], "range",
                         "WARN", note=f"{r['hover_power_W']:.0f} W vs {mine['P_total_W']:.0f} W")
            thr = r.get("ap_QTUN_ThO_median")
            if thr is not None:
                G(f"Q-mode hover throttle (QTUN.ThO)", thr, 0.70, "<=", "FAIL")
    # energy: SITL-measured Wh/km calibrates the propulsion stage's mission energy
    energy = None
    base = next((r for n, r in results.items() if n.startswith("mission_l1") and "course_Wh_per_km" in r), None)
    if base and prop and prop.get("missions"):
        pc = prop["cruise_unit"].get("power_curve", [])
        V = base.get("course_mean_cas", plan["envelope"]["v_cruise"])
        whkm_pred = float(np.interp(V, [p["V"] for p in pc], [p["Wh_per_km"] for p in pc])) if pc else None
        ratio = base["course_Wh_per_km"] / whkm_pred if whkm_pred else None
        mi = prop["missions"][0]
        E_sitl = mi["energy_Wh"] * ratio if ratio else None
        energy = dict(sitl_Wh_per_km=base["course_Wh_per_km"], predicted_Wh_per_km=whkm_pred, ratio=ratio, mission=mi["name"],
                      mission_energy_steady_Wh=mi["energy_Wh"], mission_energy_sitl_scaled_Wh=E_sitl,
                      available_Wh=mi["available_after_reserve_Wh"])
        ctx.gate("SITL energy per km / steady-state propulsion prediction", ratio, [0.8, 1.3], "range", "WARN",
                 note=f"{base['course_Wh_per_km']:.3f} vs {whkm_pred:.3f} Wh/km at {V:.1f} m/s (turns, TECS, ArduPilot loops)")
        ctx.gate(f"mission '{mi['name']}' energy (SITL-calibrated) <= usable after reserve [Wh]", E_sitl, mi["available_after_reserve_Wh"],
                 "<=", "FAIL", "Wh")
        for n, r in results.items():
            if n.startswith("mission_wind") and "course_Wh_per_km" in r:
                Ew = mi["energy_Wh"] * r["course_Wh_per_km"] / whkm_pred
                own = n == f"mission_wind_{mi.get('environment')}"
                ctx.gate(f"{n}: mission energy with this wind/turbulence (SITL-scaled) [Wh]", Ew, mi["available_after_reserve_Wh"], "<=",
                         "FAIL" if own else "WARN", "Wh",
                         note=("mission's own environment" if own else "other environment case -> operating limit for this mission")
                              + f"; {r['course_Wh_per_km']:.2f} Wh/km")
    # L1 sweep summary
    l1 = None
    l1runs = [str(res_dir / n) for n in results if n.startswith("mission_l1") and "xt_truth_rms_m" in results[n]]
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
                wall_min=round(sum(v or 0 for v in walls.values()) / 60, 1), energy_ratio=energy and energy.get("ratio"))
    return dict(summary=summ, data=data)
