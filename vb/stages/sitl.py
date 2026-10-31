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


def matrix(ctx, is_vtol, fm):
    an = ctx.plan.get("analysis", {}).get("sitl", {}) or {}
    envs = ctx.plan.get("environment_cases") or []
    std = next((e for e in envs if e["name"] == (ctx.plan.get("missions") or [{}])[0].get("environment")), envs[0] if envs else {})
    acc, dur = an.get("launch_accel", 25), an.get("launch_duration", 0.45)
    fid = ctx.fidelity
    T = []
    if is_vtol:
        box = [(200, 0), (200, 150), (0, 150), (-60, 0)] if fid == "quick" else [(400, 0), (400, 300), (0, 300), (-100, 0)]
        T.append(("vtol", "vtol", dict(vtol_box=box)))
        if fid in ("standard", "full") and std:
            T.append(("vtol", f"vtol_wind_{std['name']}", dict(vtol_box=box, wind=f"{std.get('wind_mean', 4)},270")))
        if fid == "full":
            for e in envs:
                if e is not std:
                    T.append(("vtol", f"vtol_wind_{e['name']}", dict(vtol_box=box, wind=f"{e.get('wind_mean', 4)},270")))
        return T
    if fid == "quick":
        return [("smoke", "smoke", {})]
    base_l1 = an.get("l1_base", 15)
    T.append(("mission", f"mission_l1_{base_l1:g}", dict(l1=base_l1)))
    periods = an.get("l1_periods_full" if fid == "full" else "l1_periods_standard", [10, 15])
    for p in periods:
        if p != base_l1:
            T.append(("mission", f"mission_l1_{p:g}", dict(l1=p)))
    T.append(("stall", "stall", {}))
    T.append(("launch", "launch_nominal", dict(accel=acc, dur=dur)))
    T.append(("launch", "launch_weak", dict(accel=an.get("weak_accel", 15), dur=an.get("weak_duration", 0.40))))
    T.append(("fs_rc", "fs_rc", {}))
    if fid == "full":
        T.append(("cruise", "cruise_sweep", {}))
        T.append(("fs_batt", "fs_batt", {}))
        T.append(("fs_gcs", "fs_gcs", {}))
        for e in envs:
            tb = TURB.get(e.get("turbulence", "none"))
            T.append(("mission", f"mission_wind_{e['name']}", dict(l1=base_l1, wind=f"{e.get('wind_mean', 0)},270",
                                                                    turb=f"{tb},{e.get('wind_gust', 6)}" if tb else None)))
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
    RT.CFG.update(results=str(res_dir), gen_dir=str(gen), model=model, parm_files=[str(sp), str(plumb)],
                  survey_alt=an.get("survey_alt", 60), takeoff_alt=an.get("takeoff_alt", 40),
                  cruise_v=plan["envelope"]["v_cruise"])
    params = fm["params"]
    Vs = params.get("AIRSPEED_STALL", 9)
    S = plan["energy"]["series"]
    is_vtol = fm.get("is_vtol")
    tests = matrix(ctx, is_vtol, fm)
    ctx.note(f"SITL matrix ({ctx.fidelity}): " + ", ".join(n for _, n, _ in tests))
    results = {}
    for kind, name, kw in tests:
        a = types.SimpleNamespace(name=name, l1=None, wind=None, turb=None, spacing=an.get("wp_spacing", 25.0),
                                  periods="", speeds=an.get("cruise_speeds", "10,11,12,13,14,15,16,18,20"), speeds_min=params.get("AIRSPEED_STALL", 9),
                                  settle=25, hold=25, accel=25, dur=0.45, soc=0.22, compensate=False, backend="jsbsim", rate=400,
                                  batt_low_volt=round(S * 3.49, 2))
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
            meta["model_json"] = str(gen / f"{model}.json")
            (rd / "meta.json").write_text(json.dumps(meta, indent=1))
            r = AN.analyze_run(str(rd))
        except Exception as e:
            r = dict(r or {}, analyze_error=str(e))
        r["wall_s_total"] = round(time.time() - t0, 1)
        results[name] = r
        (ctx.dir / "sitl_results.json").write_text(json.dumps(results, indent=1, default=str))
        for f in sorted(rd.glob("*.png")) if rd.exists() else []:
            ctx.add_file(f, f"SITL {name}: {f.stem}", "figure")

    # ---- gates ---------------------------------------------------------------------------------------
    env_ = plan["envelope"]
    nlim = env_["n_limit_pos"]
    xt_max = (plan.get("gates") or {}).get("sitl", {}).get("xt_rms_max_m", 10.0)
    for name, r in results.items():
        err = r.get("run_error") or r.get("analyze_error")
        ctx.gate(f"{name}: run completed without scripted-step errors", not err, True, "bool",
                 "WARN" if name.startswith(("fs_batt", "launch_weak")) else "FAIL", note=str(err or "")[:150])
        if r.get("min_airspeed_inflight") is not None and not name.startswith(("stall", "launch_weak")) and "vtol" not in name:
            ctx.gate(f"{name}: min airspeed in flight >= AIRSPEED_STALL [m/s]", r["min_airspeed_inflight"], Vs, ">=", "FAIL", "m/s")
        if r.get("max_cas") is not None:
            ctx.gate(f"{name}: max airspeed <= VNE [m/s]", r["max_cas"], env_["v_ne"], "<=", "FAIL", "m/s")
        if r.get("max_cas") is not None and "vtol" not in name and not name.startswith("stall"):
            ctx.gate(f"{name}: max airspeed <= AIRSPEED_MAX + 2 m/s (climb/descent overspeed)", r["max_cas"],
                     params.get("AIRSPEED_MAX", 22) + 2, "<=", "WARN", "m/s")
        if r.get("max_nz") is not None and not name.startswith("stall"):
            ctx.gate(f"{name}: max load factor <= n_limit", r["max_nz"], nlim, "<=", "FAIL")
        if r.get("max_throttle_saturation_s") is not None and name.startswith(("mission", "smoke")):
            ctx.gate(f"{name}: longest full-throttle stretch [s] (launch climb excluded)", r["max_throttle_saturation_s"],
                     an.get("max_wot_s", 20.0), "<=", "WARN", "s")
        if name.startswith("mission"):
            ctx.gate(f"{name}: course completed and RTL reached", "xt_truth_rms_m" in r, True, "bool", "FAIL")
            if "xt_truth_rms_m" in r:
                ctx.gate(f"{name}: cross-track RMS to the canal centreline [m]", r["xt_truth_rms_m"], xt_max, "<=", "WARN", "m",
                         note=f"max {r.get('xt_truth_max_m')} m; worst bend " + str(max(r.get("xt_truth_by_bend", {}).items(),
                                                                                         key=lambda kv: kv[1]["rms"], default=("-", {}))[0]))
        if name.startswith("smoke"):
            ctx.gate("smoke: launch, box and RTL completed", any(m[1] == 11 for m in r.get("mode_changes", []) if isinstance(m, (list, tuple))), True,
                     "bool", "FAIL")
        if name.startswith("launch"):
            weak = "weak" in name
            ok = not r.get("crashed", True) and (r.get("height_at_10s_m") or 0) >= 15
            ctx.gate(f"{name}: survives the throw and climbs (>=15 m after 10 s)", ok, True, "bool", "WARN" if weak else "FAIL",
                     note=f"release {r.get('release_tas')} m/s, motor +{r.get('motor_start_delay_s')} s, min CAS {r.get('min_cas_after_release')} m/s, "
                          f"h(10 s) {r.get('height_at_10s_m')} m")
            if not weak and r.get("min_cas_after_release") is not None:
                ctx.gate(f"{name}: min airspeed after release / Vs", r["min_cas_after_release"] / Vs, 1.1, ">=", "WARN")
        if name == "stall" and r.get("V_stall_1g_mean"):
            ctx.gate("stall: SITL Vs(1g) vs aero stage (|ratio-1|)", abs(r["V_stall_1g_mean"] / Vs - 1), 0.10, "<=", "WARN",
                     note=f"SITL {r['V_stall_1g_mean']} m/s vs {Vs} m/s; alt loss {max((s['alt_loss_m'] for s in r.get('stalls', [])), default=0)} m")
        if name.startswith("fs_") and name != "fs_batt":
            fc = r.get("first_mode_change_after_trigger") or {}
            ctx.gate(f"{name}: failsafe switches to RTL", fc.get("mode") == 11, True, "bool", "FAIL",
                     note=f"mode {fc.get('mode')} after {fc.get('dt_s')} s")
        if name == "fs_batt":
            fc = r.get("first_mode_change_after_trigger") or {}
            ctx.gate("fs_batt: battery failsafe -> RTL", fc.get("mode") == 11, True, "bool", "FAIL")
        if name.startswith("vtol"):
            ok = r.get("landing_pos_error_m") is not None and not r.get("run_error")
            ctx.gate(f"{name}: VTOL takeoff, transitions both ways, VTOL land, disarm", ok, True, "bool", "FAIL")
            if r.get("landing_pos_error_m") is not None:
                ctx.gate(f"{name}: landing position error [m]", r["landing_pos_error_m"], 2.0, "<=", "WARN", "m")
            thr = r.get("ap_QTUN_ThO_median")
            if thr is not None:
                ctx.gate(f"{name}: Q-mode hover throttle (QTUN.ThO)", thr, 0.70, "<=", "FAIL")
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
                ctx.gate(f"{n}: mission energy with this wind (SITL-scaled) [Wh]", Ew, mi["available_after_reserve_Wh"], "<=", "FAIL", "Wh")
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
