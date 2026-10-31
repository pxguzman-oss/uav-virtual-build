#!/usr/bin/env python3
"""
run_tests.py - automated, headless ArduPlane SITL test matrix for the Stork (JSBSim bridge).

  run_tests.py mission   [--l1 15] [--wind 6,270] [--turb 3,6] [--name X]
  run_tests.py l1sweep   [--periods 10,13,15,18,22]
  run_tests.py cruise    [--speeds 10,11,...]
  run_tests.py stall
  run_tests.py launch    [--accel 25 --dur 0.45]   (hand-launch detection / weak throw)
  run_tests.py fs_rc | fs_gcs | fs_batt
  run_tests.py all

Every run writes results/<name>/{truth.csv, logs/00000001.BIN, flight.tlog, events.json, ...}
and is then analysed by analyze.py (metrics.json + PNG plots).
"""
import argparse, json, math, os, sys, time, traceback
from pymavlink import mavutil

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sitl import SitlRun, ll_offset
import course
import analyze

# Set by the caller (vb.stages.sitl): where results go and which generated model / parameter files to use.
CFG = dict(results=os.path.abspath("results"), gen_dir=None, model="model", parm_files=None, home=None,
           survey_alt=60, takeoff_alt=40, cruise_v=13.0)


def env_bargs(a):
    """Bridge arguments for the run's environment (wind, Dryden turbulence, scripted gusts, air temperature)."""
    b = []
    if getattr(a, "wind", None):
        b += ["--wind", a.wind]
    if getattr(a, "turb", None):
        b += ["--turb", a.turb]
    if getattr(a, "gusts", None):
        b += ["--gusts", a.gusts, "--seed", str(getattr(a, "seed", 1) or 1)]
    if getattr(a, "temp_c", None) is not None:
        b += ["--temp-c", str(a.temp_c)]
    return b


def env_meta(a):
    return dict(wind=getattr(a, "wind", None), turb=getattr(a, "turb", None), gusts=getattr(a, "gusts", None),
                temp_c=getattr(a, "temp_c", None), home_alt_m=getattr(a, "home_alt", None),
                env_label=getattr(a, "env_label", None) or "calm (no wind, no turbulence, no gusts)")


def run_home(a):
    alt = getattr(a, "home_alt", None)
    return f"{HOME[0]},{HOME[1]},{alt if alt is not None else HOME[2]},0"


def new_run(name, a=None, **kw):
    if a is not None:
        kw.setdefault("home", run_home(a))
        if getattr(a, "gen_dir", None):
            kw.setdefault("gen_dir", a.gen_dir)
        if getattr(a, "model", None):
            kw.setdefault("model", a.model)
    kw.setdefault("model", CFG["model"])
    kw.setdefault("gen_dir", CFG["gen_dir"])
    kw.setdefault("parm_files", CFG["parm_files"])
    kw.setdefault("home", home_str())
    kw.setdefault("instance", CFG.get("instance", 0))
    return SitlRun(os.path.join(CFG["results"], name), **kw)

M = mavutil.mavlink
HOME = (-35.363262, 149.165237, 584.0, 0.0)
ALT = 60  # survey altitude AGL (CFG['survey_alt'] wins)


def home_str():
    return CFG.get("home") or ",".join(str(x) for x in HOME)


def takeoff_items(alt=40, pitch=12):
    return [dict(cmd=M.MAV_CMD_NAV_WAYPOINT, lat=HOME[0], lon=HOME[1], alt=0),       # 0 home
            dict(cmd=M.MAV_CMD_NAV_TAKEOFF, p1=pitch, alt=alt)]                       # 1


def canal_items(spacing=25.0, cam_dist=20.0, alt=None, comp_l1=None):
    alt = alt or CFG['survey_alt']
    cl = course.centreline()
    wps, L = course.waypoints(cl, spacing)
    if comp_l1:
        wps = course.compensate(wps, cl, comp_l1)
    items = takeoff_items()
    items.append(dict(cmd=M.MAV_CMD_DO_SET_CAM_TRIGG_DIST, p1=cam_dist, p3=1))
    first = len(items)
    for n, e in wps:
        lat, lon = ll_offset(HOME[0], HOME[1], n, e)
        items.append(dict(cmd=M.MAV_CMD_NAV_WAYPOINT, lat=lat, lon=lon, alt=alt))
    last = len(items) - 1
    items.append(dict(cmd=M.MAV_CMD_DO_SET_CAM_TRIGG_DIST, p1=0))
    lat, lon = ll_offset(HOME[0], HOME[1], wps[-1][0], wps[-1][1])
    items.append(dict(cmd=M.MAV_CMD_NAV_LOITER_TURNS, p1=2, p3=80, lat=lat, lon=lon, alt=alt))
    items.append(dict(cmd=M.MAV_CMD_NAV_RETURN_TO_LAUNCH))
    meta = dict(first_wp=first, last_wp=last, course_len_m=L, n_wp=len(wps), spacing=spacing)
    return items, meta, cl


def launch_and_climb(s, items, alt=40):
    s.wait_ready()
    s.upload_mission(items)
    s.set_mode("AUTO")
    s.arm()
    s.sleep(2)
    s.launch()
    s.wait(lambda: s.msgs.get("GLOBAL_POSITION_INT") and s.msgs["GLOBAL_POSITION_INT"].relative_alt > (alt - 5) * 1000,
           timeout=120)
    s.event("takeoff_alt_reached")


def finish(s, name, kind, meta=None):
    s.stop()
    meta = meta or {}
    meta.update(kind=kind, wall_s=s.wall_s, sim_s=s.t)
    json.dump(meta, open(os.path.join(s.outdir, "meta.json"), "w"), indent=1)
    try:
        res = analyze.analyze_run(s.outdir)
    except Exception:
        traceback.print_exc()
        res = {}
    print(f"== {name}: sim {s.t:.0f}s in wall {s.wall_s:.0f}s (x{s.t / max(s.wall_s, 1):.1f})")
    return res


# ----------------------------------------------------------------------------- scenarios
def t_mission(a, name=None, l1=None, wind=None, turb=None, speedup=100):
    name = name or a.name or "mission"
    params = {}
    if l1 or a.l1:
        params["NAVL1_PERIOD"] = l1 or a.l1
    w = wind or a.wind
    tb = turb or a.turb
    bargs = env_bargs(a)
    comp = None
    if a.compensate:
        per = params.get("NAVL1_PERIOD", 15)
        comp = 0.75 * per * CFG["cruise_v"] / math.pi   # L1 distance = damping*period*V/pi
    items, meta, cl = canal_items(spacing=a.spacing, comp_l1=comp)
    meta.update(params=params, compensated_L1_m=comp, **env_meta(a))
    extra = []
    if a.backend == "simplane":
        extra = [os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "data", "ardupilot", "simplane_vtail.parm")]
    meta["backend"] = a.backend
    s = new_run(name, a, params=params, bridge_args=bargs, speedup=speedup,
                rate_hz=a.rate if a.backend == "jsbsim" else None, backend=a.backend, extra_parm_files=extra)
    s.start()
    try:
        launch_and_climb(s, items)
        s.wait(lambda: s.mission_seq >= meta["first_wp"] + 2, timeout=200)
        s.event("course_start")
        s.wait(lambda: s.mission_seq > meta["last_wp"], timeout=1200)
        s.event("course_end")
        s.wait(lambda: s.mode == "RTL", timeout=600)
        s.event("rtl")
        s.sleep(90)
    except Exception as ex:
        traceback.print_exc()
        meta["error"] = str(ex)
    return finish(s, name, "mission", meta)


def t_smoke(a):
    """Quick end-to-end check: hand launch, climb, 4-waypoint box (~1.2 km), RTL."""
    name = a.name or "smoke"
    items = takeoff_items(alt=CFG["takeoff_alt"])
    for dn, de in [(300, 0), (300, 250), (0, 250), (0, 0)]:
        lat, lon = ll_offset(HOME[0], HOME[1], dn, de)
        items.append(dict(cmd=M.MAV_CMD_NAV_WAYPOINT, lat=lat, lon=lon, alt=CFG["survey_alt"]))
    items.append(dict(cmd=M.MAV_CMD_NAV_RETURN_TO_LAUNCH))
    meta = dict(n_wp=4, **env_meta(a))
    s = new_run(name, a, bridge_args=env_bargs(a))
    s.start()
    try:
        launch_and_climb(s, items, alt=CFG["takeoff_alt"])
        s.wait(lambda: s.mode == "RTL", timeout=600)
        s.event("rtl")
        s.sleep(30)
    except Exception as ex:
        traceback.print_exc()
        meta["error"] = str(ex)
    return finish(s, name, "smoke", meta)


def t_l1sweep(a):
    out = {}
    for p in [float(x) for x in a.periods.split(",")]:
        out[p] = t_mission(a, name=f"l1_{p:g}" + (f"_{a.name}" if a.name else ""), l1=p)
    return out


def t_cruise(a):
    name = a.name or "cruise_sweep"
    items = takeoff_items(alt=60)
    far_lat, far_lon = ll_offset(HOME[0], HOME[1], 30000, 0)
    items.append(dict(cmd=M.MAV_CMD_NAV_WAYPOINT, lat=far_lat, lon=far_lon, alt=60))
    s = new_run(name, a, params={"AIRSPEED_MIN": max(a.speeds_min, 9)}, bridge_args=env_bargs(a))
    s.start()
    meta = {"segments": [], **env_meta(a)}
    try:
        launch_and_climb(s, items, alt=55)
        s.sleep(20)
        for v in [float(x) for x in a.speeds.split(",")]:
            s.change_speed(v)
            s.event("speed", v=v)
            t0 = s.t
            s.sleep(a.settle)
            t1 = s.t
            s.sleep(a.hold)
            meta["segments"].append(dict(v=v, t_start=t1, t_end=s.t, t_cmd=t0))
    except Exception as ex:
        traceback.print_exc(); meta["error"] = str(ex)
    return finish(s, name, "cruise", meta)


def t_stall(a):
    name = a.name or "stall"
    items = takeoff_items(alt=80)
    far_lat, far_lon = ll_offset(HOME[0], HOME[1], 20000, 0)
    items.append(dict(cmd=M.MAV_CMD_NAV_WAYPOINT, lat=far_lat, lon=far_lon, alt=100))
    s = new_run(name, a, params={"RC_OVERRIDE_TIME": -1, "STALL_PREVENTION": 0}, bridge_args=env_bargs(a))
    s.start()
    meta = {"stalls": [], **env_meta(a)}
    try:
        launch_and_climb(s, items, alt=75)
        s.sleep(25)
        for rep in range(2):
            # FBWA, idle, wings level, ramp pitch stick to full back over 12 s
            s.rc_override(rc1=1500, rc2=1500, rc3=1000, rc4=1500)
            s.set_mode("FBWA")
            s.sleep(3)
            s.event("stall_entry", rep=rep)
            t0 = s.t
            while s.t - t0 < 15:
                f = min((s.t - t0) / 12.0, 1.0)
                s.rc_override(rc1=1500, rc2=int(1500 + 500 * f), rc3=1000, rc4=1500)  # RC2 high = nose up (verified: low PWM dived)
                time.sleep(0.01)
            s.event("stall_hold_end", rep=rep)
            meta["stalls"].append(dict(t0=t0, t1=s.t))
            s.rc_override(rc1=1500, rc2=1500, rc3=1000, rc4=1500)
            s.sleep(2)
            s.rc_override(rc1=0, rc2=0, rc3=0, rc4=0)  # release overrides
            s.set_mode("AUTO")
            s.sleep(40)
    except Exception as ex:
        traceback.print_exc(); meta["error"] = str(ex)
    return finish(s, name, "stall", meta)


def t_launch(a):
    name = a.name or f"launch_a{a.accel:g}_d{a.dur:g}"
    items = takeoff_items(alt=40)
    items.append(dict(cmd=M.MAV_CMD_NAV_LOITER_UNLIM, lat=HOME[0] + 0.002, lon=HOME[1], alt=40))
    # override launch strength through a temp model copy
    gdir = getattr(a, "gen_dir", None) or CFG["gen_dir"]
    mname = getattr(a, "model", None) or CFG["model"]
    mpath = os.path.join(gdir, mname + ".json")
    s = new_run(name, a, bridge_args=[], log_hz=100)
    # patch launch params via bridge model copy
    m = json.load(open(mpath))
    m["launch"]["accel_mps2"] = a.accel
    m["launch"]["duration_s"] = a.dur
    meta = dict(accel=a.accel, dur=a.dur, release_speed_est=a.accel * a.dur, model=mname, **env_meta(a))
    tmp = os.path.join(CFG["results"], name + "_model.json")
    json.dump(m, open(tmp, "w"))
    s.bridge_args = env_bargs(a) + ["--model", tmp]  # later --model wins in argparse
    s.start()
    try:
        s.wait_ready()
        s.upload_mission(items)
        s.set_mode("AUTO")
        s.arm()
        s.sleep(3)
        s.launch()
        s.sleep(40)
    except Exception as ex:
        traceback.print_exc(); meta["error"] = str(ex)
    return finish(s, name, "launch", meta)


def t_failsafe(a, kind):
    name = a.name or f"fs_{kind}"
    items, meta, cl = canal_items(spacing=a.spacing)
    params = {}
    bargs = env_bargs(a)
    if kind == "gcs":
        params["FS_GCS_ENABL"] = 1
    if kind == "batt":
        bargs += ["--soc", str(a.soc)]
        if getattr(a, "batt_low_mah", None):
            # time-box via the mAh failsafe (the one derived.param relies on for Li-ion): trips after ~150 mAh used
            params["BATT_LOW_MAH"] = a.batt_low_mah
        else:
            # legacy: raise the low-voltage threshold so the (sag-compensated) voltage trips it within minutes
            params["BATT_LOW_VOLT"] = a.batt_low_volt
    s = new_run(name, a, params=params, bridge_args=bargs)
    s.start()
    meta.update(fs=kind, **env_meta(a))
    try:
        launch_and_climb(s, items)
        s.wait(lambda: s.mission_seq >= meta["first_wp"] + 10, timeout=300)
        if kind == "rc":
            s.set_param("SIM_RC_FAIL", 1)
            s.event("rc_lost")
        elif kind == "gcs":
            s._hb = False
            s.event("gcs_lost")
        if kind in ("rc", "gcs"):
            s.sleep(60)
            if kind == "rc":
                s.set_param("SIM_RC_FAIL", 0)
            else:
                s._hb = True
            s.event(f"{kind}_restored")
            s.sleep(30)
        else:
            s.wait(lambda: any("Battery" in t or "battery" in t for _, t in s.statustext), timeout=1500)
            s.event("batt_fs_text", text=[t for _, t in s.statustext if "atter" in t][-1])
            s.sleep(90)
    except Exception as ex:
        traceback.print_exc(); meta["error"] = str(ex)
    return finish(s, name, f"fs_{kind}", meta)


def t_vtol(a):
    """QuadPlane: VTOL takeoff from the ground, transition, 4-leg box, VTOL land at home."""
    name = a.name or "vtol"
    items = [dict(cmd=M.MAV_CMD_NAV_WAYPOINT, lat=HOME[0], lon=HOME[1], alt=0),
             dict(cmd=M.MAV_CMD_NAV_VTOL_TAKEOFF, alt=30)]
    for dn, de in getattr(a, "vtol_box", [(400, 0), (400, 300), (0, 300), (-100, 0)]):
        lat, lon = ll_offset(HOME[0], HOME[1], dn, de)
        items.append(dict(cmd=M.MAV_CMD_NAV_WAYPOINT, lat=lat, lon=lon, alt=50))
    items.append(dict(cmd=M.MAV_CMD_NAV_VTOL_LAND, lat=HOME[0], lon=HOME[1], alt=0))
    bargs = ["--start", "ground"] + env_bargs(a)
    s = new_run(name, a, rate_hz=1200, bridge_args=bargs)
    meta = dict(model=CFG["model"], n_items=len(items), **env_meta(a))
    s.start()
    try:
        s.wait_ready()
        s.upload_mission(items)
        s.set_mode("AUTO")
        s.arm()
        s.wait(lambda: s.mission_seq >= 2, timeout=120)
        s.event("vtol_takeoff_done")
        s.wait(lambda: s.mission_seq >= len(items) - 1, timeout=900)
        s.event("vtol_land_start")
        s.wait(lambda: not s.armed, timeout=300)
        s.event("disarmed")
        s.sleep(3)
    except Exception as ex:
        traceback.print_exc(); meta["error"] = str(ex)
    return finish(s, name, "vtol", meta)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("test")
    ap.add_argument("--name", default=None)
    ap.add_argument("--l1", type=float, default=None)
    ap.add_argument("--wind", default=None)
    ap.add_argument("--turb", default=None)
    ap.add_argument("--spacing", type=float, default=25.0)
    ap.add_argument("--periods", default="10,13,15,18,22")
    ap.add_argument("--speeds", default="10,11,12,13,14,15,16,18,20")
    ap.add_argument("--settle", type=float, default=25)
    ap.add_argument("--hold", type=float, default=25)
    ap.add_argument("--accel", type=float, default=25)
    ap.add_argument("--dur", type=float, default=0.45)
    ap.add_argument("--soc", type=float, default=0.22)
    ap.add_argument("--compensate", action="store_true", help="pre-distort arc waypoints outward by L1^2/2R")
    ap.add_argument("--backend", choices=["jsbsim", "simplane"], default="jsbsim")
    ap.add_argument("--rate", type=int, default=400, help="SIM_RATE_HZ (physics/IMU rate)")
    a = ap.parse_args()
    t = a.test
    if t == "mission":
        r = t_mission(a)
    elif t == "l1sweep":
        r = t_l1sweep(a)
    elif t == "cruise":
        r = t_cruise(a)
    elif t == "stall":
        r = t_stall(a)
    elif t == "launch":
        r = t_launch(a)
    elif t == "vtol":
        r = t_vtol(a)
    elif t.startswith("fs_"):
        r = t_failsafe(a, t[3:])
    else:
        raise SystemExit("unknown test")
    print(json.dumps(r, indent=1, default=str)[:4000])


if __name__ == "__main__":
    main()
