#!/usr/bin/env python3
"""
sitl.py - headless launcher + MAVLink helper for ArduPlane SITL driven by bridge.py (JSBSim).
No MAVProxy / GUI needed: pymavlink talks straight to SITL's TCP 5760.

    with SitlRun("results/mission1", model="stork_p0", speedup=100) as s:
        s.wait_ready(); s.upload_mission(items); s.set_mode("AUTO"); s.arm(); s.launch()
        s.wait(lambda: s.mission_seq >= n, timeout=900)
"""
import json, math, os, shutil, signal, socket, subprocess, threading, time
from pymavlink import mavutil

import sys
ROOT = os.path.abspath(os.path.dirname(__file__))
ARDUPLANE = os.environ.get("VB_ARDUPLANE", "arduplane")       # set by vb.stages.sitl from vb.env
PY = sys.executable
DEFAULT_HOME = "-35.363262,149.165237,584,0"   # ArduPilot CMAC test field, heading north


class SitlRun:
    def __init__(self, outdir, model="stork_p0", speedup=100, home=DEFAULT_HOME, params=None,
                 bridge_args=None, extra_parm_files=None, instance=0, backend="jsbsim", rate_hz=None,
                 log_hz=25, gen_dir=None, parm_files=None):
        self.outdir = os.path.abspath(outdir)
        self.gen_dir = gen_dir
        self.parm_files = parm_files
        self.model = model
        self.speedup = speedup
        self.home = home
        self.params = params or {}
        self.bridge_args = bridge_args or []
        self.extra = extra_parm_files or []
        self.inst = instance
        self.backend = backend
        self.rate_hz = rate_hz
        self.log_hz = log_hz
        self.procs = []
        self.mav = None
        self.t_boot_ms = 0
        self.mission_seq = 0
        self.mode = ""
        self.armed = False
        self.msgs = {}
        self.statustext = []
        self._stop = False
        self._hb = True
        self.ctl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.events = []
        self.mission_reached = set()

    # --------------------------------------------------------------- process control
    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *a):
        self.stop()

    def start(self):
        try:
            self._start()
        except Exception:
            self.stop()
            raise

    def _start(self):
        if os.path.exists(self.outdir):
            shutil.rmtree(self.outdir)
        os.makedirs(self.outdir)
        gen = self.gen_dir or os.path.join(ROOT, "generated", self.model)
        parms = list(self.parm_files) if self.parm_files else [os.path.join(gen, f"{self.model}_model.parm")]
        parms += self.extra
        # write run-specific param overrides into a parm file so they are defaults at boot
        over = os.path.join(self.outdir, "overrides.parm")
        with open(over, "w") as f:
            if self.rate_hz:
                f.write(f"SIM_RATE_HZ {self.rate_hz}\n")
            for k, v in self.params.items():
                f.write(f"{k} {v}\n")
        parms.append(over)
        self.t_wall0 = time.time()
        if self.backend == "jsbsim":
            blog = open(os.path.join(self.outdir, "bridge.out"), "w")
            hdg = self.home.split(",")
            cmd = [PY, os.path.join(ROOT, "bridge.py"), "--model", os.path.join(gen, f"{self.model}.json"),
                   "--jsbsim-dir", os.path.join(gen, "jsbsim"), "--home=" + self.home,
                   "--port", str(9002 + 10 * self.inst), "--ctl-port", str(9012 + 10 * self.inst),
                   "--log", os.path.join(self.outdir, "truth.csv"), "--log-hz", str(self.log_hz)] + self.bridge_args
            self.procs.append(subprocess.Popen(cmd, stdout=blog, stderr=subprocess.STDOUT, cwd=self.outdir))
            model_arg = "JSON:127.0.0.1"
        else:  # built-in SIM_Plane with generated coefficient file (fallback)
            # NB: SIM_Plane's JSON loader fails on ABSOLUTE paths (AP_Filesystem) -> copy into cwd, pass relative name
            shutil.copy(os.path.join(gen, self.model + '_simplane.json'), os.path.join(self.outdir, "simplane.json"))
            model_arg = "plane-vtail:simplane.json"
        slog = open(os.path.join(self.outdir, "sitl.out"), "w")
        cmd = [ARDUPLANE, "-w", "--model", model_arg, "--speedup", str(self.speedup),
               "--defaults", ",".join(parms), "--home=" + self.home, "-I", str(self.inst)]
        if self.backend == "jsbsim":
            cmd += ["--sim-port-out", str(9002 + 10 * self.inst)] if self.inst else []
        self.procs.append(subprocess.Popen(cmd, stdout=slog, stderr=subprocess.STDOUT, cwd=self.outdir))
        with open(os.path.join(self.outdir, "cmdline.txt"), "w") as f:
            f.write(" ".join(cmd) + "\n")
        port = 5760 + 10 * self.inst
        for _ in range(100):
            try:
                self.mav = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}", source_system=255,
                                                      autoreconnect=False, retries=1)
                break
            except Exception:
                time.sleep(0.2)
        self.tlog = open(os.path.join(self.outdir, "flight.tlog"), "wb")
        self.mav.setup_logfile_raw = None
        self.rx = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx.start()
        self.hb = threading.Thread(target=self._hb_loop, daemon=True)
        self.hb.start()
        self.wait(lambda: self.t_boot_ms > 0, timeout=60, sim=False)
        for _ in range(3):
            self.mav.mav.request_data_stream_send(1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
            time.sleep(0.1)

    def stop(self):
        self._stop = True
        time.sleep(0.3)
        for p in self.procs[::-1]:
            try:
                p.send_signal(signal.SIGINT); p.wait(timeout=5)
            except Exception:
                p.kill()
        for p in self.procs:
            if p.poll() is None:
                p.kill()
        try:
            self.tlog.close()
        except Exception:
            pass
        self.wall_s = time.time() - self.t_wall0
        with open(os.path.join(self.outdir, "events.json"), "w") as f:
            json.dump({"events": self.events, "statustext": self.statustext, "wall_s": self.wall_s,
                       "sim_s": self.t_boot_ms / 1000.0}, f, indent=1)

    # --------------------------------------------------------------- mavlink plumbing
    def _rx_loop(self):
        while not self._stop:
            try:
                m = self.mav.recv_match(blocking=True, timeout=0.5)
            except Exception:
                continue
            if m is None:
                continue
            try:
                self.tlog.write(m.get_msgbuf())
            except Exception:
                pass
            t = m.get_type()
            self.msgs[t] = m
            if hasattr(m, "time_boot_ms"):
                self.t_boot_ms = max(self.t_boot_ms, m.time_boot_ms)
            if t == "HEARTBEAT" and m.get_srcComponent() == 1 and m.type != mavutil.mavlink.MAV_TYPE_GCS:
                self.mode = mavutil.mode_string_v10(m)
                self.armed = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif t == "STATUSTEXT":
                self.statustext.append((self.t_boot_ms / 1000.0, m.text))
            elif t == "MISSION_CURRENT":
                self.mission_seq = m.seq
            elif t == "MISSION_ITEM_REACHED":
                self.mission_reached.add(m.seq)

    def _hb_loop(self):
        while not self._stop:
            if self._hb:
                try:
                    self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                except Exception:
                    pass
            time.sleep(0.05)

    @property
    def t(self):
        return self.t_boot_ms / 1000.0

    def event(self, name, **kw):
        e = dict(t=self.t, name=name, **kw)
        self.events.append(e)
        print(f"[{self.t:8.1f}s] {name} {kw if kw else ''}", flush=True)
        self.bridge(f"mark {name}")

    def wait(self, cond, timeout=600, sim=True, poll=0.02):
        t0 = self.t if sim else time.time()
        w0 = time.time()
        while True:
            if cond():
                return True
            now = self.t if sim else time.time()
            if now - t0 > timeout:
                raise TimeoutError(f"timeout after {timeout}s")
            if time.time() - w0 > max(timeout, 60) * 3 + 60:
                raise TimeoutError("wall-clock timeout")
            for p in self.procs:
                if p.poll() is not None:
                    raise RuntimeError(f"process exited: {p.args[:2]} rc={p.returncode}")
            time.sleep(poll)

    def sleep(self, sim_s):
        t0 = self.t
        self.wait(lambda: self.t - t0 >= sim_s, timeout=sim_s + 30)

    def bridge(self, cmd):
        if self.backend == "jsbsim":
            self.ctl.sendto(cmd.encode(), ("127.0.0.1", 9012 + 10 * self.inst))

    # --------------------------------------------------------------- params / modes / arming
    def set_param(self, name, value, retries=5):
        for _ in range(retries):
            self.mav.mav.param_set_send(1, 1, name.encode(), float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            t0 = time.time()
            while time.time() - t0 < 1.0:
                pv = self.msgs.get("PARAM_VALUE")
                if pv is not None and pv.param_id == name and abs(pv.param_value - float(value)) < 1e-3 * max(1, abs(value)):
                    return True
                time.sleep(0.02)
        raise RuntimeError(f"param set failed {name}")

    def set_mode(self, mode):
        mode_id = self.mav.mode_mapping()[mode]
        for _ in range(20):
            self.mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                           mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
            try:
                self.wait(lambda: self.mode == mode, timeout=2)
                self.event("mode", mode=mode)
                return
            except TimeoutError:
                pass
        raise RuntimeError(f"could not set mode {mode}")

    def wait_ready(self, timeout=180):
        """wait for EKF to be using GPS and prearm checks to pass"""
        def ok():
            ss = self.msgs.get("SYS_STATUS")
            ekf = self.msgs.get("EKF_STATUS_REPORT")
            gps = self.msgs.get("GPS_RAW_INT")
            if ss is None or gps is None or gps.fix_type < 3:
                return False
            prearm = ss.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
            return bool(prearm) and (ekf is None or (ekf.flags & 0x10))  # pred horiz pos abs
        self.wait(ok, timeout=timeout)
        self.event("ready")

    def arm(self, force=False):
        for _ in range(20):
            self.mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1,
                                           21196 if force else 0, 0, 0, 0, 0, 0)
            try:
                self.wait(lambda: self.armed, timeout=3)
                self.event("armed")
                return
            except TimeoutError:
                pass
        raise RuntimeError("arming failed: " + str(self.statustext[-5:]))

    def launch(self):
        self.bridge("launch")
        self.event("hand_launch")

    def rc_override(self, **ch):
        vals = [65535] * 18
        for k, v in ch.items():
            vals[int(k[2:]) - 1] = int(v)
        self.mav.mav.rc_channels_override_send(1, 1, *vals)

    def command(self, cmd, *p):
        p = list(p) + [0] * (7 - len(p))
        self.mav.mav.command_long_send(1, 1, cmd, 0, *p)

    def change_speed(self, v):
        self.command(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, v, -1)

    def guided_goto(self, lat, lon, alt_rel):
        self.mav.mav.mission_item_int_send(1, 1, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                           mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 1, 0, 0, 0, 0,
                                           int(lat * 1e7), int(lon * 1e7), alt_rel)

    # --------------------------------------------------------------- missions
    def upload_mission(self, items):
        """items: list of dicts {cmd, p1..p4, lat, lon, alt, frame}; item 0 = home placeholder"""
        n = len(items)
        mav = self.mav.mav
        self.msgs.pop("MISSION_REQUEST", None); self.msgs.pop("MISSION_REQUEST_INT", None); self.msgs.pop("MISSION_ACK", None)
        mav.mission_clear_all_send(1, 1)
        time.sleep(0.3)
        mav.mission_count_send(1, 1, n)
        sent = set()
        t0 = time.time()
        while time.time() - t0 < 30:
            ack = self.msgs.get("MISSION_ACK")
            if ack is not None and len(sent) == n:
                break
            req = self.msgs.get("MISSION_REQUEST_INT") or self.msgs.get("MISSION_REQUEST")
            if req is not None:
                seq = req.seq
                self.msgs.pop("MISSION_REQUEST_INT", None); self.msgs.pop("MISSION_REQUEST", None)
                it = items[seq]
                mav.mission_item_int_send(1, 1, seq, it.get("frame", mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT),
                                          it["cmd"], 0, 1, it.get("p1", 0), it.get("p2", 0), it.get("p3", 0),
                                          it.get("p4", 0), int(it.get("lat", 0) * 1e7), int(it.get("lon", 0) * 1e7),
                                          it.get("alt", 0))
                sent.add(seq)
            time.sleep(0.01)
        else:
            raise RuntimeError("mission upload timeout")
        self.event("mission_uploaded", n=n)


def ll_offset(lat, lon, dn, de):
    R = 6378137.0
    return lat + math.degrees(dn / R), lon + math.degrees(de / (R * math.cos(math.radians(lat))))
