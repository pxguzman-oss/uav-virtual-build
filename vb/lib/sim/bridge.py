#!/usr/bin/env python3
"""
bridge.py - JSBSim (python wheel, in-process) <-> ArduPilot SITL "JSON" physics backend.

  arduplane --model JSON:127.0.0.1  <--UDP 9002-->  bridge.py  (JSBSim FDM + our propulsion,
                                                               battery, wind/gust, hand launch)

* lock-stepped: one physics step per SITL servo packet, timestamp = our sim time, so SITL runs
  as fast as the CPU allows (use arduplane --speedup 100).
* aero / mass / inertia / ground contacts / actuators: generated JSBSim XML (gen_model.py)
* propulsion: motor equivalent circuit (Kv, Rm, I0) + prop CT/CP(J[,rpm]) map + rotor inertia,
  thrust/torque injected as JSBSim external_reactions (so it works for N motors = QuadPlane).
* battery: Rint + 1-RC Thevenin, OCV(SOC) table, feeds voltage/current to SITL (BATT_MONITOR 4).
* wind: steady + JSBSim Milspec/Tustin turbulence + scripted 1-cosine gusts (bridge-side,
  because the SITL JSON backend ignores SIM_WIND_*).
* hand launch: aircraft is "held" (static, accelerometers see 1 g) until a 'launch' command,
  then a body-x throw force (accel_mps2 for duration_s) + hand support, then free flight.
* control port (UDP 9012, text): launch | wind <spd> <from_deg> [down] | turb <sev 0-7> <w20_mps>
  | gust <N> <E> <D> <dur_s> | soc <0..1> | fail_motor <name> | mark <text> | quit
* truth log CSV (default 25 Hz) for analysis (alpha, CL, thrust, rpm, battery ...).
"""
import argparse, json, math, os, socket, struct, sys, time
import numpy as np
import jsbsim

FT = 0.3048
LBF = 4.4482216152605
LBFFT = 1.3558179483314
SLUG = 14.5939029
G = 9.80665
KT = 0.514444


from bisect import bisect_right


def interp(x, xs, ys):
    """linear interpolation with linear EXTRApolation at the ends (pure python: faster than
    numpy for scalars)"""
    n = len(xs)
    if x <= xs[0]:
        i = 1
    elif x >= xs[-1]:
        i = n - 1
    else:
        i = bisect_right(xs, x)
    x0, x1 = xs[i - 1], xs[i]
    return ys[i - 1] + (ys[i] - ys[i - 1]) * (x - x0) / (x1 - x0)


def interp_clamped(x, xs, ys):
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    i = bisect_right(xs, x)
    return ys[i - 1] + (ys[i] - ys[i - 1]) * (x - xs[i - 1]) / (xs[i] - xs[i - 1])


class PropMap:
    def __init__(self, p):
        t = p["table"]
        self.D = p["diameter_m"]
        self.J = list(t["J"])
        self.rpm = t.get("rpm")
        if self.rpm:
            self.CT = [list(r) for r in t["CT"]]
            self.CP = [list(r) for r in t["CP"]]
        else:
            self.CT = [list(t["CT"])]
            self.CP = [list(t["CP"])]
            self.rpm = [0.0]
        self.ts = p.get("thrust_scale", 1.0)
        self.ps = p.get("power_scale", 1.0)
        self.inertia = p.get("inertia_kgm2", 2e-5)

    def coeffs(self, J, rpm):
        J = min(max(J, -0.2), 1.6)
        rows = self.rpm
        if len(rows) == 1 or rpm <= rows[0]:
            i0, i1, f = 0, 0, 0.0
        elif rpm >= rows[-1]:
            i0, i1, f = len(rows) - 1, len(rows) - 1, 0.0
        else:
            i1 = next(i for i, r in enumerate(rows) if r >= rpm)
            i0 = i1 - 1
            f = (rpm - rows[i0]) / (rows[i1] - rows[i0])
        ct = (1 - f) * interp(J, self.J, self.CT[i0]) + f * interp(J, self.J, self.CT[i1])
        cp = (1 - f) * interp(J, self.J, self.CP[i0]) + f * interp(J, self.J, self.CP[i1])
        return ct * self.ts, cp * self.ps


class Motor:
    def __init__(self, d):
        self.d = d
        self.name = d["name"]
        self.ch = d["pwm"]["channel"] - 1
        esc = d.get("esc", {})
        self.pmin = esc.get("pwm_min", 1000); self.pmax = esc.get("pwm_max", 2000)
        self.regen = esc.get("regen", False); self.esc_eff = esc.get("efficiency", 0.97)
        mo = d["motor"]
        self.kv = mo["kv_rpm_per_v"] * 2 * math.pi / 60.0      # rad/s per V
        self.kt = 1.0 / self.kv                                  # N m / A
        self.rm = mo["rm_ohm"]; self.i0 = mo["i0_a"]; self.i0v = mo.get("i0_ref_v", 10.0)
        self.imax = mo.get("i_max_a", 1e9)
        self.prop = PropMap(d["prop"])
        self.J = mo.get("rotor_inertia_kgm2", 1.5e-5) + self.prop.inertia
        dv = np.array(d["thrust_dir_body"], float)
        self.dir = (dv / np.linalg.norm(dv)).tolist()
        self.spin = d.get("spin", 1)
        self.omega = 0.0
        self.failed = False
        # outputs
        self.thr = self.T = self.Q = self.Im = self.Ibat = self.Pshaft = 0.0
        self.Jadv = 0.0

    def step(self, pwm, vbat, v_air_body, rho, dt):
        thr = 0.0 if (pwm <= 0 or self.failed) else min(max((pwm - self.pmin) / (self.pmax - self.pmin), 0.0), 1.0)
        self.thr = thr
        vm = thr * vbat
        D = self.prop.D
        n = self.omega / (2 * math.pi)
        dv = self.dir
        vax = v_air_body[0] * dv[0] + v_air_body[1] * dv[1] + v_air_body[2] * dv[2]       # inflow along thrust axis (advance velocity)
        if n > 2.0:
            J = vax / (n * D)
            ct, cp = self.prop.coeffs(J, n * 60)
            T = ct * rho * n * n * D ** 4
            Qp = cp * rho * n * n * D ** 5 / (2 * math.pi)
        else:
            J = 0.0
            # windmilling start-up / stopped prop: small drag torque only
            ct, cp = self.prop.coeffs(0.0, 0)
            T = ct * rho * n * abs(n) * D ** 4
            Qp = cp * rho * n * abs(n) * D ** 5 / (2 * math.pi)
        emf = self.omega / self.kv
        Im = (vm - emf) / self.rm
        if not self.regen and Im < 0:
            Im = 0.0
        Im = min(Im, self.imax)
        i0 = self.i0 * min(emf / self.i0v, 3.0)
        Qm = self.kt * (Im - i0) if self.omega > 0 else self.kt * Im
        # semi-implicit integration of rotor speed (stable for stiff motor constants)
        dQdw = -self.kt * self.kt / self.rm - 2 * Qp / max(self.omega, 1.0)
        dw = dt * (Qm - Qp) / (self.J - dt * dQdw)
        self.omega = max(self.omega + dw, 0.0)
        self.T, self.Q, self.Im, self.Jadv = T, Qp, Im, J
        self.Ibat = thr * Im / self.esc_eff if Im > 0 else thr * Im * self.esc_eff
        self.Pshaft = Qp * self.omega
        return T, Qp

    @property
    def rpm(self):
        return self.omega * 60 / (2 * math.pi)


class Battery:
    def __init__(self, b, aux_w):
        self.ns, self.np_ = b["cells_series"], b["cells_parallel"]
        self.cap = b["cell_capacity_ah"] * self.np_
        self.R0 = self.ns * b["cell_r_dc_ohm"] / self.np_ + b.get("pack_extra_r_ohm", 0.01)
        self.R1 = self.ns * b.get("cell_rc_r_ohm", 0.0) / self.np_
        self.tau = b.get("cell_rc_tau_s", 30.0)
        self.soc_t = b["ocv_table"]["soc"]; self.ocv_t = b["ocv_table"]["v_cell"]
        self.soc = b.get("initial_soc", 1.0)
        self.vrc = 0.0
        self.I = 0.0
        self.aux_w = aux_w
        self.Ah = 0.0; self.Wh = 0.0
        self.V = self.ocv() * self.ns

    def ocv(self):
        return interp_clamped(self.soc, self.soc_t, self.ocv_t)

    def step(self, I_load, dt):
        V = self.ns * self.ocv() - I_load * self.R0 - self.vrc
        I = I_load + self.aux_w / max(V, 1.0)
        V = self.ns * self.ocv() - I * self.R0 - self.vrc
        if self.R1 > 0:
            self.vrc += dt * (I * self.R1 - self.vrc) / self.tau
        self.soc = max(self.soc - I * dt / 3600.0 / self.cap, 0.0)
        self.Ah += I * dt / 3600.0
        self.Wh += V * I * dt / 3600.0
        self.V, self.I = V, I
        return V, I


def euler_to_dcm(phi, theta, psi):
    cph, sph, cth, sth, cps, sps = math.cos(phi), math.sin(phi), math.cos(theta), math.sin(theta), math.cos(psi), math.sin(psi)
    # body->NED
    return np.array([
        [cth * cps, sph * sth * cps - cph * sps, cph * sth * cps + sph * sps],
        [cth * sps, sph * sth * sps + cph * cps, cph * sth * sps - sph * cps],
        [-sth, sph * cth, cph * cth]])


class Bridge:
    def __init__(self, a):
        self.a = a
        self.m = json.load(open(a.model))
        self.mass = self.m["mass"]["mass_kg"]
        self.surfaces = self.m["surfaces"]
        self.motors = [Motor(d) for d in self.m["propulsion"]["motors"]]
        self.bat = Battery(self.m["battery"], self.m.get("electrical", {}).get("avionics_power_w", 4.0))
        if a.soc is not None:
            self.bat.soc = a.soc
        self.launch = dict(self.m.get("launch", {}))
        self.home = [float(x) for x in a.home.split(",")]
        self.state = "HELD" if a.start == "hand" else "GROUND"
        self.t = 0.0
        self.dt = None
        self.fdm = None
        self.wind_ned = np.zeros(3)
        self.gusts = []  # (t0, dur, vec)
        self.turb = (0, 0.0)
        self.launch_t0 = None
        self.last_frame = -1
        self.last_reply = None
        self.log = None
        self.log_next = 0.0
        self.mark = ""
        self.pwm = [0] * 32
        if a.wind:
            spd, frm = [float(x) for x in a.wind.split(",")[:2]]
            self.set_wind(spd, frm, 0.0)
        if a.turb:
            sev, w20 = a.turb.split(",")
            self.turb = (int(sev), float(w20))
        # scripted discrete 1-cosine HORIZONTAL gusts (review C2): amp [m/s] above the mean wind, every period [s],
        # duration [s], random horizontal direction (seeded -> reproducible); start once airborne
        self.gust_cfg = None
        self.next_gust = None
        if getattr(a, "gusts", None):
            amp, per, dur = [float(x) for x in a.gusts.split(",")[:3]]
            self.gust_cfg = (amp, per, dur)
            self.rng = np.random.default_rng(int(getattr(a, "seed", 1) or 1))

    # ------------------------------------------------------------------ JSBSim
    def init_fdm(self, dt):
        jdir = self.a.jsbsim_dir
        fdm = jsbsim.FGFDMExec(jdir, None)
        fdm.set_debug_level(0)
        if not fdm.load_model(self.m["name"]):
            raise RuntimeError("JSBSim failed to load model")
        fdm.set_dt(dt)
        lat, lon, alt, hdg = self.home
        fdm["ic/lat-geod-deg"] = lat
        fdm["ic/long-gc-deg"] = lon
        fdm["ic/terrain-elevation-ft"] = alt / FT
        if getattr(self.a, "temp_c", None) is not None:
            # ISA + delta-T so the air density matches the plan environment case (review M1/C2)
            T_isa = 15.0 - 0.0065 * alt
            fdm["atmosphere/delta-T"] = (float(self.a.temp_c) - T_isa) * 1.8
        L = self.launch
        if self.state in ("HELD", "LAUNCH", "FREE"):
            fdm["ic/h-agl-ft"] = L.get("hold_height_m", 1.6) / FT
            fdm["ic/theta-deg"] = L.get("release_pitch_deg", 10)
        else:
            fdm["ic/h-agl-ft"] = 0.15 / FT
            fdm["ic/theta-deg"] = 0
        fdm["ic/psi-true-deg"] = hdg
        fdm["ic/phi-deg"] = 0
        fdm["ic/u-fps"] = 0; fdm["ic/v-fps"] = 0; fdm["ic/w-fps"] = 0
        fdm["ic/p-rad_sec"] = 0; fdm["ic/q-rad_sec"] = 0; fdm["ic/r-rad_sec"] = 0
        fdm.run_ic()
        self.fdm = fdm
        self.dt = dt
        self.lat0, self.lon0 = math.radians(lat), math.radians(lon)
        self.apply_wind()

    def set_wind(self, spd, from_deg, down=0.0):
        to = math.radians(from_deg + 180.0)
        self.wind_ned = np.array([spd * math.cos(to), spd * math.sin(to), down])

    def scripted_gusts(self):
        if not self.gust_cfg or self.state not in ("FREE", "GROUND") or self.fdm["position/h-agl-ft"] * FT < 3.0:
            return
        amp, per, dur = self.gust_cfg
        if self.next_gust is None:
            self.next_gust = self.t + 0.5 * per
        if self.t >= self.next_gust:
            ang = self.rng.uniform(0, 2 * math.pi)
            self.gusts.append((self.t, dur, [amp * math.cos(ang), amp * math.sin(ang), 0.0]))
            self.mark = f"gust {amp:.1f} m/s dir {math.degrees(ang):.0f}"
            self.next_gust = self.t + per
            self.gusts = [g for g in self.gusts if g[0] + g[1] >= self.t - 1]

    def wind_now(self):
        w = self.wind_ned.copy()
        for (t0, dur, vec) in self.gusts:
            if t0 <= self.t <= t0 + dur:
                w += np.array(vec) * 0.5 * (1 - math.cos(2 * math.pi * (self.t - t0) / dur))
        return w

    def apply_wind(self):
        if self.fdm is None:
            return
        w = self.wind_now()
        self.fdm["atmosphere/wind-north-fps"] = w[0] / FT
        self.fdm["atmosphere/wind-east-fps"] = w[1] / FT
        self.fdm["atmosphere/wind-down-fps"] = w[2] / FT
        sev, w20 = self.turb
        if sev > 0:
            self.fdm["atmosphere/turb-type"] = 3   # Milspec Dryden
            self.fdm["atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps"] = w20 / FT
            self.fdm["atmosphere/turbulence/milspec/severity"] = sev
        else:
            self.fdm["atmosphere/turb-type"] = 0

    # ------------------------------------------------------------------ controls
    def surface_cmds(self, pwm):
        if not hasattr(self, "_surf"):
            self._surf = []
            for s in self.surfaces:
                p = s["pwm"]
                self._surf.append((p["channel"] - 1, p.get("min", 1000), p.get("trim", 1500), p.get("max", 2000),
                                   p.get("sign", 1), math.radians(s["max_deflection_deg"]),
                                   math.radians(s.get("min_deflection_deg", s["max_deflection_deg"])),
                                   f"fcs/{s['name']}-cmd-rad"))
        f = self.fdm
        for ch, mn, tr, mx, sign, amax, amin, prop in self._surf:
            v = pwm[ch]
            if v <= 0:
                v = tr
            nrm = (v - tr) / (mx - tr) if v >= tr else (v - tr) / (tr - mn)
            nrm = max(-1.0, min(1.0, nrm)) * sign
            f[prop] = nrm * (amax if nrm >= 0 else amin)

    def solve_bus_voltage(self, pwm):
        """Solve the battery/ESC/motor algebraic loop for the bus voltage this step
        (V = E - R0*I, I = sum thr*(thr*V - emf)/(Rm*eff)); explicit one-step-lag coupling
        oscillates when several high-current motors share a stiff pack (QuadPlane hover)."""
        b = self.bat
        E = b.ns * b.ocv() - b.vrc
        R0 = b.R0
        act = []
        for mo in self.motors:
            v = pwm[mo.ch]
            thr = 0.0 if (v <= 0 or mo.failed) else min(max((v - mo.pmin) / (mo.pmax - mo.pmin), 0.0), 1.0)
            if thr > 0:
                act.append((thr, mo.omega / mo.kv, mo.rm * mo.esc_eff, mo))
        i_aux = b.aux_w / max(b.V, 1.0)
        V = E - R0 * i_aux
        for _ in range(len(act) + 1):
            A = sum(t * t / r for t, e, r, _m in act)
            B = sum(t * e / r for t, e, r, _m in act)
            V = (E - R0 * i_aux + R0 * B) / (1.0 + R0 * A)
            keep = [x for x in act if x[0] * V - x[1] > 0 or x[3].regen]
            if len(keep) == len(act):
                break
            act = keep
        return V

    # ------------------------------------------------------------------ main step
    def step(self, pwm):
        f = self.fdm
        rho = f["atmosphere/rho-slugs_ft3"] * SLUG / FT ** 3
        u, v, w = f["velocities/u-aero-fps"] * FT, f["velocities/v-aero-fps"] * FT, f["velocities/w-aero-fps"] * FT
        vab = (u, v, w)
        if self.state == "HELD":
            vab = (0.0, 0.0, 0.0)
        Itot = 0.0
        vbus = self.solve_bus_voltage(pwm)
        for mo in self.motors:
            T, Q = mo.step(pwm[mo.ch], vbus, vab, rho, self.dt)
            f[f"external_reactions/{mo.name}/magnitude"] = T / LBF
            f[f"external_reactions/{mo.name}_torque/magnitude-lbsft"] = -mo.spin * Q / LBFFT
            Itot += mo.Ibat
        self.bat.step(Itot, self.dt)
        self.surface_cmds(pwm)
        self.scripted_gusts()
        self.apply_wind()

        if self.state == "HELD":
            self.t += self.dt
            return
        if self.state == "LAUNCH":
            L = self.launch
            if self.t - self.launch_t0 < L.get("duration_s", 0.45):
                f["external_reactions/launch/magnitude"] = self.mass * L.get("accel_mps2", 25) / LBF
                f["external_reactions/hand/magnitude"] = self.mass * G / LBF
            else:
                f["external_reactions/launch/magnitude"] = 0
                f["external_reactions/hand/magnitude"] = 0
                self.state = "FREE"
                self.mark = "released"
        f.run()
        self.t += self.dt

    def held_state(self):
        L = self.launch
        th = math.radians(L.get("hold_pitch_deg", 8)); psi = math.radians(self.home[3])
        C = euler_to_dcm(0, th, psi)
        fb = C.T @ np.array([0, 0, -G])
        return dict(gyro=[0, 0, 0], accel=fb.tolist(), pos=[0, 0, -L.get("hold_height_m", 1.6)],
                    vel=[0, 0, 0], att=[0, th, psi], airspeed=0.0)

    def fdm_state(self):
        f = self.fdm
        m = f["inertia/mass-slugs"]
        fx = f["forces/fbx-total-lbs"] / m * FT
        fy = f["forces/fby-total-lbs"] / m * FT
        fz = f["forces/fbz-total-lbs"] / m * FT
        lat, lon = math.radians(f["position/lat-geod-deg"]), math.radians(f["position/long-gc-deg"])
        R = 6378137.0
        n = (lat - self.lat0) * R
        e = (lon - self.lon0) * R * math.cos(self.lat0)
        d = -(f["position/h-sl-ft"] * FT - self.home[2])
        return dict(gyro=[f["velocities/p-rad_sec"], f["velocities/q-rad_sec"], f["velocities/r-rad_sec"]],
                    accel=[fx, fy, fz], pos=[n, e, d],
                    vel=[f["velocities/v-north-fps"] * FT, f["velocities/v-east-fps"] * FT, f["velocities/v-down-fps"] * FT],
                    att=[f["attitude/phi-rad"], f["attitude/theta-rad"], f["attitude/psi-rad"]],
                    airspeed=max(f["velocities/vc-kts"] * KT, 0.0))

    def reply(self):
        s = self.held_state() if self.state == "HELD" else self.fdm_state()
        w = self.wind_now()
        msg = {"timestamp": self.t,
               "imu": {"gyro": s["gyro"], "accel_body": s["accel"]},
               "position": s["pos"], "attitude": s["att"], "velocity": s["vel"],
               "airspeed": s["airspeed"],
               "velocity_wind": w.tolist(),
               "battery": {"voltage": self.bat.V, "current": self.bat.I}}
        self._last_state = s
        return ("\n" + json.dumps(msg, separators=(",", ":")) + "\n").encode()

    # ------------------------------------------------------------------ truth log
    COLS = ["t", "state", "N", "E", "h_agl", "V_tas", "V_cas", "alpha_deg", "beta_deg", "phi_deg", "theta_deg",
            "psi_deg", "p", "q", "r", "nz", "CL", "CD", "gamma_deg", "Vz_up", "wind_N", "wind_E", "wind_D",
            "throttle", "rpm", "thrust_N", "torque_Nm", "J", "I_motor", "P_shaft_W", "V_bat", "I_bat", "P_elec_W",
            "soc", "Wh", "Ah"] + ["defl_" + s["name"] for s in []] + ["mark"]

    def open_log(self, path):
        self.log = open(path, "w")
        self.cols = self.COLS[:-1] + ["defl_" + s["name"] for s in self.surfaces] + \
            [f"rpm_{m.name}" for m in self.motors[1:]] + ["mark"]
        self.log.write(",".join(self.cols) + "\n")

    def write_log(self):
        if self.log is None or self.t < self.log_next:
            return
        self.log_next = self.t + 1.0 / self.a.log_hz
        f = self.fdm
        s = self._last_state
        held = self.state == "HELD"
        qS = f["aero/qbar-psf"] * f["metrics/Sw-sqft"]
        CL = (f["forces/fwz-aero-lbs"] / qS) if qS > 0.05 and not held else 0.0
        CD = (f["forces/fwx-aero-lbs"] / qS) if qS > 0.05 and not held else 0.0
        w = self.wind_now()
        m0 = self.motors[0]
        row = [self.t, self.state, s["pos"][0], s["pos"][1], -s["pos"][2],
               0 if held else f["velocities/vt-fps"] * FT, s["airspeed"],
               0 if held else f["aero/alpha-deg"], 0 if held else f["aero/beta-deg"],
               math.degrees(s["att"][0]), math.degrees(s["att"][1]), math.degrees(s["att"][2]) % 360,
               s["gyro"][0], s["gyro"][1], s["gyro"][2], -s["accel"][2] / G, CL, CD,
               0 if held else f["flight-path/gamma-deg"], -s["vel"][2], w[0], w[1], w[2],
               m0.thr, m0.rpm, m0.T, m0.Q, m0.Jadv, m0.Im, m0.Pshaft,
               self.bat.V, self.bat.I, self.bat.V * self.bat.I, self.bat.soc, self.bat.Wh, self.bat.Ah]
        row += [math.degrees(f[f"fcs/{x['name']}-pos-rad"]) for x in self.surfaces]
        row += [mo.rpm for mo in self.motors[1:]]
        out = []
        for v in row:
            out.append(v if isinstance(v, str) else f"{v:.8g}")
        out.append(self.mark.replace(",", ";"))
        self.mark = ""
        self.log.write(",".join(out) + "\n")

    # ------------------------------------------------------------------ control port
    def handle_cmd(self, txt):
        p = txt.strip().split()
        if not p:
            return
        c = p[0]
        if c == "launch" and self.state == "HELD":
            self.state = "LAUNCH"
            self.launch_t0 = self.t
            if self.fdm is not None:
                self.init_fdm(self.dt)
            self.mark = "launch"
        elif c == "wind":
            self.set_wind(float(p[1]), float(p[2]), float(p[3]) if len(p) > 3 else 0.0)
            self.mark = txt
        elif c == "turb":
            self.turb = (int(p[1]), float(p[2])); self.mark = txt
        elif c == "gust":
            self.gusts.append((self.t, float(p[4]), [float(p[1]), float(p[2]), float(p[3])])); self.mark = txt
        elif c == "soc":
            self.bat.soc = float(p[1]); self.mark = txt
        elif c == "fail_motor":
            for mo in self.motors:
                if mo.name == p[1]:
                    mo.failed = True
            self.mark = txt
        elif c == "mark":
            self.mark = " ".join(p[1:])
        elif c == "quit":
            raise SystemExit(0)

    # ------------------------------------------------------------------ loop
    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", self.a.port))
        ctl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ctl.bind(("127.0.0.1", self.a.ctl_port))
        ctl.setblocking(False)
        if self.a.log:
            self.open_log(self.a.log)
        print(f"bridge: waiting for SITL on UDP {self.a.port}, control on {self.a.ctl_port}", flush=True)
        nsteps = 0
        wall0 = time.time()
        while True:
            data, addr = sock.recvfrom(1024)
            if len(data) < 8:
                continue
            magic, frame_rate, frame_count = struct.unpack_from("<HHI", data, 0)
            if magic == 18458:
                pwm = struct.unpack_from("<16H", data, 8)
            elif magic == 29569:
                pwm = struct.unpack_from("<32H", data, 8)
            else:
                continue
            if self.fdm is None or frame_count < self.last_frame:
                if self.fdm is not None:
                    print("bridge: SITL restarted, resetting", flush=True)
                    self.__init__(self.a)
                self.init_fdm(1.0 / frame_rate)
            if frame_count == self.last_frame and self.last_reply is not None:
                sock.sendto(self.last_reply, addr)
                continue
            if frame_count != self.last_frame + 1 and self.last_frame >= 0:
                pass  # dropped frames: we just step once (lockstep normally prevents this)
            self.last_frame = frame_count
            try:
                while True:
                    cmd, _ = ctl.recvfrom(1024)
                    self.handle_cmd(cmd.decode())
            except BlockingIOError:
                pass
            self.step(pwm)
            self.last_reply = self.reply()
            sock.sendto(self.last_reply, addr)
            self.write_log()
            nsteps += 1
            if nsteps % (int(frame_rate) * 60) == 0:
                el = time.time() - wall0
                print(f"bridge: t={self.t:.0f}s state={self.state} x{self.t / el:.1f} realtime "
                      f"V={self._last_state['airspeed']:.1f} soc={self.bat.soc:.3f}", flush=True)
                if self.log:
                    self.log.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="model json (generated/<name>/<name>.json)")
    ap.add_argument("--jsbsim-dir", required=True, help="dir containing aircraft/<name>/<name>.xml")
    ap.add_argument("--home", default="-35.363262,149.165237,584,90", help="lat,lon,alt_amsl,heading (must match SITL --home)")
    ap.add_argument("--port", type=int, default=9002)
    ap.add_argument("--ctl-port", type=int, default=9012)
    ap.add_argument("--start", choices=["hand", "ground"], default="hand")
    ap.add_argument("--soc", type=float, default=None)
    ap.add_argument("--wind", default=None, help="speed_mps,from_deg")
    ap.add_argument("--turb", default=None, help="severity(1-7),w20_mps")
    ap.add_argument("--gusts", default=None, help="amp_mps,period_s,duration_s: periodic 1-cos horizontal gusts once airborne")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--temp-c", dest="temp_c", type=float, default=None, help="air temperature at home [C] (ISA + delta-T)")
    ap.add_argument("--log", default=None)
    ap.add_argument("--log-hz", type=float, default=25)
    a = ap.parse_args()
    b = Bridge(a)
    try:
        b.run()
    finally:
        if b.log:
            b.log.close()


if __name__ == "__main__":
    main()
