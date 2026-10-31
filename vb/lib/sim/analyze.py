#!/usr/bin/env python3
"""
analyze.py - metrics + plots for one SITL run directory (or several).

Inputs per run dir: meta.json, events.json, truth.csv (bridge ground truth),
logs/*.BIN (ArduPilot DataFlash).  Output: metrics.json + *.png in the same dir.

  analyze.py results/mission                # one run
  analyze.py --l1-summary results/l1_*      # cross-run summary plot for NAVL1_PERIOD sweep
"""
import argparse, glob, json, math, os, sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pymavlink import mavutil

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import course

# reference palette (dataviz skill, light mode) - fixed categorical order
C = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"
plt.rcParams.update({
    "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "axes.edgecolor": INK2,
    "axes.labelcolor": INK, "xtick.color": INK2, "ytick.color": INK2, "axes.grid": True,
    "grid.color": GRID, "grid.linewidth": 0.8, "lines.linewidth": 1.6, "font.size": 9,
    "axes.spines.top": False, "axes.spines.right": False, "legend.frameon": False,
    "axes.prop_cycle": matplotlib.cycler(color=C),
})
G = 9.80665
HOME = (-35.363262, 149.165237)
USABLE_WH = 144.0 * 0.85      # replaced per run from the model battery


def load_df(path, types):
    mlog = mavutil.mavlink_connection(path)
    data = {t: [] for t in types}
    while True:
        m = mlog.recv_match(type=types)
        if m is None:
            break
        d = m.to_dict()
        data[m.get_type()].append(d)
    out = {}
    for t, rows in data.items():
        df = pd.DataFrame(rows)
        if "TimeUS" in df:
            df["t"] = df["TimeUS"] * 1e-6
        out[t] = df
    return out


def bin_path(run):
    b = sorted(glob.glob(os.path.join(run, "logs", "*.BIN")), key=os.path.getmtime)
    return b[-1] if b else None


def ne_from_ll(lat, lon):
    R = 6378137.0
    return (np.radians(lat - HOME[0]) * R, np.radians(lon - HOME[1]) * R * math.cos(math.radians(HOME[0])))


def ev_time(events, name, default=None):
    for e in events:
        if e["name"] == name:
            return e["t"]
    return default


def save(fig, run, fname):
    fig.tight_layout()
    p = os.path.join(run, fname)
    fig.savefig(p, dpi=130)
    plt.close(fig)
    return p


def time_offset(tr, df):
    """bridge truth time == SITL sim time since process start; DataFlash TimeUS is the same clock."""
    return 0.0


# --------------------------------------------------------------------------------- common
def common_metrics(run, tr, df):
    out = {}
    fl = tr[tr.state != "HELD"]
    if len(fl):
        out["flight_time_s"] = round(float(fl.t.iloc[-1] - fl.t.iloc[0]), 1)
        out["max_alt_agl_m"] = round(float(fl.h_agl.max()), 1)
        out["energy_used_Wh_truth"] = round(float(tr.Wh.iloc[-1]), 2)
        out["charge_used_Ah_truth"] = round(float(tr.Ah.iloc[-1]), 3)
        out["soc_end"] = round(float(tr.soc.iloc[-1]), 3)
        out["min_V_bat"] = round(float(fl.V_bat.min()), 2)
        out["max_I_bat"] = round(float(fl.I_bat.max()), 1)
        out["min_airspeed_inflight"] = round(float(fl[fl.h_agl > 5].V_cas.min()), 2) if (fl.h_agl > 5).any() else None
    if df and "BAT" in df and len(df["BAT"]):
        b = df["BAT"]
        out["ap_BAT_CurrTot_mAh"] = round(float(b.CurrTot.iloc[-1]), 0)
        if "EnrgTot" in b:
            out["ap_BAT_EnrgTot_Wh"] = round(float(b.EnrgTot.iloc[-1]), 2)
    if df and "MODE" in df and len(df["MODE"]):
        out["mode_changes"] = [(round(r.t, 1), r.Mode if "Mode" in r else r.ModeNum, int(r.get("Rsn", -1)))
                               for _, r in df["MODE"].iterrows()]
    return out


def plot_timeseries(run, tr, events, title, fname="timeseries.png", t0=None, t1=None):
    d = tr.copy()
    if t0 is not None:
        d = d[(d.t >= t0) & (d.t <= t1)]
    fig, ax = plt.subplots(5, 1, figsize=(9, 10), sharex=True)
    ax[0].plot(d.t, d.h_agl, color=C[0]); ax[0].set_ylabel("height AGL [m]")
    ax[1].plot(d.t, d.V_cas, color=C[0], label="airspeed (CAS)")
    gs = np.hypot(np.gradient(d.N, d.t), np.gradient(d.E, d.t)) if len(d) > 3 else d.V_cas
    ax[1].plot(d.t, gs, color=C[1], label="ground speed", lw=1.0)
    ax[1].set_ylabel("speed [m/s]"); ax[1].legend(loc="upper right")
    ax[2].plot(d.t, d.phi_deg, color=C[0], label="roll"); ax[2].plot(d.t, d.theta_deg, color=C[1], label="pitch")
    ax[2].plot(d.t, d.alpha_deg.where(d.V_tas > 3.0), color=C[2], label="alpha (V > 3 m/s)", lw=1.0)
    ax[2].set_ylabel("angle [deg]"); ax[2].legend(loc="upper right", ncol=3)
    ax[3].plot(d.t, d.throttle * 100, color=C[0]); ax[3].set_ylabel("throttle [%]")
    ax[4].plot(d.t, d.P_elec_W, color=C[0]); ax[4].set_ylabel("battery power [W]"); ax[4].set_xlabel("sim time [s]")
    for a in ax:
        for e in events:
            if e["name"] in ("hand_launch", "course_start", "course_end", "rtl", "rc_lost", "gcs_lost",
                             "batt_fs_text", "stall_entry", "speed"):
                if t0 is None or t0 <= e["t"] <= t1:
                    a.axvline(e["t"], color=INK2, lw=0.6, ls=":")
    ax[0].set_title(title, loc="left", color=INK)
    return save(fig, run, fname)


# --------------------------------------------------------------------------------- mission
def analyze_mission(run, meta, tr, df, events):
    res = {}
    cl = course.centreline()
    t0 = ev_time(events, "course_start"); t1 = ev_time(events, "course_end")
    if t0 is None or t1 is None:
        return {"error": "course not completed"}
    seg = tr[(tr.t >= t0) & (tr.t <= t1)]
    xt0, _, _ = course.cross_track(cl, seg.N.values, seg.E.values)
    # tracking metrics start once the aircraft has joined the course (first |XT| < 5 m): the entry from the climb
    # loiter is reported separately, it is not path-tracking performance
    on = np.where(xt0 < 5.0)[0]
    j0 = int(on[0]) if len(on) else 0
    res["course_entry_s"] = round(float(seg.t.iloc[j0] - seg.t.iloc[0]), 1)
    res["course_entry_xt_max_m"] = round(float(xt0[:j0 + 1].max()), 1)
    t0 = float(seg.t.iloc[j0])
    seg = seg.iloc[j0:]
    xt, R, idx = course.cross_track(cl, seg.N.values, seg.E.values)
    res["xt_truth_rms_m"] = round(float(np.sqrt(np.mean(xt ** 2))), 2)
    res["xt_truth_p95_m"] = round(float(np.percentile(xt, 95)), 2)
    res["xt_truth_max_m"] = round(float(xt.max()), 2)
    by_r = {}
    for r in sorted(set(R)):
        m = R == r
        key = "straight" if r == 0 else f"R{int(r)}"
        by_r[key] = dict(rms=round(float(np.sqrt(np.mean(xt[m] ** 2))), 2), max=round(float(xt[m].max()), 2),
                         n=int(m.sum()))
    res["xt_truth_by_bend"] = by_r
    # roll weave (review M10). High-pass the roll angle (minus a 4 s moving mean, which keeps the steady bank of the
    # bends); (1) whole-course weave = half peak-to-peak (2.5..97.5 %) of the high-passed roll; (2) the part at the
    # waypoint-passing frequency f_wp = V_ground / spacing (FFT band 0.75..1.25 f_wp, inverse FFT, half p2p).
    try:
        dt_ = float(np.median(np.diff(seg.t)))
        phi = seg.phi_deg.values
        k = max(int(round(4.0 / dt_)), 3)
        hp = phi - np.convolve(phi, np.ones(k) / k, mode="same")
        hp = hp[k:-k] - hp[k:-k].mean()
        gs = float(np.mean(np.hypot(np.gradient(seg.N.values, seg.t.values), np.gradient(seg.E.values, seg.t.values))))
        f_wp = gs / float(meta.get("spacing", 25.0))
        X = np.fft.rfft(hp)
        fr = np.fft.rfftfreq(len(hp), dt_)
        band = (fr >= 0.75 * f_wp) & (fr <= 1.25 * f_wp)
        xb = np.fft.irfft(np.where(band, X, 0), n=len(hp))
        Fm = np.abs(X)
        sel = (fr > 0.08) & (fr < 3.0)
        res["roll_weave_amp_deg"] = round(float((np.percentile(hp, 97.5) - np.percentile(hp, 2.5)) / 2), 2)
        res["roll_weave_wp_amp_deg"] = round(float((np.percentile(xb, 97.5) - np.percentile(xb, 2.5)) / 2), 2)
        res["roll_weave_freq_Hz"] = round(float(fr[sel][np.argmax(Fm[sel])]), 3)
        res["wp_passing_freq_Hz"] = round(f_wp, 3)
        res["wp_spacing_m"] = float(meta.get("spacing", 25.0))
    except Exception as ex:  # metric missing -> ERROR gate in the stage
        res["roll_weave_error"] = str(ex)
    # course distance & energy
    dist = float(np.sum(np.hypot(np.diff(seg.N), np.diff(seg.E))))
    dWh = float(seg.Wh.iloc[-1] - seg.Wh.iloc[0])
    res["course_ground_dist_m"] = round(dist, 0)
    res["course_time_s"] = round(float(seg.t.iloc[-1] - seg.t.iloc[0]), 1)
    res["course_energy_Wh"] = round(dWh, 2)
    res["course_Wh_per_km"] = round(dWh / (dist / 1000), 3)
    res["course_mean_power_W"] = round(float(seg.P_elec_W.mean()), 1)
    res["course_mean_cas"] = round(float(seg.V_cas.mean()), 2)
    res["course_max_bank_deg"] = round(float(seg.phi_deg.abs().max()), 1)
    res["course_alt_std_m"] = round(float(seg.h_agl.std()), 2)
    res["course_alt_min_max"] = [round(float(seg.h_agl.min()), 1), round(float(seg.h_agl.max()), 1)]
    cam = None
    if df:
        nt = df.get("NTUN")
        if nt is not None and len(nt):
            n2 = nt[(nt.t >= t0) & (nt.t <= t1)]
            res["ntun_xt_rms_m"] = round(float(np.sqrt(np.mean(n2.XT ** 2))), 2)
            res["ntun_xt_max_m"] = round(float(n2.XT.abs().max()), 2)
        for camtype in ("CAM", "TRIG"):
            c = df.get(camtype)
            if c is not None and len(c):
                cam = c
                break
        if cam is not None:
            cn, ce = ne_from_ll(cam.Lat.values, cam.Lng.values)
            sp = np.hypot(np.diff(cn), np.diff(ce))
            res["camera_triggers"] = int(len(cam))
            if len(sp):
                res["camera_spacing_m_mean_min_max"] = [round(float(sp.mean()), 1), round(float(sp.min()), 1),
                                                        round(float(sp.max()), 1)]
    # --- plots
    fig, ax = plt.subplots(1, 2, figsize=(11, 6), gridspec_kw={"width_ratios": [1.1, 1]})
    a = ax[0]
    a.plot(cl[:, 1], cl[:, 0], color=GRID, lw=6, label="canal centreline", solid_capstyle="round")
    fl = tr[tr.state != "HELD"]
    a.plot(fl.E, fl.N, color=C[0], lw=1.2, label="flown track (truth)")
    wps, _ = course.waypoints(cl, meta.get("spacing", 25))
    a.plot(wps[:, 1], wps[:, 0], "o", ms=3, color=C[1], label="waypoints")
    if cam is not None:
        a.plot(ce, cn, "^", ms=3.5, color=C[2], label="camera triggers")
    a.plot(0, 0, "s", color=INK, ms=5); a.annotate("home / launch", (0, 0), (10, 2), textcoords="offset points", color=INK2, fontsize=8)
    a.set_aspect("equal"); a.set_xlabel("East [m]"); a.set_ylabel("North [m]")
    a.legend(loc="upper center", bbox_to_anchor=(0.5, -0.09), ncol=2, fontsize=7.5)
    a.set_title(f"{os.path.basename(run)} - ground track", loc="left")
    b = ax[1]
    along = idx  # metres along centreline (ds = 1 m)
    b.plot(along, xt, color=C[0], lw=1.0, label="|XT| to canal centreline (truth)")
    for r in sorted(set(cl[:, 2])):
        if r == 0:
            continue
        m = cl[:, 2] == r
        ii = np.where(m)[0]
        # shade contiguous arcs
        splits = np.split(ii, np.where(np.diff(ii) > 1)[0] + 1)
        for k, sgm in enumerate(splits):
            b.axvspan(sgm[0], sgm[-1], color=GRID, alpha=0.6, lw=0)
            b.text((sgm[0] + sgm[-1]) / 2, b.get_ylim()[1] * 0.0 + max(xt) * 1.02, f"R{int(r)}", ha="center",
                   color=INK2, fontsize=8)
    b.set_xlabel("distance along canal [m]"); b.set_ylabel("cross-track error [m]")
    b.set_title(f"XT rms {res['xt_truth_rms_m']} m, max {res['xt_truth_max_m']} m (after joining; entry {res['course_entry_s']} s, "
                f"max {res['course_entry_xt_max_m']} m)", loc="left", fontsize=9)
    save(fig, run, "track_xt.png")
    plot_timeseries(run, tr, events, f"{os.path.basename(run)} - flight overview")
    return res


# --------------------------------------------------------------------------------- cruise
def analyze_cruise(run, meta, tr, df, events, model=None):
    rows = []
    mass = model["mass"]["mass_kg"] if model else 1.775
    for sgm in meta.get("segments", []):
        d = tr[(tr.t >= sgm["t_start"]) & (tr.t <= sgm["t_end"])]
        if len(d) < 10:
            continue
        P = float(d.P_elec_W.mean()); V = float(d.V_tas.mean()); Vz = float(np.polyfit(d.t, d.h_agl, 1)[0])
        Psh = float(d.P_shaft_W.mean()); T = float(d.thrust_N.mean())
        eta_tot = max(T * V / P, 0.05)
        Pcorr = P - mass * G * Vz / eta_tot     # remove energy going into climb/descent
        rows.append(dict(v_cmd=sgm["v"], V_tas=round(V, 2), V_cas=round(float(d.V_cas.mean()), 2),
                         P_elec_W=round(P, 2), P_level_W=round(Pcorr, 2), I_A=round(float(d.I_bat.mean()), 2),
                         V_bat=round(float(d.V_bat.mean()), 2), throttle=round(float(d.throttle.mean()), 3),
                         rpm=round(float(d.rpm.mean()), 0), thrust_N=round(T, 3), alpha_deg=round(float(d.alpha_deg.mean()), 2),
                         Vz=round(Vz, 3), eta_total=round(eta_tot, 3), eta_prop=round(T * V / max(Psh, 1e-3), 3),
                         Wh_per_km=round(Pcorr / (V * 3.6), 3)))
    if not rows:
        return {"error": "no segments"}
    usable = USABLE_WH
    for r in rows:
        r["range_km_usable"] = round(usable / r["Wh_per_km"], 1)
        r["endurance_h_usable"] = round(usable / r["P_level_W"], 2)
    best_r = min(rows, key=lambda r: r["Wh_per_km"]); best_e = min(rows, key=lambda r: r["P_level_W"])
    res = dict(table=rows, usable_energy_Wh=usable, best_range=best_r, best_endurance=best_e)
    v = [r["V_tas"] for r in rows]
    fig, ax = plt.subplots(1, 3, figsize=(12, 3.8))
    ax[0].plot(v, [r["P_level_W"] for r in rows], "o-", color=C[0]); ax[0].set_ylabel("battery power, level [W]")
    ax[1].plot(v, [r["Wh_per_km"] for r in rows], "o-", color=C[0]); ax[1].set_ylabel("energy per km [Wh/km]")
    ax[2].plot(v, [r["range_km_usable"] for r in rows], "o-", color=C[0]); ax[2].set_ylabel(f"range on {usable:.0f} Wh usable [km]")
    ax[1].annotate(f"best range {best_r['V_tas']} m/s", (best_r["V_tas"], best_r["Wh_per_km"]), (5, 10),
                   textcoords="offset points", color=INK2)
    ax[0].annotate(f"min power {best_e['V_tas']} m/s", (best_e["V_tas"], best_e["P_level_W"]), (5, 10),
                   textcoords="offset points", color=INK2)
    for a in ax:
        a.set_xlabel("true airspeed [m/s]")
    ax[0].set_title("Cruise sweep (SITL + JSBSim, still air)", loc="left")
    save(fig, run, "cruise_power.png")
    plot_timeseries(run, tr, events, f"{os.path.basename(run)} - overview")
    return res


# --------------------------------------------------------------------------------- stall
def analyze_stall(run, meta, tr, df, events, model):
    W = model["mass"]["mass_kg"] * G
    S = model["geometry"]["wing_area_m2"]
    out = []
    for k, st in enumerate(meta.get("stalls", [])):
        d = tr[(tr.t >= st["t0"]) & (tr.t <= st["t1"])]      # stick-hold window only: recovery pull-ups come later
        if len(d) < 10:
            continue
        cl = d.CL.values
        # FIRST break: first local maximum above 85 % of the window maximum (later peaks are recovery pull-ups)
        thr = 0.85 * cl.max()
        i = int(np.argmax(cl))
        for j in range(1, len(cl) - 1):
            if cl[j] >= thr and cl[j] >= cl[j - 1] and cl[j] >= cl[j + 1] and cl[j] >= cl[min(j + 25, len(cl) - 1)]:
                i = j
                break
        r = d.iloc[i]
        nz = max(float(r.nz), 0.2)
        rho = 1.225 * (r.V_cas / max(r.V_tas, 0.1)) ** 2 * (1.0)  # CAS-based -> use sea-level
        v_s1g = float(r.V_cas) / math.sqrt(nz)
        dd = tr[(tr.t >= st["t0"]) & (tr.t <= st["t1"] + 15)]       # include the recovery
        after = dd[dd.t >= r.t]
        # post-stall behaviour (review M3): wing drop, pitch break, locked-in stall, recovery after stick release
        tb, a_b, phi_b, th_b = float(r.t), float(r.alpha_deg), float(r.phi_deg), float(r.theta_deg)
        w5 = tr[(tr.t >= tb) & (tr.t <= tb + 5)]
        wing_drop = float((w5.phi_deg - phi_b).abs().max()) if len(w5) else None
        held = tr[(tr.t >= tb) & (tr.t <= st["t1"])]
        dt_ = float(np.median(np.diff(tr.t))) if len(tr) > 2 else 0.04
        locked_s = float((held.alpha_deg > a_b + 3).sum() * dt_) if len(held) else 0.0
        w3 = tr[(tr.t >= tb) & (tr.t <= tb + 3)]
        nose_drop = float(th_b - w3.theta_deg.min()) if len(w3) else None
        rec = tr[(tr.t > st["t1"]) & (tr.t <= st["t1"] + 15) & (tr.alpha_deg < a_b - 2) & (tr.phi_deg.abs() < 30) & (tr.Vz_up > -2.0)]
        t_rec = float(rec.t.iloc[0]) if len(rec) else None
        win = tr[(tr.t >= tb) & (tr.t <= (t_rec if t_rec is not None else st["t1"] + 15))]
        alt_loss_rec = float(r.h_agl - win.h_agl.min()) if len(win) else None
        wr = tr[(tr.t >= st["t1"]) & (tr.t <= (t_rec if t_rec is not None else st["t1"] + 15) + 2)]
        h_rel = float(np.interp(st["t1"], tr.t, tr.h_agl))
        alt_loss_release = float(h_rel - wr.h_agl.min()) if len(wr) else None
        out.append(dict(rep=k, t_stall=round(tb, 2), CL_max_seen=round(float(r.CL), 3),
                        alpha_at_CLmax_deg=round(a_b, 2), V_cas_at_CLmax=round(float(r.V_cas), 2),
                        nz=round(nz, 2), V_stall_1g_est=round(v_s1g, 2),
                        min_cas=round(float(dd.V_cas.min()), 2), max_alpha_deg=round(float(dd.alpha_deg.max()), 1),
                        alt_loss_m=round(float(dd.h_agl.max() - after.h_agl.min()), 1),
                        max_abs_roll_deg=round(float(dd.phi_deg.abs().max()), 1),
                        wing_drop_deg=None if wing_drop is None else round(wing_drop, 1),
                        nose_drop_deg=None if nose_drop is None else round(nose_drop, 1),
                        locked_in_s=round(locked_s, 1), t_stick_release=round(float(st["t1"]), 2),
                        recovery_s=None if t_rec is None else round(t_rec - float(st["t1"]), 1),
                        alt_loss_break_to_recovery_m=None if alt_loss_rec is None else round(alt_loss_rec, 1),
                        alt_loss_release_to_recovery_m=None if alt_loss_release is None else round(alt_loss_release, 1)))
    res = {"stalls": out}
    if out:
        res["V_stall_1g_mean"] = round(float(np.mean([o["V_stall_1g_est"] for o in out])), 2)
    res["V_stall_analytic_CLmax"] = round(math.sqrt(2 * W / (1.225 * S * model["aero"]["derivatives"]["CLmax"])), 2)
    if df and "AOA" in df and len(df["AOA"]):
        res["ap_AOA_max_deg"] = round(float(df["AOA"].AOA.max()), 1)
    for k, st in enumerate(meta.get("stalls", [])):
        d = tr[(tr.t >= st["t0"] - 5) & (tr.t <= st["t1"] + 10)]
        fig, ax = plt.subplots(4, 1, figsize=(8, 8), sharex=True)
        ax[0].plot(d.t, d.V_cas, color=C[0]); ax[0].set_ylabel("airspeed CAS [m/s]")
        ax[1].plot(d.t, d.alpha_deg, color=C[0], label="alpha"); ax[1].plot(d.t, d.theta_deg, color=C[1], label="pitch")
        ax[1].plot(d.t, d.phi_deg, color=C[2], label="roll"); ax[1].legend(ncol=3, loc="upper left"); ax[1].set_ylabel("[deg]")
        ax[2].plot(d.t, d.CL, color=C[0]); ax[2].set_ylabel("CL (truth)")
        ax[3].plot(d.t, d.h_agl, color=C[0]); ax[3].set_ylabel("height AGL [m]"); ax[3].set_xlabel("sim time [s]")
        if k < len(out):
            for a in ax:
                a.axvline(out[k]["t_stall"], color=INK2, lw=0.8, ls=":")
            ax[0].set_title(f"Stall {k} (L/R CLmax asymmetry {'on' if meta.get('stall_asymmetry') else 'off'}): CLmax {out[k]['CL_max_seen']} "
                            f"at {out[k]['V_cas_at_CLmax']} m/s; wing drop {out[k]['wing_drop_deg']} deg\nrecovery {out[k]['recovery_s']} s after "
                            f"release; height lost after release {out[k]['alt_loss_release_to_recovery_m']} m (break->recovery "
                            f"{out[k]['alt_loss_break_to_recovery_m']} m, stick held ~7 s)", loc="left", fontsize=8.5)
            for a in ax:
                a.axvline(meta["stalls"][k]["t1"], color=C[3], lw=0.8, ls="--")
            ax[3].text(meta["stalls"][k]["t1"], ax[3].get_ylim()[0], " stick released", color=C[3], fontsize=8, va="bottom")
        save(fig, run, f"stall_{k}.png")
    return res


# --------------------------------------------------------------------------------- launch
def analyze_launch(run, meta, tr, df, events):
    tl = ev_time(events, "hand_launch")
    rel = tr[tr["mark"].astype(str).str.contains("released", na=False)]
    res = {}
    if len(rel):
        r = rel.iloc[0]
        res["release_t"] = round(float(r.t), 2); res["release_tas"] = round(float(r.V_tas), 2)
        res["release_height_m"] = round(float(r.h_agl), 2)
    st = tr[tr["mark"].astype(str).str.contains("launch", na=False)]
    t_launch = float(st.t.iloc[0]) if len(st) else tl
    thr = tr[(tr.t > t_launch) & (tr.throttle > 0.05)]
    if len(thr):
        res["motor_start_delay_s"] = round(float(thr.t.iloc[0] - t_launch), 2)
        res["motor_start_height_m"] = round(float(thr.h_agl.iloc[0]), 2)
    post = tr[(tr.t > t_launch) & (tr.t < t_launch + 15)]
    res["min_height_after_release_m"] = round(float(post[post.t > t_launch + 0.5].h_agl.min()), 2)
    res["min_cas_after_release"] = round(float(post[post.t > t_launch + 0.5].V_cas.min()), 2)
    res["max_alpha_deg_first5s"] = round(float(post[post.t < t_launch + 5].alpha_deg.max()), 1)
    res["height_at_10s_m"] = round(float(np.interp(t_launch + 10, tr.t, tr.h_agl)), 1)
    res["crashed"] = bool((post.h_agl < 0.2).any())
    d = tr[(tr.t > t_launch - 1) & (tr.t < t_launch + 15)]
    fig, ax = plt.subplots(4, 1, figsize=(8, 7.5), sharex=True)
    tt = d.t - t_launch
    ax[0].plot(tt, d.h_agl, color=C[0]); ax[0].set_ylabel("height [m]")
    ax[1].plot(tt, d.V_tas, color=C[0]); ax[1].set_ylabel("TAS [m/s]")
    ax[2].plot(tt, d.throttle * 100, color=C[0]); ax[2].set_ylabel("throttle [%]")
    ax[3].plot(tt, d.alpha_deg.where(d.V_tas > 3.0), color=C[0], label="alpha (V > 3 m/s)"); ax[3].plot(tt, d.theta_deg, color=C[1], label="pitch")
    ax[3].legend(ncol=2, loc="upper right"); ax[3].set_ylabel("[deg]"); ax[3].set_xlabel("time since throw start [s]")
    ax[0].set_title(f"{os.path.basename(run)}: {meta.get('accel')} m/s^2 x {meta.get('dur')} s -> release {res.get('release_tas')} m/s, "
                    f"motor +{res.get('motor_start_delay_s')} s\nmodel {meta.get('model', '?')}; {str(meta.get('env_label') or 'calm')[:95]}",
                    loc="left", fontsize=8.5)
    ax[3].plot([], [])
    save(fig, run, "launch.png")
    return res


# --------------------------------------------------------------------------------- failsafe
def analyze_fs(run, meta, tr, df, events):
    res = {}
    trig = None
    for e in events:
        if e["name"] in ("rc_lost", "gcs_lost", "batt_fs_text"):
            trig = e
            break
    if df and "MODE" in df and trig is not None:
        md = df["MODE"]
        after = md[md.t >= trig["t"] - (5 if trig["name"] == "batt_fs_text" else 0)]
        if len(after):
            r = after.iloc[0]
            res["first_mode_change_after_trigger"] = dict(t=round(float(r.t), 1), dt_s=round(float(r.t - trig["t"]), 1),
                                                          mode=int(r.ModeNum), reason=int(r.get("Rsn", -1)))
    res["trigger"] = trig
    st = json.load(open(os.path.join(run, "events.json")))["statustext"]
    if trig:
        res["statustext_after_trigger"] = [(round(t, 1), x) for t, x in st if t >= trig["t"] - 30 and
                                           any(k in x for k in ("ailsafe", "atter", "RTL", "Mode", "GCS", "RC", "Low"))][:12]
    plot_timeseries(run, tr, events, f"{os.path.basename(run)} - failsafe")
    fl = tr[tr.state != "HELD"]
    fig, a = plt.subplots(figsize=(6, 6))
    cl = course.centreline()
    a.plot(cl[:, 1], cl[:, 0], color=GRID, lw=6, label="canal")
    a.plot(fl.E, fl.N, color=C[0], lw=1.2, label="track")
    if trig:
        p = tr.iloc[(tr.t - trig["t"]).abs().argmin()]
        a.plot(p.E, p.N, "X", color=C[7], ms=9, label=trig["name"])
    a.set_aspect("equal"); a.legend(loc="upper center", bbox_to_anchor=(0.5, -0.09), ncol=3); a.set_xlabel("East [m]"); a.set_ylabel("North [m]")
    save(fig, run, "fs_track.png")
    return res


def analyze_vtol(run, meta, tr, df, events):
    res = {}
    lift_cols = [c for c in tr.columns if c.startswith("rpm_lift")]
    tr = tr.copy()
    tr["lift_rpm_mean"] = tr[lift_cols].mean(axis=1) if lift_cols else 0
    hov = tr[(tr.lift_rpm_mean > 3000) & (tr.V_tas < 3) & (tr.h_agl > 5)]
    # VTOL phases (lift motors turning): energy and time = what the mission model must budget (review M7)
    vt = tr[tr.lift_rpm_mean > 500]
    if len(vt) > 2:
        dt = np.diff(tr.t.values, prepend=tr.t.values[0])
        m_ = (tr.lift_rpm_mean > 500).values
        res["vtol_phase_time_s"] = round(float(dt[m_].sum()), 1)
        res["vtol_phase_energy_Wh"] = round(float((tr.P_elec_W.values * dt)[m_].sum() / 3600), 2)
    if len(hov):
        res["hover_power_W"] = round(float(hov.P_elec_W.median()), 1)
        res["hover_current_A"] = round(float(hov.I_bat.median()), 1)
        res["hover_lift_rpm"] = round(float(hov.lift_rpm_mean.median()), 0)
        res["hover_endurance_min_on_usable"] = round(USABLE_WH / res["hover_power_W"] * 60, 1)
        res["hover_throttle_lift"] = round(float(hov[[c for c in tr.columns if c.startswith("thr_lift")]].mean(axis=1).median()), 3) \
            if any(c.startswith("thr_lift") for c in tr.columns) else None      # = ESC duty (PWM fraction), like propulsion
        res["hover_V_bat"] = round(float(hov.V_bat.median()), 2)
        if df and "QTUN" in df and len(df["QTUN"]):
            q = df["QTUN"]
            res["ap_QTUN_ThO_median"] = round(float(q.ThO.median()), 3)
    fw = tr[(tr.lift_rpm_mean < 500) & (tr.h_agl > 20)]
    if len(fw):
        res["fw_cruise_power_W"] = round(float(fw.P_elec_W.median()), 1)
        res["fw_cruise_cas"] = round(float(fw.V_cas.median()), 1)
    if df and "MODE" in df:
        pass
    landed = tr[tr.h_agl < 0.3]
    if len(landed):
        l = landed.iloc[-1]
        res["landing_pos_error_m"] = round(float(math.hypot(l.N, l.E)), 2)
    res["energy_Wh"] = round(float(tr.Wh.iloc[-1]), 2)
    fig, ax = plt.subplots(4, 1, figsize=(9, 8), sharex=True)
    ax[0].plot(tr.t, tr.h_agl, color=C[0]); ax[0].set_ylabel("height AGL [m]")
    ax[1].plot(tr.t, tr.V_cas, color=C[0]); ax[1].set_ylabel("airspeed [m/s]")
    ax[2].plot(tr.t, tr.lift_rpm_mean, color=C[0], label="lift motors (mean)")
    ax[2].plot(tr.t, tr.rpm, color=C[1], label="pusher"); ax[2].set_ylabel("rpm"); ax[2].legend(loc="upper left", ncol=2)
    ax[3].plot(tr.t, tr.P_elec_W, color=C[0]); ax[3].set_ylabel("battery power [W]"); ax[3].set_xlabel("sim time [s]")
    ax[0].set_title(f"{os.path.basename(run)}: QuadPlane VTOL mission (JSBSim bridge, 5 motors)\n{str(meta.get('env_label') or 'calm')[:110]}",
                    loc="left", fontsize=9)
    save(fig, run, "vtol.png")
    return res


def envelope_metrics(tr, alpha_stall_deg=None):
    """max |nz|, max airspeed, throttle saturation and time above the stall alpha in flight (truth log)."""
    out = {}
    fl = tr[(tr.state != "HELD") & (tr.h_agl > 5)] if "state" in tr else tr
    if len(fl):
        if alpha_stall_deg is not None:
            dts = float(np.median(np.diff(fl.t))) if len(fl) > 2 else 0.04
            # wing-borne flight only: with the lift motors turning (VTOL hover/transition) a high wing alpha is not a stall hazard
            lc = [c for c in fl.columns if c.startswith("rpm_lift")]
            wing = (fl[lc].mean(axis=1) <= 500) if lc else pd.Series(True, index=fl.index)
            out["stall_time_s"] = round(float(((fl.alpha_deg > alpha_stall_deg) & (fl.V_tas > 3) & wing).sum() * dts), 2)
            out["alpha_stall_model_deg"] = alpha_stall_deg
        out["max_nz"] = round(float(fl.nz.max()), 2)
        out["min_nz"] = round(float(fl.nz.min()), 2)
        out["max_cas"] = round(float(fl.V_cas.max()), 2)
        out["max_alpha_deg"] = round(float(fl.alpha_deg.max()), 1)
        dt = float(np.median(np.diff(fl.t))) if len(fl) > 2 else 0.04
        sat = (fl.throttle >= 0.99).values
        run_, best = 0, 0
        for v in sat:
            run_ = run_ + 1 if v else 0
            best = max(best, run_)
        out["max_throttle_saturation_s"] = round(best * dt, 1)
    return out


def truth_from_bin(bp):
    """pseudo-truth from DataFlash for back-ends without a bridge (SIM_Plane fallback). Uses SITL
    'SIM' message (simulator truth) when available, else EKF POS."""
    d = load_df(bp, ["SIM", "POS", "ATT", "ARSP", "BAT", "CTUN"])
    pos = d["POS"]
    t = pos.t.values
    n, e = ne_from_ll(pos.Lat.values, pos.Lng.values)
    out = pd.DataFrame({"t": t, "N": n, "E": e, "h_agl": pos.RelHomeAlt.values})
    out["state"] = "FREE"
    def put(col, df, field, scale=1.0):
        out[col] = np.interp(t, df.t.values, df[field].values * scale) if len(df) else 0.0
    put("phi_deg", d["ATT"], "Roll"); put("theta_deg", d["ATT"], "Pitch"); put("psi_deg", d["ATT"], "Yaw")
    put("V_cas", d["ARSP"], "Airspeed"); out["V_tas"] = out["V_cas"]
    put("V_bat", d["BAT"], "Volt"); put("I_bat", d["BAT"], "Curr"); put("Wh", d["BAT"], "EnrgTot")
    put("Ah", d["BAT"], "CurrTot", 1e-3)
    out["P_elec_W"] = out.V_bat * out.I_bat
    put("throttle", d["CTUN"], "ThO", 0.01)
    for c in ["alpha_deg", "beta_deg", "CL", "CD", "nz", "rpm", "thrust_N", "P_shaft_W", "soc"]:
        out[c] = 0.0
    out["mark"] = ""
    return out


def analyze_run(run):
    run = os.path.abspath(run)
    meta = json.load(open(os.path.join(run, "meta.json")))
    events = json.load(open(os.path.join(run, "events.json")))["events"]
    bp = bin_path(run)
    tpath = os.path.join(run, "truth.csv")
    tr = pd.read_csv(tpath) if os.path.exists(tpath) else truth_from_bin(bp)
    fl = tr[(tr.state != "HELD") & (tr.h_agl > 10)] if "state" in tr else tr
    if len(fl) and fl.CL.median() < 0:   # logs written before the JSBSim wind-axis sign fix
        tr["CL"] = -tr["CL"]; tr["CD"] = -tr["CD"]
    mpath = meta.get("model_json") or os.path.join(os.path.dirname(__file__), "..", "generated", meta.get("model", "m"),
                                                    meta.get("model", "m") + ".json")
    model = json.load(open(mpath))
    global USABLE_WH
    b = model["battery"]
    USABLE_WH = b["cells_series"] * b["cells_parallel"] * b["cell_capacity_ah"] * 3.6 * 0.85
    df = load_df(bp, ["NTUN", "BAT", "MODE", "CAM", "TRIG", "CTUN", "AOA", "ARSP", "QTUN"]) if bp else None
    res = {"run": os.path.basename(run), "kind": meta["kind"], "bin": bp, "sim_s": meta.get("sim_s"),
           "wall_s": round(meta.get("wall_s", 0), 1)}
    res["speed_x_realtime"] = round(meta.get("sim_s", 0) / max(meta.get("wall_s", 1), 1), 1)
    res.update(common_metrics(run, tr, df))
    k = meta["kind"]
    if k == "mission":
        res.update(analyze_mission(run, meta, tr, df, events))
    elif k == "cruise":
        res.update(analyze_cruise(run, meta, tr, df, events, model))
    elif k == "stall":
        res.update(analyze_stall(run, meta, tr, df, events, model))
    elif k == "launch":
        res.update(analyze_launch(run, meta, tr, df, events))
    elif k.startswith("fs_"):
        res.update(analyze_fs(run, meta, tr, df, events))
    elif k == "vtol":
        res.update(analyze_vtol(run, meta, tr, df, events))
    elif k == "smoke":
        plot_timeseries(run, tr, events, f"{os.path.basename(run)} - smoke flight")
    res.update(envelope_metrics(tr, (model.get("aero", {}).get("derivatives") or {}).get("alpha_stall_deg")))
    if "error" in meta:
        res["run_error"] = meta["error"]
    json.dump(res, open(os.path.join(run, "metrics.json"), "w"), indent=1, default=str)
    return res


def l1_summary(runs, out):
    rows = []
    for r in runs:
        m = json.load(open(os.path.join(r, "metrics.json")))
        meta = json.load(open(os.path.join(r, "meta.json")))
        p = meta.get("params", {}).get("NAVL1_PERIOD")
        if p is None or "xt_truth_by_bend" not in m:
            continue
        rows.append((p, m))
    rows.sort(key=lambda x: x[0])
    keys = ["straight", "R300", "R150", "R80", "R50"]
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    P = [p for p, _ in rows]
    for i, k in enumerate(keys):
        ax[0].plot(P, [m["xt_truth_by_bend"].get(k, {}).get("rms", np.nan) for _, m in rows], "o-", color=C[i], label=k)
        ax[1].plot(P, [m["xt_truth_by_bend"].get(k, {}).get("max", np.nan) for _, m in rows], "o-", color=C[i], label=k)
    ax[0].set_ylabel("RMS cross-track [m]"); ax[1].set_ylabel("max cross-track [m]")
    for a in ax:
        a.set_ylim(0, a.get_ylim()[1] * 1.25)
    for a in ax:
        a.set_xlabel("NAVL1_PERIOD [s]"); a.legend(ncol=5, loc="upper left", fontsize=8)
    ax[0].set_title("Path tracking vs L1 period, by canal bend radius (truth vs centreline)", loc="left")
    fig.tight_layout(); fig.savefig(out, dpi=130); plt.close(fig)
    summ = [{"NAVL1_PERIOD": p, "xt_rms": m["xt_truth_rms_m"], "xt_max": m["xt_truth_max_m"],
             "by_bend": m["xt_truth_by_bend"], "Wh_per_km": m.get("course_Wh_per_km"),
             "ntun_xt_rms": m.get("ntun_xt_rms_m")} for p, m in rows]
    json.dump(summ, open(out.replace(".png", ".json"), "w"), indent=1)
    return summ


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--l1-summary", action="store_true")
    a = ap.parse_args()
    if a.l1_summary:
        print(json.dumps(l1_summary(a.runs, os.path.join(os.path.dirname(a.runs[0]), "l1_sweep_summary.png")), indent=1))
    else:
        for r in a.runs:
            print(json.dumps(analyze_run(r), indent=1, default=str))
