"""Stage 9 - report: report.md + self-contained index.html (embedded PNGs), GO/NO-GO table, soft inputs,
configuration, per-stage gates and figures, risk register CSV, build checklist, derived.param,
calibration-needed list. Never prints GO with an unresolved FAIL, ERROR or a stage that did not run."""
import base64
import csv
import html
import io
import json
import math
import re
import shutil
import subprocess
import time
from pathlib import Path

from .. import env, plan as P
from ..stages import NAMES

ROOT = Path(__file__).resolve().parents[2]
TITLES = {"ingest": "0 Ingest (plan validation)", "geometry": "1 Geometry & renders", "mass": "2 Mass, CG, inertia",
          "clash": "3 Fit & clash", "aero_lowfi": "6a Aerodynamics (low order)", "propulsion": "5 Propulsion & electrical",
          "structures": "4 Structures & build risk", "cfd": "6b CFD", "flightmodel": "7 Flight model & ArduPilot params",
          "sitl": "8 SITL test matrix"}
SEVERITY = {"structures": 5, "sitl": 5, "propulsion": 4, "aero_lowfi": 4, "mass": 4, "clash": 3, "flightmodel": 4, "ingest": 3,
            "geometry": 2, "cfd": 2}
VERIFY = {"mass": "C0 weigh every part + balance-point check", "geometry": "compare against the CAD/STL pack (cad_import)",
          "clash": "dry-fit the printed parts and components", "propulsion": "C2 thrust stand (thrust/current/rpm at 3 voltages), C3 discharge test",
          "structures": "C4 sandbag proof load to n_limit + tap test (phone accelerometer)", "aero_lowfi": "F2 glide polar, F4 doublets / AUTOTUNE",
          "cfd": "F2/F3 flight-test polar", "flightmodel": "review derived.param; SITL re-run", "sitl": "re-run SITL after the fix; F1 first-flight log review",
          "ingest": "fix the plan file"}
MITIGATION = [
    (r"Tg", "Print skins and spar sockets in LW-ASA/ASA/PETG, or paint the top skins white and keep the aircraft shaded; re-run with the new material card."),
    (r"CG x vs design", "Move the battery (see mass stage shift) or add nose/tail ballast; mark the balanced battery position in the bay."),
    (r"battery travel", "Battery cannot reach the CG target: move other items or add ballast."),
    (r"EI step", "Stagger the tube ends (+-30 mm) and add a printed doubler/sleeve where the tube ends."),
    (r"servo stall torque", "Stronger servo, higher supply voltage or a better horn-arm ratio; torque-check on the bench."),
    (r"ESC continuous", "Higher-rated ESC or cap THR_MAX."),
    (r"WOT time to", "Cap TKOFF_THR_MAX / climb time; IR-thermometer check of the motor after the first WOT climb."),
    (r"gust load factor", "Lower VNE (AIRSPEED_MAX) or justify a higher n_limit with a proof test."),
    (r"AWG", "Use thicker battery leads (one gauge up) or keep WOT bursts short."),
    (r"protruding", "Move the part inside the skin or add a fairing/cut-out in the CAD."),
    (r"closer than", "Re-position parts for >= 2 mm clearance (wiring, vibration, assembly)."),
    (r"clearance to", "Increase rotor clearance (spacer, smaller prop, move the motor)."),
    (r"mission .*energy|energy \(SITL", "Shorten the mission, add capacity or reduce drag; calibrate drag (F2) and propulsion (C2)."),
    (r"max level speed", "Cap THR_MAX or AIRSPEED_MAX so level flight stays below VNE."),
    (r"static margin|Cm_alpha", "Move the CG or resize the tail; re-run aero."),
    (r"spiral", "More dihedral or vertical tail; the autopilot copes if the time to double >= 12 s."),
    (r"Dutch", "More vertical-tail area / yaw damper (KFF_RDDRMIX, YAW2SRV_DAMP)."),
    (r"VTOL T/W|hover duty", "Bigger lift motors/props, higher voltage or lower AUW; set Q_M_BAT_CURR_MAX."),
    (r"per-cell current", "Higher-rate cells / more parallel strings, or current-limit (Q_M_BAT_CURR_MAX)."),
    (r"cell temperature", "Limit hover time; log pack temperature; consider more cells in parallel."),
    (r"share of AUW", "Weigh all parts (calibration C0) and update the plan masses."),
    (r"printed .* plan mass", "Weigh the printed part and update its plan mass."),
    (r"boom", "Stiffen the boom (larger tube) and/or notch-filter the boom frequency."),
    (r"landing position", "Check EKF/GPS lag and Q position-controller tuning."),
    (r"SITL energy per km", "Calibrate drag (F2 glide test) and propulsion (C2 thrust stand); check TECS throttle activity."),
    (r"cross-track", "Use the best NAVL1_PERIOD from the sweep and pre-shift arc waypoints inward by L1^2/(2R)."),
    (r"full-throttle|max airspeed <= AIRSPEED_MAX", "Lower TKOFF_THR_MAX / THR_MAX for the climb, or raise TECS_PITCH_MAX."),
    (r"survives the throw|after release", "Train a throw >= 10 m/s; TKOFF_THR_MINSPD 2-3 with TKOFF_THR_DELAY kept for prop safety."),
    (r"failsafe", "Check FS_* / BATT_FS_* parameters and repeat the failsafe test."),
    (r"stall: SITL", "Aero tables and SITL disagree: review CLmax and trim; calibrate with a stall test (F1)."),
    (r"interference", "Move one of the clashing parts (see the fit report pair) or resize the bay; re-run geometry/clash."),
    (r"CFD", "Investigate the low-order vs CFD difference before relying on derivatives."),
    (r"wrinkle", "Thicker skin or tighter infill spacing on the upper skin."),
    (r"tip deflection", "Stiffer spar (larger tube) or shorter unsupported span."),
    (r"roll authority|trim", "Larger/longer control surfaces or more throw; re-check CG."),
    (r"volume coefficient", "Larger V-tail or longer tail arm."),
]
CALIB = [
    ("C0", "weigh every printed part and component (0.1 g scale), AUW, CG on a balancer", "/components masses & positions", "mass", "AUW +-2 %, CG +-2 mm"),
    ("C1", "bifilar/trifilar pendulum (optional)", "inertia override", "mass", "+-15 %"),
    ("C2", "thrust stand: thrust, current, rpm vs throttle at 3 pack voltages", "prop ct_scale/cp_scale, motor rm/i0", "propulsion", "thrust +-5 %, current +-7 %"),
    ("C3", "battery 10 A / 20 A discharge (or flight BAT log)", "cell r_dc, OCV curve", "propulsion", "sag +-0.05 V/cell"),
    ("C4", "wing-tip sandbag proof load to n_limit; tap test (wing bending/torsion, boom f1)", "material E knockdown, frequencies", "structures", "deflection +-20 %, f +-15 %"),
    ("C5", "servo current at full deflection, linkage slop <= 1 deg", "horn ratios", "structures", "-"),
    ("F2", "power-off glide segments (sink rate vs airspeed)", "aero CD0 / e override", "aero_lowfi", "cruise throttle +-5 pts"),
    ("F3", "cruise legs at 3 speeds (throttle, current)", "drag polar + propulsion", "propulsion", "endurance +-10 %"),
    ("F4", "doublets / AUTOTUNE", "control & damping derivatives", "aero_lowfi", "rate response NRMSE < 25 %"),
    ("F5", "VTOL hover: Q_M_THST_HOVER learned, RCOU spread, VIBE/FFT", "hover throttle, notch centre", "propulsion", "hover throttle +-0.05"),
]


def esc(s):
    return html.escape(str(s))


def fmt(v):
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, float):
        if v != v:
            return "-"
        a = abs(v)
        if a < 1e-6:
            return "0"
        if a >= 1e5:
            return f"{v:.3g}"
        if a >= 1000:
            return f"{v:.0f}"
        return f"{v:.3g}" if a < 0.01 else f"{v:.3f}".rstrip("0").rstrip(".") if a < 10 else f"{v:.1f}"
    if isinstance(v, list):
        return "[" + ", ".join(fmt(x) for x in v[:6]) + (", ..." if len(v) > 6 else "") + "]"
    if isinstance(v, dict):
        return ", ".join(f"{k} {fmt(x)}" for k, x in list(v.items())[:6])
    return "-" if v is None else str(v)


class Doc:
    """Tiny block model rendered to both Markdown and HTML."""

    def __init__(self):
        self.blocks = []

    def h(self, level, text, anchor=None):
        self.blocks.append(("h", level, text, anchor))

    def p(self, text):
        self.blocks.append(("p", text))

    def table(self, head, rows, status_col=None):
        self.blocks.append(("table", head, rows, status_col))

    def img(self, path, caption):
        self.blocks.append(("img", Path(path), caption))

    def pre(self, text):
        self.blocks.append(("pre", text))

    def ul(self, items):
        self.blocks.append(("ul", items))

    def md(self, imgrel):
        out = []
        for b in self.blocks:
            if b[0] == "h":
                out.append("\n" + "#" * b[1] + " " + b[2] + "\n")
            elif b[0] == "p":
                out.append(b[1] + "\n")
            elif b[0] == "table":
                out.append("| " + " | ".join(b[1]) + " |")
                out.append("|" + "---|" * len(b[1]))
                for r in b[2]:
                    out.append("| " + " | ".join(str(c).replace("|", "/").replace("\n", " ") for c in r) + " |")
                out.append("")
            elif b[0] == "img":
                out.append(f"![{b[2]}]({imgrel(b[1])})\n*{b[2]}*\n")
            elif b[0] == "pre":
                out.append("```\n" + b[1] + "\n```\n")
            elif b[0] == "ul":
                out += [f"- {i}" for i in b[1]] + [""]
        return "\n".join(out)

    def html(self, embed, title):
        css = """body{font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;max-width:1180px;margin:24px auto;padding:0 18px;color:#1b1b1b;background:#fcfcfb;line-height:1.45}
h1{font-size:26px;margin-bottom:4px}h2{font-size:20px;border-bottom:2px solid #e4e3df;padding-bottom:4px;margin-top:34px}h3{font-size:16px;margin-top:22px}
table{border-collapse:collapse;width:100%;font-size:12.5px;margin:8px 0 14px}th,td{border-bottom:1px solid #e4e3df;padding:4px 6px;text-align:left;vertical-align:top}
th{background:#f1f0ec;font-weight:600}.PASS{color:#0d7a52;font-weight:700}.WARN{color:#a86f00;font-weight:700}.FAIL,.ERROR,.BLOCKED,.NO-GO{color:#c92a2a;font-weight:700}
.SKIP,.INFO,.NOTRUN{color:#77756f;font-weight:700}.GO{color:#0d7a52;font-weight:700}.CONDITIONAL{color:#a86f00;font-weight:700}
figure{margin:10px 0 18px}figure img{max-width:100%;border:1px solid #e4e3df;border-radius:4px}figcaption{font-size:12px;color:#52514e}
pre{background:#f4f3ef;padding:8px;font-size:11.5px;overflow-x:auto}.verdict{font-size:22px;padding:10px 14px;border-radius:6px;display:inline-block;margin:6px 0}
.v-GO{background:#e3f5ec}.v-NO-GO{background:#fbe4e4}.v-CONDITIONAL{background:#fdf1d8}.v-INCOMPLETE{background:#ececec}"""
        out = [f"<!DOCTYPE html><html><head><meta charset='utf-8'><title>{esc(title)}</title><style>{css}</style></head><body>"]
        for b in self.blocks:
            if b[0] == "h":
                a = f" id='{b[3]}'" if b[3] else ""
                out.append(f"<h{b[1]}{a}>{esc(b[2])}</h{b[1]}>")
            elif b[0] == "p":
                t = esc(b[1])
                t = re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", t)
                t = re.sub(r"\[(.+?)\]\(#(.+?)\)", r"<a href='#\2'>\1</a>", t)
                for w in ("CONDITIONAL GO", "NO-GO", "INCOMPLETE"):
                    if t.startswith("VERDICT: " + w) or t.startswith("VERDICT: " + w.split()[0]):
                        pass
                if t.startswith("VERDICT: "):
                    v = t[9:].split(" ")[0]
                    t = f"<span class='verdict v-{v if v != 'CONDITIONAL' else 'CONDITIONAL'}'>{t}</span>"
                out.append(f"<p>{t}</p>")
            elif b[0] == "table":
                out.append("<table><tr>" + "".join(f"<th>{esc(h)}</th>" for h in b[1]) + "</tr>")
                for r in b[2]:
                    cells = []
                    for i, c in enumerate(r):
                        cs = str(c)
                        m = re.match(r"\[(.+?)\]\(#(.+?)\)", cs)
                        if m:
                            cells.append(f"<td><a href='#{m.group(2)}'>{esc(m.group(1))}</a></td>")
                        elif b[3] is not None and i == b[3]:
                            cells.append(f"<td class='{esc(cs.split()[0] if cs else '')}'>{esc(cs)}</td>")
                        else:
                            cells.append(f"<td>{esc(cs)}</td>")
                    out.append("<tr>" + "".join(cells) + "</tr>")
                out.append("</table>")
            elif b[0] == "img":
                src = embed(b[1])
                if src:
                    out.append(f"<figure><img src='{src}' alt='{esc(b[2])}'><figcaption>{esc(b[2])}</figcaption></figure>")
            elif b[0] == "pre":
                out.append(f"<pre>{esc(b[1])}</pre>")
            elif b[0] == "ul":
                out.append("<ul>" + "".join(f"<li>{esc(i)}</li>" for i in b[1]) + "</ul>")
        out.append("</body></html>")
        return "\n".join(out)


VERIFY_KW = [(r"Tg", "IR-thermometer skin temperature in sun on a hot day; heat-soak test of a printed coupon"),
             (r"EI step|tip deflection|spar", "C4 sandbag proof load to n_limit; inspect skin at tube ends"),
             (r"servo", "bench torque test at supply voltage; C5 servo current at full throw"),
             (r"launch|throw", "SITL re-run + field hand-launch practice with logging"),
             (r"failsafe", "bench failsafe test (props off), then SITL re-run"),
             (r"AWG|ESC|connector|current", "C2 thrust stand with current logging; feel/IR-check leads after a WOT run"),
             (r"WOT time", "IR-thermometer motor check after the first WOT climb"),
             (r"energy|Wh", "F3 cruise legs + BAT log; re-run with calibrated drag/propulsion")]


def verification(stage, check):
    for pat, txt in VERIFY_KW:
        if re.search(pat, check, re.I):
            return txt
    return VERIFY.get(stage, "review")


def mitigation(check):
    for pat, txt in MITIGATION:
        if re.search(pat, check, re.I):
            return txt
    return "Engineering review; add a test or analysis that retires it."


def run(ctx):
    from PIL import Image
    plan = ctx.plan
    rd = ctx.run_dir
    out = ctx.dir
    imgd = out / "img"
    imgd.mkdir(exist_ok=True)
    R = {}
    for s in NAMES:
        if s == "report":
            continue
        f = rd / s / "result.json"
        R[s] = json.loads(f.read_text()) if f.exists() else None
    last = {}
    if (rd / "last_run.json").exists():
        last = json.loads((rd / "last_run.json").read_text())

    def stage_ok(s):
        return R.get(s) and R[s].get("status") == "ok"

    # ---- verdict -----------------------------------------------------------------------------
    rows, n_fail, n_warn, problems = [], 0, 0, []
    for s in NAMES[:-1]:
        r = R.get(s)
        if r is None:
            rows.append([f"[{TITLES[s]}](#{s})", "NOTRUN", 0, 0, "stage did not run"])
            problems.append(f"{s} not run")
            continue
        g = r.get("gates", [])
        f = [x for x in g if x["status"] == "FAIL"]
        w = [x for x in g if x["status"] == "WARN"]
        n_fail += len(f); n_warn += len(w)
        v = r.get("verdict", "?")
        if r.get("status") in ("error", "blocked"):
            v = "ERROR" if r["status"] == "error" else "BLOCKED"
            problems.append(f"{s} {r['status']}: {r.get('error', '')[:100]}")
        top = (f or w or [None])[0]
        topt = f"{top['check']} ({fmt(top['value'])})" if top else (r.get("error", "")[:120] if r.get("status") != "ok" else "-")
        if r.get("status") == "skipped":
            v, topt = "SKIP", (r.get("summary") or {}).get("skipped", "skipped")
        rows.append([f"[{TITLES[s]}](#{s})", v, len(f), len(w), topt])
    if n_fail or any(r[1] in ("ERROR", "BLOCKED") for r in rows):
        overall = "NO-GO"
    elif problems:
        overall = "INCOMPLETE"
    elif n_warn:
        overall = "CONDITIONAL GO"
    else:
        overall = "GO"

    # ---- soft inputs ranked by the gates they drive ------------------------------------------
    soft = (R.get("ingest") or {}).get("data", {}).get("soft_inputs", [])
    bad_gates = [(s, g) for s in R if R[s] for g in R[s].get("gates", []) if g["status"] in ("FAIL", "WARN")]

    def toks(p):
        return [t for t in re.split(r"[/ ()*,]+", p.lower()) if len(t) > 2 and t not in ("components", "airframe", "lifting_surfaces")]
    ranked = []
    for si in soft:
        st = set(toks(si["path"]))
        hits = []
        for s, g in bad_gates:
            for d in g.get("drivers", []):
                dt = set(toks(d))
                if dt and (dt <= st or (st & dt and len(st & dt) >= max(1, min(len(dt), 2)))):
                    hits.append(g)
                    break
        score = sum(2 if h["status"] == "FAIL" else 1 for h in hits)
        ranked.append(dict(si, hits=hits, score=score))
    ranked.sort(key=lambda x: (-x["score"], -(x.get("uncertainty") or 0)))

    def measure_hint(path):
        p = path.lower()
        if "/mass" in p or "printed" in p:
            return "C0 weigh"
        if "/motor" in p or "/prop" in p:
            return "C2 thrust stand"
        if "r_dc" in p or "/cell" in p:
            return "C3 discharge test"
        if "/materials" in p or "/spars" in p:
            return "C4 proof load / tap test"
        if "cl_max" in p:
            return "F1 stall test"
        return "measure / confirm from CAD"

    # ---- calibration: predicted vs measured --------------------------------------------------------
    meas = (plan.get("calibration") or {}).get("measured") or {}
    calrows, model_err = [], []
    if meas:
        md, pd_, ad_, sd_, sid = ((R.get(s) or {}).get("data", {}) for s in ("mass", "propulsion", "aero_lowfi", "structures", "sitl"))
        pv = {}
        if md:
            pv["auw_kg"] = (md["mass_kg"], ("rel", 0.02)); pv["cg_x_mm"] = (1000 * md["cg_m"][0], ("abs", 2.0))
        if pd_:
            st = pd_["cruise_unit"]["static_wot"]["full"]
            pv["static_thrust_g_full"] = (st["thrust_g"], ("rel", 0.05)); pv["static_current_A_full"] = (st["I_bus_total"], ("rel", 0.07))
            pv["cruise_current_A"] = (pd_["cruise_unit"]["cruise"]["I_bus_total"], ("rel", 0.15))
            ht = [x for x in pd_.get("vtol", {}).get("hover_table", []) if x.get("is_plan_auw")]
            if ht:
                pv["hover_throttle"] = (ht[0]["duty"], ("abs", 0.05))
        if ad_:
            pv["v_stall"] = (ad_["V_stall"], ("rel", 0.10)); pv["cd0"] = (ad_["CD0"], ("rel", 0.15))
        if sd_:
            pv["f_bend_Hz"] = (sd_["f_bend_Hz"], ("rel", 0.15)); pv["f_tors_Hz"] = (sd_["f_tors_Hz"], ("rel", 0.15))
        if sid and sid.get("energy"):
            pv["wh_per_km"] = (sid["energy"]["sitl_Wh_per_km"], ("rel", 0.10))
        for k, mv in meas.items():
            if k not in pv:
                calrows.append([k, "-", fmt(mv), "-", "no prediction"])
                continue
            p_, (kind, tol) = pv[k]
            err = (mv - p_) / p_ if kind == "rel" else mv - p_
            ok = abs(err) <= tol
            calrows.append([k, fmt(p_), fmt(mv), (f"{100*err:+.1f} %" if kind == "rel" else f"{err:+.3g}") + f" (band {tol*100 if kind == 'rel' else tol:g}{'%' if kind == 'rel' else ''})",
                            "OK" if ok else "MODEL ERROR"])
            if not ok:
                model_err.append(dict(check=f"model error: {k} predicted {fmt(p_)} vs measured {fmt(mv)}", status="WARN", value=err,
                                      limit=tol, note="outside the calibration acceptance band", drivers=[]))
        bad_gates_extra = [("calibration", g) for g in model_err]
    else:
        bad_gates_extra = []

    # ---- risk register -------------------------------------------------------------------------
    bad_gates = bad_gates + bad_gates_extra
    n_warn += len(model_err)
    if model_err and overall == "GO":
        overall = "CONDITIONAL GO"
    risks = []
    for s, g in bad_gates:
        L = 4 if g["status"] == "FAIL" else 3
        S = SEVERITY.get(s, 3) if g["status"] == "FAIL" else max(SEVERITY.get(s, 3) - 1, 2)
        risks.append(dict(ID=f"R{len(risks) + 1:02d}", stage=s, status_gate=g["status"], hazard=g["check"],
                          cause=(g.get("note") or "")[:200], effect=f"value {fmt(g['value'])} vs limit {fmt(g['limit'])} {g.get('unit', '')}".strip(),
                          L=L, S=S, score=L * S, mitigation=mitigation(g["check"]), verification=verification(s, g["check"]),
                          owner="", status="open", evidence=f"{s}/result.json"))
    risks.sort(key=lambda r: -r["score"])
    with open(out / "risk_register.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(risks[0].keys()) if risks else ["ID"])
        w.writeheader()
        for r in risks:
            w.writerow(r)

    # ---- build checklist -------------------------------------------------------------------------
    chk = []
    m = (R.get("mass") or {}).get("data", {})
    if m:
        chk.append(f"Balance check: CG must be at x = {1000*(m.get('cg_target_m') or m['cg_m'][0]):.0f} +- {1000*m.get('cg_tol_m', 0.005):.0f} mm "
                   f"from the wing-root LE (predicted {1000*m['cg_m'][0]:.1f} mm, AUW {1000*m['mass_kg']:.0f} g).")
        if m.get("battery_shift_m") is not None and abs(m["battery_shift_m"]) > 0.002:
            chk.append(f"Battery position: shift {1000*m['battery_shift_m']:+.0f} mm in x from the plan position to hit the CG target; mark the bay.")
    for ls in plan["airframe"]["lifting_surfaces"]:
        for cs in ls.get("control_surfaces", []):
            chk.append(f"Set {ls['name']}/{cs['name']} throws to +-{cs['max_defl_deg']} deg; check servo direction in FBWA before the first flight.")
    if stage_ok("structures"):
        for g in R["structures"]["gates"]:
            if "servo stall torque" in g["check"]:
                chk.append(f"Torque-check the {g['check'].split(':')[1].split('servo')[0].strip()} servo at its supply voltage (margin {fmt(g['value'])}x, need >= 2).")
        for g in R["structures"]["gates"]:
            if g["status"] != "PASS":
                chk.append(f"[{g['status']}] {g['check']}: {mitigation(g['check'])}")
    for s in ("clash", "propulsion", "mass", "aero_lowfi"):
        for g in (R.get(s) or {}).get("gates", []):
            if g["status"] == "FAIL":
                chk.append(f"[FAIL] {g['check']}: {mitigation(g['check'])}")
    if stage_ok("propulsion"):
        pd = R["propulsion"]["data"]
        st = pd["cruise_unit"]["static_wot"]["full"]
        chk.append(f"Bench: static WOT on a full pack should give ~{st['thrust_g']:.0f} g thrust at ~{st['I_bus_total']:.0f} A "
                   f"({st['rpm']:.0f} rpm); stop if current exceeds the ESC rating.")
    chk += ["Load derived.param (report folder) into the autopilot; verify SERVOn_FUNCTION map and reversals on the bench.",
            "RC-loss and battery failsafe bench test (props off) before the first flight.",
            "Weigh all printed parts and update the plan (calibration C0), then re-run the pipeline."]

    # ---- assemble the document ---------------------------------------------------------------------
    D = Doc()
    title = f"Virtual build report - {plan['meta']['name']} ({ctx.variant})"
    D.h(1, title)
    try:
        ghash = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    except Exception:
        ghash = "?"
    vers = env.versions()
    D.table(["item", "value"], [
        ["plan", f"{ctx.plan_path.name} - {plan['meta']['name']} rev {plan['meta'].get('revision')} ({plan['meta'].get('phase', '')})"],
        ["variant / fidelity", f"{ctx.variant} / {ctx.fidelity}"],
        ["plan hash", (R.get("ingest") or {}).get("data", {}).get("plan_hash", "?")],
        ["pipeline (git)", ghash or "?"],
        ["firmware (SITL)", vers.get("ardupilot", "?") + f" (plan: {plan.get('avionics', {}).get('autopilot', {}).get('firmware_version', '?')})"],
        ["date", time.strftime("%Y-%m-%d %H:%M")],
        ["run wall time", f"{last.get('total_wall_s', 0)/60:.1f} min (this invocation; cached stages count 0)"]])
    D.p(f"VERDICT: {overall} - {n_fail} FAIL, {n_warn} WARN across {sum(1 for r in R.values() if r)} stages"
        + (f"; problems: {'; '.join(problems)}" if problems else ""))
    D.p("Rules: FAIL blocks GO; every WARN needs an owner and a mitigation in the risk register (owners are blank until "
        "the team assigns them, hence CONDITIONAL GO); a stage that errored or did not run makes the result INCOMPLETE/NO-GO.")
    ad0 = (R.get("aero_lowfi") or {}).get("data") or {}
    lm0 = ad0.get("lift_model") or {}
    key_rows = []
    if lm0:
        key_rows.append(["stall speed Vs (trimmed, 1 g, ISA SL)", f"nominal {lm0['V_stall_nominal']:.2f} m/s, PESSIMISTIC {lm0['V_stall_pessimistic']:.2f} m/s "
                         f"(CLmax {lm0['CLmax_trimmed_nominal']:.3f} / {lm0['CLmax_trimmed_pessimistic']:.3f}, -{100*lm0['uncertainty_rel']:.0f} %)",
                         f"CLmax source: {lm0['clmax_source']}; safety gates use the pessimistic value; F1 stall test retires it"])
    if key_rows:
        D.h(2, "Headline numbers (with uncertainty)", "headline")
        D.table(["quantity", "value", "basis"], key_rows)
    D.h(2, "GO / NO-GO by stage", "gonogo")
    D.table(["stage", "verdict", "FAIL", "WARN", "top issue"], rows, status_col=1)
    fails = [(s, g) for s, g in bad_gates if g["status"] == "FAIL"]
    if fails:
        D.h(3, "Blocking items (FAIL) and suggested changes")
        D.table(["stage", "check", "value", "limit", "suggested change"],
                [[s, g["check"], fmt(g["value"]), fmt(g["limit"]), mitigation(g["check"])] for s, g in fails])

    D.h(2, "Soft inputs (estimates) - measure these first", "soft")
    D.p(f"{len(soft)} soft inputs in the plan. Ranked by the number of non-PASS gates they drive (FAIL counts double), then by stated uncertainty.")
    D.table(["#", "plan path", "value", "source", "+-", "drives non-PASS gates", "measure"],
            [[i + 1, r["path"], fmt(r["value"]), r["source"][:70], fmt(r.get("uncertainty")), len(r["hits"]),
              measure_hint(r["path"])] for i, r in enumerate(ranked[:25])])
    uns = (R.get("ingest") or {}).get("data", {}).get("unsourced_masses", [])
    if uns:
        D.p("Masses given as plain numbers (no source string, treated as known): " + ", ".join(uns))

    # ---- configuration -------------------------------------------------------------------------
    D.h(2, "Configuration", "config")
    gd = rd / "geometry"
    for v, cap in (("iso", "Isometric"), ("top", "Top"), ("side", "Side"), ("xray_side", "X-ray side: component placement"),
                   ("xray_top", "X-ray top"), ("xray", "X-ray isometric")):
        p = gd / "renders" / f"{v}.png"
        if p.exists():
            D.img(p, f"Render - {cap}")
    if (gd / "balance.png").exists():
        D.img(gd / "balance.png", "Balance chart (geometry mass model, masses from the plan)")
    if m:
        I = m["inertia_frd"]
        D.table(["quantity", "value"], [
            ["AUW", f"{1000*m['mass_kg']:.0f} g ({m['model']} mass model)"],
            ["CG (x aft of root LE, y, z)", f"{1000*m['cg_m'][0]:.1f}, {1000*m['cg_m'][1]:.1f}, {1000*m['cg_m'][2]:.1f} mm = {100*m['cg_pct_mac']:.1f} % MAC"],
            ["CG target", "-" if m.get("cg_target_m") is None else f"{1000*m['cg_target_m']:.1f} +- {1000*m['cg_tol_m']:.1f} mm"],
            ["Ixx, Iyy, Izz, Ixz (body FRD, about CG)", f"{I['Ixx']:.4f}, {I['Iyy']:.4f}, {I['Izz']:.4f}, {I['Ixz']:+.5f} kg m^2"],
            ["plan point-mass model", f"{1000*m['plan_model']['mass_kg']:.0f} g, CG x {1000*m['plan_model']['cg_m'][0]:.1f} mm, "
                                      f"Ixx {m['plan_model']['inertia_frd']['Ixx']:.4f}"]])
        items = sorted(m["items"], key=lambda r: -r["mass_g"])
        D.h(3, "Mass table")
        D.table(["item", "category", "mass g", "x mm", "y mm", "z mm", "source"],
                [[r["name"], r["category"], r["mass_g"], r["x_mm"], r["y_mm"], r["z_mm"], r["source"][:60]] for r in items])
        if (rd / "mass" / "mass_breakdown.png").exists():
            D.img(rd / "mass" / "mass_breakdown.png", "Mass by category")

    # ---- per stage sections -------------------------------------------------------------------------
    for s in NAMES[:-1]:
        r = R.get(s)
        D.h(2, TITLES[s], s)
        if r is None:
            D.p("Not run in this invocation (see --until/--only).")
            continue
        D.p(f"Status **{r.get('status')}**, verdict **{r.get('verdict')}**, wall {r.get('wall_s', 0)} s"
            + (f", cache key {r.get('key', '')[:10]}" if r.get("key") else ""))
        if r.get("status") in ("error", "blocked"):
            D.pre((r.get("error") or "") + "\n" + (r.get("traceback") or r.get("log_tail") or "")[-1500:])
        sm = r.get("summary") or {}
        if sm:
            D.table(["key figure", "value"], [[k, fmt(v)] for k, v in sm.items() if k != "timings_s"])
        if r.get("gates"):
            D.table(["status", "check", "value", "limit", "note"],
                    [[g["status"], g["check"], fmt(g["value"]) + (f" {g['unit']}" if g.get("unit") and isinstance(g["value"], (int, float)) else ""),
                      (g["kind"] + " " + fmt(g["limit"])) if g["kind"] not in ("bool", "info") else ("must be true" if g["kind"] == "bool" else "-"),
                      (g.get("note") or "")[:160]] for g in r["gates"]], status_col=0)
        if r.get("notes"):
            D.ul(r["notes"])
        figs = [f for f in r.get("files", []) if f.get("kind") == "figure" and not (s == "geometry")]
        if s == "sitl":
            pref = ("timeseries", "track_xt", "stall_0", "launch", "fs_track", "vtol", "cruise_power")
            figs = [f for f in figs if Path(f["path"]).stem in pref]
            figs = [f for f in figs if not ("mission_l1" in f["path"] and "timeseries" in f["path"] and "_l1_15" not in f["path"])]
        for f in figs:
            p = rd / s / f["path"]
            if p.exists():
                D.img(p, f["caption"])
        if s == "sitl" and (rd / "sitl" / "l1_sweep.png").exists() and not any("l1_sweep" in f["path"] for f in figs):
            D.img(rd / "sitl" / "l1_sweep.png", "L1 sweep")
        if s == "propulsion" and stage_ok("propulsion"):
            pd = r["data"]
            ops = pd["cruise_unit"]
            D.table(["operating point", "duty", "rpm", "thrust N", "battery A", "battery W", "motor loss W"],
                    [[k, fmt(v["duty"]), f"{v['rpm']:.0f}", fmt(v["thrust_N"]), fmt(v["I_bus_total"]), fmt(v["P_elec_per_motor"]),
                      fmt(v["motor_loss_W"])] for k, v in [("static WOT, full", ops["static_wot"]["full"]),
                                                            ("static WOT, low (10 %)", ops["static_wot"]["low"]),
                                                            (f"cruise {ops['cruise']['V']} m/s", ops["cruise"])]])
            for mi in pd.get("missions", []):
                D.table(["segment", "time s", "energy Wh", "power W"],
                        [[x["kind"], f"{x['time_s']:.0f}", fmt(x["energy_Wh"]), fmt(x["power_W"])] for x in mi["segments"]]
                        + [["TOTAL", f"{60*mi['time_min']:.0f}", fmt(mi["energy_Wh"]), f"usable after reserve {mi['available_after_reserve_Wh']:.0f} Wh"]])
            ht = pd.get("vtol", {}).get("hover_table")
            if ht:
                D.table(["AUW kg", "T/W", "hover duty", "rpm", "battery A", "A/cell", "power W", "hover min"],
                        [[fmt(x["auw_kg"]), fmt(x.get("TW_max")), fmt(x.get("duty")), fmt(x.get("rpm")), fmt(x.get("I_bat")),
                          fmt(x.get("I_cell")), fmt(x.get("P_total_W")), fmt(x.get("hover_minutes"))] for x in ht])
        if s == "aero_lowfi" and stage_ok("aero_lowfi"):
            d = r["data"]
            md = d.get("modes", {})
            D.table(["mode", "characteristics"], [[k, fmt({kk: vv for kk, vv in v.items() if kk != "eig"}) if isinstance(v, dict) else fmt(v)]
                                                  for k, v in md.items() if k != "trim"])
            D.table(["derivative", "value /rad"], [[k, fmt(v)] for k, v in d["derivatives"].items() if v is not None])
        if s == "sitl" and stage_ok("sitl"):
            res = r["data"]["results"]
            keys = ["wall_s_total", "flight_time_s", "energy_used_Wh_truth", "course_Wh_per_km", "course_mean_power_W", "xt_truth_rms_m",
                    "max_cas", "min_airspeed_inflight", "max_nz", "max_throttle_saturation_s", "hover_power_W", "landing_pos_error_m"]
            D.table(["run"] + keys, [[n] + [fmt(v.get(k)) for k in keys] for n, v in res.items()])
            e = r["data"].get("energy")
            if e:
                D.p(f"Energy calibration: SITL {fmt(e['sitl_Wh_per_km'])} Wh/km vs steady-state {fmt(e['predicted_Wh_per_km'])} Wh/km "
                    f"(ratio {fmt(e['ratio'])}); mission '{e['mission']}' -> {fmt(e['mission_energy_sitl_scaled_Wh'])} Wh vs "
                    f"{fmt(e['available_Wh'])} Wh available after reserve.")
        if s == "flightmodel" and stage_ok("flightmodel"):
            d = r["data"]
            D.table(["derived parameter", "value", "why"], [[k, d["params"].get(k, v), d["derived_why"].get(k, "")] for k, v in d["derived"].items()])
            shutil.copy(rd / "flightmodel" / "derived.param", out / "derived.param")
            shutil.copy(rd / "flightmodel" / "derived_vs_defaults.txt", out / "derived_vs_defaults.txt")
            D.p("Full parameter file: derived.param (in this report folder); diff against the defaults: derived_vs_defaults.txt.")

    D.h(2, "Risk register", "risks")
    D.p(f"{len(risks)} rows (one per WARN/FAIL), sorted by L x S. CSV: risk_register.csv. Owner/status columns are for the team.")
    D.table(["ID", "stage", "gate", "hazard", "L", "S", "score", "mitigation", "verification"],
            [[r["ID"], r["stage"], r["status_gate"], r["hazard"][:90], r["L"], r["S"], r["score"], r["mitigation"][:110], r["verification"][:60]]
             for r in risks], status_col=2)
    D.h(2, "Build checklist", "checklist")
    D.ul(chk)
    D.h(2, "ArduPilot outputs", "params")
    if (out / "derived.param").exists():
        D.pre((out / "derived_vs_defaults.txt").read_text()[:6000])
    if calrows:
        D.h(2, f"Calibration status: predicted vs measured ({(plan.get('calibration') or {}).get('date', '')})", "calstatus")
        D.table(["quantity", "predicted", "measured", "error", "status"], calrows, status_col=4)
    D.h(2, "Calibration needed (close the loop after the first build)", "calibration")
    pred = {}
    if m:
        pred["C0"] = f"AUW {1000*m['mass_kg']:.0f} g, CG x {1000*m['cg_m'][0]:.1f} mm"
    if stage_ok("propulsion"):
        pd = R["propulsion"]["data"]
        st = pd["cruise_unit"]["static_wot"]["full"]
        pred["C2"] = f"static WOT {st['thrust_g']:.0f} g, {st['I_bus_total']:.1f} A, {st['rpm']:.0f} rpm (full pack)"
        pred["F3"] = f"cruise {pd['cruise_unit']['cruise']['V']} m/s: {pd['cruise_unit']['cruise']['I_bus_total']:.2f} A, duty {pd['cruise_unit']['cruise']['duty']:.2f}"
        if pd.get("vtol", {}).get("hover_table"):
            mine = next((x for x in pd["vtol"]["hover_table"] if x.get("is_plan_auw")), {})
            pred["F5"] = f"hover throttle ~{fmt(mine.get('duty'))}, {fmt(mine.get('P_total_W'))} W"
    if stage_ok("structures"):
        sd = R["structures"]["data"]
        pred["C4"] = f"wing 1st bending {sd['f_bend_Hz']:.1f} Hz, torsion {sd['f_tors_Hz']:.1f} Hz, tip deflection {sd['tip_deflection_mm']:.0f} mm at n_design"
    if stage_ok("aero_lowfi"):
        ad = R["aero_lowfi"]["data"]
        pred["F2"] = f"CD0 {ad['CD0']:.4f}, L/D {ad['LD_cruise']:.1f} at cruise, Vs {ad['V_stall']:.1f} m/s"
    D.table(["step", "measurement", "updates in plan", "stage", "acceptance", "prediction to compare"],
            [[c[0], c[1], c[2], c[3], c[4], pred.get(c[0], "-")] for c in CALIB])
    D.p("Process: record measurements in the plan (masses with source 'measured <date>', prop ct_scale/cp_scale, motor rm/i0, cell r_dc, "
        "material E), re-run `python run.py <plan>`; the report then shows the updated margins. See README 'Calibration loop'.")

    # ---- write -------------------------------------------------------------------------------------
    def thumb(p):
        p = Path(p)
        if not p.exists():
            return None
        key = re.sub(r"[^A-Za-z0-9_.-]", "_", str(p.relative_to(rd)))
        dst = imgd / (Path(key).stem + ".png")
        if not dst.exists():
            im = Image.open(p)
            if im.width > 1400:
                im = im.resize((1400, int(im.height * 1400 / im.width)), Image.LANCZOS)
            im.save(dst, optimize=True)
        return dst

    def imgrel(p):
        t = thumb(p)
        return f"img/{t.name}" if t else ""

    def embed(p):
        t = thumb(p)
        return "data:image/png;base64," + base64.b64encode(t.read_bytes()).decode() if t else None
    (out / "report.md").write_text(D.md(imgrel))
    (out / "index.html").write_text(D.html(embed, title))
    for fn, cap in (("index.html", "self-contained HTML report"), ("report.md", "Markdown report"), ("risk_register.csv", "risk register"),
                    ("derived.param", "ArduPilot parameters")):
        if (out / fn).exists():
            ctx.add_file(out / fn, cap, Path(fn).suffix.lstrip("."))
    summ = dict(overall=overall, fails=n_fail, warns=n_warn, risks=len(risks), soft_inputs=len(soft),
                html_MB=round((out / "index.html").stat().st_size / 1e6, 2))
    return dict(summary=summ, data=dict(overall=overall, fails=n_fail, warns=n_warn, stage_rows=rows, problems=problems,
                                        top_fail=[f"{s}: {g['check']}" for s, g in fails][:10],
                                        top_warn=[f"{s}: {g['check']}" for s, g in bad_gates if g["status"] == "WARN"][:10]))
