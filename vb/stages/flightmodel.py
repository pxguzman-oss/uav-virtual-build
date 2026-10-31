"""Stage 7 - flight model: aircraft_model JSON (plan + mass + aero + propulsion) -> JSBSim XML, SIM_Plane
fallback, servo map, static performance; plus derived.param = ArduPilot parameters implied by the airframe
(airspeeds from aero, trim throttle and battery thresholds from propulsion, pitch-rate limit from structures,
QuadPlane hover throttle / current limit / notch from the lift-unit analysis), then plan overrides."""
import json
import math
import subprocess
import sys
from pathlib import Path

from .. import plan as P
from .. import liftmodel as LM
from ..adapters import model2sim

ROOT = Path(__file__).resolve().parents[2]
LIB = ROOT / "vb" / "lib" / "sim"
LEGACY = {"ARSPD_FBW_MIN": "AIRSPEED_MIN", "ARSPD_FBW_MAX": "AIRSPEED_MAX", "LIM_ROLL_CD": "ROLL_LIMIT_DEG"}


def read_parm(path):
    out = {}
    for line in Path(path).read_text().splitlines():
        line = line.split("#")[0].strip()
        if line:
            k, v = line.replace(",", " ").split()[:2]
            out[k] = float(v) if "." in v or "e" in v.lower() else int(v)
    return out


def run(ctx):
    import jsonschema
    plan = ctx.plan
    mass = ctx.data("mass")
    aero = ctx.data("aero_lowfi")
    prop = ctx.data("propulsion")
    struct = ctx.data("structures", required=False)
    cfd = ctx.data("cfd", required=False)
    adir = ctx.updir("aero_lowfi")
    ds = json.loads((adir / "aero_dataset.json").read_text())
    sim_block = json.loads((adir / "sim_aero_block.json").read_text())
    sitl_cfg = plan.get("analysis", {}).get("sitl", {}) or {}
    m = model2sim.build(plan, mass, ds, sim_block, prop, ctx.plan_path.parent, sitl_cfg)
    if cfd and cfd.get("cl_ratio_mean"):
        m["provenance"]["cfd"] = f"CFD/VLM lift ratio {cfd['cl_ratio_mean']:.3f} (not applied; see report)"
    m["ardupilot"] = {}
    schema = json.loads((LIB / "aircraft_model.schema.json").read_text())
    errs = [e.message for e in jsonschema.Draft202012Validator(schema).iter_errors(m)]
    ctx.gate("aircraft model JSON valid (aircraft_model.schema.json)", len(errs), 0, "<=", "FAIL", note="; ".join(errs[:3]))
    gen = ctx.dir / "gen"
    gen.mkdir(exist_ok=True)
    mf = ctx.dir / f"{m['name']}.json"
    mf.write_text(json.dumps(m, indent=1))
    r = subprocess.run([sys.executable, str(LIB / "gen_model.py"), str(mf), "-o", str(gen)], capture_output=True, text=True)
    print(r.stdout[-2000:], r.stderr[-2000:])
    if r.returncode != 0:
        raise RuntimeError("gen_model failed: " + r.stderr[-800:])
    sp = json.loads((gen / f"{m['name']}_static_perf.json").read_text())
    lm = aero.get("lift_model") or {}
    cl_un = lm.get("CLmax_untrimmed_nominal")
    ctx.gate("JSBSim tables: CLmax = lift-model CLmax (untrimmed), abs(ratio-1) (self-consistency)",
             None if not cl_un else abs(sp["CLmax"] / cl_un - 1), 0.02, "<=", "FAIL",
             note=f"tables {sp['CLmax']} vs lift model {cl_un}", drivers=["/airframe/lifting_surfaces/wing/cl_max"])
    # pessimistic-CLmax twin of the model (same everything, CL table capped at CLmax x (1-u)) for SITL launch checks
    cl_p = lm.get("CLmax_untrimmed_pessimistic")
    gen_p = ctx.dir / "gen_pess"
    name_p = None
    if cl_p:
        import copy
        mp = copy.deepcopy(m)
        mp["name"] = m["name"] + "_pess"
        t = mp["aero"]["alpha_tables"]
        t["CL"] = LM.cap_table(t["alpha_deg"], t["CL"], cl_p).tolist()
        mp["aero"]["derivatives"] = dict(mp["aero"]["derivatives"], CLmax=cl_p)
        mp["provenance"]["aero"] += f"; PESSIMISTIC twin: CL table capped at CLmax {cl_p:.3f}"
        gen_p.mkdir(exist_ok=True)
        mfp = ctx.dir / f"{mp['name']}.json"
        mfp.write_text(json.dumps(mp, indent=1))
        rp = subprocess.run([sys.executable, str(LIB / "gen_model.py"), str(mfp), "-o", str(gen_p)], capture_output=True, text=True)
        if rp.returncode != 0:
            raise RuntimeError("gen_model (pessimistic twin) failed: " + rp.stderr[-800:])
        name_p = mp["name"]

    # ---- derived ArduPilot parameters ---------------------------------------------------------
    env = plan["envelope"]
    is_vtol = bool(P.units(plan, "lift"))
    base = read_parm(ROOT / "data" / "ardupilot" / "plane_defaults.parm")
    if is_vtol:
        base.update(read_parm(ROOT / "data" / "ardupilot" / "quadplane_defaults.parm"))
    pf = plan.get("avionics", {}).get("params_file")
    if pf:
        base.update(read_parm((ctx.plan_path.parent / pf).resolve()))
    servo = read_parm(gen / f"{m['name']}_model.parm")
    hints = prop.get("ardupilot_hints", {})
    Vs = aero["V_stall"]
    Vs_p = aero.get("V_stall_pess") or Vs
    der, why = {}, {}

    def put(k, v, reason):
        der[k] = v
        why[k] = reason
    put("AIRSPEED_STALL", math.ceil(Vs_p * 10) / 10, f"PESSIMISTIC trimmed 1 g stall speed (nominal {Vs:.2f} m/s; lift-model CLmax band)")
    put("AIRSPEED_MIN", math.ceil(max(1.25 * Vs, 1.2 * Vs_p, env.get("v_stall_target", 0)) * 2) / 2,
        "max(1.25 x Vs nominal, 1.2 x Vs pessimistic), rounded up to 0.5 m/s")
    put("AIRSPEED_CRUISE", env["v_cruise"], "envelope.v_cruise")
    vmax_level = prop.get("cruise_unit", {}).get("v_max_level") or env.get("v_max_level", 22)
    put("AIRSPEED_MAX", round(min(env.get("v_max_level", 99), 0.88 * env["v_ne"], vmax_level - 1), 1),
        "min(v_max_level, 0.88 VNE, propulsion max level speed - 1)")
    put("TRIM_THROTTLE", hints.get("TRIM_THROTTLE", 40), "propulsion cruise duty at 50 % SOC")
    for k in ("BATT_CAPACITY", "BATT_ARM_VOLT", "BATT_LOW_VOLT", "BATT_CRT_VOLT", "BATT_LOW_MAH", "BATT_CRT_MAH"):
        if k in hints:
            put(k, hints[k], "propulsion pack model (Li-ion: use mAh thresholds with voltage as backup)")
    put("BATT_LOW_TIMER", 10, "Li-ion sag under climb current trips voltage-only failsafes early (SITL lesson)")
    nlim = env["n_limit_pos"]
    roll_max = math.degrees(math.acos(1 / min(nlim * 0.8, 3.0)))
    put("ROLL_LIMIT_DEG", int(min(env.get("max_bank_deg", 45), roll_max)), "envelope.max_bank_deg, capped so 1/cos(bank) <= 0.8 n_limit")
    if struct:
        fw = struct["firmware"]
        put("PTCH2SRV_RMAX_UP", fw["PTCH2SRV_RMAX_UP"], "structures: pitch rate keeping n <= n_limit at VNE")
        put("PTCH2SRV_RMAX_DN", fw["PTCH2SRV_RMAX_DN"], "structures: pitch rate keeping n >= ... at VNE")
    if plan["airframe"].get("launch", {}).get("method") == "hand":
        if "TKOFF_THR_MAX" in hints:
            put("TKOFF_THR_MAX", hints["TKOFF_THR_MAX"], "propulsion: duty giving static T/W 0.65 (limits climb overspeed)")
        put("TKOFF_THR_MINSPD", 3, "weak throws: GPS speed lags (SITL lesson); keep TKOFF_THR_DELAY for prop safety")
    if is_vtol:
        put("Q_M_THST_HOVER", hints.get("Q_M_THST_HOVER", 0.45), "propulsion hover at plan AUW, converted from ESC duty to the "
            "thrust-linearised throttle domain (Q_M_THST_EXPO, Q_M_SPIN_MIN/MAX)")
        put("Q_M_BAT_CURR_MAX", hints.get("Q_M_BAT_CURR_MAX", 0), "0.8 x pack continuous current")
        put("Q_ASSIST_SPEED", der["AIRSPEED_MIN"], "VTOL assist below AIRSPEED_MIN")
        put("INS_HNTCH_ENABLE", 1, "lift rotor 1P notch")
        put("INS_HNTCH_MODE", 1, "throttle-referenced")
        put("INS_HNTCH_FREQ", hints.get("INS_HNTCH_FREQ", 150), "hover rpm / 60 (propulsion)")
        put("INS_HNTCH_REF", der["Q_M_THST_HOVER"], "= Q_M_THST_HOVER")
        put("INS_HNTCH_BW", max(int(hints.get("INS_HNTCH_FREQ", 150) / 2), 20), "half the centre frequency")
    used = set()
    for k, v in servo.items():
        if k.startswith("SERVO") and k.endswith("_FUNCTION") and v:
            used.add(int(k[5:-9]))
    cam = next(ch for ch in range(6, 17) if ch not in used)
    put(f"SERVO{cam}_FUNCTION", 10, "camera trigger on the first free output")
    plan_p = {}
    for k, v in (plan.get("avionics", {}).get("params") or {}).items():
        if k == "TRIM_ARSPD_CM":
            plan_p["AIRSPEED_CRUISE"] = v / 100
        else:
            plan_p[LEGACY.get(k, k)] = v
    final = dict(base)
    final.update(servo)
    final.update(der)
    final.update(plan_p)
    # gates on the final values
    ctx.gate("AIRSPEED_MIN / PESSIMISTIC Vs >= 1.2", final["AIRSPEED_MIN"] / Vs_p, 1.2, ">=", "FAIL",
             note=f"AIRSPEED_MIN {final['AIRSPEED_MIN']} m/s vs Vs pessimistic {Vs_p:.2f} (nominal {Vs:.2f}) m/s",
             drivers=["/avionics/params/AIRSPEED_MIN", "/airframe/lifting_surfaces/wing/cl_max"])
    ctx.gate("AIRSPEED_MIN / (PESSIMISTIC Vs x sqrt(n at ROLL_LIMIT)) >= 1.05 (no stall in a max-bank turn)",
             final["AIRSPEED_MIN"] / (Vs_p * math.sqrt(1 / math.cos(math.radians(final["ROLL_LIMIT_DEG"])))), 1.05, ">=", "WARN",
             note=f"ROLL_LIMIT_DEG {final['ROLL_LIMIT_DEG']}", drivers=["/avionics/params/AIRSPEED_MIN", "/airframe/lifting_surfaces/wing/cl_max"])
    ctx.gate("AIRSPEED_MAX <= 0.88 VNE", final["AIRSPEED_MAX"], 0.88 * env["v_ne"], "<=", "FAIL", "m/s")
    ctx.gate("AIRSPEED_MAX >= 1.5 x AIRSPEED_MIN (TECS range)", final["AIRSPEED_MAX"] / final["AIRSPEED_MIN"], 1.5, ">=", "WARN")
    ctx.gate("AIRSPEED_CRUISE between MIN and MAX", final["AIRSPEED_MIN"] <= final["AIRSPEED_CRUISE"] <= final["AIRSPEED_MAX"], True, "bool", "FAIL")
    conflicts = [f"{k}: plan {plan_p[k]} vs derived {der[k]}" for k in plan_p if k in der and abs(float(plan_p[k]) - float(der[k])) > 1e-6]
    ctx.gate("plan parameter overrides that differ from derived values", len(conflicts), 0, "info", "INFO", note="; ".join(conflicts[:5]))
    lines = [f"# derived.param for {plan['meta']['name']} rev {plan['meta'].get('revision')} variant {ctx.variant}",
             "# generated by vbuild flightmodel stage. Order: defaults < servo map < derived < plan avionics.params.",
             "# NOT a tune: run AUTOTUNE (and QAUTOTUNE for VTOL) in SITL and in the air."]
    for k in sorted(final):
        tag = "plan" if k in plan_p else "derived" if k in der else "servo map" if k in servo else "default"
        note = why.get(k, "")
        lines.append(f"{k:20s} {final[k]:<10}  # {tag}{': ' + note if note and tag == 'derived' else ''}")
    (ctx.dir / "derived.param").write_text("\n".join(lines) + "\n")
    diff = [f"{k:20s} {base.get(k, '-')!s:>10} -> {final[k]}" for k in sorted(final) if base.get(k) != final[k]]
    (ctx.dir / "derived_vs_defaults.txt").write_text("parameter              default -> derived\n" + "\n".join(diff) + "\n")
    ctx.add_file(ctx.dir / "derived.param", "ArduPilot parameter file for this airframe", "param")
    ctx.add_file(mf, "aircraft model JSON (sim input contract)", "json")
    data = dict(model_name=m["name"], model_json=str(mf.relative_to(ctx.dir)), gen_dir="gen", static_perf=sp,
                model_name_pess=name_p, gen_dir_pess="gen_pess" if name_p else None, V_stall=Vs, V_stall_pess=Vs_p,
                params=final, derived=der, derived_why=why, plan_overrides=plan_p, is_vtol=is_vtol, camera_channel=cam)
    summ = dict(model=m["name"], CLmax_tables=sp["CLmax"], Vs_nominal=round(Vs, 2), Vs_pessimistic=round(Vs_p, 2),
                AIRSPEED_STALL=final.get("AIRSPEED_STALL"), AIRSPEED_MIN=final.get("AIRSPEED_MIN"), LD_max=sp["LD_max"],
                params=len(final), derived=len(der))
    return dict(summary=summ, data=data)
