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
    ctx.gate("JSBSim tables: stall speed vs aero stage abs(ratio-1)", abs(sp["Vs_1g_mps"] / aero["V_stall"] - 1), 0.08, "<=", "WARN",
             note=f"tables {sp['Vs_1g_mps']} m/s vs aero {aero['V_stall']:.2f} m/s (untrimmed vs trimmed CLmax)")

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
    der, why = {}, {}

    def put(k, v, reason):
        der[k] = v
        why[k] = reason
    put("AIRSPEED_STALL", round(Vs, 1), "aero_lowfi trimmed stall speed (flaps 0, 1 g)")
    put("AIRSPEED_MIN", math.ceil(max(1.25 * Vs, env.get("v_stall_target", 0)) * 2) / 2, "1.25 x Vs")
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
        put("Q_M_THST_HOVER", hints.get("Q_M_THST_HOVER", 0.45), "propulsion hover duty at plan AUW")
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
    ctx.gate("AIRSPEED_MIN >= 1.2 x Vs", final["AIRSPEED_MIN"] / Vs, 1.2, ">=", "FAIL", note=f"{final['AIRSPEED_MIN']} vs Vs {Vs:.1f} m/s",
             drivers=["/avionics/params/AIRSPEED_MIN", "/airframe/lifting_surfaces/wing/cl_max"])
    ctx.gate("AIRSPEED_MIN x sqrt(n at ROLL_LIMIT) >= Vs (no stall in max-bank turn)",
             final["AIRSPEED_MIN"] / (Vs * math.sqrt(1 / math.cos(math.radians(final["ROLL_LIMIT_DEG"])))), 1.05, ">=", "WARN")
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
                params=final, derived=der, derived_why=why, plan_overrides=plan_p, is_vtol=is_vtol, camera_channel=cam)
    summ = dict(model=m["name"], Vs_tables=sp["Vs_1g_mps"], LD_max=sp["LD_max"], params=len(final), derived=len(der))
    return dict(summary=summ, data=data)
