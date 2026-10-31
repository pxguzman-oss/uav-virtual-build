"""Stage 0 - ingest: schema validation (raw plan and variant-resolved plan), reference checks,
YAML pitfalls, soft-input list, plan hash."""
import re
from pathlib import Path

from .. import plan as P

NUMLIKE = re.compile(r"^[-+]?\d+(\.\d*)?[eE][-+]?\d+$|^[-+]?\d+\.\d+$")
LEGACY_PARAMS = {"ARSPD_FBW_MIN": "AIRSPEED_MIN", "ARSPD_FBW_MAX": "AIRSPEED_MAX", "TRIM_ARSPD_CM": "AIRSPEED_CRUISE (m/s)",
                 "LIM_ROLL_CD": "ROLL_LIMIT_DEG", "LIM_PITCH_MAX": "PTCH_LIM_MAX_DEG", "LIM_PITCH_MIN": "PTCH_LIM_MIN_DEG"}
ROOT = Path(__file__).resolve().parents[2]


def run(ctx):
    raw, plan = ctx.raw_plan, ctx.plan
    errs = P.validate(raw)
    ctx.gate("schema valid (raw plan)", len(errs), 0, "<=", "FAIL", note="; ".join(errs[:3]))
    errs2 = P.validate(plan)
    ctx.gate(f"schema valid (resolved, variant {ctx.variant})", len(errs2), 0, "<=", "FAIL", note="; ".join(errs2[:3]))

    # numbers that YAML read as strings (1.9e9 -> '1.9e9' in YAML 1.1)
    bad = []

    def walk(n, path):
        if isinstance(n, dict):
            for k, v in n.items():
                walk(v, f"{path}/{k}")
        elif isinstance(n, list):
            for i, v in enumerate(n):
                walk(v, f"{path}/{i}")
        elif isinstance(n, str) and NUMLIKE.match(n.strip()) and not path.endswith(("revision", "firmware_version")):
            bad.append(path)
    walk(raw, "")
    ctx.gate("numbers parsed as strings (YAML exponent trap: write 1.9e+9)", len(bad), 0, "<=", "FAIL", note=", ".join(bad[:5]))

    names = [c["name"] for c in plan["components"]]
    dup = sorted({n for n in names if names.count(n) > 1})
    ctx.gate("duplicate component names", len(dup), 0, "<=", "FAIL", note=", ".join(dup))
    exp = {c["name"] for c in P.expanded_components(plan)}
    missing = []
    for s in plan["airframe"]["lifting_surfaces"]:
        for cs in s.get("control_surfaces", []):
            if cs["servo"] not in names:
                missing.append(f"{s['name']}/{cs['name']} servo '{cs['servo']}'")
        for sp in s.get("spars", []):
            if sp["tube"].get("material") not in plan["materials"]:
                missing.append(f"spar {sp['name']} material")
    for b in plan["airframe"].get("booms", []):
        for t in b.get("tip_mass_components", []):
            if t not in exp:
                missing.append(f"boom {b['name']} tip mass '{t}'")
    for c in plan["components"]:
        if c.get("material") and c["material"] not in plan["materials"]:
            missing.append(f"component {c['name']} material '{c['material']}'")
        if c.get("bay") and c["bay"] not in (plan["airframe"]["fuselage"].get("bays") or {}):
            missing.append(f"component {c['name']} bay '{c['bay']}'")
        if c.get("mirror_y") and abs(c["position"][1]) < 1e-6:
            missing.append(f"component {c['name']} mirror_y with y=0 (double counted)")
    for u in plan["propulsion"]["units"]:
        d = u["prop"].get("data", "")
        if d.startswith("embedded:") and not (ROOT / "data" / "props" / f"{d[9:]}.json").exists():
            missing.append(f"prop table {d}")
        if len(u.get("positions", [])) not in (0, u["count"]):
            missing.append(f"unit {u['name']}: {len(u['positions'])} positions for count {u['count']}")
    ctx.gate("broken references (servos, materials, bays, prop tables, boom masses)", len(missing), 0, "<=", "FAIL",
             note="; ".join(missing[:6]))
    legacy = [f"{k}->{v}" for k, v in LEGACY_PARAMS.items() if k in (plan.get("avionics", {}).get("params") or {})]
    ctx.gate("ArduPilot params use current (4.5+) names", len(legacy), 0, "<=", "WARN", note=", ".join(legacy))
    cg_t = P.val(plan.get("analysis", {}).get("cg_target_x"))
    ctx.gate("design CG target given (analysis.cg_target_x)", cg_t is not None, True, "bool", "WARN",
             note="needed for the CG gate; manual value preferred")

    soft = P.soft_inputs(plan)
    uns = P.unsourced_masses(plan)
    ctx.gate("soft (estimated) inputs", len(soft), 0, "info", "INFO", note="listed in the report; measure the top ones first")
    mp = P.mass_properties(plan)
    w = P.main_wing(plan)
    summary = dict(plan=plan["meta"]["name"], revision=plan["meta"].get("revision"), variant=ctx.variant,
                   n_components=len(plan["components"]), n_items_expanded=mp["n_items"],
                   auw_point_mass_kg=round(mp["mass"], 4), soft_inputs=len(soft), unsourced_masses=len(uns),
                   variants=list((raw.get("variants") or {}).keys()))
    data = dict(plan_hash=P.sha(plan, 16), soft_inputs=soft, unsourced_masses=uns,
                wing=dict(span=w["span"], area=w.get("area"), root_chord=w["root_chord"], tip_chord=w["tip_chord"]),
                meta=plan["meta"], variants=summary["variants"])
    return dict(summary=summary, data=data)
