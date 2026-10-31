"""Build-plan loader shared by every stage.

* validates against schema/buildplan_schema.yaml (jsonschema draft 2020-12) - raw plan AND the
  variant-resolved plan
* applies a variant overlay (merge-patch; arrays of named objects merge by `name`; `<key>_add` appends)
* `val()` resolves `sourced_number` objects -> floats; `soft_inputs()` lists the estimates
* `expanded_components()` / `mass_properties()` = point/box mass model of the component list
"""
from __future__ import annotations

import copy
import hashlib
import json
import re
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
SCHEMA = ROOT / "schema" / "buildplan_schema.yaml"
SOFT_WORDS = re.compile(r"estimat|assum|placeholder|guess|tbd|unknown", re.I)


def val(x, default=None):
    """Numeric value of a plain number or a sourced_number dict."""
    if x is None:
        return default
    if isinstance(x, dict) and "value" in x:
        return float(x["value"])
    return x


def _merge(base, patch):
    out = copy.deepcopy(base)
    for k, v in patch.items():
        if k.endswith("_add") and isinstance(v, list):
            tgt = k[:-4]
            if tgt == "propulsion":
                out.setdefault("propulsion", {}).setdefault("units", []).extend(copy.deepcopy(v))
            else:
                out.setdefault(tgt, []).extend(copy.deepcopy(v))
        elif v is None:
            out.pop(k, None)
        elif isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        elif (isinstance(v, list) and isinstance(out.get(k), list) and v
              and all(isinstance(i, dict) and "name" in i for i in v)):
            idx = {i.get("name"): n for n, i in enumerate(out[k]) if isinstance(i, dict)}
            for item in v:
                if item["name"] in idx:
                    out[k][idx[item["name"]]] = _merge(out[k][idx[item["name"]]], item)
                else:
                    out[k].append(copy.deepcopy(item))
        else:
            out[k] = copy.deepcopy(v)
    return out


def schema():
    return yaml.safe_load(SCHEMA.read_text())


def validate(plan):
    import jsonschema
    v = jsonschema.Draft202012Validator(schema())
    errs = sorted(v.iter_errors(plan), key=lambda e: list(e.path))
    return [f"/{'/'.join(map(str, e.path))}: {e.message}" for e in errs]


def load_raw(path):
    return yaml.safe_load(Path(path).read_text())


# Geometry/envelope numbers that consumers read as plain floats. A plan may give them as sourced numbers
# {value, source, uncertainty|range}; the loader replaces them by the value and keeps the provenance in
# plan["_sourced"][path] so they appear in the soft-input table (review M4).
UNWRAP_KEYS = {"span", "area", "root_chord", "tip_chord", "sweep_le_deg", "sweep_c4_deg", "dihedral_deg", "twist_tip_deg",
               "incidence_deg", "x_le_root", "z_le_root", "thickness_ratio", "at_v_cruise", "at_v_ne", "solar_heating_dT_C",
               "launch_speed"}


def unwrap(plan):
    src = dict(plan.get("_sourced") or {})

    def walk(node, path):
        if isinstance(node, dict):
            for k, v in list(node.items()):
                if k == "_sourced":
                    continue
                p = f"{path}/{k}"
                if k in UNWRAP_KEYS and isinstance(v, dict) and "value" in v:
                    src[p] = dict(v)
                    node[k] = v["value"]          # keep the YAML number type (cache keys stay identical)
                else:
                    walk(v, p)
        elif isinstance(node, list):
            for i, v in enumerate(node):
                nm = v.get("name") if isinstance(v, dict) else None
                walk(v, f"{path}/{nm if nm else i}")
    walk(plan, "")
    plan["_sourced"] = src
    return plan


def resolve(raw, variant=None):
    plan = copy.deepcopy(raw)
    if variant:
        variants = plan.get("variants", {})
        if variant not in variants:
            raise KeyError(f"variant '{variant}' not in plan (have: {list(variants)})")
        plan = _merge({k: v for k, v in plan.items() if k != "variants"}, variants[variant])
        plan["meta"]["variant"] = variant
    else:
        plan = {k: v for k, v in plan.items() if k != "variants"}
        plan["meta"]["variant"] = "base"
    return unwrap(plan)


def apply_calibration(plan, cal_path):
    """Calibration file (YAML): {date, overrides: <overlay merged like a variant>, measured: {...}}."""
    cal = yaml.safe_load(Path(cal_path).read_text()) or {}
    plan = _merge(plan, cal.get("overrides") or {})
    plan["calibration"] = dict(file=str(cal_path), date=str(cal.get("date", "")), measured=cal.get("measured") or {},
                               notes=cal.get("notes", ""))
    return unwrap(plan)


def load(path, variant=None, validate_schema=True, calibration=None):
    raw = load_raw(path)
    if validate_schema:
        errs = validate(raw)
        if errs:
            raise ValueError("schema errors:\n  " + "\n  ".join(errs[:20]))
    plan = resolve(raw, variant)
    if calibration:
        plan = apply_calibration(plan, calibration)
    if validate_schema:
        errs = validate(plan)
        if errs:
            raise ValueError(f"schema errors after applying variant {variant}:\n  " + "\n  ".join(errs[:20]))
    return plan


def slug(s):
    return re.sub(r"[^a-z0-9]+", "_", str(s).lower()).strip("_")


def canonical(o):
    return json.dumps(o, sort_keys=True, separators=(",", ":"), default=str)


def sha(o, n=None):
    h = hashlib.sha256(canonical(o).encode()).hexdigest()
    return h[:n] if n else h


def get_ptr(plan, ptr):
    """JSON-pointer-ish getter ('/airframe/fuselage'); missing -> None."""
    node = plan
    for p in [x for x in ptr.split("/") if x]:
        if isinstance(node, dict):
            node = node.get(p)
        elif isinstance(node, list):
            try:
                node = node[int(p)]
            except (ValueError, IndexError):
                return None
        else:
            return None
        if node is None:
            return None
    return node


# --------------------------------------------------------------------------- provenance
def soft_inputs(plan):
    """Walk the plan; every sourced_number (or item-level `source`) whose source says estimate/assumed,
    or that carries an uncertainty, is a SOFT input. Returns [{path, value, source, uncertainty}]."""
    out = []

    def walk(node, path):
        if isinstance(node, dict):
            if "value" in node and isinstance(node.get("value"), (int, float)):
                src = str(node.get("source", ""))
                if SOFT_WORDS.search(src) or node.get("uncertainty") or node.get("range") or not src:
                    out.append(dict(path=path, value=node["value"], source=src or "(no source)",
                                    uncertainty=node.get("uncertainty"), range=node.get("range")))
                return
            src = node.get("source")
            if isinstance(src, str) and SOFT_WORDS.search(src) and "name" in node:
                out.append(dict(path=path, value=None, source=src, uncertainty=None))
            for k, v in node.items():
                if k in ("variants", "meta", "references", "_sourced"):
                    continue
                walk(v, f"{path}/{k}")
        elif isinstance(node, list):
            for i, v in enumerate(node):
                nm = v.get("name") if isinstance(v, dict) else None
                walk(v, f"{path}/{nm if nm else i}")
    walk(plan, "")
    for p, node in (plan.get("_sourced") or {}).items():
        walk(node, p)
    return out


def unsourced_masses(plan):
    return [c["name"] for c in plan.get("components", []) if not isinstance(c.get("mass"), dict)]


# --------------------------------------------------------------------------- point-mass model
def mirror_name(n):
    """wing_print_R -> wing_print_L, lift_pod_FR -> lift_pod_FL, servo_aileron -> servo_aileron_L"""
    head, _, last = n.rpartition("_")
    if head and last.isupper() and last.endswith("R"):
        return f"{head}_{last[:-1]}L"
    return n + "_L"


def expanded_components(plan):
    comps = []
    for c in plan["components"]:
        c2 = dict(c)
        c2["mass"] = val(c["mass"])
        comps.append(c2)
        if c.get("mirror_y"):
            m = dict(c2)
            m["name"] = mirror_name(c["name"])
            p = list(c["position"]); p[1] = -p[1]
            m["position"] = p
            comps.append(m)
    return comps


def mass_properties(plan):
    """Point masses + box self-inertia (dims). Inertia about CG in the DESIGN frame (x aft, y right, z up)."""
    comps = expanded_components(plan)
    m = np.array([c["mass"] for c in comps], float)
    r = np.array([c["position"] for c in comps], dtype=float)
    M = m.sum()
    cg = (m[:, None] * r).sum(0) / M
    I = np.zeros((3, 3))
    for c, mi, ri in zip(comps, m, r):
        d = ri - cg
        I += mi * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
        if c.get("dims"):
            a, b, h = c["dims"]
            I += np.diag([mi * (b * b + h * h), mi * (a * a + h * h), mi * (a * a + b * b)]) / 12.0
    return {"mass": M, "cg": cg, "inertia": I, "n_items": len(comps), "items": comps}


def design_to_frd_inertia(I_design):
    """design frame (x aft, y right, z up) -> body FRD (x fwd, y right, z down)."""
    R = np.diag([-1.0, 1.0, -1.0])
    return R @ np.asarray(I_design) @ R.T


def mac_geometry(wing):
    cr, ct, b = wing["root_chord"], wing["tip_chord"], wing["span"]
    lam = ct / cr
    mac = 2 / 3 * cr * (1 + lam + lam ** 2) / (1 + lam)
    s = b / 2
    y_mac = s / 3 * (1 + 2 * lam) / (1 + lam)
    if wing.get("sweep_le_deg") is not None and wing.get("sweep_c4_deg") is None:
        tan_le = np.tan(np.radians(wing.get("sweep_le_deg", 0.0)))
    else:
        tan_le = np.tan(np.radians(wing.get("sweep_c4_deg", 0.0) or 0.0)) + 0.25 * (cr - ct) / s
    x_le_mac = wing.get("x_le_root", 0.0) + tan_le * y_mac
    return dict(mac=mac, y_mac=y_mac, x_le_mac=x_le_mac, AR=b * b / wing.get("area", (cr + ct) / 2 * b),
                area=wing.get("area", (cr + ct) / 2 * b))


def main_wing(plan):
    ls = plan["airframe"]["lifting_surfaces"]
    return next((s for s in ls if s.get("kind", "wing") == "wing"), ls[0])


def units(plan, role):
    return [u for u in plan["propulsion"]["units"] if u["role"] == role]
