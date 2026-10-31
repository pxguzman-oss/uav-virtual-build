"""Build-plan loader shared by every pipeline stage (prototype).

    python buildplan.py stork_phase0.buildplan.yaml [--variant vtol]

* validates against buildplan_schema.yaml (jsonschema, draft 2020-12)
* resolves `sourced_number` objects -> floats (keeping provenance in .soft)
* applies a variant overlay (merge-patch; `<key>_add` lists are appended)
* computes mass, CG and inertia about the CG (point masses + box/cylinder
  self-inertia when `dims` are given)
"""
from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

import numpy as np
import yaml
import jsonschema

HERE = Path(__file__).resolve().parent


def val(x, default=None):
    """Return the numeric value of a plain number or a sourced_number dict."""
    if x is None:
        return default
    if isinstance(x, dict) and "value" in x:
        return float(x["value"])
    return x


def _merge(base, patch):
    """JSON merge patch, plus: lists of dicts with `name` merge by name,
    and `foo_add: [...]` appends to `foo`."""
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


def load(path, variant=None, validate=True):
    plan = yaml.safe_load(Path(path).read_text())
    if validate:
        schema = yaml.safe_load((HERE / "buildplan_schema.yaml").read_text())
        jsonschema.Draft202012Validator(schema).validate(plan)
    if variant:
        overlay = plan.get("variants", {})[variant]
        plan = _merge({k: v for k, v in plan.items() if k != "variants"}, overlay)
        plan["meta"]["variant"] = variant
    return plan


def expanded_components(plan):
    comps = []
    for c in plan["components"]:
        c2 = dict(c)
        c2["mass"] = val(c["mass"])
        comps.append(c2)
        if c.get("mirror_y"):
            m = dict(c2)
            m["name"] = c["name"].replace("_R", "_L") if c["name"].endswith("_R") else c["name"] + "_mirror"
            p = list(c["position"]); p[1] = -p[1]
            m["position"] = p
            comps.append(m)
    return comps


def mass_properties(plan):
    comps = expanded_components(plan)
    m = np.array([c["mass"] for c in comps])
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
    soft = [c["name"] for c in plan["components"]
            if isinstance(c["mass"], dict) and "estimate" in str(c["mass"].get("source", ""))]
    return {"mass": M, "cg": cg, "inertia": I, "n_items": len(comps), "soft_mass_items": soft}


if __name__ == "__main__":
    p = sys.argv[1] if len(sys.argv) > 1 else HERE / "stork_phase0.buildplan.yaml"
    var = sys.argv[sys.argv.index("--variant") + 1] if "--variant" in sys.argv else None
    plan = load(p, var)
    mp = mass_properties(plan)
    print(f"{plan['meta']['name']} {var or 'base'}: schema OK")
    print(f"  mass = {mp['mass']:.3f} kg over {mp['n_items']} items")
    print(f"  CG   = x {mp['cg'][0]*1000:.1f} mm aft of root LE, y {mp['cg'][1]*1000:.1f} mm, z {mp['cg'][2]*1000:.1f} mm")
    print("  Ixx, Iyy, Izz [kg m^2] =", np.round(np.diag(mp["inertia"]), 4))
    print("  soft (estimated) masses:", mp["soft_mass_items"])
