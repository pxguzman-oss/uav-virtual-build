"""Stage context, gates and result records shared by every stage (runs inside the stage subprocess)."""
from __future__ import annotations

import json
import math
import os
import time
import traceback
from pathlib import Path

import numpy as np

from . import plan as P

STATUS_ORDER = {"PASS": 0, "INFO": 0, "WARN": 1, "FAIL": 2, "ERROR": 3}


def clean(o):
    """JSON-safe copy (numpy -> python, NaN/inf -> None)."""
    if isinstance(o, dict):
        return {str(k): clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [clean(v) for v in o]
    if isinstance(o, np.ndarray):
        return clean(o.tolist())
    if isinstance(o, (np.floating,)):
        o = float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, np.bool_):
        return bool(o)
    if isinstance(o, float) and (math.isnan(o) or math.isinf(o)):
        return None if math.isnan(o) else (1e300 if o > 0 else -1e300)
    if isinstance(o, Path):
        return str(o)
    return o


def worst(statuses):
    """Worst gate status. A stage with no gates (or INFO gates only) is INFO, never a vacuous PASS (review M6)."""
    s = [x for x in statuses if x and x != "INFO"]
    return max(s, key=lambda x: STATUS_ORDER.get(x, 0)) if s else "INFO"


MARGINAL = 0.05   # a PASS within 5 % of its limit is flagged "marginal" in the report (review M4)


def margin_of(value, limit, kind):
    """Relative distance of a passing value to its limit (>= 0 passes). None when not numeric."""
    try:
        if isinstance(value, bool) or value is None:
            return None
        v = float(value)
        if kind == ">=":
            L = float(limit)
            return (v - L) / max(abs(L), 1e-9)
        if kind == "<=":
            L = float(limit)
            return (L - v) / max(abs(L), 1e-9)
        if kind == "range":
            lo, hi = float(limit[0]), float(limit[1])
            return min(v - lo, hi - v) / max(abs(hi - lo), 1e-9)
    except (TypeError, ValueError, IndexError):
        return None
    return None


class Ctx:
    """What a stage sees: the resolved plan, fidelity, its own output dir and upstream results."""

    def __init__(self, spec: dict):
        self.spec = spec
        self.stage = spec["stage"]
        self.plan_path = Path(spec["plan_path"])
        self.variant = spec.get("variant")
        self.fidelity = spec.get("fidelity", "standard")
        self.dir = Path(spec["stage_dir"])
        self.run_dir = Path(spec["run_dir"])          # out/<plan>/<variant>
        self.dir.mkdir(parents=True, exist_ok=True)
        self.calibration = spec.get("calibration")
        self.plan = P.load(self.plan_path, self.variant if self.variant != "base" else None, calibration=self.calibration)
        self.raw_plan = P.load_raw(self.plan_path)
        self.gates: list[dict] = []
        self.notes: list[str] = []
        self.files: list[str] = []
        self.t0 = time.time()
        self.env = spec.get("env", {})

    # ---------------------------------------------------------------- upstream
    def updir(self, stage) -> Path:
        return self.run_dir / stage

    def up(self, stage, required=True):
        f = self.updir(stage) / "result.json"
        if not f.exists():
            if required:
                raise RuntimeError(f"upstream stage '{stage}' has no result")
            return None
        r = json.loads(f.read_text())
        if r.get("status") not in ("ok",):
            if required:
                raise RuntimeError(f"upstream stage '{stage}' status {r.get('status')}")
            return None
        return r

    def data(self, stage, required=True):
        r = self.up(stage, required)
        return (r or {}).get("data") if r else None

    # ---------------------------------------------------------------- outputs
    def gate(self, check, value, limit, kind=">=", severity="FAIL", unit="", note="", drivers=None, stage=None):
        """kind: '>=' (value must be >= limit), '<=', 'range' (limit=[lo, hi]), 'info' (never fails),
        'bool' (value truthy passes)."""
        if value is None:
            st = "WARN" if severity != "INFO" else "INFO"
            note = (note + " (value not available)").strip()
        elif kind == ">=":
            st = "PASS" if value >= limit else severity
        elif kind == "<=":
            st = "PASS" if value <= limit else severity
        elif kind == "range":
            st = "PASS" if limit[0] <= value <= limit[1] else severity
        elif kind == "bool":
            st = "PASS" if value else severity
        elif kind == "error":
            st = "ERROR"
        else:
            st = "INFO"
        g = dict(stage=stage or self.stage, check=check, value=value, limit=limit, kind=kind, status=st,
                 unit=unit, note=note, drivers=drivers or [])
        mg = margin_of(value, limit, kind)
        if mg is not None:
            g["margin"] = mg
            if st == "PASS" and mg < MARGINAL:
                g["marginal"] = True
        self.gates.append(clean(g))
        v = f"{value:.4g}" if isinstance(value, (int, float)) and not isinstance(value, bool) else str(value)
        print(f"   [{st:4s}] {check}: {v} {kind} {limit} {unit} {note}", flush=True)
        return st == "PASS"

    def missing(self, check, note=""):
        """Mandatory evidence that is absent (run not flown, metric not logged): an ERROR gate, never silence."""
        return self.gate(check, None, None, "error", "ERROR", note=(note + " (missing evidence)").strip())

    def add_file(self, path, caption=None, kind=None):
        p = Path(path)
        rel = os.path.relpath(p, self.dir) if p.is_absolute() else str(p)
        self.files.append(dict(path=rel, caption=caption or "", kind=kind or p.suffix.lstrip(".")))

    def note(self, s):
        self.notes.append(s)
        print("   note:", s, flush=True)


def run_stage(spec_path):
    """Entry point of the stage subprocess."""
    import importlib
    spec = json.loads(Path(spec_path).read_text())
    ctx = Ctx(spec)
    mod = importlib.import_module(f"vb.stages.{spec['stage']}")
    res = dict(stage=spec["stage"], status="ok", key=spec.get("key"), variant=ctx.variant,
               fidelity=ctx.fidelity, plan=ctx.plan["meta"]["name"], provenance=spec.get("provenance"))
    try:
        out = mod.run(ctx) or {}
        if out.get("skipped"):
            res["status"] = "skipped"
        res.update(summary=out.get("summary", {}), data=out.get("data", {}),
                   figures=out.get("figures", []), tables=out.get("tables", {}))
    except Exception as e:  # recorded, never propagated to the orchestrator
        traceback.print_exc()
        res.update(status="error", error=f"{type(e).__name__}: {e}", traceback=traceback.format_exc()[-4000:])
        res.setdefault("data", {})
    res["gates"] = ctx.gates
    res["notes"] = ctx.notes
    res["files"] = ctx.files
    res["wall_s"] = round(time.time() - ctx.t0, 2)
    res["data_hash"] = P.sha(clean(res.get("data", {})), 16)
    res["verdict"] = ("ERROR" if res["status"] == "error" else
                      "SKIP" if res["status"] == "skipped" else worst(g["status"] for g in ctx.gates))
    (ctx.dir / "result.json").write_text(json.dumps(clean(res), indent=1))
    return 0 if res["status"] != "error" else 1


if __name__ == "__main__":
    import sys
    sys.exit(run_stage(sys.argv[1]))
