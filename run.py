#!/usr/bin/env python3
"""Virtual build of a small 3D-printed UAV from ONE build-plan YAML.

  python run.py plan.yaml [--variant vtol] [--fidelity quick|standard|full]
                [--only STAGE ...] [--until STAGE] [--force STAGE|all ...] [--out DIR]

Stages (in order): ingest geometry mass clash aero_lowfi propulsion vtol thermal balance structures cfd flightmodel sitl report
Outputs: <out>/<plan>/<variant>/<stage>/result.json (gates[], summary, files) and
         <out>/<plan>/<variant>/report/{index.html, report.md, derived.param, risk_register.csv, ...}
Exit code: 0 = report GO, 2 = NO-GO (any FAIL/ERROR/BLOCKED), 1 = orchestrator problem.
"""
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))


def _ensure_env():
    """Run under the project venv (VB_VENV env var, setup/paths.env VB_VENV, or ./.venv) when one exists;
    otherwise the current interpreter must have the packages."""
    import os
    venv = os.environ.get("VB_VENV")
    pe = ROOT / "setup" / "paths.env"
    if not venv and pe.exists():
        for line in pe.read_text().splitlines():
            if line.startswith("VB_VENV="):
                venv = line.split("=", 1)[1].strip()
    for cand in [venv, str(ROOT / ".venv")]:
        if cand and Path(cand, "bin", "python").exists():
            if Path(sys.prefix).resolve() != Path(cand).resolve():
                os.execv(str(Path(cand, "bin", "python")), [str(Path(cand, "bin", "python"))] + sys.argv)
            return
    try:
        import numpy, yaml, jsonschema, trimesh  # noqa: F401
    except ImportError:
        sys.exit("python packages missing: run setup/install.sh (or activate the venv)")


_ensure_env()

from vb.dag import Runner  # noqa: E402
from vb.stages import NAMES  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("plan")
    ap.add_argument("--variant", default=None, help="variant name from plan.variants (default: base)")
    ap.add_argument("--fidelity", default="standard", choices=["quick", "standard", "full"])
    ap.add_argument("--only", nargs="+", choices=NAMES, help="run only these stages (upstream results must exist)")
    ap.add_argument("--until", choices=NAMES, help="run this stage and everything it depends on")
    ap.add_argument("--force", nargs="+", default=[], choices=NAMES + ["all"],
                    help="ignore the cache for these stages and everything downstream of them")
    ap.add_argument("--out", default=str(ROOT / "out"))
    ap.add_argument("--quiet", action="store_true", help="do not echo stage logs (they are in <stage>/stage.log)")
    ap.add_argument("--calibration", default=None,
                    help="calibration YAML (measured overrides + measured values) -> <variant>_calibrated run + predicted-vs-measured table")
    a = ap.parse_args(argv)
    r = Runner(a.plan, a.variant, a.fidelity, a.out, a.force, a.only, a.until, a.quiet, a.calibration)
    r.run()
    rep = r.run_dir / "report" / "result.json"
    if "report" in r.selected and rep.exists():
        d = json.loads(rep.read_text()).get("data", {})
        v = d.get("overall")
        if v:
            print(f"OVERALL: {v}")
            return 0 if v in ("GO", "CONDITIONAL GO") else 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
