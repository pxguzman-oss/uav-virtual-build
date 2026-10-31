#!/usr/bin/env python3
"""Report which tools are installed and which pipeline stages can run.   python setup/check_env.py [--json]"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vb import env  # noqa: E402


def main():
    s = env.summary()
    if "--json" in sys.argv:
        print(json.dumps(dict(s, versions=env.versions()), indent=1))
        return 0
    print(f"python {sys.version.split()[0]} ({sys.executable})")
    print("\nTOOLS")
    for k, v in s["tools"].items():
        print(f"  {'ok ' if v else '-- '} {k:12s} {v or 'not found'}")
    print("\nPYTHON PACKAGES")
    print("  " + "  ".join(f"{'+' if ok else '-'}{m}" for m, ok in s["python"].items()))
    print("\nSTAGES")
    for k, ok in s["stages"].items():
        print(f"  {'RUNNABLE ' if ok else 'MISSING  '} {k}")
    v = env.versions()
    print("\nVERSIONS " + ", ".join(f"{k} {x}" for k, x in v.items()))
    core = [k for k in ("ingest", "geometry", "mass", "clash", "aero_lowfi", "propulsion", "structures", "flightmodel", "report")
            if not s["stages"][k]]
    if core:
        print("\nNOT READY: core stages missing:", ", ".join(core), "-> run setup/install.sh")
        return 1
    print("\nREADY: quick fidelity runs" + ("" if s["stages"]["sitl"] else " without SITL (no ArduPlane binary)")
          + ("" if s["stages"]["cfd"] else "; standard/full CFD needs OpenFOAM"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
