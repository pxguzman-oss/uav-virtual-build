#!/usr/bin/env bash
# Test tiers, cheapest first. Stop at the first tier that fails: a later tier cannot be trusted on top of it.
#
#   bash tests/run_tests.sh fast    # seconds : arithmetic and file handling (thermal network, balance maths, STEP, intake)
#   bash tests/run_tests.sh desk    # ~4 min  : CAD + parts list -> plan -> design loop on the Stork, against known numbers
#   bash tests/run_tests.sh smoke   # ~4 min  : whole pipeline at quick fidelity with a SITL flight (fixed wing + VTOL)
#   bash tests/run_tests.sh all     # the three in order (default)
#
# Not covered here (hours, run by hand before a release): standard and full fidelity with CFD.
set -uo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
PY="$ROOT/.venv/bin/python"; [[ -x "$PY" ]] || PY=python3
tier=${1:-all}
run() { echo; echo "=== tier $1 ==="; shift; "$@" || { echo "TIER FAILED: fix this before running the next tier"; exit 1; }; }
[[ $tier == fast || $tier == all ]] && run fast "$PY" "$ROOT/tests/test_cad_balance_thermal.py"
[[ $tier == desk || $tier == all ]] && run desk "$PY" "$ROOT/tests/test_design_loop.py"
if [[ $tier == smoke || $tier == all ]]; then
  run "smoke (fixed wing)" bash "$ROOT/tests/smoke.sh"
  run "smoke (vtol)" bash "$ROOT/tests/smoke.sh" vtol
fi
echo; echo "ALL REQUESTED TIERS PASSED ($tier)"
