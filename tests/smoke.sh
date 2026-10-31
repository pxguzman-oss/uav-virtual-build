#!/usr/bin/env bash
# End-to-end smoke test: quick fidelity on the Stork example (base). Expected < 20 min (incl. cold caches).
# Pass = every stage finished (ok or skipped) and the report exists; the verdict itself may be NO-GO.
set -uo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
PY=${PY:-$( [[ -x "$ROOT/.venv/bin/python" ]] && echo "$ROOT/.venv/bin/python" || echo python3 )}
OUT=${OUT:-$ROOT/out_smoke}
VARIANT=${1:-}
t0=$(date +%s)
"$PY" "$ROOT/run.py" "$ROOT/examples/stork_phase0.buildplan.yaml" --fidelity quick --out "$OUT" ${VARIANT:+--variant $VARIANT} --quiet
rc=$?
dt=$(( $(date +%s) - t0 ))
V=${VARIANT:-base}
"$PY" - "$OUT/stork_phase_0/$V" "$dt" <<'PY'
import json, sys
from pathlib import Path
d = Path(sys.argv[1]); bad = []
for s in ["ingest", "geometry", "mass", "clash", "aero_lowfi", "propulsion", "structures", "cfd", "flightmodel", "sitl", "report"]:
    f = d / s / "result.json"
    st = json.loads(f.read_text())["status"] if f.exists() else "missing"
    print(f"  {s:12s} {st}")
    if st not in ("ok", "skipped"):
        bad.append(s)
rep = d / "report" / "index.html"
print(f"report: {rep} ({rep.stat().st_size/1e6:.1f} MB)" if rep.exists() else "report missing")
print(f"wall time {sys.argv[2]} s")
sys.exit(1 if bad or not rep.exists() else 0)
PY
st=$?
[[ $st -eq 0 ]] && echo "SMOKE PASS (run.py exit $rc: 0 GO/CONDITIONAL, 2 NO-GO)" || echo "SMOKE FAIL"
exit $st
