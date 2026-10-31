---
name: virtual-build
description: Virtual build of a small 3D-printed fixed-wing or quadplane UAV from one build-plan YAML - geometry and renders, mass/CG/inertia, fit/clash, low-order aero + CFD, propulsion/electrical, structures, JSBSim model, ArduPilot SITL test matrix, GO/NO-GO report with risk register, build checklist and derived ArduPilot parameters. Use when a team wants to know what will go wrong before printing/flying, or after calibration measurements.
---

# Virtual build

## Workflow
1. Environment: `python setup/check_env.py`. If core stages are missing, run `bash setup/install.sh`.
   It is idempotent and takes about 40 min the first time, mostly the ArduPilot build.
2. Plan: copy `examples/stork_phase0.buildplan.yaml` and edit it. Every estimate gets
   `{value, source: "estimate ...", uncertainty}`. Variants (VTOL, payload, real STL parts) are overlays
   under `variants:`. Write exponents as `1.2e+11`.
3. Validate fast: `python run.py PLAN --until mass --fidelity quick`. Fix every ingest FAIL first.
4. Run:
   - `--fidelity quick`: under 5 min warm. Smoke SITL flight, no CFD.
   - `standard`: about 15-25 min. 1-alpha CFD, the main SITL matrix.
   - `full`: hours. 3+ CFD alphas, every wind case, all failsafes.
   - Run each variant with `--variant NAME`.
5. Read `out/<plan>/<variant>/report/index.html`. Start with the GO/NO-GO table, then the FAIL list
   with its suggested changes, then the soft inputs to measure first.
6. Report back to the team:
   - the verdict;
   - every FAIL and the fix suggested for it;
   - the top WARNs, each needing an owner;
   - the soft inputs that drive them;
   - the path to `derived.param`.
   Never claim GO while any FAIL, ERROR, BLOCKED or not-run stage remains.
7. Propose plan changes as JSON-pointer edits with a reason, re-run, and compare.
8. After the first build or flights, ask for calibration data: weights, CG, thrust stand, discharge,
   tap test, glide, hover logs. Put it in a calibration file and run
   `python run.py PLAN --calibration CAL.yaml`.

## Parallel sub-agents (optional)

| Sub-agent | Command |
|---|---|
| A | `--until mass` |
| B | `--only cfd` (long, standard/full) |
| C | `--only sitl` (once flightmodel exists) |
| D | an independent reviewer who reads only the report and the plan and looks for unsupported claims |

Stages share results only through `out/<plan>/<variant>/<stage>/result.json`.

## Key facts
- The design frame is: x aft, y right, z up, origin at the wing-root LE.
- Positions are item CGs.
- The plan's component list is the single source of mass. The geometry stage only distributes it.
- Stage errors never stop the run. They show as ERROR/BLOCKED in the report.
- Architecture, plan fields, STL import, runtimes, limitations and the calibration loop are in README.md.
