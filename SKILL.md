---
name: uav-virtual-build
description: Take a small 3D-printed fixed-wing or quadplane VTOL drone from a CAD assembly (Fusion STEP) and a parts list to a balanced, simulated design - weight and CG, proven balance options with pros and cons, fit, structures, propulsion, power and range, hover and transition, thermal, aero/CFD, ArduPilot flight sim, GO/NO-GO report. Use whenever the team wants to balance a design, place or swap parts, check centre of gravity, simulate before printing, ask "will it fly", "will it overheat" or "what range", re-check after a CAD or parts change, or feed back weights and flight logs, even if they only mention one of these.
---

# UAV virtual build

Find out what will go wrong with a drone **before** it is printed, bought or flown, and what to change so it balances and flies. The work is done by the **vbuild** pipeline, a code repo; this file is the map, and the repo's `docs/skill/` playbooks hold the detail for each step. Read a playbook when you reach its step, not before.

A simulation narrows the risk; it does not guarantee a flight. Every number is "uncalibrated" until the team's own measurements are fed back (step 6), and you say so.

## Start where the user is

| The user brings | Do | Steps |
|---|---|---|
| CAD assembly + parts list (new design or new revision) | the whole flow | 0 -> 5 |
| A change to a design already taken in ("heavier camera", "other motors", "move the battery") | update the parts list or STEP, re-run the intake, then the design loop. Flight-prove only what the team wants to commit to | 1 -> 2, then 3 on request |
| One question ("is the CG right", "does it fit", "is this ESC enough") | `python run.py PLAN --phase layout` (seconds) or `--phase desk` (~1 min). Answer the question; add one line if the desk check shows other FAILs | 2 |
| A manual and a parts list, no CAD | build the plan by hand from the nearest example (playbook 2), then the design loop | 1 -> 5 |
| Weights, thrust-stand data, flight logs | calibration, then the design loop again | 6 -> 2 |
| "Give us the parameter file" | only for an approved design that is GO or CONDITIONAL GO (step 5). Otherwise say what blocks it | 5 |

## The three questions, cheapest first

```
1 LAYOUT   Does it add up?             plan valid, parts fit, weight, CG, balance options      seconds
2 SIZING   Does it work on paper?      stall, stability, thrust, energy, hover, heat, loads    ~1 min
3 FLIGHT   Does it fly in simulation?  CFD, flight model, ArduPilot SITL missions, failsafes   30 min to hours
```

Work them in order. A layout that does not balance or fit changes every number after it, so never spend a flight simulation on a design that still fails phase 1, and never report a phase 2 or 3 number from an unbalanced layout as if it meant something. The report and the design loop are both organised around these three questions.

## Step 0 - Get the pipeline (playbook `docs/skill/01-setup.md`)

1. Look for a checkout: `~/uav-virtual-build`, `/home/claude/uav-virtual-build`, `/home/claude/vbuild`, the session's repos. If none: `add_repo` (owner `pxguzman-oss`, repo `uav-virtual-build`, `read` or `push`) and clone to `~/uav-virtual-build`. If it cannot be reached, ask the user for the repo or a tarball; never rewrite the pipeline from memory.
2. `bash setup/install.sh` in the background with a log (40 min on 2 cores, 15-20 on the team's 12-core desktop), then `.venv/bin/python setup/check_env.py` and `bash tests/run_tests.sh fast`.
3. Prefer the team's desktop (Windows + WSL2 Ubuntu 24.04, 12 cores, 64 GB; `docs/DESKTOP_SETUP.md`) when this session has a shell on it. In the 2-core cloud workspace keep to the design loop and quick / standard fidelity, and say so.

## Step 1 - Intake: ask, then CAD + parts list -> plan (playbook `02-intake.md`)

**Ask before running anything** (one short round; skip what they already said). You can ask while the install runs.

- **May any part be used, or only US-made parts?** Always ask. It screens the parts list and limits the swaps you may propose (`--policy any | us_made | ndaa`). It is a screening aid, not a compliance ruling.
- Where is the wing-root leading edge in CAD, and which CAD axes point aft, right and up?
- Print material and settings, or slicer weights? (LW-PLA unless told otherwise; slicer or scale weights win.)
- Hottest and coldest day, wind, field altitude, mission, payload? (Current mission: canal inspection in Texas. The example's Texas cases are assumptions to confirm.)
- What may change: move parts, change the frame, swap parts, ballast as a last resort?

Then `python tools/intake.py --cad ... --parts ... --base ... --frame ... --policy ... --out work/<name>`.

**STOP if it exits 2.** It has a QUESTIONS list (unmatched bodies, missing masses, a guessed frame, CAD that disagrees with the base plan). Take every question to the user; an assumption here moves the CG silently. Datasheet values for new motors, ESCs, propellers and cells are looked up and sourced, never invented.

## Step 2 - Design loop: desk check, prove the balance options, decide (playbook `03-design-loop.md`)

```
python tools/design_loop.py work/<name>/<plan>.buildplan.yaml [--variant V]
```

One command, a few minutes: desk-checks the design as drawn (phases 1 and 2), turns the best balance options into plan variants, desk-checks each one, and writes `out/<plan>/design_loop/DECISION.md` with a side-by-side table of **proven** numbers, what each option fixes, what it newly breaks, and the FAILs no option removes.

**STOP: the team decides.** Show several options with pros and cons in plain words, say which you would pick and why, and say the recommendation is a ranking rule. Do not start a long flight proof on a candidate nobody chose, unless they asked you to run the recommended one.

If no candidate clears the layout (exit 2), go back with the specific constraint to relax (a bay too short, a fixed heavy part, missing travel limits), not a generic "does not balance".

## Step 3 - Flight proof of the chosen candidate (playbook `04-flight-proof.md`)

```
python tools/design_loop.py PLAN --fly <variant> [--fidelity standard|full]
```

CFD, flight model and the ArduPilot SITL matrix, in the background with a log. It refuses a candidate that still has a layout FAIL. Quick fidelity and desk checks are never GO.

## Step 4 - Independent review (playbook `05-review.md`)

Before reporting a flight proof, a reviewer that did not build the plan re-derives the key numbers by hand (weight, CG, the balance option's CG shift, stall, hover power, one thermal balance) and checks frames, signs, provenance and figures. No sub-agents available: do the hand calculations yourself in a fresh pass and say the review was not independent. For a one-question desk check, hand-check the number you are about to quote.

## Step 5 - Report, hand back, approval (playbook `06-report.md`)

The report (`out/<plan>/<variant>/report/index.html`) opens with the three questions and a "What to do next" list grouped by who acts. Answer the team in this shape:

```
VERDICT  ->  THE THREE QUESTIONS  ->  BALANCE (candidates, your pick, why)  ->  WHAT TO DO NEXT (FAILs first,
by who acts)  ->  KEY NUMBERS (conservative end first)  ->  WHAT IS SOFT (measure first)  ->  PARAMETER FILE  ->  FILES
```

- Hand back CAD for the chosen option: `tools/balance_apply.py ... --step-out` writes the moved STEP; frame changes go in `fusion_change_list.csv` for a designer.
- **STOP: approval.** The loadable `derived.param` is released only when the team sets `meta.approval: {design_approved: true, by, date}` and the run is GO or CONDITIONAL GO. Never hand it over earlier, never copy it out of the flight-model folder, never set the approval yourself.

## Step 6 - Calibrate (playbook `07-calibration.md`)

Weights and balance point, thrust stand, battery sag, proof load, temperatures on the flagged parts, flight logs: into a calibration YAML, re-run with `--calibration`, then the design loop again, because measured weights usually move the CG.

## Rules that do not bend

1. Never invent a mass, a datasheet value, a part origin or a frame. Unknown means a question to the user or a sourced lookup, written into the plan as an estimate with its uncertainty.
2. Always ask the sourcing question before proposing or accepting a part.
3. Balance first. No flight proof, and no quoted performance number, from a layout that fails phase 1.
4. Options are offered only after they are proven by the design loop, and always more than one.
5. GO needs standard or full fidelity with zero FAIL, ERROR or NOT RUN. Never round a NO-GO into "probably fine".
6. The parameter file follows the team's approval, not yours.
7. When something in the pipeline breaks, fix it in the repo (commit with the reason) and re-run; a crashed stage is a finding, not a gap to skip. `docs/skill/pitfalls.md` lists the known traps.
