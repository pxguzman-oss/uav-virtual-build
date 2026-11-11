---
name: uav-virtual-build
description: Take a small 3D-printed fixed-wing or quadplane VTOL drone from a CAD assembly (Fusion STEP) and a parts list to a full virtual build - mass and CG, auto-balance options with pros and cons, fit, structures, propulsion, power and range, hover/transition/motor-out checks, thermal, aero/CFD, ArduPilot flight sim, GO/NO-GO report. Use whenever the team wants to balance a design, place or swap parts, check centre of gravity, simulate before printing, ask "will it fly", size vents or batteries, or re-check after a CAD or parts change, even if they only mention one of these.
---

# UAV virtual build

Use this skill when the team wants to know what will go wrong with a drone **before** printing, buying or flying it, and what to change so it balances and flies. Typical triggers: "here is the CAD and the parts list", "balance this design", "where should the battery go", "simulate the build", "run CFD / a flight sim", "will it fly / is the CG right", "will it overheat", "what range do we get", "check the VTOL version", or after first flights when there is calibration data.

The work is done by the **vbuild** pipeline, a code repo. This file says how to get it, how to turn the team's CAD and parts list into a build plan, how to run it, how to close the balance loop, how to review the results independently, and how to report. Accuracy matters more than speed: runs can take hours, and that is fine. A simulation narrows the risk; it does not guarantee a flight, so every number stays "uncalibrated" until the team's own measurements are fed back (section 7).

## 0. Get the pipeline

1. Look for an existing checkout: `~/uav-virtual-build`, `/home/claude/uav-virtual-build`, or the session's repos. If none exists:
   - Attach the private repo with `add_repo` (owner `pxguzman-oss`, repo `uav-virtual-build`, access `read`, or `push` to commit fixes), then clone it to `~/uav-virtual-build`.
   - If the repo can't be reached, tell the user and ask for the repo or a tarball. Don't rewrite the pipeline from memory.
2. `bash setup/install.sh` (idempotent, Ubuntu 24.04; ~40 min on 2 cores, ~15-20 min on the team's 12-core desktop, mostly the ArduPilot SITL build). Run it in the background with a log and poll, because containers can restart. It installs one venv, Blender with xvfb, OpenFOAM v1912 plus the bundled sha1 shim, CalculiX, AVL, xfoil-python, JSBSim, OpenCascade (`cadquery-ocp`, for STEP) and ArduPilot Plane-4.7.2.
3. `.venv/bin/python setup/check_env.py` reports which stages can run. A stage whose tools are missing shows as ERROR/NOT RUN in the report; it never silently passes.
4. **Where to run.** The team's machine is a Windows desktop (Ryzen 9 5900X, 64 GB, RTX 4080) running the pipeline in WSL2 Ubuntu 24.04; `docs/DESKTOP_SETUP.md` is the step-by-step to give them. When this session has a shell on that machine, run there (full fidelity is practical). In the cloud workspace (2 cores) keep to quick and standard fidelity and say so. Nothing in the pipeline needs the GPU or paid software.

## 1. Intake: ask first, then CAD + parts list -> plan

The pipeline reads **one build-plan YAML**. For a design that exists as CAD, build that plan with `tools/intake.py` rather than by hand, because hand-copied positions are where CG errors come from. Read `docs/CAD_TO_SIM.md` once before the first intake: it has the Fusion export rules and the parts-list columns.

**Ask the user before running anything** (one round, short; skip what they already told you):

- **Sourcing: may any part be used, or only US-made parts?** (`--policy any | us_made | ndaa`). Always ask this one. It screens the parts list and limits the swaps the balance stage may propose. It is a screening aid, not a compliance ruling; say so.
- The datum: where the wing-root leading edge is in CAD coordinates and which CAD axes point aft, right and up. If they can't say, let the intake guess and show them a top view to confirm.
- Print material and settings per part, or slicer weights (slicer or scale weights always win over computed ones). The team prints LW-PLA unless told otherwise.
- Where and how it will fly: hottest and coldest day, wind, field altitude, mission legs, payload. The team's current mission is canal inspection in Texas; the example carries assumed Texas cases (Gulf coast and West Texas summer) to confirm or replace.
- What the optimiser may change: move internal parts, change the frame, swap parts, ballast as a last resort. Default: all four, ballast last.

Then:

```
python tools/intake.py --cad assembly.step --parts parts.csv --base BASE_PLAN.yaml [--variant vtol] \
       --frame "origin=X,Y,Z; x_aft=+z; y_right=+x; z_up=+y" --policy any [--merge datasheets.yaml] --out work/<name>
```

- `--base` supplies what neither CAD nor a parts list can give: aerofoil and planform as analysed, motor/propeller/cell data, mission, environment. For a Stork derivative start from `examples/stork_phase0.buildplan.yaml`; `examples/stork_vtol_flightory/` is a complete worked intake (Flightory's VTOL parts list, `datasheets.yaml`, a synthetic STEP stand-in because the real CAD pack is a paid download).
- Look up datasheet values for any new motor, ESC, propeller or cell (web search, manufacturer pages) and put them in the `--merge` overlay with their source. Never invent them.
- The intake exits 2 with a **QUESTIONS** list (unmatched bodies, rows with no mass or position, quantity mismatches, a guessed frame, CAD dimensions that disagree with the base plan). Take every question to the user; do not paper over one with an assumption.
- Every uncertain number is `{value, source: "estimate ...", uncertainty}`. Never bury an assumption in a comment: the soft-input table and the corner cases only see explicit fields.
- Frame: origin at the wing-root leading edge on the centreline; x aft, y right, z up; SI units. A component `position` is its own CG. Write YAML exponents as `1.2e+11`.
- Validate fast: `python run.py PLAN --until mass --fidelity quick`, and fix every ingest FAIL before going further.

No CAD yet (paper design)? Fill the plan by hand from the manual and the parts list as before; everything below still applies.

## 2. Run (fidelity ladder)

```
python run.py PLAN --fidelity quick       # minutes: no CFD, smoke flight; verdict is always INCOMPLETE
python run.py PLAN                        # standard: wing-body CFD + SITL matrix in the mission wind
python run.py PLAN --fidelity full        # hours: wing-body-tail CFD at 3+ alphas, every wind case and failsafe
python run.py PLAN --variant NAME ...     # each variant (VTOL, payload, balance option) separately
```

Options: `--only STAGE`, `--until STAGE`, `--force STAGE|all`, `--out DIR`, `--calibration CAL.yaml`. Stages are cached by input hash: cheap stages always re-run, CFD and SITL are reused when their inputs are unchanged.

```
ingest -> geometry -> mass -> clash -> aero_lowfi -> propulsion -> vtol -> thermal -> balance -> structures -> cfd -> flightmodel -> sitl -> report
```

What the three newer stages answer:

- **vtol** (quadplane 4+1): load on each lift motor from the real CG, thrust margin in the thinnest air of the plan with a low battery, roll/pitch/yaw authority, crosswind hover, transition time/distance/current, back-transition distance, one lift motor out.
- **thermal**: lumped network of battery, ESCs, electronics, bay air, printed skin and motors over the mission in the hottest and coldest case, nominal and pessimistic. Checks part limits and the print's softening temperature (LW-PLA softens near 55 C, which a Texas summer reaches on its own), sizes bay vents, and lists the parts worth a thermocouple or detailed CFD. Conjugate heat-transfer CFD is not automated: if the team wants it for a flagged bay, set it up by hand in OpenFOAM and say it is a one-off.
- **balance**: section 3.

Runtimes on 2 cores / 7 GB (Stork): quick ~2 min; standard ~30 min cold (CFD ~17 min, SITL ~11 min), ~12 min warm; full 1.5-3 h; report only ~25 s. Run anything over 5 minutes in the background with a log and poll; if the container restarts, re-run the same command.

Sub-agents help on standard and full runs with several variants: one per variant up to the flight model, CFD as its own job, SITL per variant, then the report and the independent reviewer. Give each a narrow brief and the repo path; they share results only through `out/<plan>/<variant>/<stage>/result.json`. On 2 cores run at most 2 heavy jobs at once; on the desktop, 3.

## 3. Balance loop

The balance stage never edits the plan. It writes ranked options to `out/<plan>/<variant>/balance/`:

- `balance_options.md` - every option with CG error, static margin, CG-to-thrust-centre offset, AUW, range estimate, pros, cons and its change list
- `fusion_change_list.csv` - item, CAD body, from, to, delta, why
- `options/<id>.overlay.yaml` - the option as a plan variant

Options it can propose: slide the battery; re-arrange internal parts; moves plus the least ballast; move the lift motors/booms; move the wing on the fuselage; extend the battery bay; swap the battery (catalog and parts-list alternatives that pass the sourcing policy); ballast alone.

How to work it:

1. **Show the team several options, not one.** Give the table and, for the top three or four that reach the target, the pros and cons in plain words. The ranking is a heuristic (effort, range cost, what is left unsolved); the choice is theirs. Say which you would pick and why.
2. **Prove the candidates.** `python tools/balance_apply.py PLAN --from out/.../balance --option B` adds variant `<variant>_balB`; run it through the whole pipeline at the same fidelity as the baseline. The option table's range and static-margin numbers are estimates until this run. Compare verdicts before and after.
3. **Hand back CAD.** For the chosen option add `--step-out balanced_B.step --cad-map work/<name>/cad_map.json`: the STEP has the moved bodies, to import into Fusion. Frame changes (wing, boom, bay moves) are not applied to the STEP - they need a designer - so give the change list for those.
4. **Iterate** when the proving run turns up something new (a clash, a hot ESC next to the battery, a worse hover split): fix the parts list or CAD, re-run the intake, repeat.
5. **Approval gates the autopilot file.** The report always shows the derived ArduPilot parameters for review, but the loadable `derived.param` is released only when the team approves a balanced design - `meta.approval: {design_approved: true, by: ..., date: ...}` in the plan - and the run is GO or CONDITIONAL GO. Do not hand over a parameter file before that, and do not set the approval yourself.

## 4. Independent review (always, before reporting)

Spawn a reviewer sub-agent that did **not** build the plan (when sub-agents are not available, do the hand calculations yourself in a fresh pass and say that it was not independent). Give it the plan and the `out/` folder, not your conclusions. Ask for `REVIEW.txt` (ID, severity, location, finding, evidence, fix) covering at least:

1. **Hand calculations** for mass, CG (% MAC), static margin, stall speed, cruise power, static thrust, spar stress, gust load factor, hover power and thrust-to-weight, the CG shift of the chosen balance option (sum of mass x move / AUW), and one thermal balance (heat in a bay vs wall conductance). Flag anything off by more than 20 %, or with a wrong sign, unit or axis.
2. **Intake.** Do the parts-list masses add up to the plan AUW? Is every CAD body accounted for? Is the frame right (nose forward, right wing at +y)?
3. **One stall speed everywhere**: aero, structures, flight model and ArduPilot parameters use the same CLmax and Vs; safety gates use the pessimistic one.
4. **SITL environment**: flown in the mission wind as well as calm air, each result labelled.
5. **Energy accounting**: steady model consistent with SITL; range and endurance shown as an uncalibrated band.
6. **Provenance and vacuous passes**: code hash, plan hash and fidelity match across stages; a stage that didn't run or has no gates never counts as PASS; a quick run never says GO.
7. **Frames and signs**: Ixz in the JSBSim export, control-surface signs, Cm reference point, lift-motor order (front-right, rear-left, front-left, rear-right).
8. **Figures**: units, labels, misleading scales.

Fix the critical and major findings (commit each with its finding ID), re-run, and repeat the review if anything critical changed.

## 5. Report to the team

- Open `out/<plan>/<variant>/report/index.html`. It is self-contained, so publish it as an artifact when the team will share it.
- **Lead with the verdict.** GO only with zero FAIL/ERROR/NOT RUN; CONDITIONAL GO needs an owner for every WARN. Then: every FAIL with its suggested change; the balance options and the one you recommend; the top WARNs; marginal passes (within 5 % of a limit); the soft inputs to measure first; stall speed nominal/pessimistic; the range band (headline the conservative end); the power budget by flight phase; the hottest parts and what to do about them.
- Say plainly that numbers are uncalibrated until the calibration loop has run. Never round a NO-GO into "probably fine".
- Propose changes as concrete edits (part, from -> to, reason), re-run, and show the before/after verdict.
- Say whether the parameter file was released or withheld, and why.
- Save a short summary of the verdict, key numbers and open items to the project when one is attached.

## 6. Parts and sourcing

- `meta.sourcing.policy` comes from the user's answer in section 1. Under `us_made` or `ndaa` the ingest stage fails parts whose stated origin breaks the policy and warns about unknown or unverified origins.
- When a swap is needed and the catalog (`data/catalog/`) has nothing that passes, search for candidates, check the maker's own statement of origin, and add them to the parts list as alternatives with `origin` and `origin_verified: false` until the team confirms. Don't state an origin you have not seen on the maker's or vendor's page.

## 7. Calibration loop (after building and flying)

Ask for measurements in this order: C0 part weights, AUW and measured CG; C2 thrust stand (thrust, current, rpm vs throttle); C3 battery discharge; C4 wingtip proof load and tap test; thermocouple or IR readings on the flagged parts after a hover and a ground soak; F1-F5 flight logs (glide/cruise legs, stall, doublets/AUTOTUNE, hover throttle and vibration). Put them in a calibration YAML (`examples/stork_calibration_example.yaml`) and run with `--calibration`. The report gains a predicted-vs-measured table, and anything outside its band becomes a "model error" risk.

## Known pitfalls (handled in the code; recheck if tools change)

- **OpenFOAM v1912 (apt):** function objects crash without the sha1 LD_PRELOAD shim; scotch is a stub (hierarchical decomposition); set `analysis.cfd.np` to the physical core count on big machines.
- **Steady RANS limits:** fully turbulent RANS over-predicts drag on low-Re laminar sections by ~50 %. Use CFD for lift slope, Cm, interference and non-wing drag increments, not absolute wing drag or CLmax.
- **xfoil / NeuralFoil / AVL:** use xfoil-python (the apt binary can't run headless); deflect the geometry instead of NeuralFoil's flap model; Cm_alpha comes from AVL; AVL truncates paths over 80 characters.
- **ArduPilot SITL:** needs `empy==3.3.4`; only the JSBSim bridge gives usable numbers. A hand launch under ~6 m/s never passes the launch check: a real finding, not a sim bug.
- **OpenCascade (STEP):** class locations moved between versions (`vb/cadio.py` handles 7.x and 8.x); it prints transfer statistics to stdout, which the code silences.
- **Fusion names:** bodies need the part id at the start of the name; a re-exported assembly with renamed bodies silently becomes "unmatched" questions, which is the intended failure.
- **Thermal numbers** come from a lumped model with +-30-50 % on convection: trust the ranking and the FAIL/WARN split more than the absolute temperatures.
- **Sub-agents** may be blocked from writing `.md` files; ask for `.txt` and put full findings in their final message.
