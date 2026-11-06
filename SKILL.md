---
name: uav-virtual-build
description: Virtually build and fly a small 3D-printed fixed-wing or VTOL drone from its build plan: 3D model and renders, weight/CG/inertia, fit check, structures, propulsion, aero + CFD, ArduPilot flight simulation, and a GO/NO-GO report with fixes.
---

# UAV virtual build

Use this skill when the team wants to know what will go wrong with a drone **before** printing, buying or flying it. Typical triggers: "simulate the build", "model and render the drone", "run CFD / a flight sim", "will it fly / is the CG right / can the servos hold", "check the VTOL version", or after first flights when there is calibration data (weights, thrust stand, logs).

The work is done by the **vbuild** pipeline, a code repo. This file says how to get it, how to turn the team's documents into a build plan, how to run it with sub-agents, how to review the results independently, and how to report. Accuracy matters more than speed: runs can take hours, and that is fine.

## 0. Get the pipeline

1. Look for an existing checkout: `/home/claude/vbuild`, `~/uav-virtual-build`, or the session's repos. If none exists:
   - Attach the private repo with `add_repo` (owner `pxguzman-oss`, repo `uav-virtual-build`, access `read`, or `push` to commit fixes). Then `git clone https://github.com/pxguzman-oss/uav-virtual-build ~/uav-virtual-build`.
   - If the repo can't be reached, tell the user and ask for the repo or a tarball. Don't rewrite the pipeline from memory.
2. `bash setup/install.sh` (idempotent, Ubuntu 24.04). The first run takes ~40 min, mostly the ArduPilot SITL build (`./waf plane`, pinned to Plane-4.7.2).
   - Run it with `nohup … > /tmp/install.log 2>&1 &` and poll, because containers can restart.
   - It installs one venv (`.venv` or the path in `setup/paths.env`), Blender (apt) with xvfb, OpenFOAM v1912 (apt) plus the bundled `foam_sha1_shim` (LD_PRELOAD fix), CalculiX, AVL built from source, xfoil-python, JSBSim (pip), and ArduPilot with only the needed submodules and `empy==3.3.4`.
3. `python setup/check_env.py` reports which stages can run. A stage whose tools are missing shows as ERROR/NOT RUN in the report; it never silently passes.

## 1. Build the plan (the most important step)

The pipeline reads **one build-plan YAML** (`schema/buildplan_schema.yaml`). Start from `examples/stork_phase0.buildplan.yaml`, which has `base`, `vtol` and `stl_demo` variants, and fill it from the team's real sources:

- **Sources to use:** the airframe manual (span, area, chords, airfoil, CG, spar list); the project's parts list, BOM spreadsheet or design doc (masses, motor/prop/ESC/battery, avionics); the real STL/STEP files if supplied; the firmware and parameters; the mission (distance, speed, altitude, wind case).
- **Every uncertain number** is written as `{value, source: "estimate …"|"assumption …"|"measured …", uncertainty}`. Never bury an assumption in a YAML comment: the soft-input table and the corner cases only see explicit fields.
- **Frame:** origin at the wing-root leading edge on the centreline; x aft, y right, z up; SI units. A component `position` is its own CG, not its mounting point. Use `mirror_y` for wing-mounted pairs.
- **Real parts:** put real STL/STEP parts in a variant (see `stl_demo`), with a mapping of part → transform, material, wall thickness and infill. Manufacturer files are usually laid out for printing, so set transforms or `center_at`. Open seams are closed automatically and reported.
- **Variants** (VTOL, payloads, battery options) are overlays under `variants:`, so every variant shares the base airframe numbers.
- Write YAML exponents as `1.2e+11`; PyYAML reads `1.2e11` as a string.
- Validate fast: `python run.py PLAN --until mass --fidelity quick`. Fix every ingest FAIL before going further.

## 2. Run (fidelity ladder)

```
python run.py PLAN --fidelity quick                    # minutes: smoke flight, no CFD; verdict is always INCOMPLETE
python run.py PLAN                                     # standard: 1-alpha wing-body CFD + SITL matrix in mission wind
python run.py PLAN --fidelity full                     # hours: wing-body-tail CFD at 3+ alphas, all wind cases and failsafes
python run.py PLAN --variant vtol [--fidelity …]       # each variant separately
```

Options: `--only STAGE`, `--until STAGE`, `--force STAGE|all`, `--out DIR`, `--calibration CAL.yaml`. Stages are cached by input hash: cheap stages always re-run, CFD and SITL are reused when their inputs are unchanged.

**Stage DAG:**
```
ingest → geometry(+renders) → mass/CG/inertia → clash → aero_lowfi → propulsion → structures → cfd → flightmodel → sitl → report
```

**Runtimes on 2 cores / 7 GB** (measured on the Stork):

| Run | Time |
|---|---|
| quick, base | ≈ 1.7 min |
| standard, base | ≈ 12 min warm, ≈ 30 min cold (≈ 17 min wing-body CFD, ≈ 11 min SITL in calm and mission wind) |
| standard, VTOL variant, CFD cached | ≈ 4.5 min |
| full | ≈ 1.5–3 h (estimate) |
| re-running only the report | ≈ 25 s |

Run anything over 5 minutes with `nohup` and poll the log. If the container restarts, re-run the same command: finished stages are cached.

**Sub-agents** (use them for standard and full runs, and for multiple variants). Give each a narrow brief and the repo path. They share results only through `out/<plan>/<variant>/<stage>/result.json`.

| Wave | Sub-agents in parallel |
|---|---|
| A | one per variant: `--until flightmodel` |
| B | `--only cfd` (long pole); `--only sitl` per variant (SITL picks a free instance, so parallel runs are safe) |
| C | report per variant, then the **independent reviewer** (section 3) |

On 2 cores, don't run more than 2 heavy jobs (CFD or SITL) at once.

## 3. Independent review (always, before reporting)

Spawn a reviewer sub-agent that did **not** build the plan. Give it the plan and the `out/` folder, not your conclusions. Ask it to write `REVIEW.txt` (ID, severity, location, finding, evidence, fix) and to check at least:

1. **Hand calculations** for mass, CG (% MAC), static margin, stall speed, cruise power, static thrust, spar stress, gust load factor, and VTOL hover power/thrust-to-weight. Flag anything off by more than 20 %, or with a wrong sign, unit or axis.
2. **One stall speed (Vs) everywhere.** The same CLmax and Vs must be used by aero, structures, the flight model and the ArduPilot parameters. Is the lift model anchored to CFD/VLM? Are safety gates evaluated on the pessimistic CLmax?
3. **SITL environment.** Did SITL fly in the plan's mission environment (wind, gusts, turbulence) as well as calm air? Is each result labelled with its environment?
4. **Energy accounting.** Is the steady-state energy model consistent with SITL (hotel load, control drag, air density)? Are range and endurance shown as an uncalibrated band?
5. **Provenance and vacuous passes.** Do code hash, plan hash and fidelity match across stages? A stage that didn't run, has no gates or is missing a metric must never count as PASS. A quick run must never say GO.
6. **Frames and signs.** Is Ixz correct in the JSBSim export? Are control-surface signs and Cm reference points consistent?
7. **Figures.** Look at every figure: units, labels, misleading scales.

Fix the critical and major findings (commit each with its finding ID), re-run, and repeat the review if anything critical changed.

## 4. Report to the team

- Open `out/<plan>/<variant>/report/index.html`. It is self-contained, so publish it as an artifact when the team will share it.
- **Lead with the verdict.** GO is allowed only with zero FAIL/ERROR/NOT RUN; CONDITIONAL GO needs owners for every WARN. Then:
  - every FAIL with its suggested change;
  - the top WARNs;
  - "marginal" passes (within 5 % of a limit);
  - the soft inputs to measure first;
  - Vs as nominal/pessimistic;
  - the range band (headline the conservative end);
  - the path to `derived.param`, the ArduPilot parameters derived from the airframe.
- Say plainly that numbers are uncalibrated until the calibration loop has run. Never round a NO-GO into "probably fine".
- Propose plan changes as concrete edits (plan path, old → new, reason), re-run, and show the before/after verdict.
- Save a short summary of the verdict, key numbers and open items to the project when one is attached.

## 5. Calibration loop (after building and flying)

Ask for measurements in this order:

| Step | Measurement |
|---|---|
| C0 | Part and component weights, AUW, measured CG |
| C2 | Thrust stand: thrust, current and RPM vs throttle |
| C3 | Battery discharge / sag |
| C4 | Wingtip proof load and tap test (frequencies) |
| F1–F5 | Flight logs: glide/cruise legs for CD0 and e, stall at known mass, doublets/AUTOTUNE for derivatives, VTOL hover throttle and vibration |

Put them in a calibration YAML (see `examples/stork_calibration_example.yaml`) and run `python run.py PLAN --calibration CAL.yaml`. The report gains a predicted-vs-measured table, and anything outside its band becomes a "model error" risk.

## Known pitfalls (already handled in the code; recheck if tools change)

- **OpenFOAM v1912 (apt):** function objects crash without the sha1 LD_PRELOAD shim; scotch is a stub (use hierarchical decomposition); the tutorials are empty.
- **xfoil:** the apt binary can't run headless; use xfoil-python. Repanel S3021 before XFOIL.
- **NeuralFoil:** its `control_surfaces` flap model is wrong; deflect the geometry instead.
- **AeroBuildup:** it has no downwash, so it is too stable in pitch; Cm_alpha comes from AVL.
- **AVL:** file paths longer than 80 characters are truncated (use relative paths). Give Nspan per section.
- **Steady RANS limits:** fully turbulent RANS over-predicts drag on low-Re laminar sections by ~50 %. Use CFD for lift slope, Cm, interference and non-wing drag increments, not absolute wing drag or CLmax.
- **ArduPilot SITL:** needs `empy==3.3.4`; the built-in SIM_Plane hard-codes mass and fakes propulsion, so use only the JSBSim bridge for numbers.
- **Weak hand launch:** a throw under ~6 m/s never passes ArduPilot's GPS-speed launch check. Treat that as a real finding, not a sim bug.
- **Sub-agents** may be blocked from writing `.md` files; ask them for `.txt` and put full findings in their final message.
