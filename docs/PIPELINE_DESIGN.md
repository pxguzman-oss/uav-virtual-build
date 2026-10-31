# Virtual-Build Pipeline — design for a reusable Claude skill

Goal: given a **build plan** for a small 3D-printed UAV, produce a virtual build that catches the
problems a team would otherwise find on the bench or in the field. That means mis-fits,
CG out of range, under-sized servos or ESCs, cracked spars, overheating, flutter, a VTOL that
can't hover, and ArduPilot parameters that don't match the airframe. The output is a report
with a go/no-go table and a risk register, plus an ArduPilot parameter file and SITL model
that the first real flights then calibrate.

Prototype code in this folder covers the plan loader (`buildplan.py`), the
structures stage (`structures.py`), the propulsion stage (`propulsion.py`) and an FEA
cross-check (`calculix_check.py`). The geometry, aero/CFD and SITL stages are being built
by sibling agents. This document defines the contracts between all of them.

---------------------------------------------------------------------------------------------------

## 1. Principles

1. **One source of truth.** Every stage reads `buildplan.yaml` plus the JSON outputs of upstream stages.
   No stage reads another stage's internals. No numbers are hard-coded in stage scripts except
   documented fallbacks, and each fallback is printed and tagged in the report.
2. **Provenance on every number** (`{value, source, uncertainty}`). The report lists the *soft
   inputs* (estimates) that drive each FAIL/WARN, so the team knows what to measure first.
3. **Gates, not just plots.** Each stage emits `gates[]` records:
   `{stage, check, value, limit, kind, status: PASS|WARN|FAIL, note}`.
   The report aggregates them into the go/no-go table.
4. **Low order first, high order only where it changes a decision.** Beam theory matched
   CalculiX within 1 % on the tubes (see REPORT §A5). Panel/VLM aero comes before CFD, and
   CFD runs only to answer named questions.
5. **Close the loop.** The plan's estimates get replaced by measurements (scale, thrust
   stand, tap test, flight logs). The pipeline is re-run, and the calibrated model becomes the
   baseline for the next variant.
6. **Firmware is part of the airframe.** Structural and propulsion limits are exported as
   ArduPilot parameter suggestions (pitch-rate limit, `ARSPD_FBW_MAX`, `TRIM_THROTTLE`,
   `Q_M_THST_HOVER`, battery failsafes, notch-filter centre). SITL then flies with exactly those values.

## 2. Build-plan schema (`buildplan_schema.yaml`)

The top-level blocks and what they feed:

| block | content | consumed by |
|---|---|---|
| `meta`, `frames` | name, revision, phase, references; datum = wing-root LE, x aft, y right, z up | all |
| `airframe.lifting_surfaces[]` | planform, airfoil, `cl_max`, skin, **spar segments** (tube od/id/material, y-range, x/c), **control surfaces** (span, cf/c, max deflection, horn/servo arms, servo ref) | geometry, structures, aero, flight model |
| `airframe.fuselage`, `booms[]`, `landing`, `launch`, `cad_files` | fuselage envelope and payload bay; booms with supports and the components at the tips; sink rate / crush stroke; launch method | geometry, fit/clash, structures |
| `materials{}` | E, G, strengths, knockdown, Tg, density, **the print settings the data refers to** | structures, thermal, mass |
| `components[]` | name, category, mass, position, dims, material, mount, thermal limit, electrical ratings, servo torque, `mirror_y` | mass/CG/inertia, clash, structures, electrical |
| `propulsion.units[]` | role (cruise/lift/tilt), count, positions, thrust axis, motor (Kv, Rm, I0, limits), prop (D, P, data table, `ct_scale`/`cp_scale`), ESC, wiring | propulsion, flight model, SITL |
| `energy` | cell model (capacity, DC IR, current limit, voltages, mass), S/P, wiring R, usable fraction, reserve | propulsion, SITL battery model |
| `avionics` | autopilot board, firmware and version, frame class, sensors, baseline `.param`, overrides | SITL, report |
| `envelope` | Vs target, Vc, VNE, n limits, FoS, gust velocities, max bank | structures, aero, SITL test matrix |
| `environment_cases[]` | temperature, solar ΔT, altitude, wind and gust, turbulence | thermal, propulsion (density), SITL |
| `missions[]` | segments (launch/climb/cruise/loiter/VTOL…) with distance/duration/speed, reserve | propulsion (energy), SITL missions |
| `gates{}` | per-project overrides of default thresholds | all |
| `variants{}` | overlays (merge-patch; arrays merged by `name`; `<key>_add` appends) | all (each variant is a separate run) |

Lessons already baked in:

* **PyYAML exponent trap.** YAML 1.1 parses `1.9e9` as a *string*, so it must be written `1.9e+9`.
  The schema caught this on the first run, which is a good reason to always validate.
* **Positions are item CGs**, not mounting points. **Mirrored items** use `mirror_y`, so wing-mounted
  masses appear once in the plan but twice in the mass model.
* **Spar placement is explicit** (`y_start`/`y_end`). The tube-end stiffness step it creates is a
  real crack location in printed wings (structures gate "EI step").
* **The VTOL variant is an overlay**, so base and VTOL always share the same airframe numbers.

`stork_phase0.buildplan.yaml` is the filled example: base plus `variants.vtol`.

## 3. Stage DAG

```
                ┌────────────┐
                │ 0 ingest   │  validate schema, resolve variant, hash inputs, list soft inputs
                └─────┬──────┘
                ┌─────▼──────┐
                │ 1 geometry │  OpenVSP/CadQuery model from plan (+STEP/STL import), renders
                └─────┬──────┘
          ┌───────────┼──────────────┐
   ┌──────▼─────┐ ┌───▼────────┐ ┌───▼───────────┐
   │2 mass/CG/I │ │3 fit/clash │ │ 6a aero low-  │  AeroSandbox/AVL/XFLR-style VLM + XFOIL polars
   └──────┬─────┘ └───┬────────┘ │    order      │  -> CL/CD/Cm, derivatives, Vs, L/D, neutral point
          │           │          └───┬───────────┘
          │     ┌─────┴──────────────┼───────────────┐
   ┌──────▼─────▼──┐  ┌──────────────▼──┐   ┌────────▼───────┐
   │ 5 propulsion  │◄─┤ (drag polar)    │   │ 6b CFD (opt.)  │  SU2/OpenFOAM on named questions
   │  & electrical │  └─────────────────┘   └────────┬───────┘   (pusher/fuselage interaction, tail
   └──────┬────────┘                                 │            effectiveness, stall)
   ┌──────▼────────┐                                 │
   │ 4 structures  │◄── rpm bands, max thrust        │
   └──────┬────────┘                                 │
          └───────────────┬──────────────────────────┘
                   ┌──────▼───────┐
                   │7 flight-model│  ArduPilot SIM_Plane JSON coefficients and/or JSBSim XML,
                   │  generation  │  battery/motor model, inertia, + derived .param file
                   └──────┬───────┘
                   ┌──────▼───────┐
                   │8 SITL matrix │  ArduPlane/QuadPlane SITL: missions × environment × faults
                   └──────┬───────┘
                   ┌──────▼───────┐
                   │ 9 report     │  go/no-go, risk register, figures, params, build checklist
                   └──────────────┘
```

Dependency table (what invalidates what):

| stage | needs | plan keys hashed for cache |
|---|---|---|
| 0 ingest | plan | whole file |
| 1 geometry | 0 | airframe.*, cad_files |
| 2 mass/CG/I | 0 (1 optional for printed-part volumes) | components, materials.density, airframe (if volumes from CAD) |
| 3 fit/clash | 1, 2 | components.{dims,position,cad}, airframe.fuselage |
| 6a aero low-order | 1, 2 (CG for static margin) | airframe.lifting_surfaces, fuselage, envelope |
| 5 propulsion | 2, 6a (drag polar; else fallback) | propulsion, energy, envelope, environment_cases, missions |
| 4 structures | 2, 5 (rpm bands, max lift thrust), 6a (optional span loading) | airframe, materials, components, envelope, environment_cases |
| 6b CFD | 1, 6a (operating points) | geometry hash + case list |
| 7 flight model | 2, 5, 6a (6b if present) | outputs of 2/5/6a/6b + avionics |
| 8 SITL | 7, avionics, missions, environment_cases | flight-model hash + firmware version + test matrix |
| 9 report | everything | — |

Structures uses propulsion outputs (boom-vibration RPM band, max lift thrust). Propulsion uses
only the aero drag polar. Running propulsion before structures removes a cycle.

## 4. Gates — default pass/fail criteria ("no problems when built")

Status semantics:
* **FAIL** blocks GO.
* **WARN** needs a named mitigation, or a test that retires it, in the risk register.
* **PASS** needs nothing more.

| stage | check | default gate | rationale |
|---|---|---|---|
| ingest | schema valid; no missing mass/position; soft-input list | must pass | garbage in |
| mass/CG | AUW vs plan target | within ±5 % | wing loading, launch |
| | CG vs design CG (Stork: 47 mm from root LE) | within ±3 % MAC | static margin |
| | static margin (from aero) | 5–15 % MAC | stable but not nose-heavy |
| fit/clash | component-to-component and component-to-printed-wall interference | 0 overlaps; ≥ 2 mm clearance; servo/linkage swept volume clear at full throw | the #1 bench problem |
| | battery insertion path, connector access, hatch closes | must pass | |
| structures | spar MS at ultimate (limit × 1.5) with knocked-down allowable (×0.45 for unknown-vendor CF tube) | MS ≥ 0 | |
| | tip deflection at n_design | ≤ 15 % semi-span (WARN) | aileron binding, cracking |
| | EI step at internal tube ends | ≤ 3 (WARN) | printed-skin crack location |
| | skin wrinkle onset | ≥ n_limit (WARN) | serviceability |
| | divergence and aileron reversal | ≥ 1.5 VNE / ≥ 1.2 VNE | |
| | torsion/bending frequency ratio | ≥ 2 (WARN) | flutter screen; flight-test the envelope |
| | servo stall torque / required (VNE, full deflection, +25 % friction) | ≥ 2 | holding at ≤ 50 % stall |
| | boom MS, thrust-axis tilt ≤ 0.5°, boom f1 outside 0.7–1.3 × hover 1P | MS ≥ 0; WARN otherwise | VTOL yaw authority, vibration |
| | wing torsion with lift pods | ≥ 15 Hz (WARN) | couples with Q-mode attitude loop and flutter |
| | printed-material Tg − 10 °C ≥ hottest skin temperature | FAIL | LW-PLA Tg 55–60 °C |
| | gust n at VNE ≤ n_limit | WARN → lower VNE or raise n_limit | |
| propulsion | ESC continuous ≥ 1.2 × WOT motor current | FAIL | |
| | connector ≥ 1.3 × WOT battery current; wire ampacity | FAIL / WARN | |
| | per-cell current ≤ 0.5 × rating at WOT; cell heating during hover | FAIL / WARN | |
| | motor current ≤ vendor 60 s max / 1.15 | FAIL | |
| | motor loss at cruise ≤ limit / 1.5; WOT time-to-100 °C ≥ launch climb time | FAIL / WARN | |
| | cruise duty ≤ 0.65; static T/W ≥ 0.6 (hand launch) | FAIL | |
| | endurance with reserve ≥ mission | FAIL | |
| | VTOL: T/W ≥ 1.6 at max AUW and hover duty ≤ 0.70 | FAIL (< 1.3), WARN (1.3–1.6) | ArduPilot needs control headroom |
| aero | Vs ≤ hand-launch speed / 1.2; L/D at cruise vs propulsion assumption | FAIL / WARN | |
| | Cm_α < 0, Cn_β > 0, Cl_β < 0; spiral/Dutch-roll modes acceptable | FAIL | |
| | tail volume and V-tail mixing authority at Vs | WARN | |
| SITL | auto-launch reaches 30 m with airspeed ≥ 1.2 Vs | FAIL | |
| | mission completes within energy budget plus reserve, all wind cases | FAIL | |
| | no stall or overspeed (≤ VNE); max load factor ≤ n_limit; throttle not saturated > 5 s in cruise | FAIL / WARN | |
| | VTOL: transitions both ways; Q-mode hover throttle ≤ 0.7; motor-out survivability (report only) | FAIL / info | |
| | failsafes (RC loss, battery low, GPS loss) end in the expected mode | FAIL | |

## 5. Report format

`report/<plan-name>-<variant>-<hash8>/index.html` (also `report.md`), produced by a Jinja2 template:

1. **Header.** Plan name, revision, variant, git hash of the pipeline, firmware version, date.
2. **GO / NO-GO table.** One row per stage: status (worst gate), number of FAIL/WARN, the
   top issue, and the link to that stage's section. The overall verdict is GO only if there are
   zero FAILs and every WARN has an owner and a mitigation in the risk register.
3. **Soft inputs.** Estimates that drive any non-PASS gate, ranked by sensitivity (tornado
   from a ±uncertainty sweep), with "measure this first" actions.
4. **Configuration.** Renders (iso, 3-view, exploded), mass breakdown table, CG/inertia, CG
   envelope plot.
5. **Per-stage sections.** Gates table, key figures (V-n diagram, spar moment/stress,
   thrust/current vs throttle, polars, Cp contours, SITL time histories), assumptions.
6. **Risk register** (CSV plus table):
   `ID | stage | hazard | cause | effect | L(1-5) | S(1-5) | score | mitigation | verification (test/analysis) | owner | status | evidence link`.
   Every WARN/FAIL creates or updates a row automatically. Humans own the mitigation fields.
7. **Build checklist** derived from the gates. Examples: "torque-check ES08MA II at 6 V",
   "stagger 10 mm tube end ±30 mm from 8 mm tube end", "print motor mount in PETG/PC",
   "white paint on top skins".
8. **ArduPilot outputs.** `derived.param` (diff against the baseline) and the SITL model files.
9. **Calibration status.** Predicted vs measured for every calibrated quantity (section 7).

## 6. Caching and re-run strategy

* Each stage declares `inputs = [plan JSON-pointer paths] + [upstream artifact hashes] + tool versions`.
  The cache key is `sha256(canonical_json(inputs))`.
  Outputs go in `out/<stage>/<key>/`, with `manifest.json` holding the key, inputs, timings and
  gates.
* A stage is skipped if its key exists. `--force <stage>` deletes downstream keys.
  Because keys include upstream hashes, invalidation follows the DAG automatically.
* Variants share cache entries for any stage whose hashed keys are unchanged. Example: base and
  VTOL share CFD of the clean wing if the booms are excluded from the CFD case list.
* Expensive artefacts (CFD meshes and solutions, SITL logs) are content-addressed and can live
  in a shared bucket. Cheap stages (≤ 10 s: mass, structures, propulsion) always re-run, so the
  report never shows stale numbers.
* Monte-Carlo / corner cases: the plan's `uncertainty` fields drive N perturbed plans
  (Latin hypercube, N≈50 for cheap stages, 3 corners for CFD/SITL). The gate is evaluated on
  the 5th percentile margin.
* Pin tool versions in `manifest.json`: ccx 2.21, numpy/scipy, OpenVSP, SU2, ArduPilot git tag.

## 7. Parallel sub-agents in the skill

The skill's orchestrator runs stage 0, then dispatches sub-agents in waves:

| wave | parallel sub-agents | why independent |
|---|---|---|
| A | geometry/render · mass/CG/inertia | mass works from component list; geometry adds printed-part volumes later (re-run mass if CAD volumes differ > 5 %) |
| B | fit/clash · aero low-order · CFD meshing (start) | all need only geometry (+CG for aero) |
| C | propulsion · structures (after propulsion JSON lands, seconds) · CFD solve | structures/propulsion are seconds; CFD is the long pole, so start it early |
| D | flight-model generation (one agent per variant) | needs mass + aero + propulsion |
| E | SITL: one sub-agent per test-matrix shard (mission × wind × fault) | cases are independent; each sub-agent runs its own SITL instance on a separate port (`-I n`) |
| F | report writer + independent **reviewer agent** that sees only the report and plan and tries to find unsupported claims | separation of builder/reviewer |

Sub-agent contract: input is plan path, variant, and upstream artifact paths. Output is
`out/<stage>/<key>/result.json` with `gates[]`, figures and a 10-line summary. Sub-agents never
edit the plan. They propose changes as `suggested_changes[]`
(JSON-pointer, old, new, reason), which the orchestrator surfaces in the report.
This scales with a 2-core container: waves C–E dominate, and CFD/SITL shards are the units of
parallelism.

## 8. Calibration loop: virtual model vs first real flights

| step | measurement (cheap) | updates in plan | acceptance (model vs measured) |
|---|---|---|---|
| C0 weigh | every printed part and component on a 0.1 g scale; AUW; CG on balancer | `components[].mass` (source: measured), positions | AUW ±2 %, CG ±2 mm |
| C1 inertia | bifilar/trifilar pendulum (optional) | inertia override | ±15 % |
| C2 thrust stand | static thrust, current and RPM vs throttle at 3 pack voltages | `prop.ct_scale`, `cp_scale`, `motor.rm/i0` (least squares) | thrust ±5 %, current ±7 % |
| C3 battery | 10 A and 20 A discharge (or flight BAT log) | `cell.r_dc`, OCV curve | sag ±0.05 V/cell |
| C4 structure | wingtip "sandbag" proof load to n_limit at the root (no FoS); tap test with a phone accelerometer (wing bending/torsion, boom f1) | material `E` knockdown, frequencies | deflection ±20 %, f ±15 % |
| C5 servos | servo current at full deflection on the bench, linkage slop ≤ 1° | horn ratios | — |
| F1 first flight | ArduPilot `.bin`: CTUN, ARSP, BAT, RCOU, ATT, IMU, VIBE, NTUN | — | — |
| F2 glide/power-off segments | sink rate vs airspeed → CD0, e (fit polar) | aero.cd0/e override | cruise throttle within ±5 % pts; current ±15 % |
| F3 cruise legs at 3 speeds | throttle, current → drag polar + propulsion check | | endurance prediction ±10 % |
| F4 doublets (or ArduPilot `SYSID` / AUTOTUNE results) | rate responses → control derivatives, damping | flight-model derivatives | SITL vs log rate response NRMSE < 25 % |
| F5 VTOL hover | `Q_M_THST_HOVER` learned value, per-motor RCOU spread, VIBE, FFT (`INS_LOG_BAT_*`) | hover throttle, notch centre | hover throttle ±0.05; motor spread < 10 % |

Process:
1. Each calibration writes `calibration/<date>.json` (measured values + raw-file hashes).
2. Rerun the pipeline with `--calibrated` to apply those overrides on top of the plan.
3. The report gains a "predicted vs measured" table. Any parameter outside its acceptance band
   becomes a risk-register row of type "model error".
4. Replay the flown mission in SITL with the calibrated model (same wind estimate from
   `NKF2`/`XKF2`). Pass if the trajectory and energy agree within the bands above.

Calibrated parameters from Phase 0 become priors for the VTOL variant, because the variant is
an overlay on the same plan.

## 9. Prior art and how it maps to stages

* **SUAVE / RCAIDE** (Stanford ADL; RCAIDE is the successor, on PyPI as RCAIDE-LEADS). These are
  multi-fidelity conceptual design frameworks with mission/energy networks:
  https://github.com/suavecode/SUAVE, https://suave.stanford.edu, https://pypi.org/project/RCAIDE-LEADS/0.3.0/.
  Borrow the concepts (energy network, mission segment types). The dependency is heavy for 2-kg UAVs.
* **AeroSandbox** (P. Sharpe), https://pypi.org/project/AeroSandbox/: VLM/AeroBuildup, mass
  properties, propulsion and differentiable optimisation. This is the best fit for stage 6a and
  for optimisation loops over the plan.
* **OpenVSP** for parametric geometry, VSPAERO and mass properties (stage 1). APC PE0 → BEM for
  VSPAERO is discussed at https://groups.google.com/g/openvsp/c/X6dXh4VTcok.
* **ArduPilot SITL `SIM_Plane` JSON coefficient models** (stage 7), e.g.
  https://cocalc.com/github/Ardupilot/ardupilot/blob/master/Tools/autotest/models/skywalker_2013.json,
  https://github.com/ArduPilot/ardupilot/blob/2a218221f06b7ec5b6bb78ed82fdcf35c4db1e32/libraries/SITL/SIM_Plane.cpp,
  https://discuss.ardupilot.org/t/arduplane-sitl-using-json-structure/69492,
  https://discuss.ardupilot.org/t/aerodynamic-coefficients-in-sitl/105108.
  Optionally also JSBSim XML.
* **Propeller data**: APC PER3 files (https://www.apcprop.com/files/PER3_10x6E.dat) and UIUC
  measured data (https://m-selig.ae.illinois.edu/props/propDB.html) for `ct_scale` priors.
* **Gap these tools leave** (and this skill fills): build-risk gates for *printed* airframes
  (Tg vs environment, tube-end EI steps, skin wrinkling, servo torque, Li-ion sag and heat,
  VTOL pod-wing torsion) and firmware limits derived from structure and propulsion.

## 10. Packaging as a skill

```
virtual-build/
  SKILL.md                 # when to use, workflow, gates, how to dispatch sub-agents
  schema/buildplan_schema.yaml
  examples/stork_phase0.buildplan.yaml
  stages/ ingest.py geometry.py mass.py clash.py structures.py propulsion.py
          aero_lowfi.py cfd.py flightmodel.py sitl_matrix.py report.py
  data/props/*.json        # APC PER3 → J,Ct,Cp tables (with source URL + date)
  data/materials.yaml      # LW-PLA/ASA/PETG/CF cards with sources
  templates/report.html.j2
  run.py                   # DAG runner: run.py plan.yaml --variant vtol --until structures
```
`SKILL.md` workflow:
1. Validate and print soft inputs.
2. Run waves A–F.
3. Write the report.
4. List FAILs with suggested plan changes.
5. Never claim GO with an unresolved FAIL.
6. Ask for calibration data after first flights.
