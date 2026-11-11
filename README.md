VBUILD - virtual build of small 3D-printed UAVs from ONE build-plan YAML
========================================================================

One command turns a build plan (geometry, materials, components, propulsion, battery, firmware,
envelope, environment, missions) into a virtual build. That covers geometry and renders, mass/CG/inertia,
fit/clash, low-order aero, CFD, propulsion and electrical margins, structures, a JSBSim flight model,
the ArduPilot SITL test matrix, and a report. The report has a GO/NO-GO table, a risk register, a build
checklist, a derived ArduPilot parameter file and a list of calibrations still needed.

    bash setup/install.sh                      # once (idempotent, Ubuntu 24.04)
    python setup/check_env.py                  # which stages can run on this machine
    python run.py examples/stork_phase0.buildplan.yaml --fidelity quick
    python run.py examples/stork_phase0.buildplan.yaml                           # standard
    python run.py examples/stork_phase0.buildplan.yaml --variant vtol --fidelity quick
    bash tests/smoke.sh                        # quick end-to-end check (base); `bash tests/smoke.sh vtol`

The report is written to `out/<plan>/<variant>/report/index.html`. The HTML is self-contained, with the
images embedded. The same folder holds `report.md`, `risk_register.csv`, `derived.param` and
`derived_vs_defaults.txt`.

Exit code of run.py: 0 = GO or CONDITIONAL GO, 2 = NO-GO / INCOMPLETE, 1 = orchestrator failure.

If `python` lacks the packages, run.py re-executes itself with the project venv, found from
`setup/paths.env` (VB_VENV) or `./.venv`.



NEW: CAD + PARTS LIST IN, BALANCE OPTIONS OUT
---------------------------------------------
    python tools/intake.py --cad assembly.step --parts parts.csv --base BASE.yaml --frame "..." --policy any --out work/x
    python run.py work/x/<name>.buildplan.yaml
    python tools/balance_apply.py work/x/<name>.buildplan.yaml --from out/<name>/base/balance --option B
    python run.py work/x/<name>.buildplan.yaml --variant balB

  tools/intake.py         Fusion STEP assembly + parts list (CSV/XLSX) + base plan -> build plan, with a QUESTIONS
                          list for anything uncertain (unmatched bodies, missing masses, guessed frame)
  stage vtol              quadplane: per-motor hover load from the real CG, thrust margin in thin air, roll/pitch/yaw
                          authority, crosswind hover, transition, one lift motor out
  stage thermal           lumped thermal network over the mission (hot and cold case, nominal and pessimistic),
                          print softening temperature, vent sizing, parts to instrument
  stage balance           ranked ways to put the CG on target and the lift-thrust centre on the CG, each with pros,
                          cons, a Fusion change list and a variant overlay
  tools/balance_apply.py  turns an option into a plan variant to re-run, and writes the re-positioned STEP
  sourcing policy         meta.sourcing.policy: any | us_made | ndaa screens the parts list and the proposed swaps
  approval gate           derived.param is released into the report only when meta.approval.design_approved is true
                          and the run is GO / CONDITIONAL GO

Workflow, parts-list columns and Fusion export rules: docs/CAD_TO_SIM.md. Windows desktop (WSL2) setup:
docs/DESKTOP_SETUP.md. Worked example: examples/stork_vtol_flightory/. Tests that need no CFD or SITL:
    .venv/bin/python tests/test_cad_balance_thermal.py

Stage order: ingest geometry mass clash aero_lowfi propulsion vtol thermal balance structures cfd flightmodel sitl report


1. COMMAND LINE
---------------
    python run.py PLAN.yaml [--variant NAME] [--fidelity quick|standard|full]
                  [--only STAGE ...] [--until STAGE] [--force STAGE|all ...] [--out DIR] [--quiet]
                  [--calibration CAL.yaml]

    --variant      apply plan.variants.NAME (merge-patch overlay), e.g. vtol, stl_demo
    --only         run just these stages; their upstream results must already exist in the out dir
    --until        run this stage and everything it depends on
    --force        ignore the cache for these stages AND everything downstream of them ("all" = everything)
    --calibration  measured overrides + measured values -> out/<plan>/<variant>_calibrated, plus a
                   predicted-vs-measured table (see section 7)
    --quiet        stage logs go only to <stage>/stage.log (the console shows stage headers and verdicts)


2. ARCHITECTURE
---------------
    run.py                      CLI -> vb/dag.py Runner
    vb/dag.py                   DAG orchestrator: cache keys, one SUBPROCESS per stage (python -m vb.core spec.json)
                                with a timeout. A crash, timeout or exception in a stage is recorded as status
                                "error"; downstream stages that need it are "blocked". The run itself never dies.
    vb/core.py                  stage context (resolved plan, upstream results, gate(), add_file()) and result writer
    vb/plan.py                  plan loader: JSON-Schema validation (raw + variant-resolved), variant merge, soft-input
                                scan, point/box mass model, MAC
    vb/stages/__init__.py       registry: deps / soft deps / hashed plan keys / cheap / fidelity / code files / timeout
    vb/stages/<stage>.py        ingest geometry mass clash aero_lowfi propulsion structures cfd flightmodel sitl report
    vb/adapters/plan2geom.py    plan (SI) -> geometry-library plan (mm, g): surfaces, spars->tubes, booms, components,
                                printed-part mass targets, rotor discs, bays
    vb/adapters/geom2aero.py    plan + mass-stage CG -> aero-library geometry (wing, V-tail, fuselage, controls, VTOL items)
    vb/adapters/model2sim.py    plan + mass + aero dataset + propulsion -> aircraft_model JSON (schema/aircraft_model.schema.json)
    vb/flight_dyn.py            linear modes (short period, phugoid, Dutch roll, spiral, roll) from the derivatives
    vb/figures.py               shared plot style (validated palette)
    vb/lib/geom  (from research/geom)    parametric builder (geomkit), STL/STEP importer, mass properties, fit check,
                                          Blender renders, balance chart
    vb/lib/aero  (from research/aero)    AeroBuildup + AVL + VLM dataset builder, OpenFOAM case generator, CFD STL writer
    vb/lib/struct (from research/struct) propulsion (eCalc-style) and structures (beam theory) models
    vb/lib/sim   (from research/sim)     JSBSim/SIM_Plane generator, SITL<->JSBSim bridge, SITL launcher, test scenarios,
                                          log analysis
    schema/                     buildplan_schema.yaml (MASTER), aircraft_model.schema.json, aero_dataset.schema.json
    data/props/*.json           APC PER3 tables (CT, CP vs J [, rpm]) with source
    data/ardupilot/*.parm       plane_defaults, quadplane_defaults, sitl_plumbing (SITL only), simplane_vtail (fallback)
    setup/                      install.sh, check_env.py, requirements.txt, paths.env (machine-specific, generated)
    tests/smoke.sh              quick end-to-end test
    examples/                   stork_phase0.buildplan.yaml (base + vtol + stl_demo variants), stl_demo/*.stl,
                                stork_calibration_example.yaml

Stage DAG (execution order; deps -> stage):

    ingest
      -> geometry (+renders) -> mass (geometry optional) -> clash (geometry, mass)
      -> aero_lowfi (geometry, mass) -> propulsion (mass; aero drag polar if present)
      -> structures (mass, propulsion: rpm bands, max lift thrust)
      -> cfd (mass; quick = skipped) -> flightmodel (mass, aero, propulsion; structures/cfd optional)
      -> sitl (flightmodel) -> report (everything)

The task brief listed propulsion before aero. aero_lowfi runs first because propulsion needs the drag
polar (docs/PIPELINE_DESIGN.md section 3). Without the aero stage, propulsion falls back to a stated polar and
flags it.

Stage result contract: `out/<plan>/<variant>/<stage>/result.json`

    {stage, status ok|error|skipped|blocked, verdict PASS|WARN|FAIL|ERROR|SKIP, key (cache key), wall_s,
     gates: [{stage, check, value, limit, kind (>=,<=,range,bool,info), status PASS|WARN|FAIL|INFO, unit, note, drivers[]}],
     summary {key figures}, data {machine-readable outputs for downstream stages}, files [{path, caption, kind}],
     notes [], data_hash}

Caching (docs/PIPELINE_DESIGN.md section 6):

- key = sha256 of the hashed plan subset, the stage code (stage + library files), the fidelity (when it
  matters) and the data_hash of every upstream result.
- Cheap stages (ingest, mass, clash, propulsion, structures, flightmodel, report; each 10 s or less)
  always re-run, so the report never shows stale numbers.
- Expensive stages (geometry, aero_lowfi, cfd, sitl) are skipped when their key matches. Their outputs
  are also stored in `out/<plan>/_cache/<stage>/<key>/`, so variants and repeated runs share them.
- The CFD key depends only on the analysed shape, the moment reference and the CFD settings. Base and
  VTOL therefore share the clean-wing CFD.
- A cheap stage that re-runs with identical data keeps its data_hash, so its downstream cache stays
  valid.


3. THE BUILD PLAN (schema/buildplan_schema.yaml is the master; examples/stork_phase0.buildplan.yaml)
------------------------------------------------------------------------------------------------
Units are SI (m, kg, N, s, V, A, Ohm); angles are in degrees. Design frame: origin at the wing-root leading
edge on the centreline, +x aft, +y right, +z up. Any number may be written as
`{value, source, uncertainty}`. A source containing "estimate", "assumed" or "placeholder", or any given
uncertainty, makes it a SOFT input. Soft inputs are listed and ranked in the report.

PyYAML trap: write `1.2e+11`, not `1.2e11`. The second is read as a string, and ingest FAILs it.

| block | contents |
|---|---|
| `meta` | name, revision, phase, base_design, references, notes |
| `airframe.lifting_surfaces[]` | `name`, `kind` (wing, v_tail, htail, fin, canard), `span` (wing: tip-to-tip; V-tail and fin: per panel along the panel), `area`, `root_chord`, `tip_chord`, `sweep_c4_deg` or `sweep_le_deg`, `dihedral_deg`, `twist_tip_deg`, `x_le_root`, `z_le_root`, `airfoil` (UIUC name, NACA4 or .dat path), `thickness_ratio`, `cl_max`, `sections_per_side`, `skin` {material, thickness [m], walls, infill fraction}, `spars[]` {name, tube {od, id, shape, material}, y_start, y_end, x_c}, `control_surfaces[]` {name, function, y_start, y_end, chord_fraction, max_defl_deg, horn_arm, servo_arm, servo (component name), hinge_x_c, servo_rate_deg_s} |
| `airframe.fuselage` | length, max_width, max_height, x_nose; `stations` [[x, width, height, z_centre]...] (superellipse OML), `exponent`, `skin`, `bays` {name: {center, size}} |
| `airframe.booms[]` | name, tube, root, tip, supports[], tip_mass_components[] (mirrored at -y automatically) |
| `airframe.landing` / `airframe.launch` | method, sink_rate, crush_stroke / method (hand, vtol, ...), launch_speed |
| `airframe.cad_import` | real STL/STEP parts (section 4) |
| `materials{}` | kind, density, E, G, strengths, knockdown, tg_C, print settings |
| `components[]` | name, category, part_number, `mass`, `position` (item CG), `dims` [l, w, h], `shape` (box, cylinder + axis), `mirror_y`, `bay`, `external` (may protrude), `geom: false` (point mass only), `geom_part` (printed shell this mass belongs to), `geom_spar` (spars this tube mass belongs to), torque_stall_Nm / supply_V (servos), thermal_limit_C, electrical |
| `propulsion.units[]` | name, role (cruise, lift, tilt), count, positions (lift: ArduPilot quad-X order 1..4), thrust_axis (design frame), motor {kv, rm, i0, i0_ref_V, i_max, p_max, mass, rotor_inertia, loss_limit_W}, prop {model, diameter, pitch, data: "embedded:<data/props key>" or a file path, ct_scale, cp_scale}, esc {i_cont, r_on}, wiring {awg, length, connector, connector_i_cont} |
| `energy` | cell {model, capacity_Ah, r_dc, i_cont, v_nom/max/min, mass, optional ocv_soc/ocv_v}, series, parallel, pack_wiring_r, usable_fraction, reserve_fraction |
| `avionics` | autopilot {board, firmware, firmware_version, frame_class}, avionics_power_W, `params` (overrides that win over derived values), params_file (extra baseline .parm) |
| `envelope` | v_stall_target, v_cruise, v_ne, v_max_level, n_limit_pos/neg, fos, gust_u_de, max_bank_deg |
| `environment_cases[]` | name, altitude_msl, temperature_C, solar_heating_dT_C, wind_mean, wind_gust, turbulence |
| `missions[]` | name, environment, segments [{kind launch, climb, cruise, loiter, ..., distance, duration, altitude, airspeed}], reserve_fraction |
| `analysis` | cg_target_x, cg_tolerance_mac, auw_target, mass_model (geometry, plan), static_margin_range, aero {speeds, excrescence_factor, prop_parking}, cfd {kind wing, wingbody or wingbodytail; alphas_standard; alphas_full; surf_level; iters; model}, sitl {survey_alt, takeoff_alt, l1_base, l1_periods_standard, l1_periods_full, launch_accel, launch_duration, weak_accel, max_wot_s, cruise_speeds}, vtol_auw_cases_kg |
| `gates` | per-project thresholds, e.g. {clash: {min_clearance_mm, prop_clearance_mm}, sitl: {xt_rms_max_m}, propulsion: {installation_factor}} |
| `variants` | name: overlay (merge-patch; arrays of named objects merge by name; `components_add` / `propulsion_add` append) |

Mass model: the plan component list is the single source of masses.

- The geometry stage builds meshes. It scales every printed shell (`geom_part`) and tube (`geom_spar`)
  to the plan mass. The geometry therefore distributes mass, which gives better CG and inertia; it does
  not invent mass.
- The raw shell-model estimate of every printed part is compared with the plan value (WARN above 25 %).
- `analysis.mass_model: plan` uses point and box masses only.


4. ADDING REAL STL / STEP PARTS
-------------------------------
Add `airframe.cad_import` to the plan, or to a variant (see `variants.stl_demo` in the example):

    airframe:
      cad_import:
        folder: my_stl_pack            # relative to the plan file
        replace_structure: true        # parametric printed skins are dropped; spars, components, bays stay
        oml_method: union              # union (manifold) or voxel; union falls back to voxel when the parts do not touch
        defaults: {material: lw_pla_230C, mass_model: shell, wall_mm: 0.45, infill: 0.04, group: printed}
        parts:
          - {name: wing_R, file: "WING 1 R.stl", expected_extent_mm: 800}       # units auto-detected (m / mm / in)
          - {name: wing_L, file: "WING 1 R.stl", transform: {mirror: y}}
          - {glob: "FUSE*.stl", wall_mm: 0.8}                                  # one part per matching file
          - {name: vtail_R, file: tail.stl, units: in, transform: {rotate_deg: [0, 0, 0], translate: [440, 0, 14]}}
          - {name: canopy, file: canopy.step, mass: 12.0}                     # STEP needs build123d; mass in g = weighed
        ignore: ["*support*"]

Notes:

- Files arrive in print orientation, so give each one a `transform` (mirror, rotate_deg, translate in
  mm) or `center_at`.
- Meshes are repaired automatically: flipped normals, holes and open trailing edges.
- `mass_model: shell` means A x wall x rho plus infill x V. `solid` means the full volume x rho, for STLs
  that model every wall. A `mass:` in g overrides both.
- Files in the folder that are not mapped are listed in the geometry stage log.
- Printed-part masses then come from the STLs. The mass stage reports the difference from the plan's
  `structure_printed` components (WARN when it exceeds 3 %). Update the plan, or weigh the parts.
- STEP import needs `pip install build123d` in the venv (not in requirements.txt, because it is large).
- Aero and CFD still use the parametric description in the plan, so keep `lifting_surfaces` and
  `fuselage.stations` consistent with the CAD.
- Test pack: `python run.py examples/stork_phase0.buildplan.yaml --variant stl_demo --until clash`


5. FIDELITY LEVELS AND RUNTIMES (2 vCPU / 7 GB container, measured)
-----------------------------------------------------------------
Wall times measured on this 2-vCPU container. Cold means empty out/ and caches; warm means only the
cheap stages re-run.

| fidelity | geometry | CFD | SITL matrix (fixed wing) | SITL (variant with lift units) | measured wall |
|---|---|---|---|---|---|
| quick | 2 Workbench views | skipped | smoke: hand launch, 4-wp box, RTL (~35 s) | VTOL takeoff, transition, 1 km box, VTOL land (~65 s) | base 68 s cold (tests/smoke.sh); vtol 1.7 min |
| standard | 6 views | `analysis.cfd.kind` (Stork: wing-body), alphas_standard (1 alpha), surf level 5/6, ~0.4 M cells: 6 min | canal mission L1 15 + L1 subset (10) + stall + launch nominal/weak + RC-loss failsafe: 6 runs, 4.7 min | vtol + vtol in the mission wind | base 11.6 min cold; vtol 4.1 min (CFD shared from base cache, SITL 2 runs 3.2 min) |
| full | 7 views | `kind_full` (wing-body-tail), alphas_full (0/4/8), surf level 6/7, ~1-1.5 M cells: est. 1-1.7 h (not timed end-to-end; research l67 wing case: mesh 9 min + 8 min/alpha) | standard + L1 10/13/15/18/22 + cruise sweep + battery and GCS failsafes + mission in every environment case: 15 runs, 20.7 min measured | + every environment case | est. 1.5-2.5 h, dominated by CFD |

Per-stage costs (base):

| stage | time |
|---|---|
| ingest | <1 s |
| geometry | 13 s (quick) / 33 s (standard; Blender renders ~5 s/view) |
| mass | 1 s |
| clash | 2 s |
| aero_lowfi | 7 s (AeroBuildup 3 speeds + AVL 5 alphas + VLM) |
| propulsion | 4 s |
| structures | 1 s |
| flightmodel | 1 s |
| report | 3-6 s |

- SITL runs at about 2-4.5x real time.
- A warm re-run of an unchanged plan takes about 20 s.
- A plan edit re-runs only the stages whose hashed inputs changed. For example, editing a component
  mass re-runs geometry, aero and SITL, but not CFD unless the shape changed.
- Base and VTOL share the CFD cache entry, because the moment reference is the design CG.
- The first-time install takes about 2 min plus 20-30 min for the ArduPilot build. The other steps
  (venv from the pip cache, AVL 20 s, xfoil-python 30 s) were measured in a scratch copy.

Sub-agents / sharding:

- Each stage reads only `result.json` files, so `--only STAGE` on a shared out dir works.
- `VB_SITL_TESTS=fs_batt,stall` restricts the SITL matrix to named runs. The subset is part of the
  cache key.
- Parallel SITL runs pick a free instance automatically (`-I n`, ports 5760+10n / 9002+10n).


6. GATES AND THE VERDICT
------------------------
Defaults follow docs/PIPELINE_DESIGN.md section 4: ingest validity; CG within +-3 % MAC; fit (bays, protrusions,
interference, >= 2 mm clearance, rotor clearance >= 10 mm, battery travel can fix the CG); static margin
5-20 %; Cm_a < 0, Cn_b > 0, Cl_b < 0; linear modes; roll authority; V-tail volumes; stall speed vs hand
launch; ESC/connector/wire/cell/motor current margins; motor thermal; cruise duty <= 0.65; T/W; mission
energy with reserve; VTOL T/W >= 1.6 (FAIL < 1.3) and hover duty <= 0.7; spar margins at ultimate; tip
deflection; EI steps; skin wrinkling and stress; divergence and reversal; frequency separation; servo
torque >= 2; booms; Tg - 10 C vs the hottest skin; airspeed parameters vs Vs and VNE; and SITL (completion,
stall/overspeed/load factor, launch, failsafes, path tracking, energy calibrated by SITL, hover throttle).

Verdict rules:

- NO-GO: any FAIL, or a stage that errored or was blocked.
- INCOMPLETE: a stage that did not run.
- CONDITIONAL GO: only WARNs remain. Each WARN is a risk-register row whose owner is blank until the
  team assigns one.
- GO: everything PASS.

GO is never printed with an unresolved FAIL.

Every WARN or FAIL becomes a row in `risk_register.csv`, with L x S, a mitigation and a verification step.
The soft-input table ranks the estimates by how many non-PASS gates they drive, through each gate's
`drivers` list.


7. CALIBRATION LOOP
-------------------
Bench and flight measurements replace estimates, step by step (docs/PIPELINE_DESIGN.md section 8):

| step | measurement | what it updates |
|---|---|---|
| C0 | weigh every part, then balance the aircraft | `components[].mass` (source "measured <date>") and positions |
| C1 | pendulum | inertia |
| C2 | thrust stand at 3 voltages | `prop.ct_scale`, `prop.cp_scale`, `motor.rm`, `motor.i0` |
| C3 | discharge test | `cell.r_dc`, OCV curve (`ocv_soc`, `ocv_v`) |
| C4 | sandbag proof load + tap test | material E, knockdown |
| C5 | servo current and linkage slop | horn ratios |
| F2 | glide polar | `analysis.aero` overrides of CD0 / e |
| F3 | cruise legs | drag polar + propulsion check |
| F4 | doublets / AUTOTUNE | control and damping derivatives |
| F5 | VTOL hover logs | hover throttle, notch centre |

There are two ways to feed the measurements back:

1. Edit the plan (preferred for permanent values) and re-run. Cheap stages re-run automatically;
   expensive ones re-run only when their inputs changed.
2. Write a calibration file (example: `examples/stork_calibration_example.yaml`). It has an `overrides`
   overlay, merged like a variant, and a `measured` block: auw_kg, cg_x_mm, static_thrust_g_full,
   static_current_A_full, cruise_current_A, hover_throttle, v_stall, cd0, f_bend_Hz, f_tors_Hz,
   wh_per_km. Run it with:

       python run.py PLAN --calibration CAL.yaml

   The report gains a "predicted vs measured" table with acceptance bands. Every quantity outside its
   band becomes a WARN/risk row of type "model error".

After the first flights, replay the flown mission in SITL with the calibrated model. Phase-0
calibrations carry into the VTOL variant automatically, because the variant is an overlay of the same plan.


8. KNOWN LIMITATIONS
--------------------
- Aero (AeroBuildup / AVL) and CFD geometry supports one main wing plus a V-tail plus a fuselage, and
  optional VTOL booms and rotors. The geometry builder also handles htail/fin/canard surfaces, but
  aero_lowfi raises NotImplementedError for them. A conventional tail needs a small extension of
  `vb/lib/aero/stork_aero.py`: two extra surfaces in build_asb and write_avl.
- Linear modes use the cruise derivatives only (no altitude or Mach effects).
- Static margin uses the AVL neutral point. AeroBuildup's own Cm_alpha is too stable (no downwash) and
  has been replaced.
- CFD answers one named question: does the low-order model predict lift and Cm on the same shape?
  Fully turbulent SST over-predicts the drag of laminar S3021-class sections by about 50 %, so CFD drag
  is information only. CFD results are not fed into the flight model; the CL ratio is recorded in the
  model provenance. Steady RANS cannot predict CLmax or stall.
- Propulsion:
  - Prop data are APC PER3, computed rather than measured. The plan's `ct_scale` 0.92 is a prior; C2
    replaces it.
  - The 7x5E table has a single rpm block.
  - The motor thermal model is lumped, with R_th 1.5 K/W (estimate).
  - The mission energy model is steady-state per segment, with out-leg headwind and back-leg tailwind.
    SITL then rescales it with the measured Wh/km of the 6-DOF flight.
- Structures: beam theory with Schrenk loading. Skin GJ comes from a closed printed shell, and the
  panel width for wrinkling is an estimate (20 mm). There is no FEA of printed parts. CalculiX is only
  used as a method cross-check (vb/lib/struct/calculix_check.py, Stork tubes) and does not run per plan.
  Flutter is screened with frequency ratios, not a flutter solve.
- Fit/clash:
  - Components are boxes or cylinders from `dims`; real component CAD can be mapped through cad_import
    with role: component.
  - The battery insertion path is checked as fore-aft travel only. Connector access and the hatch are
    not modelled.
  - Servo and linkage swept volumes are NOT modelled.
- SITL:
  - The canal S-course from the prototype (1.9 km, bends R300 to R50) stands in for every mission. Plan
    mission energy is extrapolated from its measured Wh/km.
  - The SITL home is ArduPilot's CMAC field.
  - Gains in data/ardupilot/plane_defaults.parm are sane defaults, not a tune. AUTOTUNE is not run in
    SITL.
  - QuadPlane runs use SIM_RATE_HZ 1200 and the same 4S pack for lift and cruise.
  - The weak-throw case (15 m/s^2 for 0.4 s, released at 6 m/s, below Vs) crashes even though the
    motor starts after 0.5 s. It is a WARN that drives a training item (throw >= 10 m/s), not a FAIL.
- The soft-input ranking is gate-driver matching, not a Monte-Carlo or tornado sensitivity analysis.
  The `uncertainty` fields are recorded but no corner cases are run yet (docs/PIPELINE_DESIGN.md section 6:
  N=50 LHS for cheap stages is a planned extension).
- One SITL instance runs at a time; stages run sequentially. Sub-agents can shard with --only on separate
  out dirs.
- Renders use Blender Workbench, about 5 s per view. Cycles is available in vb/lib/geom/render.py
  (--engine CYCLES) but is not wired to a fidelity level.
- Python 3.11-3.13. Only Ubuntu 24.04 has been tested. OpenFOAM is the apt v1912 package, which needs
  the LD_PRELOAD shim. Newer OpenFOAM repositories were blocked from the build machine.


9. EXAMPLE PLAN PROVENANCE (how conflicts between the prototypes were resolved)
------------------------------------------------------------------------------
Sources: research/struct `stork_phase0.buildplan.yaml` (struct), research/geom `stork.yaml` (geom),
research/aero (aero), research/sim `models/stork_p0.json` (sim).

| item | struct | geom / aero / sim | chosen | why |
|---|---|---|---|---|
| wing planform | 1.62 m, 0.31 m2, 230/153 mm | same | same | manual |
| wing dihedral | 2 deg (estimate) | 0 (straight 1000 mm centre spars) | 0 | physical argument from the spar layout |
| washout, wing z, sweep | - | -1.5 deg, 22 mm, 0 | geom | only source (ASSUMED) |
| V-tail | 0.265 panel, 150/100, x 0.46 | 0.300 panel, 150/95, x 0.44, sweep 20 | geom | aero dataset and renders were validated with it (ASSUMED) |
| fuselage | 1.00 m box, x_nose -0.42 | 11 stations, x_nose -0.335, bays from the manual | geom | sourced bay size (manual 220 x 80 x 80) |
| LW-PLA density | 750 kg/m3 (CNC Kitchen @230 C) | 0.55 g/cc nominal | 750 | sourced, matches the print temperature |
| printed part masses | 380 / 2x140 / 2x30 g (estimate) | geometric 407 g at 0.55 g/cc | geometry shell model at 750 kg/m3: 180 / 2x154 / 2x33 g | derived from sourced density + geometry; still SOFT (weigh, C0) |
| battery | 600 g (8 x 69-71 g + leads), 145 x 45 x 72 | 595 g, 90 x 46 x 73 in the battery bay | 600 g, 90 x 46 x 73 | sourced mass; dims fit 4S2P 21700 standing |
| battery x | -77 mm | -210 mm | -210 mm (forward end of the bay) | puts the CG within +-3 % MAC of the manual's 47 mm |
| motor | 77 g, Kv 910 | 76.5 g (getfpv) | 76.5 g | listing |
| avionics / payload | fc, gps 20, airspeed, rx | fc, pi zero, camera, gps 8, telemetry, remote ID | union (geom list + airspeed sensor), gps 20 g conservative | union of the two parts lists |
| aileron servo position | x 0.12 | 55 % chord | x 0.058 (25 % chord) | ES08MA laid flat protrudes at 55 % chord (clash stage FAIL in the first run) |
| control surfaces | aileron 0.38-0.78, 25 % chord | aero added flaps (ASSUMED) | struct (no flaps) | struct values tied to servo checks; the manual lists no flaps |
| motor Rm / I0 | 80 mOhm / 0.85 A (2810 data) | 85 mOhm / 0.8 A (placeholder) | struct | closer to a datasheet |
| ct_scale | 0.92 | thrust_scale 0.9 | 0.92 | struct (prior) |
| AUW | built up | 1.775 kg placeholder | built up: 1.72 kg (base), 2.17 kg (VTOL) | the component list is the source |
| lift motor positions | +-0.30 m, x -0.20 / 0.30 | +-0.35 m (sim), x -0.195/0.275 (geom) | struct | planned boom geometry |
| ArduPilot params | 4.4-era names (ARSPD_FBW_*, TRIM_ARSPD_CM) | 4.7 names | 4.7 names | SITL firmware Plane-4.7.2; ingest WARNs on legacy names |


10. PITFALLS ALREADY HANDLED (do not re-learn them)
--------------------------------------------------
- OpenFOAM v1912 (apt): every function object aborts on dictionary::digest(). `tools/foam_sha1_shim`
  is LD_PRELOADed. The package's scotch is a stub, so the runs use hierarchical decomposition.
- The apt xfoil cannot run headless. Use xfoil-python, built from source (the PyPI sdist is broken).
- NeuralFoil's built-in flap model only shifts alpha, so the aero library deflects the geometry instead.
- AeroBuildup has no downwash, so its Cm_alpha is too stable. The linear part of Cm(alpha) is replaced
  by AVL's.
- AVL needs left and right surfaces written separately, with sections ordered by increasing y. Only then
  is positive deflection trailing-edge-down on every surface.
- The ArduPlane SITL JSON backend ignores SIM_WIND_*, so the bridge applies wind and turbulence.
- SIM_Plane's JSON loader panics on absolute paths. Hard-coded 2 kg, so it is a fallback only.
- With Li-ion sag, a voltage-only battery failsafe trips during the climb. derived.param therefore uses
  BATT_LOW_MAH plus BATT_LOW_TIMER.
- Weak hand throws: GPS speed lags, so derived.param sets TKOFF_THR_MINSPD to 3.
- `pkill -f <pattern>` can kill your own shell. SITL processes are stopped by PID.
