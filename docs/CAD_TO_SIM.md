# From a CAD assembly and a parts list to a simulated, balanced aircraft

This is the workflow for a team that designs in Fusion, 3D-prints the airframe and wants to know, before
printing, whether the aircraft balances, hovers, transitions, cruises, stays cool and has the range.

```
Fusion assembly (STEP) + parts list (CSV/XLSX) + base plan
        |  tools/intake.py                 names matched, masses and CGs computed, questions listed
        v
   build plan (YAML)
        |  tools/design_loop.py            1 LAYOUT  does it add up?         (seconds)
        |                                  2 SIZING  does it work on paper?  (~1 min)
        |                                  every promising balance option is added as a variant and desk-checked
        v
   DECISION.md: candidates side by side with proven numbers, what each fixes and breaks, what stays open
        |  the team picks a candidate
        v
   tools/design_loop.py --fly <variant>    3 FLIGHT  does it fly in simulation?  CFD + ArduPilot SITL (30 min to hours)
        |  tools/balance_apply.py --step-out    moved CAD back into Fusion
        |  team approves                         meta.approval.design_approved: true
        v
   report with the ArduPilot parameter file released
```

The order matters: a layout that does not balance changes every number after it, so the loop settles balance
with one-minute desk checks and spends CFD and flight simulation only on the candidate the team chose.

## 1. What to export from Fusion

* One STEP of the whole assembly (File > Export > STEP). Every printed part and every bought component is its
  own body or component. Per-part STEP files are welcome as well (`--cad folder`), but positions are taken
  from the assembly.
* Name every body or component so that it STARTS WITH the part id used in the parts list:
  `P030_battery_6S`, `P020_lift_motor`. Fusion's version and instance suffixes (` v3`, `:2`) are ignored.
  Without ids the intake falls back to name similarity and asks about anything below 0.8.
* Model bought parts at their real outer size (a box or cylinder is enough). Their bounding box is used
  for the fit check, the thermal surface area and the travel limits.
* Tell the intake where the datum is: the wing-root leading edge on the centreline, in CAD millimetres, and
  which CAD axis points aft, right and up: `--frame "origin=0,5,335; x_aft=+z; y_right=+x; z_up=+y"`.
  Without `--frame` the intake guesses from the bodies named wing/tail and lists the guess as a question.

## 2. The parts list

One row per part. Columns (header names are case-insensitive; extra columns are ignored):

| column | meaning |
|---|---|
| `id` | part id that starts the CAD body name (`P030`) |
| `name` | description |
| `plan_name` | component name to use in the plan. Needed when the plan refers to the part by name (servos of control surfaces, booms, spars). `{pos}` becomes FR/RR for a set of four |
| `category` | structure_printed, structure_tube, propulsion, energy, avionics, servo, payload, wiring, hardware, ballast |
| `role` | lift_motor, cruise_motor, lift_esc, cruise_esc, battery, fc, gps, vtx ... (motor roles set the motor positions of the propulsion units) |
| `qty` | bodies expected in CAD (checked) |
| `mass_g`, `mass_source` | mass of ONE unit and where the number comes from. "estimate" in the source marks it soft |
| `material`, `print_mode`, `wall_mm`, `infill`, `slicer_g` | printed parts. `slicer_g` (slicer or scale weight) wins. Otherwise mass = density x CAD volume for thin-wall bodies (`print_mode: solid`), walls + infill for solid bodies (`shell`); the mode is chosen from the mean wall thickness when the column is empty |
| `bay` | fuselage bay it sits in (travel limits, thermal) |
| `movable`, `move_min_mm`, `move_max_mm` | may the optimiser slide it, and between which CG x positions |
| `power_w`, `t_max_c` | heat when powered, and the temperature limit (thermal stage) |
| `external` | yes for parts outside the skin |
| `origin`, `origin_verified` | country of manufacture and whether the team has confirmed it (sourcing policy) |
| `vendor_part` | make and model |
| `x_mm`, `y_mm`, `z_mm` | position in the design frame for parts with no CAD body (wiring, hardware) |
| `extra` | YAML merged into the component, e.g. `{geom_spar: main_center}` or `{torque_stall_Nm: 0.157}` |

The base plan (`--base`, optionally `--variant` and `--merge overlay.yaml`) supplies what neither CAD nor a
parts list can: the aerofoil and planform as analysed, motor/propeller/cell data, mission, environment cases.
Example: `examples/stork_vtol_flightory/` (parts list from the Flightory Stork VTOL page, a synthetic STEP
stand-in for the paid CAD pack, `datasheets.yaml`).

Ask the user which parts are allowed and pass it as `--policy any | us_made | ndaa`. It is stored in
`meta.sourcing.policy`, screens the parts list in the ingest stage and filters the swaps the balance stage
proposes. It is a screening aid, not a compliance determination.

The intake exits with code 2 and a QUESTIONS list when something is not certain: unmatched bodies, rows
without mass or position, quantity mismatches, a guessed frame, CAD dimensions that disagree with the base
plan. Settle them with the user before trusting a run.

## 3. Auto-balance

The `balance` stage targets the plan's design CG (`analysis.cg_target_x` +- `cg_tolerance_mac`; without a
design CG, the middle of `analysis.static_margin_range`) and, for a quadplane, wants the lift-thrust centre
on the CG. It proposes, cheapest first:

| option | what it changes |
|---|---|
| slide the battery | battery CG x inside its bay |
| re-arrange internal parts | least total travel of all movable parts; parts sharing a footprint may not pass through each other |
| moves + ballast | as above, then the smallest ballast for the remainder |
| move the lift motors/booms | thrust centre onto the design CG (frame change) |
| move the wing on the fuselage | everything not on the wing shifts; tail arm and static margin change (frame change) |
| extend the battery bay | when the battery runs out of travel by a few cm (frame change) |
| swap the battery | same cell with one parallel group more or fewer, catalog cells (`data/catalog/batteries.yaml`), parts-list alternatives; filtered by the sourcing policy |
| ballast | last resort, with its range and hover-margin cost |

Each option reports CG error, static margin, CG-to-thrust-centre offset, AUW change, a range estimate,
pros, cons and a change list (`fusion_change_list.csv`: item, CAD body, from, to, delta, why). The ranking is
a heuristic (effort + range cost + what is left unsolved): the team chooses.

`python tools/design_loop.py PLAN` proves the best options automatically: each becomes a plan variant
(`<variant>_balB`), is desk-checked, and lands in `out/<plan>/design_loop/DECISION.md` next to the others.
`python tools/balance_apply.py PLAN --from out/<plan>/<variant>/balance --option B` does the same for one
option by hand. With `--step-out file.step --cad-map <intake>/cad_map.json` it also writes the assembly with the moved
bodies, to import back into Fusion. Frame changes are not applied to the STEP; they are in the change list.

## 4. What the vtol and thermal stages add

* `vtol`: load on each lift motor from the real CG, thrust margin in the thinnest air of the plan with a low
  battery, roll/pitch/yaw authority, crosswind hover (tilt, weathervane vs yaw authority), transition time,
  distance and battery current, back-transition distance, one lift motor out.
* `thermal`: lumped network of battery, ESCs, electronics, bay air, printed skin and motors over the mission
  timeline in the hottest and coldest case, nominal and pessimistic. It checks part limits, the print's
  softening temperature (LW-PLA softens near 55 C), parts touching the print, motor mounts, how long it can
  hover, and sizes bay vents for forward flight. Parts within 10 K of a limit are the list for detailed
  CFD or a thermocouple. Conjugate heat-transfer CFD of a bay is NOT automated yet.

## 5. Approval and the parameter file

The report shows the derived ArduPilot parameters for review at every run, but the loadable `derived.param`
is copied into the report folder only when the plan says the design is approved AND the run is GO or
CONDITIONAL GO:

```yaml
meta:
  approval: {design_approved: true, by: "name", date: "2026-11-12"}
```
