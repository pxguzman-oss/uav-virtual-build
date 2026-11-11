# Design decision brief - Stork VTOL payload test (base)

Desk check only (layout and sizing phases, no CFD or flight simulation). 2026-11-10 19:48. Numbers are uncalibrated until the team's measurements are fed back.

## Where the design stands as drawn

- Weight 2294 g; CG -34.5 mm from the design CG (OUTSIDE the allowed window).
- Phase 1 layout: 2 FAIL, 4 WARN. Phase 2 sizing: 12 FAIL, 30 WARN.

## Candidates, each proven by a desk check

| # | change | CG error mm | static margin % | CG to lift-thrust centre mm | hover T/W (std day / worst air) | range km (pessimistic-nominal) | parts over temperature | layout FAIL | sizing FAIL | fixes | new problems |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | As drawn | -34.5 | 38.5 | -37.5 | 1.58 / 1.18 | 69-89 | 4 | 2 | 12 | 0 | 0 |
| A (recommended) | Move the wing 52 mm forward on the fuselage | -0.0 | 23.2 | -3.0 | 1.58 / 1.18 | 69-89 | 4 | 0 | 11 | 3 | 0 |
| B | Add 146 g of ballast in the tail | 0.0 | 21.1 | -3.0 | 1.49 / 1.11 | 66-85 | 4 | 0 | 11 | 3 | 0 |
| C | Move internal parts as far as they go, then 49 g of ballast in the tail | 0.0 | 20.9 | -3.0 | 1.55 / 1.16 | 68-87 | 4 | 0 | 11 | 3 | 0 |
| D | Swap the battery for 4S1P Molicel INR21700-P50B (300 g) | -0.4 | 20.6 | -3.4 | 1.52 / 1.16 | 37-48 | 4 | 0 | 13 | 5 | 4 |

## Recommendation

**A - Move the wing 52 mm forward on the fuselage** (variant `balA`): fewest layout FAILs, then fewest sizing FAILs and new problems, then least effort. This is a ranking rule, not a decision: the team chooses.

### A - Move the wing 52 mm forward on the fuselage

Pros: CG -0.0 mm from target (inside the +-6 mm window); CG -3 mm from the lift-thrust centre (front motors carry 51 %)

Cons: static margin 22 % MAC (outside the plan's 5-20 % range: sluggish in pitch, more trim drag); frame change: re-print and re-validate; re-print the wing saddle / centre fuselage; re-run aero, CFD and SITL (tail arm changes); tail arm 465 -> 517 mm (+11.2 % tail volume)

Estimate vs proven: CG error -0.0 -> -0.0 mm; static margin 22.4 -> 23.2 %.

Fixes: CG x vs design CG [mm] (tolerance +-5.8 mm = 3% MAC) (mass); battery travel in its bay can put the CG on target (clash); most-loaded lift motor: hover thrust / average (vtol)

### B - Add 146 g of ballast in the tail

Pros: CG -0.1 mm from target (inside the +-6 mm window); CG -3 mm from the lift-thrust centre (front motors carry 51 %)

Cons: static margin 21 % MAC (outside the plan's 5-20 % range: sluggish in pitch, more trim drag); AUW +146 g; hover thrust-to-weight 1.58 -> 1.49; range about -5 % (estimate); ballast is a last resort; dead weight: costs range and hover margin on every flight

Estimate vs proven: CG error -0.1 -> 0.0 mm; static margin 20.7 -> 21.1 %.

Fixes: CG x vs design CG [mm] (tolerance +-5.8 mm = 3% MAC) (mass); battery travel in its bay can put the CG on target (clash); most-loaded lift motor: hover thrust / average (vtol)

### C - Move internal parts as far as they go, then 49 g of ballast in the tail

Pros: CG -0.0 mm from target (inside the +-6 mm window); CG -3 mm from the lift-thrust centre (front motors carry 51 %)

Cons: static margin 21 % MAC (outside the plan's 5-20 % range: sluggish in pitch, more trim drag); AUW +50 g; hover thrust-to-weight 1.58 -> 1.55; range about -2 % (estimate); ballast is a last resort; dead weight, but far less than ballast alone

Estimate vs proven: CG error -0.0 -> 0.0 mm; static margin 20.7 -> 20.9 %.

Fixes: CG x vs design CG [mm] (tolerance +-5.8 mm = 3% MAC) (mass); battery travel in its bay can put the CG on target (clash); most-loaded lift motor: hover thrust / average (vtol)

### D - Swap the battery for 4S1P Molicel INR21700-P50B (300 g)

Pros: CG -0.0 mm from target (inside the +-6 mm window); CG -3 mm from the lift-thrust centre (front motors carry 51 %); AUW -300 g; hover thrust-to-weight 1.58 -> 1.82

Cons: static margin 21 % MAC (outside the plan's 5-20 % range: sluggish in pitch, more trim drag); range about -44 % (estimate); new part to buy and test; origin: Taiwan (unverified); re-run propulsion, thermal and SITL: voltage sag, current per cell and hover time change

Estimate vs proven: CG error -0.0 -> -0.4 mm; static margin 20.7 -> 20.6 %.

Fixes: CG x vs design CG [mm] (tolerance +-5.8 mm = 3% MAC) (mass); VTOL hover ESC duty at max AUW case 3.10 kg (propulsion); battery travel in its bay can put the CG on target (clash); most-loaded lift motor: hover thrust / average (vtol); thermal: lw_pla_230C: Tg - 10 C margin vs skin temp [C] (scenario: 'west_texas_summer' ground soak in sun) (structures)

NEW problems: mission 'canal_inspection_out_and_back' energy vs usable after 20 % reserve [Wh] (propulsion); per-cell current at WOT / cell continuous rating (propulsion); per-cell current, all lift motors WOT / cell rating (propulsion); structures stage crashed: KeyError: 'wot_thrust_g'

## Still open whichever option is chosen

These FAILs are in every candidate, so no balance option removes them:

- AIRSPEED_MIN / PESSIMISTIC Vs >= 1.2 (flightmodel)
- VTOL T/W at max AUW case 3.10 kg >= 1.6 x 1.10 (stand-in prop data margin) (propulsion)
- air in 'battery_bay' vs printed-skin softening (Tg 55 C - 10) [C] (thermal)
- air in 'electronics_bay' vs printed-skin softening (Tg 55 C - 10) [C] (thermal)
- air in 'fuselage' vs printed-skin softening (Tg 55 C - 10) [C] (thermal)
- camera peak temperature in 'west_texas_summer' [C] (thermal)
- flight_controller peak temperature in 'west_texas_summer' [C] (thermal)
- telemetry_rx peak temperature in 'west_texas_summer' [C] (thermal)
- vtx peak temperature in 'west_texas_summer' [C] (thermal)
- lift T/W in the thinnest air of the plan: 'west_texas_summer' (45 C, 1200 m), 10 % battery (vtol)

## Next

1. Team picks a candidate. Reports per candidate: `out/stork_vtol_payload_test/<variant>/report/index.html` (opens with the three questions and a what-to-do-next list grouped by who acts).
2. Flight proof (CFD + SITL): `python tools/design_loop.py examples/stork_vtol_payload/stork_vtol_payload_test.buildplan.yaml --fly balA`
3. Moved CAD for the chosen option: `python tools/balance_apply.py examples/stork_vtol_payload/stork_vtol_payload_test.buildplan.yaml --from out/stork_vtol_payload_test/base/balance --option <id> --step-out balanced.step --cad-map <intake folder>/cad_map.json`
4. When the team approves a design that is GO or CONDITIONAL GO, set `meta.approval` in the plan and re-run the report to release the ArduPilot parameter file.
