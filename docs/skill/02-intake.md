# Playbook 2 - Intake: ask, then CAD + parts list -> plan

The plan is the single source of truth for every stage. For a design that exists as CAD, build it with
`tools/intake.py`: hand-copied positions are where CG errors come from. `docs/CAD_TO_SIM.md` has the Fusion
export rules and every parts-list column; read it once and give it to the team if their export does not match.

## Ask first (one round; skip what they already said)

| Ask | Why it matters | Default if they do not care |
|---|---|---|
| **May any part be used, or only US-made parts?** | screens the parts list and limits the swaps the balance stage may propose. ALWAYS ask. | none: do not assume |
| Where is the wing-root leading edge in CAD, and which CAD axes point aft, right, up? | every position and the CG depend on it | let the intake guess, show a top view, get a yes |
| Print material and settings per part, or slicer weights? | printed mass is the largest soft input | LW-PLA; slicer or scale weights always win |
| Hottest and coldest day, wind, field altitude, mission legs, payload? | thrust margin, thermal and range are judged in the worst case of the plan | the assumed Texas cases in the example, flagged as assumed |
| What may the optimiser change? | which balance options are worth showing | move parts, change the frame, swap parts; ballast last |

Sourcing is a screening aid, not a compliance ruling: say so when you report it.

## Run the intake

```
python tools/intake.py --cad assembly.step --parts parts.csv --base BASE_PLAN.yaml [--variant vtol] \
       --frame "origin=X,Y,Z; x_aft=+z; y_right=+x; z_up=+y" --policy any|us_made|ndaa \
       [--merge datasheets.yaml] [--cad-import] --out work/<name>
```

- `--base`: what neither CAD nor a parts list can give (aerofoil and planform as analysed, motor / propeller / cell data, mission, environment). Stork derivatives: `examples/stork_phase0.buildplan.yaml`. Worked example: `examples/stork_vtol_flightory/`.
- `--merge`: datasheet values for any new motor, ESC, propeller or cell, each with its source. Look them up (manufacturer pages); never invent them.
- `--cad-import`: use the real printed shapes for renders, fit and mass distribution (do this for real CAD; the box-and-cylinder example does not need it).

## The QUESTIONS list

Exit code 2 means the intake has questions: unmatched bodies, rows with no mass or position, quantity
mismatches, a guessed frame, CAD dimensions that disagree with the base plan, no sourcing policy. Take every
one to the user. Do not paper over a question with an assumption: a wrong body match moves the CG silently.

## Rules for the plan

- Every uncertain number is `{value, source: "estimate ...", uncertainty}`. An assumption in a YAML comment is invisible to the soft-input table and the corner cases.
- Frame: origin at the wing-root leading edge on the centreline; x aft, y right, z up; SI units. `position` is the part's own CG.
- Wing-mounted pairs are one component with `mirror_y`. Variants are overlays, so every variant shares the base airframe numbers.
- YAML exponents: `1.2e+11` (PyYAML reads `1.2e11` as text).

## Other ways in

- **Paper design (manual + parts list, no CAD):** copy the nearest example plan and fill it by hand from the manual (span, area, chords, aerofoil, CG, spars) and the parts list. Then the design loop as usual.
- **Print STL pack, no assembly:** `airframe.cad_import` (see the `stl_demo` variant): part -> transform, material, wall thickness, infill. Manufacturer STLs are laid out for printing, so set transforms or `center_at`. Open seams are closed automatically and reported.
- **A change to a design already taken in:** edit the parts list or re-export the STEP and run the intake again into the same folder. Never patch positions in the plan by hand when CAD exists.
