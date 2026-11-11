# Playbook 3 - The design loop: desk check, balance, decide

## The three questions, cheapest first

| Phase | Question | Stages | Time |
|---|---|---|---|
| 1 layout | Does it add up? | ingest, geometry, mass, clash, balance | seconds |
| 2 sizing | Does it work on paper? | aero_lowfi, propulsion, vtol, thermal, structures | ~1 min |
| 3 flight | Does it fly in simulation? | cfd, flightmodel, sitl | minutes to hours |

Work them in order. A layout that does not balance or fit changes every number after it, so a sizing or flight
result on an unbalanced layout is noise. That is why the loop refuses to flight-prove a candidate that still
has a layout FAIL.

## One command

```
python tools/design_loop.py PLAN.yaml [--variant V] [--max-options 4] [--options B,C] [--fly best|ID|VARIANT]
```

1. Desk-checks the design as drawn (phases 1 and 2).
2. If the layout does not balance: adds the best balance options to the plan as variants (`balA`, `vtol_balB`, ...) and desk-checks each. The comparison therefore shows proven numbers; the balance stage's own table is estimates.
3. Writes `out/<plan>/design_loop/DECISION.md` and `comparison.json`: per candidate CG, static margin, CG to lift-thrust centre, hover margin (standard day / worst air), range band, parts over temperature, FAILs per phase, what it fixes, what it newly breaks, and the FAILs no option removes.
4. Recommends by rule: fewest layout FAILs, then fewest sizing FAILs and new problems, then lowest cost score (effort, range and hover margin lost; ballast penalised).

Exit 0: a candidate clears the layout phase. Exit 2: none does.

## What the balance stage can propose

Slide the battery; re-arrange internal parts (least total travel, parts may not pass through each other);
moves plus the least ballast; move the lift motors / booms; move the wing on the fuselage; extend the battery
bay; swap the battery (same cell with one parallel group more or fewer, catalog cells, parts-list alternatives,
all filtered by the sourcing policy); ballast alone. Targets: the plan's design CG +- tolerance, and for a
quadplane the lift-thrust centre on the CG.

## Presenting it (STOP: the team decides)

- Show the comparison table and, for each candidate that clears the layout, the pros and cons in plain words. Several options, never one.
- Say which you would pick and why, and say that the recommendation is a ranking rule.
- Point out "new problems": an option that fixes the CG and breaks something else (a battery swap that cannot hover at the heavy case, a move that puts the ESC against the battery) is the reason options are proven before they are offered.
- List what stays open whichever option is chosen (thermal, thrust margin in hot thin air, material): no balance option fixes those.
- Do not start a long flight proof on a candidate the team has not chosen, unless they asked for the recommended one to be run.

## After the choice

- Flight proof: `python tools/design_loop.py PLAN --fly <variant>` (playbook 4).
- Moved CAD: `python tools/balance_apply.py PLAN --from out/<plan>/<variant>/balance --option <id> --step-out balanced.step --cad-map <intake>/cad_map.json`. The STEP contains part moves only. Frame changes (wing, boom, bay) are in `fusion_change_list.csv` for a designer to apply.
- When the proving run turns up something new, fix the parts list or CAD, re-run the intake and the loop. Do not stack hand edits on a variant.

## When no option works

Exit 2. Read the balance stage's reach (`internal parts can move the CG only ...`). Usual causes: a heavy
fixed part far from the CG, a bay too short, travel limits missing from the parts list (`move_min_mm` /
`move_max_mm`, `bay`). Go back to the team with the specific constraint to relax, not a generic "does not balance".

## Smaller questions

- "Is the CG right / does it fit?" -> `python run.py PLAN --phase layout` (seconds). Answer that, and mention in one line if the full desk check would add FAILs.
- "Is this ESC / battery / motor enough?" -> `python run.py PLAN --phase desk`, read the propulsion, vtol and thermal sections.
