# Playbook 4 - Flight proof: CFD and simulated flight

Only for a candidate that cleared the layout phase and that the team chose (or asked you to run).

```
python tools/design_loop.py PLAN --fly <variant> [--fidelity standard|full]
python run.py PLAN --variant <variant> [--fidelity quick|standard|full]      # same thing, no layout guard
```

| Fidelity | What runs | 2 cores | Verdict |
|---|---|---|---|
| quick | no CFD, one smoke flight | ~2 min | always INCOMPLETE |
| standard | wing-body CFD at 2 angles, SITL matrix calm + mission wind | ~30 min cold, ~12 min warm | GO / CONDITIONAL GO / NO-GO |
| full | wing-body-tail CFD at 3+ angles, every environment case and failsafe | 1.5-3 h | as standard, more evidence |

Options: `--phase layout|desk`, `--only STAGE`, `--until STAGE`, `--force STAGE|all`, `--out DIR`, `--calibration CAL.yaml`.
Cheap stages always re-run; geometry, aero, CFD and SITL are reused when their inputs are unchanged.

Stage order: `ingest geometry mass clash aero_lowfi propulsion vtol thermal balance structures cfd flightmodel sitl report`.

## Sub-agents (standard and full runs with several variants)

One per variant up to the flight model; CFD as its own job; SITL per variant (it picks a free instance); then
the report and the independent reviewer. Narrow brief plus the repo path; results are shared only through
`out/<plan>/<variant>/<stage>/result.json`. Heavy jobs at once: 2 on 2 cores, 3 on the desktop.

## What the newer stages check

- **vtol** (quadplane 4+1): load on each lift motor from the real CG, thrust margin in the thinnest air of the plan with a low battery, roll / pitch / yaw authority, crosswind hover, transition time / distance / current, back-transition distance, one lift motor out.
- **thermal**: lumped network (battery, ESCs, electronics, bay air, printed skin, motors) over the mission, hottest and coldest case, nominal and pessimistic. Part limits, print softening (LW-PLA softens near 55 C), parts touching the print, motor mounts, how long it can hover, vent sizing. Conjugate heat-transfer CFD is not automated: for a flagged bay, set it up by hand in OpenFOAM and say it is a one-off.

## Reading the result

The report opens with the three questions and "What to do next" (every FAIL and WARN grouped by who acts:
CAD, print material, bought part, setting, operating limit, measurement). Use those two sections to brief the
team; the per-stage sections are the evidence.
