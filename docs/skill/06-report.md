# Playbook 6 - Reporting, hand-back and approval

## The answer to the team (use this shape)

```
VERDICT            GO / CONDITIONAL GO / NO-GO / INCOMPLETE, fidelity, and the one-line reason
THE THREE QUESTIONS  layout / sizing / flight: PASS, WARN, FAIL or NOT RUN, with the first thing to fix in each
BALANCE            where the CG is, the candidates (proven numbers), the one you would pick and why
WHAT TO DO NEXT    FAILs first, grouped by who acts: CAD, print material, bought part, setting, operating limit
KEY NUMBERS        weight, CG, stall (nominal / pessimistic), hover margin (standard day / worst air),
                   range band (lead with the conservative end), power by flight phase, hottest parts
WHAT IS SOFT       the estimates that drive the FAILs, in the order to measure them
PARAMETER FILE     released or withheld, and why
FILES              report, decision brief, change list, moved STEP
```

- Lead with the verdict. GO only with zero FAIL / ERROR / NOT RUN at standard or full fidelity; CONDITIONAL GO needs an owner for every WARN.
- Never round a NO-GO into "probably fine". Say plainly that numbers are uncalibrated until the calibration loop has run.
- Changes are concrete: part, from -> to, reason; then the before / after verdict.
- `out/<plan>/<variant>/report/index.html` is self-contained: publish it as an artifact when the team will share it. Send `DECISION.md`, `fusion_change_list.csv` and the moved STEP with it.
- Save a short summary (verdict, key numbers, open items) to the project when one is attached.

## Approval gates the autopilot file

The report always shows the derived ArduPilot parameters for review. The loadable `derived.param` appears in
the report folder only when the plan carries the team's approval AND the run is GO or CONDITIONAL GO:

```yaml
meta:
  approval: {design_approved: true, by: "name", date: "2026-11-12"}
```

Do not hand over a parameter file before that, do not copy it out of the flight-model folder for them, and do
not set the approval yourself. When asked for the file early, say what is still blocking it.

## Parts and sourcing

- `meta.sourcing.policy` is the user's answer from the intake. Under `us_made` or `ndaa` the ingest stage fails parts whose stated origin breaks the policy and warns about unknown or unverified origins.
- When a swap is needed and `data/catalog/` has nothing that passes: search, check the maker's own statement of origin, add candidates to the parts list as alternatives with `origin` and `origin_verified: false`. Never state an origin you have not seen on the maker's or vendor's page.
