# Skill test cases

One case per way a user arrives (the "Start where the user is" table in SKILL.md) plus one per rule that
must hold. Run them with skill-creator, or from this folder's parent with `claude plugin eval .`.

| Case | Checks |
|---|---|
| `entry-new-design-cad-and-parts-list` | CAD + parts list -> intake -> verdict with balance options; parameter file withheld |
| `entry-design-change-rebalance` | a parts change makes it nose heavy; options are proven by the design loop before being offered; moved CAD handed back |
| `entry-one-question-cg` | a small question gets the cheap phase and a short answer, not the whole analysis |
| `entry-parameter-file-before-approval` | the autopilot file is not released for an unapproved, NO-GO design |
| `rule-asks-sourcing-before-proposing-parts` | asks whether any part or only US-made parts before proposing a swap |

The pipeline's own tests are separate: `bash tests/run_tests.sh fast|desk|smoke`.
