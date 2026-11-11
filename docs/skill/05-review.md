# Playbook 5 - Independent review (always, before reporting a flight proof)

Spawn a reviewer sub-agent that did not build the plan. Give it the plan and the `out/` folder, not your
conclusions. When sub-agents are not available, do the hand calculations yourself in a fresh pass and say in
the report that the review was not independent.

Ask for `REVIEW.txt` (ID, severity, location, finding, evidence, fix) covering at least:

1. **Hand calculations**: mass, CG (% MAC), static margin, stall speed, cruise power, static thrust, spar stress, gust load factor, hover power and thrust-to-weight, the CG shift of the chosen balance option (sum of mass x move / AUW), one thermal balance (heat in a bay vs wall conductance). Flag anything off by more than 20 %, or with a wrong sign, unit or axis.
2. **Intake**: parts-list masses add up to the plan AUW; every CAD body accounted for; frame right (nose forward, right wing at +y).
3. **One stall speed everywhere**: aero, structures, flight model and ArduPilot parameters use the same CLmax and Vs; safety gates use the pessimistic one.
4. **SITL environment**: flown in the mission wind as well as calm air, each result labelled.
5. **Energy accounting**: steady model consistent with SITL; range and endurance shown as an uncalibrated band.
6. **Provenance and vacuous passes**: code hash, plan hash and fidelity match across stages; a stage that did not run or has no gates never counts as PASS; a quick run or a desk check never says GO.
7. **Frames and signs**: Ixz in the JSBSim export, control-surface signs, Cm reference point, lift-motor order (front-right, rear-left, front-left, rear-right).
8. **Figures**: units, labels, misleading scales.

Fix the critical and major findings (commit each with its finding ID), re-run, and repeat the review if
anything critical changed. A desk check for a quick question does not need the full review; the hand
calculation of the number you are about to quote does.
