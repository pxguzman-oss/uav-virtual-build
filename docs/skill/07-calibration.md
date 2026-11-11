# Playbook 7 - Calibration loop (after building and flying)

A simulation narrows the risk; measurements retire it. Ask for them in this order (the report's "soft inputs"
table says which ones drive the FAILs for this design):

| Step | Measurement | Updates |
|---|---|---|
| C0 | part weights, all-up weight, measured balance point | masses, CG |
| C2 | thrust stand: thrust, current, rpm vs throttle (lift and pusher) | prop `ct_scale` / `cp_scale`, motor `rm` / `i0` |
| C3 | battery discharge and sag | cell `r_dc`, usable fraction |
| C4 | wingtip proof load, tap test | material E, frequencies |
| T1 | thermocouple or IR readings on the flagged parts after a hover and after a ground soak in the sun | `power_W`, limits, vents |
| F1-F5 | flight logs: glide and cruise legs, stall at known mass, doublets / AUTOTUNE, hover throttle and vibration | CD0, e, CLmax, derivatives, hover throttle |

Put them in a calibration YAML (`examples/stork_calibration_example.yaml`) and run
`python run.py PLAN --variant V --calibration CAL.yaml`. The report gains a predicted-vs-measured table;
anything outside its band becomes a "model error" risk. After C0, re-run the design loop: measured weights
usually move the CG enough to change the balance answer.
