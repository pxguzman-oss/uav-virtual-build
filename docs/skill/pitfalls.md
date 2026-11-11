# Known pitfalls (handled in the code; recheck if tools change)

- **OpenFOAM v1912 (apt):** function objects crash without the sha1 LD_PRELOAD shim; scotch is a stub (hierarchical decomposition); the tutorials are empty; set `analysis.cfd.np` to the physical core count on big machines.
- **Steady RANS limits:** fully turbulent RANS over-predicts drag on low-Re laminar sections by ~50 %. Use CFD for lift slope, Cm, interference and non-wing drag increments, not absolute wing drag or CLmax.
- **xfoil:** the apt binary cannot run headless; use xfoil-python. Repanel S3021 before XFOIL.
- **NeuralFoil:** its `control_surfaces` flap model is wrong; deflect the geometry instead.
- **AeroBuildup:** no downwash, so too stable in pitch; Cm_alpha comes from AVL.
- **AVL:** file paths longer than 80 characters are truncated (use relative paths). Give Nspan per section.
- **ArduPilot SITL:** needs `empy==3.3.4`; the built-in SIM_Plane hard-codes mass and fakes propulsion, so use only the JSBSim bridge for numbers.
- **Weak hand launch:** a throw under ~6 m/s never passes ArduPilot's GPS-speed launch check. A real finding, not a sim bug.
- **OpenCascade (STEP):** class locations moved between versions (`vb/cadio.py` handles 7.x and 8.x); it prints transfer statistics to stdout, which the code silences.
- **Fusion names:** bodies need the part id at the start of the name; a re-exported assembly with renamed bodies becomes "unmatched" questions, which is the intended failure.
- **Thermal numbers:** lumped model, +-30-50 % on convection. Trust the ranking and the FAIL / WARN split more than the absolute temperatures.
- **Balance option tables are estimates** until the design loop has proven them: a battery swap that balances can still fail to hover.
- **Variants pile up:** the design loop writes `bal<ID>` variants into the plan (a `.bak` is kept). Option ids are per run; re-running the loop after a parts change overwrites variants of the same name.
- **Sub-agents** may be blocked from writing `.md` files; ask for `.txt` and put full findings in their final message.
