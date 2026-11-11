# Playbook 1 - Setup and where to run

## Get the pipeline

1. Existing checkout? Look in `~/uav-virtual-build`, `/home/claude/uav-virtual-build`, `/home/claude/vbuild`, the session's repos.
2. Otherwise attach the private repo with `add_repo` (owner `pxguzman-oss`, repo `uav-virtual-build`; `read`, or `push` to commit fixes) and clone it to `~/uav-virtual-build`. If it cannot be reached, tell the user and ask for the repo or a tarball. Never rewrite the pipeline from memory.
3. `bash setup/install.sh` (idempotent, Ubuntu 24.04). About 40 min on 2 cores, 15-20 min on the team's 12-core desktop; the ArduPilot SITL build dominates. Run it in the background with a log and poll: containers can restart.
   Installs one venv, Blender + xvfb, OpenFOAM v1912 + the bundled sha1 shim, CalculiX, AVL, xfoil-python, JSBSim, OpenCascade (`cadquery-ocp`, STEP in and out), ArduPilot Plane-4.7.2.
4. `.venv/bin/python setup/check_env.py`: which stages can run here. A stage whose tools are missing shows as ERROR / NOT RUN in the report; it never passes silently.
5. `bash tests/run_tests.sh fast` (seconds). If it fails, stop and fix the install before touching a design.

## Where to run

| Machine | Use it for |
|---|---|
| The team's desktop: Windows + WSL2 Ubuntu 24.04, Ryzen 9 5900X (12 cores), 64 GB | everything, including full fidelity. `docs/DESKTOP_SETUP.md` is the step-by-step to give them. Set `analysis.cfd.np: 12`. |
| Cloud workspace (2 cores, 7 GB) | the design loop and quick / standard fidelity. Say so when full fidelity was not run. |

Prefer the desktop whenever this session has a shell on it. Nothing needs the GPU or paid software.

## Test tiers (`tests/run_tests.sh`)

| Tier | Time | What it proves |
|---|---|---|
| `fast` | seconds | arithmetic and file handling: thermal network against closed forms, balance maths, STEP round trip, intake totals |
| `desk` | ~4 min | CAD + parts list -> plan -> design loop on the Stork against known numbers (34.5 mm nose heavy, three options proven, parameter file withheld) |
| `smoke` | ~4 min | the whole pipeline at quick fidelity with a simulated flight, fixed wing and VTOL |

Run `fast` after every install or pull, `desk` after changing intake / balance / vtol / thermal / report code, `smoke` after changing anything in the flight chain. Stop at the first failing tier.

## Background runs

Anything over about 5 minutes: run in the background with a log, poll, and keep working. Stages are cached by input hash, so re-running the same command after an interruption only repeats what did not finish.
