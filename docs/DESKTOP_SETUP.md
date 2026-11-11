# Desktop setup: Windows + WSL2 (Ubuntu 24.04)

The pipeline is Linux software (OpenFOAM, AVL, CalculiX, ArduPilot SITL, Blender, OpenCascade). On a Windows
desktop it runs inside WSL2; Fusion stays on Windows. Everything is free. Nothing needs the GPU.

1. **Install WSL2 and Ubuntu.** PowerShell as Administrator:
   `wsl --install -d Ubuntu-24.04`, reboot, choose a Linux user name and password when Ubuntu first opens.
2. **Give WSL the machine.** By default WSL2 takes half the RAM. Create `C:\Users\<you>\.wslconfig`:
   ```
   [wsl2]
   memory=48GB
   processors=24
   ```
   then `wsl --shutdown` in PowerShell and reopen Ubuntu. (Ryzen 9 5900X, 64 GB: this leaves 16 GB for
   Windows and Fusion.)
3. **Get the pipeline.** In the Ubuntu terminal, in the Linux home folder (NOT under `/mnt/c`: Windows drives
   are slow from Linux):
   ```
   sudo apt update && sudo apt install -y git
   git clone https://github.com/pxguzman-oss/uav-virtual-build ~/uav-virtual-build
   cd ~/uav-virtual-build && bash setup/install.sh        # ~15-20 min on 12 cores, ArduPilot build dominates
   .venv/bin/python setup/check_env.py                    # every stage should say RUNNABLE
   bash tests/smoke.sh
   ```
4. **Where files go.** Windows files are at `/mnt/c/Users/<you>/...` from Ubuntu; the Linux home is
   `\\wsl$\Ubuntu-24.04\home\<you>` in Windows Explorer. Export the STEP and the parts list from Windows into
   a project folder, copy them into `~/projects/<name>/` and run from there. Open
   `out/<plan>/<variant>/report/index.html` from Explorer through the `\\wsl$` path.
5. **Cores.** CFD uses every logical core by default. On a 12-core / 24-thread CPU set
   `analysis.cfd.np: 12` in the plan (physical cores) and keep at most three heavy jobs (CFD or SITL) at once.
   With 64 GB the full-fidelity mesh fits comfortably.
6. **Updating.** `git pull && bash setup/install.sh` (idempotent).

Native Ubuntu 24.04 works the same way from step 3.
