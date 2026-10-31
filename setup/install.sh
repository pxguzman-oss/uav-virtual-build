#!/usr/bin/env bash
# =====================================================================================================
# vbuild installer - idempotent, for a clean Ubuntu 24.04 (container or VM, root or sudo).
#
#   bash setup/install.sh                 # everything (~35-45 min on 2 cores, ArduPilot build dominates)
#   VB_SKIP_ARDUPILOT=1 bash setup/install.sh   # no SITL stage
#   VB_SKIP_OPENFOAM=1  bash setup/install.sh   # no CFD stage
#   VB_SKIP_BLENDER=1   bash setup/install.sh   # renders fall back to matplotlib
#   VB_VENV=/path/venv  bash setup/install.sh   # venv location (default <repo>/.venv)
#
# Re-running skips every step whose result already exists. Afterwards: python setup/check_env.py
# What it installs
#   apt   : python3-venv/dev, build tools, gfortran, git, libx11-dev (AVL plot lib), liblapack-dev,
#           blender + xvfb (Workbench renders), openfoam (v1912 apt) + openmpi, calculix-ccx
#   venv  : ONE virtualenv for every stage (setup/requirements.txt) + xfoil-python (built from source:
#           the PyPI sdist is broken)
#   tools : AVL 3.40 (github.com/mayerrobert/avl) -> tools/bin/avl
#           OpenFOAM v1912 function-object fix -> tools/foam_sha1_shim/libfoam_sha1_shim.so (LD_PRELOAD)
#           ArduPilot Plane-4.7.2 (only the submodules SITL needs) -> third_party/ardupilot, ./waf plane
#   writes setup/paths.env (tool locations read by vb/env.py)
# =====================================================================================================
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
VENV=${VB_VENV:-$ROOT/.venv}
JOBS=${VB_JOBS:-$(nproc)}
AP_TAG=${VB_ARDUPILOT_TAG:-Plane-4.7.2}
SUDO=""; [[ $(id -u) -ne 0 ]] && SUDO="sudo"
log() { printf '\n\033[1m[vbuild install] %s\033[0m\n' "$*"; }
export DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------------- 1. apt packages
PKGS="python3 python3-venv python3-dev python3-pip build-essential git cmake pkg-config gfortran libx11-dev
      liblapack-dev libblas-dev libgl1 libglu1-mesa libxrender1 libxi6 libxkbcommon0 rsync ca-certificates curl
      calculix-ccx"
[[ -z "${VB_SKIP_BLENDER:-}" ]] && PKGS="$PKGS blender xvfb xauth"
[[ -z "${VB_SKIP_OPENFOAM:-}" ]] && PKGS="$PKGS openfoam openmpi-bin libopenmpi-dev"
MISSING=""
for p in $PKGS; do dpkg -s "$p" >/dev/null 2>&1 || MISSING="$MISSING $p"; done
if [[ -n "$MISSING" ]]; then
  log "apt install:$MISSING"
  $SUDO apt-get update -qq
  $SUDO apt-get install -y -qq --no-install-recommends $MISSING
else
  log "apt packages present"
fi

# ---------------------------------------------------------------- 2. one virtualenv
if [[ ! -x "$VENV/bin/python" ]]; then
  log "creating venv $VENV"
  python3 -m venv "$VENV"
fi
"$VENV/bin/python" -m pip install -q --upgrade pip wheel setuptools
if ! "$VENV/bin/python" -c "import aerosandbox, jsbsim, trimesh, manifold3d, pymavlink, fcl" 2>/dev/null; then
  log "pip install -r setup/requirements.txt"
  "$VENV/bin/pip" install -q -r "$ROOT/setup/requirements.txt"
else
  log "python packages present"
fi
mkdir -p "$ROOT/tools/src" "$ROOT/tools/bin" "$ROOT/third_party"
if ! "$VENV/bin/python" -c "import xfoil" 2>/dev/null; then
  log "xfoil-python (Fortran build)"
  [[ -d "$ROOT/tools/src/xfoil-python" ]] || git clone -q --depth 1 https://github.com/DARcorporation/xfoil-python "$ROOT/tools/src/xfoil-python"
  "$VENV/bin/pip" install -q "$ROOT/tools/src/xfoil-python" || echo "WARNING: xfoil-python failed (optional: airfoil cross-checks only)"
fi

# ---------------------------------------------------------------- 3. AVL 3.40
if [[ ! -x "$ROOT/tools/bin/avl" ]]; then
  log "building AVL"
  [[ -d "$ROOT/tools/src/avl" ]] || git clone -q --depth 1 https://github.com/mayerrobert/avl "$ROOT/tools/src/avl"
  # the Makefile's final 'install' step fails harmlessly ("same file"); the binary is in bin/
  make -s -C "$ROOT/tools/src/avl" -f Makefile.gfortran >"$ROOT/tools/src/avl_build.log" 2>&1 || true
  cp "$ROOT/tools/src/avl/bin/avl" "$ROOT/tools/bin/avl"
fi
echo "quit" | "$ROOT/tools/bin/avl" >/dev/null 2>&1 && log "AVL ok"

# ---------------------------------------------------------------- 4. OpenFOAM function-object shim
if [[ -z "${VB_SKIP_OPENFOAM:-}" && ! -f "$ROOT/tools/foam_sha1_shim/libfoam_sha1_shim.so" ]]; then
  log "building OpenFOAM v1912 sha1 shim"
  gcc -O2 -shared -fPIC -o "$ROOT/tools/foam_sha1_shim/libfoam_sha1_shim.so" "$ROOT/tools/foam_sha1_shim/foam_sha1_shim.c"
fi

# ---------------------------------------------------------------- 5. ArduPilot SITL
AP=${VB_ARDUPILOT:-$ROOT/third_party/ardupilot}
if [[ -z "${VB_SKIP_ARDUPILOT:-}" ]]; then
  if [[ ! -x "$AP/build/sitl/bin/arduplane" ]]; then
    if [[ ! -d "$AP/.git" ]]; then
      log "cloning ArduPilot $AP_TAG"
      git clone -q --depth 1 --branch "$AP_TAG" https://github.com/ArduPilot/ardupilot.git "$AP"
    fi
    log "ArduPilot submodules (SITL subset)"
    git -C "$AP" submodule update --init --recursive --depth 1 modules/waf modules/mavlink modules/DroneCAN/DSDL \
        modules/DroneCAN/pydronecan modules/DroneCAN/dronecan_dsdlc modules/DroneCAN/libcanard modules/littlefs \
        modules/lwip modules/Micro-XRCE-DDS-Client modules/Micro-CDR modules/gtest modules/gbenchmark
    log "./waf configure --board sitl && ./waf plane (20-30 min on 2 cores)"
    ( cd "$AP" && PATH="$VENV/bin:$PATH" ./waf configure --board sitl >/dev/null && PATH="$VENV/bin:$PATH" ./waf plane -j"$JOBS" )
  else
    log "ArduPlane SITL present"
  fi
fi

# ---------------------------------------------------------------- 6. paths.env + check
OFRC=/usr/share/openfoam/etc/bashrc
{
  echo "# written by setup/install.sh $(date -u +%F)"
  echo "VB_VENV=$VENV"
  echo "VB_AVL=$ROOT/tools/bin/avl"
  [[ -z "${VB_SKIP_ARDUPILOT:-}" ]] && echo "VB_ARDUPILOT=$AP"
  [[ -f "$OFRC" ]] && echo "VB_OPENFOAM_BASHRC=$OFRC"
  [[ -f "$ROOT/tools/foam_sha1_shim/libfoam_sha1_shim.so" ]] && echo "VB_FOAM_SHIM=$ROOT/tools/foam_sha1_shim/libfoam_sha1_shim.so"
} > "$ROOT/setup/paths.env"
log "done -> $ROOT/setup/paths.env"
"$VENV/bin/python" "$ROOT/setup/check_env.py" || true
echo
echo "Run:  $VENV/bin/python $ROOT/run.py $ROOT/examples/stork_phase0.buildplan.yaml --fidelity quick"
