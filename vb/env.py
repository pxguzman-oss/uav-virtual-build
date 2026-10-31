"""Locate external tools. Order: environment variable -> setup/paths.env (written by install.sh) -> repo
defaults (tools/bin, third_party/) -> PATH. `python setup/check_env.py` prints the result."""
from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def _paths_env():
    f = ROOT / "setup" / "paths.env"
    out = {}
    if f.exists():
        for line in f.read_text().splitlines():
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, v = line.split("=", 1)
                out[k.strip().replace("export ", "")] = os.path.expandvars(v.strip().strip('"'))
    return out


_PE = _paths_env()


def _get(name, *defaults):
    v = os.environ.get(name) or _PE.get(name)
    if v:
        return v
    for d in defaults:
        if d and Path(d).exists():
            return str(d)
    return None


def avl():
    return _get("VB_AVL", ROOT / "tools" / "bin" / "avl", shutil.which("avl"))


def ardupilot_dir():
    return _get("VB_ARDUPILOT", ROOT / "third_party" / "ardupilot")


def arduplane():
    d = ardupilot_dir()
    if not d:
        return None
    b = Path(d) / "build" / "sitl" / "bin" / "arduplane"
    return str(b) if b.exists() else None


def blender():
    return _get("VB_BLENDER", shutil.which("blender"))


def openfoam_bashrc():
    return _get("VB_OPENFOAM_BASHRC", "/usr/share/openfoam/etc/bashrc", "/usr/lib/openfoam/openfoam/etc/bashrc")


def foam_shim():
    return _get("VB_FOAM_SHIM", ROOT / "tools" / "foam_sha1_shim" / "libfoam_sha1_shim.so")


def ccx():
    return _get("VB_CCX", shutil.which("ccx"))


def xvfb():
    return shutil.which("xvfb-run")


def python_has(mod):
    try:
        __import__(mod)
        return True
    except Exception:
        return False


def summary():
    """Tool availability -> which stages can run."""
    t = dict(avl=avl(), arduplane=arduplane(), blender=blender(), xvfb=xvfb(), openfoam=openfoam_bashrc(),
             simpleFoam=shutil.which("simpleFoam"), foam_shim=foam_shim(), ccx=ccx())
    py = {m: python_has(m) for m in ("numpy", "scipy", "yaml", "jsonschema", "matplotlib", "trimesh", "manifold3d",
                                      "mapbox_earcut", "fcl", "shapely", "aerosandbox", "neuralfoil", "xfoil",
                                      "jsbsim", "pymavlink", "pandas", "PIL", "build123d")}
    core = all(py[m] for m in ("numpy", "scipy", "yaml", "jsonschema", "matplotlib"))
    geom = core and all(py[m] for m in ("trimesh", "manifold3d", "mapbox_earcut", "shapely"))
    stages = {
        "ingest": core, "mass": core, "propulsion": core, "structures": core, "report": core and py["PIL"],
        "geometry": geom, "geometry.renders(blender)": geom and bool(t["blender"]),
        "clash": geom and py["fcl"],
        "aero_lowfi": core and py["aerosandbox"] and py["neuralfoil"] and bool(t["avl"]),
        "cfd": bool(t["simpleFoam"]) and bool(t["foam_shim"]) and py["aerosandbox"],
        "flightmodel": core,
        "sitl": py["jsbsim"] and py["pymavlink"] and py["pandas"] and bool(t["arduplane"]),
        "structures.calculix_crosscheck": bool(t["ccx"]),
        "geometry.step_import": py["build123d"],
    }
    return dict(tools=t, python=py, stages=stages)


def versions():
    v = {}
    for m in ("numpy", "scipy", "trimesh", "aerosandbox", "neuralfoil", "jsbsim", "pymavlink"):
        try:
            v[m] = __import__(m).__version__
        except Exception:
            pass
    ap = ardupilot_dir()
    if ap and (Path(ap) / ".git").exists():
        try:
            v["ardupilot"] = subprocess.run(["git", "-C", ap, "describe", "--tags", "--always"], capture_output=True,
                                            text=True, timeout=10).stdout.strip()
        except Exception:
            pass
    return v
