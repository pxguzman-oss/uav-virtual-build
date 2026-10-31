#!/usr/bin/env python3
"""Mass properties from a model directory (manifest.json + parts/*.stl).

Writes mass_properties.json, jsbsim_mass_balance.xml, sdf_inertial.xml, mass_table.csv
  python mass_properties.py out/stork
  python mass_properties.py out/stork --set options.calibrate_masses=false   # raw geometric estimate
"""
import argparse
import csv
import json
import os
import sys
import time

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geomkit import manifest, massprops as MP  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("model_dir")
    ap.add_argument("--set", action="append", default=[], help="override plan keys, e.g. materials.lw_pla_foamed=0.6")
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args(argv)

    t0 = time.perf_counter()
    man, parts = manifest.read(a.model_dir)
    plan = man["plan"]
    for ov in a.set:
        k, v = ov.split("=", 1)
        node = plan
        for kk in k.split(".")[:-1]:
            node = node.setdefault(kk, {})
        node[k.split(".")[-1]] = yaml.safe_load(v)
    res = MP.compute(parts, plan)
    summ = MP.summarize(res, plan, man)
    dt = time.perf_counter() - t0
    summ["compute_time_s"] = round(dt, 3)

    d = a.model_dir
    json.dump(manifest._clean(summ), open(os.path.join(d, "mass_properties.json"), "w"), indent=1)
    open(os.path.join(d, "jsbsim_mass_balance.xml"), "w").write(MP.jsbsim_xml(summ))
    open(os.path.join(d, "sdf_inertial.xml"), "w").write(MP.sdf_inertial(summ))
    with open(os.path.join(d, "mass_table.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["name", "group", "mass_g", "raw_mass_g", "x_mm", "y_mm", "z_mm"])
        for p in summ["parts"]:
            w.writerow([p["name"], p["group"], p["mass_g"], p["raw_mass_g"], *p["cg_mm"]])

    if not a.quiet:
        grp = {}
        for p in summ["parts"]:
            grp[p["group"]] = grp.get(p["group"], 0) + p["mass_g"]
        print("group masses [g]: " + ", ".join(f"{k}={v:.1f}" for k, v in grp.items()))
        for g, c in summ["calibration"].items():
            print(f"  calibration {g}: geometric estimate {c['raw_estimate_g']} g -> target {c['target_g']} g (scale {c.get('scale')})")
        cg = summ["cg_struct_mm"]
        print(f"total mass {summ['mass_kg']*1000:.1f} g")
        print(f"CG (struct, mm from wing-root LE): x={cg[0]:.1f} y={cg[1]:.2f} z={cg[2]:.1f}")
        if "cg_target_x_mm" in summ:
            print(f"CG target x={summ['cg_target_x_mm']} -> error {summ['cg_error_x_mm']:+.1f} mm "
                  f"({'OK' if summ['cg_ok'] else 'OUT OF TOLERANCE'}); {summ['cg_pct_mac']:.1f}% MAC; "
                  f"move battery {summ.get('battery_shift_to_target_mm', 0):+.1f} mm in x to hit target")
            if summ.get("nose_ballast_g"):
                print(f"  or add {summ['nose_ballast_g']:.0f} g ballast at x={summ['nose_ballast_x_mm']:.0f} mm")
        b = summ["body_axes_about_cg"]
        print("inertia about CG, body axes FRD [kg m^2]: "
              f"Ixx={b['Ixx']:.5f} Iyy={b['Iyy']:.5f} Izz={b['Izz']:.5f} Ixz={b['Ixz']:+.6f} "
              f"(Ixy={b['Ixy']:+.1e}, Iyz={b['Iyz']:+.1e})")
        print(f"computed in {dt:.2f} s -> {d}/mass_properties.json, jsbsim_mass_balance.xml, sdf_inertial.xml")
    return 0


if __name__ == "__main__":
    sys.exit(main())
