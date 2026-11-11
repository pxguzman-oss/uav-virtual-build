#!/usr/bin/env python3
"""CAD assembly + parts list -> build plan (see vb/intake.py and docs/PARTS_LIST.md).

  python tools/intake.py --cad assembly.step --parts parts.csv --base examples/stork_phase0.buildplan.yaml \\
         --variant vtol --frame "origin=0,5,335; x_aft=+z; y_right=+x; z_up=+y" --policy any --out work/stork_vtol

  --cad        STEP assembly, or a folder with one STEP per part (or both: repeat the option)
  --frame      wing-root leading edge in CAD mm and the CAD axis of aft / right / up. Omit to get a GUESS
               (from bodies named wing / tail) that must be confirmed with the user before the run is trusted
  --policy     any | us_made | ndaa : which parts may be proposed and how the parts list is screened. ASK THE USER.
  --merge      YAML overlay merged into the base plan first (datasheet values for new motors, ESCs, cells ...)
  --cad-import also export every printed body as STL and use the real shapes for renders, fit and mass distribution
Writes <out>/<name>.buildplan.yaml, intake_report.md, intake.json, cad_map.json. Exit code 2 = open questions."""
import argparse
import json
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from vb import cadio, intake, plan as P  # noqa: E402


def parse_frame(s):
    kv = dict(x.split("=", 1) for x in s.replace(" ", "").split(";") if x)
    return cadio.Frame([float(v) for v in kv.get("origin", "0,0,0").split(",")], kv.get("x_aft", "+x"), kv.get("y_right", "+y"), kv.get("z_up", "+z"))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cad", action="append", required=True)
    ap.add_argument("--parts", required=True)
    ap.add_argument("--base", required=True)
    ap.add_argument("--variant", default=None)
    ap.add_argument("--frame", default=None)
    ap.add_argument("--policy", choices=["any", "us_made", "ndaa"], default=None)
    ap.add_argument("--merge", default=None)
    ap.add_argument("--name", default=None)
    ap.add_argument("--out", required=True)
    ap.add_argument("--cad-import", action="store_true")
    a = ap.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    bodies = []
    for c in a.cad:
        bodies += cadio.read_folder(c) if Path(c).is_dir() else cadio.read_step(c)
    if not bodies:
        sys.exit("no solid bodies found in the CAD input")
    rows, problems = intake.read_parts(a.parts)
    suggested = False
    if a.frame:
        frame = parse_frame(a.frame)
        notes = []
    else:
        frame, notes = cadio.suggest_frame(bodies)
        suggested = True
        if frame is None:
            sys.exit("cannot guess the frame: " + "; ".join(notes))
    raw = P.load_raw(a.base)
    base = P._merge({k: v for k, v in raw.items() if k != "variants"}, raw["variants"][a.variant]) if a.variant else raw
    merge = yaml.safe_load(Path(a.merge).read_text()) if a.merge else None
    name = a.name or (raw["meta"]["name"] + (f" {a.variant}" if a.variant else "") + " (CAD intake)")
    plan, rep = intake.build(bodies, rows, base, frame, suggested, defaults=dict(material=next(
        (k for k, m in base["materials"].items() if m.get("kind") == "printed_polymer"), None)), policy=a.policy, name=name, merge=merge)
    rep["questions"] = problems + notes + rep["questions"]
    if not a.policy:
        rep["questions"].append("sourcing policy not set: ask the user whether any part is allowed or only US-made / NDAA-compliant "
                                "parts (--policy any|us_made|ndaa)")
    if a.cad_import:
        sd = out / "cad_parts"
        sd.mkdir(exist_ok=True)
        by_path = {b.path: b for b in bodies}
        parts = []
        for c in plan["components"]:
            if c.get("category") != "structure_printed" or not c.get("cad_body"):
                continue
            paths = c["cad_body"].split("; ")
            m_g = P.val(c["mass"]) * 1000
            for k, pth in enumerate(paths):
                nm = c["name"] if k == 0 else P.mirror_name(c["name"])
                cadio.to_stl(by_path[pth].shape, sd / f"{nm}.stl", frame)
                parts.append(dict(name=nm, file=f"{nm}.stl", units="mm", material=c.get("material"), mass_model="solid", mass=round(m_g, 2)))
            c["cad_part"] = c["name"]
            c["geom"] = False
            c.pop("geom_part", None)
        plan["airframe"]["cad_import"] = dict(folder="cad_parts", replace_structure=True, oml_method="union", voxel_pitch=2.0, parts=parts)
    slug = P.slug(name)
    pf = out / f"{slug}.buildplan.yaml"
    pf.write_text("# Written by tools/intake.py from the CAD assembly and the parts list. Edit the parts list and re-run,\n"
                  "# or edit here; balance options are added below as variants by tools/balance_apply.py.\n"
                  + yaml.safe_dump(json.loads(json.dumps(plan)), sort_keys=False, width=160))
    errs = P.validate(yaml.safe_load(pf.read_text()))
    if errs:
        rep["questions"].append("the generated plan does not validate: " + "; ".join(errs[:5]))
    meas = intake.compare(intake.measure(bodies, frame), plan)
    for r in meas:
        if r["differs"]:
            rep["questions"].append(f"CAD and base plan disagree on {r['quantity']}: CAD {r['cad']} vs plan {r['plan']} - which is right?")
    (out / "intake.json").write_text(json.dumps(dict(rep, plan=str(pf), cad=a.cad, parts=a.parts), indent=1))
    (out / "cad_map.json").write_text(json.dumps(dict(frame=frame.spec, cad=a.cad, components=rep["cad_map"]), indent=1))
    intake.write_report(out / "intake_report.md", rep, meas, pf)
    print(f"plan: {pf}\nreport: {out / 'intake_report.md'}")
    print(f"matched {len(rep['matched'])} components, {len(rep['unmatched_bodies'])} unmatched bodies, {len(rep['questions'])} open question(s)")
    for q in rep["questions"]:
        print("  ?", q)
    if rep["totals"]:
        print(f"total {rep['totals']['mass_g']:.0f} g, CG {rep['totals']['cg_mm']} mm")
    return 2 if rep["questions"] else 0


if __name__ == "__main__":
    sys.exit(main())
