#!/usr/bin/env python3
"""Fabricate *messy* stand-ins for user STL files from the parametric model, to test
import_stl.py without the licensed Flightory files:
  - wing_R.stl     in METRES
  - fuselage_*.stl 3 segments in mm, touching at the split planes; nose with flipped
                   normals; mid segment hollow (closed internal void like a printed shell)
  - vtail.stl      in INCHES, 1 panel, with 30 faces deleted (holes)
Left wing / left V-tail panel are produced in the assembly YAML by mirroring.

  python tests/make_test_stls.py out/stork tests/stl
"""
import os
import sys

import numpy as np
import trimesh

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "vb", "lib", "geom"))
from geomkit import geometry as G  # noqa: E402


def main(model_dir, out):
    os.makedirs(out, exist_ok=True)
    P = lambda n: trimesh.load(os.path.join(model_dir, "parts", f"{n}.stl"), force="mesh")

    # wing in METRES (solid)
    w = P("wing_R")
    w.apply_scale(1e-3)
    w.export(os.path.join(out, "wing_R.stl"))

    # fuselage in 3 segments, touching at x = -100 and 250
    f = G.to_manifold(P("fuselage"))
    segs = []
    import manifold3d as mf
    a, rest = f.split_by_plane((1, 0, 0), -100.0)    # returns (positive side, negative side)
    b, nose = a, rest
    mid, tail = None, None
    pos, neg = b.split_by_plane((1, 0, 0), 250.0)
    tail, mid = pos, neg
    for name, part in (("fuselage_nose", nose), ("fuselage_mid", mid), ("fuselage_tail", tail)):
        m = G.from_manifold(part)
        if name == "fuselage_nose":
            m.invert()                              # flipped normals
        if name == "fuselage_mid":                  # closed internal void (like a hollow print)
            m = G.boolean([m, G.box([200, 60, 60], [75, 0, 0])], "difference")
        m.export(os.path.join(out, f"{name}.stl"))
        segs.append(m)

    # V-tail panel in inches with holes
    v = P("vtail_R")
    rng = np.random.default_rng(1)
    keep = np.ones(len(v.faces), bool)
    keep[rng.choice(len(v.faces), 30, replace=False)] = False
    v.update_faces(keep)
    v.apply_translation([-440, 0, -14])            # CAD origin at the panel root LE
    v.apply_scale(1 / 25.4)
    v.export(os.path.join(out, "vtail_R.stl"))
    print("wrote", sorted(os.listdir(out)))


if __name__ == "__main__":
    main(*sys.argv[1:3])
