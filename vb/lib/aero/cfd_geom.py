#!/usr/bin/env python
"""
Write closed, watertight-ish STL surfaces of the Stork (or any stork.yaml-like definition) for the CFD stage.
Frame = structural frame of geom/stork.yaml in METRES: +X aft, +Y right, +Z up, origin wing-root LE.
Half-model: components are cut at the symmetry plane by snappyHexMesh, so every part is extended slightly
past y = 0 (wing root, V-tail root) or written full (fuselage).

  python cfd_geom.py --yaml <geometry stork.yaml> --kind wing      --out geom/wing
  python cfd_geom.py --yaml ...                                  --kind wingbody  --out geom/wingbody   (wing+fuselage+vtail)
Each component -> <out>/<component>.stl (ASCII, solid name = component). Also writes <out>/geom.json with refs.
"""
import argparse, json, math, os, sys
import numpy as np
import aerosandbox as asb

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import stork_aero as sa  # noqa: E402


def write_stl(path, name, verts, faces):
    v = np.asarray(verts, float)
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for tri in faces:
            p0, p1, p2 = v[tri[0]], v[tri[1]], v[tri[2]]
            n = np.cross(p1 - p0, p2 - p0)
            ln = np.linalg.norm(n)
            if ln < 1e-16:
                continue
            n /= ln
            f.write(f" facet normal {n[0]:.6e} {n[1]:.6e} {n[2]:.6e}\n  outer loop\n")
            for p in (p0, p1, p2):
                f.write(f"   vertex {p[0]:.7e} {p[1]:.7e} {p[2]:.7e}\n")
            f.write("  endloop\n endfacet\n")
        f.write(f"endsolid {name}\n")


def quads_to_tris(faces):
    out = []
    for fc in faces:
        if len(fc) == 3:
            out.append(list(fc))
        else:
            out.append([fc[0], fc[1], fc[2]]); out.append([fc[0], fc[2], fc[3]])
    return out


def section_loop(af_coords, xle, y, zle, chord, twist_deg, n=None):
    """Airfoil coordinates (Selig order, TE->upper->LE->lower->TE) placed in 3-D; twist about LE (nose-up +)."""
    c = np.asarray(af_coords)
    x = c[:, 0] * chord; z = c[:, 1] * chord
    t = math.radians(twist_deg)
    xr = x * math.cos(t) + z * math.sin(t)
    zr = -x * math.sin(t) + z * math.cos(t)
    return np.column_stack([xle + xr, np.full_like(xr, y), zle + zr])


def loft(loops, cap_ends=True, closed_loop=True):
    """Loft a list of equal-length point loops into a closed triangulated surface."""
    loops = [np.asarray(l) for l in loops]
    n = len(loops[0])
    V = np.vstack(loops)
    F = []
    m = n if closed_loop else n - 1
    for i in range(len(loops) - 1):
        for j in range(m):
            a = i * n + j; b = i * n + (j + 1) % n
            c = (i + 1) * n + (j + 1) % n; d = (i + 1) * n + j
            F.append([a, b, c]); F.append([a, c, d])
    if cap_ends:
        for k, sign in ((0, 1), (len(loops) - 1, -1)):
            ctr = loops[k].mean(axis=0)
            V = np.vstack([V, ctr]); ic = len(V) - 1
            for j in range(n):
                a = k * n + j; b = k * n + (j + 1) % n
                F.append([ic, b, a] if sign > 0 else [ic, a, b])
    return V, F


def wing_surface(g, n_af=80, n_span=18, y_start=-0.02):
    af = asb.Airfoil(g["wing"]["airfoil"]).repanel(n_points_per_side=n_af)
    co = af.coordinates.copy()
    # close a blunt TE: force first/last point to the mean so the loop is closed (sharp TE)
    te = 0.5 * (co[0] + co[-1]); co[0] = te; co[-1] = te
    co = co[:-1]
    semi = g["wing"]["span"] / 2
    ys = np.concatenate([[y_start], semi * (1 - np.cos(np.linspace(0, np.pi / 2, n_span)))])
    loops = []
    for y in ys:
        xle, zle, ch, tw = sa.wing_station(g, max(y, 0.0))
        if y < 0:
            zle = sa.wing_station(g, 0.0)[1]
        loops.append(section_loop(co, xle, y, zle, ch, tw))
    return loft(loops)


def vtail_surface(g, side=+1, n_af=50, s_start=-0.12):
    af = asb.Airfoil(g["vtail"]["airfoil"]).repanel(n_points_per_side=n_af)
    co = af.coordinates.copy(); te = 0.5 * (co[0] + co[-1]); co[0] = te; co[-1] = te; co = co[:-1]
    loops = []
    for s in np.concatenate([[s_start], np.linspace(0, 1, 8)]):
        xle, y, z, ch = sa.vtail_station(g, s, side)
        if s < 0:
            ch = g["vtail"]["cr"]
        # local airfoil plane: chord along x, thickness along the panel normal
        dih = math.radians(g["vtail"]["dihedral_deg"])
        nrm = np.array([0, -side * math.sin(dih), math.cos(dih)])
        pts = np.array([[xle + px * ch, 0, 0] for px, pz in co])
        pts[:, 1] = y + co[:, 1] * ch * nrm[1]
        pts[:, 2] = z + co[:, 1] * ch * nrm[2]
        loops.append(pts)
    return loft(loops)


def fuselage_surface(g, n_theta=64, n_x=60):
    st = np.array(g["fuse"]["stations"])  # x, width, height, zc
    ex = g["fuse"]["exponent"]
    # smooth nose: cosine-distribute x and interpolate
    xs = st[:, 0]
    xx = xs.min() + (xs.max() - xs.min()) * 0.5 * (1 - np.cos(np.linspace(0, np.pi, n_x)))
    W = np.interp(xx, xs, st[:, 1]); H = np.interp(xx, xs, st[:, 2]); Z = np.interp(xx, xs, st[:, 3])
    th = np.linspace(0, 2 * np.pi, n_theta, endpoint=False)
    loops = []
    for x, w, h, zc in zip(xx, W, H, Z):
        c, s = np.cos(th), np.sin(th)
        yy = (w / 2) * np.sign(c) * np.abs(c) ** (2 / ex)
        zz = zc + (h / 2) * np.sign(s) * np.abs(s) ** (2 / ex)
        loops.append(np.column_stack([np.full_like(yy, x), yy, zz]))
    return loft(loops)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--yaml", required=True)
    ap.add_argument("--kind", choices=["wing", "wingbody", "wingbodytail"], default="wing")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    g = sa.load_params(a.yaml)
    comps = {}
    comps["wing"] = wing_surface(g)
    if a.kind in ("wingbody", "wingbodytail"):
        comps["fuselage"] = fuselage_surface(g)
    if a.kind == "wingbodytail":
        comps["vtail"] = vtail_surface(g, +1)
    for nm, (V, F) in comps.items():
        write_stl(os.path.join(a.out, f"{nm}.stl"), nm, V, F)
        print(nm, len(F), "triangles, bbox", np.round(V.min(0), 3), np.round(V.max(0), 3))
    meta = dict(kind=a.kind, yaml=os.path.abspath(a.yaml), S_ref_full=g["ref"]["S"], c_ref=g["ref"]["c"],
                b_ref=g["ref"]["b"], moment_ref=[g["ref"]["x_cg"], 0.0, 0.0], half_model=True,
                components=list(comps.keys()))
    json.dump(meta, open(os.path.join(a.out, "geom.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
