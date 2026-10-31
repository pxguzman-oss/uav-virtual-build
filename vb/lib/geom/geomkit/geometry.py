"""Low-level geometry helpers: airfoils, lofting, primitives, manifold booleans.

All lengths are millimetres. Frame convention used throughout the package
("structural frame", same orientation as JSBSim's structural frame):
    origin = wing root leading edge on the fuselage centreline
    +x aft, +y out the right wing, +z up
"""
from __future__ import annotations

import numpy as np
import trimesh
import manifold3d as mf
import mapbox_earcut as earcut


# --------------------------------------------------------------------------- airfoils
def naca4(code: str = "0009", n: int = 80) -> np.ndarray:
    """NACA 4-digit coordinates, TE->upper->LE->lower->TE (Selig order), unit chord."""
    m, p, t = int(code[0]) / 100, int(code[1]) / 10, int(code[2:]) / 100
    beta = np.linspace(0, np.pi, n // 2 + 1)
    x = 0.5 * (1 - np.cos(beta))
    yt = 5 * t * (0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * x**2 + 0.2843 * x**3 - 0.1036 * x**4)
    if m == 0:
        yc = np.zeros_like(x)
    else:
        yc = np.where(x < p, m / p**2 * (2 * p * x - x**2), m / (1 - p) ** 2 * (1 - 2 * p + 2 * p * x - x**2))
    up = np.c_[x, yc + yt][::-1]
    lo = np.c_[x, yc - yt][1:]
    return np.vstack([up, lo])


def load_airfoil(name: str, n: int = 100) -> np.ndarray:
    """Return Selig-ordered unit-chord coordinates.

    name: 'naca0009' style, a path to a .dat (Selig format), or any airfoil
    known to AeroSandbox's bundled UIUC database (e.g. 's3021').
    """
    import os
    if name.lower().startswith("naca") and len(name) == 8:
        return naca4(name[4:], n)
    if os.path.isfile(name):
        rows = []
        for line in open(name):
            parts = line.split()
            try:
                rows.append([float(parts[0]), float(parts[1])])
            except (ValueError, IndexError):
                continue
        return np.asarray(rows)
    import aerosandbox as asb  # bundled UIUC database, no network needed
    return np.asarray(asb.Airfoil(name).coordinates)


def resample_airfoil(xy: np.ndarray, n: int = 100, te_gap: float = 0.002) -> np.ndarray:
    """Cosine-resample upper and lower surfaces to n points total, open TE of `te_gap`
    (fraction of chord) so the section is a simple polygon without duplicate points.
    Returned order: upper TE -> LE -> lower TE (counter-clockwise when x right, z up
    is NOT guaranteed; orientation is fixed later)."""
    xy = np.asarray(xy, float)
    i_le = int(np.argmin(xy[:, 0]))
    up = xy[: i_le + 1][::-1]  # LE -> TE
    lo = xy[i_le:]             # LE -> TE
    k = n // 2
    xs = 0.5 * (1 - np.cos(np.linspace(0, np.pi, k)))
    yu = np.interp(xs, up[:, 0], up[:, 1])
    yl = np.interp(xs, lo[:, 0], lo[:, 1])
    # blend in a finite TE thickness linearly over the chord
    yu = yu + 0.5 * te_gap * xs
    yl = yl - 0.5 * te_gap * xs
    upper = np.c_[xs, yu][::-1]          # TE -> LE
    lower = np.c_[xs, yl][1:]            # LE(+1) -> TE
    return np.vstack([upper, lower])


# --------------------------------------------------------------------------- lofting
def _polygon_area2d(p: np.ndarray) -> float:
    x, y = p[:, 0], p[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def loft(sections: list[np.ndarray], cap_2d: np.ndarray | None = None) -> trimesh.Trimesh:
    """Loft closed 3D polylines (each shape (n,3), same n, same point order) into a
    closed, consistently oriented triangle mesh. Caps are triangulated with earcut
    using `cap_2d` (the common 2D profile, shape (n,2)) so concave airfoils work.
    """
    S = np.asarray(sections, float)
    m, n, _ = S.shape
    verts = S.reshape(-1, 3)
    faces = []
    for i in range(m - 1):
        a = i * n + np.arange(n)
        b = a + n
        a1 = i * n + (np.arange(n) + 1) % n
        b1 = a1 + n
        faces.append(np.c_[a, b, b1])
        faces.append(np.c_[a, b1, a1])
    if cap_2d is None:
        cap_2d = S[0][:, :2]
    tri = earcut.triangulate_float64(np.ascontiguousarray(cap_2d, dtype=np.float64), np.array([n], np.uint32)).reshape(-1, 3)
    faces.append(tri[:, ::-1])                 # first cap
    faces.append(tri + (m - 1) * n)            # last cap
    mesh = trimesh.Trimesh(verts, np.vstack(faces), process=True)
    trimesh.repair.fix_normals(mesh)          # make outward
    if mesh.volume < 0:
        mesh.invert()
    return mesh


def wing_section(af2d: np.ndarray, chord: float, le: tuple[float, float, float],
                 twist_deg: float = 0.0, roll_deg: float = 0.0) -> np.ndarray:
    """Place a unit airfoil (x along chord, y thickness) in the structural frame.
    The section lies in a plane normal to the panel span direction; roll_deg is the
    panel dihedral (rotation about x). Twist is about the leading edge (y axis)."""
    p = af2d * chord
    x, z = p[:, 0], p[:, 1]
    t = np.radians(-twist_deg)  # positive twist = nose up
    xr = x * np.cos(t) - z * np.sin(t)
    zr = x * np.sin(t) + z * np.cos(t)
    r = np.radians(roll_deg)
    yy = -zr * np.sin(r)
    zz = zr * np.cos(r)
    return np.c_[xr + le[0], yy + le[1], zz + le[2]]


def superellipse(width: float, height: float, n: int = 64, e: float = 2.5) -> np.ndarray:
    """Closed superellipse section in the (y,z) plane, CCW, n points."""
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    c, s = np.cos(t), np.sin(t)
    y = 0.5 * width * np.sign(c) * np.abs(c) ** (2 / e)
    z = 0.5 * height * np.sign(s) * np.abs(s) ** (2 / e)
    return np.c_[y, z]


# --------------------------------------------------------------------------- primitives
def box(size, center) -> trimesh.Trimesh:
    m = trimesh.creation.box(extents=size)
    m.apply_translation(center)
    return m


def _align_z_to(axis) -> np.ndarray:
    axis = np.asarray(axis, float)
    axis /= np.linalg.norm(axis)
    return trimesh.geometry.align_vectors([0, 0, 1], axis)


def cylinder(diameter, length, center, axis=(0, 0, 1), sections=48) -> trimesh.Trimesh:
    m = trimesh.creation.cylinder(radius=diameter / 2, height=length, sections=sections)
    m.apply_transform(_align_z_to(axis))
    m.apply_translation(center)
    return m


def tube(od, wall, length, center, axis=(1, 0, 0), sections=48) -> trimesh.Trimesh:
    """Hollow round tube (annulus), watertight. wall >= od/2 -> solid rod."""
    if wall >= od / 2 - 1e-9:
        return cylinder(od, length, center, axis, sections)
    m = trimesh.creation.annulus(r_min=od / 2 - wall, r_max=od / 2, height=length, sections=sections)
    m.apply_transform(_align_z_to(axis))
    m.apply_translation(center)
    return m


def square_tube(outer, wall, length, center, axis=(1, 0, 0)) -> trimesh.Trimesh:
    o = box([outer, outer, length], [0, 0, 0])
    if wall * 2 < outer:
        i = box([outer - 2 * wall, outer - 2 * wall, length + 2], [0, 0, 0])
        o = boolean([o, i], "difference")
    o.apply_transform(_align_z_to(axis))
    o.apply_translation(center)
    return o


# --------------------------------------------------------------------------- manifold bridge
def to_manifold(mesh: trimesh.Trimesh) -> mf.Manifold:
    m = mf.Manifold(mf.Mesh(vert_properties=np.asarray(mesh.vertices, np.float32),
                            tri_verts=np.asarray(mesh.faces, np.uint32)))
    if m.status() != mf.Error.NoError:
        raise ValueError(f"mesh is not a valid manifold: {m.status()}")
    return m


def from_manifold(m: mf.Manifold) -> trimesh.Trimesh:
    mesh = m.to_mesh()
    # process=False: trimesh vertex merging can collapse manifold slivers (touching parts) into
    # non-manifold edges; manifold output is already indexed and watertight
    return trimesh.Trimesh(mesh.vert_properties[:, :3], mesh.tri_verts, process=False)


def boolean(meshes, op: str = "union") -> trimesh.Trimesh:
    """Robust boolean via manifold3d. op in union|difference|intersection.
    difference = meshes[0] - union(meshes[1:])."""
    ms = [to_manifold(m) for m in meshes]
    if op == "union":
        r = mf.Manifold.batch_boolean(ms, mf.OpType.Add)
    elif op == "difference":
        r = ms[0] - mf.Manifold.batch_boolean(ms[1:], mf.OpType.Add) if len(ms) > 2 else ms[0] - ms[1]
    elif op == "intersection":
        r = mf.Manifold.batch_boolean(ms, mf.OpType.Intersect)
    else:
        raise ValueError(op)
    return from_manifold(r)


def mesh_report(mesh: trimesh.Trimesh) -> dict:
    return dict(watertight=bool(mesh.is_watertight), winding_consistent=bool(mesh.is_winding_consistent),
                volume_mm3=float(mesh.volume), area_mm2=float(mesh.area), faces=int(len(mesh.faces)),
                euler=int(mesh.euler_number), bodies=int(len(mesh.split(only_watertight=False))),
                bounds_mm=np.round(mesh.bounds, 2).tolist())
