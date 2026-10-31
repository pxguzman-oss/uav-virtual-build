"""Import user-supplied STL/STEP/OBJ/3MF parts, detect units, repair, place by
transforms, and merge into a watertight OML.
"""
from __future__ import annotations

import os

import numpy as np
import trimesh

from . import geometry as G

UNIT_SCALES = {"mm": 1.0, "cm": 10.0, "m": 1000.0, "in": 25.4}


# --------------------------------------------------------------------------- loading
def load_any(path: str) -> trimesh.Trimesh:
    ext = os.path.splitext(path)[1].lower()
    if ext in (".step", ".stp"):
        # STEP needs OCCT: tessellate with build123d / cadquery (installed separately)
        try:
            import build123d as bd
        except ImportError as e:
            raise RuntimeError("STEP import needs build123d (pip install build123d)") from e
        import tempfile
        shape = bd.import_step(path)
        tmp = os.path.join(tempfile.mkdtemp(prefix="vb_step_"), os.path.basename(path) + ".stl")
        bd.export_stl(shape, tmp, tolerance=0.05, angular_tolerance=0.2)   # STEP is unit-tagged -> mm
        path = tmp
    m = trimesh.load(path, force="mesh", process=False)
    if isinstance(m, trimesh.Scene):
        m = m.dump(concatenate=True)
    return m


def detect_units(mesh: trimesh.Trimesh, expected_extent_mm: float | None = None, declared: str = "auto"):
    """Return (scale_to_mm, unit, reason). With an expected size, pick the unit whose
    scaled max extent is closest (log ratio) to it; otherwise use a size heuristic for
    small-UAV parts: < 3 -> metres, otherwise millimetres (inch files are flagged when
    the extent is suspiciously small for its role)."""
    if declared != "auto":
        return UNIT_SCALES[declared], declared, "declared"
    ext = float(mesh.extents.max())
    if expected_extent_mm:
        best = min(UNIT_SCALES, key=lambda u: abs(np.log(ext * UNIT_SCALES[u] / expected_extent_mm)))
        return UNIT_SCALES[best], best, f"extent {ext:.4g} vs expected {expected_extent_mm} mm"
    if ext < 3.0:
        return 1000.0, "m", f"extent {ext:.4g} < 3 -> metres"
    return 1.0, "mm", f"extent {ext:.4g} -> millimetres (default)"


# --------------------------------------------------------------------------- repair
def boundary_loops(mesh: trimesh.Trimesh) -> list[np.ndarray]:
    """Closed loops of boundary edges (edges used by exactly one face), as vertex indices."""
    from trimesh.grouping import group_rows
    e = mesh.edges_sorted
    b = e[group_rows(e, require_count=1)]
    if not len(b):
        return []
    nbr = {}
    for u, v in b:
        nbr.setdefault(int(u), []).append(int(v))
        nbr.setdefault(int(v), []).append(int(u))
    if any(len(n) != 2 for n in nbr.values()):      # non-manifold boundary (bow-ties) -> let MeshFix handle it
        return None
    loops, seen = [], set()
    for start in nbr:
        if start in seen:
            continue
        loop, prev, cur = [start], None, start
        seen.add(start)
        while True:
            nxt = nbr[cur][0] if nbr[cur][0] != prev else nbr[cur][1]
            if nxt == start:
                break
            if nxt in seen:
                return None
            loop.append(nxt); seen.add(nxt); prev, cur = cur, nxt
        loops.append(np.array(loop))
    return loops


def close_loops(mesh: trimesh.Trimesh):
    """Cap every boundary loop by ear-clipping it in its best-fit (PCA) plane. Fast and exact
    for planar-ish openings such as the open trailing-edge seams of print STLs (a long thin
    slit along the TE) or missing end caps. Returns (mesh, n_loops) or (None, 0) if unsuitable."""
    import mapbox_earcut as earcut
    loops = boundary_loops(mesh)
    if not loops:
        return None, 0
    new = []
    for L in loops:
        P = mesh.vertices[L]
        c = P.mean(0)
        _, _, vt = np.linalg.svd(P - c, full_matrices=False)
        uv = (P - c) @ vt[:2].T
        tri = earcut.triangulate_float64(np.ascontiguousarray(uv), np.array([len(L)], np.uint32)).reshape(-1, 3)
        if len(tri) != len(L) - 2:
            return None, 0
        new.append(L[tri])
    m = trimesh.Trimesh(mesh.vertices, np.vstack([mesh.faces] + new), process=False)
    trimesh.repair.fix_winding(m)
    if not m.is_watertight:
        return None, 0
    if m.volume < 0:
        m.invert()
    return m, len(loops)


def repair(mesh: trimesh.Trimesh, use_pymeshfix: bool = True, merge_tol: float = 1e-5):
    """Standard repair ladder. Returns (mesh, log list)."""
    log = []
    m = mesh.copy()
    m.merge_vertices(digits_vertex=int(-np.log10(merge_tol)) if merge_tol else None)
    m.update_faces(m.nondegenerate_faces())
    m.update_faces(m.unique_faces())
    m.remove_unreferenced_vertices()
    log.append(f"after merge: watertight={m.is_watertight} volume_sign={'+' if m.volume > 0 else '-'}")
    trimesh.repair.fix_winding(m)
    # NOT multibody=True: that would turn internal void shells of hollow prints inside-out
    # into separate solids. Only flip the whole mesh if its total volume is negative.
    if m.is_watertight and m.volume < 0:
        m.invert()
        log.append("inverted (normals pointed inward)")
    if not m.is_watertight:
        # repair body by body: MeshFix keeps only the largest component, so never feed it a
        # multi-body mesh (it would silently drop extra shells of a print part)
        bodies = m.split(only_watertight=False)     # trimesh 5.x split() also closes tri/quad holes
        log.append(f"split: {len(bodies)} body(ies), {sum(b.is_watertight for b in bodies)} closed after split")
        fixed, n_fill, n_fix = [], 0, 0
        n_loop = 0
        for b in bodies:
            if not b.is_watertight:
                c, k = close_loops(b)
                if c is not None:
                    b, n_loop = c, n_loop + k
            if not b.is_watertight:
                trimesh.repair.fill_holes(b)
                n_fill += 1
            if not b.is_watertight and use_pymeshfix and len(b.faces) >= 4:
                import pymeshfix
                v, f = pymeshfix.clean_from_arrays(np.asarray(b.vertices), np.asarray(b.faces))
                b = trimesh.Trimesh(v, f, process=True)
                n_fix += 1
            fixed.append(b)
        m = trimesh.util.concatenate(fixed) if len(fixed) > 1 else fixed[0]
        log.append(f"{n_loop} boundary loop(s) capped (earcut); {n_fill} body(ies) -> trimesh.fill_holes; {n_fix} -> pymeshfix")
    if m.is_watertight and m.volume < 0:
        m.invert()
        log.append("inverted")
    log.append(f"watertight={m.is_watertight} bodies={m.body_count}")
    return m, log


def transform_matrix(t: dict | None) -> np.ndarray:
    """t: {scale, mirror: 'x'|'y'|'z', rotate_deg: [rx, ry, rz] (applied x then y then z),
    translate: [x, y, z]} -> 4x4, applied in that order. (`center_at` is handled in place())."""
    t = t or {}
    M = np.eye(4)
    if "scale" in t:
        M = trimesh.transformations.scale_matrix(t["scale"]) @ M
    if t.get("mirror"):
        ax = "xyz".index(t["mirror"])
        S = np.eye(4)
        S[ax, ax] = -1
        M = S @ M
    if "rotate_deg" in t:
        rx, ry, rz = np.radians(t["rotate_deg"])
        M = trimesh.transformations.euler_matrix(rx, ry, rz, "sxyz") @ M
    if "translate" in t:
        M = trimesh.transformations.translation_matrix(t["translate"]) @ M
    return M


def place(mesh: trimesh.Trimesh, t: dict | None) -> trimesh.Trimesh:
    m = mesh.copy()
    M = transform_matrix(t)
    m.apply_transform(M)   # trimesh re-winds faces itself when M is a reflection (mirror)
    if t and t.get("center_at") is not None:
        # quick placement for print-oriented parts: put the bounding-box centre at a point
        m.apply_translation(np.asarray(t["center_at"], float) - m.bounds.mean(axis=0))
    if m.is_watertight and m.volume < 0:
        m.invert()
    return m


# --------------------------------------------------------------------------- OML
def outer_shells(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """Keep only outer surfaces: drop void shells (negative volume) and any shell nested
    inside a bigger one (voids whose normals were flipped outward by some tool)."""
    pieces = [p for p in mesh.split(only_watertight=False) if p.is_watertight and p.volume > 0]
    pieces.sort(key=lambda p: -p.volume)
    keep = []
    for p in pieces:
        nested = any(np.all(p.bounds[0] >= k.bounds[0]) and np.all(p.bounds[1] <= k.bounds[1])
                     and k.contains(p.vertices[:1])[0] for k in keep)
        if not nested:
            keep.append(p)
    return trimesh.util.concatenate(keep) if keep else mesh


def union_oml(meshes, drop_voids=True):
    u = G.boolean(meshes, "union")
    return outer_shells(u) if drop_voids else u


def _occupancy_scanline(mesh: trimesh.Trimesh, lo, shape, pitch):
    """Inside/outside on a regular grid by z-ray parity per (x,y) column. Vectorised over
    triangles; memory ~ one byte per voxel (no trimesh.voxelized blow-up)."""
    nx, ny, nz = shape
    tri = mesh.triangles - lo                       # grid-local coordinates
    occ_par = np.zeros((nx, ny, nz), np.int8)
    zc = (np.arange(nz) + 0.5) * pitch
    for t in np.array_split(tri, max(1, len(tri) // 4000)):
        a, b, c = t[:, 0], t[:, 1], t[:, 2]
        xmin = np.floor(np.minimum(np.minimum(a[:, 0], b[:, 0]), c[:, 0]) / pitch - 0.5).astype(int)
        xmax = np.ceil(np.maximum(np.maximum(a[:, 0], b[:, 0]), c[:, 0]) / pitch - 0.5).astype(int)
        ymin = np.floor(np.minimum(np.minimum(a[:, 1], b[:, 1]), c[:, 1]) / pitch - 0.5).astype(int)
        ymax = np.ceil(np.maximum(np.maximum(a[:, 1], b[:, 1]), c[:, 1]) / pitch - 0.5).astype(int)
        for k in range(len(t)):
            ix = np.arange(max(xmin[k], 0), min(xmax[k], nx - 1) + 1)
            iy = np.arange(max(ymin[k], 0), min(ymax[k], ny - 1) + 1)
            if not len(ix) or not len(iy):
                continue
            X, Y = np.meshgrid((ix + 0.5) * pitch, (iy + 0.5) * pitch, indexing="ij")
            v0, v1, v2 = a[k], b[k], c[k]
            d = (v1[1] - v2[1]) * (v0[0] - v2[0]) + (v2[0] - v1[0]) * (v0[1] - v2[1])
            if abs(d) < 1e-12:
                continue
            l1 = ((v1[1] - v2[1]) * (X - v2[0]) + (v2[0] - v1[0]) * (Y - v2[1])) / d
            l2 = ((v2[1] - v0[1]) * (X - v2[0]) + (v0[0] - v2[0]) * (Y - v2[1])) / d
            l3 = 1 - l1 - l2
            ins = (l1 >= 0) & (l2 >= 0) & (l3 > 0)    # half-open to avoid double counting edges
            if not ins.any():
                continue
            z = l1 * v0[2] + l2 * v1[2] + l3 * v2[2]
            ii, jj = np.nonzero(ins)
            kz = np.clip(np.searchsorted(zc, z[ii, jj]), 0, nz)
            # toggle parity for all voxels above the crossing: mark crossing, cumsum later
            m = kz < nz
            np.add.at(occ_par, (ix[ii[m]], iy[jj[m]], kz[m]), 1)
    return (np.cumsum(occ_par, axis=2) % 2).astype(bool)


def voxel_wrap(meshes, pitch: float = 2.0, close_mm: float = 2.0, smooth_iters: int = 10):
    """Fallback 'shrink-wrap' for messy inputs (gaps between print segments,
    self-intersections, overlapping non-manifold parts): scanline voxelisation of each
    part (union of occupancies), morphological closing to bridge gaps < close_mm, fill
    internal voids, marching cubes, Taubin smoothing. Always watertight; rounds sharp
    trailing edges at ~pitch scale, so use pitch <= TE thickness/2 when that matters."""
    from scipy import ndimage
    from skimage.measure import marching_cubes
    allm = trimesh.util.concatenate(meshes)
    pad = 3 * pitch + close_mm
    lo = allm.bounds[0] - pad
    shape = tuple(np.ceil((allm.bounds[1] + pad - lo) / pitch).astype(int))
    occ = np.zeros(shape, bool)
    for m in meshes:
        occ |= _occupancy_scanline(m, lo, shape, pitch)
    r = int(np.ceil(close_mm / pitch))
    if r > 0:
        occ = ndimage.binary_closing(occ, structure=ndimage.generate_binary_structure(3, 1), iterations=r)
    occ = ndimage.binary_fill_holes(occ)
    v, f, _, _ = marching_cubes(occ.astype(np.float32), level=0.5, spacing=(pitch,) * 3)
    out = trimesh.Trimesh(v + lo + 0.5 * pitch, f, process=True)
    trimesh.smoothing.filter_taubin(out, iterations=smooth_iters)
    if out.volume < 0:
        out.invert()
    return outer_shells(out)
