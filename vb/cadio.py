"""STEP assembly I/O on OpenCascade (pip: cadquery-ocp): named solids in, named solids out.

read_step(path)            -> [Body]  every leaf solid of the assembly with its full name path, placed
write_step(path, bodies)   -> flat assembly, one named solid per body (names survive a Fusion re-import)
Frame                      -> CAD coordinates (mm, any axes) <-> pipeline design frame (m, x aft, y right, z up)
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


def _ocp():
    try:
        import OCP  # noqa: F401
    except ImportError as e:  # pragma: no cover
        raise RuntimeError("STEP support needs OpenCascade: pip install cadquery-ocp (setup/install.sh does this)") from e


@dataclass
class Body:
    name: str                      # leaf name
    path: str                      # "Assembly/Component:1/Body1"
    shape: object                  # located TopoDS_Shape (CAD frame, mm)
    volume: float = 0.0            # mm3
    area: float = 0.0              # mm2
    centroid: np.ndarray = field(default_factory=lambda: np.zeros(3))          # volume centroid, CAD mm
    surf_centroid: np.ndarray = field(default_factory=lambda: np.zeros(3))
    bbox: np.ndarray = field(default_factory=lambda: np.zeros((2, 3)))         # CAD mm
    inertia: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))      # about centroid, unit density, mm5

    @property
    def key(self):
        return norm(self.path)


def norm(s):
    """'P012_Battery 6S v3:1' -> 'p012_battery_6s' (drop Fusion version / instance suffixes)."""
    s = re.sub(r"\s+v\d+\b", "", str(s))
    s = re.sub(r":\d+\b", "", s)
    s = re.sub(r"\(\d+\)$", "", s.strip())
    return re.sub(r"[^a-z0-9]+", "_", s.lower()).strip("_")


def _props(shape):
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    v, s = GProp_GProps(), GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, v)
    BRepGProp.SurfaceProperties_s(shape, s)
    c, cs = v.CentreOfMass(), s.CentreOfMass()
    m = v.MatrixOfInertia()
    I = np.array([[m.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)])
    b = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, b, False, False)
    lo, hi = b.CornerMin(), b.CornerMax()
    x0, y0, z0, x1, y1, z1 = lo.X(), lo.Y(), lo.Z(), hi.X(), hi.Y(), hi.Z()
    return (abs(v.Mass()), s.Mass(), np.array([c.X(), c.Y(), c.Z()]), np.array([cs.X(), cs.Y(), cs.Z()]),
            np.array([[x0, y0, z0], [x1, y1, z1]]), I)


class _quiet:
    """OpenCascade prints transfer statistics straight to fd 1: silence them."""

    def __enter__(self):
        import os, sys
        sys.stdout.flush()
        self.fd = os.dup(1)
        self.null = os.open(os.devnull, os.O_WRONLY)
        os.dup2(self.null, 1)

    def __exit__(self, *a):
        import os
        os.dup2(self.fd, 1)
        os.close(self.fd); os.close(self.null)


def read_step(path):
    _ocp()
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TDocStd import TDocStd_Document
    from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDF import TDF_Label
    try:
        from OCP.TDF import TDF_LabelSequence
    except ImportError:                      # OCP >= 8 moved the collection classes
        from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
    from OCP.TDataStd import TDataStd_Name
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopAbs import TopAbs_SOLID
    from OCP.TopExp import TopExp_Explorer

    doc = TDocStd_Document(TCollection_ExtendedString("vb"))
    rd = STEPCAFControl_Reader()
    rd.SetNameMode(True)
    with _quiet():
        ok = rd.ReadFile(str(path)) == IFSelect_RetDone
        if ok:
            rd.Transfer(doc)
    if not ok:
        raise ValueError(f"cannot read STEP file {path}")
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    def name_of(label):
        a = TDataStd_Name()
        return a.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), a) else ""

    out = []

    def walk(label, loc, path):
        nm = name_of(label)
        if XCAFDoc_ShapeTool.IsReference_s(label):
            ref = TDF_Label()
            XCAFDoc_ShapeTool.GetReferredShape_s(label, ref)
            l2 = loc.Multiplied(XCAFDoc_ShapeTool.GetLocation_s(label))
            rn = name_of(ref)
            walk(ref, l2, path + [nm] if nm and nm != rn else path)
            return
        here = path + ([nm] if nm else [])
        if XCAFDoc_ShapeTool.IsAssembly_s(label):
            comps = TDF_LabelSequence()
            XCAFDoc_ShapeTool.GetComponents_s(label, comps, False)
            for i in range(1, comps.Length() + 1):
                walk(comps.Value(i), loc, here)
            return
        shape = XCAFDoc_ShapeTool.GetShape_s(label).Moved(loc)
        ex = TopExp_Explorer(shape, TopAbs_SOLID)
        solids = []
        while ex.More():
            solids.append(ex.Current())
            ex.Next()
        for k, s in enumerate(solids):
            vol, area, c, cs, bb, I = _props(s)
            leaf = (here[-1] if here else Path(path).stem if isinstance(path, str) else "body") + (f"_{k+1}" if len(solids) > 1 else "")
            out.append(Body(leaf, "/".join(here) + (f"_{k+1}" if len(solids) > 1 else ""), s, vol, area, c, cs, bb, I))

    free = TDF_LabelSequence()
    st.GetFreeShapes(free)
    for i in range(1, free.Length() + 1):
        walk(free.Value(i), TopLoc_Location(), [])
    return out


def read_folder(folder):
    """One STEP per part: the body is named after the file (positions as stored in each file)."""
    out = []
    for f in sorted(Path(folder).glob("*")):
        if f.suffix.lower() in (".step", ".stp"):
            for b in read_step(f):
                b.name = f.stem if b.name in ("", "body") else b.name
                b.path = f.stem + "/" + b.path if f.stem not in b.path else b.path
                out.append(b)
    return out


def write_step(path, bodies):
    """bodies: [(name, TopoDS_Shape)] -> flat STEP assembly with names."""
    _ocp()
    from OCP.STEPCAFControl import STEPCAFControl_Writer
    from OCP.STEPControl import STEPControl_AsIs
    from OCP.TDocStd import TDocStd_Document
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.IFSelect import IFSelect_RetDone
    doc = TDocStd_Document(TCollection_ExtendedString("vb"))
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    for name, shape in bodies:
        lab = st.AddShape(shape, False)
        TDataStd_Name.Set_s(lab, TCollection_ExtendedString(name))
    w = STEPCAFControl_Writer()
    w.SetNameMode(True)
    with _quiet():
        ok = w.Transfer(doc, STEPControl_AsIs) and w.Write(str(path)) == IFSelect_RetDone
    if not ok:
        raise RuntimeError(f"cannot write {path}")
    return path


def translated(shape, d_mm):
    from OCP.gp import gp_Trsf, gp_Vec
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    t = gp_Trsf()
    t.SetTranslation(gp_Vec(float(d_mm[0]), float(d_mm[1]), float(d_mm[2])))
    return BRepBuilderAPI_Transform(shape, t, True).Shape()


def to_stl(shape, path, frame=None, deflection=0.3):
    """Tessellate a solid to STL (mm). With `frame`, the mesh is written in the design frame."""
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.StlAPI import StlAPI_Writer
    if frame is not None:
        from OCP.gp import gp_Trsf
        from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
        R, o = frame.R, frame.origin
        t = gp_Trsf()
        tr = -R @ o
        t.SetValues(R[0, 0], R[0, 1], R[0, 2], tr[0], R[1, 0], R[1, 1], R[1, 2], tr[1], R[2, 0], R[2, 1], R[2, 2], tr[2])
        shape = BRepBuilderAPI_Transform(shape, t, True).Shape()
    BRepMesh_IncrementalMesh(shape, deflection, False, 0.4, True)
    StlAPI_Writer().Write(shape, str(path))
    return path


AX = {"+x": (0, 1), "-x": (0, -1), "+y": (1, 1), "-y": (1, -1), "+z": (2, 1), "-z": (2, -1)}


class Frame:
    """design = R (cad_mm - origin_mm) / 1000.  x_aft / y_right / z_up name the CAD axis each design axis lies along."""

    def __init__(self, origin_mm=(0, 0, 0), x_aft="+x", y_right="+y", z_up="+z"):
        self.spec = dict(origin_mm=[float(v) for v in origin_mm], x_aft=x_aft.lower(), y_right=y_right.lower(), z_up=z_up.lower())
        R = np.zeros((3, 3))
        for row, key in enumerate((x_aft, y_right, z_up)):
            i, s = AX[key.lower() if key[0] in "+-" else "+" + key.lower()]
            R[row, i] = s
        if abs(abs(np.linalg.det(R)) - 1) > 1e-9:
            raise ValueError("frame axes must be three different CAD axes")
        self.R, self.origin = R, np.array(origin_mm, float)
        self.right_handed = np.linalg.det(R) > 0

    def point(self, p_mm):
        return self.R @ (np.asarray(p_mm, float) - self.origin) / 1000.0

    def vec_to_cad_mm(self, d_m):
        return self.R.T @ np.asarray(d_m, float) * 1000.0

    def extents(self, bbox_mm):
        """Bounding-box size along the design axes [m]."""
        return np.abs(self.R @ (bbox_mm[1] - bbox_mm[0])) / 1000.0

    def bbox(self, bbox_mm):
        c = np.array([self.point([bbox_mm[i][0], bbox_mm[j][1], bbox_mm[k][2]]) for i in (0, 1) for j in (0, 1) for k in (0, 1)])
        return c.min(0), c.max(0)


def suggest_frame(bodies, wing_keys=("wing",), tail_keys=("tail", "stab", "rudder", "elevator")):
    """Guess the design frame from named bodies: span = longest axis of the wing bodies, up = their thinnest
    axis, aft = towards the tail bodies. Origin = most forward point of the wing on the symmetry plane.
    Returns (Frame, notes). A guess to CONFIRM with the user (top-view render), never to trust silently."""
    wings = [b for b in bodies if any(k in b.key for k in wing_keys) and not any(k in b.key for k in tail_keys)]
    notes = []
    if not wings:
        return None, ["no body with 'wing' in its name: give the frame explicitly"]
    lo = np.min([b.bbox[0] for b in wings], 0)
    hi = np.max([b.bbox[1] for b in wings], 0)
    ext = hi - lo
    iy = int(np.argmax(ext))
    iz = int(np.argmin(ext))
    ix = 3 - iy - iz
    all_lo = np.min([b.bbox[0] for b in bodies], 0)
    all_hi = np.max([b.bbox[1] for b in bodies], 0)
    tails = [b for b in bodies if any(k in b.key for k in tail_keys)]
    wing_c = 0.5 * (lo + hi)
    if tails:
        sx = 1 if np.mean([b.centroid[ix] for b in tails]) > wing_c[ix] else -1
    else:
        sx = 1 if (all_hi[ix] - wing_c[ix]) > (wing_c[ix] - all_lo[ix]) else -1
        notes.append("no tail body found: aft taken as the longer side of the fuselage from the wing")
    below = [b for b in bodies if b not in wings and b.volume > 0]
    sz = 1
    if below:
        heavy = max(below, key=lambda b: b.volume)
        sz = 1 if wing_c[iz] >= heavy.centroid[iz] else -1
        notes.append(f"up taken as the side of the wing away from '{heavy.name}' (assumes a high or shoulder wing): confirm")
    names = "xyz"
    x_aft, z_up = ("+" if sx > 0 else "-") + names[ix], ("+" if sz > 0 else "-") + names[iz]
    for sy in ("+", "-"):
        f = Frame((0, 0, 0), x_aft, sy + names[iy], z_up)
        if f.right_handed:
            y_right = sy + names[iy]
    origin = np.zeros(3)
    origin[iy] = 0.5 * (all_lo[iy] + all_hi[iy])
    origin[ix] = lo[ix] if sx > 0 else hi[ix]
    origin[iz] = wing_c[iz]
    notes.append("origin = most forward point of the wing bodies on the symmetry plane (exact only for an unswept leading edge)")
    return Frame(origin, x_aft, y_right, z_up), notes
