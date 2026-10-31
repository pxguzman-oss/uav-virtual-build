"""Blender-side render script (run by render.py; Blender's own Python, no venv).

blender -b --factory-startup -P render_blender.py -- MODEL_DIR OUT_DIR \
        --engine CYCLES --samples 32 --res 1600 1000 --views iso,top,side,front,xray

Reads MODEL_DIR/manifest.json (+ mass_properties.json if present), imports
parts/*.stl (mm -> m), assigns materials by part role/colour, renders each view and
writes OUT_DIR/<view>.png plus OUT_DIR/<view>_labels.json (2D pixel positions of
component centres + CG markers) for the label overlay done outside Blender.
"""
import argparse
import json
import math
import os
import sys
import time

import bpy
from bpy_extras.object_utils import world_to_camera_view
from mathutils import Vector

S = 0.001  # mm -> m


def args():
    # works both as `blender -b -P render_blender.py -- ARGS` and `python render_blender.py ARGS` (pip bpy)
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[1:]
    ap = argparse.ArgumentParser()
    ap.add_argument("model_dir")
    ap.add_argument("out_dir")
    ap.add_argument("--engine", default="CYCLES", choices=["CYCLES", "EEVEE", "BLENDER_WORKBENCH"])
    ap.add_argument("--samples", type=int, default=32)
    ap.add_argument("--res", type=int, nargs=2, default=[1600, 1000])
    ap.add_argument("--views", default="iso,top,side,front,xray")
    ap.add_argument("--threads", type=int, default=0)
    ap.add_argument("--no-denoise", action="store_true")
    ap.add_argument("--outline", action="store_true", help="Freestyle outlines (technical look, slower)")
    ap.add_argument("--shadow", action="store_true", help="Cycles shadow-catcher ground plane (slow; needs OIDN to be clean)")
    return ap.parse_args(argv)


def reset():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def import_stl(path):
    if hasattr(bpy.ops.wm, "stl_import"):          # Blender >= 4.1 (C++ importer)
        bpy.ops.wm.stl_import(filepath=path)
    else:                                          # 4.0 and older (python addon)
        if not hasattr(bpy.ops.import_mesh, "stl"):
            bpy.ops.preferences.addon_enable(module="io_mesh_stl")
        bpy.ops.import_mesh.stl(filepath=path)
    ob = bpy.context.selected_objects[0]
    ob.scale = (S, S, S)
    return ob


def _blend(m):
    """Alpha blending across Blender 4.0 (blend_method) and 4.2+/5.x (surface_render_method)."""
    if hasattr(m, "surface_render_method"):
        m.surface_render_method = "BLENDED"
    elif hasattr(m, "blend_method"):
        m.blend_method = "BLEND"
    if hasattr(m, "shadow_method"):
        m.shadow_method = "NONE"


def _denoiser_available():
    try:
        import _cycles
        return bool(getattr(_cycles, "with_openimagedenoise", False))
    except Exception:
        return False


def principled(name, color, rough=0.5, metal=0.0, alpha=1.0, emit=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*color[:3], 1)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metal
    b.inputs["Alpha"].default_value = alpha
    if emit:
        key = "Emission Color" if "Emission Color" in b.inputs else "Emission"
        b.inputs[key].default_value = (*color[:3], 1)
        b.inputs["Emission Strength"].default_value = emit
    if alpha < 1:
        _blend(m)
    m.diffuse_color = (*color[:3], alpha)  # used by Workbench "MATERIAL" colouring
    return m


def xray_skin_material():
    """Glass-like ghost skin: mostly transparent, faint fresnel edges."""
    m = bpy.data.materials.new("xray_skin")
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    mix = nt.nodes.new("ShaderNodeMixShader")
    tr = nt.nodes.new("ShaderNodeBsdfTransparent")
    em = nt.nodes.new("ShaderNodeBsdfDiffuse")
    em.inputs["Color"].default_value = (0.55, 0.75, 1.0, 1)
    lw = nt.nodes.new("ShaderNodeLayerWeight")
    lw.inputs["Blend"].default_value = 0.25
    mr = nt.nodes.new("ShaderNodeMapRange")
    mr.inputs["To Min"].default_value = 0.06
    mr.inputs["To Max"].default_value = 0.55
    nt.links.new(lw.outputs["Facing"], mr.inputs["Value"])
    nt.links.new(mr.outputs["Result"], mix.inputs["Fac"])
    nt.links.new(tr.outputs[0], mix.inputs[1])
    nt.links.new(em.outputs[0], mix.inputs[2])
    nt.links.new(mix.outputs[0], out.inputs["Surface"])
    _blend(m)
    m.diffuse_color = (0.55, 0.75, 1.0, 0.15)
    return m


def add_light(name, kind, loc, energy, size=1.0, rot=None):
    d = bpy.data.lights.new(name, kind)
    d.energy = energy
    if kind == "AREA":
        d.size = size
    o = bpy.data.objects.new(name, d)
    o.location = loc
    if rot:
        o.rotation_euler = rot
    bpy.context.collection.objects.link(o)
    return o


def look_at(obj, target):
    d = Vector(target) - obj.location
    obj.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()


def setup_world(engine):
    w = bpy.data.worlds.new("w")
    bpy.context.scene.world = w
    w.use_nodes = True
    bg = w.node_tree.nodes["Background"]
    bg.inputs["Color"].default_value = (0.80, 0.84, 0.90, 1)
    bg.inputs["Strength"].default_value = 0.35
    w.color = (0.92, 0.93, 0.95)


def build_scene(model_dir, xray):
    man = json.load(open(os.path.join(model_dir, "manifest.json")))
    mp_path = os.path.join(model_dir, "mass_properties.json")
    mp = json.load(open(mp_path)) if os.path.exists(mp_path) else None
    mats = {}
    objs = {}
    for p in man["parts"]:
        if not p.get("file"):
            continue
        ob = import_stl(os.path.join(model_dir, p["file"]))
        ob.name = p["name"]
        col = p.get("color") or [0.6, 0.6, 0.6]
        role = p.get("role")
        if role == "structure":
            key = "skin_x" if xray else "skin"
            if key not in mats:
                mats[key] = xray_skin_material() if xray else principled("skin", col, rough=0.45)
            m = mats[key]
            # smooth shading for organic skins
            for f in ob.data.polygons:
                f.use_smooth = True
        elif role == "prop":
            m = mats.setdefault("prop", principled("prop", [0.15, 0.15, 0.15], alpha=0.18 if not xray else 0.10))
        elif role == "tube":
            m = principled(p["name"], col, rough=0.35, metal=0.2)
        else:
            m = principled(p["name"], col, rough=0.4, emit=0.6 if xray else 0.0)
        ob.data.materials.append(m)
        objs[p["name"]] = (ob, p)
    markers = {}
    if mp:
        cg = Vector(mp["cg_struct_mm"]) * S
        bpy.ops.mesh.primitive_uv_sphere_add(radius=0.012, location=cg)
        s = bpy.context.active_object
        s.name = "CG"
        s.data.materials.append(principled("cg", [1.0, 0.85, 0.0], emit=2.0))
        s.hide_render = not xray
        markers["CG"] = cg
        if "cg_target_x_mm" in mp:
            t = Vector((mp["cg_target_x_mm"], 0, mp["cg_struct_mm"][2])) * S
            bpy.ops.mesh.primitive_torus_add(location=t, major_radius=0.016, minor_radius=0.003,
                                             rotation=(math.pi / 2, 0, 0))
            r = bpy.context.active_object
            r.name = "CG_target"
            r.data.materials.append(principled("cgt", [0.0, 0.8, 0.2], emit=2.0))
            r.hide_render = not xray
            markers["CG target"] = t
    return man, mp, objs, markers


def scene_bounds(objs):
    import numpy as np  # Blender ships numpy (python3-numpy on Ubuntu)
    pts = []
    for ob, p in objs.values():
        if p.get("role") == "prop":
            continue
        pts += [ob.matrix_world @ Vector(c) for c in ob.bound_box]
    a = np.array([[v.x, v.y, v.z] for v in pts])
    return a.min(0), a.max(0)


VIEWS = {
    # name: (direction from target to camera, orthographic?, framing, x-ray materials?)
    "iso":      ((-1.0, -0.95, 1.05), False, "all", False),
    "top":      ((0.0, 0.0, 1.0), True, "all", False),
    "side":     ((0.0, -1.0, 0.0), True, "all", False),
    "front":    ((-1.0, 0.0, 0.0), True, "all", False),
    "rear":     ((1.2, 0.9, 0.7), False, "all", False),
    "xray":     ((-0.75, -1.0, 1.25), False, "components", True),
    "xray_top": ((0.0, 0.0, 1.0), True, "all", True),
    "xray_side": ((0.0, -1.0, 0.0), True, "components", True),
}


def make_camera(view, lo, hi, res, margin=1.0):
    import numpy as np
    c = (lo + hi) / 2
    if view.startswith("xray_top"):
        view = "top"
    elif view.startswith("xray_side"):
        view = "side"
    ext = hi - lo
    d = Vector(VIEWS[view][0]).normalized()
    ortho = VIEWS[view][1]
    cam_d = bpy.data.cameras.new("cam")
    cam = bpy.data.objects.new("cam", cam_d)
    bpy.context.collection.objects.link(cam)
    aspect = res[0] / res[1]
    if ortho:
        cam_d.type = "ORTHO"
        if view == "top":
            # planform, nose up, right wing on the right: image x = +y, image up = -x
            cam.location = Vector(c) + d * 3
            cam.rotation_euler = (0, 0, math.pi / 2)
            w, h = ext[1], ext[0]
        elif view == "side":
            cam.location = Vector(c) + d * 3
            cam.rotation_euler = (math.pi / 2, 0, 0)
            w, h = ext[0], ext[2]
        else:  # front, looking aft from the nose
            cam.location = Vector(c) + d * 3
            cam.rotation_euler = (math.pi / 2, 0, -math.pi / 2)
            w, h = ext[1], ext[2]
        cam_d.ortho_scale = 1.10 * margin * max(w, h * aspect)
    else:
        cam_d.lens = 50
        r = 0.5 * float(np.linalg.norm(ext))
        fov = 2 * math.atan(cam_d.sensor_width / 2 / cam_d.lens)
        dist = 0.95 * margin * r / math.sin(fov / 2)
        cam.location = Vector(c) + d * dist
        look_at(cam, c)
    cam_d.clip_end = 100
    bpy.context.scene.camera = cam
    return cam


def setup_render(a):
    sc = bpy.context.scene
    eng = a.engine
    if eng == "EEVEE":   # 4.2-4.x call it BLENDER_EEVEE_NEXT, 4.0/4.1 and 5.x BLENDER_EEVEE
        ids = [e.identifier for e in sc.render.bl_rna.properties["engine"].enum_items]
        eng = "BLENDER_EEVEE_NEXT" if "BLENDER_EEVEE_NEXT" in ids else "BLENDER_EEVEE"
    sc.render.engine = eng
    sc.render.resolution_x, sc.render.resolution_y = a.res
    sc.render.resolution_percentage = 100
    sc.render.film_transparent = True   # composite on a clean background later (render.py)
    sc.render.image_settings.file_format = "PNG"
    if a.threads:
        sc.render.threads_mode = "FIXED"
        sc.render.threads = a.threads
    try:
        sc.view_settings.view_transform = "AgX" if "AgX" in [i.identifier for i in sc.view_settings.bl_rna.properties["view_transform"].enum_items] else "Filmic"
    except Exception:
        pass
    for look in ("AgX - Medium High Contrast", "Medium High Contrast"):
        try:
            sc.view_settings.look = look
            break
        except Exception:
            pass
    if a.engine == "CYCLES":
        sc.cycles.device = "CPU"
        sc.cycles.samples = a.samples
        sc.cycles.use_adaptive_sampling = True
        sc.cycles.max_bounces = 6
        sc.cycles.transparent_max_bounces = 12
        sc.cycles.use_denoising = (not a.no_denoise) and _denoiser_available()
        try:
            sc.cycles.denoiser = "OPENIMAGEDENOISE"
        except TypeError:
            pass
    if getattr(a, "outline", False):
        sc.render.use_freestyle = True
        sc.render.line_thickness_mode = "ABSOLUTE"
        sc.render.line_thickness = 1.2
        ls = sc.view_layers[0].freestyle_settings.linesets[0] if sc.view_layers[0].freestyle_settings.linesets else \
            sc.view_layers[0].freestyle_settings.linesets.new("lines")
        ls.select_by_visibility = True
        ls.linestyle.color = (0.15, 0.17, 0.2)
    if a.engine == "BLENDER_WORKBENCH":
        sh = sc.display.shading
        sh.light = "STUDIO"
        sh.color_type = "MATERIAL"
        sh.show_cavity = True
        sh.cavity_type = "BOTH"
        sh.show_object_outline = True
        sh.show_shadows = True
        sh.show_xray = False
        sc.display.render_aa = "16"
    elif a.engine == "EEVEE":
        sc.eevee.taa_render_samples = a.samples


def lights(lo, hi):
    c = (lo + hi) / 2
    span = float(max(hi - lo))
    k = lambda o: tuple(Vector(c) + Vector(o))
    key = add_light("key", "AREA", k((-1.2 * span, -1.0 * span, 1.4 * span)), 55 * span**2, size=0.8 * span)
    look_at(key, c)
    fill = add_light("fill", "AREA", k((1.0 * span, 1.2 * span, 0.6 * span)), 18 * span**2, size=span)
    look_at(fill, c)
    rim = add_light("rim", "AREA", k((1.5 * span, -0.3 * span, -0.2 * span)), 20 * span**2, size=0.5 * span)
    look_at(rim, c)
    add_light("sun", "SUN", (0, 0, 5), 0.6, rot=(0.35, -0.25, 0))


def label_positions(cam, objs, markers, res):
    sc = bpy.context.scene
    out = []
    for name, (ob, p) in objs.items():
        if p.get("role") not in ("component",) or not p.get("label"):
            continue
        bb = [ob.matrix_world @ Vector(v) for v in ob.bound_box]
        cen = sum(bb, Vector()) / 8
        co = world_to_camera_view(sc, cam, cen)
        out.append(dict(name=name, label=p["label"], group=p.get("group"), color=p.get("color"),
                        px=[co.x * res[0], (1 - co.y) * res[1]], depth=co.z))
    for name, v in markers.items():
        co = world_to_camera_view(sc, cam, v)
        out.append(dict(name=name, label=name, group="marker", px=[co.x * res[0], (1 - co.y) * res[1]], depth=co.z))
    return out


def main():
    a = args()
    os.makedirs(a.out_dir, exist_ok=True)
    timings = {}
    for view in a.views.split(","):
        reset()
        _, _, frame, xray = VIEWS[view]
        man, mp, objs, markers = build_scene(a.model_dir, xray)
        setup_world(a.engine)
        lo, hi = scene_bounds(objs)
        if frame == "components":   # zoom on the fuselage and its contents
            import numpy as np
            sel = {k: v for k, v in objs.items() if v[1].get("role") == "component"}
            clo, chi = scene_bounds(sel)
            pad = 0.06
            lo = np.maximum(lo, clo - pad)
            hi = np.minimum(hi, chi + pad)
            lo[1], hi[1] = max(lo[1], -0.3), min(hi[1], 0.3)
        margin = {"xray": 1.6, "xray_side": 1.3, "xray_top": 1.25}.get(view, 1.0)   # room for label columns
        cam = make_camera(view, lo, hi, a.res, margin=margin)
        lights(lo, hi)
        if a.shadow and not xray and a.engine == "CYCLES":
            # soft ground shadow (composited via transparent film); only with OIDN, it is noisy otherwise
            bpy.ops.mesh.primitive_plane_add(size=8, location=(float((lo[0] + hi[0]) / 2), float((lo[1] + hi[1]) / 2), float(lo[2]) - 0.12))
            bpy.context.active_object.is_shadow_catcher = True
        setup_render(a)
        sc = bpy.context.scene
        if a.engine == "BLENDER_WORKBENCH" and xray:
            sc.display.shading.show_xray = False
        sc.render.filepath = os.path.join(a.out_dir, f"{view}.png")
        t0 = time.perf_counter()
        bpy.ops.render.render(write_still=True)
        timings[view] = round(time.perf_counter() - t0, 2)
        json.dump(label_positions(cam, objs, markers, a.res), open(os.path.join(a.out_dir, f"{view}_labels.json"), "w"))
        print(f"[render_blender] {view}: {timings[view]} s", flush=True)
    json.dump(dict(engine=a.engine, samples=a.samples, res=a.res, timings_s=timings, blender=bpy.app.version_string,
                   denoise=bpy.context.scene.cycles.use_denoising if a.engine == "CYCLES" else None),
              open(os.path.join(a.out_dir, "render_timings.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
