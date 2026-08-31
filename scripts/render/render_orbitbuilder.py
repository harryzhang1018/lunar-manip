"""Render a Chrono Blender export of the OrbitBuilder demo with Cycles.

Reads the `exported.assets.py` + `output/stateNNNNN.py` files that
`scenarios/TrackedVeh_OrbitBuilder.py --export-blender DIR` writes, imports them
with Chrono's `chrono_import` add-on, restyles every body by name (the stock
export carries the M113's game textures, a stretched dirt tile on the terrain and
flat brown rocks), frames a camera on the builder, and renders one frame -- or,
with `--animate`, the whole sequence -- with Cycles on the GPU.

Runs inside Blender, headless:

    ~/blender-5.1.2-linux-x64/blender --background --factory-startup \\
        --python scripts/render/render_orbitbuilder.py -- \\
        --export-dir artifacts/blender/trackedveh_orbitbuilder --frame 0 \\
        --output artifacts/renders/orbitbuilder/frame0000.png

(`scripts/render/render.sh` wraps that.) Blender >= 4.1 is required: the
pychrono 10 exporter emits `bpy.ops.object.shade_auto_smooth` for its cylinder
primitives, which 4.0 does not have. `--factory-startup` matters: the add-on
must not be pre-loaded from the user prefs, or enabling it again here breaks its
operator registration; it is enabled explicitly below from the user add-ons dir
(`~/.config/blender/<ver>/scripts/addons/chrono_import.py`).

How the add-on works, and what that forces here: it builds the scene for one
frame at a time. `scene.frame_set(f)` fires its `callback_post`, which deletes
every object in the `chrono_frame_objects` collection and re-creates them from
`stateNNNNN.py` -- so anything done to those objects (materials, visibility) is
lost on the next frame change, and rendering itself fires the handler too. Hence
`apply_style` runs after every `frame_set`, and the handler is removed around each
render call (`freeze_chrono_frame`) and restored afterwards.

Materials are assigned on the *mesh data*, which the add-on shares between every
body drawing the same Chrono visual shape (all rocks of one variant, say), so a
scene of thousands of rocks costs a dozen assignments; the per-object material
override the add-on installs is switched back to the mesh's.

Camera: by default a raised view from the builder's right (the rock-pile side)
across the hull onto the wall, anchored on the M113 hull and offset in the hull's
own frame (`--cam-back/--cam-side/--cam-height`, aiming `--look-*` relative to it),
so the same settings frame the builder at any station of the orbit. `--cam-pos`
and `--look-at` (world coordinates) override that.

Adapted from NeDM's `blender-render/render_tracked_arm_rollout.py`.
"""

from __future__ import annotations

import argparse
import atexit
import json
import math
import os
import sys
import time
from pathlib import Path

import bpy
from mathutils import Quaternion, Vector


def blender_script_args() -> list[str]:
    if "--" not in sys.argv:
        return []
    return sys.argv[sys.argv.index("--") + 1:]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Render a Chrono OrbitBuilder Blender export.")
    p.add_argument("--export-dir", type=Path, default=None,
                   help="Directory written by --export-blender (holds exported.assets.py "
                        "and output/); or give --asset directly.")
    p.add_argument("--asset", type=Path, default=None, help="Chrono *.assets.py file.")
    p.add_argument("--output", type=Path, default=None,
                   help="Output PNG (single frame). Default: <export-dir>/renders/frameNNNN.png")
    p.add_argument("--frame", type=int, default=0, help="Frame (state file index) to render.")
    p.add_argument("--time", type=float, default=None,
                   help="Sim time (s) to render instead of --frame; looked up in "
                        "blender_export_summary.json (nearest exported frame).")
    p.add_argument("--width", type=int, default=1920)
    p.add_argument("--height", type=int, default=1080)
    p.add_argument("--samples", type=int, default=128, help="Cycles samples per pixel.")
    p.add_argument("--device", default="OPTIX", choices=("OPTIX", "CUDA", "CPU"),
                   help="Cycles device; falls back to CPU if the GPU backend is unavailable.")
    p.add_argument("--engine", default="CYCLES", choices=("CYCLES", "EEVEE"),
                   help="EEVEE is a quick preview (no proper shadows/GI in background mode).")
    # Camera in the hull frame: x forward, y left, z up (Chrono vehicle ISO frame).
    p.add_argument("--cam-back", type=float, default=6.0, help="Behind the hull (m); negative = ahead.")
    p.add_argument("--cam-side", type=float, default=-13.0,
                   help="Lateral offset (m); negative = right side, the rock-pile side.")
    p.add_argument("--cam-height", type=float, default=7.0, help="Above the ground (m).")
    p.add_argument("--look-ahead", type=float, default=-1.0, help="Aim point ahead of the hull (m).")
    p.add_argument("--look-side", type=float, default=2.0,
                   help="Aim point lateral offset (m); positive = left, the wall side.")
    p.add_argument("--look-height", type=float, default=0.8, help="Aim point height (m).")
    p.add_argument("--lens", type=float, default=35.0, help="Focal length (mm, 36 mm sensor).")
    p.add_argument("--cam-pos", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
                   help="Camera position in world coordinates (overrides the hull-frame offsets).")
    p.add_argument("--look-at", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
                   help="Camera aim point in world coordinates.")
    p.add_argument("--ortho", type=float, default=None,
                   help="Use an orthographic camera with this scale (m across the frame).")
    # Light and sky.
    p.add_argument("--sun-elevation", type=float, default=38.0, help="Sun elevation (deg).")
    p.add_argument("--sun-azimuth", type=float, default=-60.0,
                   help="Sun azimuth (deg from +x toward +y, world frame).")
    p.add_argument("--sun-strength", type=float, default=3.0)
    p.add_argument("--sky-strength", type=float, default=0.6, help="World (ambient) strength.")
    p.add_argument("--bg-color", type=float, nargs=3, default=(0.62, 0.72, 0.85),
                   metavar=("R", "G", "B"), help="Sky colour.")
    p.add_argument("--exposure", type=float, default=0.0, help="Colour-management exposure (stops).")
    p.add_argument("--view-transform", default="AgX", choices=("AgX", "Filmic", "Standard"))
    p.add_argument("--no-shadows", action="store_true")
    p.add_argument("--hide-markers", action="store_true",
                   help="Hide the orbit rings and place-point discs.")
    p.add_argument("--hide-terrain", action="store_true",
                   help="Hide the terrain patch (e.g. to composite your own ground).")
    # Animation.
    p.add_argument("--animate", action="store_true",
                   help="Render frames [--anim-start, --anim-end] as PNGs into --anim-dir.")
    p.add_argument("--anim-start", type=int, default=0)
    p.add_argument("--anim-end", type=int, default=None, help="Default: last exported frame.")
    p.add_argument("--anim-dir", type=Path, default=None,
                   help="Default: <export-dir>/renders/anim")
    p.add_argument("--skip-existing", action="store_true")
    p.add_argument("--debug", action="store_true", help="Print the object classification.")
    args = p.parse_args(blender_script_args())

    if args.asset is None:
        if args.export_dir is None:
            p.error("give --export-dir or --asset")
        args.asset = args.export_dir / "exported.assets.py"
    if args.export_dir is None:
        args.export_dir = args.asset.parent
    args.asset = args.asset.expanduser().resolve()
    args.export_dir = args.export_dir.expanduser().resolve()
    if not args.asset.is_file():
        p.error(f"asset file not found: {args.asset}")
    return args


# ---------------------------------------------------------------------------
# export bookkeeping
# ---------------------------------------------------------------------------

def read_summary(export_dir: Path) -> dict:
    path = export_dir / "blender_export_summary.json"
    if path.is_file():
        return json.loads(path.read_text(encoding="utf-8"))
    return {}


def count_state_files(export_dir: Path) -> int:
    out = export_dir / "output"
    return len(list(out.glob("state*.py"))) if out.is_dir() else 0


def frame_for_time(summary: dict, t: float) -> int:
    times = summary.get("frame_times") or []
    if not times:
        raise SystemExit("--time needs frame_times in blender_export_summary.json")
    return min(range(len(times)), key=lambda i: abs(times[i] - t))


# ---------------------------------------------------------------------------
# add-on / frame handler plumbing
# ---------------------------------------------------------------------------

def enable_chrono_addon() -> None:
    import addon_utils

    if not hasattr(bpy.types, "IMPORT_CHRONO_OT_data"):
        try:
            addon_utils.enable("chrono_import", default_set=False, persistent=True)
        except Exception as exc:  # noqa: BLE001
            print(f"[warn] chrono_import enable raised (continuing): {exc}")
    if not hasattr(bpy.types, "IMPORT_CHRONO_OT_data"):
        raise RuntimeError(
            "chrono_import operator unavailable. Install Chrono's chrono_import.py "
            "(chrono/src/importer_blender/for_blender_X/) into "
            "~/.config/blender/<ver>/scripts/addons/ and enable it once in the UI.")


def freeze_chrono_frame() -> list:
    removed = []
    for handler in list(bpy.app.handlers.frame_change_post):
        if getattr(handler, "__name__", "") == "callback_post":
            bpy.app.handlers.frame_change_post.remove(handler)
            removed.append(handler)
    return removed


def restore_chrono_handlers(handlers: list) -> None:
    for handler in handlers:
        if handler not in bpy.app.handlers.frame_change_post:
            bpy.app.handlers.frame_change_post.append(handler)


# ---------------------------------------------------------------------------
# materials
# ---------------------------------------------------------------------------

def _principled(mat):
    return mat.node_tree.nodes.get("Principled BSDF")


def make_material(name, color, roughness=0.6, metallic=0.0, alpha=1.0):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = (*color[:3], alpha)
    mat.use_nodes = True
    bsdf = _principled(mat)
    if bsdf:
        bsdf.inputs["Base Color"].default_value = (*color[:3], 1.0)
        bsdf.inputs["Roughness"].default_value = roughness
        bsdf.inputs["Metallic"].default_value = metallic
        if "Alpha" in bsdf.inputs:
            bsdf.inputs["Alpha"].default_value = alpha
    if alpha < 1.0:
        try:
            mat.blend_method = "BLEND"
        except (AttributeError, TypeError):
            pass
    return mat


def make_rock_material(name="rock_regolith"):
    """Grey-brown rock with per-object tint jitter (Object Info -> Random), so a
    wall of a dozen shared meshes does not read as tiling, and a fine noise bump."""
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nt = mat.node_tree
    bsdf = _principled(mat)
    bsdf.inputs["Roughness"].default_value = 0.85
    bsdf.inputs["Metallic"].default_value = 0.0

    info = nt.nodes.new("ShaderNodeObjectInfo")
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.interpolation = "LINEAR"
    ramp.color_ramp.elements[0].color = (0.16, 0.13, 0.11, 1.0)
    ramp.color_ramp.elements[1].color = (0.36, 0.31, 0.26, 1.0)
    mid = ramp.color_ramp.elements.new(0.5)
    mid.color = (0.26, 0.22, 0.18, 1.0)
    nt.links.new(info.outputs["Random"], ramp.inputs["Fac"])
    nt.links.new(ramp.outputs["Color"], bsdf.inputs["Base Color"])

    noise = nt.nodes.new("ShaderNodeTexNoise")
    noise.inputs["Scale"].default_value = 60.0
    noise.inputs["Detail"].default_value = 6.0
    coords = nt.nodes.new("ShaderNodeTexCoord")
    nt.links.new(coords.outputs["Object"], noise.inputs["Vector"])
    bump = nt.nodes.new("ShaderNodeBump")
    bump.inputs["Strength"].default_value = 0.35
    bump.inputs["Distance"].default_value = 0.02
    nt.links.new(noise.outputs["Fac"], bump.inputs["Height"])
    nt.links.new(bump.outputs["Normal"], bsdf.inputs["Normal"])
    mat.diffuse_color = (0.26, 0.22, 0.18, 1.0)
    return mat


def make_ground_material(name="regolith_ground"):
    """Matte grey regolith: large-scale noise mottling plus a fine grain bump."""
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nt = mat.node_tree
    bsdf = _principled(mat)
    bsdf.inputs["Roughness"].default_value = 0.95
    bsdf.inputs["Metallic"].default_value = 0.0

    coords = nt.nodes.new("ShaderNodeTexCoord")
    mapping = nt.nodes.new("ShaderNodeMapping")
    mapping.inputs["Scale"].default_value = (0.35, 0.35, 0.35)
    nt.links.new(coords.outputs["Object"], mapping.inputs["Vector"])
    big = nt.nodes.new("ShaderNodeTexNoise")
    big.inputs["Scale"].default_value = 1.0
    big.inputs["Detail"].default_value = 4.0
    big.inputs["Roughness"].default_value = 0.6
    nt.links.new(mapping.outputs["Vector"], big.inputs["Vector"])
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].position = 0.35
    ramp.color_ramp.elements[0].color = (0.46, 0.44, 0.41, 1.0)
    ramp.color_ramp.elements[1].position = 0.7
    ramp.color_ramp.elements[1].color = (0.60, 0.58, 0.54, 1.0)
    nt.links.new(big.outputs["Fac"], ramp.inputs["Fac"])
    nt.links.new(ramp.outputs["Color"], bsdf.inputs["Base Color"])

    fine = nt.nodes.new("ShaderNodeTexNoise")
    fine.inputs["Scale"].default_value = 40.0
    fine.inputs["Detail"].default_value = 8.0
    nt.links.new(mapping.outputs["Vector"], fine.inputs["Vector"])
    bump = nt.nodes.new("ShaderNodeBump")
    bump.inputs["Strength"].default_value = 0.25
    bump.inputs["Distance"].default_value = 0.05
    nt.links.new(fine.outputs["Fac"], bump.inputs["Height"])
    nt.links.new(bump.outputs["Normal"], bsdf.inputs["Normal"])
    mat.diffuse_color = (0.43, 0.40, 0.37, 1.0)
    return mat


def build_materials():
    return {
        "hull": make_material("m113_hull", (0.72, 0.73, 0.71), roughness=0.55, metallic=0.05),
        "track": make_material("track_rubber", (0.05, 0.05, 0.05), roughness=0.85),
        "wheel": make_material("road_wheel", (0.22, 0.22, 0.23), roughness=0.5, metallic=0.2),
        "arm": make_material("arm_link", (0.85, 0.85, 0.86), roughness=0.45, metallic=0.1),
        "gripper": make_material("gripper", (0.30, 0.30, 0.32), roughness=0.5, metallic=0.3),
        "rock": make_rock_material(),
        "ground": make_ground_material(),
        "ring_wall": make_material("ring_wall", (0.90, 0.72, 0.10), roughness=0.5),
        "ring_vehicle": make_material("ring_vehicle", (0.20, 0.50, 0.90), roughness=0.5),
        "disc": make_material("place_disc", (0.95, 0.30, 0.05), roughness=0.5),
    }


# ---------------------------------------------------------------------------
# object classification
# ---------------------------------------------------------------------------

def body_name(obj) -> str:
    """The Chrono body name: the add-on parents each shape to an empty named after
    the body; the shape objects themselves are called shape_<id>.NNN."""
    parent = obj.parent
    while parent is not None and parent.parent is not None:
        parent = parent.parent
    return (parent.name if parent is not None else obj.name)


DARK_TOKENS = ("trackshoe", "_shoe")
WHEEL_TOKENS = ("roadwheel", "idler", "sprocket", "suspension", "_wheel", "_gear", "_arm", "_carrier")
ARM_TOKENS = ("base-1", "shoulder-1", "bicep-1", "elbow-1", "wrist-1")
GRIPPER_TOKENS = ("endeffector-1", "finger-1", "finger-2")


def classify(obj) -> str:
    name = body_name(obj).lower()
    if name.startswith("patch"):
        return "ground"
    if name.startswith(("rock_", "wall_")) or (name[:1] == "s" and "_" in name
                                                and name.split("_", 1)[0][1:].isdigit()):
        return "rock"
    if name.startswith(("orbit_markers", "place_markers")):
        return "marker"
    if any(t in name for t in DARK_TOKENS):
        return "track"
    if any(t in name for t in WHEEL_TOKENS):
        return "wheel"
    if any(t in name for t in GRIPPER_TOKENS):
        return "gripper"
    if any(t in name for t in ARM_TOKENS):
        return "arm"
    if "chassis" in name or "m113" in name:
        return "hull"
    return "hull"


def set_mesh_material(mesh, mat) -> None:
    mesh.materials.clear()
    mesh.materials.append(mat)


def marker_material(obj, materials):
    """Ring segments are thin boxes (long x, tiny z); discs are short cylinders.
    Which ring is which we read from the imported material's colour."""
    if obj.dimensions.z > 0.019 and obj.dimensions.x < 0.5 and obj.dimensions.y < 0.5:
        return materials["disc"]
    for slot in obj.material_slots:
        src = slot.material
        if src is None:
            continue
        # The add-on builds its materials from nodes only; diffuse_color stays default.
        bsdf = _principled(src) if src.node_tree else None
        rgb = (bsdf.inputs["Base Color"].default_value[:3] if bsdf else src.diffuse_color[:3])
        return materials["ring_vehicle"] if rgb[2] > rgb[0] else materials["ring_wall"]
    return materials["ring_wall"]


def apply_style(args, materials, quiet=False):
    """Restyle the CURRENT chrono_frame_objects; returns {class: [objects]}."""
    coll = bpy.data.collections.get("chrono_frame_objects")
    if coll is None:
        raise RuntimeError("chrono_frame_objects collection not found after import")
    objects = list(getattr(coll, "all_objects", coll.objects))
    groups: dict[str, list] = {}
    mesh_done = {}
    for obj in objects:
        if obj.type == "EMPTY":
            obj.empty_display_size = 0.0001
            obj.hide_select = True
            continue
        if obj.type != "MESH":
            obj.hide_render = obj.hide_viewport = True
            continue
        kind = classify(obj)
        groups.setdefault(kind, []).append(obj)
        if kind == "marker":
            mat = marker_material(obj, materials)
            hide = args.hide_markers
        else:
            mat = materials[kind]
            hide = args.hide_terrain and kind == "ground"
        obj.hide_render = obj.hide_viewport = hide
        # The add-on links each slot to an OBJECT-level material; the mesh data
        # is shared between every body drawing the same shape, so style that
        # once and point the slots back at it.
        key = obj.data.name
        if mesh_done.get(key) is not mat:
            set_mesh_material(obj.data, mat)
            mesh_done[key] = mat
        for slot in obj.material_slots:
            slot.link = "DATA"
        # Rocks are shaded smooth by the exporter; the edge-split modifier it
        # adds keeps the facets crisp. Fine.
    if not quiet:
        counts = {k: len(v) for k, v in sorted(groups.items())}
        print(f"[style] {counts}", flush=True)
        if args.debug:
            for kind, objs in sorted(groups.items()):
                names = sorted({body_name(o) for o in objs})
                print(f"  {kind}: {names[:12]}{' ...' if len(names) > 12 else ''}")
    return groups


# ---------------------------------------------------------------------------
# camera, light, render settings
# ---------------------------------------------------------------------------

def hull_frame(groups):
    """World position and yaw quaternion of the M113 hull (from its parent empty)."""
    for obj in groups.get("hull", []):
        if "chassis" in body_name(obj).lower():
            root = obj
            while root.parent is not None:
                root = root.parent
            return root.matrix_world.translation.copy(), root.matrix_world.to_quaternion()
    raise RuntimeError("no chassis body found to anchor the camera on")


def look_at(obj, target: Vector) -> None:
    direction = target - obj.location
    obj.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()


def setup_camera(scene, args, groups):
    cam_data = bpy.data.cameras.new("render_camera")
    cam = bpy.data.objects.new("render_camera", cam_data)
    scene.collection.objects.link(cam)
    scene.camera = cam

    if args.cam_pos is not None and args.look_at is not None:
        pos, aim = Vector(args.cam_pos), Vector(args.look_at)
    else:
        hull_pos, hull_rot = hull_frame(groups)
        yaw_only = Quaternion((0.0, 0.0, 1.0), hull_rot.to_euler().z)
        rel_cam = Vector((-args.cam_back, args.cam_side, 0.0))
        rel_aim = Vector((args.look_ahead, args.look_side, 0.0))
        pos = hull_pos + yaw_only @ rel_cam
        pos.z = args.cam_height
        aim = hull_pos + yaw_only @ rel_aim
        aim.z = args.look_height
        if args.cam_pos is not None:
            pos = Vector(args.cam_pos)
        if args.look_at is not None:
            aim = Vector(args.look_at)
    cam.location = pos
    look_at(cam, aim)
    if args.ortho:
        cam_data.type = "ORTHO"
        cam_data.ortho_scale = float(args.ortho)
    else:
        cam_data.type = "PERSP"
        cam_data.lens = float(args.lens)
    cam_data.clip_start = 0.1
    cam_data.clip_end = 2000.0
    print(f"[camera] pos={tuple(round(v, 2) for v in pos)} aim={tuple(round(v, 2) for v in aim)}",
          flush=True)
    return cam


def setup_lights_and_world(scene, args):
    sun_data = bpy.data.lights.new("sun", "SUN")
    sun = bpy.data.objects.new("sun", sun_data)
    scene.collection.objects.link(sun)
    sun_data.energy = float(args.sun_strength)
    sun_data.angle = math.radians(1.5)  # slightly soft shadow edges
    sun_data.color = (1.0, 0.97, 0.92)
    elev, azim = math.radians(args.sun_elevation), math.radians(args.sun_azimuth)
    # Point the light *from* (elev, azim) toward the origin: -Z of the lamp is its beam.
    direction = -Vector((math.cos(elev) * math.cos(azim),
                         math.cos(elev) * math.sin(azim),
                         math.sin(elev)))
    sun.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
    if args.no_shadows:
        for attr in ("use_shadow",):
            try:
                setattr(sun_data, attr, False)
            except (AttributeError, TypeError):
                pass
        try:
            sun_data.cycles.cast_shadow = False
        except (AttributeError, TypeError):
            pass

    world = scene.world or bpy.data.worlds.new("World")
    scene.world = world
    world.use_nodes = True
    r, g, b = args.bg_color
    world.color = (r, g, b)
    bg = world.node_tree.nodes.get("Background")
    if bg is not None:
        bg.inputs["Color"].default_value = (r, g, b, 1.0)
        bg.inputs["Strength"].default_value = float(args.sky_strength)


def setup_engine(scene, args) -> str:
    scene.render.resolution_x = int(args.width)
    scene.render.resolution_y = int(args.height)
    scene.render.resolution_percentage = 100
    scene.render.film_transparent = False
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGB"
    for transform in (args.view_transform, "AgX", "Filmic", "Standard"):
        try:
            scene.view_settings.view_transform = transform
            break
        except TypeError:
            continue
    scene.view_settings.exposure = float(args.exposure)
    scene.view_settings.gamma = 1.0

    if args.engine == "EEVEE":
        for name in ("BLENDER_EEVEE_NEXT", "BLENDER_EEVEE"):
            try:
                scene.render.engine = name
                break
            except TypeError:
                continue
        return scene.render.engine

    scene.render.engine = "CYCLES"
    scene.cycles.samples = int(args.samples)
    scene.cycles.use_denoising = True
    scene.cycles.max_bounces = 6
    scene.cycles.diffuse_bounces = 3
    scene.cycles.glossy_bounces = 3
    scene.cycles.transparent_max_bounces = 8
    device = "CPU"
    if args.device != "CPU":
        prefs = bpy.context.preferences.addons.get("cycles")
        if prefs is not None:
            cp = prefs.preferences
            try:
                cp.compute_device_type = args.device
                cp.get_devices()
                enabled = 0
                for dev in cp.devices:
                    use = dev.type == args.device
                    dev.use = use
                    enabled += int(use)
                if enabled:
                    scene.cycles.device = "GPU"
                    device = args.device
                    try:
                        scene.cycles.denoiser = "OPTIX" if args.device == "OPTIX" else "OPENIMAGEDENOISE"
                    except TypeError:
                        pass
            except (TypeError, AttributeError) as exc:
                print(f"[warn] {args.device} unavailable ({exc}); rendering on CPU")
    if device == "CPU":
        scene.cycles.device = "CPU"
        try:
            scene.cycles.denoiser = "OPENIMAGEDENOISE"
        except TypeError:
            pass
    return f"CYCLES/{device}"


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def render_still(scene, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    scene.render.filepath = str(path)
    frozen = freeze_chrono_frame()
    try:
        bpy.ops.render.render(write_still=True)
    finally:
        restore_chrono_handlers(frozen)


def main() -> None:
    args = parse_args()
    summary = read_summary(args.export_dir)
    n_frames = summary.get("state_file_count") or count_state_files(args.export_dir)
    if n_frames == 0:
        raise SystemExit(f"no output/state*.py frames under {args.export_dir}")
    if args.time is not None:
        args.frame = frame_for_time(summary, args.time)
    times = summary.get("frame_times") or []

    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()
    enable_chrono_addon()
    t0 = time.time()
    bpy.ops.import_chrono.data(filepath=str(args.asset), setting_materials=True,
                               setting_merge=False)
    print(f"[import] {args.asset} in {time.time() - t0:.1f}s; {n_frames} frames exported",
          flush=True)

    scene = bpy.context.scene
    scene.frame_start = 0
    scene.frame_end = n_frames - 1
    materials = build_materials()
    setup_lights_and_world(scene, args)
    engine = setup_engine(scene, args)
    print(f"[engine] {engine}, {args.width}x{args.height}, {args.samples} spp", flush=True)

    if args.animate:
        anim_dir = (args.anim_dir or args.export_dir / "renders" / "anim").expanduser().resolve()
        anim_dir.mkdir(parents=True, exist_ok=True)
        start = int(args.anim_start)
        end = n_frames - 1 if args.anim_end is None else min(int(args.anim_end), n_frames - 1)
        scene.frame_set(start)
        bpy.context.view_layer.update()
        groups = apply_style(args, materials)
        setup_camera(scene, args, groups)   # fixed camera, framed on the first frame
        total, done, t0 = end - start + 1, 0, time.time()
        for f in range(start, end + 1):
            out = anim_dir / f"frame{f:04d}.png"
            if args.skip_existing and out.exists() and out.stat().st_size > 0:
                done += 1
                continue
            scene.frame_set(f)
            bpy.context.view_layer.update()
            apply_style(args, materials, quiet=True)
            render_still(scene, out)
            done += 1
            if done % 10 == 0 or done == total:
                el = time.time() - t0
                print(f"[anim] {done}/{total} ({el / done:.1f}s/frame, "
                      f"ETA {(total - done) * el / done / 60:.1f} min)", flush=True)
        print(f"ANIM_DONE {done} frames -> {anim_dir}", flush=True)
        return

    frame = max(0, min(int(args.frame), n_frames - 1))
    scene.frame_set(frame)
    bpy.context.view_layer.update()
    frozen = freeze_chrono_frame()
    atexit.register(restore_chrono_handlers, frozen)
    groups = apply_style(args, materials)
    setup_camera(scene, args, groups)
    out = args.output or (args.export_dir / "renders" / f"frame{frame:04d}.png")
    out = out.expanduser().resolve()
    t_label = f" (t={times[frame]:.2f} s)" if frame < len(times) else ""
    t0 = time.time()
    render_still(scene, out)
    print(f"RENDERED frame {frame}{t_label} -> {out} in {time.time() - t0:.1f}s", flush=True)
    # Put the add-on's handler back now (its unregister at exit expects it).
    restore_chrono_handlers(frozen)


if __name__ == "__main__":
    main()
