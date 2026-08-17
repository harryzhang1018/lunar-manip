"""Build a .blend file directly from a Chrono::Postprocess ChBlender export folder.

This replaces the workflow of installing Chrono's own `chrono_import.py` add-on
and using File > Import > Chrono import inside Blender's UI. That add-on works
by registering a frame-change handler that tears down and rebuilds the whole
per-frame object set every time the timeline is scrubbed -- convenient for
live playback, but it means the animation is never actually baked: nothing
plays back correctly without the add-on installed and its handler alive, and
scrubbing performance degrades with scene size.

This script instead reads the same export format (one `*.assets.py` file
defining the static/shared meshes and materials, plus `output/stateNNNNN.py`
files each describing one simulated frame) and bakes everything into ordinary
Blender keyframes on ordinary objects, once, up front. The resulting .blend
opens, scrubs, and renders anywhere -- no add-on required.

Usage (via the real Blender executable's bundled Python -- `bpy` is not
pip-installable independently of it):

    blender --background --python blend_from_chrono_export.py -- \\
        <export_dir> <output.blend> [fps]

<export_dir> is the folder ChBlender wrote (e.g. .../BLENDER), containing one
`*.assets.py` file and an `output/` directory of `stateNNNNN.py` files.
[fps] defaults to 30, matching this repo's scenarios' export cadence.

Example -- regenerate orbit_builder.blend, carrying over the manually-built
terrain/camera/lighting/world shader from an earlier hand-assembled file:

    # blender --background --python scripts/blend_from_chrono_export.py -- \\
    #   DEMO_OUTPUT/BLENDER ~/Documents/BlenderDocuments/blenderfiles/blendFiles/orbit_builder.blend 30 \\
    #   ~/Documents/BlenderDocuments/blenderfiles/blendFiles/builder5.blend "Collection.001" \\
    #   ~/Documents/BlenderDocuments/blenderfiles/blendFiles/builder5.blend "Collection 6" \\
    #   --world ~/Documents/BlenderDocuments/blenderfiles/blendFiles/builder5.blend "World" \\
    #   --texture ~/Downloads/rock rock \\
    #   --robot-texture ~/Poliigon/Library/MetalGalvanizedSteelWorn001 "M113_TrackShoeLeft_shoe,M113_TrackShoeRight_shoe"
"""

import glob
import math
import os
import re
import sys

import bpy
import mathutils


def parse_args():
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1:]
    else:
        argv = []
    if len(argv) < 2:
        raise SystemExit(
            "usage: blender --background --python blend_from_chrono_export.py "
            "-- <export_dir> <output.blend> [fps] "
            "[<append_blend> <collection_name>]... "
            "[--world <append_blend> <world_name>] "
            "[--texture <folder> <family_name>[,<family_name>...]]... "
            "[--robot-texture <folder> <comma_separated_exclude_families_or_''>]")
    export_dir = argv[0]
    out_path = argv[1]
    fps = int(argv[2]) if len(argv) > 2 else 30
    rest = argv[3:]
    appends = []
    world_append = None
    textures = []
    robot_texture = None
    i = 0
    while i < len(rest):
        if rest[i] == "--world":
            world_append = (rest[i + 1], rest[i + 2])
            i += 3
        elif rest[i] == "--texture":
            textures.append((rest[i + 1], rest[i + 2]))
            i += 3
        elif rest[i] == "--robot-texture":
            robot_texture = (rest[i + 1], rest[i + 2])
            i += 3
        else:
            appends.append((rest[i], rest[i + 1]))
            i += 2
    return export_dir, out_path, fps, appends, world_append, textures, robot_texture


def frame_index(path):
    return int(re.search(r"(\d+)\.py$", os.path.basename(path)).group(1))


def family_key(name):
    """`name` with any purely-numeric '_'/'-'-delimited token dropped, so e.g.
    'M113_RoadWheelLeft_0_wheel' and '..._4_wheel' collapse to the same key
    ('M113_RoadWheelLeft_wheel'), and 'finger-1'/'finger-2' collapse to
    'finger'. Digits embedded in a word ('M113') are left alone since they
    aren't their own token."""
    tokens = [t for t in re.split(r"([_\- ])", name) if not t.isdigit()]
    collapsed = re.sub(r"[_\- ]{2,}", "_", "".join(tokens))
    return collapsed.strip("_- ") or name


def group_into_family_collections(instances_collection):
    """Move every object that shares a `family_key` with at least one other
    object into a sub-collection named after that key, so e.g. all 173 track
    shoes end up together instead of loose in chrono_instances. Objects whose
    name is unique even after stripping numbers are left where they are."""
    families = {}
    for obj in instances_collection.objects:
        families.setdefault(family_key(obj.name), []).append(obj)

    for key, members in families.items():
        if len(members) < 2:
            continue
        sub = bpy.data.collections.new(key)
        instances_collection.children.link(sub)
        for obj in members:
            instances_collection.objects.unlink(obj)
            sub.objects.link(obj)


def append_collection(filepath, collection_name):
    """Copy a named collection (and everything it references -- objects,
    meshes, materials) from another .blend file into the current one, and
    link it under the scene. Used to carry over manually-built scene dressing
    (camera, lighting, terrain) that isn't part of the Chrono export itself."""
    filepath = os.path.abspath(os.path.expanduser(filepath))
    with bpy.data.libraries.load(filepath, link=False) as (data_from, data_to):
        if collection_name not in data_from.collections:
            print(f"  warning: collection '{collection_name}' not found in {filepath} "
                 f"(has: {list(data_from.collections)})")
            return
        data_to.collections = [collection_name]
    for coll in data_to.collections:
        if coll is not None:
            bpy.context.scene.collection.children.link(coll)
            print(f"  appended collection '{coll.name}' from {filepath}")


def append_world(filepath, world_name):
    """Copy a named World datablock (background/environment shader settings)
    from another .blend file and make it the current scene's world."""
    filepath = os.path.abspath(os.path.expanduser(filepath))
    with bpy.data.libraries.load(filepath, link=False) as (data_from, data_to):
        if world_name not in data_from.worlds:
            print(f"  warning: world '{world_name}' not found in {filepath} "
                 f"(has: {list(data_from.worlds)})")
            return
        data_to.worlds = [world_name]
    for world in data_to.worlds:
        if world is not None:
            bpy.context.scene.world = world
            print(f"  appended world '{world.name}' from {filepath} and set as scene world")


# Token keywords (matched against a filename's '_'/'-'-delimited stem, after
# stripping trailing resolution/workflow tags -- see `_core_tokens` below)
# identifying each PBR map across Poliigon's various naming conventions, e.g.
# 'Rock020_4K-JPG_Color.jpg' and 'MetalGalvanizedSteelWorn001_COL_2K_METALNESS.jpg'.
_TEXTURE_MAP_KEYWORDS = {
    "color": ("color", "albedo", "basecolor", "diffuse", "col"),
    "roughness": ("roughness", "rough"),
    "metalness": ("metalness", "metallic", "metal"),
    "normal": ("normalgl", "normal_gl", "normal", "nrm"),
    "ao": ("ambientocclusion", "ao"),
    "displacement": ("displacement", "height", "disp"),
}

# Trailing tokens that are resolution/workflow tags, not the map type -- e.g.
# every file in a Poliigon "Metalness workflow" set ends in '_METALNESS'
# regardless of which map it actually is, and '_2K' is just the resolution.
_TEXTURE_NON_MAP_SUFFIXES = {"metalness", "specular", "metallic", "gloss"}


def _core_tokens(filename):
    """`filename`'s stem split into lowercase tokens, with a trailing
    workflow-tag token (e.g. 'metalness') and then a trailing resolution
    token (e.g. '2k') stripped -- each at most once, and in that order, since
    the real map-type token can itself equal a workflow-tag word (the
    metalness map in a "Metalness workflow" set is literally named
    '..._METALNESS_2K_METALNESS.ext'; only the very last one is the tag)."""
    stem = os.path.splitext(filename)[0]
    tokens = [t.lower() for t in re.split(r"[_\-]+", stem) if t]
    if tokens and tokens[-1] in _TEXTURE_NON_MAP_SUFFIXES:
        tokens = tokens[:-1]
    if tokens and re.fullmatch(r"\d+k", tokens[-1]):
        tokens = tokens[:-1]
    return tokens


def build_pbr_material_from_folder(name, folder):
    """Build a Principled BSDF material by auto-detecting and wiring up the
    image maps in `folder`, matching what Node Wrangler's 'Add Principled
    Texture Setup' does in the UI. That operator is bound to an active Shader
    Editor context (a selected Principled node in an open node-editor area),
    which isn't reliably driveable from a --background run with no UI, so
    this reproduces its node graph directly instead."""
    folder = os.path.abspath(os.path.expanduser(folder))
    files = os.listdir(folder)
    file_tokens = {fname: _core_tokens(fname) for fname in files}

    def find(map_key):
        for keyword in _TEXTURE_MAP_KEYWORDS[map_key]:
            for fname, tokens in file_tokens.items():
                if keyword in tokens:
                    return os.path.join(folder, fname)
        return None

    paths = {key: find(key) for key in _TEXTURE_MAP_KEYWORDS}
    found = {k: v for k, v in paths.items() if v}
    print(f"  texture '{name}' from {folder}: {list(found)}")

    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    nodes.clear()

    output = nodes.new("ShaderNodeOutputMaterial")
    output.location = (400, 0)
    bsdf = nodes.new("ShaderNodeBsdfPrincipled")
    bsdf.location = (0, 0)
    links.new(bsdf.outputs["BSDF"], output.inputs["Surface"])

    def add_image_node(path, non_color, y):
        img = bpy.data.images.load(path)
        img.colorspace_settings.name = 'Non-Color' if non_color else 'sRGB'
        node = nodes.new("ShaderNodeTexImage")
        node.image = img
        node.location = (-600, y)
        return node

    y = 400
    if paths["color"]:
        color_node = add_image_node(paths["color"], non_color=False, y=y)
        if paths["ao"]:
            ao_node = add_image_node(paths["ao"], non_color=True, y=y - 300)
            mix = nodes.new("ShaderNodeMixRGB")
            mix.blend_type = 'MULTIPLY'
            mix.inputs["Fac"].default_value = 1.0
            mix.location = (-300, y)
            links.new(color_node.outputs["Color"], mix.inputs["Color1"])
            links.new(ao_node.outputs["Color"], mix.inputs["Color2"])
            links.new(mix.outputs["Color"], bsdf.inputs["Base Color"])
        else:
            links.new(color_node.outputs["Color"], bsdf.inputs["Base Color"])
        y -= 300
    if paths["roughness"]:
        rough_node = add_image_node(paths["roughness"], non_color=True, y=y)
        links.new(rough_node.outputs["Color"], bsdf.inputs["Roughness"])
        y -= 300
    if paths["metalness"]:
        metal_node = add_image_node(paths["metalness"], non_color=True, y=y)
        links.new(metal_node.outputs["Color"], bsdf.inputs["Metallic"])
        y -= 300
    if paths["normal"]:
        normal_img_node = add_image_node(paths["normal"], non_color=True, y=y)
        normal_map_node = nodes.new("ShaderNodeNormalMap")
        normal_map_node.location = (-300, y)
        links.new(normal_img_node.outputs["Color"], normal_map_node.inputs["Color"])
        links.new(normal_map_node.outputs["Normal"], bsdf.inputs["Normal"])
        y -= 300
    if paths["displacement"]:
        disp_img_node = add_image_node(paths["displacement"], non_color=True, y=y)
        disp_node = nodes.new("ShaderNodeDisplacement")
        disp_node.location = (100, -400)
        links.new(disp_img_node.outputs["Color"], disp_node.inputs["Height"])
        links.new(disp_node.outputs["Displacement"], output.inputs["Displacement"])

    return mat


# Name prefixes identifying the M113 vehicle and the gripper arm (see
# make_bsdf_material call sites in a state file dump, and the SearchBody
# names in model/arm_model.py) -- used to scope UV unwrapping to "the robot"
# without touching orbit_markers, place discs, or terrain.
_ROBOT_NAME_PREFIXES = (
    "M113", "Chassis", "base-", "shoulder-", "bicep", "elbow-", "wrist-",
    "endeffector-", "finger-",
)


def is_robot(name):
    return any(name.startswith(p) for p in _ROBOT_NAME_PREFIXES)


def is_robot_or_rock(name):
    return name.startswith("rock") or is_robot(name)


def all_objects_in_collection(collection):
    """Every object under `collection`, including ones sitting in nested
    sub-collections (e.g. the per-family collections `group_into_family_collections`
    builds) -- `collection.objects` alone only sees the loose top-level ones."""
    objs = list(collection.objects)
    for child in collection.children:
        objs.extend(all_objects_in_collection(child))
    return objs


def get_family_objects(instances_collection, family_name):
    """Objects in the named family sub-collection built by
    `group_into_family_collections`, or the single loose object of that name
    if it was never grouped (no other object shared its stripped name)."""
    sub = instances_collection.children.get(family_name)
    if sub:
        return list(sub.objects)
    return [o for o in instances_collection.objects if o.name == family_name]


def apply_material_to_family(instances_collection, family_name, material):
    """Assign `material` (replacing whatever's there) to every object in the
    named family."""
    objs = get_family_objects(instances_collection, family_name)
    if not objs:
        print(f"  warning: no objects found for family '{family_name}'")
        return
    for obj in objs:
        obj.data.materials.clear()
        obj.data.materials.append(material)
    print(f"  applied material '{material.name}' to {len(objs)} object(s) in '{family_name}'")


def smart_uv_unwrap(objects, angle_limit_degrees=66.0):
    """Smart UV Project every unique mesh among `objects`, once per mesh
    datablock even if several objects share it (Chrono's exported rock meshes
    have no usable UVs of their own, so an image texture applied without this
    would map garbage). Unlike Node Wrangler's texture-setup operator, this
    one only needs an active object in Edit Mode, not a live Shader Editor
    area, so it runs fine in a --background session."""
    view_layer = bpy.context.view_layer
    seen = set()
    for obj in objects:
        if obj.type != 'MESH' or obj.data.name in seen:
            continue
        seen.add(obj.data.name)
        for o in view_layer.objects:
            o.select_set(False)
        obj.select_set(True)
        view_layer.objects.active = obj
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.uv.smart_project(angle_limit=math.radians(angle_limit_degrees))
        bpy.ops.object.mode_set(mode='OBJECT')
    print(f"  smart-UV-unwrapped {len(seen)} unique mesh(es) across {len(objects)} object(s)")


def get_or_create_collection(name, hide_render=False):
    coll = bpy.data.collections.get(name)
    if coll is None:
        coll = bpy.data.collections.new(name)
        bpy.context.scene.collection.children.link(coll)
    coll.hide_render = hide_render
    return coll


# ---------------------------------------------------------------------------
# Custom functions the exported .py files call. Names, signatures and behavior
# mirror src/importer_blender/for_blender_5.0/chrono_import.py in the Chrono
# source tree, since the exported scripts are generated to run against that
# add-on's namespace -- these are drop-in equivalents, not reinterpretations.
# ---------------------------------------------------------------------------

def make_bsdf_material_pre40(nameID, colorRGB, metallic=0, specular=0, specular_tint=0,
                             roughness=0.5, index_refraction=1.450, transmission=0,
                             emissionRGB=(0, 0, 0, 1), emission_strength=1,
                             bump_map=0, bump_height=1.0):
    new_mat = bpy.data.materials.new(name=nameID)
    new_mat.use_nodes = True
    nodes = new_mat.node_tree.nodes
    bsdf = nodes.get("Principled BSDF")
    if isinstance(colorRGB, tuple):
        bsdf.inputs["Base Color"].default_value = colorRGB
    if isinstance(metallic, (int, float)):
        bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Specular"].default_value = specular
    if isinstance(roughness, (int, float)):
        bsdf.inputs["Roughness"].default_value = roughness
    bsdf.inputs["IOR"].default_value = index_refraction
    if isinstance(transmission, (int, float)):
        bsdf.inputs["Transmission"].default_value = transmission
    if isinstance(emissionRGB, tuple):
        bsdf.inputs["Emission"].default_value = emissionRGB
    bsdf.inputs["Emission Strength"].default_value = emission_strength
    return new_mat


def make_bsdf_material_modern(nameID, colorRGB, metallic=0, specular=0, specular_tint=(1, 0, 0, 0),
                              roughness=0.5, index_refraction=1.450, transmission=0,
                              emissionRGB=(0, 0, 0, 1), emission_strength=1,
                              bump_map=0, bump_height=1.0):
    new_mat = bpy.data.materials.new(name=nameID)
    new_mat.use_nodes = True
    nodes = new_mat.node_tree.nodes
    bsdf = nodes.get("Principled BSDF")
    if isinstance(colorRGB, tuple):
        bsdf.inputs["Base Color"].default_value = colorRGB
    if isinstance(metallic, (int, float)):
        bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Specular IOR Level"].default_value = specular
    bsdf.inputs["Specular Tint"].default_value = specular_tint
    if isinstance(roughness, (int, float)):
        bsdf.inputs["Roughness"].default_value = roughness
    bsdf.inputs["IOR"].default_value = index_refraction
    if isinstance(transmission, (int, float)):
        bsdf.inputs["Transmission Weight"].default_value = transmission
    if isinstance(emissionRGB, tuple):
        bsdf.inputs["Emission Color"].default_value = emissionRGB
    bsdf.inputs["Emission Strength"].default_value = emission_strength
    return new_mat


# Chrono's own add-on picks between these two by Blender version, since the
# Principled BSDF socket names changed in 4.0 -- match that switch exactly.
make_bsdf_material = (make_bsdf_material_pre40 if bpy.app.version < (4, 0, 0)
                      else make_bsdf_material_modern)


def create_chrono_path(name_id, list_points, rgba_color, line_width,
                       my_list_materials, my_collection):
    """Grease-pencil poly-line asset, for ChLinePath-type visuals (e.g. track
    chains). Every export this repo's scenarios have produced so far declares
    these with all-zero points and never references them from a per-frame
    state file, so this mostly just has to not crash; it is implemented for
    real in case a future export does use it."""
    try:
        gpencil_data = bpy.data.grease_pencils.new(name_id)
        gpencil = bpy.data.objects.new(name_id, gpencil_data)
        my_collection.objects.link(gpencil)
        gp_layer = gpencil_data.layers.new("chrono_lines")
        gp_layer.use_lights = False
        gp_frame = gp_layer.frames.new(bpy.context.scene.frame_current)
        if list_points:
            gp_frame.drawing.add_strokes([len(list_points)])
            gp_stroke = gp_frame.drawing.strokes[0]
            for i, point in enumerate(list_points):
                gp_stroke.points[i].position = point
                gp_stroke.points[i].radius = line_width
        mat = bpy.data.materials.new(name="chrono_path_mat")
        bpy.data.materials.create_gpencil_data(mat)
        gpencil.data.materials.append(mat)
        mat.grease_pencil.color = rgba_color
        my_list_materials.append(mat)
        return gpencil
    except AttributeError:
        # Grease-pencil-v3 API (Blender 4.3+) not available -- fall back to a
        # plain curve so older Blenders still get a valid, if plainer, asset.
        curve_data = bpy.data.curves.new(name_id, type='CURVE')
        curve_data.dimensions = '3D'
        curve_data.bevel_depth = max(line_width, 0.001) * 0.5
        if list_points:
            spline = curve_data.splines.new('POLY')
            spline.points.add(len(list_points) - 1)
            for i, point in enumerate(list_points):
                spline.points[i].co = (point[0], point[1], point[2], 1.0)
        obj = bpy.data.objects.new(name_id, curve_data)
        my_collection.objects.link(obj)
        mat = bpy.data.materials.new(name="chrono_path_mat")
        mat.diffuse_color = rgba_color
        obj.data.materials.append(mat)
        my_list_materials.append(mat)
        return obj


class SceneBuilder:
    """Replays `make_chrono_object_assetlist` calls from every frame's state
    file as real, baked keyframes on persistent objects -- built once per
    body/asset the first time it's seen, updated and keyframed on every frame
    after that -- rather than the add-on's destroy-and-rebuild-per-scrub
    approach.
    """

    def __init__(self, assets_collection, instances_collection):
        self.assets_collection = assets_collection
        self.instances_collection = instances_collection
        self.children = {}  # (body name, asset slot index) -> instanced object
        self.frame = 0

    def make_chrono_object_assetlist(self, mname, mpos, mrot, masset_list):
        # No per-body Empty: bake each shape's transform directly in world
        # space by composing it with the body's transform ourselves, instead
        # of parenting to an Empty like the real add-on does. That keeps
        # chrono_instances made up of nothing but real mesh objects, so
        # box-selecting and batch-assigning materials in the viewport isn't
        # cluttered by hundreds of axis-only parent objects.
        parent_pos = mathutils.Vector(mpos)
        parent_rot = mathutils.Quaternion(mrot)

        for i, entry in enumerate(masset_list):
            asset_name, rel_pos, rel_rot = entry[0], entry[1], entry[2]
            mat_names = entry[3] if len(entry) > 3 else []
            scale = entry[4] if len(entry) > 4 else None
            key = (mname, i)
            chasset = self.children.get(key)
            if chasset is None:
                template = self.assets_collection.objects.get(asset_name)
                if template is None:
                    print(f"  warning: asset '{asset_name}' not found, skipping")
                    continue
                chasset = template.copy()  # instanced -- .data (mesh) stays shared
                # Name it after the body it belongs to (what the removed Empty
                # used to be called), not the shared asset's shape_<ptr> name --
                # that's meaningless once there's no parent to read it off of.
                chasset.name = mname if len(masset_list) == 1 else f"{mname}_{i}"
                chasset.rotation_mode = 'QUATERNION'
                self.instances_collection.objects.link(chasset)
                if mat_names:
                    while len(chasset.data.materials) < len(mat_names):
                        chasset.data.materials.append(None)
                    for slot_i, matname in enumerate(mat_names):
                        # 'OBJECT' link keeps this instance's material
                        # independent of every other body sharing the same
                        # mesh data -- otherwise assigning a material to one
                        # rock/wheel in Blender would repaint every instance
                        # of that shared mesh.
                        chasset.material_slots[slot_i].link = 'OBJECT'
                        chasset.material_slots[slot_i].material = bpy.data.materials.get(matname)
                self.children[key] = chasset

            world_rot = parent_rot @ mathutils.Quaternion(rel_rot)
            world_pos = parent_pos + parent_rot @ mathutils.Vector(rel_pos)
            chasset.location = world_pos
            chasset.rotation_quaternion = world_rot
            chasset.keyframe_insert(data_path="location", frame=self.frame)
            chasset.keyframe_insert(data_path="rotation_quaternion", frame=self.frame)
            if scale is not None:
                chasset.scale = scale
                chasset.keyframe_insert(data_path="scale", frame=self.frame)


def main():
    export_dir, out_path, fps, appends, world_append, textures, robot_texture = parse_args()

    assets_files = glob.glob(os.path.join(export_dir, "*.assets.py"))
    if not assets_files:
        raise SystemExit(f"no *.assets.py found in {export_dir}")
    assets_file = assets_files[0]

    state_files = sorted(glob.glob(os.path.join(export_dir, "output", "state*.py")),
                         key=frame_index)
    if not state_files:
        raise SystemExit(f"no output/state*.py files found in {export_dir}/output")

    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.context.scene.render.fps = fps
    bpy.context.scene.render.engine = 'BLENDER_EEVEE'
    bpy.context.scene.eevee.use_raytracing = True
    bpy.context.scene.eevee.shadow_step_count = 10

    assets_collection = get_or_create_collection("chrono_assets", hide_render=True)
    instances_collection = get_or_create_collection("chrono_instances")

    chrono_materials = []
    base_ns = {
        "bpy": bpy,
        "chrono_assets": assets_collection,
        "chrono_materials": chrono_materials,
        "create_chrono_path": create_chrono_path,
        "make_bsdf_material": make_bsdf_material,
        "chrono_view_contacts": False,
    }

    print(f"building static assets from {assets_file} ...")
    with open(assets_file, "rb") as f:
        exec(compile(f.read(), assets_file, "exec"), dict(base_ns))

    for obj in list(assets_collection.objects):
        obj.hide_set(True)
        obj.hide_render = True

    builder = SceneBuilder(assets_collection, instances_collection)
    for n, path in enumerate(state_files):
        builder.frame = frame_index(path)
        frame_ns = dict(base_ns)
        frame_ns["make_chrono_object_assetlist"] = builder.make_chrono_object_assetlist
        with open(path, "rb") as f:
            exec(compile(f.read(), path, "exec"), frame_ns)
        if n % 100 == 0:
            print(f"  frame {builder.frame} ({n + 1}/{len(state_files)}) ...")

    group_into_family_collections(instances_collection)

    unwrap_targets = [o for o in all_objects_in_collection(instances_collection)
                     if is_robot_or_rock(o.name)]
    smart_uv_unwrap(unwrap_targets)

    for folder, family_names_csv in textures:
        family_names = [f.strip() for f in family_names_csv.split(",") if f.strip()]
        material = build_pbr_material_from_folder(f"{'_'.join(family_names)}_pbr", folder)
        for family_name in family_names:
            apply_material_to_family(instances_collection, family_name, material)

    if robot_texture is not None:
        folder, excludes_csv = robot_texture
        excludes = {e.strip() for e in excludes_csv.split(",") if e.strip()}
        material = build_pbr_material_from_folder("robot_pbr", folder)
        targets = [o for o in all_objects_in_collection(instances_collection)
                  if is_robot(o.name) and family_key(o.name) not in excludes]
        for obj in targets:
            obj.data.materials.clear()
            obj.data.materials.append(material)
        print(f"  applied material 'robot_pbr' to {len(targets)} robot object(s)"
             f"{', excluding ' + str(excludes) if excludes else ''}")

    for append_path, collection_name in appends:
        append_collection(append_path, collection_name)

    if world_append is not None:
        append_world(*world_append)

    bpy.context.scene.frame_start = frame_index(state_files[0])
    bpy.context.scene.frame_end = frame_index(state_files[-1])
    bpy.context.scene.frame_set(bpy.context.scene.frame_start)

    out_path = os.path.abspath(out_path)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=out_path)
    print(f"saved {out_path}  ({len(state_files)} frames, {len(builder.children)} shapes)")


if __name__ == "__main__":
    main()
