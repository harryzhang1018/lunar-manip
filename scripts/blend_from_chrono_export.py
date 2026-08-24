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

    blender --background --python blend_from_chrono_export.py -- \
        <export_dir> <output.blend> [fps]

<export_dir> is the folder ChBlender wrote (e.g. .../BLENDER), containing one
`*.assets.py` file and an `output/` directory of `stateNNNNN.py` files.
[fps] defaults to 30, matching this repo's scenarios' export cadence.

Pass multiple export dirs comma-separated (no spaces) -- e.g.
`BLENDER_section1,BLENDER_section2` -- to concatenate them into ONE continuous
animation in a single .blend: each dir's own frame numbers are shifted to start
right after the previous dir's last frame, and a body name shared between dirs
(e.g. the M113 chassis, since every run's scenario code names it the same way)
just keeps animating on the same object rather than becoming a separate one.
This is how to combine `TrackedVeh_OrbitBuilder.py --two-sections`-less output --
run it once fresh and once with `--continue` (two separate Blender export
folders, since each is tied to its own Chrono system) -- into the one file the
two-command workflow can't produce on its own. A body that only exists in one
dir (e.g. that dir's own rocks) just starts or stops animating at that dir's
part of the timeline, same as it would in a single-export run.

Example -- regenerate orbit_builder.blend, carrying over the manually-built
terrain/camera/lighting/world shader from an earlier hand-assembled file.
`--texture ~/Downloads/rock rock` also automatically unwraps, UV-scales, and
textures the merged 'wall_stage_*_dir0' rock piles the same way -- no extra
flag needed, that's just what having a 'rock_pbr' material around triggers
(see merge_group_key / _bake_merged_group / the rock_material block in main()):

    blender --background --python scripts/blend_from_chrono_export.py -- \
      DEMO_OUTPUT/BLENDER_section1 ~/Documents/BlenderDocuments/blenderfiles/blendFiles/section1.blend 30 \
      ~/Documents/BlenderDocuments/blenderfiles/blendFiles/builder5.blend "Collection.001" \
      ~/Documents/BlenderDocuments/blenderfiles/blendFiles/orbit_builder2.blend "Collection 6" \
      --world ~/Documents/BlenderDocuments/blenderfiles/blendFiles/builder5.blend "World" \
      --texture ~/Downloads/rock rock \
      --texture ~/Poliigon/Library/Poliigon_PlasticMoldDryBlast_7495/2K "M113_TrackShoeLeft_shoe,M113_TrackShoeRight_shoe" \
      --robot-texture ~/Poliigon/Library/MetalGalvanizedSteelWorn001 "M113_TrackShoeLeft_shoe,M113_TrackShoeRight_shoe"

Example -- merge both sections of a two-section orbit build into one
continuous combined.blend. First produce the two Chrono export dirs (each
`TrackedVeh_OrbitBuilder.py` invocation is its own Chrono system, so each
gets its own ChBlender export -- moving the first one aside is what keeps the
second run's setup from deleting it before this script ever sees it):
    conda activate chrono
    cd lunar-manip
    python scenarios/TrackedVeh_OrbitBuilder.py --headless --export-blender
    mv DEMO_OUTPUT/BLENDER DEMO_OUTPUT/BLENDER_section1
    python scenarios/TrackedVeh_OrbitBuilder.py --headless --export-blender --continue
    mv DEMO_OUTPUT/BLENDER DEMO_OUTPUT/BLENDER_section2

That last move isn't optional either, for the same reason as the first: leaving
section 2 at the bare DEMO_OUTPUT/BLENDER path means any later run (a redo of
section 1, a third section, anything using --export-blender) deletes it again
before you get a chance to convert it.

Then merge both into one .blend:

    blender --background --python scripts/blend_from_chrono_export.py -- \
      DEMO_OUTPUT/BLENDER_section1,DEMO_OUTPUT/BLENDER_section2 \
      ~/Documents/BlenderDocuments/blenderfiles/blendFiles/combined.blend 30 \
      ~/Documents/BlenderDocuments/blenderfiles/blendFiles/builder5.blend "Collection.001" \
      ~/Documents/BlenderDocuments/blenderfiles/blendFiles/orbit_builder2.blend "Collection 6" \
      --world ~/Documents/BlenderDocuments/blenderfiles/blendFiles/builder5.blend "World" \
      --texture ~/Downloads/rock rock \
      --texture ~/Poliigon/Library/Poliigon_PlasticMoldDryBlast_7495/2K "M113_TrackShoeLeft_shoe,M113_TrackShoeRight_shoe" \
      --robot-texture ~/Poliigon/Library/MetalGalvanizedSteelWorn001 "M113_TrackShoeLeft_shoe,M113_TrackShoeRight_shoe"

full command to render one section scene from chrono.

    conda activate chrono
    cd lunar-manip
    python scenarios/TrackedVeh_OrbitBuilder.py --headless --export-blender
    mv DEMO_OUTPUT/BLENDER DEMO_OUTPUT/BLENDER_section1
    python scenarios/TrackedVeh_OrbitBuilder.py --headless --export-blender --continue
    mv DEMO_OUTPUT/BLENDER DEMO_OUTPUT/BLENDER_section2
    blender --background --python scripts/blend_from_chrono_export.py -- \
          DEMO_OUTPUT/BLENDER_section2\
          ~/Documents/BlenderDocuments/blenderfiles/blendFiles/section2.blend 30 \
          ~/Documents/BlenderDocuments/blenderfiles/blendFiles/orbit_builder2.blend "Collection.001" \
          ~/Documents/BlenderDocuments/blenderfiles/blendFiles/orbit_builder2.blend "Collection 6" \
          --world ~/Documents/BlenderDocuments/blenderfiles/blendFiles/builder5.blend "World" \
          --texture ~/Downloads/rock rock \
          --texture ~/Poliigon/Library/Poliigon_PlasticMoldDryBlast_7495/2K "M113_TrackShoeLeft_shoe,M113_TrackShoeRight_shoe" \
          --robot-texture ~/Poliigon/Library/MetalGalvanizedSteelWorn001 "M113_TrackShoeLeft_shoe,M113_TrackShoeRight_shoe"
    cd /home/zacharyrichmond/Documents/BlenderDocuments/blenderfiles/blendFiles
    blender -b section1.blend -s 12 -e 4048 -o //builder/builder1 -F FFMPEG -a
    blender -b section2.blend -s 5 -e 5090 -o //builder/builder2 -F FFMPEG -a
    blender -b section1.blend -E CYCLES -s 12 -e 4048 -o //builder/builder1 -F FFMPEG -a
  printf "file '%s'\nfile '%s'\n" /home/zacharyrichmond/Documents/BlenderDocuments/blenderfiles/blendFiles/builder/builder10012-4048.mkv /home/zacharyrichmond/Documents/BlenderDocuments/blenderfiles/blendFiles/builder/builder20005-5090.mkv > /home/zacharyrichmond/Documents/BlenderDocuments/blenderfiles/blendFiles/builder/concat_list.txt

ffmpeg -f concat -safe 0 -i /home/zacharyrichmond/Documents/BlenderDocuments/blenderfiles/blendFiles/builder/concat_list.txt -c copy /home/zacharyrichmond/Documents/BlenderDocuments/blenderfiles/blendFiles/builder/output.mp4

  
"""

import array
import glob
import math
import os
import re
import resource
import sys

import bmesh
import bpy
import mathutils


def _rss_mb():
    """Current process peak resident set size, in MB (Linux: ru_maxrss is KB)."""
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024


def parse_args():
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1:]
    else:
        argv = []
    if len(argv) < 2:
        raise SystemExit(
            "usage: blender --background --python blend_from_chrono_export.py "
            "-- <export_dir>[,<export_dir2>,...] <output.blend> [fps] "
            "[<append_blend> <collection_name>]... "
            "[--world <append_blend> <world_name>] "
            "[--texture <folder> <family_name>[,<family_name>...]]... "
            "[--robot-texture <folder> <comma_separated_exclude_families_or_''>]")
    export_dirs = argv[0].split(",")
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
    return export_dirs, out_path, fps, appends, world_append, textures, robot_texture


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


def scale_uv_coords(obj, scale_x, scale_y):
    """Scale every UV coordinate of `obj`'s active UV layer by (scale_x,
    scale_y) around the UV space's center (0.5, 0.5) -- adjusts how large
    the texture reads across the mesh (tiling density) without needing
    Edit Mode or any bpy.ops call, so it's safe to run on an object
    regardless of its current hide/selection state."""
    uv_layer = obj.data.uv_layers.active
    if uv_layer is None:
        return
    for loop in uv_layer.data:
        u, v = loop.uv
        loop.uv.x = 0.5 + (u - 0.5) * scale_x
        loop.uv.y = 0.5 + (v - 0.5) * scale_y



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


# Name patterns for the stand-in rocks that are spawned fixed and never move
# again once placed: `wall_<stage>_<course>_<i>` for a fresh dump in this
# export, and `s<section>_...` for a previous section's rocks re-imported as
# static scenery (see spawn_wall_rocks / spawn_scenery_rocks in
# TrackedVeh_OrbitBuilder.py). Grouped by stage/section since everything in
# one group appears (and stops appearing) at the same frame -- merging across
# groups would blur the wall's real two-stage growth into one reveal.
_WALL_STAGE_RE = re.compile(r"^wall_(\d+)_")
_SCENERY_SECTION_RE = re.compile(r"^s(\d+)_")


def merge_group_key(name):
    """Which static-rock merge group `name` belongs to, or None if it's not
    one of these (e.g. the robot, or a rock_N the gripper actually carries --
    those still need independent per-object animation, not a static merge).

    Doesn't disambiguate by export dir -- callers merging multiple dirs must
    combine this with `dir_scope` (see there for why). Once merged, a
    'wall_stage_*' pile also gets unwrapped, UV-scaled, and (if a 'rock_pbr'
    material was built via --texture) textured the same as the ground rocks
    -- see `_bake_merged_group` and the rock_material block in main()."""
    m = _WALL_STAGE_RE.match(name)
    if m:
        return f"wall_stage_{m.group(1)}"
    m = _SCENERY_SECTION_RE.match(name)
    if m:
        return f"scenery_section_{m.group(1)}"
    return None


# Chrono body names that a fresh scenario run reconstructs from scratch every
# time, using purely local counters with no run/section identifier baked in
# -- so the SAME name means a completely different physical rock in each
# export dir being merged (confirmed empirically: section 1 and section 2 of
# a real two-section run both produce bodies literally named 'rock_0' and
# 'wall_1_1_0'). Treating same-named bodies across dirs as one continuing
# object -- which is exactly right for the M113/arm, whose body names ARE
# stable across a whole multi-section run -- would silently drop one dir's
# real position data for these, since only the first-seen sample is kept.
_DIR_SCOPED_NAME_RE = re.compile(r"^(rock_\d+$|wall_\d+_|s\d+_)")


def dir_scope(name, dir_index):
    """`dir_index` if `name` needs per-dir identity, else None (shared/
    continuous identity across every merged dir, e.g. the M113 chassis)."""
    return dir_index if _DIR_SCOPED_NAME_RE.match(name) else None


def add_popin_visibility(obj, first_frame, global_first_frame):
    """Keyframe `obj` invisible up to (not including) `first_frame`, visible
    from there on -- with hard CONSTANT steps, not a fade. Without this,
    every object is visible for the whole timeline the moment it has any
    keyframe at all, so a wall dumped 80s into the sim would appear already
    built at frame 0. Objects present from the very start of the combined
    timeline need no popin at all."""
    if first_frame <= global_first_frame:
        return
    obj.hide_viewport = True
    obj.hide_render = True
    obj.keyframe_insert(data_path="hide_viewport", frame=first_frame - 1)
    obj.keyframe_insert(data_path="hide_render", frame=first_frame - 1)
    obj.hide_viewport = False
    obj.hide_render = False
    obj.keyframe_insert(data_path="hide_viewport", frame=first_frame)
    obj.keyframe_insert(data_path="hide_render", frame=first_frame)
    channelbag = obj.animation_data.action.layers[0].strips[0].channelbags[0]
    for data_path in ("hide_viewport", "hide_render"):
        fc = channelbag.fcurves.find(data_path)
        for kp in fc.keyframe_points:
            kp.interpolation = 'CONSTANT'


class SceneBuilder:
    """Replays `make_chrono_object_assetlist` calls from every frame's state
    file into persistent objects -- built once per body/asset the first time
    it's seen -- rather than the add-on's destroy-and-rebuild-per-scrub
    approach.

    Keyframes are NOT inserted per frame during this pass (`obj.keyframe_insert`
    called thousands of times per object is the dominant cost once a run has
    thousands of frames -- it was observed stalling partway through a ~4000
    frame export). Instead every frame's transform is buffered and written to
    real F-Curves in one bulk pass by `bake_keyframes()`, called once after all
    state files are read.

    Buffers are flat `array('d')`s (raw packed C doubles) rather than lists of
    Python tuples: a two-section merge (~730 objects x ~8100 combined frames)
    buffered as boxed tuples/floats grew to ~24GB resident and got the whole
    process OOM-killed. Packed doubles use a small fraction of that per value,
    and each object's buffers are freed the moment that object is baked
    (`bake_keyframes` deletes them from these dicts as it goes) instead of
    holding everything in memory until the very end.
    """

    def __init__(self, assets_collection, instances_collection):
        self.assets_collection = assets_collection
        self.instances_collection = instances_collection
        self.children = {}  # (body name, asset slot index) -> instanced object
        self.frames = {}    # same key -> array('d') of frame numbers
        self.locs = {}      # same key -> array('d') of x,y,z per frame (flattened)
        self.rots = {}      # same key -> array('d') of w,x,y,z per frame (flattened)
        self.scales = {}    # same key -> array('d') of x,y,z per frame (flattened);
        #                     absent entirely for a key that's never scaled
        self.frame = 0
        self.dir_index = 0  # which export dir is currently being read (see dir_scope)

    def make_chrono_object_assetlist(self, mname, mpos, mrot, masset_list):
        # No per-body Empty: bake each shape's transform directly in world
        # space by composing it with the body's transform ourselves, instead
        # of parenting to an Empty like the real add-on does. That keeps
        # chrono_instances made up of nothing but real mesh objects, so
        # box-selecting and batch-assigning materials in the viewport isn't
        # cluttered by hundreds of axis-only parent objects.
        parent_pos = mathutils.Vector(mpos)
        parent_rot = mathutils.Quaternion(mrot)
        scope = dir_scope(mname, self.dir_index)

        for i, entry in enumerate(masset_list):
            asset_name, rel_pos, rel_rot = entry[0], entry[1], entry[2]
            mat_names = entry[3] if len(entry) > 3 else []
            scale = entry[4] if len(entry) > 4 else None
            key = (scope, mname, i)
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
                self.frames[key] = array.array('d')
                self.locs[key] = array.array('d')
                self.rots[key] = array.array('d')

            world_rot = parent_rot @ mathutils.Quaternion(rel_rot)
            world_pos = parent_pos + parent_rot @ mathutils.Vector(rel_pos)
            locs = self.locs[key]
            rots = self.rots[key]
            # ChBlender's exporter re-lists every body in every frame's state
            # file, even ones that are SetFixed and never move again (the
            # thousands of stand-in wall rocks) -- recording a fresh sample
            # for those on every one of the ~2000+ frames after they're
            # spawned is what blew memory up enough to get the whole process
            # OOM-killed. Holding between two identical keyframes is exactly
            # the same animation as one, so skip the append when nothing
            # actually changed since the last recorded sample -- correct for
            # any object (a parked vehicle or a settled rock benefits the
            # same way), not just the merge-eligible static rocks.
            unchanged = (locs and tuple(locs[-3:]) == tuple(world_pos)
                        and tuple(rots[-4:]) == tuple(world_rot))
            if not unchanged:
                self.frames[key].append(float(self.frame))
                locs.extend(world_pos)
                rots.extend(world_rot)
                if scale is not None:
                    self.scales.setdefault(key, array.array('d')).extend(scale)

    def bake_keyframes(self):
        """Write every object's buffered samples into real F-Curves in bulk,
        and merge the static wall/scenery rock groups (see `merge_group_key`)
        into one mesh per group instead of leaving thousands of individual
        objects -- they never move once placed, so nothing but object-count
        overhead is lost by joining them.

        Per-object keyframing bootstraps each F-Curve with one ordinary
        `keyframe_insert()` (which auto-creates whatever action/layer/strip/
        channelbag hierarchy this Blender version needs -- reimplementing
        that by hand would be version-fragile), then overwrites all of that
        curve's points in a single `foreach_set()` call, which is ~300x
        faster than calling `keyframe_insert()` once per frame (measured on a
        3000-frame benchmark) since it avoids the growing per-call
        insert/sort cost.
        """
        global_first_frame = min(f[0] for f in self.frames.values())

        merge_groups = {}     # merge key -> [key, ...]
        individual_keys = []
        for key in self.children:
            scope, mname, _i = key
            base_gkey = merge_group_key(mname)
            # Dir-scoped even for a merge key: two dirs' own 'wall_stage_1'
            # dump are two different rock piles that happen to share a name
            # (see dir_scope), so they must never end up in the same group.
            gkey = f"{base_gkey}_dir{scope}" if base_gkey is not None else None
            if gkey is not None:
                merge_groups.setdefault(gkey, []).append(key)
            else:
                individual_keys.append(key)

        total = len(individual_keys) + len(merge_groups)
        n = 0
        for key in individual_keys:
            self._bake_one(key, global_first_frame)
            n += 1
            if n % 100 == 0:
                print(f"  baked keyframes for {n}/{total} objects ... rss={_rss_mb():.0f}MB")

        for gkey, keys in merge_groups.items():
            self._bake_merged_group(gkey, keys, global_first_frame)
            n += 1
            if n % 100 == 0:
                print(f"  baked keyframes for {n}/{total} objects ... rss={_rss_mb():.0f}MB")

    def _bake_one(self, key, global_first_frame):
        obj = self.children[key]
        frames = self.frames.pop(key)
        locs = self.locs.pop(key)
        rots = self.rots.pop(key)
        scales = self.scales.pop(key, None)
        num_frames = len(frames)

        obj.location = locs[0:3]
        obj.rotation_quaternion = rots[0:4]
        obj.keyframe_insert(data_path="location", frame=frames[0])
        obj.keyframe_insert(data_path="rotation_quaternion", frame=frames[0])
        if scales is not None:
            obj.scale = scales[0:3]
            obj.keyframe_insert(data_path="scale", frame=frames[0])

        channelbag = obj.animation_data.action.layers[0].strips[0].channelbags[0]

        def bulk_fill(data_path, num_channels, flat_values):
            for idx in range(num_channels):
                fc = channelbag.fcurves.find(data_path, index=idx)
                fc.keyframe_points.add(num_frames - 1)
                co = array.array('d', (0.0,)) * (2 * num_frames)
                co[0::2] = frames
                co[1::2] = flat_values[idx::num_channels]
                fc.keyframe_points.foreach_set('co', co)
                fc.update()

        bulk_fill("location", 3, locs)
        bulk_fill("rotation_quaternion", 4, rots)
        if scales is not None:
            bulk_fill("scale", 3, scales)

        add_popin_visibility(obj, int(frames[0]), global_first_frame)

    def _bake_merged_group(self, gkey, keys, global_first_frame):
        """Set each member's static (never-animated) transform, then merge
        them all into one mesh object named after the group (e.g. a
        'wall_stage_1_dir0' rock pile). Also unwraps and UV-scales that
        merged object here, before it can be hidden by a popin keyframe --
        main() textures it with 'rock_pbr' afterward, once that material
        exists (see the rock_material block there).

        Builds the merged mesh manually with bmesh instead of
        `bpy.ops.object.join()` -- that operator is built for joining a
        handful of objects in an ordinary modeling workflow, and was
        observed ballooning to 30GB+ resident and getting the whole process
        OOM-killed once a group's member count reached the thousands (which
        a single wall dump's stand-in rocks routinely do; 944 members alone
        triggered it). The manual version merges 2000 test objects in ~1s
        with flat, small memory use.
        """
        group_first_frame = None
        objs = []
        for key in keys:
            obj = self.children[key]
            frames = self.frames.pop(key)
            locs = self.locs.pop(key)
            rots = self.rots.pop(key)
            self.scales.pop(key, None)  # static rocks are never scaled per-frame
            obj.rotation_mode = 'QUATERNION'
            obj.location = locs[0:3]
            obj.rotation_quaternion = rots[0:4]
            first = int(frames[0])
            group_first_frame = first if group_first_frame is None else min(group_first_frame, first)
            objs.append(obj)

        bm = bmesh.new()
        for obj in objs:
            # Read the transform straight off location/rotation/scale rather
            # than matrix_world -- matrix_world doesn't reflect a
            # just-assigned location without an intervening depsgraph
            # update, and silently reads as identity otherwise (confirmed by
            # a failing test before adding this comment).
            mat = mathutils.Matrix.LocRotScale(obj.location, obj.rotation_quaternion, obj.scale)
            temp = obj.data.copy()
            temp.transform(mat)
            bm.from_mesh(temp)
            bpy.data.meshes.remove(temp)
        merged_mesh = bpy.data.meshes.new(gkey)
        bm.to_mesh(merged_mesh)
        bm.free()

        merged = objs[0]
        for obj in objs[1:]:
            bpy.data.objects.remove(obj, do_unlink=True)
        merged.data = merged_mesh
        # The transform is now baked into the mesh's vertices -- reset the
        # object's own transform to identity so it isn't applied twice.
        merged.location = (0.0, 0.0, 0.0)
        merged.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
        merged.name = gkey

        # Unwrap while `merged` is still visible/unkeyframed -- exactly one
        # object here, not thousands, so the per-call bpy.ops overhead
        # smart_uv_unwrap normally batches against doesn't matter. Applies to
        # every merge group alike -- a fresh wall dump ('wall_stage_*') or a
        # previous section's rocks re-imported as scenery
        # ('scenery_section_*') -- from either section, since this runs once
        # per group regardless of which one it is.
        #
        # A fixed target scale (e.g. 0.691x/0.601y) doesn't behave
        # predictably here: smart_project()'s raw output isn't reliably
        # normalized to the 0-1 UV square -- it was ~1.0 for a small test
        # mesh but ~8.8 for the true full-scale scenery merge (thousands of
        # members), so the same "target" number landed very differently on
        # different-sized piles. A relative 6x multiplier on whatever
        # smart_project already produced is well-defined regardless of that.
        smart_uv_unwrap([merged])
        scale_uv_coords(merged, 6.0, 6.0)

        add_popin_visibility(merged, group_first_frame, global_first_frame)


def main():
    export_dirs, out_path, fps, appends, world_append, textures, robot_texture = parse_args()

    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.context.scene.render.fps = fps
    bpy.context.scene.render.engine = 'BLENDER_EEVEE'
    bpy.context.scene.eevee.use_raytracing = True
    bpy.context.scene.eevee.shadow_step_count = 10
    # If this file is ever rendered with Cycles instead of EEVEE, use GPU
    # compute rather than Cycles' CPU default. This is the one Cycles device
    # setting that's per-file (saved with the scene) -- which compute
    # backend (OptiX/CUDA/...) and which physical devices are enabled is a
    # global Blender preference, not something a script can usefully set
    # here since it isn't saved into the .blend at all.
    bpy.context.scene.cycles.device = 'GPU'
    # 2.5x render resolution. A percentage multiplier rather than hardcoded
    # pixel values, so it stays correct whatever base resolution_x/y the
    # Chrono export's own assets.py sets (currently 1024x768 -> 2560x1920).
    bpy.context.scene.render.resolution_percentage = 250

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

    builder = SceneBuilder(assets_collection, instances_collection)
    first_frame, last_frame, total_state_files, frame_offset = None, None, 0, 0
    for dir_index, export_dir in enumerate(export_dirs):
        builder.dir_index = dir_index
        assets_files = glob.glob(os.path.join(export_dir, "*.assets.py"))
        if not assets_files:
            raise SystemExit(f"no *.assets.py found in {export_dir}")
        assets_file = assets_files[0]

        state_files = sorted(glob.glob(os.path.join(export_dir, "output", "state*.py")),
                             key=frame_index)
        if not state_files:
            raise SystemExit(f"no output/state*.py files found in {export_dir}/output")

        print(f"building static assets from {assets_file} ...")
        with open(assets_file, "rb") as f:
            exec(compile(f.read(), assets_file, "exec"), dict(base_ns))

        for n, path in enumerate(state_files):
            builder.frame = frame_offset + frame_index(path)
            first_frame = builder.frame if first_frame is None else first_frame
            last_frame = builder.frame
            frame_ns = dict(base_ns)
            frame_ns["make_chrono_object_assetlist"] = builder.make_chrono_object_assetlist
            with open(path, "rb") as f:
                exec(compile(f.read(), path, "exec"), frame_ns)
            if n % 100 == 0:
                print(f"  {export_dir}: frame {builder.frame} ({n + 1}/{len(state_files)}) ... rss={_rss_mb():.0f}MB")

        # Next dir's frames (if any) pick up right where this one left off, so
        # multiple export dirs concatenate into one continuous timeline instead
        # of each restarting at frame 0 and overlapping the others.
        frame_offset = last_frame + 1
        total_state_files += len(state_files)

    for obj in list(assets_collection.objects):
        obj.hide_set(True)
        obj.hide_render = True

    # Unwrap before baking: bake_keyframes() keyframes hide_viewport/hide_render
    # for objects that first appear mid-timeline (see add_popin_visibility), and
    # bpy.ops.object.mode_set(EDIT) refuses to touch a currently-hidden object --
    # whatever the scene's current frame happens to be when this runs would leave
    # some of these hidden and break the unwrap. Every object already exists at
    # this point (linked in during the per-frame reading loop), and UV data lives
    # on the mesh, unaffected by animation/visibility either way.
    unwrap_targets = [o for o in all_objects_in_collection(instances_collection)
                     if is_robot_or_rock(o.name)]
    smart_uv_unwrap(unwrap_targets)

    builder.bake_keyframes()

    group_into_family_collections(instances_collection)

    for folder, family_names_csv in textures:
        family_names = [f.strip() for f in family_names_csv.split(",") if f.strip()]
        material = build_pbr_material_from_folder(f"{'_'.join(family_names)}_pbr", folder)
        for family_name in family_names:
            apply_material_to_family(instances_collection, family_name, material)

    # The merged wall/scenery rock piles (see merge_group_key) aren't part of
    # the "rock" family group.get_family_objects can find by name, since
    # group_into_family_collections may have folded 'wall_stage_1_dir0' and
    # 'wall_stage_2_dir0' together under one 'wall_stage_dir0' sub-collection
    # (their names differ only by the stage number, which family_key strips).
    # Give them the same rock_pbr material as the ground rocks, if it exists
    # -- each merged pile was already unwrapped and UV-scaled by
    # 6x back in _bake_merged_group, so this is just the material
    # assignment; nothing here needs to touch its UVs. Covers both a fresh
    # wall dump ('wall_stage_*') and a previous section's rocks re-imported
    # as static scenery ('scenery_section_*', see merge_group_key) --
    # missing the latter left section 2's carried-over section-1 rocks
    # textureless even though its own fresh dumps were textured correctly.
    rock_material = bpy.data.materials.get("rock_pbr")
    if rock_material is not None:
        wall_pile_objs = [o for o in all_objects_in_collection(instances_collection)
                         if o.name.startswith(("wall_stage_", "scenery_section_"))]
        for obj in wall_pile_objs:
            obj.data.materials.clear()
            obj.data.materials.append(rock_material)
        print(f"  applied material 'rock_pbr' to {len(wall_pile_objs)} wall-pile/scenery-rock object(s)")

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

    # The USA flag (a mesh with a Shrinkwrap modifier, appended as part of
    # Collection 6) needs that modifier's target repointed at THIS run's own
    # 'Chassis body' -- not whatever chassis happened to exist when the flag
    # was last saved -- so it's done here rather than in that source file.
    # It also needs to be parented to the chassis: the Shrinkwrap modifier's
    # normal-projection ray only reaches the hull if the flag is already
    # nearby, and with no parent the flag's own base position never moves,
    # so once the chassis drives away the ray has nothing left to hit. The
    # local offset below is fixed rather than derived from the chassis's
    # current pose -- verified to sit inside the chassis mesh's own local
    # footprint (x: -4.9..0.5, y: -1.34..1.34), which is all the Shrinkwrap
    # modifier needs to find the hull and do the exact surface placement.
    flag = next((o for o in bpy.data.objects if "usa-flag" in o.name.lower()), None)
    chassis = bpy.data.objects.get("Chassis body")
    if flag is not None and chassis is not None:
        shrinkwrap = next((m for m in flag.modifiers if m.type == 'SHRINKWRAP'), None)
        if shrinkwrap is not None:
            shrinkwrap.target = chassis
        flag.parent = chassis
        flag.matrix_parent_inverse = mathutils.Matrix.Identity(4)
        flag.rotation_euler.z = math.radians(-90)
        flag.location = mathutils.Vector((-3.5337, 0.0304, 0.6827))
        print(f"  parented '{flag.name}' to 'Chassis body' and set its Shrinkwrap target")

    bpy.context.scene.frame_start = first_frame
    bpy.context.scene.frame_end = last_frame
    bpy.context.scene.frame_set(first_frame)

    out_path = os.path.abspath(out_path)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=out_path)
    print(f"saved {out_path}  ({total_state_files} frames across {len(export_dirs)} export dir(s), "
         f"{len(builder.children)} shapes)")


if __name__ == "__main__":
    main()
