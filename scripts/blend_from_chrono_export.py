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
    #   --world ~/Documents/BlenderDocuments/blenderfiles/blendFiles/builder5.blend "World"
"""

import glob
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
            "[--world <append_blend> <world_name>]")
    export_dir = argv[0]
    out_path = argv[1]
    fps = int(argv[2]) if len(argv) > 2 else 30
    rest = argv[3:]
    appends = []
    world_append = None
    i = 0
    while i < len(rest):
        if rest[i] == "--world":
            world_append = (rest[i + 1], rest[i + 2])
            i += 3
        else:
            appends.append((rest[i], rest[i + 1]))
            i += 2
    return export_dir, out_path, fps, appends, world_append


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
    export_dir, out_path, fps, appends, world_append = parse_args()

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
