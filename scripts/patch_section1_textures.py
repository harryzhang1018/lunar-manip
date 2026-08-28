"""Patch data/section1.blend in place with the texture/lighting pass we
worked out interactively:

  1. landscape (world.terrain) -> the same terrain2_full.png + moon2-detail
     material already used for the ground in section2_mode2.blend, appended
     directly from that file (not rebuilt) so it's pixel-identical.
  2. every "trackshoe"-named object (M113 treads) AND every "spindle"-named
     object (Polaris collector wheel/hub/tire assemblies) -> the same
     plastic material (Poliigon_PlasticMoldDryBlast_7495) used for the M113
     treads in section2_mode2.blend.
  3. every other mesh whose current material carries no real texture or
     distinguishing color (the achromatic col_* placeholders, or the
     generic procedural 'rock' fallback some parts like sprockets got by
     default) -> the same metal material (MetalGalvanizedSteelWorn001 /
     'robot_pbr') used in section2_mode2.blend.
  4. anything with "rock" in its name (but not "sprocket") -> the same
     rock_pbr material (Rock020_4K-JPG), after a smart-UV-unwrap since
     these are UV-less procedural rock primitives, not real OBJ imports.
  5. delete section1's own Light object and replace it with section2_mode2's
     Light object (same data-block, re-linked).
  6. replace section1's World with section2_mode2's World.

All appended materials reference their images by the same //textures/...
relative path section2_mode2.blend already uses, and both files live in
data/ alongside data/textures/ -- so nothing here depends on an external
asset folder (Poliigon library, Downloads) on any particular machine.

Safe to re-run: every append step first checks whether the target
datablock already exists (by name) and reuses it instead of appending a
duplicate.

    blender --background data/section1.blend --python scripts/patch_section1_textures.py
"""
import math
import os

import bpy

SECTION2_MODE2 = os.path.join(os.path.dirname(__file__), "..", "data", "section2_mode2.blend")
TEXTURES_DIR = os.path.join(os.path.dirname(__file__), "..", "data", "textures")

TERRAIN_MAT_NAME = "terrain2_full.png_detail.001"
PLASTIC_MAT_NAME = "M113_TrackShoeLeft_shoe_M113_TrackShoeRight_shoe_pbr"
METAL_MAT_NAME = "robot_pbr"
ROCK_MAT_NAME = "rock_pbr"
LIGHT_OBJ_NAME = "Light.001"
WORLD_NAME = "World"


def log(msg):
    print(f"[patch] {msg}", flush=True)


def append_materials():
    wanted = {TERRAIN_MAT_NAME, PLASTIC_MAT_NAME, METAL_MAT_NAME, ROCK_MAT_NAME}
    already_have = {n for n in wanted if bpy.data.materials.get(n) is not None}
    to_fetch = wanted - already_have
    if already_have:
        log(f"materials already present, reusing: {sorted(already_have)}")
    if to_fetch:
        with bpy.data.libraries.load(SECTION2_MODE2, link=False) as (data_from, data_to):
            data_to.materials = [m for m in data_from.materials if m in to_fetch]
        log(f"appended materials from section2_mode2.blend: {[m.name for m in data_to.materials]}")

    materials = {n: bpy.data.materials[n] for n in wanted}

    # Verify every image these materials use resolves into data/textures
    # BEFORE touching anything else -- abort without saving if not, rather
    # than baking in a broken/external texture link.
    bad = []
    for mat in materials.values():
        for n in mat.node_tree.nodes:
            if n.type == 'TEX_IMAGE' and n.image:
                fp = n.image.filepath
                if not fp.startswith("//textures/"):
                    bad.append((mat.name, n.image.name, fp))
                    continue
                abs_path = os.path.join(TEXTURES_DIR, fp[len("//textures/"):])
                if not os.path.isfile(abs_path):
                    bad.append((mat.name, n.image.name, f"missing file: {abs_path}"))
    if bad:
        for m, img, reason in bad:
            log(f"BAD TEXTURE: material={m!r} image={img!r} -> {reason}")
        raise SystemExit(f"{len(bad)} texture path problem(s) -- aborting without saving")
    log("verified all appended material images resolve into data/textures/")
    return materials


def patch_terrain(terrain_mat):
    terrain_obj = bpy.data.objects.get("world.terrain")
    if terrain_obj is None:
        raise SystemExit("world.terrain not found")
    mesh = terrain_obj.data
    if not mesh.uv_layers:
        uv_layer = mesh.uv_layers.new(name="UVMap")
        xs = [v.co.x for v in mesh.vertices]
        ys = [v.co.y for v in mesh.vertices]
        minx, maxx = min(xs), max(xs)
        miny, maxy = min(ys), max(ys)
        for loop in mesh.loops:
            co = mesh.vertices[loop.vertex_index].co
            u = (co.x - minx) / (maxx - minx) if maxx > minx else 0.0
            v = (co.y - miny) / (maxy - miny) if maxy > miny else 0.0
            uv_layer.data[loop.index].uv = (u, v)
        log(f"added planar UV to world.terrain ({len(mesh.vertices)} verts)")
    terrain_obj.data.materials.clear()
    terrain_obj.data.materials.append(terrain_mat)
    log("assigned terrain material to world.terrain")


def patch_plastic(plastic_mat):
    targets = [o for o in bpy.data.objects if o.type == 'MESH'
               and ('trackshoe' in o.name.lower() or 'spindle' in o.name.lower())]
    for o in targets:
        o.data.materials.clear()
        o.data.materials.append(plastic_mat)
    log(f"assigned plastic material to {len(targets)} track-shoe/collector-wheel object(s)")
    return targets


def patch_rocks(rock_mat):
    rock_targets = [o for o in bpy.data.objects if o.type == 'MESH'
                    and 'rock' in o.name.lower() and 'sprocket' not in o.name.lower()]

    view_layer = bpy.context.view_layer
    seen_meshes = set()
    restore_hide = []
    for o in rock_targets:
        if o.data.uv_layers or o.data.name in seen_meshes:
            continue
        seen_meshes.add(o.data.name)
        was_hidden = o.hide_get()
        if was_hidden:
            o.hide_set(False)
            restore_hide.append(o)
        for ob in view_layer.objects:
            ob.select_set(False)
        o.select_set(True)
        view_layer.objects.active = o
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.uv.smart_project(angle_limit=math.radians(66.0))
        bpy.ops.object.mode_set(mode='OBJECT')
    for o in restore_hide:
        o.hide_set(True)
    log(f"smart-UV-unwrapped {len(seen_meshes)} unique rock mesh datablock(s)")

    for o in rock_targets:
        o.data.materials.clear()
        o.data.materials.append(rock_mat)
    log(f"assigned rock material to {len(rock_targets)} rock object(s)")
    return rock_targets


def is_achromatic(mat):
    if mat is None or mat.name == "rock":
        return True
    if not mat.use_nodes:
        c = mat.diffuse_color
    else:
        bsdf = next((n for n in mat.node_tree.nodes if n.type == 'BSDF_PRINCIPLED'), None)
        c = bsdf.inputs["Base Color"].default_value if bsdf else (0.8, 0.8, 0.8, 1.0)
    return (max(c[0], c[1], c[2]) - min(c[0], c[1], c[2])) <= 0.05


def patch_metal(metal_mat, handled_names):
    metal_targets = []
    for o in bpy.data.objects:
        if o.type != 'MESH' or o.name in handled_names:
            continue
        mat = o.material_slots[0].material if o.material_slots else None
        if is_achromatic(mat):
            metal_targets.append(o)
    for o in metal_targets:
        o.data.materials.clear()
        o.data.materials.append(metal_mat)
    log(f"assigned metal material to {len(metal_targets)} object(s) with no real texture/color")


def patch_light_and_world():
    for obj in [o for o in bpy.data.objects if o.type == 'LIGHT']:
        data = obj.data
        bpy.data.objects.remove(obj, do_unlink=True)
        if data.users == 0:
            bpy.data.lights.remove(data)
    log("removed section1's existing light object(s)")

    with bpy.data.libraries.load(SECTION2_MODE2, link=False) as (data_from, data_to):
        data_to.objects = [o for o in data_from.objects if o == LIGHT_OBJ_NAME]
    new_light = data_to.objects[0]
    bpy.context.scene.collection.objects.link(new_light)
    log(f"appended and linked light {new_light.name!r} from section2_mode2.blend")

    old_world = bpy.context.scene.world
    if old_world is not None and old_world.name == WORLD_NAME:
        log(f"scene world is already {WORLD_NAME!r}, leaving it alone")
        return
    with bpy.data.libraries.load(SECTION2_MODE2, link=False) as (data_from, data_to):
        data_to.worlds = [w for w in data_from.worlds if w == WORLD_NAME]
    bpy.context.scene.world = data_to.worlds[0] if data_to.worlds else bpy.data.worlds[WORLD_NAME]
    if old_world is not None and old_world.users == 0:
        bpy.data.worlds.remove(old_world)
    log(f"replaced scene world with {bpy.context.scene.world.name!r} from section2_mode2.blend")


def main():
    materials = append_materials()
    patch_terrain(materials[TERRAIN_MAT_NAME])
    plastic_targets = patch_plastic(materials[PLASTIC_MAT_NAME])
    rock_targets = patch_rocks(materials[ROCK_MAT_NAME])
    handled = {o.name for o in plastic_targets} | {o.name for o in rock_targets} | {"world.terrain"}
    patch_metal(materials[METAL_MAT_NAME], handled)
    patch_light_and_world()

    bpy.ops.wm.save_mainfile()
    log("saved section1.blend in place")


if __name__ == "__main__":
    main()
