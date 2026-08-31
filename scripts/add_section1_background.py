"""add_section1_background.py -- copy a few far-away section1 vehicles into a
section2 .blend so section2 renders show the rest of the site in the
background.

section1.blend and section2_mode*.blend are the same site (same 1024 m
terrain heightmap, same 15-slot ring of builder/collector teams around the
origin), but section2 only contains its own hero rover. This script appends
selected section1 "rank" groups (a rank = one builder M113 + arm, one
collector Polaris LRV + arm + trailer) into a section2 file as static
background props, without touching the section2 file itself -- the result
is written to a new .blend next to it.

Run inside Blender, with the *section2* file open:

    blender --factory-startup -b data/section2_mode1.blend \
        --python scripts/add_section1_background.py -- \
        --out data/section2_mode1_bg.blend \
        [--section1 data/section1.blend] \
        [--groups rank_8.builder,rank_7.collector] \
        [--animate | --pose-frame N] [--rocks] [--snap-step 25] \
        [--offset 0.5 0.2 -2.9] [--tire-mesh data/vehicle/LRV/meshes/LRVtire_red_m.obj]

The normal recipe -- background vehicles that *move* during section2 -- is
to keep section1's animation in mode1 and freeze its last pose in mode2:

    blender ... -b data/section2_mode1.blend --python ... -- --out data/section2_mode1_bg.blend --animate
    blender ... -b data/section2_mode2.blend --python ... -- --out data/section2_mode2_bg.blend --pose-frame 1975

Options:
  --groups      comma-separated <rank>.<builder|collector>[@frame] groups to
                import. The default is one builder and one collector that
                stay inside both section2 cameras' frustums for the whole of
                section1's motion and as far from the hero rover as the ring
                allows: rank_8's builder (56 m from the camera, far left --
                rank_9's is already out of frame) and rank_7's collector
                (drives 60 -> 100 m across the frame). rank_4/5 sit on top of
                the hero, rank_6's builder is directly behind it, rank_7's
                builder was judged too close at 47 m. An optional @frame
                poses that one vehicle at a different section1 frame than
                --pose-frame (static mode only).
  --pose-frame  section1 frame whose pose is baked in as a static prop
                (default 1). Ignored with --animate.
  --animate     keep the section1 animation: the vehicles drive / dig for
                section1's 1975 frames (66 s at 30 fps, starting at file
                frame 1) and then hold their last pose (Blender's constant
                extrapolation). section2_mode2 continues where mode1 leaves
                off, so build it with --pose-frame 1975 instead, or the
                motion would replay from the start after the cut.
  --rocks       also import each rank's rocks (the harvest field the
                collector digs from and the builder stacks). Off by default:
                only the two vehicles and their motion are wanted, and rocks
                are a few pixels at this distance anyway.
  --tire-mesh   OBJ used for the Polaris / trailer tyres (default
                data/vehicle/LRV/meshes/LRVtire_red_m.obj, the one the
                dataset's own renders use); '' to keep the box placeholders.
  --snap-step   with --animate: sample the vehicle-vs-terrain gap every N
                section1 frames (default 25) and keyframe it away.
  --offset      section1 -> section2 world offset. section2's terrain object
                sits at (0.5, 0.2, -2.9) while section1's is at the origin,
                so section2 = section1 + (0.5, 0.2, -2.9). Imported vehicles
                are additionally ray-cast onto section2's terrain so they
                sit on the ground even though the two terrains disagree by
                up to ~0.6 m (section1 vehicles drove on a deformable SCM
                patch): a static prop is shifted once, an animated vehicle
                gets a per-vehicle `<group>.ground_offset` empty between the
                rank root and its parts whose Z is keyframed along the path.

The imported builders are re-skinned to look like section2's hero rover
(its textured `robot_pbr` / track-shoe materials and its flat chassis
plate) instead of section1's flat-green box hull; the Polaris collectors
keep their section1 grey since section2 has nothing equivalent.

section1.blend's primitive shapes need repair on the way in:
scripts/blend_from_amd_uw_export.py builds every box as
`primitive_cube_add(size=1.0)` + `scale = size/2` (the dataset's own
blender_import.py uses `scale = size`), so every box -- trailer bed,
suspension links, the Polaris tyre "cylinders" -- is drawn at exactly half
its recorded size while its centre is right. This script doubles them, and
then does for the tyres what the dataset's mesh-map does: the 0.82 m tyre
boxes become LRVtire_red_m.obj fitted to the box with the exporter's own
fit_to_aabb(). (The exporter's cylinder shapes carry no aabb and come out as
0.05 m default cubes -- those are left alone; on the M113 they're hidden
inside the running gear.)
"""

import argparse
import os
import sys

import bpy
from mathutils import Vector

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
import blend_from_amd_uw_export as amduw   # noqa: E402  (MeshCache, fit_to_aabb)

DEFAULT_GROUPS = "rank_8.builder,rank_7.collector"
DEFAULT_OFFSET = (0.5, 0.2, -2.9)
BG_COLLECTION = "section1_background"
TERRAIN_NAME = "static.world.terrain.terrain0"
LOOSE_PART_MAX_DIST = 5.0   # m; `world` parts farther than this from any imported vehicle are dropped
PRIMITIVE_FACTOR = 2.0      # section1.blend draws box primitives at size/2 -- see module docstring
DEFAULT_TIRE_MESH = os.path.join("vehicle", "LRV", "meshes", "LRVtire_red_m.obj")   # under data/

# section1 object-name prefixes that belong to each vehicle role. `world`
# holds per-rank loose parts (trailer axle, front ballast) that are
# assigned to whichever vehicle they're closest to at the pose frame.
ROLE_PREFIXES = {
    "builder": ["builder.", "builder_arm."],
    "collector": ["collector.", "collector_arm.", "collector_trailer."],
}


def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    p = argparse.ArgumentParser(prog="add_section1_background.py")
    p.add_argument("--out", required=True)
    p.add_argument("--section1", default=None)
    p.add_argument("--groups", default=DEFAULT_GROUPS)
    p.add_argument("--pose-frame", type=int, default=1)
    p.add_argument("--animate", action="store_true")
    p.add_argument("--offset", type=float, nargs=3, default=DEFAULT_OFFSET)
    p.add_argument("--rocks", action="store_true")
    p.add_argument("--snap-step", type=int, default=25)
    p.add_argument("--tire-mesh", default=None)
    return p.parse_args(argv)


def wanted_names(all_names, groups, rocks=True):
    """Object names in section1 to append for `groups` (+ each rank's root
    empty, its loose `world` parts and, optionally, its rocks)."""
    ranks = sorted({g.split(".")[0] for g in groups})
    prefixes = []
    for g in groups:
        rank, role = g.split(".")
        prefixes += [f"{rank}.{p}" for p in ROLE_PREFIXES[role]]
    prefixes += [f"{r}.world." for r in ranks]
    if rocks:
        prefixes += [f"{r}.rock." for r in ranks]
    names = [n for n in all_names if n in ranks or any(n.startswith(p) for p in prefixes)]
    return ranks, names


def append_from_section1(section1_path, groups, rocks=True):
    with bpy.data.libraries.load(section1_path, link=False) as (data_from, data_to):
        ranks, names = wanted_names(list(data_from.objects), groups, rocks)
        # Blender fills the assigned list in place with the loaded Objects,
        # so hand it a copy and keep `names` as strings
        data_to.objects = list(names)
    objs = [o for o in data_to.objects if o is not None]
    missing = set(names) - {o.name for o in objs}
    if missing:
        raise SystemExit(f"failed to append {len(missing)} objects, e.g. {sorted(missing)[:5]}")
    return ranks, objs


def part_and_shape(name):
    """'rank_7.builder.M113_RoadWheelLeft_0_wheel.shape0' ->
    ('builder', 'M113_RoadWheelLeft_0_wheel', 0); None if not a shape."""
    head, _, tail = name.rpartition(".")
    if not tail.startswith("shape"):
        return None
    rank, role, part = head.split(".", 2)
    return role, part, int(tail[len("shape"):])


def reskin_builders(objs):
    """Give imported M113 builders section2's hero-rover look."""
    s2_chassis = bpy.data.objects.get("Chassis body")
    swapped = remapped = 0
    done_meshes = set()
    for o in objs:
        if o.type != "MESH":
            continue
        info = part_and_shape(o.name)
        if info is None or info[0] not in ("builder", "builder_arm"):
            continue
        role, part, k = info
        if role == "builder_arm":
            # 'builder_7_bicep-1' -> section2 'bicep-1'
            part = part.split("_", 2)[-1]
        if role == "builder" and part == "Chassis body" and k == 0 and s2_chassis is not None:
            o.data = s2_chassis.data      # section2's flat chassis plate
            swapped += 1
            continue
        s2 = bpy.data.objects.get(f"{part}_{k}") or (bpy.data.objects.get(part) if k == 0 else None)
        if s2 is None or s2.type != "MESH" or not s2.data.materials:
            continue
        if o.data.name in done_meshes:
            continue
        done_meshes.add(o.data.name)
        o.data.materials.clear()
        for m in s2.data.materials:
            o.data.materials.append(m)
        remapped += 1
    print(f"reskin: {swapped} chassis mesh swap(s), {remapped} mesh material remap(s)")


def fix_section1_shapes(objs, coll, tire_path):
    """Undo the exporter's half-size box primitives and put real tyres on
    the Polaris and its trailer (see module docstring)."""
    tire_tpl = None
    if tire_path:
        if os.path.exists(tire_path):
            tire_tpl = amduw.MeshCache().get(tire_path)
        if tire_tpl is None:
            print(f"  WARNING: tyre mesh {tire_path} not loadable, keeping box placeholders")
    doubled = tyres = 0
    bpy.context.view_layer.update()
    for o in list(objs):
        if o.type != "MESH" or len(o.data.vertices) != 8 or not o.data.name.startswith("Cube"):
            continue
        o.scale = o.scale * PRIMITIVE_FACTOR
        doubled += 1
        info = part_and_shape(o.name)
        if tire_tpl is None or info is None:
            continue
        role, part, _k = info
        # size from the mesh itself: freshly appended objects have no
        # evaluated bound_box yet, so o.dimensions would read (0, 0, 0)
        vs = [v.co for v in o.data.vertices]
        extent = Vector([max(v[i] for v in vs) - min(v[i] for v in vs) for i in range(3)])
        size = Vector([extent[i] * o.scale[i] for i in range(3)])
        is_tyre = (role in ("collector", "collector_trailer") and "spindle" in part
                   and size.x > 0.5 and abs(size.x - size.z) < 0.05 and size.y < size.x)
        if not is_tyre:
            continue
        tyre = tire_tpl.copy()
        tyre.data = tire_tpl.data
        tyre.name = o.name + ".tyre"
        coll.objects.link(tyre)
        tyre.parent = o.parent
        tyre.matrix_parent_inverse = o.matrix_parent_inverse.copy()
        tyre.rotation_mode = o.rotation_mode
        tyre.rotation_euler = o.rotation_euler
        half = size / 2.0
        amduw.fit_to_aabb(tyre, o.location - half, o.location + half)
        if o.data.materials and not tire_tpl.data.materials:
            tire_tpl.data.materials.append(o.data.materials[0])   # the box's black
        objs.append(tyre)
        bpy.data.objects.remove(o)
        tyres += 1
    if tire_tpl is not None:
        bpy.data.objects.remove(tire_tpl)
    print(f"shape repair: {doubled} box primitives doubled, {tyres} tyre boxes replaced by "
          f"{os.path.basename(tire_path) if tyres else 'nothing'}")


def bake_pose(objs, frame):
    """Freeze the animated objects among `objs` at section1 frame `frame`."""
    scn = bpy.context.scene
    scn.frame_set(frame)
    animated = [o for o in objs if o.animation_data and o.animation_data.action]
    # parents first so matrix_world assignment sees settled parents
    animated.sort(key=lambda o: len(o.parent_recursive) if hasattr(o, "parent_recursive") else 0)
    for o in animated:
        m = o.matrix_world.copy()
        o.animation_data_clear()
        o.matrix_world = m
    bpy.context.view_layer.update()
    return len(animated)


def terrain_z(terrain, x, y):
    """Terrain height under world (x, y), or None if the ray misses."""
    inv = terrain.matrix_world.inverted()
    origin = inv @ Vector((x, y, 1000.0))
    direction = (inv.to_3x3() @ Vector((0, 0, -1))).normalized()
    hit, loc, _normal, _idx = terrain.ray_cast(origin, direction)
    return (terrain.matrix_world @ loc).z if hit else None


def vehicle_groups(objs, ranks, groups):
    """{group: [direct children of the rank root that belong to it]}, with
    each rank's loose `world` parts attached to the nearest vehicle."""
    roots = {r: next(o for o in objs if o.name == r) for r in ranks}
    out = {g: [] for g in groups}
    loose = []
    for o in objs:
        if o.parent is None or o.parent.name not in roots:
            continue
        rank, role, _rest = o.name.split(".", 2)
        if role == "world":
            loose.append(o)
            continue
        base_role = role.split("_")[0]
        g = f"{rank}.{base_role}"
        if g in out:
            out[g].append(o)
    for o in loose:
        rank = o.name.split(".")[0]
        best, best_d = None, None
        for g, parts in out.items():
            if not g.startswith(rank + ".") or not parts:
                continue
            c = sum((p.matrix_world.translation for p in parts), Vector()) / len(parts)
            d = (c - o.matrix_world.translation).length
            if best_d is None or d < best_d:
                best, best_d = g, d
        if best is not None and best_d <= LOOSE_PART_MAX_DIST:
            out[best].append(o)
            print(f"  loose part {o.name!r} -> {best} ({best_d:.1f} m)")
        else:
            # belongs to a vehicle of this rank that wasn't imported --
            # don't leave it floating on its own
            print(f"  loose part {o.name!r}: nearest imported vehicle is "
                  f"{best_d:.1f} m away, dropped")
            for c in list(o.children_recursive):
                bpy.data.objects.remove(c)
            bpy.data.objects.remove(o)
    return out


def alive(o):
    """False for a Python wrapper whose Blender object has been removed."""
    try:
        o.name
        return True
    except ReferenceError:
        return False


def mesh_world_points(o):
    return [o.matrix_world @ Vector(c) for c in o.bound_box]


def group_gap(parts, terrain):
    """How far (m) `parts` would have to move in Z, at the current frame,
    for their lowest points to touch section2's terrain: (dz, min_z, ground,
    cx, cy), or None if there's nothing to measure."""
    meshes = []
    for p in parts:
        meshes += [c for c in [p] + list(p.children_recursive) if c.type == "MESH"]
    pts = [pt for m in meshes for pt in mesh_world_points(m)]
    if not pts:
        return None
    min_z = min(pt.z for pt in pts)
    # ground-contact samples: the lowest points (track shoes / tyres)
    contact = [pt for pt in pts if pt.z < min_z + 0.15]
    tz = [terrain_z(terrain, pt.x, pt.y) for pt in contact]
    tz = [z for z in tz if z is not None]
    if not tz:
        return None
    ground = sum(tz) / len(tz)
    cx = sum(pt.x for pt in pts) / len(pts)
    cy = sum(pt.y for pt in pts) / len(pts)
    return ground - min_z, min_z, ground, cx, cy


def snap_to_ground(groups_objs, terrain):
    """Shift each static vehicle once so it sits on section2's terrain."""
    for g, parts in groups_objs.items():
        r = group_gap(parts, terrain)
        if r is None:
            print(f"  {g}: nothing to measure / terrain ray-cast missed, left as is")
            continue
        dz, min_z, ground, cx, cy = r
        status = "ok"
        if abs(dz) > 0.02:
            for p in parts:
                p.location.z += dz
            status = f"shifted by {dz:+.2f} m"
        print(f"  {g}: at ({cx:.1f}, {cy:.1f}); lowest point {min_z:.2f} vs terrain {ground:.2f} -> {status}")


def snap_to_ground_animated(groups_objs, terrain, roots, coll, frame_start, frame_end, step):
    """Keep each animated vehicle on section2's terrain along its whole
    path: insert a `<group>.ground_offset` empty between the rank root and
    the vehicle's parts and keyframe its Z to the sampled gap."""
    scn = bpy.context.scene
    empties = {}
    for g, parts in groups_objs.items():
        if not parts:
            continue
        emp = bpy.data.objects.new(f"{g}.ground_offset", None)
        emp.parent = roots[g.split(".")[0]]
        coll.objects.link(emp)
        for p in parts:
            p.parent = emp      # emp is at identity under the same root: world transform unchanged
        empties[g] = emp
    frames = list(range(frame_start, frame_end + 1, step))
    if frames[-1] != frame_end:
        frames.append(frame_end)
    samples = {g: [] for g in empties}
    for f in frames:
        scn.frame_set(f)
        for g in empties:
            r = group_gap(groups_objs[g], terrain)
            if r is not None:
                samples[g].append((f, r[0], r[3], r[4]))
    for g, emp in empties.items():
        if not samples[g]:
            print(f"  {g}: terrain ray-cast missed at every sample, no ground offset")
            continue
        for f, dz, _cx, _cy in samples[g]:
            emp.location.z = dz
            emp.keyframe_insert("location", index=2, frame=f)
        dzs = [dz for _f, dz, _cx, _cy in samples[g]]
        first, last = samples[g][0], samples[g][-1]
        print(f"  {g}: ground offset keyed at {len(samples[g])} frames "
              f"({min(dzs):+.2f}..{max(dzs):+.2f} m); path ({first[2]:.1f}, {first[3]:.1f}) -> "
              f"({last[2]:.1f}, {last[3]:.1f})")
    scn.frame_set(scn.frame_start)
    return list(empties.values())


def main():
    args = parse_args()
    blend_dir = os.path.dirname(bpy.data.filepath)
    section1 = args.section1 or os.path.join(blend_dir, "section1.blend")
    section1 = os.path.abspath(section1)
    out = os.path.abspath(args.out)
    groups, pose_frames = [], {}
    for spec in args.groups.split(","):
        spec = spec.strip()
        if not spec:
            continue
        g, _, frame = spec.partition("@")
        rank, role = g.split(".")
        if role not in ROLE_PREFIXES:
            raise SystemExit(f"bad group {g!r}: role must be one of {list(ROLE_PREFIXES)}")
        groups.append(g)
        pose_frames[g] = int(frame) if frame else args.pose_frame

    scn = bpy.context.scene
    terrain = bpy.data.objects.get(TERRAIN_NAME)
    if terrain is None:
        raise SystemExit(f"section2 file has no {TERRAIN_NAME!r} terrain object")
    if bpy.data.collections.get(BG_COLLECTION):
        raise SystemExit(f"{bpy.data.filepath} already has a {BG_COLLECTION!r} collection")

    print(f"appending {groups} from {section1}")
    ranks, objs = append_from_section1(section1, groups, rocks=args.rocks)
    print(f"appended {len(objs)} objects from ranks {ranks}")
    anim_ranges = [o.animation_data.action.frame_range for o in objs
                   if o.animation_data and o.animation_data.action]
    anim_start = int(min(r[0] for r in anim_ranges)) if anim_ranges else 1
    anim_end = int(max(r[1] for r in anim_ranges)) if anim_ranges else 1
    print(f"section1 animation covers frames {anim_start}-{anim_end}")

    coll = bpy.data.collections.new(BG_COLLECTION)
    scn.collection.children.link(coll)
    for o in objs:
        coll.objects.link(o)
        o.hide_render = False
        o.hide_viewport = False

    reskin_builders(objs)
    tire_path = args.tire_mesh
    if tire_path is None:
        tire_path = os.path.join(blend_dir, DEFAULT_TIRE_MESH)
    fix_section1_shapes(objs, coll, os.path.abspath(tire_path) if tire_path else "")
    objs = [o for o in objs if alive(o)]

    # loose-part assignment by proximity, with everything at the same frame
    scn.frame_set(args.pose_frame)
    grouped = vehicle_groups(objs, ranks, groups)
    objs = [o for o in objs if alive(o)]     # dropped loose parts are gone

    if not args.animate:
        for g, parts in grouped.items():
            n = bake_pose(parts, pose_frames[g])
            print(f"  {g}: baked pose at section1 frame {pose_frames[g]} into {n} objects")

    roots = {}
    for r in ranks:
        roots[r] = next(o for o in objs if o.name == r)
        roots[r].location = Vector(args.offset)
    bpy.context.view_layer.update()

    print("ground check:")
    if args.animate:
        snap_to_ground_animated(grouped, terrain, roots, coll, anim_start, anim_end, args.snap_step)
    else:
        snap_to_ground(grouped, terrain)
    bpy.context.view_layer.update()

    for g, parts in grouped.items():
        if parts:
            c = sum((p.matrix_world.translation for p in parts), Vector()) / len(parts)
            print(f"  {g}: {len(parts)} parts, centred at ({c.x:.1f}, {c.y:.1f}, {c.z:.1f})")

    bpy.ops.wm.save_as_mainfile(filepath=out, compress=True, relative_remap=True)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
