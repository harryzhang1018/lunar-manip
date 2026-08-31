"""make_birdview_demo.py -- build the 10 s bird's-eye "everyone builds the
rock orbit" demo scene from section2_mode2.blend.

The scene shows, from a tilted aerial camera over the whole rock orbit
(centre (0,0), r ~ 31.7 m -- low enough that the ~1 m wall height reads):

  * the hero rover exactly as baked in section2_mode2.blend over file
    frames 3555-3854 (logical 4:00-4:10 -- parked at its slot, arm placing
    rocks, its own fake-stacked wall stages appearing: stage 1 already
    visible, stage 2 pops in at frame 3589);
  * n zombie builders (and a fetcher LRV) imported from section1.blend at
    their own ring slots, their section1 motion time-shifted into the
    window with NLA strips (section1 frame --anim-start lands on scene
    frame --start-frame; the strips HOLD-extrapolate outside);
  * one copy of the hero's two wall stages per zombie builder, rotated
    about the orbit centre to that builder's slot, appearing hidden ->
    visible (the same fake-stacking mechanism the hero's own wall uses) at
    staggered frames: at t=0 only the hero's wall exists, by t=10 s every
    zombie has a wall section, the wave spreading outward from the hero.

Run inside Blender:

    blender --factory-startup -b data/section2_mode2.blend \
        --python scripts/make_birdview_demo.py -- \
        [--out data/birdview_demo.blend] \
        [--builders 1,2,3,4,6,7,8,9,10,11,12,13,14] [--collectors 7] \
        [--anim-start 400] [--start-frame 3555] [--duration 300] \
        [--cam-target 0 0 0] [--cam-azimuth -45] [--cam-elevation 30] \
        [--cam-distance 85] [--cam-lens 35]

Then render stills / the clip straight from the saved file (the scene's
frame range is set to the window, camera already active):

    blender --factory-startup -b data/birdview_demo.blend -o //builder/bird_ -F PNG -f 3555,3705,3854
    scripts/render_section.sh data/birdview_demo.blend 3555 3854 builder/bird

Slot geometry: section1's 15 teams sit every 24 degrees, rank k at
(k-1)*24 deg, and the hero occupies rank_5's slot (96 deg) -- so a wall
copy for rank k is the hero's wall rotated by (k-5)*24 deg, dropped onto
the terrain there. Vehicle import (tyre repair, re-skin, per-vehicle
ground-offset keying) is all reused from scripts/add_section1_background.py.
"""

import argparse
import math
import os
import sys

import bpy
from mathutils import Matrix, Vector

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
import add_section1_background as bg   # noqa: E402

HERO_RANK = 5                    # the hero rover sits in rank_5's ring slot
SLOT_DEG = 24.0                  # ring slots every 24 degrees
WALL_STAGES = ("wall_stage_1_dir0", "wall_stage_2_dir0")
STAGE2_LAG = 45                  # frames between a zombie's stage 1 and stage 2 pop


def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    p = argparse.ArgumentParser(prog="make_birdview_demo.py")
    p.add_argument("--out", default=None)
    p.add_argument("--section1", default=None)
    p.add_argument("--builders", default="1,2,3,4,6,7,8,9,10,11,12,13,14,15")
    p.add_argument("--collectors", default="7,3,11")
    p.add_argument("--anim-start", type=int, default=400,
                   help="section1 frame that plays at --start-frame")
    p.add_argument("--anim-span", type=int, default=1500,
                   help="how many section1 frames play during the clip (1500 into a "
                        "300-frame clip = the zombies move at 5x speed)")
    p.add_argument("--no-fill-gaps", action="store_true",
                   help="don't add the midpoint wall arcs between adjacent slots")
    p.add_argument("--start-frame", type=int, default=3555,
                   help="first mode2 file frame of the clip (3555 = logical 4:00)")
    p.add_argument("--duration", type=int, default=300, help="clip length in frames")
    p.add_argument("--cam-target", type=float, nargs=3, default=(0.0, 0.0, 0.0))
    p.add_argument("--cam-azimuth", type=float, default=-45.0,
                   help="deg; compass direction of the camera FROM the target (0 = +x)")
    p.add_argument("--cam-elevation", type=float, default=30.0, help="deg above the horizon")
    p.add_argument("--cam-distance", type=float, default=85.0, help="m from the target")
    p.add_argument("--cam-lens", type=float, default=35.0)
    p.add_argument("--offset", type=float, nargs=3, default=bg.DEFAULT_OFFSET)
    return p.parse_args(argv)


def import_zombies(args, coll, terrain):
    """section1 vehicles at their own slots, motion NLA-shifted into the
    clip window. Reuses add_section1_background's machinery end to end."""
    blend_dir = os.path.dirname(bpy.data.filepath)
    section1 = os.path.abspath(args.section1 or os.path.join(blend_dir, "section1.blend"))
    groups = ([f"rank_{r}.builder" for r in args.builder_ranks]
              + [f"rank_{r}.collector" for r in args.collector_ranks])
    print(f"appending {groups} from {section1}")
    ranks, objs = bg.append_from_section1(section1, groups, rocks=False)
    print(f"appended {len(objs)} objects from ranks {ranks}")
    for o in objs:
        coll.objects.link(o)
        o.hide_render = False
        o.hide_viewport = False
    bg.reskin_builders(objs)
    bg.fix_section1_shapes(objs, coll, os.path.join(blend_dir, bg.DEFAULT_TIRE_MESH))
    objs = [o for o in objs if bg.alive(o)]

    bpy.context.scene.frame_set(1)
    grouped = bg.vehicle_groups(objs, ranks, groups)
    objs = [o for o in objs if bg.alive(o)]

    roots = {}
    for r in ranks:
        roots[r] = next(o for o in objs if o.name == r)
        roots[r].location = Vector(args.offset)
    bpy.context.view_layer.update()

    # keyed ground offsets, sampled over the whole section1 span we play
    print("ground check:")
    a0, a1 = args.anim_start, args.anim_start + args.anim_span
    empties = bg.snap_to_ground_animated(grouped, terrain, roots, coll, a0, a1, 25)
    bpy.context.view_layer.update()

    # time-shift AND time-compress: push every imported action (the ground
    # offset empties' too) into an NLA strip so section1 frame a plays at
    # scene frame start_frame + (a - anim_start) * scale
    scale = args.duration / args.anim_span
    shifted = 0
    for o in objs + empties:
        ad = o.animation_data
        if not (ad and ad.action):
            continue
        action = ad.action
        slot = getattr(ad, "action_slot", None)
        afs = int(action.frame_range[0])
        track = ad.nla_tracks.new()
        strip = track.strips.new(o.name[:56], args.start_frame, action)
        strip.extrapolation = "HOLD"
        strip.scale = scale
        strip.frame_start = args.start_frame - (args.anim_start - afs) * scale
        ad.action = None
        if slot is not None:
            try:
                strip.action_slot = slot
            except Exception:
                pass
        shifted += 1
    print(f"NLA-shifted {shifted} actions: section1 frames {a0}-{a1} -> scene frames "
          f"{args.start_frame}-{args.start_frame + args.duration} ({1 / scale:.1f}x speed)")
    return grouped


def add_zombie_walls(args, coll, terrain):
    """Wall arcs popping in (hidden -> visible, the hero's own fake-stacking
    mechanism) around the ring: one rotated copy of the hero's wall stages
    per zombie builder, staggered outward from the hero -- and, unless
    --no-fill-gaps, an extra arc at the midpoint between each pair of
    adjacent walled slots (the slot arcs span ~16 of every 24 degrees, so
    without the midpoints the finished ring keeps ~8 degree gaps)."""
    originals = [bpy.data.objects[n] for n in WALL_STAGES]
    # terrain height under the hero's wall arc, for per-slot z correction
    mids = {}
    for o in originals:
        pts = [o.matrix_world @ Vector(c) for c in o.bound_box]
        mids[o.name] = sum(pts, Vector()) / len(pts)
    hero_mid = sum(mids.values(), Vector()) / len(mids)
    z_hero = bg.terrain_z(terrain, hero_mid.x, hero_mid.y)
    t_min = args.start_frame + 20
    t_max = args.start_frame + args.duration - 15

    def spawn_arc(theta_deg, stage1_at, label):
        rot = Matrix.Rotation(math.radians(theta_deg), 4, "Z")
        stage1_at = min(int(stage1_at), t_max - STAGE2_LAG)
        for si, src in enumerate(originals):
            copy = src.copy()                      # shares the mesh data
            copy.data = src.data
            copy.name = f"zombie_wall_{label}_stage{si + 1}"
            copy.animation_data_clear()
            coll.objects.link(copy)
            copy.matrix_world = rot @ src.matrix_world
            mid = rot @ mids[src.name]
            z_here = bg.terrain_z(terrain, mid.x, mid.y)
            if z_here is not None and z_hero is not None:
                copy.location.z += z_here - z_hero
            appear = stage1_at if si == 0 else min(stage1_at + STAGE2_LAG, t_max)
            for prop in ("hide_render", "hide_viewport"):
                setattr(copy, prop, True)
                copy.keyframe_insert(prop, frame=appear - 1)
                setattr(copy, prop, False)
                copy.keyframe_insert(prop, frame=appear)
        print(f"  wall {label}: {theta_deg + (HERO_RANK - 1) * SLOT_DEG:.0f} deg on the ring, "
              f"stage1 at frame {stage1_at} (t={(stage1_at - args.start_frame) / 30.0:.1f} s)")
        return stage1_at

    # wave spreads outward from the hero: nearest slots get walls first
    def slot_dist(rank):
        d = abs(rank - HERO_RANK) % 15
        return min(d, 15 - d)
    order = sorted(args.builder_ranks, key=slot_dist)
    n = len(order)
    t_slots_end = args.start_frame + int(args.duration * 0.70)
    appear_at = {HERO_RANK: t_min - 10}            # the hero's wall is already there
    for i, rank in enumerate(order):
        stage1 = t_min + i * (t_slots_end - t_min) / max(n - 1, 1)
        appear_at[rank] = spawn_arc((rank - HERO_RANK) * SLOT_DEG, stage1, f"rank{rank}")

    if not args.no_fill_gaps:
        for k in sorted(appear_at):
            k2 = k % 15 + 1
            if k2 not in appear_at:
                continue
            stage1 = max(appear_at[k], appear_at[k2]) + 18
            spawn_arc((k - HERO_RANK) * SLOT_DEG + SLOT_DEG / 2.0, stage1, f"mid{k}_{k2}")


def add_birdview_camera(args):
    """Tilted aerial view: the camera sits at (azimuth, elevation, distance)
    on a sphere around --cam-target and looks at it. Low elevation keeps the
    walls' ~1 m height visible; straight top-down would flatten it away."""
    scn = bpy.context.scene
    cam_data = bpy.data.cameras.new("birdview")
    cam_data.lens = args.cam_lens
    cam_data.clip_end = 5000.0
    cam = bpy.data.objects.new("birdview", cam_data)
    scn.collection.objects.link(cam)
    target = Vector(args.cam_target)
    az = math.radians(args.cam_azimuth)
    el = math.radians(args.cam_elevation)
    pos = target + args.cam_distance * Vector(
        (math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)))
    cam.location = pos
    cam.rotation_euler = (target - pos).to_track_quat("-Z", "Y").to_euler()
    scn.camera = cam
    print(f"birdview camera at ({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f}), looking at "
          f"({target.x:.0f}, {target.y:.0f}, {target.z:.0f}), lens {args.cam_lens} mm "
          f"(azimuth {args.cam_azimuth:.0f} deg, elevation {args.cam_elevation:.0f} deg, "
          f"distance {args.cam_distance:.0f} m)")


def main():
    args = parse_args()
    args.builder_ranks = [int(r) for r in args.builders.split(",") if r.strip()]
    args.collector_ranks = [int(r) for r in args.collectors.split(",") if r.strip()]
    if HERO_RANK in args.builder_ranks:
        raise SystemExit(f"rank_{HERO_RANK} is the hero's slot -- pick other builders")
    out = os.path.abspath(args.out or os.path.join(
        os.path.dirname(bpy.data.filepath), "birdview_demo.blend"))

    scn = bpy.context.scene
    terrain = bpy.data.objects.get(bg.TERRAIN_NAME)
    if terrain is None:
        raise SystemExit(f"no {bg.TERRAIN_NAME!r} in this file -- open section2_mode2.blend")
    if bpy.data.collections.get(bg.BG_COLLECTION):
        raise SystemExit("this file already has imported background vehicles -- "
                         "start from the plain section2_mode2.blend")
    coll = bpy.data.collections.new(bg.BG_COLLECTION)
    scn.collection.children.link(coll)

    import_zombies(args, coll, terrain)
    add_zombie_walls(args, coll, terrain)
    add_birdview_camera(args)

    scn.frame_start = args.start_frame
    scn.frame_end = args.start_frame + args.duration - 1
    scn.frame_set(args.start_frame)
    bpy.ops.wm.save_as_mainfile(filepath=out, compress=True, relative_remap=True)
    print(f"wrote {out}  (frames {scn.frame_start}-{scn.frame_end}, "
          f"{args.duration / 30.0:.1f} s at 30 fps)")


if __name__ == "__main__":
    main()
