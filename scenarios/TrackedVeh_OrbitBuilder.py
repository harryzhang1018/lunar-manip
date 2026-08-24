"""M113 builder on a circular wall-building orbit -- orbit-planned pick-and-place.

Same rig as `TrackedVeh_Builder.py` (M113 tracked vehicle, LRV gripper arm welded
to the hull, SMC contact, brakes held) dropped into the global site layout from
the concept sketch: concentric orbits about a single center, with the wall being
built on the inner ring, the builders on the middle ring, and the fetchers
unloading rocks on the outer ring.

What is different from `TrackedVeh_Builder.py`:
  * `orbit_planning.OrbitPlanner` owns the site geometry and hands the builder its
    work: the start pose on the vehicle orbit, and NUM_PLACE_POINTS discrete points
    on the wall orbit. Those points replace the single fixed `PLACE_POINT` -- rock
    i is dropped on wall point i, so the rocks lay out along the wall arc instead
    of piling up at one spot.
  * The builder starts on the vehicle orbit headed *along* it (station 90 deg ->
    (0, 33) headed -X, the CCW direction of travel), so the wall arc it is
    building runs alongside it on its left and the rocks sit alongside on its
    right, on the outer unloading side -- the sketch's layout. At station 0 deg,
    (33, 0), that same tangent heading is +Y.
  * NUM_ROCKS rocks instead of 5, all spawned dynamic and colliding in a wedge
    alongside the builder, and laid in phases rather than all at once: BUILD_PLAN
    alternates pick-and-place phases with dumps of stand-in rock, which is how a
    rubble wall actually goes up -- lay some rock, bring the fill up over it, lay
    some more on the new surface. As shipped: 5 rocks on the arc, a dump to a
    0.6 m crest, 3 rocks on that crest, a dump to 0.9 m, 2 rocks on the new crest.
  * The wall and vehicle orbits and the assigned place points are drawn as
    visual-only markers so the plan is visible in the render window.
  * The hull is pinned for the parked build (PARK_CHASSIS_FIXED), because a braked
    M113 only holds still at heading 0 -- see that constant for the measurements.
  * The dumps are stand-in rocks (`build_wall`), spawned fixed in courses
    WALL_COURSE_RISE apart: a course costs ~14 s of sim per rock to lay for real
    and nothing to fake, and nothing in this scenario interacts with the wall's
    interior anyway. Their shape comes from `orbit_planning.SlopedWallStacker` --
    a constant-slope section, so the wall is a triangle in cross-section rather
    than a straight-sided stack, WALL_SLOPE being the crest height over the base
    half-width. It is prescribed on the wall orbit rather than measured off the
    rocks the gripper laid, so its geometry is known before the run starts, and
    each dump raises the crest and widens the base together at that fixed ratio.
  * No two rocks are the same rock. Every body in the scene -- the ten the gripper
    lays and the couple of thousand the dumps spawn -- draws a geometry from the
    shape library in `LRV_Arm` (see ROCK_VARIANTS there, and `rock_variant` here
    for which rock draws which). The variants are lumped and stretched copies of
    the base mesh, held inside 0.8-1.2 of its size because the gripper has to close
    on all of them, and they are shared rather than rebuilt per body, so a wall of
    them costs no more to spawn than a wall of one mesh did.
  * The rocks of a crest phase are real: the gripper picks them out of the wedge
    and sets them down on the wall as it stands. Their place points are worked out
    at run time from the crest that exists (`top_rock_targets`), and the wall keeps
    its collision so they land on it instead of falling through. Each is pinned as
    it touches down, and the next dump then buries it -- placing rock and filling
    over it is the point.
  * At the end of the run every rock's final pose is written to CSV (see
    `write_rock_state`), so a later run can respawn the site as this one left it.

The builder stays parked while it builds, as in `TrackedVeh_Builder.py`. That is
what caps the arc one builder can cover in one go: the arm reaches ~2-4 m and 1
deg of the 30 m wall orbit is 0.52 m, so PLACE_HALF_SPAN_DEG is 5 deg (a ~5.2 m
arc), not the sketch's 15 deg. A longer wall is therefore built in *sections* --
park, build, move along the orbit, build again -- which is the scenario's second
mode.

MODE 2 (`--continue`) builds the next section onto a finished one:

  * Every rock of the previous run is respawned from its `rock_state.csv` as
    scenery -- fixed and with no collision model at all, so the wall that is
    already standing costs nothing to carry (see `spawn_scenery_rocks`).
  * The builder starts where the last section was built and drives
    SECTION_STEP_DEG (= WALL_RIDGE_SPAN_DEG, 10 deg) further along its orbit,
    following a Bezier arc of the orbit itself (`OrbitDrive`). The hull is
    unpinned for the drive and re-pinned on arrival.
  * A fresh load of NUM_ROCKS rocks is dropped in the wedge beside it *there*,
    and the same BUILD_PLAN runs again, on a wall arc re-centred on wherever the
    drive actually parked the arm.
  * A step of exactly one ridge span means consecutive sections' crest lines abut
    end to end while their (much wider) bases overlap -- one continuous berm
    rather than a row of separate mounds. The run writes both sections out, so
    `--continue` can be run again on its own output for a third.

Run with the project's conda env:

    conda run -n chrono python scenarios/TrackedVeh_OrbitBuilder.py
    conda run -n chrono python scenarios/TrackedVeh_OrbitBuilder.py --continue

Add `--headless` to step the simulation without opening a render window (used for
smoke tests). Add `--vsg` to render with the VSG (Vulkan) backend instead of the
default Irrlicht (OpenGL) one. Add `--save-frames [dir]` to dump each rendered
Irrlicht frame as a PNG sequence. Add `--plan-only` to print the orbit plan (and
the arm's reach to each rock's drop point) and exit without simulating. Add
`--state-dir DIR` to write the end-of-run state CSVs somewhere other than
DEFAULT_STATE_DIR, or `--no-state` to skip writing them. Add `--no-wall` to drop
the dumps from the build plan and run the pick-and-place phases on their own. Add
`--run-time N` to stop somewhere other than DEFAULT_RUN_TIME (0 = never, i.e. run
until the window is closed). Add `--continue [CSV]` for mode 2, reading the state
of the finished section from CSV (default: DEFAULT_STATE_DIR/rock_state.csv). Add
`--export-blender` to export a Chrono::Postprocess Blender scene (30 frames/s of
sim time) to the Chrono output dir's BLENDER subfolder, independent of rendering.

`wall_stack_demo.py` is this scenario's wall on its own -- no vehicle, no gripper,
no real rocks -- which is the quick way to look at the wall geometry or tune it.

To record the demo as a movie, render it with a stride and encode the PNGs. A
stride of 4 means the 30 frames/s the renderer draws are written at 7.5/s, so
playing them back at 30 fps runs the demo at 4x -- DEFAULT_RUN_TIME of sim
becomes ~1000 frames and a ~34 s film. Frame capture needs the Irrlicht window,
so no `--headless` and no `--vsg`:

    conda run -n chrono python scenarios/TrackedVeh_OrbitBuilder.py \\
        --save-frames artifacts/frames/demo --frame-stride 4
    ffmpeg -framerate 30 -i artifacts/frames/demo/frame_%06d.png -pix_fmt yuv420p out.mp4

The run ends by writing every rock in the scene -- the ones the gripper laid,
every stand-in rock of every dump, and (in mode 2) every rock imported from the
sections before it -- to `rock_state.csv` under DEFAULT_STATE_DIR, with the pose
needed to respawn each one exactly (see `write_rock_state`).
"""

import os
import sys
import csv
import math
import zlib
import random
import shutil
import datetime

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.postprocess as postprocess

# Make the repo root (for the `model` package) and the scenarios dir (for the
# sibling scenario modules reused below) importable regardless of the CWD.
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_root)
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Reuse the grasp state machine, the SMC finger-pad rebuild, and the rock recipe
# from the trailer/arm scenarios so the pick-and-place behaves identically here.
from LRV_Trailer import TrailerArm, match_gripper_contact_material
from LRV_Arm import (place_rock, rock_shape, ROCK_MESH, ROCK_SCALE, ROCK_DENSITY,
                     ROCK_VARIANTS, ROCK_SIZE_RANGE, ROCK_ASPECT, ROCK_LOBES,
                     ROCK_LOBE_GAIN, ROCK_SHAPE_SEED)
from orbit_planning import OrbitPlanner, SlopedWallStacker

# ---- site layout (see orbit_planning.py) ----
ORBIT_CENTER = (0.0, 0.0)
WALL_RADIUS = 30.0        # the wall being built; place points land here
# The ring the builder's hull rides. 3.4 m outside the wall orbit, not the 3.0 it
# was: at WALL_SLOPE 0.6 a finished 0.9 m crest needs a 1.5 m base half-width, and
# on a 3.0 m separation that put the wall's toe *into* the near track. Measured on
# the parked rig, taking the shoe bodies' own 0.19 m half-width into account (the
# thing the first pass at this missed -- shoe centres sit 0.19 m outboard of the
# belt's inner face, which is what the wall actually touches):
#
#     inner belt face                        r = 31.73 m
#     0.60 m crest, toe incl. rock size      r = 31.14 m   +0.59 m clear
#     0.90 m crest, toe incl. rock size      r = 31.64 m   +0.09 m  -- touching
#
# Moving the ring out is the cheap fix because the arm has the reach to spare: its
# IK tops out at 5.36 m horizontal at place height, against 4.21 m to the far end
# of its own arc. 33.4 spends 0.31 m of that (far place point 4.52 m, still 0.84 m
# short of the limit) and buys the 0.9 m crest +0.49 m of clearance -- which is
# what the wall used to have at the old 0.8 slope, so this restores a known-good
# geometry rather than inventing one. Going further is possible (34.0 gives
# +1.09 m and still holds 0.35 m of reach margin), but a near-full-extension arm
# places less accurately, and the ground layer is already the least accurate part.
VEHICLE_RADIUS = 33.4
BUILDER_STATION_DEG = 90.0  # builder 1 -- the only one for now -- on the +Y side
# Angular half-width of the wall arc this (parked) builder is assigned. See the
# module docstring: 5 deg is what the arm can reach without driving.
PLACE_HALF_SPAN_DEG = 5.0
PLACE_HEIGHT = 0.35       # release height above the ground over a wall point

# Build plan: what the builder does, in order. `("place", n)` is n pick-and-place
# cycles with the gripper -- real rocks, one per drop point -- and `("wall", h)` is
# one dump of stand-in rock, raising the wall's crest to h metres (see
# WALL_STAGE_HEIGHTS). The two alternate, which is how a rubble wall actually goes
# up: lay some rock, bring the fill up over it, lay some more on the new surface.
#
# The first place phase lands on the ground along the wall arc, one rock per point
# over NUM_PLACE_POINTS points. Every later one lands on the wall as it stands at
# that moment -- its drop points are worked out at run time from the crest that
# exists (`top_rock_targets`), since the wall has to be there first. Rocks laid on
# the crest are pinned as they touch down (PIN_TOP_ROCKS), and the next dump then
# buries them: they are inside the wall, not on it, which is what placing rock and
# then filling over it means.
BUILD_PLAN = (
    ("place", 5),    # ground layer, on the wall arc
    ("wall", 0.6),   # first dump: 0.60 m of crest on a 1.50 m base
    ("place", 3),    # laid on that crest
    ("wall", 0.9),   # second dump: 0.90 m of crest on a 2.25 m base
    ("place", 2),    # laid on the new crest
)
# A nominal rock height (m) -- see LRV_Arm.ROCK_SCALE. Sets how far a stand-in
# rock is bedded below the height it was sampled at, and how much of a rock stands
# above the top course's centres (i.e. where the wall's surface is).
#
# Nominal in two senses now. The library's rocks are 0.15-0.23 m tall, so no rock
# is exactly this; and the base rock is 0.182 m, so this is generous even for the
# average. That is deliberate on both counts: the bedding it feeds is overwritten a
# line later (`spawn_wall_rocks` lifts each rock's centre of mass onto the sampled
# height regardless), and the other use is the release height for a rock being set
# on the crest, where erring high costs a 3 cm drop and erring low would drive the
# gripper into the wall.
ROCK_HEIGHT = 0.26

# ...and what that plan works out to. The gripper lays NUM_ROCKS real rocks in
# all, GROUND_ROCKS of them on the arc and the rest on the wall; ROCK_PHASE names
# the phase each one belongs to, in build order ("ground", then "crest1" for the
# rocks laid on the first dump, and so on).
PLACE_PHASES = tuple(n for kind, n in BUILD_PLAN if kind == "place")
WALL_STAGE_HEIGHTS = tuple(h for kind, h in BUILD_PLAN if kind == "wall")
NUM_PLACE_POINTS = PLACE_PHASES[0]   # drop points on the wall arc, one per rock
GROUND_ROCKS = PLACE_PHASES[0]
NUM_ROCKS = sum(PLACE_PHASES)


def _rock_phases(plan):
    """Phase label per rock, in build order -- see ROCK_PHASE."""
    labels, dumps = [], 0
    for kind, value in plan:
        if kind == "wall":
            dumps += 1
        else:
            labels += [("ground" if dumps == 0 else f"crest{dumps}")] * value
    return tuple(labels)


ROCK_PHASE = _rock_phases(BUILD_PLAN)

BUILDER_FACING = "tangent"  # ride along the orbit; "outward" faces away from the wall
RIDE_HEIGHT = 0.7           # reference-frame height above the terrain at spawn

# Pin the chassis to ground for the parked build (TrackedVeh_Builder leaves it
# free and just holds the brakes). A braked M113 on flat rigid terrain only stays
# put when it is spawned at heading 0: measured over 8 s with no arm and no rocks,
# a braked, zero-throttle M113 drifts 0.08 m / 0.6 deg at heading 0, but 0.38 m /
# 7 deg at heading 180 and up to 0.9 m / 17 deg at 45/90/135 deg -- a sustained
# parasitic creep (it persists with the brakes released, and the track/suspension
# assembly is geometrically identical in the chassis frame at every heading, so it
# is not a strained spawn). This builder has to sit at whatever heading its orbit
# station implies, and at ~2 deg/s the hull rotates out from under a grasp that
# was planned against the world point seconds earlier -- the gripper then closes
# on empty ground. Pinning the hull removes it. Set False once the builder drives
# (a path-following controller closes the loop on this anyway), and expect the
# grasps to need re-planning against the live chassis frame.
PARK_CHASSIS_FIXED = True
# Side of the flat terrain patch (m), centered on the site: big enough to hold the
# whole layout (the outermost orbit is ~35 m out) with room to spare.
TERRAIN_SIZE = 100.0

planner = OrbitPlanner(center=ORBIT_CENTER, wall_radius=WALL_RADIUS,
                       vehicle_radius=VEHICLE_RADIUS,
                       num_points=NUM_PLACE_POINTS, half_span_deg=PLACE_HALF_SPAN_DEG,
                       place_height=PLACE_HEIGHT, ground_z=0.0)

# Initial vehicle pose (from the planner) and integration step. The builder rides
# the vehicle orbit tangentially, so at station 90 deg it sits at (0, 33) headed
# -X with the wall arc off its left flank.
INIT_LOC, INIT_ROT = planner.vehicle_pose(BUILDER_STATION_DEG, BUILDER_FACING,
                                          height=RIDE_HEIGHT)
STEP_SIZE = 5e-4

# Uniform geometric scale of the gripper arm (1.0 = as exported). Mass/inertia
# stay at 1x values (geometry-only scaling).
ARM_SCALE = 2.0

# Mount point of the arm base in the chassis *reference* frame (X forward, Y left,
# Z up; the reference origin sits at the front of the hull, over the sprockets).
# 2.5 m back puts the arm base on top of the hull, ahead of the idlers.
ARM_OFFSET = chrono.ChVector3d(-2.5, 0.0, 0.4)

# Where the arm base -- not the chassis reference -- ends up, and therefore the
# station angle the wall arc is centered on. With a tangential heading the arm
# base sits 2.5 m *along* the orbit from the chassis reference, so centering the
# arc on the vehicle's own station angle would leave one end of it ~6 m from the
# arm (out of reach) and the other end ~3 m. Centering on the arm base keeps all
# NUM_ROCKS points a symmetric ~3-4 m away, inside the arm's working range.
ARM_BASE = INIT_LOC + INIT_ROT.Rotate(ARM_OFFSET)
BUILD_STATION_DEG = planner.station_of(ARM_BASE)

# Orientation the arm is welded at, relative to the chassis. The whole arm is
# rotated rigidly about the mount point by this quaternion (here: 180 deg yaw
# about +Z). Set to None to mount in the imported orientation.
ARM_MOUNT_ROT = chrono.QuatFromAngleZ(math.pi)

# Rock spawn: NUM_ROCKS rocks on the ground in a polar wedge around the *arm
# base*. d is the distance from the base; theta is measured CCW from the chassis
# +X (forward) axis, so the wedge rides with the vehicle's heading. The wedge sits
# alongside the hull on the builder's right, which with a tangential heading is
# the outer, unloading side of the orbit -- where a fetcher would have dropped the
# rocks. It stays clear of the tracks (ROCK_MIN_LATERAL against a ~1.25 m track
# half-width) and well clear of the wall arc, which is off the other flank.
#
# The wedge is centered on the arm base rather than on the chassis reference
# (which is 2.5 m forward of it) because the reach that has to hold every rock is
# the arm's. Twice as many rocks need roughly twice the wedge area, and a
# chassis-centered wedge grown that far runs its near corner into the track and
# its far corner past the arm's reach; centered on the base it is just an annulus
# sector inside the reach. d spans 2.0-4.2 m, inside the 1.85-4.45 m the 10-rock
# wedge already grasped reliably. Each spawn is re-sampled until the gripper's IK
# can reach it and it is clear (ROCK_MIN_SEP) of the rocks already placed.
ROCK_D_RANGE = (2.0, 4.2)                 # m from the arm base
ROCK_THETA_RANGE_DEG = (-100.0, -40.0)    # deg CCW from chassis +X (forward)
# Minimum lateral offset (m) from the hull centerline. The arm base sits on that
# centerline, so this simply gates |d * sin(theta)|; it trims the near-track
# corner of the wedge, which the d/theta ranges on their own would leave in.
ROCK_MIN_LATERAL = 1.6
# Keep spawned rocks at least this far apart (m). The rocks are ~0.2 m across and
# are NOT scaled with the arm, so this is an absolute distance -- it has to exceed
# the rock width (or they spawn interpenetrating and the SMC contact kicks them
# apart), while still letting NUM_ROCKS of them fit in the wedge above.
ROCK_MIN_SEP = 0.3


# ---- which rock gets which shape ----
# Every rock in the scene draws a geometry from `LRV_Arm`'s shape library rather
# than all of them sharing one mesh -- the ten the gripper lays and the couple of
# thousand the dumps spawn alike. See ROCK_VARIANTS there for the library and for
# why the sizes stay inside 0.8-1.2 of the base rock: the gripper aims at a fixed
# height and has to close on all of them.
#
# The draw is a stable hash of the rock's name rather than the next number out of
# an RNG. That is worth the odd-looking crc32 for one reason: a `--continue` run
# respawns a finished section from CSV, and the shapes it comes back as have to be
# the shapes it was built with, or the seam between two sections re-rolls every
# time the wall is reloaded. A hash gives that for free -- and it gives it to the
# state files written before the `shape` column existed, which have no draw
# recorded at all. The section is folded in so section 2's `rock_0` is not
# section 1's `rock_0` wearing the same face a ridge span away.
def rock_variant(name, section=1):
    """Shape-library variant for the rock called `name` in `section`."""
    return zlib.crc32(f"{section}:{name}".encode()) % ROCK_VARIANTS


def rock_shape_for(name, section=1):
    """The `RockShape` for that rock -- built on first use, shared thereafter."""
    return rock_shape(rock_variant(name, section))


def strip_visual_materials(body):
    """Clear the textures/colors on `body`'s visual shapes in place.

    Every rock variant's visual shape is a single object shared across all
    rocks drawing that variant (see `place_rock`), so clearing it here via one
    rock also clears it for every other rock sharing that shape.
    """
    vis = body.GetVisualModel()
    if vis is None:
        return
    for i in range(vis.GetNumShapes()):
        vis.GetShape(i).GetMaterials().clear()


# End-effector grasp height: how high above the ground (z) the gripper center aims
# when reaching for a rock. The rocks are a fixed ~0.2 m tall AND the gripper
# fingers are NOT scaled with the arm (they stay 1x so they can grasp the small
# rocks), so the grab height does NOT scale with ARM_SCALE.
GRAB_HEIGHT_FACTOR = 1.0
GRAB_HEIGHT = GRAB_HEIGHT_FACTOR * 0.22

# Max gripper speed (m/s) at the moment a rock is released over its wall point.
# TrackedVeh_Builder drops every rock on one pile, so it lets go the instant the
# gripper reaches the point -- typically mid-swing at several m/s, which throws
# the rock ~0.8 m past the target. Here each rock has its own point 0.58 m from
# its neighbours, so the arm has to settle before letting go.
PLACE_SPEED_TOL = 0.2

# Shoulder angle (rad) the arm raises to after releasing, before it slews back to
# its home pose. Without this the arm swings home straight from the low place
# pose, dragging the open fingers through the rock it just set down and punting it
# several metres off the wall (it happened to ~1 rock in 3).
STOW_LIFT_THETA2 = math.radians(60.0)

# Pick-and-place timing. The vehicle holds its brakes the whole time; once
# T_GRASP_START has elapsed (rig + rocks settled) the gripper picks each rock up
# in turn and carries it to its own place point on the wall orbit.
T_GRASP_START = 6.0
PLACE_POINTS = planner.place_points(BUILD_STATION_DEG)

# A pick-and-place cycle that has not finished in this long is stuck, not slow: a
# good one takes ~12.8 s of sim, worst case ~15. Stuck has one cause -- the grasp
# plans its IK once, from where the rock was when the cycle began, and its
# close-and-retry path does not re-solve it (see `TrailerArm.replan_grasp`). A rock
# the approach nudges out from under the pads is then unreachable, and the arm
# closes on the spot it used to be, over and over, for the rest of the run.
# Measured: rock 3 of a 10-rock run burned 90 s that way and took 7 rocks down with
# it. So re-aim once at where the rock actually is, and if that does not take
# either, leave the rock where it lies and get on with the build -- one missing
# rock beats a run that never finishes.
GRASP_REPLAN_AFTER = 18.0   # s into a cycle: re-solve the IK against the rock now
GRASP_SKIP_AFTER = 30.0     # s into a cycle: abandon this rock, move to the next

# How long a run lasts (seconds of sim time), with or without a render window.
# NUM_ROCKS pick-and-place cycles at ~12.8 s each plus T_GRASP_START come to ~134
# s; the dumps cost no sim time at all, since the wall is spawned rather than
# built. So this is the last rock settling and not much after it -- `--run-time N`
# stretches it (0 = run until the window is closed, which needs a window).
DEFAULT_RUN_TIME = 135.0
DEFAULT_FRAME_DIR = os.path.join(project_root, "artifacts", "frames", "trackedveh_orbitbuilder")

# Where the end-of-run state CSVs land (see write_rock_state / write_site_plan).
# Both files are rewritten in place by every run, so a resume always reads the
# latest; the run timestamp is recorded inside them.
DEFAULT_STATE_DIR = os.path.join(project_root, "artifacts", "states", "trackedveh_orbitbuilder")
ROCK_STATE_FILE = "rock_state.csv"
SITE_PLAN_FILE = "site_plan.csv"

# ---- the stand-in wall (see orbit_planning.SlopedWallStacker) ----
# The wall the BUILD_PLAN dumps raise. Every rock in it is spawned fixed and is
# never integrated: they are scenery, and what they buy is the ~14 s of sim per
# rock per course that laying them with the gripper would cost. All the arm builds
# for real is NUM_ROCKS rocks -- the ground layer, and a handful set on the crest
# after each dump.
#
# The shape is a constant-slope triangle in cross-section, WALL_SLOPE being the
# height over the base half-width -- the tangent of the face angle. Dumped rubble
# cannot stand steeper than its slip angle, so a wall of this stuff is a berm with
# sloping faces, not a straight-sided stack: 0.6 m of crest needs 2.0 m of base.
# `SlopedWallStacker` holds that ratio fixed as the wall grows, so each `("wall",
# h)` entry in BUILD_PLAN is one dump of rock -- a taller crest on a
# proportionally wider base, with only the shell between the two profiles actually
# added.
#
# The wall is *prescribed*, not measured off the rocks the gripper laid: it is
# centred on the wall orbit over the arc the builder was assigned, so its geometry
# is known before the run starts and does not inherit the ground layer's scatter
# (which is ~1.3 m wide, wider than a 0.6 m wall should be at the base).
#
# 0.60 is a 31 deg face, and it replaces the 0.80 (38.7 deg) this scenario shipped
# with, which read as too steep for dumped rubble -- 38.7 deg is at the top of the
# range dry granular material will hold, and lunar regolith poured into a pile
# stands at about 30-35.
#
# What the slope costs is clearance for the builder's own tracks: the wall grows
# symmetrically about the wall orbit, so a gentler face pushes the outer toe out
# toward the vehicle orbit. The toe of a finished 0.9 m crest, measured to the
# outermost rock's *surface* rather than its centre (the widest variant in the
# shape library adds 0.14 m), against an inner belt face at VEHICLE_RADIUS - 1.67:
#
#     slope   face    base half-width   toe r    clear at V_RADIUS 33.4
#      0.80   38.7d           1.125 m   31.27 m        +0.47 m   (was shipped)
#      0.70   35.0d           1.286 m   31.43 m        +0.31 m
#      0.60   31.0d           1.500 m   31.64 m        +0.09 m   (now, at 33.4)
#      0.50   26.6d           1.800 m   31.94 m        -0.21 m
#
# Those are the numbers at the *old* 3.0 m orbit separation, and the 0.6 row is why
# VEHICLE_RADIUS moved out to 33.4 -- see it for the measurements. At 33.4 the 0.6
# row reads +0.49 m, i.e. this slope now has the clearance the 0.8 slope used to.
# Going gentler again is a matter of moving the ring out again, until the arm runs
# out of reach at about VEHICLE_RADIUS 34.5.
WALL_SLOPE = 0.6              # crest height / base half-width -- 31.0 deg faces
WALL_RIDGE_SPAN_DEG = 2.0 * PLACE_HALF_SPAN_DEG   # crest line = the built arc
# Courses of rock centres this far apart in z. Well under a rock height (0.26 m),
# so consecutive courses overlap and the faces knit into something solid instead
# of reading as shelves -- the same trick as the density below, which is several
# times what the gripper's own layer achieves. Stacking scenery wants the opposite
# of what physics would give you: none of these bodies collide with each other, so
# interpenetration is free and is what closes the gaps.
WALL_COURSE_RISE = 0.10
WALL_ROCK_DENSITY = 24.0   # rocks per m^2 of course footprint (~240 per m^3)
WALL_MIN_SEP = 0.08        # m; under the rock width, so rocks knit rather than tile
WALL_SEED = 0              # fixed so the same run gives the same scatter

# ---- the rocks laid on the wall's crest ----
# These are real rocks: spawned dynamic and colliding like the ground layer, picked
# up by the gripper, and carried onto the stand-in wall once it is standing. For
# them to land on it rather than drop through it, the wall is spawned colliding
# (FAKE_WALL_COLLIDES) -- fixed bodies with a collision model are just static
# geometry. The alternative -- releasing over the wall and pinning the rock mid-air
# the instant the gripper opens -- needs no collision at all, but the rock then
# hangs wherever the release happened instead of settling into the rubble.
#
# The wall is only solid to things that are not part of it: collision families put
# every stand-in rock in FAMILY_FAKE_WALL and bar that family from colliding with
# itself or with the built ground layer. Both exclusions are load-bearing.
#   * Internally: the courses are packed to overlap on purpose (WALL_MIN_SEP is
#     under a rock width and WALL_COURSE_RISE well under a rock height), so every
#     stand-in rock overlaps its neighbours and each pair would be a narrowphase
#     test, every step, for contacts the solver then throws away because both
#     bodies are fixed.
#   * Against the built layer: the wall's lower courses stand in and around the
#     rocks the gripper laid -- it is prescribed on the wall orbit, they scattered
#     about it -- and those are live bodies. Under SMC that overlap is a penalty
#     force with nothing to balance it, and it would fire the built layer across
#     the site the moment the wall appeared.
#
# Families are not enough on their own, because they only skip the *narrowphase*.
# The broadphase still has to find every one of those pairs before it can filter
# them, and a few hundred deliberately interpenetrating AABBs packed into a few
# cubic metres is a lot of pairs to find. Measured over 400 steps of the real
# scene, one 672-rock dump:
#
#     no wall                                    1.91 ms/step
#     672 rocks, collision off                   2.03 ms/step   (free)
#     672 rocks, collision + families           16.12 ms/step   (8.5x)
#     672 rocks, top 2 courses solid             2.13 ms/step   (1.1x)
#     672 rocks, collision, no families        319.49 ms/step   (164x)
#
# So the wall is solid only where something can actually touch it: the courses
# within WALL_SOLID_DEPTH of the crest. Everything below that is buried inside the
# wall -- nothing in this scenario can reach it, and a rock that could would have
# had to pass through the solid shell first. It also means a dump's solid band
# stops mattering once the next dump buries it; leaving those rocks collidable
# costs nothing measurable, so they are left alone rather than re-bound.
FAKE_WALL_COLLIDES = True
WALL_SOLID_DEPTH = 0.25  # m below the crest that keeps its collision
FAMILY_BUILT_ROCKS = 2   # the ground-layer rocks the gripper lays
FAMILY_FAKE_WALL = 3     # every stand-in rock
# Where along the wall's crest line the crest rocks go: a phase of n of them is
# spread evenly over this fraction range of the ridge. Kept off the ends: the
# ridge's midpoint is ~3.2 m from the arm base and its ends ~4.2 m, and these
# points sit ~0.7 m higher than the wall course, so the middle is where the reach
# is most comfortable.
TOP_ROCK_ARC_RANGE = (0.3, 0.7)
# Gripper release height above the wall's top surface (m). A rock hangs ~0.12 m
# below the gripper centre and its own centre sits ~0.10 m above whatever it rests
# on, so 0.22 m would have it touching down as the fingers open; 0.23 leaves it a
# centimetre to fall. Every centimetre of that drop is bounce, and bounce on a
# crest only ~2x a rock wide is what sends a rock over the side.
TOP_PLACE_CLEARANCE = 0.23
# Pin each crest rock as soon as it is down: once the gripper has let go, the rock
# is frozen the first step it is slower than TOP_SETTLE_SPEED, or has drifted
# TOP_SETTLE_DRIFT from the point it was aimed at, or after TOP_SETTLE_TIMEOUT
# seconds -- whichever comes first.
#
# Timing is the whole point, and the sloped wall tightened it. Waiting for the
# pick-and-place cycle to end pins the rock ~5 s after release, and a rock does not
# sit still on a rubble crest for 5 s. But the crest of a constant-slope wall is a
# ridge one rock wide, with 38.7 deg faces either side of it: measured, a rock set
# on it and left for 1.5 s was not on the wall's shoulder, it was on the ground at
# the foot of the face (2 of 3 rocks, at z=0.12 and z=0.26 under a 0.68 m crest).
# It is not released badly -- each one reaches its aim point and is set down below
# the speed tolerance -- it just cannot balance on a ridge. So the drift trigger,
# which fires the moment it starts to go, and a timeout short enough that the rock
# is pinned where it landed rather than where it rolled to. Set PIN_TOP_ROCKS False
# to leave the crest rocks live and let them roll where they will.
PIN_TOP_ROCKS = True
TOP_SETTLE_SPEED = 0.05    # m/s below which the rock counts as come to rest
TOP_SETTLE_DRIFT = 0.12    # m horizontally from the aim point -- it is rolling off
TOP_SETTLE_TIMEOUT = 0.6   # s after release, pin it regardless


# Drop points for the ground phase: rock i on wall point i, so the builder walks
# the arc once. The later phases' targets do not exist yet -- they are aimed at a
# wall that has not been dumped (`top_rock_targets`), and `run` appends them as
# each dump finishes.
GROUND_TARGETS = list(PLACE_POINTS)


# ---- mode 2: the next section along (--continue) ----
# One parked builder covers PLACE_HALF_SPAN_DEG either side of its arm, so a wall
# longer than that arc goes up in sections. `--continue` builds the next one: it
# imports a finished run's rocks as scenery, drives the builder this far along its
# orbit, drops a fresh load of rock beside it and runs BUILD_PLAN again.
#
# One ridge span is the natural step. The crest line a dump lays runs
# WALL_RIDGE_SPAN_DEG of orbit (it is the arc the builder was assigned), so
# stepping by exactly that much butts consecutive crest lines end to end, while
# the bases -- half_width wider than the ridge at each end, i.e. 2.2 deg either
# side for a 0.9 m crest -- overlap into one continuous berm.
SECTION_STEP_DEG = WALL_RIDGE_SPAN_DEG
# ...but a ridge span is *not* enough to clear the previous section's footprint.
# The base runs half_width past the ridge at each end (1.125 m for a 0.9 m crest),
# so stepping by the ridge span leaves the two bases overlapping by 4.3 deg of
# orbit -- which is exactly what makes the finished wall continuous, since each
# section's sloping end stands where its neighbour's does, and the crest never
# dips. Step far enough to clear the footprint instead (12 deg) and the crest
# notches down to 0.48 m between sections.
#
# What that overlap does break is the *ground layer*: the near end of a
# continuation section's reach is under the previous section's toe. Measured, the
# first drop point of section 2 landed at 90.58 deg -- 0.09 deg from the end of
# section 1's ridge, i.e. under 0.9 m of wall, and the rock fell straight through
# the (collision-free) scenery to the ground inside it.
#
# So the wall keeps its ridge-span step and the ground layer gets clipped instead:
# `ground_targets` pulls the near end of the arc in to the last section's outermost
# rock plus this much clearance, and respreads the points over what is left. One
# rock width plus a little air -- the rock being laid and the one it would touch
# are each ~0.26 m across, so 0.35 m of centre-to-centre clearance leaves a gap.
SECTION_CLEARANCE = 0.35
# The state file `--continue` reads when it is given no path of its own. Note that
# the run then *rewrites* this same file at the end, with the imported rocks and
# the new section in it -- the whole file is read into memory before anything is
# spawned, so re-reading and rewriting one path is safe.
DEFAULT_RESUME_FILE = os.path.join(DEFAULT_STATE_DIR, ROCK_STATE_FILE)
# Imported rocks are scenery in the strictest sense: `SetFixed(True)` so they are
# never integrated and `EnableCollision(False)` so they never reach the broadphase
# either. Nothing in the new section can touch the old one -- it is a full ridge
# span away, and the builder's orbit passes 1.6 m outside the widest base -- so
# they only have to be *there*. Measured in the same benchmark as
# FAKE_WALL_COLLIDES: 672 collision-off rocks cost 0.12 ms/step over an empty
# scene, i.e. nothing. Their names are prefixed `s<section>_` so a rock imported
# from section 1 cannot collide (by name) with the identically named rock the new
# section is about to spawn -- the gripper looks bodies up by name.
SCENERY_NAME_PREFIX = "s{section}_"

# ---- the relocation drive ----
# The builder follows a Bezier arc of its own orbit with a ChPathFollowerDriver,
# unpinned for the drive and re-pinned when it parks (see PARK_CHASSIS_FIXED --
# the creep that constant exists for is exactly what a *parked* hull suffers, and
# a driven one is closing the loop on its path anyway).
#
# Measured on the bare rig over a 10 deg step: cruising at 0.9 m/s the whole way
# and braking on arrival overshoots by 1.6 deg (0.9 m), because a braked M113
# takes ~1.5 s to stop and the tracks carry it on. So the target speed is ramped
# down over the last DRIVE_DECEL_DEG to DRIVE_CRAWL_SPEED, and the brakes go on
# DRIVE_STOP_LEAD_DEG early to absorb what is left. The drive takes ~11 s of sim.
#
# How far *along* it stops is not load-bearing -- the new section's wall arc and
# place points are both re-centred on the arm base the drive achieved (see `run`),
# so the geometry the arm has to reach is identical to the first section's however
# the drive went, and the lead only keeps the seam between sections tight.
#
# The *radius* it holds is load-bearing, and it is what set these gains. A pure-P
# controller chasing a sentinel 3 m ahead cuts the corner of the orbit and settles
# inside it, and the arm's mass makes that worse than the bare rig showed -- which
# matters because every centimetre it parks inside the ring is a centimetre off the
# clearance VEHICLE_RADIUS was sized to give. Measured over the 10 deg step with
# the arm fitted, against the 33.00 m orbit these were run on:
#
#     Kp 0.8  Ki 0.05  look 3.0     parked r 32.31   0.69 m inside the ring
#     Kp 0.8  Ki 0.40  look 3.0     parked r 32.66   0.34 m inside
#     Kp 1.2  Ki 0.40  look 2.0     parked r 32.82   0.18 m inside
#     Kp 1.2  Ki 1.00  look 2.0     parked r 32.95   0.05 m inside   <- shipped
#
# The absolute track-gap figures that used to be quoted here were computed from an
# assumed 1.6 m hull half-width; the real inner belt face is 1.67 m in, and the
# ring has since moved to 33.4, so what the sweep actually establishes is the
# ranking and the 0.05 m radius error -- which is what VEHICLE_RADIUS budgets for.
#
# The heavy integral term is what removes the steady-state bias on a curve; the
# shorter look-ahead is what stops it cutting the corner in the first place. It
# costs a little stopping accuracy (0.8 deg of coast rather than 0.3), which
# DRIVE_STOP_LEAD_DEG is set from.
DRIVE_SPEED = 0.9            # m/s cruise along the orbit
DRIVE_DECEL_DEG = 3.0        # deg of arc to ramp the target speed down over
DRIVE_CRAWL_SPEED = 0.15     # m/s at the end of that ramp
DRIVE_STOP_LEAD_DEG = 0.8    # deg short of target to hit the brakes
DRIVE_LOOK_AHEAD = 2.0       # m; steering controller's sentinel distance
DRIVE_STEER_GAINS = (1.2, 1.0, 0.0)    # Kp, Ki, Kd -- Ki holds the orbit radius
DRIVE_SPEED_GAINS = (0.6, 0.1, 0.0)
DRIVE_SETTLE = 2.0           # s braked on arrival before the hull is re-pinned
DRIVE_TIMEOUT = 45.0         # s of sim rolling after which it gives up and parks
DRIVE_START_DELAY = T_GRASP_START  # s braked at the start -- the same settle mode
#                                    1 takes before the arm is allowed to work
# Mode 2's run length. The relocation costs DRIVE_START_DELAY + ~11 s of driving +
# DRIVE_SETTLE before the arm may start, and the rocks it drops need the same
# settle again -- measured, the first grasp of the second section begins at ~25 s
# against mode 1's 6 s, and the tenth rock is down at ~142 s. The margin over that
# is wider than mode 1's because the rock wedge is sampled unseeded, so how long a
# cycle takes varies with where the rocks happen to land; a long tail costs a few
# seconds of film, a short one costs the last rock of the section.
CONTINUE_RUN_TIME = DEFAULT_RUN_TIME + 35.0


def measure_built_belt(rocks):
    """Measure the belt `rocks` occupy, straight off the live bodies.

    `OrbitPlanner.measure_belt` reads the rows of a `rock_state.csv`; this hands it
    the same fields taken from bodies in the running system, so the belt can be
    measured mid-run without a round trip through the file.
    """
    rows = [{"placed": 1, "com_x": r.GetPos().x, "com_y": r.GetPos().y,
             "com_z": r.GetPos().z} for r in rocks]
    return planner.measure_belt(rows, rock_height=ROCK_HEIGHT)


def wall_stacker(theta_center_deg=BUILD_STATION_DEG, seed=WALL_SEED,
                 slope=WALL_SLOPE, rise=WALL_COURSE_RISE,
                 density=WALL_ROCK_DENSITY, min_sep=WALL_MIN_SEP,
                 ridge_span_deg=WALL_RIDGE_SPAN_DEG):
    """The `SlopedWallStacker` for this site: a wall on the arc the builder works.

    Prescribed geometry, so it can be made before anything has been built -- the
    crest line runs along the wall orbit over the same arc the place points cover,
    centred on the arm's station. Call `grow()` on it to raise the wall (see
    `build_wall`); a fresh stacker is a wall of zero height.

    `theta_center_deg` defaults to where the builder starts, which is the only
    station mode 1 ever builds at. Mode 2 passes the station the relocation drive
    actually reached, so the second section sits on the arm that will build it
    rather than on the plan the drive was aiming at.
    """
    return SlopedWallStacker(planner, theta_center_deg=theta_center_deg,
                             ridge_span_deg=ridge_span_deg, slope=slope,
                             rise=rise, density=density, min_sep=min_sep,
                             seed=seed)


def arm_base_of(vehicle):
    """World position of the arm's mount point for the vehicle's *current* pose.

    ARM_BASE is this for the spawn pose; once the builder has driven anywhere
    (mode 2) the live one is what the arm's reach, its rock wedge and the wall arc
    it can work all hang off.
    """
    rot = vehicle.GetChassisBody().GetRot()
    return vehicle.GetChassis().GetPos() + rot.Rotate(ARM_OFFSET)


def read_rock_rows(path):
    """A `rock_state.csv` as a list of raw string dicts, in file order.

    Deliberately unparsed, unlike `orbit_planning.read_rock_state`: mode 2 respawns
    these rocks and then writes them straight back out again (see
    `write_rock_state`), so keeping the columns as they were written round-trips a
    section unchanged however many times it is carried forward.
    """
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def row_section(row, default=1):
    """Which build section a `rock_state.csv` row belongs to (1 if unrecorded)."""
    try:
        return int(float(row.get("section") or default))
    except (TypeError, ValueError):
        return default


def row_variant(row, name, section):
    """Shape-library variant a `rock_state.csv` row was built with.

    The `shape` column if the file records one, and otherwise the name hash that
    wrote it -- which is the same answer, and is what lets a file written before
    the column existed come back as the wall it was rather than as a re-roll.
    Reading the column rather than always re-deriving it means the rule in
    `rock_variant` can change without invalidating the walls already built.
    """
    try:
        return int(float(row["shape"])) % ROCK_VARIANTS
    except (KeyError, TypeError, ValueError):
        return rock_variant(name, section)


def occupied_arc_end(rows, default=None):
    """Station (deg) the wall in `rows` reaches to -- the far edge of its toe.

    Measured off the stand-in rocks rather than the whole file: they *are* the wall
    and they were sampled over a prescribed profile, so there are no strays, while
    a rock the gripper punted off the arc would stretch this by a metre and push
    the next section's ground layer needlessly far along. The laid rocks stand
    inside the wall's footprint anyway. Falls back to every row if a file has no
    stand-in rock in it, and to `default` if it has nothing measurable at all.
    """
    wall = [row for row in rows if str(row.get("fake", "")).strip() in ("1", "1.0")]
    thetas = [float(row["theta_deg"]) for row in (wall or rows)
              if row.get("theta_deg") not in (None, "")]
    return max(thetas) if thetas else default


def ground_targets(station_deg, occupied_to_deg=None, count=NUM_PLACE_POINTS,
                   clearance=SECTION_CLEARANCE):
    """Drop points for a ground layer on the arc the builder can reach.

    With nothing in the way this is `planner.place_points`: `count` points spread
    evenly over PLACE_HALF_SPAN_DEG either side of the station, endpoints included.

    `occupied_to_deg` is how far along the orbit the sections already built reach
    (`occupied_arc_end`). Because sections step by one ridge span their bases
    overlap -- see SECTION_CLEARANCE -- so the near end of a continuation section's
    reach is under the previous section's toe, and a rock dropped there ends up
    inside the wall rather than on the ground. The near end is therefore pulled in
    to clear it and the points respread over what is left. The far end never moves,
    so the longest reach the arm has to make is exactly the first section's.
    """
    lo = station_deg - PLACE_HALF_SPAN_DEG
    hi = station_deg + PLACE_HALF_SPAN_DEG
    if occupied_to_deg is not None:
        lo = max(lo, occupied_to_deg + math.degrees(clearance / WALL_RADIUS))
    if lo >= hi:
        raise ValueError(
            f"nothing left of the builder's arc at station {station_deg:.2f} deg: "
            f"earlier sections reach to {occupied_to_deg:.2f} deg, past its far end "
            f"at {hi:.2f} deg -- SECTION_STEP_DEG is too small to build on")
    step = (hi - lo) / (count - 1) if count > 1 else 0.0
    return [planner.to_world(WALL_RADIUS, lo + i * step, PLACE_HEIGHT)
            for i in range(count)]


def spawn_scenery_rocks(system, rows, vis=None):
    """Respawn every rock in `rows` as scenery: fixed, and with no collision at all.

    This is how mode 2 puts the sections already built back into the scene. Each
    row's *reference* frame is what gets restored (`write_rock_state` explains
    why), so the rock stands exactly where the run that wrote it left it -- down to
    its rotation, since the ref frame carries both.

    Two things make this cheap enough to do for thousands of rocks. Geometry comes
    from the shape library, whose meshes, mass properties and visual shapes are
    built once and shared by every body that draws them, rather than re-read from
    the .obj per rock. And no collision model is created at all -- not a disabled
    one, none -- so these bodies never reach the broadphase (see
    SCENERY_NAME_PREFIX for why nothing in the new section needs them to be solid).

    Each row's shape is the one that row was *built* with: the `shape` column if
    the file has it, and otherwise the same name hash that produced it, so a state
    file written before the column existed still comes back as the wall it was.
    See `rock_variant`.

    Names are prefixed with the section the row came from, because rock names
    repeat across sections and the gripper looks bodies up by name.
    """
    bodies = []
    for row in rows:
        section = row_section(row)
        name = str(row.get("rock", "rock"))
        shape = rock_shape(row_variant(row, name, section))
        body = chrono.ChBodyAuxRef()
        body.SetName(SCENERY_NAME_PREFIX.format(section=section) + name)
        # Same COM offset and mass as `place_rock` gives a live rock: the CSV
        # records the ref frame, and ref -> COM is what turns it back into a pose.
        body.SetFrameCOMToRef(chrono.ChFramed(shape.com, shape.principal_rot))
        body.SetMass(shape.mass * ROCK_DENSITY)
        body.SetInertiaXX(shape.principal_I * ROCK_DENSITY)
        body.SetFrameRefToAbs(chrono.ChFramed(
            chrono.ChVector3d(float(row["ref_x"]), float(row["ref_y"]),
                              float(row["ref_z"])),
            chrono.ChQuaterniond(float(row["ref_e0"]), float(row["ref_e1"]),
                                 float(row["ref_e2"]), float(row["ref_e3"]))))
        body.SetFixed(True)
        body.EnableCollision(False)
        body.AddVisualShape(shape.visual)
        strip_visual_materials(body)
        system.Add(body)
        if vis is not None:
            vis.BindItem(body)
        bodies.append(body)
    return bodies


class OrbitDrive:
    """Walks the builder `step_deg` further along its vehicle orbit, then parks it.

    A `ChPathFollowerDriver` on a Bezier arc of the orbit itself. Progress is
    measured at the *arm base*, not the chassis reference, for the same reason the
    wall arc is centred there (see ARM_BASE): the arm is what has to end up over
    the next section.

    Drive it from the simulation loop:

        inputs = drive.step(time)      # None once it has parked
        ...
        drive.advance(STEP_SIZE)

    `step` returns driver inputs until it has reached its target, braked, and sat
    still for DRIVE_SETTLE; then None, and `park()` re-pins the hull. The hull is
    unpinned by the constructor, so build the drive when you want it to start.
    """

    def __init__(self, vehicle, step_deg=SECTION_STEP_DEG, speed=DRIVE_SPEED):
        self.vehicle = vehicle
        self._last_deg = planner.station_of(arm_base_of(vehicle))
        self.start_deg = self._last_deg
        self.step_deg = step_deg
        self.target_deg = self.start_deg + step_deg
        self.speed = speed
        # The path runs from a little behind the start to well past the target:
        # the steering controller chases a sentinel DRIVE_LOOK_AHEAD ahead of the
        # vehicle, and it must never run off the end of the curve.
        points = chrono.vector_ChVector3d()
        lo, hi, segments = -0.1 * step_deg, 1.6 * step_deg, 64
        for i in range(segments + 1):
            world = planner.to_world(VEHICLE_RADIUS,
                                     self.start_deg + lo + i * (hi - lo) / segments,
                                     0.0)
            points.push_back(chrono.ChVector3d(world.x, world.y, 0.0))
        self.path = chrono.ChBezierCurve(points)
        self.driver = veh.ChPathFollowerDriver(vehicle, self.path, "vehicle orbit",
                                               speed)
        self.driver.GetSteeringController().SetLookAheadDistance(DRIVE_LOOK_AHEAD)
        self.driver.GetSteeringController().SetGains(*DRIVE_STEER_GAINS)
        self.driver.GetSpeedController().SetGains(*DRIVE_SPEED_GAINS)
        self.driver.Initialize()
        self.braking_since = None
        self.rolling_since = None
        self.timed_out = False
        self.t0 = None

    @property
    def station_deg(self):
        """Arm-base station now, unwrapped so progress accumulates past +-180."""
        deg = planner.station_of(arm_base_of(self.vehicle))
        while deg - self._last_deg > 180.0:
            deg -= 360.0
        while deg - self._last_deg < -180.0:
            deg += 360.0
        self._last_deg = deg
        return deg

    @staticmethod
    def _braked():
        inputs = veh.DriverInputs()
        inputs.m_throttle, inputs.m_steering, inputs.m_braking = 0.0, 0.0, 1.0
        return inputs

    def step(self, time):
        """Driver inputs for this step, or None once the builder has parked."""
        if self.t0 is None:
            self.t0 = time
        if time - self.t0 < DRIVE_START_DELAY:
            return self._braked()   # let the rig settle before pulling away
        if self.rolling_since is None:
            # A pinned hull cannot drive. Unpinning is left until the moment it
            # pulls away, because an unpinned M113 sitting on its brakes at this
            # heading creeps (see PARK_CHASSIS_FIXED) -- doing it in the
            # constructor would let it wander through the settle. Progress is
            # measured from wherever it is at that moment, not from the pose the
            # path was drawn through.
            self.rolling_since = time
            self.vehicle.GetChassisBody().SetFixed(False)
            self.vehicle.GetSystem().Setup()
            self.start_deg = self.station_deg
            self.target_deg = self.start_deg + self.step_deg
        if self.braking_since is None:
            # Brake DRIVE_STOP_LEAD_DEG early and ramp the target speed down over
            # the last DRIVE_DECEL_DEG before that, so the coast is short.
            remaining = self.target_deg - self.station_deg - DRIVE_STOP_LEAD_DEG
            self.timed_out = time - self.rolling_since > DRIVE_TIMEOUT
            if remaining <= 0.0 or self.timed_out:
                self.braking_since = time
            else:
                self.driver.SetDesiredSpeed(
                    self.speed if remaining > DRIVE_DECEL_DEG
                    else max(DRIVE_CRAWL_SPEED,
                             self.speed * remaining / DRIVE_DECEL_DEG))
                self.driver.Synchronize(time)
                return self.driver.GetInputs()
        if time - self.braking_since > DRIVE_SETTLE:
            return None
        return self._braked()

    def advance(self, step_size):
        """Advance the path follower (a no-op before it rolls or after it brakes)."""
        if self.rolling_since is not None and self.braking_since is None:
            self.driver.Advance(step_size)

    def park(self):
        """Re-pin the hull if it was parked pinned; return the arm base it reached."""
        if PARK_CHASSIS_FIXED:
            self.vehicle.GetChassisBody().SetFixed(True)
            self.vehicle.GetSystem().Setup()
        return arm_base_of(self.vehicle)

    def describe(self):
        """One-line summary of where the drive ended up, against where it aimed."""
        base = arm_base_of(self.vehicle)
        r, station = planner.to_polar(base)
        chassis_r, _ = planner.to_polar(self.vehicle.GetChassis().GetPos())
        return (f"  relocated     : arm station {self.start_deg:.2f} -> "
                f"{station:.2f} deg ({station - self.start_deg:+.2f} of "
                f"{self.target_deg - self.start_deg:+.2f} asked), arm base r="
                f"{r:.2f} m, hull r={chassis_r:.2f} m (orbit {VEHICLE_RADIUS:.1f})"
                + ("  [TIMED OUT]" if self.timed_out else ""))


def spawn_wall_rocks(system, courses, stage=1, collides=FAKE_WALL_COLLIDES,
                     crest_z=None, solid_depth=WALL_SOLID_DEPTH, vis=None,
                     section=1, exporter=None):
    """Spawn a stand-in rock body at every pose in `courses`; return the bodies.

    `courses` is what `SlopedWallStacker.grow` hands back -- poses grouped by
    course -- and the return is the same shape, with bodies in place of poses.

    Every rock is `SetFixed(True)`, which is what makes the shortcut safe as well
    as cheap: pinned bodies are not integrated, so they cannot sag, drift or
    explode. With `collides` the wall is solid to anything that is not part of it
    -- so a rock laid on the crest lands on it instead of dropping through -- while
    collision families keep it transparent to itself and to the built layer
    standing in it. Only the courses within `solid_depth` of `crest_z` are given
    that collision; the rest of the wall is visual only, because nothing can reach
    it and paying the broadphase for it costs 8x the step time. Pass `crest_z=None`
    to make every course solid. See FAKE_WALL_COLLIDES for the measurements behind
    all three of those.

    `place_rock` beds a rock on a ground plane, so each course's plane is passed as
    `ground_z` half a rock below the course height, and the body is then shifted so
    its centre of mass lands exactly on the sampled height. That last step matters:
    the rock mesh's centre of mass sits ~0.09 m above its base, not at half its
    0.26 m height, so bedding alone would put every course ~4 cm low and leave the
    section shallower than the profile it was sampled from. The pose's rotation is
    applied about that same centre of mass, so the rock spins in place instead of
    swinging away from the point it was sampled at.

    With a render window already up, pass `vis` so each new body gets bound to it
    -- Irrlicht builds its scene nodes when the visual system binds, so a body
    added mid-run is invisible until it is bound. Likewise pass `exporter` (a
    `postprocess.ChBlender` set up by the caller) to register each new rock with
    it: `AddAll()` only snapshots the bodies that exist when it is called, so a
    wall dumped after that call is invisible to the Blender export otherwise --
    `ExportData()` only ever writes state for items the exporter knows about.
    """
    layers = []
    solid_any = False
    for k, poses in enumerate(courses, start=1):
        layer = []
        # A course is solid only if something could reach it: near enough the
        # crest to be on the wall's surface rather than inside it. The epsilon is
        # not cosmetic -- WALL_SOLID_DEPTH is a multiple of WALL_COURSE_RISE, so
        # the cut lands exactly on a course plane and float error alone decides
        # which side of it that course falls on.
        solid = collides and (crest_z is None
                              or poses[0][0].z > crest_z - solid_depth - 1e-9)
        solid_any = solid_any or solid
        for i, (pos, rot) in enumerate(poses):
            # Each stand-in draws its own shape from the library, keyed on the
            # name it is about to get: a face made of one mesh repeated reads as
            # tiling however the rotations are scattered.
            name = f"wall_{stage}_{k}_{i}"
            rock = place_rock(system, pos, ground_z=pos.z - ROCK_HEIGHT / 2.0,
                              contact_method=chrono.ChContactMethod_SMC,
                              shape=rock_shape_for(name, section))
            rock.SetName(name)
            strip_visual_materials(rock)
            # Turn the rock about its own centre of mass: the reference frame sits
            # off to one side of the mesh, so rotating that frame in place would
            # carry the rock with it, off the sampled point. The same step lifts
            # the centre of mass onto the sampled height (see the docstring).
            com = rock.GetPos()
            ref = rock.GetFrameRefToAbs()
            lift = chrono.ChVector3d(0.0, 0.0, pos.z - com.z)
            rock.SetFrameRefToAbs(chrono.ChFramed(
                com + rot.Rotate(ref.GetPos() - com) + lift, rot * ref.GetRot()))
            rock.SetFixed(True)          # scenery: never integrated
            # Solid to the rocks laid on top of it, transparent to itself and to
            # the built layer it stands in -- see FAKE_WALL_COLLIDES.
            model = rock.GetCollisionModel()
            model.SetFamily(FAMILY_FAKE_WALL)
            model.DisallowCollisionsWith(FAMILY_FAKE_WALL)
            model.DisallowCollisionsWith(FAMILY_BUILT_ROCKS)
            rock.EnableCollision(solid)
            if vis is not None:
                # Bind just this body, not the whole scene: BindAll would walk
                # every item already bound (hull, road wheels, all 100+ track
                # shoes) as well.
                vis.BindItem(rock)
            if exporter is not None:
                exporter.Add(rock)
            layer.append(rock)
        layers.append(layer)

    if solid_any:
        # Register the new bodies with the collision system. Adding a body to a
        # ChSystem that has already been stepped does NOT put its collision model
        # in the collision system -- the body simulates, renders and reports
        # contacts of zero, and anything aimed at it passes straight through. This
        # wall is built ~290 s into the run, so without this the whole thing is
        # phantom: measured, the rocks laid on top of it fell through it and landed
        # on the ground. `ChSystem::Setup()` does not do it; only re-binding the
        # collision system does.
        system.GetCollisionSystem().BindAll()
    return layers


def build_wall(system, stacker=None, heights=WALL_STAGE_HEIGHTS,
               collides=FAKE_WALL_COLLIDES, vis=None, section=1, exporter=None):
    """Raise the stand-in wall to each crest height in `heights`, in turn.

    One `grow()` + spawn per entry, so a wall can come up in dumps: each stage
    raises the crest and widens the base by the same ratio (the slope never
    changes) and only the shell between the old profile and the new one is
    spawned. Each dump's own crest is what decides which of its courses are solid
    (see `spawn_wall_rocks`), so the surface a rock can be laid on is always the
    one standing at the time. Returns `(profile, layers)` -- the finished profile,
    and every course of bodies from every stage, in the order they were spawned.
    """
    stacker = wall_stacker() if stacker is None else stacker
    layers = []
    for height in heights:
        if height <= stacker.profile.height + 1e-9:
            continue
        profile, courses = stacker.grow(height=height)
        spawned = spawn_wall_rocks(system, courses, stage=stacker.stage,
                                   collides=collides, crest_z=profile.crest_z,
                                   vis=vis, section=section, exporter=exporter)
        layers += spawned
        solid = sum(1 for course in spawned for rock in course
                    if rock.IsCollisionEnabled())
        print(f"  wall stage {stacker.stage}: +{sum(len(c) for c in spawned)} rocks "
              f"({solid} of them solid) -> {2 * profile.half_width:.2f} m base, "
              f"{profile.height:.2f} m high (crest z={profile.crest_z:.2f} m)")
    print(stacker.describe())
    return stacker.profile, layers


def wall_top_z(profile, fake=()):
    """Height (m) of the top surface of the finished wall.

    Measured off the stand-in bodies when there are any -- the highest rock centre
    plus half a rock, which is the surface the top rocks are laid on -- and off the
    profile's crest otherwise. The two differ by about half a rock: a course of
    centres at the crest height puts rock tops above it.
    """
    if not fake:
        return profile.crest_z
    highest = max(rock.GetPos().z for layer in fake for rock in layer)
    return highest + ROCK_HEIGHT / 2.0


def top_rock_targets(profile, surface_z, num):
    """Release points for one phase of crest rocks, along the wall's crest line.

    `num` points spread evenly over TOP_ROCK_ARC_RANGE of the ridge, on the wall's
    centerline radius (the thickest part of the section, and the only part of it
    that reaches the crest), a TOP_PLACE_CLEARANCE drop above `surface_z`.
    """
    lo, hi = TOP_ROCK_ARC_RANGE
    fracs = ([0.5 * (lo + hi)] if num < 2
             else [lo + i * (hi - lo) / (num - 1) for i in range(num)])
    # The ridge span, taken from the ridge itself rather than from
    # `theta_span_deg(crest_z)`: the footprint at exactly the crest is the ridge,
    # but it is also where the profile's width rounds through zero.
    span = 2.0 * math.degrees(profile.ridge_half_length / profile.r_center)
    start = profile.theta_center_deg - 0.5 * span
    return [planner.to_world(profile.r_center, start + frac * span,
                             surface_z + TOP_PLACE_CLEARANCE)
            for frac in fracs]


def sample_rock_position(arm_base, chassis_rot, terrain):
    """Random ground position for a rock, in polar coordinates around `arm_base`.

    theta is drawn from ROCK_THETA_RANGE_DEG (measured CCW from the chassis +X
    axis) and d from ROCK_D_RANGE, both relative to `arm_base` (the arm mount
    point). The sampled offset is rotated by `chassis_rot`, so the wedge rides with
    the vehicle's heading rather than sitting in world axes. Returns a world
    `ChVector3d` resting on the terrain, or None if the draw fell inside
    ROCK_MIN_LATERAL of the hull centerline (i.e. on the tracks) -- the caller is
    re-sampling anyway, for reach and separation.
    """
    lo, hi = ROCK_THETA_RANGE_DEG
    theta = math.radians(random.uniform(lo, hi))
    d = random.uniform(*ROCK_D_RANGE)
    # The arm base sits on the hull centerline, so the lateral (chassis y) offset
    # of the sample is just this -- no transform needed to gate it.
    if abs(d * math.sin(theta)) < ROCK_MIN_LATERAL:
        return None
    offset = chassis_rot.Rotate(chrono.ChVector3d(d * math.cos(theta),
                                                  d * math.sin(theta), 0.0))
    tx, ty = arm_base.x + offset.x, arm_base.y + offset.y
    ground_z = terrain.GetHeight(chrono.ChVector3d(tx, ty, arm_base.z + 5.0))
    return chrono.ChVector3d(tx, ty, ground_z)


def add_orbit_visuals(system, place_points=()):
    """Draw the site plan: the wall and vehicle orbits, and the assigned points.

    Everything hangs off one fixed, collision-free body, so it is render-only and
    never touches the dynamics: the two orbit rings as chains of thin boxes just
    above the ground, and a flat disc marker on each assigned place point. Mode 2
    passes no points here -- it does not know them until the drive has parked, and
    adds them with `add_place_markers` then.
    """
    markers = chrono.ChBody()
    markers.SetFixed(True)
    markers.EnableCollision(False)
    markers.SetName("orbit_markers")

    # Keep the rings thin: the place markers below are drawn on the wall ring, and
    # a wide band swallows them.
    for radius, color, width in ((WALL_RADIUS, chrono.ChColor(0.9, 0.75, 0.1), 0.08),
                                 (VEHICLE_RADIUS, chrono.ChColor(0.2, 0.5, 0.9), 0.08)):
        for mid, yaw, length in planner.ring_segments(radius, z=0.03):
            segment = chrono.ChVisualShapeBox(length, width, 0.02)
            segment.SetColor(color)
            markers.AddVisualShape(segment,
                                   chrono.ChFramed(mid, chrono.QuatFromAngleZ(yaw)))

    _add_place_discs(markers, place_points)
    system.Add(markers)
    return markers


def _add_place_discs(body, points):
    """Flat disc markers on `points`, on the visual model of `body`."""
    for point in points:
        disc = chrono.ChVisualShapeCylinder(0.20, 0.02)
        disc.SetColor(chrono.ChColor(0.95, 0.3, 0.05))
        body.AddVisualShape(disc, chrono.ChFramed(
            chrono.ChVector3d(point.x, point.y, 0.05), chrono.QUNIT))


def add_place_markers(system, points, name="place_markers", vis=None, exporter=None):
    """Place-point discs added on their own, mid-run (mode 2, after the drive).

    Same markers `add_orbit_visuals` draws, on a separate render-only body, so a
    section whose arc is only known once the builder has parked still shows where
    it is aiming. `vis` binds the new body to a render window that is already up.
    Likewise pass `exporter` so a Blender export -- if one is running -- learns
    about this body too; being added mid-run means `AddAll()` never saw it (see
    `spawn_wall_rocks` for why that doesn't happen automatically).
    """
    markers = chrono.ChBody()
    markers.SetFixed(True)
    markers.EnableCollision(False)
    markers.SetName(name)
    _add_place_discs(markers, points)
    system.Add(markers)
    if vis is not None:
        vis.BindItem(markers)
    if exporter is not None:
        exporter.Add(markers)
    return markers


def spawn_wedge_rocks(system, gripper, terrain, arm_pos, chassis_rot,
                      count=NUM_ROCKS, first=0, vis=None, section=1, exporter=None):
    """Drop `count` rocks on the ground in the spawn wedge beside the builder.

    Each spot is re-sampled until it clears the tracks, the gripper's IK can reach
    it, and it is ROCK_MIN_SEP clear of the rocks already down (see ROCK_D_RANGE
    for the wedge itself). The attempt budget is per rock and generous because the
    last few are fitted into whatever gaps the first ones left.

    `arm_pos` and `chassis_rot` are the builder's *current* arm base and heading,
    so mode 2 gets its wedge beside wherever the relocation drive parked rather
    than beside the spawn pose. `first` offsets the rock numbering, and with `vis`
    (a render window already up) each new body is bound as it is made -- both are
    for that mid-run spawn. Likewise pass `exporter` for a mid-run spawn (mode 2's
    fresh load of rock at the new section) so the Blender export -- if one is
    running -- learns about these rocks too; see `spawn_wall_rocks` for why that
    doesn't happen automatically.
    """
    rocks, placed_xy = [], []
    for i in range(first, first + count):
        for _attempt in range(400):
            rock_pos = sample_rock_position(arm_pos, chassis_rot, terrain)
            if rock_pos is None:
                continue  # too close to the tracks -> resample
            try:
                gripper.plan_grasp(rock_pos.x, rock_pos.y)
            except ValueError:
                continue  # out of reach -> resample
            if all(math.hypot(rock_pos.x - px, rock_pos.y - py) > ROCK_MIN_SEP
                   for px, py in placed_xy):
                break     # reachable and well clear of the other rocks
        else:
            raise RuntimeError("no reachable, well-separated rock position found")
        name = f"rock_{i}"  # unique name so the gripper locks the right one, and
        #                     what its shape is drawn from -- see `rock_variant`
        rock = place_rock(system, rock_pos, ground_z=rock_pos.z,
                          contact_method=chrono.ChContactMethod_SMC,
                          shape=rock_shape_for(name, section))
        rock.SetName(name)
        strip_visual_materials(rock)
        if ROCK_PHASE[i] == "ground":
            # Tag the rocks destined for the ground layer, so the stand-in wall
            # that later gets dumped over them can be told to ignore them (see
            # FAKE_WALL_COLLIDES). The crest rocks stay in the default family: the
            # wall has to be solid to those, or they drop through it.
            rock.GetCollisionModel().SetFamily(FAMILY_BUILT_ROCKS)
        if vis is not None:
            vis.BindItem(rock)
        if exporter is not None:
            exporter.Add(rock)
        rocks.append(rock)
        placed_xy.append((rock_pos.x, rock_pos.y))
    if system.GetChTime() > 0.0:
        # Bodies added to a system that has already been stepped are not in the
        # collision system until it is re-bound -- without this a mode 2 rock is
        # phantom: it falls through the terrain and the gripper cannot touch it.
        # See `spawn_wall_rocks` for the same trap.
        system.GetCollisionSystem().BindAll()
    return rocks


def build_scene(spawn_rocks=True, scenery_rows=(), section=1):
    """Create a system with the M113 tracked vehicle, welded arm, terrain, rocks.

    Returns (m113, vehicle, terrain, gripper, rocks), where `m113` is the M113
    wrapper and `vehicle` is its underlying `ChTrackedVehicle`.

    Mode 2 passes `spawn_rocks=False` -- its rocks are dropped beside the builder
    after the relocation drive, not at the spawn pose -- and `scenery_rows`, the
    rows of a finished run's `rock_state.csv`, which are respawned as fixed,
    collision-free scenery (`spawn_scenery_rocks`).
    """
    # The M113 model files live under the Chrono data root's `vehicle/` tree, and
    # some (e.g. the track shoe collision hulls) are referenced relative to the
    # data root itself -- so point the vehicle data path at the env's vehicle
    # dir and leave the Chrono data root at its default. (The arm loads its own
    # meshes from the project's data dir, independent of these.)
    veh.SetVehicleDataPath(os.path.join(chrono.GetChronoDataPath(), "vehicle") + os.sep)

    # ---- M113 tracked vehicle ----
    # Configuration matches TrackedVeh_Builder.py (which mirrors demo_VEH_M113.cpp
    # apart from SMC contact and the SIMPLE engine, which stops the braked vehicle
    # creeping forward under idle torque -- though not the heading-dependent creep
    # PARK_CHASSIS_FIXED deals with). The high-iteration BB solver set below is
    # what keeps the single-pin track shoes from drifting off the wheels.
    m113 = veh.M113()
    m113.SetContactMethod(chrono.ChContactMethod_SMC)
    m113.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
    m113.SetDoublePinTrackShoeType(veh.DoublePinTrackShoeType_ONE_CONNECTOR)
    m113.SetTrackBushings(False)          # SMC iterative solvers can't use bushings
    m113.SetSuspensionBushings(False)
    m113.SetTrackStiffness(False)
    m113.SetDrivelineType(veh.DrivelineTypeTV_BDS)
    m113.SetBrakeType(veh.BrakeType_SHAFTS)
    m113.SetEngineType(veh.EngineModelType_SIMPLE)
    m113.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    m113.SetChassisCollisionType(veh.CollisionType_NONE)
    m113.SetChassisFixed(PARK_CHASSIS_FIXED)  # see PARK_CHASSIS_FIXED above
    m113.CreateTrack(True)
    m113.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    m113.Initialize()

    track_vis = chrono.VisualizationType_MESH
    m113.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    m113.SetSprocketVisualizationType(chrono.VisualizationType_MESH)
    m113.SetIdlerVisualizationType(track_vis)
    m113.SetSuspensionVisualizationType(track_vis)
    m113.SetIdlerWheelVisualizationType(track_vis)
    m113.SetRoadWheelVisualizationType(track_vis)
    m113.SetTrackShoeVisualizationType(track_vis)

    # Swap in a custom chassis mesh in place of the stock M113 Chassis.obj.
    chassis_body = m113.GetVehicle().GetChassisBody()
    chassis_body.GetVisualModel().Clear()
    custom_chassis_mesh = chrono.ChTriangleMeshConnected()
    custom_chassis_mesh.LoadWavefrontMesh(
        os.path.expanduser("~/Downloads/Chassis.obj"), True, True)
    custom_chassis_shape = chrono.ChVisualShapeTriangleMesh()
    custom_chassis_shape.SetMesh(custom_chassis_mesh)
    chassis_body.AddVisualShape(custom_chassis_shape, chrono.ChFramed())

    # Strip the mesh textures/colors baked into the M113's OBJ/MTL files, while
    # keeping the real tread/hull geometry (VisualizationType_MESH above) rather
    # than falling back to primitive shapes. Only the vehicle's own bodies exist
    # in the system at this point -- terrain, arm and markers are added later --
    # so this can't touch anything else's visuals.
    for body in m113.GetSystem().GetBodies():
        strip_visual_materials(body)

    system = m113.GetSystem()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    solver = chrono.ChSolverBB()
    solver.SetMaxIterations(100)
    solver.SetOmega(0.8)
    solver.SetSharpnessLambda(1.0)
    system.SetSolver(solver)
    system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

    vehicle = m113.GetVehicle()

    # ---- Gripper arm welded to the hull ----
    # `TrailerArm` locks its base to the chassis body, so the arm rides rigidly
    # with the M113. ARM_OFFSET is a *chassis-frame* offset, so rotate it into the
    # world by the chassis heading before adding it to the chassis reference point
    # -- otherwise a builder parked at a non-zero station angle would get its arm
    # welded off to the side of the hull. ARM_MOUNT_ROT is likewise
    # chassis-relative (arm_model composes it with the chassis rotation), and the
    # grasp IK solves in that same frame. The arm builds NSC finger pads, so
    # rebuild them as SMC to match this (SMC) scene.
    chassis_rot = vehicle.GetChassisBody().GetRot()
    arm_pos = vehicle.GetChassis().GetPos() + chassis_rot.Rotate(ARM_OFFSET)
    gripper = TrailerArm(system, arm_pos, vehicle, scale=ARM_SCALE,
                         mount_rot=ARM_MOUNT_ROT)

    # Same texture/color strip as the M113 above, applied to the arm's own
    # bodies now that they exist -- the earlier pass ran before the gripper
    # was built, so it couldn't reach these.
    for body in (gripper.base, gripper.shoulder, gripper.biceps, gripper.elbow,
                gripper.wrist, gripper.endoffactor, gripper.finger_1, gripper.finger_2):
        strip_visual_materials(body)

    match_gripper_contact_material(gripper, chrono.ChContactMethod_SMC, ARM_SCALE)
    gripper.GRAB_HEIGHT = GRAB_HEIGHT
    gripper.PLACE_SPEED_TOL = PLACE_SPEED_TOL    # set the rock down, don't throw it
    gripper.STOW_LIFT_THETA2 = STOW_LIFT_THETA2  # and lift clear before slewing home

    # ---- Flat rigid patch at ground level, centered on the site ----
    # Material matches the demo: mu=0.9, cr=0.2, Y=2e7 (Y unused by SMC). Sized to
    # cover the whole orbit layout, not just the builder.
    terrain = veh.RigidTerrain(system)
    minfo = chrono.ChContactMaterialData()
    minfo.mu = 0.9
    minfo.cr = 0.2
    minfo.Y = 2e7
    patch_mat = minfo.CreateMaterial(chrono.ChContactMethod_SMC)
    # `visualization=False`: SetColor/SetTexture only style the patch's visual
    # box shape, which AddPatch builds regardless of them (Patch::m_visualize
    # defaults True) -- this is the actual switch that skips building it.
    patch = terrain.AddPatch(
        patch_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(ORBIT_CENTER[0], ORBIT_CENTER[1], 0),
                           chrono.QUNIT),
        TERRAIN_SIZE, TERRAIN_SIZE, 1, False, 1, False)
    terrain.Initialize()

    # ---- Rocks on the ground in the spawn wedge alongside the builder ----
    rocks = (spawn_wedge_rocks(system, gripper, terrain, arm_pos, chassis_rot,
                               section=section)
             if spawn_rocks else [])

    # ---- Sections already built, as scenery (mode 2) ----
    if scenery_rows:
        spawn_scenery_rocks(system, scenery_rows)

    # Mode 2 has no place points to draw yet: they follow from where the
    # relocation drive parks, so `run` adds them once it has.
    add_orbit_visuals(system, PLACE_POINTS if spawn_rocks else ())

    return m113, vehicle, terrain, gripper, rocks


def make_vis(vehicle, title, use_vsg=False):
    """Create and initialize a tracked-vehicle visualization attached to `vehicle`.

    Defaults to Irrlicht (OpenGL); pass `use_vsg=True` for the VSG (Vulkan)
    backend. Both expose the same run/render/step interface, so only the setup
    differs: VSG has its own lighting/sky API and wants `AttachVehicle()` before
    `Initialize()`.
    """
    if use_vsg:
        vis = veh.ChTrackedVehicleVisualSystemVSG()
        vis.SetWindowTitle(title)
        vis.SetWindowSize(1280, 1024)
        vis.SetLightIntensity(1.0)
        vis.SetLightDirection(2.0, 0.75)
        vis.EnableShadows()
        vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 2.75), 15.0, 1.5)
        vis.AttachVehicle(vehicle)
        vis.Initialize()
        return vis

    vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle(title)
    vis.SetWindowSize(1280, 1024)
    # Further back and higher than TrackedVeh_Builder's chase camera: the work
    # area here is a ~5 m arc off the builder's flank plus the rock pile off the
    # other flank, and a close chase view only frames the hull.
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 14.0, 3.0)
    vis.Initialize()
    # Chrono's typical fill lights only -- no extra shadow light over the builder.
    # Those fill lights sit near the site center, which the builder is only 33 m
    # from (TrackedVeh_Builder's scene is ~120 m out, where they are much weaker),
    # so on their own they light this scene fully; stacking a shadow light on top
    # blows the hull and the rocks out to white.
    vis.AddTypicalLights()
    vis.AttachVehicle(vehicle)
    return vis


def parse_frame_output_dir(argv):
    """Return the PNG output directory requested by --save-frames, or None."""
    for arg in argv:
        if arg.startswith("--save-frames="):
            return os.path.expanduser(arg.split("=", 1)[1])
    if "--save-frames" not in argv:
        return None
    idx = argv.index("--save-frames")
    if idx + 1 < len(argv) and not argv[idx + 1].startswith("--"):
        return os.path.expanduser(argv[idx + 1])
    return DEFAULT_FRAME_DIR


def parse_state_dir(argv):
    """Return the state-CSV output directory: DEFAULT_STATE_DIR unless overridden.

    `--no-state` turns the dump off (None); `--state-dir DIR` (or `--state-dir=DIR`)
    writes somewhere else.
    """
    if "--no-state" in argv:
        return None
    for arg in argv:
        if arg.startswith("--state-dir="):
            return os.path.expanduser(arg.split("=", 1)[1])
    if "--state-dir" in argv:
        idx = argv.index("--state-dir")
        if idx + 1 < len(argv) and not argv[idx + 1].startswith("--"):
            return os.path.expanduser(argv[idx + 1])
    return DEFAULT_STATE_DIR


def parse_resume_file(argv):
    """Path `--continue` should import a finished section from, or None.

    `--continue` on its own means DEFAULT_RESUME_FILE; `--continue PATH` (or
    `--continue=PATH`) reads somewhere else.
    """
    for arg in argv:
        if arg.startswith("--continue="):
            return os.path.expanduser(arg.split("=", 1)[1])
    if "--continue" not in argv:
        return None
    idx = argv.index("--continue")
    if idx + 1 < len(argv) and not argv[idx + 1].startswith("--"):
        return os.path.expanduser(argv[idx + 1])
    return DEFAULT_RESUME_FILE


def parse_number_option(argv, flag, default, cast=int):
    """Value of `--flag N` or `--flag=N` in `argv`, cast to `cast`, or `default`."""
    for arg in argv:
        if arg.startswith(f"{flag}="):
            return cast(arg.split("=", 1)[1])
    if flag in argv:
        idx = argv.index(flag)
        if idx + 1 < len(argv) and not argv[idx + 1].startswith("--"):
            return cast(argv[idx + 1])
    return default


ROCK_STATE_COLUMNS = ["section", "rock", "shape", "index", "place_point", "course",
                      "placed", "fake", "layer",
                      "target_x", "target_y", "target_z",
                      "ref_x", "ref_y", "ref_z",
                      "ref_e0", "ref_e1", "ref_e2", "ref_e3",
                      "com_x", "com_y", "com_z",
                      "r", "theta_deg", "place_err", "place_err_xy",
                      "sim_time", "run_stamp"]


def write_rock_state(path, rocks, targets, placed, sim_time, stamp, fake=(),
                     carried=(), section=1):
    """Write every rock's final pose to `path` as CSV, one row per rock.

    This is the file to resume from. The pose that respawns a rock exactly is the
    *reference* frame (ref_x..ref_e3): `place_rock` builds each rock as a
    `ChBodyAuxRef` and sets that frame, so a later run rebuilds the same body and
    calls

        rock.SetFrameRefToAbs(ChFramed(ChVector3d(ref_x, ref_y, ref_z),
                                       ChQuaterniond(ref_e0, ref_e1, ref_e2, ref_e3)))

    to put it back where this run left it. The COM position (com_*, what
    `rock.GetPos()` returns) and the site polar coordinates (r, theta_deg) are
    recorded alongside for analysis, and the target columns say which wall point
    and which course the rock was aimed at. `place_err` is the 3D distance from
    that target and `place_err_xy` the horizontal one: the target is a *release*
    point PLACE_HEIGHT up, so a rock set down perfectly still reads ~0.25 m of
    `place_err` from the drop alone -- `place_err_xy` is the one that says whether
    it landed on its wall point.

    All NUM_ROCKS rocks are written, placed or not. `placed` is a per-rock flag
    sequence -- 1 for those whose full pick-carry-release-stow cycle finished. A
    run cut off mid-cycle leaves its last rock reading 0 even if it had already
    been released, and so does one the arm gave up on (GRASP_SKIP_AFTER), which is
    why this is not simply a count of how many rocks were reached.

    `fake` (the stand-in wall from `build_wall`, grouped per course) is written
    after them with `fake=1` and `layer` counting up from 1, and no target columns
    -- they were sampled over a profile, not aimed at a point. Anything measuring a
    belt from this file must filter them out (`fake == 0`), or it measures the wall
    instead of the rocks the gripper laid. The rocks set on the crest are real
    (`fake=0`) but are marked with their phase (`layer=crest1`, `crest2`, ...) and
    carry no place point, since they are aimed at the wall's surface rather than at
    a planned point on the arc.

    `carried` is the rows of the earlier sections this run imported as scenery
    (mode 2), written out ahead of this run's own rocks exactly as they came in;
    `section` numbers the rows this run contributes. So the file always holds the
    whole wall, and `--continue` can be pointed at its own output. Note that
    `rock` names repeat across sections -- the pair (section, rock) is the key.

    `shape` is the rock's index into the shape library (see `rock_variant`), and
    is what makes the pose columns mean anything: a ref frame respawns the rock in
    the right place, but only with the mesh it was built from does it respawn as
    the same rock. Rows carried forward from a file written before this column keep
    the shape their names imply, resolved once and written out explicitly here.
    """
    def pose_columns(rock):
        """The ref-frame pose, COM and site polar columns shared by every row."""
        ref = rock.GetFrameRefToAbs()
        pos, rot, com = ref.GetPos(), ref.GetRot(), rock.GetPos()
        r, theta = planner.to_polar(com)
        return [f"{pos.x:.6f}", f"{pos.y:.6f}", f"{pos.z:.6f}",
                f"{rot.e0:.9f}", f"{rot.e1:.9f}", f"{rot.e2:.9f}", f"{rot.e3:.9f}",
                f"{com.x:.6f}", f"{com.y:.6f}", f"{com.z:.6f}",
                f"{r:.4f}", f"{theta:.4f}"]

    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(ROCK_STATE_COLUMNS)
        # The sections built before this one, carried through verbatim: they were
        # written by this same writer, they have not moved (mode 2 respawns them
        # fixed), and re-emitting the row rather than re-deriving it means a
        # section survives any number of `--continue` runs unchanged.
        for row in carried:
            values = [row.get(column, "") for column in ROCK_STATE_COLUMNS]
            # A file written before the section column existed is all section 1;
            # leaving it blank would make the *next* `--continue` number its own
            # section 2 as well, and two sections would share body names.
            carried_section = row_section(row)
            values[ROCK_STATE_COLUMNS.index("section")] = carried_section
            # Same for the shape: a file written before that column existed has
            # its rocks' geometry only implied by their names, and pinning it now
            # is what stops a later change to `rock_variant` re-rolling a wall
            # that has already been built and filmed.
            values[ROCK_STATE_COLUMNS.index("shape")] = row_variant(
                row, str(row.get("rock", "rock")), carried_section)
            writer.writerow(values)
        for i, rock in enumerate(rocks):
            # A ground-phase rock is aimed at a numbered place point on the arc; a
            # crest rock gets none, and its target only exists once the run has
            # worked out where the wall's surface ended up.
            ground = ROCK_PHASE[i] == "ground"
            place_point = i if ground else ""
            course = 0 if ground else ""
            layer = "" if ground else ROCK_PHASE[i]
            variant = rock_variant(rock.GetName(), section)
            if i >= len(targets):
                writer.writerow([section, rock.GetName(), variant, i, place_point,
                                 course, int(i < len(placed) and placed[i]), 0, layer,
                                 "", "", ""]
                                + pose_columns(rock)
                                + ["", "", f"{sim_time:.3f}", stamp])
                continue
            target, com = targets[i], rock.GetPos()
            writer.writerow([section, rock.GetName(), variant, i, place_point, course,
                             int(i < len(placed) and placed[i]), 0, layer,
                             f"{target.x:.6f}", f"{target.y:.6f}", f"{target.z:.6f}"]
                            + pose_columns(rock)
                            + [f"{(com - target).Length():.4f}",
                               f"{math.hypot(com.x - target.x, com.y - target.y):.4f}",
                               f"{sim_time:.3f}", stamp])
        index = len(rocks)
        for layer_idx, layer in enumerate(fake, start=1):
            for rock in layer:
                # `course` carries the dump a stand-in rock belongs to (its name
                # leads with it too), `layer` the course, counting up across the
                # whole wall rather than restarting at each dump.
                stage = rock.GetName().split("_")[1]
                writer.writerow([section, rock.GetName(),
                                 rock_variant(rock.GetName(), section),
                                 index, "", stage, 1, 1,
                                 layer_idx, "", "", ""]
                                + pose_columns(rock)
                                + ["", "", f"{sim_time:.3f}", stamp])
                index += 1
    return path


def write_site_plan(path, vehicle, sim_time, stamp, placed, section=1):
    """Write the site geometry and rig setup to `path` as a key,value CSV.

    Everything a resume needs that is not per-rock: the orbit layout (from which
    `OrbitPlanner` regenerates the same place points), the builder's pose, how the
    arm is mounted, and the rock recipe the state file's poses belong to.
    """
    chassis = vehicle.GetChassis().GetPos()
    rot = vehicle.GetChassisBody().GetRot()
    rows = [
        ("scenario", os.path.basename(__file__)),
        ("run_stamp", stamp),
        ("sim_time", f"{sim_time:.3f}"),
        ("section", section),
        ("section_step_deg", SECTION_STEP_DEG),
        ("rocks_placed", sum(placed)),
        ("orbit_center_x", ORBIT_CENTER[0]),
        ("orbit_center_y", ORBIT_CENTER[1]),
        ("wall_radius", WALL_RADIUS),
        ("vehicle_radius", VEHICLE_RADIUS),
        ("ground_z", 0.0),
        ("num_place_points", NUM_PLACE_POINTS),
        ("half_span_deg", PLACE_HALF_SPAN_DEG),
        ("place_height", PLACE_HEIGHT),
        ("rock_height", ROCK_HEIGHT),
        # The shape library the state file's `shape` column indexes into (the mesh
        # and scale it is built from are recorded with the rest of the rig below).
        # A run that changes any of these is describing different rocks by the same
        # index, which is exactly what a resume needs to be able to notice.
        ("rock_variants", ROCK_VARIANTS),
        ("rock_size_range", " ".join(f"{s:g}" for s in ROCK_SIZE_RANGE)),
        ("rock_aspect", ROCK_ASPECT),
        ("rock_lobes", ROCK_LOBES),
        ("rock_lobe_gain", ROCK_LOBE_GAIN),
        ("rock_shape_seed", ROCK_SHAPE_SEED),
        ("build_plan", " ".join(f"{kind}:{value:g}" for kind, value in BUILD_PLAN)),
        ("num_rocks", NUM_ROCKS),
        ("ground_rocks", GROUND_ROCKS),
        ("crest_rocks", NUM_ROCKS - GROUND_ROCKS),
        ("wall_slope", WALL_SLOPE),
        ("wall_stage_heights", " ".join(f"{h:g}" for h in WALL_STAGE_HEIGHTS)),
        ("wall_ridge_span_deg", WALL_RIDGE_SPAN_DEG),
        ("wall_course_rise", WALL_COURSE_RISE),
        ("wall_rock_density", WALL_ROCK_DENSITY),
        ("wall_min_sep", WALL_MIN_SEP),
        ("wall_seed", WALL_SEED),
        ("builder_station_deg", f"{BUILDER_STATION_DEG:.6f}"),
        ("builder_facing", BUILDER_FACING),
        # Where the arm actually ended up, which after a relocation drive is not
        # BUILD_STATION_DEG: this is the station the section just built is on.
        ("build_station_deg",
         f"{planner.station_of(arm_base_of(vehicle)):.6f}"),
        ("chassis_x", f"{chassis.x:.6f}"),
        ("chassis_y", f"{chassis.y:.6f}"),
        ("chassis_z", f"{chassis.z:.6f}"),
        ("chassis_e0", f"{rot.e0:.9f}"),
        ("chassis_e1", f"{rot.e1:.9f}"),
        ("chassis_e2", f"{rot.e2:.9f}"),
        ("chassis_e3", f"{rot.e3:.9f}"),
        ("chassis_fixed", int(PARK_CHASSIS_FIXED)),
        ("arm_scale", ARM_SCALE),
        ("arm_offset_x", ARM_OFFSET.x),
        ("arm_offset_y", ARM_OFFSET.y),
        ("arm_offset_z", ARM_OFFSET.z),
        ("arm_mount_yaw_deg", 180.0 if ARM_MOUNT_ROT is not None else 0.0),
        ("rock_mesh", ROCK_MESH),
        ("rock_scale", ROCK_SCALE),
        ("rock_density", ROCK_DENSITY),
        ("terrain_size", TERRAIN_SIZE),
        ("step_size", STEP_SIZE),
    ]
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["key", "value"])
        writer.writerows(rows)
    return path


def run(m113, vehicle, terrain, gripper, rocks, targets=(), vis=None,
        run_time=DEFAULT_RUN_TIME, frame_output_dir=None,
        plan=BUILD_PLAN, frame_stride=1, occupied_to_deg=None, section=1,
        exporter=None):
    """Hold the brakes; after the scene settles, work through `plan`.

    Once the rig has settled, `gripper.grasp(rock, targets[i])` is called every
    step for the current rock; when it returns True (rock released on its target)
    the loop advances to the next rock *and* the next target -- the gripper's state
    machine restarts automatically for the new one. The brakes stay on the whole
    time (belt-and-braces: PARK_CHASSIS_FIXED already pins the hull).

    `plan` drives everything. `("place", n)` is n pick-and-place cycles, `("wall",
    h)` one dump of stand-in rock to crest height h, and `("drive", deg)` a
    relocation to the next section (mode 2; see `OrbitDrive`). When the last rock
    of a phase is down the loop runs the plan on: dumps happen immediately, and the
    next place phase's targets are computed *then*, because they are aimed at the
    crest that now exists rather than at a planned point. Rocks laid on the crest
    are pinned as they touch down (PIN_TOP_ROCKS). The sim keeps running after the
    last phase, so a render window shows the finished wall.

    The plan's first place phase is the ground layer, and its targets are the
    planner's place points on the arc the builder can reach *from where it is
    standing then* -- which is the spawn pose in mode 1, and wherever the drive
    parked in mode 2. A `("drive", deg)` step also drops that section's load of
    rock beside the builder on arrival (`rocks` grows, hence it being returned),
    and re-centres the wall arc on the arm base it reached. `targets` is only for a
    caller that wants to override the ground layer's points.

    The loop stops at `run_time` seconds of sim time, or when the render window is
    closed, whichever comes first; `run_time=None` runs until the window is closed
    (the default when rendering, since a window means someone is watching). Ctrl-C
    stops it cleanly either way, so the caller can still dump the state it reached.

    `frame_stride` writes only every Nth rendered frame when capturing a PNG
    sequence -- at 30 rendered frames per simulated second, a several-minute demo
    is tens of thousands of images otherwise, and a movie of it wants to run faster
    than real time anyway.

    With `exporter` (a `postprocess.ChBlender` set up by the caller), a scene
    frame is written at 30 fps of sim time, independent of whether a render
    window is open.

    Returns `(placed, fake, targets, rocks)`: a per-rock flag for the ones the
    gripper actually laid (see GRASP_SKIP_AFTER for the ones it can give up on),
    the stand-in rock bodies of every dump grouped per course, the full target list
    including the crest phases' runtime ones, and the rocks themselves (which a
    relocation appends to).

    `occupied_to_deg` is how far along the orbit the sections already standing
    reach; it keeps the ground layer off them (see `ground_targets`).

    Tracked vehicles advance through the M113 wrapper's Synchronize/Advance (which
    steps the owned system), so no explicit DoStepDynamics is needed.
    """
    system = m113.GetSystem()
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_throttle = 0.0
    driver_inputs.m_steering = 0.0
    driver_inputs.m_braking = 1.0  # hold the vehicle still throughout

    render_steps = math.ceil((1.0 / 30) / STEP_SIZE)
    steps = 0
    idx = 0  # index of the rock currently being picked, and of its place point
    # Per rock: did its cycle finish? A rock the arm gave up on (GRASP_SKIP_AFTER)
    # leaves its index False, so this is not just a count of how far idx got.
    rocks = list(rocks)
    placed = [False] * len(rocks)
    frame_idx = 0   # PNGs written
    rendered = 0    # frames drawn (frame_stride of these get written)
    next_export_time = 0.0
    export_interval = 1.0 / 30  # seconds between exported Blender frames
    fake = []  # stand-in rocks, one list per course, over every dump
    held, released_at = False, None  # settle watch for the rock being laid on top
    attempt_t0 = None    # sim time the current rock's pick-and-place cycle began
    replanned = False    # ...and whether it has already been re-aimed once
    # Each crest phase's targets are appended once its wall exists, so work on a
    # copy rather than growing the caller's (module-level) ground-phase list.
    targets = list(targets)
    station = BUILD_STATION_DEG   # arc the builder is working; a drive moves it
    stacker = wall_stacker(station)  # the wall the plan's dumps grow, empty for now
    step_idx = 0
    drive = None         # the relocation in progress, while there is one
    grasp_after = T_GRASP_START  # sim time the arm may start work

    def advance_plan():
        """Run plan steps until the next place phase has targets, or a drive starts.

        Dumps and relocations happen here; a place phase ends it, because the loop
        then has rocks to lay. A drive also ends it, and the loop calls back in
        once the builder has parked and its new arc is known.
        """
        nonlocal step_idx, drive, targets
        while step_idx < len(plan):
            kind, value = plan[step_idx]
            step_idx += 1
            if kind == "drive":
                print(f"\n  phase complete -- moving {value:.1f} deg along the "
                      f"orbit to the next section")
                drive = OrbitDrive(vehicle, step_deg=value)
                return
            if kind == "wall":
                print(f"\n  phase complete -- dumping rock to a {value:.2f} m crest")
                if idx == GROUND_ROCKS:
                    # What the arm actually laid, for comparison: the wall is
                    # prescribed on the orbit, so this does not shape it either way.
                    print(measure_built_belt(rocks[:GROUND_ROCKS]).describe())
                _, layers = build_wall(system, stacker, heights=(value,), vis=vis,
                                       section=section, exporter=exporter)
                fake.extend(layers)
                continue
            if stacker.stage == 0:
                if targets:
                    return   # the caller supplied the ground layer's points
                # The ground layer: points on the arc the builder can reach from
                # where it is standing now, clear of any section already built.
                points = ground_targets(station, occupied_to_deg, count=value)
                targets.extend(points)
                first, last = (planner.station_of(points[0]),
                               planner.station_of(points[-1]))
                print(f"  ground layer  : {value} point(s) over "
                      f"{first:.2f}..{last:.2f} deg, on the arc at station "
                      f"{station:.2f} deg\n")
            else:
                surface = wall_top_z(stacker.profile, fake)
                targets.extend(top_rock_targets(stacker.profile, surface, value))
                print(f"  crest surface: z={surface:.2f} m -- {value} rock(s) to "
                      f"lay on it, released at z={targets[-1].z:.2f} m\n")
            return

    def finish_drive(time):
        """Park the builder, re-centre its arc, and drop its load of rock beside it."""
        nonlocal drive, station, stacker, grasp_after
        base = drive.park()
        print(f"\n{drive.describe()}")
        # Both the wall arc and the place points hang off the arm base the drive
        # actually reached, not the one it aimed at, so the geometry the arm has to
        # reach is the same as the first section's however the drive went.
        station = planner.station_of(base)
        stacker = wall_stacker(station)
        drive = None
        new_rocks = spawn_wedge_rocks(system, gripper, terrain, base,
                                      vehicle.GetChassisBody().GetRot(),
                                      count=NUM_ROCKS, first=len(rocks), vis=vis,
                                      section=section, exporter=exporter)
        rocks.extend(new_rocks)
        placed.extend([False] * len(new_rocks))
        # Same points `advance_plan` is about to aim at -- clipped clear of the
        # sections already standing, so the markers show where the rocks go rather
        # than where an unobstructed arc would have put them.
        points = ground_targets(station, occupied_to_deg)
        add_place_markers(system, points, name=f"place_markers_{station:.0f}",
                          vis=vis, exporter=exporter)
        print(f"  dropped {len(new_rocks)} rocks beside the builder; wall arc "
              f"re-centred on station {station:.2f} deg")
        if occupied_to_deg is not None:
            print(f"  ground layer clear of the standing wall, which reaches "
                  f"{occupied_to_deg:.2f} deg")
        for i, point in enumerate(points):
            print(f"    wall point {i}: station {planner.station_of(point):.2f} "
                  f"deg, {(point - base).Length():.2f} m from the arm base")
        grasp_after = time + T_GRASP_START   # let the rig and the new rocks settle
        advance_plan()

    if vis is not None:
        vehicle.EnableRealtime(True)
    if frame_output_dir:
        os.makedirs(frame_output_dir, exist_ok=True)
    advance_plan()

    try:
        while True:
            if vis is not None and not vis.Run():
                break
            time = system.GetChTime()
            if run_time is not None and time >= run_time:
                break

            if vis is not None and steps % render_steps == 0:
                vis.BeginScene()
                vis.Render()
                if frame_output_dir and rendered % frame_stride == 0:
                    frame_path = os.path.join(frame_output_dir,
                                              f"frame_{frame_idx:06d}.png")
                    vis.WriteImageToFile(frame_path)
                    frame_idx += 1
                rendered += 1
                vis.EndScene()

            if exporter is not None and time >= next_export_time:
                exporter.ExportData()
                next_export_time += export_interval

            # ---- relocating to the next section, or parked and building ----
            # `step` hands back path-follower inputs until the builder has reached
            # its target, braked and settled; then None, and the arrival work runs
            # (park the hull, re-centre the arc, drop the rock). Everything below
            # is skipped meanwhile -- with no rocks yet, it has nothing to do.
            if drive is not None:
                inputs = drive.step(time)
                if inputs is None:
                    finish_drive(time)
                else:
                    driver_inputs = inputs
            if drive is None:
                # ---- hold the brakes; lay each rock on its target once settled ----
                driver_inputs.m_throttle = 0.0
                driver_inputs.m_steering = 0.0
                driver_inputs.m_braking = 1.0
            # ---- pin a crest rock the moment it is down on the wall ----
            # The gripper holds the rock locked and its collision off, so the step
            # the lock goes away is the step it was released; from there it is a
            # free body landing on the crest. See PIN_TOP_ROCKS for why it cannot
            # wait until the cycle ends.
            if PIN_TOP_ROCKS and idx < len(rocks) and ROCK_PHASE[idx] != "ground":
                rock = rocks[idx]
                if gripper.cur_lock and gripper.cur_object == rock.GetName():
                    held = True
                elif held and not rock.IsFixed():
                    if released_at is None:
                        released_at = time
                    pos, aim = rock.GetPos(), targets[idx]
                    drift = math.hypot(pos.x - aim.x, pos.y - aim.y)
                    if (rock.GetPosDt().Length() < TOP_SETTLE_SPEED
                            or drift > TOP_SETTLE_DRIFT
                            or time - released_at > TOP_SETTLE_TIMEOUT):
                        rock.SetFixed(True)
                        print(f"  -> rock {idx} settled on the wall at "
                              f"z={pos.z:.2f} m ({drift:.2f} m from its aim "
                              f"point), {time - released_at:.2f} s after release "
                              f"-- pinned")

            if (drive is None and time > grasp_after
                    and idx < min(len(rocks), len(targets))):
                if attempt_t0 is None:
                    attempt_t0 = time
                done = gripper.grasp(rocks[idx], targets[idx])
                stuck = time - attempt_t0
                # A cycle that has stalled gets one re-aim, then is given up on --
                # see GRASP_REPLAN_AFTER.
                if not done and not replanned and stuck > GRASP_REPLAN_AFTER:
                    print(f"  -> t={time:6.1f} s: rock {idx} not picked up after "
                          f"{stuck:.1f} s -- re-aiming at where it is now")
                    gripper.replan_grasp()
                    replanned = True
                if done or stuck > GRASP_SKIP_AFTER:
                    rock = rocks[idx]
                    r, theta = planner.to_polar(rock.GetPos())
                    where = (f"wall point {idx}" if ROCK_PHASE[idx] == "ground"
                             else f"the crest ({ROCK_PHASE[idx]})")
                    placed[idx] = done
                    if done:
                        print(f"  -> t={time:6.1f} s: rock {idx} placed on {where} "
                              f"(r={r:.2f} m, theta={theta:.2f} deg, "
                              f"z={rock.GetPos().z:.2f} m)")
                    else:
                        print(f"  -> t={time:6.1f} s: rock {idx} ABANDONED after "
                              f"{stuck:.1f} s -- left at r={r:.2f} m, "
                              f"theta={theta:.2f} deg; {where} stays empty")
                    idx += 1  # next rock, next target (grasp() restarts on the new one)
                    held, released_at = False, None  # reset the settle watch
                    attempt_t0, replanned = None, False
                    if idx == len(targets):
                        advance_plan()   # dumps, relocations, then the next phase

            # Keep the vehicle, terrain, and visual modules in sync, then advance.
            terrain.Synchronize(time)
            m113.Synchronize(time, driver_inputs)
            if vis is not None:
                vis.Synchronize(time, driver_inputs)
            terrain.Advance(STEP_SIZE)
            m113.Advance(STEP_SIZE)
            if drive is not None:
                drive.advance(STEP_SIZE)
            if vis is not None:
                vis.Advance(STEP_SIZE)
            steps += 1
    except KeyboardInterrupt:
        print("\n  interrupted -- stopping the run "
              f"after {sum(placed)}/{len(rocks)} rocks")
    return placed, fake, targets, rocks


def main():
    headless = "--headless" in sys.argv
    use_vsg = "--vsg" in sys.argv  # render with VSG (Vulkan) instead of Irrlicht
    plan_only = "--plan-only" in sys.argv
    frame_output_dir = parse_frame_output_dir(sys.argv)
    export_blender = "--export-blender" in sys.argv
    state_dir = parse_state_dir(sys.argv)
    # `--no-wall` drops the dumps and leaves the pick-and-place phases, which is
    # the quick way to check the arm's half of the plan on its own.
    plan = (tuple(step for step in BUILD_PLAN if step[0] != "wall")
            if "--no-wall" in sys.argv else BUILD_PLAN)
    frame_stride = parse_number_option(sys.argv, "--frame-stride", 1)
    # Mode 2: import a finished section and build the next one onto it. The plan
    # gains a relocation at the front, which is what makes `run` drive the builder
    # along its orbit and drop it a fresh load of rock when it gets there.
    resume_file = parse_resume_file(sys.argv)
    if resume_file:
        plan = (("drive", SECTION_STEP_DEG),) + tuple(plan)
    # Both headless and windowed runs stop at a fixed sim time, which is what lets
    # an unattended movie run terminate on its own with the wall finished.
    # `--run-time 0` means no limit: keep going until the window is closed.
    run_time = parse_number_option(
        sys.argv, "--run-time",
        CONTINUE_RUN_TIME if resume_file else DEFAULT_RUN_TIME, cast=float)
    if run_time is not None and run_time <= 0.0:
        run_time = None
    title = ("M113 builder on the wall orbit: "
             + " then ".join(f"{value} rocks" if kind == "place"
                             else (f"a {value:.1f} m wall" if kind == "wall"
                                   else f"{value:.0f} deg along the orbit")
                             for kind, value in plan))
    print(f"=== {title} (headless smoke test) ===" if headless else f"=== {title} ===")
    print(planner.describe(BUILD_STATION_DEG))
    print(f"  builder       : station {BUILDER_STATION_DEG:.1f} deg -> "
          f"({INIT_LOC.x:.2f}, {INIT_LOC.y:.2f}), heading "
          f"{planner.heading_deg(BUILDER_STATION_DEG, BUILDER_FACING):.1f} deg "
          f"({BUILDER_FACING})")
    print(f"  arm base      : ({ARM_BASE.x:.2f}, {ARM_BASE.y:.2f}, {ARM_BASE.z:.2f}), "
          f"r={planner.to_polar(ARM_BASE)[0]:.2f} m, station {BUILD_STATION_DEG:.2f} deg")

    # Mode 2 reads the whole state file up front -- before anything is spawned and
    # long before the run rewrites it, which is what makes reading and writing the
    # same path safe.
    carried = read_rock_rows(resume_file) if resume_file else []
    section = max((row_section(row) for row in carried), default=0) + 1
    occupied_to_deg = occupied_arc_end(carried)
    if resume_file:
        sections = sorted({row_section(row) for row in carried})
        print(f"  continuing    : {len(carried)} rocks from {resume_file} "
              f"(section(s) {', '.join(str(s) for s in sections)}) -> "
              f"building section {section}")
        print(f"  standing wall : reaches station {occupied_to_deg:.2f} deg; the "
              f"new ground layer starts {SECTION_CLEARANCE:.2f} m past it")

    if plan_only:
        # Geometry only, phase by phase: how far the arm has to reach for each
        # planned drop point (it works well at ~2-4 m), and the section each dump
        # would stand in. Nothing here needs a simulation -- the wall is
        # prescribed, so all of it is known before the run starts. The crest
        # phases' drop points are the exception: they are aimed at whatever surface
        # the dump before them produced, so they are estimated here from the
        # profile's crest rather than from the rocks that will make it.
        # A relocation shifts everything after it by that much arc; the drive's own
        # error is not known without simulating, so this is the nominal geometry.
        station = BUILD_STATION_DEG + sum(value for kind, value in plan
                                          if kind == "drive")
        arm_base = (ARM_BASE if station == BUILD_STATION_DEG
                    else planner.to_world(planner.to_polar(ARM_BASE)[0], station,
                                          ARM_BASE.z))
        stacker, idx = wall_stacker(station), 0
        for kind, value in plan:
            if kind == "drive":
                print(f"\n  relocate {value:.1f} deg along the orbit -> the next "
                      f"section's arc is centred on station {station:.2f} deg")
                continue
            if kind == "wall":
                profile, rows = stacker.plan(height=value)
                print(f"\n  dump {stacker.stage + 1} -> "
                      f"{2 * profile.half_width:.2f} m base, "
                      f"{profile.height:.2f} m high, "
                      f"{sum(n for _, _, _, n in rows)} rocks:")
                for z, half_w, half_l, count in rows:
                    print(f"    course z={z:.2f}: {2 * half_w:.2f} m wide x "
                          f"{2 * half_l:.2f} m long, +{count} rocks")
                stacker.grow(height=value)
                continue
            print()
            if stacker.stage == 0:
                points = ground_targets(station, occupied_to_deg)
            else:
                points = top_rock_targets(stacker.profile,
                                          stacker.profile.crest_z
                                          + ROCK_HEIGHT / 2.0, value)
            for point in points[:value]:
                where = ("wall point" if stacker.stage == 0
                         else f"crest {stacker.stage}")
                print(f"  rock {idx:2d} -> {where} (station "
                      f"{planner.station_of(point):.2f} deg, z={point.z:.2f}): "
                      f"{(point - arm_base).Length():.2f} m from the arm base")
                idx += 1
        print()
        print(stacker.describe())
        return

    m113, vehicle, terrain, gripper, rocks = build_scene(
        spawn_rocks=not resume_file, scenery_rows=carried, section=section)
    center = vehicle.GetChassis().GetPos()
    print(f"  arm scale     : {ARM_SCALE}")
    print(f"  chassis center: {center}")
    print(f"  arm base      : {gripper.base.GetPos()}")
    for rock in rocks:
        rel = rock.GetPos() - gripper.base.GetPos()
        print(f"  {rock.GetName()} start: {rock.GetPos()} "
              f"(d={math.hypot(rel.x, rel.y):.2f} m from the arm base)")

    if frame_output_dir and (headless or use_vsg):
        print("  frame capture : disabled (--save-frames requires Irrlicht rendering)")
        frame_output_dir = None
    elif frame_output_dir:
        print(f"  frame capture : {frame_output_dir} (every {frame_stride} frame(s) "
              f"of 30/s -> {30.0 / frame_stride:.1f} per simulated second)")
    if run_time is not None:
        print(f"  run time      : {run_time:.0f} s of sim")
    print(f"  state CSVs    : {state_dir or 'disabled (--no-state)'}")

    exporter = None
    if export_blender:
        out_dir = chrono.GetChronoOutputPath() + "BLENDER"
        if os.path.exists(out_dir):
            shutil.rmtree(out_dir)
        os.makedirs(out_dir)
        exporter = postprocess.ChBlender(m113.GetSystem())
        exporter.SetBasePath(out_dir)
        exporter.SetBlenderUp_is_ChronoZ()
        exporter.AddAll()
        exporter.ExportScript()
        print(f"  blender export: {out_dir} (30 frames/s of sim time)")

    vis = None if headless else make_vis(vehicle, title, use_vsg=use_vsg)
    placed, fake, targets = [False] * len(rocks), [], []
    try:
        placed, fake, targets, rocks = run(m113, vehicle, terrain, gripper, rocks,
                                           vis=vis, run_time=run_time,
                                           frame_output_dir=frame_output_dir,
                                           plan=plan, frame_stride=frame_stride,
                                           occupied_to_deg=occupied_to_deg,
                                           section=section, exporter=exporter)
    finally:
        # Dump the end state even if the run was cut short, so a partial build is
        # still resumable (and still tells you where every rock ended up).
        sim_time = m113.GetSystem().GetChTime()
        print(f"\n  {sum(placed)}/{len(rocks)} rocks placed in "
              f"{sim_time:.1f} s of sim time")
        for i, rock in enumerate(rocks):
            pos = rock.GetPos()
            r, theta = planner.to_polar(pos)
            where = (f"wall point {i}" if ROCK_PHASE[i] == "ground"
                     else f"the crest ({ROCK_PHASE[i]})")
            err = (f"{math.hypot(pos.x - targets[i].x, pos.y - targets[i].y):.2f} m "
                   "(horizontal) from " if i < len(targets) else "aimed at ")
            print(f"  {rock.GetName()} end  : r={r:.2f} m, theta={theta:.2f} deg, "
                  f"z={pos.z:.2f} m -> {err}{where}")
        if state_dir:
            stamp = datetime.datetime.now().isoformat(timespec="seconds")
            os.makedirs(state_dir, exist_ok=True)
            rock_csv = write_rock_state(os.path.join(state_dir, ROCK_STATE_FILE),
                                        rocks, targets, placed, sim_time,
                                        stamp, fake=fake, carried=carried,
                                        section=section)
            plan_csv = write_site_plan(os.path.join(state_dir, SITE_PLAN_FILE),
                                       vehicle, sim_time, stamp, placed,
                                       section=section)
            print(f"  rock state    : {rock_csv} "
                  f"({len(carried)} carried + "
                  f"{len(rocks) + sum(len(c) for c in fake)} new)")
            print(f"  site plan     : {plan_csv}")
        if exporter is not None:
            # Mirror the exported scene into a "latest" folder alongside the
            # user's own hand-curated builderN archive under the same Robot/
            # dir, so opening Blender's import browser doesn't mean hunting
            # through the Chrono output dir first -- named "latest" (not
            # "Robot" itself) so this can never rmtree the builderN folders.
            blender_copy_dir = os.path.expanduser(
                "~/Documents/BlenderDocuments/chronoImports/Robot/latest")
            if os.path.exists(blender_copy_dir):
                shutil.rmtree(blender_copy_dir)
            shutil.copytree(out_dir, blender_copy_dir)
            print(f"  blender copy  : {blender_copy_dir}")


if __name__ == "__main__":
    main()
