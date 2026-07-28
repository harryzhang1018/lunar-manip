"""M113 builder on a circular wall-building orbit -- orbit-planned pick-and-place.

Same rig as `TrackedVeh_Builder.py` (M113 tracked vehicle, LRV gripper arm welded
to the hull, SMC contact, brakes held) dropped into the global site layout from
the concept sketch: concentric orbits about a single center, with the wall being
built on the inner ring, the builders on the middle ring, and the fetchers
unloading rocks on the outer ring.

What is different from `TrackedVeh_Builder.py`:
  * `orbit_planning.OrbitPlanner` owns the site geometry and hands the builder its
    work: the start pose on the vehicle orbit, and NUM_ROCKS discrete points on
    the wall orbit. Those points replace the single fixed `PLACE_POINT` -- rock i
    is dropped on wall point i, so the rocks lay out along the wall arc instead of
    piling up at one spot.
  * The builder starts on the vehicle orbit headed *along* it (station 90 deg ->
    (0, 33) headed -X, the CCW direction of travel), so the wall arc it is
    building runs alongside it on its left and the rocks sit alongside on its
    right, on the outer unloading side -- the sketch's layout. At station 0 deg,
    (33, 0), that same tangent heading is +Y.
  * 23 rocks instead of 5, all spawned dynamic and colliding, in a wider spawn
    wedge (they have to fit without overlapping). The first WALL_ROCKS of them go
    two per wall place point, laid as COURSES successive passes
    over the arc: rock i goes on wall point i % NUM_PLACE_POINTS, and each pass
    after the first is released COURSE_RISE higher so the gripper clears the rock
    already sitting there. "Course" is the build order, not the result: measured
    over a full 20-rock run, every second-course rock rolled off the first-course
    rock it was set on and settled on the ground beside it, a median 0.51 m from
    its wall point (first course: 0.09 m). These are irregular ~0.2 m lumps
    released onto another lump, so the arc ends up two rocks wide rather than two
    rocks tall. Stacking would need the rocks seated, not dropped.
  * The wall and vehicle orbits and the assigned place points are drawn as
    visual-only markers so the plan is visible in the render window.
  * The hull is pinned for the parked build (PARK_CHASSIS_FIXED), because a braked
    M113 only holds still at heading 0 -- see that constant for the measurements.
  * The gripper builds the ground layer only. FAKE_LAYERS more layers are then
    stacked on top of it as stand-in rocks (`add_fake_layers`) -- sampled over the
    same (r, theta) belt the built rocks formed, FAKE_LAYER_RISE apart in z, each
    fixed and collision-free. A layer costs ~14 s of sim per rock to lay for real
    and nothing to fake, and nothing in this scenario interacts with the upper
    courses anyway. They are packed several times denser than the built layer so
    the stack reads as a solid wall instead of a see-through scatter; see the
    FAKE_* constants.
  * The last TOP_ROCKS rocks are then laid on top of that wall, for real: the
    gripper picks them out of the spawn wedge and sets them down on the finished
    crest. Their place points are worked out at run time from the wall that got
    built (`top_rock_targets`), and the wall's top course keeps its collision so
    they land on it instead of falling through.
  * At the end of the run every rock's final pose is written to CSV (see
    `write_rock_state`), so a later run can respawn the site as this one left it.

The builder stays parked for the whole run, as in
`TrackedVeh_Builder.py`. That is what caps the arc one builder can cover: the arm
reaches ~2-4 m and 1 deg of the 30 m wall orbit is 0.52 m, so PLACE_HALF_SPAN_DEG
is 5 deg (a ~5.2 m arc), not the sketch's 15 deg. Covering a wider arc needs the
builder to drive along its orbit between place points, which this scenario does
not do yet -- the planner already supports it (just raise the half span).

Run with the project's conda env:

    conda run -n chrono python scenarios/TrackedVeh_OrbitBuilder.py

Add `--headless` to step the simulation without opening a render window (used for
smoke tests). Add `--vsg` to render with the VSG (Vulkan) backend instead of the
default Irrlicht (OpenGL) one. Add `--save-frames [dir]` to dump each rendered
Irrlicht frame as a PNG sequence. Add `--plan-only` to print the orbit plan (and
the arm's reach to each rock's drop point) and exit without simulating. Add
`--state-dir DIR` to write the end-of-run state CSVs somewhere other than
DEFAULT_STATE_DIR, or `--no-state` to skip writing them. Add `--fake-layers N` to
stack a different number of stand-in layers on the built one (0 = none).

To record the demo as a movie, render it with a frame limit and a stride, then
encode the PNGs -- 4 here means the 30 frames/s the renderer draws are written at
7.5/s, so playing them back at 30 fps runs the demo at 4x:

    conda run -n chrono python scenarios/TrackedVeh_OrbitBuilder.py \\
        --save-frames artifacts/frames/demo --frame-stride 4 --run-time 400
    ffmpeg -framerate 30 -i artifacts/frames/demo/frame_%06d.png -pix_fmt yuv420p out.mp4
"""

import os
import sys
import csv
import math
import random
import datetime

import pychrono as chrono
import pychrono.vehicle as veh

# Make the repo root (for the `model` package) and the scenarios dir (for the
# sibling scenario modules reused below) importable regardless of the CWD.
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_root)
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Reuse the grasp state machine, the SMC finger-pad rebuild, and the rock recipe
# from the trailer/arm scenarios so the pick-and-place behaves identically here.
from LRV_Trailer import TrailerArm, match_gripper_contact_material
from LRV_Arm import place_rock, ROCK_MESH, ROCK_SCALE, ROCK_DENSITY
from orbit_planning import OrbitPlanner

# ---- site layout (see orbit_planning.py) ----
ORBIT_CENTER = (0.0, 0.0)
WALL_RADIUS = 30.0        # the wall being built; place points land here
VEHICLE_RADIUS = 33.0     # the ring the builder's hull rides
BUILDER_STATION_DEG = 90.0  # builder 1 -- the only one for now -- on the +Y side
# Angular half-width of the wall arc this (parked) builder is assigned. See the
# module docstring: 5 deg is what the arm can reach without driving.
PLACE_HALF_SPAN_DEG = 5.0
PLACE_HEIGHT = 0.35       # release height above the ground over a wall point

# Build plan: NUM_PLACE_POINTS points on the wall arc, covered COURSES times over
# (rock i is dropped on point i % NUM_PLACE_POINTS), so the builder walks the arc
# end to end once per course. Each course after the first is released COURSE_RISE
# higher than the one below: the rocks are ~0.26 m tall, so releasing the second
# course at the first course's height would drive the open fingers straight into
# the rock already sitting there. The rise buys that clearance; it does not make
# the rocks stack, since a dropped lump rolls off the lump below (see the module
# docstring for the measured spread).
NUM_PLACE_POINTS = 10
COURSES = 2
COURSE_RISE = 0.26        # m per course -- one rock height (see LRV_Arm.ROCK_SCALE)

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
WALL_ROCKS = NUM_PLACE_POINTS * COURSES   # one rock per place point per course
TOP_ROCKS = 3                             # ...plus these, laid on the finished wall
NUM_ROCKS = WALL_ROCKS + TOP_ROCKS
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

# Headless smoke-test duration (seconds of sim time): long enough for NUM_ROCKS
# pick-and-place cycles (~15 s each, worst case) plus settling. The tracked sim
# runs well below real time, so this is slow in wall-clock. With a render window
# the scene instead runs until the window is closed.
HEADLESS_RUN_TIME = 400.0
DEFAULT_FRAME_DIR = os.path.join(project_root, "artifacts", "frames", "trackedveh_orbitbuilder")

# Where the end-of-run state CSVs land (see write_rock_state / write_site_plan).
# Both files are rewritten in place by every run, so a resume always reads the
# latest; the run timestamp is recorded inside them.
DEFAULT_STATE_DIR = os.path.join(project_root, "artifacts", "states", "trackedveh_orbitbuilder")
ROCK_STATE_FILE = "rock_state.csv"
SITE_PLAN_FILE = "site_plan.csv"

# Layers of stand-in rocks laid on top of the built one once the gripper is done
# (`--fake-layers N` overrides). The builder lays the ground layer for real -- all
# NUM_ROCKS rocks end up there, since a rock dropped on a rock rolls off -- and
# these are the courses above it: same (r, theta) distribution as the measured
# belt, each spawned fixed with collision disabled. They are scenery. Nothing
# touches them, they hold no load, and they would fall through the layer below if
# they were live, because there is nothing solid under most of them. What they buy
# is the ~14 s of sim per rock per layer that laying them with the gripper would
# cost. Set to 0 to build only what the arm places.
#
# The numbers below are tuned to read as a wall rather than as a scatter, which
# means deliberately ignoring what physics would produce. Reproducing the built
# layer's own density -- NUM_ROCKS rocks, one rock height (0.26 m) apart -- leaves
# each layer covering only ~10% of the belt's 8 m^2, so the stack is mostly air
# and you see straight through it. Instead: FAKE_ROCKS_PER_LAYER is several times
# the built layer's count, and FAKE_LAYER_RISE is half a rock height, so each
# layer overlaps the one below and the two knit together. Together they put ~4x
# the rock into the same envelope, at the cost of the layers interpenetrating --
# which is free here, since none of these bodies collide.
FAKE_LAYERS = 6           # x FAKE_LAYER_RISE = ~0.6 m of wall over the ground layer
FAKE_LAYER_RISE = 0.10    # m between layers: well under a rock, so layers overlap
FAKE_ROCKS_PER_LAYER = 90  # vs the 20 the gripper actually laid
FAKE_MIN_SEP = 0.08       # m; under the rock width, so rocks knit rather than tile
FAKE_LAYER_SEED = 0       # fixed so the same run gives the same scatter
# Radial band the stand-in layers are scattered over, as a half-width (m) about
# the belt's median radius. The measured belt is ~1.3 m wide -- twice the stack's
# height -- because a few rocks bounced well off the wall line, and a stack that
# broad reads as a rubble bank, not a wall. Narrowing it concentrates the same
# rock into a wall-shaped cross-section. None = use the belt's full width.
FAKE_HALF_WIDTH = 0.35

# ---- the TOP_ROCKS rocks laid on top of the finished wall ----
# These are real rocks: spawned dynamic and colliding like the other 23, picked up
# by the gripper, and carried onto the stand-in wall once it is standing. For them
# to land on it rather than drop through it, the stand-in wall is spawned colliding
# (FAKE_WALL_COLLIDES) -- fixed bodies with a collision model are just static
# geometry. The alternative -- releasing over the wall and pinning the rock mid-air
# the instant the gripper opens -- needs no collision at all, but the rock then
# hangs wherever the release happened instead of settling into the rubble.
#
# The wall is only solid to things that are not part of it: collision families put
# every stand-in rock in FAMILY_FAKE_WALL and bar that family from colliding with
# itself or with the built ground layer. Both exclusions are load-bearing.
#   * Internally: the layers are packed to overlap on purpose (FAKE_MIN_SEP is
#     under a rock width), so ~540 mutually interpenetrating sphere-swept meshes
#     would be a narrowphase test each, every step, for contacts the solver then
#     throws away because both bodies are fixed. Measured at 10x slower with just
#     the top course colliding.
#   * Against the built layer: the lowest stand-in course sits ~0.16 m into the
#     rocks the gripper laid, which are live bodies. Under SMC that overlap is a
#     penalty force with nothing to balance it, and it would fire the built layer
#     across the site the moment the wall appeared.
FAKE_WALL_COLLIDES = True
FAMILY_BUILT_ROCKS = 2   # the WALL_ROCKS rocks the gripper lays
FAMILY_FAKE_WALL = 3     # every stand-in rock
# Where along the measured arc the top rocks go, as fractions of its span. Kept
# off the ends: the arc's midpoint is ~3.2 m from the arm base and its ends ~4.2 m,
# and these points sit ~0.7 m higher than the wall course, so the middle is where
# the reach is most comfortable.
TOP_ROCK_ARC_FRACS = (0.3, 0.5, 0.7)
# Gripper release height above the wall's top surface (m). A rock hangs ~0.12 m
# below the gripper centre and its own centre sits ~0.10 m above whatever it rests
# on, so 0.22 m would have it touching down as the fingers open; 0.23 leaves it a
# centimetre to fall. Every centimetre of that drop is bounce, and bounce on a
# crest only ~2x a rock wide is what sends a rock over the side.
TOP_PLACE_CLEARANCE = 0.23
# Pin each top rock as soon as it has come to rest on the wall: once the gripper
# has let go, the rock is frozen the first step it is slower than TOP_SETTLE_SPEED,
# or after TOP_SETTLE_TIMEOUT seconds regardless.
#
# Timing is the whole point. Waiting for the pick-and-place cycle to end pins it
# ~5 s after release, and a rock does not sit still on a rubble crest for 5 s: in
# testing the middle rock rolled off the outer face every time, ending up on the
# wall's shoulder in one run and on the ground 1.3 m clear of the wall in the next.
# It is not released badly -- all three reach their aim point and are set down
# below the speed tolerance -- it just will not stay on a 0.7 m wide crest. Pinning
# at touchdown keeps the landing real and the result repeatable. Set False to leave
# the top rocks live and let them roll where they will.
PIN_TOP_ROCKS = True
TOP_SETTLE_SPEED = 0.05    # m/s below which the rock counts as come to rest
TOP_SETTLE_TIMEOUT = 1.5   # s after release, pin it regardless


def rock_targets(place_points):
    """Drop point for each of the NUM_ROCKS rocks: COURSES passes over the arc.

    Rock i goes on wall point i % NUM_PLACE_POINTS, so the builder lays the whole
    arc, then lays it again. Course k is released k * COURSE_RISE above the wall
    point, i.e. on top of the course below (see COURSE_RISE).
    """
    return [chrono.ChVector3d(p.x, p.y, p.z + course * COURSE_RISE)
            for course in range(COURSES) for p in place_points]


ROCK_TARGETS = rock_targets(PLACE_POINTS)


def measure_built_belt(rocks):
    """Measure the belt `rocks` occupy, straight off the live bodies.

    `OrbitPlanner.measure_belt` reads the rows of a `rock_state.csv`; this hands it
    the same fields taken from bodies in the running system, so the belt can be
    measured mid-run without a round trip through the file.
    """
    rows = [{"placed": 1, "com_x": r.GetPos().x, "com_y": r.GetPos().y,
             "com_z": r.GetPos().z} for r in rocks]
    return planner.measure_belt(rows, rock_height=COURSE_RISE)


def wall_top_z(belt, fake=()):
    """Height (m) of the top surface of the finished wall.

    The stand-in layers if there are any, else the built belt on its own. Either
    way it is the highest rock centre plus half a rock, which is the surface the
    top rocks are laid on.
    """
    if not fake:
        return belt.top_z
    highest = max(rock.GetPos().z for layer in fake for rock in layer)
    return highest + COURSE_RISE / 2.0


def top_rock_targets(belt, surface_z, num=TOP_ROCKS):
    """Release points for the top rocks: on the wall's centerline, along its arc.

    `num` points spread across the measured arc at TOP_ROCK_ARC_FRACS, on the
    belt's median radius (its centerline, and the thickest part of the wall), a
    TOP_PLACE_CLEARANCE drop above `surface_z`.
    """
    span = belt.theta_max_deg - belt.theta_min_deg
    return [planner.to_world(belt.r_med, belt.theta_min_deg + frac * span,
                             surface_z + TOP_PLACE_CLEARANCE)
            for frac in TOP_ROCK_ARC_FRACS[:num]]


def add_fake_layers(system, rocks, num_layers=FAKE_LAYERS, seed=FAKE_LAYER_SEED,
                    rocks_per_layer=FAKE_ROCKS_PER_LAYER, rise=FAKE_LAYER_RISE,
                    min_sep=FAKE_MIN_SEP, half_width=FAKE_HALF_WIDTH,
                    collides=FAKE_WALL_COLLIDES, vis=None):
    """Stack `num_layers` of stand-in rocks on the layer `rocks` built.

    Measures the belt the built rocks occupy (`measure_built_belt`), samples that
    many more layers over the same ring (`OrbitPlanner.layer_poses`), and spawns a
    rock body at each pose. Returns `(belt, layers)` -- the measurement, and the
    new bodies grouped per layer.

    Every one of them is `SetFixed(True)`, which is what makes the shortcut safe as
    well as cheap: pinned bodies are not integrated, so they cannot sag, drift or
    explode. With `collides` they are solid to anything that is not part of the
    wall -- so a rock laid on the crest lands on it, and one that rolls off tumbles
    down the face instead of dropping through -- while collision families keep the
    wall transparent to itself and to the built layer standing in it. See
    FAKE_WALL_COLLIDES for why both of those exclusions matter.

    `place_rock` beds a rock on a ground plane, so each layer's plane is passed as
    `ground_z` half a rock below the layer's centre height, and the body is then
    shifted so its centre of mass lands exactly on the sampled height. That last
    step matters: the rock mesh's centre of mass sits ~0.09 m above its base, not
    at half its 0.26 m height, so bedding alone would put each layer ~4 cm low and
    leave the stack's spacing short of one rock. Measuring centre to centre, as
    `measure_belt` does, the layers come out exactly `rise` apart. The pose's
    rotation is applied about that same centre of mass, so the rock spins in place
    instead of swinging away from the point it was sampled at.

    With a render window already up, pass `vis` so each new body gets bound to it
    -- Irrlicht builds its scene nodes when the visual system binds, so a body
    added mid-run is invisible until it is bound.
    """
    belt = measure_built_belt(rocks)
    print(belt.describe())
    if num_layers <= 0:
        return belt, []

    r_range = None if half_width is None else (belt.r_med - half_width,
                                               belt.r_med + half_width)
    layers = []
    layer_poses = planner.layer_poses(belt, num_layers, seed=seed,
                                      rocks_per_layer=rocks_per_layer,
                                      rise=rise, min_sep=min_sep, r_range=r_range)
    for k, poses in enumerate(layer_poses, start=1):
        layer = []
        for i, (pos, rot) in enumerate(poses):
            rock = place_rock(system, pos, ground_z=pos.z - COURSE_RISE / 2.0,
                              contact_method=chrono.ChContactMethod_SMC)
            rock.SetName(f"fake_{k}_{i}")
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
            rock.EnableCollision(collides)
            if vis is not None:
                # Bind just this body, not the whole scene: BindAll would walk
                # every item already bound (hull, road wheels, all 100+ track
                # shoes) as well.
                vis.BindItem(rock)
            layer.append(rock)
        layers.append(layer)
        print(f"  fake layer {k}: {len(layer)} rocks at z={poses[0][0].z:.2f} m")

    if collides:
        # Register the new bodies with the collision system. Adding a body to a
        # ChSystem that has already been stepped does NOT put its collision model
        # in the collision system -- the body simulates, renders and reports
        # contacts of zero, and anything aimed at it passes straight through. This
        # wall is built ~290 s into the run, so without this the whole course is
        # phantom: measured, the rocks laid on top of it fell through it and landed
        # on the ground. `ChSystem::Setup()` does not do it; only re-binding the
        # collision system does.
        system.GetCollisionSystem().BindAll()

    return belt, layers


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


def add_orbit_visuals(system, place_points):
    """Draw the site plan: the wall and vehicle orbits, and the assigned points.

    Everything hangs off one fixed, collision-free body, so it is render-only and
    never touches the dynamics: the two orbit rings as chains of thin boxes just
    above the ground, and a flat disc marker on each assigned place point.
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

    for point in place_points:
        disc = chrono.ChVisualShapeCylinder(0.20, 0.02)
        disc.SetColor(chrono.ChColor(0.95, 0.3, 0.05))
        markers.AddVisualShape(disc, chrono.ChFramed(
            chrono.ChVector3d(point.x, point.y, 0.05), chrono.QUNIT))

    system.Add(markers)
    return markers


def build_scene():
    """Create a system with the M113 tracked vehicle, welded arm, terrain, rocks.

    Returns (m113, vehicle, terrain, gripper, rocks), where `m113` is the M113
    wrapper and `vehicle` is its underlying `ChTrackedVehicle`.
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
    patch = terrain.AddPatch(
        patch_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(ORBIT_CENTER[0], ORBIT_CENTER[1], 0),
                           chrono.QUNIT),
        TERRAIN_SIZE, TERRAIN_SIZE)
    # A repeating texture rather than TrackedVeh_Builder's flat color: this patch
    # is 100 m across and lit from above, and a flat light gray at that size just
    # blows out to white, hiding the orbit markers drawn on it.
    patch.SetColor(chrono.ChColor(0.45, 0.45, 0.45))
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/dirt.jpg"),
                     TERRAIN_SIZE / 4.0, TERRAIN_SIZE / 4.0)
    terrain.Initialize()

    # ---- Rocks on the ground in the spawn wedge alongside the builder, each
    # uniquely named. Re-sample each spot until it clears the tracks, the gripper's
    # IK can reach it, and it is clear (ROCK_MIN_SEP) of the rocks already placed.
    # The attempt budget is per rock and generous because the last few rocks are
    # fitted into whatever gaps the first ones left. ----
    rocks = []
    placed_xy = []
    for i in range(NUM_ROCKS):
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
        rock = place_rock(system, rock_pos, ground_z=rock_pos.z,
                          contact_method=chrono.ChContactMethod_SMC)
        rock.SetName(f"rock_{i}")  # unique name so the gripper locks the right one
        if i < WALL_ROCKS:
            # Tag the rocks destined for the ground layer, so the stand-in wall
            # that later gets stacked on top of them can be told to ignore them
            # (see FAKE_WALL_COLLIDES). The top rocks stay in the default family:
            # the wall has to be solid to those.
            rock.GetCollisionModel().SetFamily(FAMILY_BUILT_ROCKS)
        rocks.append(rock)
        placed_xy.append((rock_pos.x, rock_pos.y))

    add_orbit_visuals(system, PLACE_POINTS)

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


def write_rock_state(path, rocks, targets, placed, sim_time, stamp, fake=()):
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

    All NUM_ROCKS rocks are written, placed or not. `placed` is 1 for the first
    `placed` rocks, i.e. those whose full pick-carry-release-stow cycle finished;
    a run cut off mid-cycle leaves its last rock reading 0 even if it had already
    been released.

    `fake` (the stand-in layers from `add_fake_layers`, grouped per layer) is
    written after them with `fake=1` and `layer` counting up from 1, and no target
    columns -- they were sampled over a belt, not aimed at a point. Anything
    re-measuring a belt from this file must filter them out (`fake == 0`), or it
    measures the stack instead of the ground layer it stands on. The TOP_ROCKS
    rocks laid on the finished wall are real (`fake=0`) but are marked `layer=top`
    and carry no place point or course, since they are aimed at the wall's top
    surface rather than at a planned point on the arc.
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
        writer.writerow(["rock", "index", "place_point", "course", "placed",
                         "fake", "layer",
                         "target_x", "target_y", "target_z",
                         "ref_x", "ref_y", "ref_z",
                         "ref_e0", "ref_e1", "ref_e2", "ref_e3",
                         "com_x", "com_y", "com_z",
                         "r", "theta_deg", "place_err", "place_err_xy",
                         "sim_time", "run_stamp"])
        for i, rock in enumerate(rocks):
            # The wall rocks are aimed at a numbered place point on a numbered
            # course; the top rocks sit on the finished wall, so they get neither,
            # and their target only exists once the run has worked out where the
            # wall's top surface ended up.
            on_wall = i < WALL_ROCKS
            place_point = i % NUM_PLACE_POINTS if on_wall else ""
            course = i // NUM_PLACE_POINTS if on_wall else ""
            layer = "" if on_wall else "top"
            if i >= len(targets):
                writer.writerow([rock.GetName(), i, place_point, course,
                                 int(i < placed), 0, layer, "", "", ""]
                                + pose_columns(rock)
                                + ["", "", f"{sim_time:.3f}", stamp])
                continue
            target, com = targets[i], rock.GetPos()
            writer.writerow([rock.GetName(), i, place_point, course,
                             int(i < placed), 0, layer,
                             f"{target.x:.6f}", f"{target.y:.6f}", f"{target.z:.6f}"]
                            + pose_columns(rock)
                            + [f"{(com - target).Length():.4f}",
                               f"{math.hypot(com.x - target.x, com.y - target.y):.4f}",
                               f"{sim_time:.3f}", stamp])
        index = len(rocks)
        for layer_idx, layer in enumerate(fake, start=1):
            for rock in layer:
                writer.writerow([rock.GetName(), index, "", "", 1, 1, layer_idx,
                                 "", "", ""]
                                + pose_columns(rock)
                                + ["", "", f"{sim_time:.3f}", stamp])
                index += 1
    return path


def write_site_plan(path, vehicle, sim_time, stamp, placed):
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
        ("rocks_placed", placed),
        ("orbit_center_x", ORBIT_CENTER[0]),
        ("orbit_center_y", ORBIT_CENTER[1]),
        ("wall_radius", WALL_RADIUS),
        ("vehicle_radius", VEHICLE_RADIUS),
        ("ground_z", 0.0),
        ("num_place_points", NUM_PLACE_POINTS),
        ("half_span_deg", PLACE_HALF_SPAN_DEG),
        ("place_height", PLACE_HEIGHT),
        ("courses", COURSES),
        ("course_rise", COURSE_RISE),
        ("num_rocks", NUM_ROCKS),
        ("wall_rocks", WALL_ROCKS),
        ("top_rocks", TOP_ROCKS),
        ("fake_layers", FAKE_LAYERS),
        ("fake_layer_rise", FAKE_LAYER_RISE),
        ("fake_rocks_per_layer", FAKE_ROCKS_PER_LAYER),
        ("fake_half_width", FAKE_HALF_WIDTH),
        ("fake_layer_seed", FAKE_LAYER_SEED),
        ("builder_station_deg", f"{BUILDER_STATION_DEG:.6f}"),
        ("builder_facing", BUILDER_FACING),
        ("build_station_deg", f"{BUILD_STATION_DEG:.6f}"),
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


def run(m113, vehicle, terrain, gripper, rocks, targets, vis=None,
        run_time=HEADLESS_RUN_TIME, frame_output_dir=None,
        fake_layers=FAKE_LAYERS, frame_stride=1):
    """Hold the brakes; after the scene settles, lay rock i on `targets[i]`.

    Once `T_GRASP_START` has elapsed, `gripper.grasp(rock, targets[i])` is called
    every step for the current rock; when it returns True (rock released on its
    target) the loop advances to the next rock *and* the next target -- the
    gripper's state machine restarts automatically for the new one. `targets` comes
    from `rock_targets()`, so the arc is laid COURSES times over: rock i lands on
    wall point i % NUM_PLACE_POINTS, one course higher each pass. The brakes stay
    on the whole time (belt-and-braces: PARK_CHASSIS_FIXED already pins the hull).
    The run has two halves. The first WALL_ROCKS rocks go on the wall arc as
    above. Once the last of them is down, `fake_layers` layers of stand-in rocks
    are stacked on the belt they actually formed (see `add_fake_layers`), and the
    remaining rocks are then laid *on top of that wall*: their targets are appended
    to `targets` here rather than passed in, because where the wall's top surface
    ends up is not known until it is standing. Each one is pinned as its cycle
    finishes (PIN_TOP_ROCKS). The sim keeps running afterwards, so a render window
    shows the finished wall.

    The loop stops at `run_time` seconds of sim time, or when the render window is
    closed, whichever comes first; `run_time=None` runs until the window is closed
    (the default when rendering, since a window means someone is watching). Ctrl-C
    stops it cleanly either way, so the caller can still dump the state it reached.

    `frame_stride` writes only every Nth rendered frame when capturing a PNG
    sequence -- at 30 rendered frames per simulated second, a several-minute demo
    is tens of thousands of images otherwise, and a movie of it wants to run faster
    than real time anyway.

    Returns `(placed, fake, targets)`: how many rocks the gripper laid, the
    stand-in rock bodies added on top grouped per layer, and the full target list
    including the top rocks' runtime ones.

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
    frame_idx = 0   # PNGs written
    rendered = 0    # frames drawn (frame_stride of these get written)
    fake = []  # stand-in rocks, added once the built layer is finished
    held, released_at = False, None  # settle watch for the rock being laid on top
    # The top rocks' targets are appended once the wall exists, so work on a copy
    # rather than growing the caller's (module-level) list of wall targets.
    targets = list(targets)
    if vis is not None:
        vehicle.EnableRealtime(True)
    if frame_output_dir:
        os.makedirs(frame_output_dir, exist_ok=True)

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

            # ---- hold the brakes; lay each rock on its own target once settled ----
            driver_inputs.m_throttle = 0.0
            driver_inputs.m_steering = 0.0
            driver_inputs.m_braking = 1.0
            # ---- pin a top rock the moment it settles on the wall ----
            # The gripper holds the rock locked and its collision off, so the step
            # the lock goes away is the step it was released; from there it is a
            # free body landing on the crest. See PIN_TOP_ROCKS for why it cannot
            # wait until the cycle ends.
            if PIN_TOP_ROCKS and WALL_ROCKS <= idx < len(rocks):
                rock = rocks[idx]
                if gripper.cur_lock and gripper.cur_object == rock.GetName():
                    held = True
                elif held and not rock.IsFixed():
                    if released_at is None:
                        released_at = time
                    if (rock.GetPosDt().Length() < TOP_SETTLE_SPEED
                            or time - released_at > TOP_SETTLE_TIMEOUT):
                        rock.SetFixed(True)
                        print(f"  -> rock {idx} settled on the wall at "
                              f"z={rock.GetPos().z:.2f} m, "
                              f"{time - released_at:.2f} s after release -- pinned")

            if time > T_GRASP_START and idx < min(len(rocks), len(targets)):
                if gripper.grasp(rocks[idx], targets[idx]):
                    rock = rocks[idx]
                    r, theta = planner.to_polar(rock.GetPos())
                    where = (f"wall point {idx % NUM_PLACE_POINTS} "
                             f"(course {idx // NUM_PLACE_POINTS})" if idx < WALL_ROCKS
                             else "top of the wall")
                    print(f"  -> rock {idx} placed on {where} "
                          f"(r={r:.2f} m, theta={theta:.2f} deg, z={rock.GetPos().z:.2f} m)")
                    idx += 1  # next rock, next target (grasp() restarts on the new one)
                    held, released_at = False, None  # reset the settle watch
                    if idx == WALL_ROCKS:
                        # Ground layer done: stack the stand-in layers on the belt
                        # it actually formed, rather than the one it was aimed at,
                        # then work out where on the finished wall the top rocks go
                        # -- which cannot be known until the wall exists.
                        print(f"\n  ground layer complete -- adding {fake_layers} "
                              f"stand-in layer(s) on top")
                        belt, layers = add_fake_layers(system, rocks[:WALL_ROCKS],
                                                       fake_layers, vis=vis)
                        fake += layers
                        surface = wall_top_z(belt, layers)
                        targets += top_rock_targets(belt, surface,
                                                    len(rocks) - WALL_ROCKS)
                        print(f"  wall top surface: z={surface:.2f} m -- "
                              f"{len(targets) - WALL_ROCKS} rock(s) to lay on it, "
                              f"released at z={targets[-1].z:.2f} m\n")

            # Keep the vehicle, terrain, and visual modules in sync, then advance.
            terrain.Synchronize(time)
            m113.Synchronize(time, driver_inputs)
            if vis is not None:
                vis.Synchronize(time, driver_inputs)
            terrain.Advance(STEP_SIZE)
            m113.Advance(STEP_SIZE)
            if vis is not None:
                vis.Advance(STEP_SIZE)
            steps += 1
    except KeyboardInterrupt:
        print("\n  interrupted -- stopping the run "
              f"after {idx}/{len(rocks)} rocks")
    return idx, fake, targets


def main():
    headless = "--headless" in sys.argv
    use_vsg = "--vsg" in sys.argv  # render with VSG (Vulkan) instead of Irrlicht
    plan_only = "--plan-only" in sys.argv
    frame_output_dir = parse_frame_output_dir(sys.argv)
    state_dir = parse_state_dir(sys.argv)
    fake_layers = parse_number_option(sys.argv, "--fake-layers", FAKE_LAYERS)
    frame_stride = parse_number_option(sys.argv, "--frame-stride", 1)
    # Headless stops at HEADLESS_RUN_TIME; a render window runs until it is closed
    # unless a limit is asked for, which is what makes an unattended movie run
    # terminate on its own.
    run_time = parse_number_option(sys.argv, "--run-time",
                                   HEADLESS_RUN_TIME if headless else None,
                                   cast=float)
    title = (f"M113 builder on the wall orbit: {WALL_ROCKS} rocks on "
             f"{len(PLACE_POINTS)} wall points, {FAKE_LAYERS} stand-in layers, "
             f"{TOP_ROCKS} rocks on top")
    print(f"=== {title} (headless smoke test) ===" if headless else f"=== {title} ===")
    print(planner.describe(BUILD_STATION_DEG))
    print(f"  builder       : station {BUILDER_STATION_DEG:.1f} deg -> "
          f"({INIT_LOC.x:.2f}, {INIT_LOC.y:.2f}), heading "
          f"{planner.heading_deg(BUILDER_STATION_DEG, BUILDER_FACING):.1f} deg "
          f"({BUILDER_FACING})")
    print(f"  arm base      : ({ARM_BASE.x:.2f}, {ARM_BASE.y:.2f}, {ARM_BASE.z:.2f}), "
          f"r={planner.to_polar(ARM_BASE)[0]:.2f} m, station {BUILD_STATION_DEG:.2f} deg")

    if plan_only:
        # Geometry only: how far the arm has to reach for each planned drop point
        # (the arm works well at ~2-4 m). One line per rock, so the raised courses
        # show up too. Only the WALL_ROCKS wall targets are known ahead of the run
        # -- the top rocks are aimed at a wall that does not exist yet.
        for i, p in enumerate(ROCK_TARGETS):
            print(f"  rock {i:2d} -> place {i % NUM_PLACE_POINTS} "
                  f"course {i // NUM_PLACE_POINTS} (z={p.z:.2f}): "
                  f"{(p - ARM_BASE).Length():.2f} m from the arm base")
        print(f"  rocks {WALL_ROCKS}..{NUM_ROCKS - 1}: on top of the finished wall, "
              "targets computed at run time")
        return

    m113, vehicle, terrain, gripper, rocks = build_scene()
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

    vis = None if headless else make_vis(vehicle, title, use_vsg=use_vsg)
    placed, fake, targets = 0, [], ROCK_TARGETS
    try:
        placed, fake, targets = run(m113, vehicle, terrain, gripper, rocks,
                                    ROCK_TARGETS, vis=vis, run_time=run_time,
                                    frame_output_dir=frame_output_dir,
                                    fake_layers=fake_layers,
                                    frame_stride=frame_stride)
    finally:
        # Dump the end state even if the run was cut short, so a partial build is
        # still resumable (and still tells you where every rock ended up).
        sim_time = m113.GetSystem().GetChTime()
        print(f"\n  {placed}/{len(rocks)} rocks placed in {sim_time:.1f} s of sim time")
        for i, rock in enumerate(rocks):
            pos = rock.GetPos()
            r, theta = planner.to_polar(pos)
            where = (f"wall point {i % NUM_PLACE_POINTS}, course {i // NUM_PLACE_POINTS}"
                     if i < WALL_ROCKS else "top of the wall")
            err = (f"{math.hypot(pos.x - targets[i].x, pos.y - targets[i].y):.2f} m "
                   "(horizontal) from " if i < len(targets) else "aimed at ")
            print(f"  {rock.GetName()} end  : r={r:.2f} m, theta={theta:.2f} deg, "
                  f"z={pos.z:.2f} m -> {err}{where}")
        if state_dir:
            stamp = datetime.datetime.now().isoformat(timespec="seconds")
            os.makedirs(state_dir, exist_ok=True)
            rock_csv = write_rock_state(os.path.join(state_dir, ROCK_STATE_FILE),
                                        rocks, targets, placed, sim_time,
                                        stamp, fake=fake)
            plan_csv = write_site_plan(os.path.join(state_dir, SITE_PLAN_FILE),
                                       vehicle, sim_time, stamp, placed)
            print(f"  rock state    : {rock_csv}")
            print(f"  site plan     : {plan_csv}")


if __name__ == "__main__":
    main()
