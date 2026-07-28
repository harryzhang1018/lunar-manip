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
  * 10 rocks instead of 5, in a wider spawn wedge (they have to fit without
    overlapping) -- one rock per wall place point.
  * The wall and vehicle orbits and the assigned place points are drawn as
    visual-only markers so the plan is visible in the render window.
  * The hull is pinned for the parked build (PARK_CHASSIS_FIXED), because a braked
    M113 only holds still at heading 0 -- see that constant for the measurements.

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
the arm's reach to each place point) and exit without simulating.
"""

import os
import sys
import math
import random

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
from LRV_Arm import place_rock
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
                       num_points=10, half_span_deg=PLACE_HALF_SPAN_DEG,
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

# Rock spawn: NUM_ROCKS rocks on the ground in a polar wedge around the *vehicle
# center* (chassis reference). r is the distance from that center; theta is
# measured CCW from the chassis +X (forward) axis, so the wedge rides with the
# vehicle's heading. The wedge sits alongside the hull on the builder's right,
# which with a tangential heading is the outer, unloading side of the orbit --
# where a fetcher would have dropped the rocks. It stays clear of the tracks
# (|y| >= 1.6 m against a ~1.25 m track half-width) and well clear of the wall
# arc. Each spawn is re-sampled until the gripper's IK can reach it and it is
# clear (ROCK_MIN_SEP) of the rocks already placed.
NUM_ROCKS = 10                            # one per wall place point
ROCK_R_RANGE = (2.3, 3.3)                 # m from the chassis reference
ROCK_THETA_RANGE_DEG = (-135.0, -78.0)    # deg CCW from chassis +X (forward)
# Keep spawned rocks at least this far apart (m). The rocks are ~0.2 m across and
# are NOT scaled with the arm, so this is an absolute distance -- it has to exceed
# the rock width (or they spawn interpenetrating and the SMC contact kicks them
# apart), while still letting 10 of them fit in the wedge above.
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
HEADLESS_RUN_TIME = 170.0
DEFAULT_FRAME_DIR = os.path.join(project_root, "artifacts", "frames", "trackedveh_orbitbuilder")


def sample_rock_position(center, chassis_rot, terrain):
    """Random ground position for a rock, in polar coordinates around `center`.

    theta is drawn from ROCK_THETA_RANGE_DEG (measured CCW from the chassis +X
    axis) and r from ROCK_R_RANGE, both relative to `center` (the vehicle
    reference). The sampled offset is rotated by `chassis_rot`, so the wedge rides
    with the vehicle's heading rather than sitting in world axes. Returns a world
    `ChVector3d` resting on the terrain.
    """
    lo, hi = ROCK_THETA_RANGE_DEG
    theta = math.radians(random.uniform(lo, hi))
    r = random.uniform(*ROCK_R_RANGE)
    offset = chassis_rot.Rotate(chrono.ChVector3d(r * math.cos(theta),
                                                  r * math.sin(theta), 0.0))
    tx, ty = center.x + offset.x, center.y + offset.y
    ground_z = terrain.GetHeight(chrono.ChVector3d(tx, ty, center.z + 5.0))
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
    # uniquely named. Re-sample each spot until the gripper's IK can reach it and
    # it is clear (ROCK_MIN_SEP) of the rocks already placed. ----
    center = vehicle.GetChassis().GetPos()
    rocks = []
    placed_xy = []
    for i in range(NUM_ROCKS):
        for _attempt in range(200):
            rock_pos = sample_rock_position(center, chassis_rot, terrain)
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


def run(m113, vehicle, terrain, gripper, rocks, place_points, vis=None,
        run_time=HEADLESS_RUN_TIME, frame_output_dir=None):
    """Hold the brakes; after the scene settles, lay rock i on wall place point i.

    Once `T_GRASP_START` has elapsed, `gripper.grasp(rock, place_points[i])` is
    called every step for the current rock; when it returns True (rock released on
    its wall point) the loop advances to the next rock *and* the next place point
    -- the gripper's state machine restarts automatically for the new target. The
    brakes stay on the whole time (belt-and-braces: PARK_CHASSIS_FIXED already
    pins the hull). With `vis`, render until the window is closed; otherwise step
    headless until `run_time` seconds (smoke test).
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
    frame_idx = 0
    if vis is not None:
        vehicle.EnableRealtime(True)
    if frame_output_dir:
        os.makedirs(frame_output_dir, exist_ok=True)

    while True:
        if vis is not None and not vis.Run():
            break
        time = system.GetChTime()
        if vis is None and time >= run_time:
            break

        if vis is not None and steps % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            if frame_output_dir:
                frame_path = os.path.join(frame_output_dir, f"frame_{frame_idx:06d}.png")
                vis.WriteImageToFile(frame_path)
                frame_idx += 1
            vis.EndScene()

        # ---- hold the brakes; lay each rock on its own wall point once settled ----
        driver_inputs.m_throttle = 0.0
        driver_inputs.m_steering = 0.0
        driver_inputs.m_braking = 1.0
        if time > T_GRASP_START and idx < len(rocks):
            if gripper.grasp(rocks[idx], place_points[idx]):
                r, theta = planner.to_polar(rocks[idx].GetPos())
                print(f"  -> rock {idx} placed on wall point {idx} "
                      f"(r={r:.2f} m, theta={theta:.2f} deg)")
                idx += 1  # next rock, next wall point (grasp() restarts on the new one)

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


def main():
    headless = "--headless" in sys.argv
    use_vsg = "--vsg" in sys.argv  # render with VSG (Vulkan) instead of Irrlicht
    plan_only = "--plan-only" in sys.argv
    frame_output_dir = parse_frame_output_dir(sys.argv)
    title = (f"M113 builder on the wall orbit: lay {NUM_ROCKS} rocks on "
             f"{len(PLACE_POINTS)} planned wall points")
    print(f"=== {title} (headless smoke test) ===" if headless else f"=== {title} ===")
    print(planner.describe(BUILD_STATION_DEG))
    print(f"  builder       : station {BUILDER_STATION_DEG:.1f} deg -> "
          f"({INIT_LOC.x:.2f}, {INIT_LOC.y:.2f}), heading "
          f"{planner.heading_deg(BUILDER_STATION_DEG, BUILDER_FACING):.1f} deg "
          f"({BUILDER_FACING})")
    print(f"  arm base      : ({ARM_BASE.x:.2f}, {ARM_BASE.y:.2f}, {ARM_BASE.z:.2f}), "
          f"r={planner.to_polar(ARM_BASE)[0]:.2f} m, station {BUILD_STATION_DEG:.2f} deg")

    if plan_only:
        # Geometry only: how far the arm has to reach for each planned wall point
        # (the arm works well at ~2-4 m).
        for i, p in enumerate(PLACE_POINTS):
            print(f"  place {i}: {(p - ARM_BASE).Length():.2f} m from the arm base")
        return

    m113, vehicle, terrain, gripper, rocks = build_scene()
    center = vehicle.GetChassis().GetPos()
    print(f"  arm scale     : {ARM_SCALE}")
    print(f"  chassis center: {center}")
    print(f"  arm base      : {gripper.base.GetPos()}")
    for rock in rocks:
        rel = rock.GetPos() - center
        print(f"  {rock.GetName()} start: {rock.GetPos()} "
              f"(r={math.hypot(rel.x, rel.y):.2f} m from the vehicle)")

    if frame_output_dir and (headless or use_vsg):
        print("  frame capture : disabled (--save-frames requires Irrlicht rendering)")
        frame_output_dir = None
    elif frame_output_dir:
        print(f"  frame capture : {frame_output_dir}")

    vis = None if headless else make_vis(vehicle, title, use_vsg=use_vsg)
    run(m113, vehicle, terrain, gripper, rocks, PLACE_POINTS, vis=vis,
        frame_output_dir=frame_output_dir)

    if headless:
        for i, rock in enumerate(rocks):
            d = (rock.GetPos() - PLACE_POINTS[i]).Length()
            r, theta = planner.to_polar(rock.GetPos())
            print(f"  {rock.GetName()} end  : r={r:.2f} m, theta={theta:.2f} deg "
                  f"-> {d:.2f} m from its wall point")


if __name__ == "__main__":
    main()
