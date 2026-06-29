"""M113 tracked vehicle + gripper arm -- pick-and-place "builder" scene.

A tracked vehicle (Chrono's M113 APC) with the LRV gripper arm welded to the
*front* of the chassis, standing (braked) on flat rigid terrain with three rocks
resting on the ground nearby. The rig settles, then the mounted gripper picks the
rocks up one at a time and drops them all at a single point, piling them up --
a pick-and-place sequence like `LRV_Trailer.py`, but without the trailer.

The rocks spawn in a polar wedge around the vehicle center: r ~ 3 m, theta in
[40, 60] deg (measured CCW from the chassis +X / forward axis). Each is carried
to the same world drop point (`PLACE_POINT`) and released, so they stack.

Differences from `LRV_Trailer.py`:
  * The vehicle is the M113 (`veh.M113`, a *tracked* vehicle) instead of the
    wheeled LRV/Polaris. Tracked vehicles step through the M113 wrapper's
    `Synchronize`/`Advance` (no per-wheel tires, no explicit `DoStepDynamics`),
    and render through `ChTrackedVehicleVisualSystem{Irrlicht,VSG}`.
  * The arm links are scaled up (`ARM_SCALE`) and welded to the *front* of the
    chassis (an offset on top of the front deck) at a 180-deg yaw (`ARM_MOUNT_ROT`);
    the grasp IK accounts for that mount rotation (see `TrailerArm._ik_to`). The
    gripper fingers are NOT scaled (the rocks are a fixed small size), so the small
    gripper rides at the tip of the longer arm (see `LRV_Arm._apply_scale`).
  * The whole scene is SMC (penalty) contact, matching the gripper finger pads.
  * No trailer / dump bed: rocks are dropped at one fixed point on the ground
    instead of onto a trailer bed.

The grasp state machine (`TrailerArm`) and the rock recipe (`place_rock`) are
reused from `LRV_Trailer.py` / `LRV_Arm.py`. The arm is rigidly locked to the
M113 chassis body, so it rides with the vehicle.

Run with the project's conda env:

    conda run -n chrono python scenarios/TrackedVeh_Builder.py

Add `--headless` to step the simulation without opening a render window (used for
smoke tests). Add `--vsg` to render with the VSG (Vulkan) backend instead of the
default Irrlicht (OpenGL) one. Add `--save-frames [dir]` to dump each rendered
Irrlicht frame as a PNG sequence.
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

# Initial vehicle pose and integration step. Use the same far-from-origin X/Y as
# LRV_Trailer so Chrono's typical lights affect this scene similarly; keep the
# M113-specific Z height so the tracks settle cleanly.
INIT_LOC = chrono.ChVector3d(-83, -85, 0.7)
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)
STEP_SIZE = 5e-4

# Uniform geometric scale of the gripper arm (1.0 = as exported). Mass/inertia
# stay at 1x values (geometry-only scaling).
ARM_SCALE = 2.0

# Mount point of the arm base in the chassis *reference* frame (X forward, Y
# left, Z up; the reference origin sits near the front of the hull). The hull's
# front deck top is ~1.22 m above the reference origin and the front edge is at
# x ~ +0.5, so this offset lands the arm base on the front deck.
ARM_OFFSET = chrono.ChVector3d(-2.5, 0.0, 0.4)

# Orientation the arm is welded at, relative to its imported pose. The whole arm
# is rotated rigidly about the mount point by this quaternion (here: 90 deg yaw
# about +Z). Set to None to mount in the imported orientation.
ARM_MOUNT_ROT = chrono.QuatFromAngleZ(math.pi)

# Rock spawn: NUM_ROCKS rocks on the ground in a polar wedge around the *vehicle
# center* (chassis origin). r is the distance from that center; theta is measured
# CCW from the chassis +X (forward) axis, which at the initial identity heading
# matches world axes. Each spawn is re-sampled until the gripper's IK can reach it
# and it is clear (ROCK_MIN_SEP) of the rocks already placed.
NUM_ROCKS = 5
ROCK_R_RANGE = (2.0, 3.0)              # m from the vehicle center (~3 m)
ROCK_THETA_RANGE_DEG = (-120.0, -100.0)   # deg CCW from chassis +X (forward)
# Keep spawned rocks at least this far apart (m). The rocks are ~0.2 m across and
# are NOT scaled with the arm, so this is an absolute distance -- and it must stay
# well under the ~1.5 m extent of the (tight) spawn wedge so 3 of them can fit.
ROCK_MIN_SEP = 0.1

# End-effector grasp height: how high above the ground (z) the gripper center aims
# when reaching for a rock. The rocks are a fixed ~0.2 m tall AND the gripper
# fingers are NOT scaled with the arm (they stay 1x so they can grasp the small
# rocks), so the grab height does NOT scale with ARM_SCALE -- the small gripper has
# to come down to the rock just like the unscaled arm does.
# TUNE THIS: raise GRAB_HEIGHT_FACTOR (>1) to grab higher off the ground, lower it
# (<1) to grab nearer the ground. 1.0 reproduces TrailerArm's default (0.22).
GRAB_HEIGHT_FACTOR = 1.0
GRAB_HEIGHT = GRAB_HEIGHT_FACTOR * 0.22

# Pick-and-place timing and the single drop point. The vehicle holds its brakes
# the whole time; once T_GRASP_START has elapsed (rig + rocks settled) the gripper
# picks each rock up in turn and carries it to PLACE_POINT (a fixed world point to
# the front-left, within the arm's reach), releasing them all at the same spot so
# they pile up.
T_GRASP_START = 6.0
PLACE_POINT = chrono.ChVector3d(-0.5, 3.0, 0.45) + INIT_LOC  # front-left of the vehicle, ~0.45 m above ground

# Headless smoke-test duration (seconds of sim time): long enough for NUM_ROCKS
# pick-and-place cycles (~15 s each, worst case) plus settling. The tracked sim
# runs well below real time, so this is slow in wall-clock. With a render window
# the scene instead runs until the window is closed.
HEADLESS_RUN_TIME = 60.0
DEFAULT_FRAME_DIR = os.path.join(project_root, "artifacts", "frames", "trackedveh_builder")


def sample_rock_position(center, terrain):
    """Random ground position for a rock, in polar coordinates around `center`.

    theta is drawn from ROCK_THETA_RANGE_DEG (measured CCW from +X) and r from
    ROCK_R_RANGE, both relative to `center` (the vehicle center). Returns a world
    `ChVector3d` resting on the terrain. At the initial identity heading the polar
    angle is in world axes, which matches the chassis frame (X forward).
    """
    lo, hi = ROCK_THETA_RANGE_DEG
    theta = math.radians(random.uniform(lo, hi))
    r = random.uniform(*ROCK_R_RANGE)
    tx = center.x + r * math.cos(theta)
    ty = center.y + r * math.sin(theta)
    ground_z = terrain.GetHeight(chrono.ChVector3d(tx, ty, center.z + 5.0))
    return chrono.ChVector3d(tx, ty, ground_z)


def build_scene():
    """Create a system with the M113 tracked vehicle, front-welded arm, terrain, rocks.

    Returns (m113, vehicle, terrain, gripper, rocks), where `m113` is the M113
    wrapper and `vehicle` is its underlying `ChTrackedVehicle`.
    """
    # The M113 model files live under the Chrono data root's `vehicle/` tree, and
    # some (e.g. the track shoe collision hulls) are referenced relative to the
    # data root itself -- so point the vehicle data path at the env's vehicle
    # dir and leave the Chrono data root at its default. (The arm loads its own
    # meshes from the project's data dir, independent of these.)
    veh.SetVehicleDataPath(os.path.join(chrono.GetChronoDataPath(), "vehicle") + os.sep)
    # # print path of vehicle data dir
    # print(f"vehicle data dir: {veh.GetVehicleDataPath()}")
    # exit()
    # ---- M113 tracked vehicle ----
    # Configuration mirrors demo_VEH_M113.cpp, with two deliberate exceptions:
    # (1) the contact method is SMC so it matches the gripper arm's SMC
    # finger-contact materials, and (2) the engine is the SIMPLE model rather
    # than SIMPLE_MAP so the fully braked vehicle does not creep (see below).
    # Everything else -- track shoe type, driveline, transmission, brakes,
    # bushings, collision, visualization, solver, integrator, step size, terrain
    # material -- matches the demo. The high-iteration BB solver below is what
    # keeps the single-pin track shoes from drifting off the wheels.
    m113 = veh.M113()
    m113.SetContactMethod(chrono.ChContactMethod_SMC)
    m113.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
    m113.SetDoublePinTrackShoeType(veh.DoublePinTrackShoeType_ONE_CONNECTOR)
    m113.SetTrackBushings(False)          # SMC iterative solvers can't use bushings
    m113.SetSuspensionBushings(False)
    m113.SetTrackStiffness(False)
    m113.SetDrivelineType(veh.DrivelineTypeTV_BDS)
    m113.SetBrakeType(veh.BrakeType_SHAFTS)
    # Use the SIMPLE engine (torque proportional to throttle) rather than
    # SIMPLE_MAP. SIMPLE_MAP follows an idle torque map that is nonzero at zero
    # throttle, and the AUTOMATIC_SIMPLE_MAP transmission couples that idle
    # torque to the sprockets (torque-converter creep) strongly enough to
    # overpower the brakes -- which made the fully braked vehicle creep forward
    # at ~0.5 m/s. With SIMPLE, zero throttle means zero engine torque, so the
    # braked vehicle stays put while still driving normally under throttle.
    m113.SetEngineType(veh.EngineModelType_SIMPLE)
    m113.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    m113.SetChassisCollisionType(veh.CollisionType_NONE)
    m113.SetChassisFixed(False)
    m113.CreateTrack(True)
    m113.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    m113.Initialize()

    # Single-pin track -> MESH for every track component; chassis drawn NONE, as
    # in the demo (flip to chrono.VisualizationType_MESH to draw the hull).
    track_vis = chrono.VisualizationType_MESH
    m113.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    m113.SetSprocketVisualizationType(chrono.VisualizationType_MESH)
    m113.SetIdlerVisualizationType(track_vis)
    m113.SetSuspensionVisualizationType(track_vis)
    m113.SetIdlerWheelVisualizationType(track_vis)
    m113.SetRoadWheelVisualizationType(track_vis)
    m113.SetTrackShoeVisualizationType(track_vis)

    system = m113.GetSystem()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # BULLET, as in the demo
    # Solver + integrator, replicating the demo's SetChronoSolver(BARZILAIBORWEIN,
    # EULER_IMPLICIT_LINEARIZED): a Barzilai-Borwein VI solver with 100 iterations
    # (Omega 0.8, sharpness 1.0). The high iteration count keeps the many
    # single-pin track constraints together -- the default (~50) let track shoes
    # drift and fall off the road wheels.
    solver = chrono.ChSolverBB()
    solver.SetMaxIterations(100)
    solver.SetOmega(0.8)
    solver.SetSharpnessLambda(1.0)
    system.SetSolver(solver)
    system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

    vehicle = m113.GetVehicle()

    # ---- Gripper arm welded to the front of the chassis ----
    # `TrailerArm` (the grasp state machine from LRV_Trailer) locks its base to the
    # chassis body when a vehicle is passed in, so the arm rides rigidly with the
    # M113. The mount point is the chassis reference frame plus ARM_OFFSET (front
    # deck); ARM_MOUNT_ROT yaws the whole arm 180 deg, and the grasp IK accounts
    # for that so it still reaches the right side. The arm builds NSC finger pads,
    # so rebuild them as SMC to match this (SMC) scene.
    arm_offset = vehicle.GetChassis().GetPos() + ARM_OFFSET
    gripper = TrailerArm(system, arm_offset, vehicle, scale=ARM_SCALE,
                         mount_rot=ARM_MOUNT_ROT)
    match_gripper_contact_material(gripper, chrono.ChContactMethod_SMC, ARM_SCALE)
    # Grasp height for the (unscaled, 1x) gripper; tune via GRAB_HEIGHT_FACTOR
    # above. Set before the rock loop below, which uses plan_grasp() (and thus
    # GRAB_HEIGHT) to check reachability.
    gripper.GRAB_HEIGHT = GRAB_HEIGHT

    # ---- Flat rigid patch at ground level under the vehicle ----
    # Material matches the demo: mu=0.9, cr=0.2, Y=2e7 (Y unused by SMC).
    terrain = veh.RigidTerrain(system)
    minfo = chrono.ChContactMaterialData()
    minfo.mu = 0.9
    minfo.cr = 0.2
    minfo.Y = 2e7
    patch_mat = minfo.CreateMaterial(chrono.ChContactMethod_SMC)
    patch = terrain.AddPatch(
        patch_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(INIT_LOC.x, INIT_LOC.y, 0), chrono.QUNIT),
        200.0, 200.0)
    # Match LRV_Trailer's neutral terrain color so the same light setup reads
    # similarly in the tracked demo.
    patch.SetColor(chrono.ChColor(0.7, 0.7, 0.7))
    terrain.Initialize()

    # ---- Rocks on the ground in a polar wedge around the vehicle center, each
    # uniquely named. Re-sample each spot until the gripper's IK can reach it (the
    # mount-rotation-aware grasp IK) and it is clear (ROCK_MIN_SEP) of the rocks
    # already placed. ----
    center = vehicle.GetChassis().GetPos()
    rocks = []
    placed_xy = []
    for i in range(NUM_ROCKS):
        for _attempt in range(80):
            rock_pos = sample_rock_position(center, terrain)
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
    vis.SetChaseCamera(chrono.ChVector3d(0.0, -2.0, 1.75), -8.0, 1.5)
    vis.Initialize()
    # Light setup mirrors LRV_Trailer. The scene now starts at the same far-from-
    # origin X/Y location, so Chrono's typical origin-based fill lights no longer
    # over-brighten the tracked vehicle.
    vis.AddLightWithShadow(INIT_LOC + chrono.ChVector3d(0, 0, 30), INIT_LOC,
                           20, 1, 60, 50)
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


def run(m113, vehicle, terrain, gripper, rocks, vis=None, run_time=HEADLESS_RUN_TIME,
        frame_output_dir=None):
    """Hold the brakes; after the scene settles, pick each rock up and drop it at
    PLACE_POINT, piling them all at the same spot.

    Once `T_GRASP_START` has elapsed, `gripper.grasp(rock, PLACE_POINT)` is called
    every step for the current rock; when it returns True (rock released at the
    drop point) the loop advances to the next rock -- the gripper's state machine
    restarts automatically for the new target. The vehicle holds its brakes the
    whole time. With `vis`, render until the window is closed; otherwise step
    headless until `run_time` seconds (smoke test). Tracked vehicles advance
    through the M113 wrapper's Synchronize/Advance (which steps the owned system),
    so no explicit DoStepDynamics is needed.
    """
    system = m113.GetSystem()
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_throttle = 0.0
    driver_inputs.m_steering = 0.0
    driver_inputs.m_braking = 1.0  # hold the vehicle still throughout

    render_steps = math.ceil((1.0 / 30) / STEP_SIZE)
    steps = 0
    idx = 0  # index of the rock currently being picked and placed
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

        # ---- hold the brakes; pick/place each rock in turn once settled ----
        driver_inputs.m_throttle = 0.0
        driver_inputs.m_steering = 0.0
        driver_inputs.m_braking = 1.0
        if time > T_GRASP_START and idx < len(rocks):
            if gripper.grasp(rocks[idx], PLACE_POINT):
                print(f"  -> rock {idx} placed at the drop point")
                idx += 1  # advance to the next rock (grasp() restarts on the new one)

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
    frame_output_dir = parse_frame_output_dir(sys.argv)
    title = (f"M113 tracked vehicle: pick up {NUM_ROCKS} rocks and pile them at "
             "one point")
    print(f"=== {title} (headless smoke test) ===" if headless else f"=== {title} ===")

    m113, vehicle, terrain, gripper, rocks = build_scene()
    center = vehicle.GetChassis().GetPos()
    print(f"  arm scale     : {ARM_SCALE}")
    print(f"  chassis center: {center}")
    print(f"  arm base      : {gripper.base.GetPos()}")
    print(f"  drop point    : {PLACE_POINT}")
    for rock in rocks:
        rel = rock.GetPos() - center
        print(f"  {rock.GetName()} start: {rock.GetPos()} "
              f"(r={math.hypot(rel.x, rel.y):.2f} m, "
              f"theta={math.degrees(math.atan2(rel.y, rel.x)):.0f} deg)")

    if frame_output_dir and (headless or use_vsg):
        print("  frame capture : disabled (--save-frames requires Irrlicht rendering)")
        frame_output_dir = None
    elif frame_output_dir:
        print(f"  frame capture : {frame_output_dir}")

    vis = None if headless else make_vis(vehicle, title, use_vsg=use_vsg)
    run(m113, vehicle, terrain, gripper, rocks, vis=vis, frame_output_dir=frame_output_dir)

    if headless:
        for rock in rocks:
            d = (rock.GetPos() - PLACE_POINT).Length()
            print(f"  {rock.GetName()} end  : {rock.GetPos()} "
                  f"-> {d:.2f} m from drop point")


if __name__ == "__main__":
    main()
