"""M113 tracked vehicle + gripper arm -- assemble-and-drive "builder" scene.

A tracked vehicle (Chrono's M113 APC) with the LRV gripper arm welded to the
*front* of the chassis, standing on flat rigid terrain. This is a builder /
showcase scene: it just assembles the rig, lets it settle, and drives it forward
so you can watch the tracked vehicle move with the arm mounted. There is no
trailer, no rock, and no pick-and-place sequence (unlike `LRV_Trailer.py`, which
this is modeled on) -- the arm rides along idle in its default pose.

Differences from `LRV_Trailer.py`:
  * The vehicle is the M113 (`veh.M113`, a *tracked* vehicle) instead of the
    wheeled LRV/Polaris. Tracked vehicles step through the M113 wrapper's
    `Synchronize`/`Advance` (no per-wheel tires, no explicit `DoStepDynamics`),
    and render through `ChTrackedVehicleVisualSystem{Irrlicht,VSG}`.
  * The arm is scaled up (`ARM_SCALE = 2.0`) and welded to the *front* of the
    chassis (a +x offset, on top of the front deck) rather than the back.
  * No trailer / dump bed / rocks.

The arm comes from the `model` package (`LRV_Arm`); it is rigidly locked to the
M113 chassis body, so it moves with the vehicle.

Run with the project's conda env:

    conda run -n chrono python scenarios/TrackedVeh_Builder.py

Add `--headless` to step the simulation without opening a render window (used for
smoke tests). Add `--vsg` to render with the VSG (Vulkan) backend instead of the
default Irrlicht (OpenGL) one.
"""

import os
import sys
import math

import pychrono as chrono
import pychrono.vehicle as veh

# Make the repo root importable so the `model` package resolves regardless of
# the current working directory.
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_root)

from model.arm_model import LRV_Arm

# Initial vehicle pose and integration step. The M113 starts a little above the
# ground and settles onto the tracks. STEP_SIZE is 1e-3 s, matching the NSC step
# in demo_VEH_M113.cpp.
INIT_LOC = chrono.ChVector3d(0, 0, 0.9)
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)
STEP_SIZE = 1e-3

# Uniform geometric scale of the gripper arm (1.0 = as exported). Mass/inertia
# stay at 1x values (geometry-only scaling).
ARM_SCALE = 2.0

# Mount point of the arm base in the chassis *reference* frame (X forward, Y
# left, Z up; the reference origin sits near the front of the hull). The hull's
# front deck top is ~1.22 m above the reference origin and the front edge is at
# x ~ +0.5, so this offset lands the arm base on the front deck.
ARM_OFFSET = chrono.ChVector3d(0.3, 0.0, 1.2)

# Drive profile: hold the brakes while the rig settles, then ease the throttle in
# over DRIVE_RAMP s and drive forward so the assembled rig is seen moving. The
# tracked vehicle steers differentially via DRIVE_STEERING (0 = straight).
SETTLE_TIME = 1.5
DRIVE_THROTTLE = 0.7
DRIVE_STEERING = 0.0
DRIVE_RAMP = 2.0

# Headless smoke-test duration (seconds of sim time): long enough to settle and
# drive forward a couple of metres (the tracked sim runs well below real time).
# With a render window the scene runs until the window is closed.
HEADLESS_RUN_TIME = 6.0


def build_scene():
    """Create a system with the M113 tracked vehicle, front-welded arm, terrain.

    Returns (m113, vehicle, terrain, gripper), where `m113` is the M113 wrapper
    and `vehicle` is its underlying `ChTrackedVehicle`.
    """
    # The M113 model files live under the Chrono data root's `vehicle/` tree, and
    # some (e.g. the track shoe collision hulls) are referenced relative to the
    # data root itself -- so point the vehicle data path at the env's vehicle
    # dir and leave the Chrono data root at its default. (The arm loads its own
    # meshes from the project's data dir, independent of these.)
    veh.SetVehicleDataPath(os.path.join(chrono.GetChronoDataPath(), "vehicle") + os.sep)

    # ---- M113 tracked vehicle ----
    # Configuration mirrors demo_VEH_M113.cpp, with one deliberate exception: the
    # contact method is NSC (not the demo's SMC) so it matches the gripper arm's
    # NSC finger-contact materials. Everything else -- track shoe type, driveline,
    # powertrain, brakes, bushings, collision, visualization, solver, integrator,
    # step size, terrain material -- matches the demo. The high-iteration BB solver
    # below is what keeps the single-pin track shoes from drifting off the wheels.
    m113 = veh.M113()
    m113.SetContactMethod(chrono.ChContactMethod_NSC)
    m113.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
    m113.SetDoublePinTrackShoeType(veh.DoublePinTrackShoeType_ONE_CONNECTOR)
    m113.SetTrackBushings(False)          # NSC iterative solvers can't use bushings
    m113.SetSuspensionBushings(False)
    m113.SetTrackStiffness(False)
    m113.SetDrivelineType(veh.DrivelineTypeTV_BDS)
    m113.SetBrakeType(veh.BrakeType_SHAFTS)
    m113.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
    m113.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    m113.SetChassisCollisionType(veh.CollisionType_NONE)
    m113.SetChassisFixed(False)
    m113.CreateTrack(True)
    m113.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    m113.Initialize()

    # Single-pin track -> MESH for every track component; chassis drawn NONE, as
    # in the demo (flip to chrono.VisualizationType_MESH to draw the hull).
    track_vis = chrono.VisualizationType_MESH
    m113.SetChassisVisualizationType(chrono.VisualizationType_NONE)
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
    # `LRV_Arm` locks its base to the chassis body when a vehicle is passed in, so
    # the arm rides rigidly with the M113. The mount point is the chassis
    # reference frame plus ARM_OFFSET (front deck).
    arm_offset = vehicle.GetChassis().GetPos() + ARM_OFFSET
    gripper = LRV_Arm(system, arm_offset, vehicle, scale=ARM_SCALE)

    # ---- Flat rigid patch (100 x 100 m) at ground level under the vehicle ----
    # Material matches the demo: mu=0.9, cr=0.2, Y=2e7 (Y unused by NSC).
    terrain = veh.RigidTerrain(system)
    minfo = chrono.ChContactMaterialData()
    minfo.mu = 0.9
    minfo.cr = 0.2
    minfo.Y = 2e7
    patch_mat = minfo.CreateMaterial(chrono.ChContactMethod_NSC)
    patch = terrain.AddPatch(
        patch_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(INIT_LOC.x, INIT_LOC.y, 0), chrono.QUNIT),
        100.0, 100.0)
    patch.SetColor(chrono.ChColor(0.5, 0.8, 0.5))
    terrain.Initialize()

    return m113, vehicle, terrain, gripper


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
        vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.0), 12.0, 3.0)
        vis.AttachVehicle(vehicle)
        vis.Initialize()
        return vis

    vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle(title)
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.0), 12.0, 3.0)
    vis.Initialize()
    # Light above the vehicle, aimed at it, so the rig and arm are lit.
    vis.AddLightWithShadow(INIT_LOC + chrono.ChVector3d(0, 0, 30), INIT_LOC,
                           20, 1, 60, 50)
    vis.AddTypicalLights()
    vis.AttachVehicle(vehicle)
    return vis


def run(m113, vehicle, terrain, gripper, vis=None, run_time=HEADLESS_RUN_TIME):
    """Settle the rig with brakes held, then drive forward with the arm mounted.

    Until SETTLE_TIME the brakes are held; after that the throttle is eased in
    over DRIVE_RAMP s and the vehicle drives forward (steering DRIVE_STEERING).
    With `vis`, render until the window is closed; otherwise step headless until
    `run_time` seconds (smoke test). Tracked vehicles advance through the M113
    wrapper's Synchronize/Advance (which steps the owned system), so no explicit
    DoStepDynamics is needed.
    """
    system = m113.GetSystem()
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_throttle = 0.0
    driver_inputs.m_steering = 0.0
    driver_inputs.m_braking = 0.0  # hold the vehicle still while it settles

    render_steps = math.ceil((1.0 / 30) / STEP_SIZE)
    steps = 0
    if vis is not None:
        vehicle.EnableRealtime(True)

    while True:
        if vis is not None and not vis.Run():
            break
        time = system.GetChTime()
        if vis is None and time >= run_time:
            break

        if vis is not None and steps % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # ---- driving: hold the brakes to settle, then ease the throttle in ----
        if time < SETTLE_TIME:
            driver_inputs.m_throttle = 0.0
            driver_inputs.m_steering = 0.0
            driver_inputs.m_braking = 0.0
        else:
            since = time - SETTLE_TIME
            driver_inputs.m_throttle = DRIVE_THROTTLE * min(1.0, since / DRIVE_RAMP)
            driver_inputs.m_steering = DRIVE_STEERING
            driver_inputs.m_braking = 0.0

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
    title = "M113 tracked vehicle with a front-mounted gripper arm"
    print(f"=== {title} (headless smoke test) ===" if headless else f"=== {title} ===")

    m113, vehicle, terrain, gripper = build_scene()
    chassis_ref = vehicle.GetChassis().GetPos()
    start_x = float(chassis_ref.x)  # plain copy: GetPos() returns a live proxy
    print(f"  arm scale     : {ARM_SCALE}")
    print(f"  chassis ref   : {chassis_ref}")
    print(f"  arm base (mounted on front deck): {gripper.base.GetPos()}")

    vis = None if headless else make_vis(vehicle, title, use_vsg=use_vsg)
    run(m113, vehicle, terrain, gripper, vis=vis)

    if headless:
        end_ref = vehicle.GetChassis().GetPos()
        print(f"  chassis end   : {end_ref}")
        print(f"  drove forward : {end_ref.x - start_x:.2f} m (x)")
        print(f"  arm base end  : {gripper.base.GetPos()}")


if __name__ == "__main__":
    main()
