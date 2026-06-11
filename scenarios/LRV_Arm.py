"""LRV + gripper arm — record/replay rock placement (two-simulation demo).

A wheeled lunar rover (LRV / Polaris) with a gripper arm welded to the back of
the chassis, standing (braked) on flat terrain.

The demo runs two simulations:

  * SIM 1 (headless): pick a random target behind the arm, drive the
    end-effector there with inverse kinematics, let it settle, and *record* the
    four joint angles and the gripper-center position (the midpoint of the two
    fingers' centers of mass).

  * SIM 2 (visualized): rebuild the same scene, replay the recorded joint
    angles, and drop a rock at the recorded gripper-center position -- i.e.
    right where the gripper actually ends up. Because the scene and inputs are
    identical, the gripper settles at the same spot, so the rock lands at the
    gripper. (Recording the *actual* settled pose, rather than the IK target,
    accounts for IK residual error and dynamic sag.)

`LRV_Arm` and `RobotArmInverseKinematicsSolver` come from the `model` package.

Run with the project's conda env:

    conda run -n chrono python scenarios/LRV_Arm.py
"""

import os
import sys
import math
import random
import io
import contextlib

import pychrono as chrono
import pychrono.vehicle as veh

# Make the repo root importable so the `model` package resolves regardless of
# the current working directory.
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_root)

from model.arm_model import LRV_Arm
from model.inverseKin import RobotArmInverseKinematicsSolver

# Initial vehicle pose and integration step (shared by both simulations).
INIT_LOC = chrono.ChVector3d(-83, -85, 0.5)
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)
STEP_SIZE = 1e-3


def place_rock(system, pos, ground_z, footprint=0.12):
    """Spawn a rock resting on the ground beneath the gripper center `pos`.

    `pos` is the recorded gripper center (typically 0.2-0.3 m above ground). The
    box height (z) is sized to span from the ground up to that height, so the
    rock stands in place from the start instead of free-falling from the gripper
    and bouncing off. The footprint (x, y) stays small.
    """
    height = max(pos.z - ground_z, footprint)  # reach from ground to gripper center
    mat = chrono.ChContactMaterialNSC()
    mat.SetFriction(0.7)
    rock = chrono.ChBodyEasyBox(footprint, footprint, height, 1500, True, True, mat)
    rock.SetName("rock")
    # Center it so the box bottom sits on the ground and its top is at `pos.z`.
    rock.SetPos(chrono.ChVector3d(pos.x, pos.y, ground_z + height / 2.0))
    rock.EnableCollision(True)
    rock.GetVisualShape(0).SetColor(chrono.ChColor(0.45, 0.35, 0.28))  # rock-brown
    system.Add(rock)
    return rock


def gripper_center(gripper):
    """World-space grab point: midpoint of the two fingers' centers of mass."""
    return (gripper.finger_1.GetPos() + gripper.finger_2.GetPos()) * 0.5


def build_scene(rock_pos=None):
    """Create a system with the LRV vehicle, gripper arm, and flat terrain.

    If `rock_pos` is given, a rock is added to the scene at that world position
    (e.g. the gripper-center position recorded in sim 1).

    Returns (system, vehicle, terrain, gripper).
    """
    data_vehicle = os.path.join(project_root, "data", "vehicle") + os.sep
    veh.SetVehicleDataPath(data_vehicle)
    # The vehicle JSONs reference their meshes relative to the Chrono data root.
    chrono.SetChronoDataPath(data_vehicle)

    vehicle = veh.WheeledVehicle(veh.GetVehicleDataFile("LRV/Polaris.json"),
                                 chrono.ChContactMethod_NSC)
    vehicle.Initialize(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(chrono.VisualizationType_MESH)

    engine = veh.ReadEngineJSON(veh.GetVehicleDataFile("LRV/Polaris_EngineSimpleMap.json"))
    transmission = veh.ReadTransmissionJSON(
        veh.GetVehicleDataFile("LRV/Polaris_AutomaticTransmissionSimpleMap.json"))
    vehicle.InitializePowertrain(veh.ChPowertrainAssembly(engine, transmission))

    for axle in vehicle.GetAxles():
        for wheel in axle.GetWheels():
            tire = veh.ReadTireJSON(veh.GetVehicleDataFile("LRV/Polaris_RigidTire.json"))
            vehicle.InitializeTire(tire, wheel, chrono.VisualizationType_MESH)

    system = vehicle.GetSystem()

    # Gripper arm welded to the back of the chassis (vehicle passed in -> the
    # arm base is locked to the chassis body).
    arm_offset = vehicle.GetChassis().GetPos() + chrono.ChVector3d(-1.1, 0, 0.1)
    gripper = LRV_Arm(system, arm_offset, vehicle)

    # Flat rigid patch (100 x 100 m) at ground level under the vehicle.
    # (To use the lunar mesh instead, replace the length/width with
    #  veh.GetVehicleDataFile("terrain/lunar_env_12.obj").)
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat,
                             chrono.ChCoordsysd(chrono.ChVector3d(INIT_LOC.x, INIT_LOC.y, 0), chrono.QUNIT),
                             100.0, 100.0)
    patch.SetColor(chrono.ChColor(0.7, 0.7, 0.7))
    terrain.Initialize()

    # Optional rock, placed as part of the scene (e.g. at the recorded EE),
    # sized to rest on the ground directly under that point.
    if rock_pos is not None:
        ground_z = terrain.GetHeight(chrono.ChVector3d(rock_pos.x, rock_pos.y, rock_pos.z + 5.0))
        place_rock(system, rock_pos, ground_z)

    return system, vehicle, terrain, gripper


def sample_reachable_target(gripper, vehicle, terrain, ik_solver):
    """Random target behind the arm, solved with inverse kinematics.

    Polar around the arm base: theta in [90, 270] deg (the half-plane away from
    the vehicle, which sits toward +x), r in [2, 3] m, and 0.2-0.3 m above the
    ground. Re-samples if the point is out of reach (the arm spans ~2.7 m, so r
    near 3 m can be unreachable). Returns the four joint angles.
    """
    base = gripper.base.GetPos()
    vir_base = chrono.ChBody()
    vir_base.SetPos(base)
    vir_base.SetRot(vehicle.GetChassisBody().GetRot())  # arm-base orientation

    for _attempt in range(20):
        theta = math.radians(random.uniform(90.0, 270.0))
        r = random.uniform(2.0, 3.0)
        tx = base.x + r * math.cos(theta)
        ty = base.y + r * math.sin(theta)
        ground_z = terrain.GetHeight(chrono.ChVector3d(tx, ty, base.z + 5.0))
        target = chrono.ChVector3d(tx, ty, ground_z + random.uniform(0.2, 0.3))
        des_loc = vir_base.TransformPointParentToLocal(target)
        try:
            with contextlib.redirect_stdout(io.StringIO()):  # hush retry chatter
                final_theta = ik_solver.inverse_kinematics_solver(
                    [des_loc.x, des_loc.y, des_loc.z], elbow_up=True)
            print(f"  IK target (world): {target}")
            return final_theta
        except ValueError:
            continue  # out of reach -> sample another (theta, r)
    raise RuntimeError("No reachable target found for r in [2,3] m, theta in [90,270] deg")


def make_vis(vehicle, title):
    """Create and initialize an Irrlicht visualization attached to `vehicle`."""
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle(title)
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    # Light above the (stationary) vehicle, aimed at it, so the work area where
    # the arm operates is actually lit.
    vis.AddLightWithShadow(INIT_LOC + chrono.ChVector3d(0, 0, 30), INIT_LOC,
                           20, 1, 60, 100)
    vis.AddTypicalLights()
    vis.AttachVehicle(vehicle)
    return vis


def drive_arm(system, vehicle, terrain, gripper, theta, vis=None, settle_time=4.0):
    """Brake the vehicle and drive the arm to `theta` (staggered to avoid a snap).

    With `vis`, render until the window is closed; otherwise step headless until
    `settle_time` seconds. Returns the final gripper-center position.
    """
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_throttle = 0.0
    driver_inputs.m_steering = 0.0
    driver_inputs.m_braking = 1.0  # hold the vehicle still

    render_steps = math.ceil((1.0 / 30) / STEP_SIZE)
    moved_arm = bent_arm = False
    steps = 0
    if vis is not None:
        vehicle.EnableRealtime(True)

    while True:
        if vis is not None and not vis.Run():
            break
        time = system.GetChTime()
        if vis is None and time >= settle_time:
            break

        system.DoStepDynamics(STEP_SIZE)
        if vis is not None and steps % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # ---- drive the arm to the target pose (staggered) ----
        if time > 1.0 and not moved_arm:
            gripper.open()
            gripper.rotate_motor(gripper.motor_base_shoulder, theta[0])
            gripper.rotate_motor(gripper.motor_shoulder_biceps, theta[1])
            moved_arm = True
        if time > 2.0 and not bent_arm:
            gripper.rotate_motor(gripper.motor_biceps_elbow, theta[2])
            gripper.rotate_motor(gripper.motor_elbow_eef, theta[3])
            bent_arm = True

        # Keep the vehicle/terrain/visual modules in sync (brakes held).
        vehicle.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        if vis is not None:
            vis.Synchronize(time, driver_inputs)
        vehicle.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        if vis is not None:
            vis.Advance(STEP_SIZE)
        steps += 1

    return gripper_center(gripper)


def main():
    # ---- SIM 1 (headless): move the arm to a random spot and record. ----
    print("=== SIM 1: driving arm to a random target (recording) ===")
    system, vehicle, terrain, gripper = build_scene()
    ik_solver = RobotArmInverseKinematicsSolver()
    final_theta = sample_reachable_target(gripper, vehicle, terrain, ik_solver)
    print(f"final theta: {final_theta}")
    # Let the arm settle well before recording (it creeps fast early, then slows),
    # so the recorded gripper center matches where it ends up in sim 2.
    ee_pos = drive_arm(system, vehicle, terrain, gripper, final_theta, settle_time=4.0)
    print(f"  recorded joint angles  : {final_theta}")
    print(f"  recorded gripper center: {ee_pos}")

    # ---- SIM 2 (visualized): build with the rock at the recorded spot, replay. ----
    print("=== SIM 2: rock built into scene at recorded gripper center, replaying angles ===")
    system2, vehicle2, terrain2, gripper2 = build_scene(rock_pos=ee_pos)
    vis = make_vis(vehicle2, "LRV Gripper -- replay + rock at recorded EE")
    drive_arm(system2, vehicle2, terrain2, gripper2, final_theta, vis=vis)


if __name__ == "__main__":
    main()
