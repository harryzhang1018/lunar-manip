"""Data collection pipeline for the LRV gripper rock pickup scenario.

This keeps the same two-simulation flow as ``scenarios/LRV_Arm.py``:

* Sim 1 runs headless, samples an IK target, drives the arm there, and records
  the settled gripper-center position.
* Sim 2 rebuilds the scene with the rock at that recorded position, replays the
  arm motion, closes the gripper, locks the rock, and lifts it.

Unlike the demo, sim 2 is headless by default for data collection. Use
``--debug-irrlicht`` to open the Irrlicht vehicle viewer, and
``--sensor-visualize`` to open Chrono Sensor's camera preview window.
If the local NVIDIA/OptiX stack cannot run Chrono Sensor, ``--no-sensors`` still
runs the two-sim flow and records the mounted camera pose metadata, but no RGB
images are rendered.
By default the script will also fall back to this pose-only mode if the Chrono
Sensor preflight fails. Use ``--require-sensors`` to fail instead.

The camera follows Chrono's Gator sensor demo pattern: a ``ChCameraSensor`` is
attached to the vehicle chassis with a fixed chassis-local offset pose. The
default mount is derived from the arm-base center and looks backward along the
vehicle's -X direction toward the rock workspace.

Run with the project's conda env:

    conda run -n pychrono python scenarios/collect_data.py --episodes 1
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
import os
import random
import shutil
import subprocess
import sys
from dataclasses import asdict, dataclass
from typing import Optional

import pychrono as chrono
import pychrono.vehicle as veh

try:
    import pychrono.sensor as sens
except ImportError:
    sens = None

# Make the repo root importable so the scenario and model packages resolve
# regardless of the current working directory.
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from model.inverseKin import RobotArmInverseKinematicsSolver
from scenarios.LRV_Arm import (
    CTRL_DT,
    FINGER_CLOSE_POS,
    FINGER_CLOSE_SPEED,
    FINGER_OPEN_SEP,
    GRAB_HEIGHT,
    GRIP_STALL_TOL,
    LIFT_SPEED,
    LIFT_THETA2,
    STEP_SIZE,
    T_CLOSE,
    T_LIFT,
    build_scene,
    drive_arm,
    gripper_center,
    make_vis,
)


DEFAULT_OUTPUT_DIR = os.path.join(PROJECT_ROOT, "artifacts", "datasets", "lrv_gripper_camera")


@dataclass
class CameraConfig:
    width: int = 640
    height: int = 360
    update_rate: float = 15.0
    fov_deg: float = 90.0
    lag: float = 0.0
    exposure_time: float = 0.0
    base_offset_x: float = 0.9
    base_offset_y: float = 0.0
    base_offset_z: float = 2.6
    backward_pitch_deg: float = 50.0
    visualize: bool = False
    save_images: bool = True


@dataclass
class EpisodeConfig:
    sim1_settle_time: float = 4.0
    sim2_duration: float = 10.0
    seed: int = 7
    target_radius_min: float = 2.0
    target_radius_max: float = 3.0
    target_angle_min_deg: float = 90.0
    target_angle_max_deg: float = 270.0


def vec_to_list(v: chrono.ChVector3d) -> list[float]:
    return [float(v.x), float(v.y), float(v.z)]


def quat_to_list(q: chrono.ChQuaterniond) -> list[float]:
    return [float(q.e0), float(q.e1), float(q.e2), float(q.e3)]


def relpath_or_none(path: Optional[str], root: str) -> Optional[str]:
    if path is None:
        return None
    return os.path.relpath(path, root)


def frame_record_with_relative_paths(frame: Optional[dict], root: str) -> Optional[dict]:
    if frame is None:
        return None
    row = dict(frame)
    for key in ("image_directory", "image_path", "initial_rgb_path"):
        row[key] = relpath_or_none(row.get(key), root)
    return row


def safe_unit(v: chrono.ChVector3d, fallback: chrono.ChVector3d) -> chrono.ChVector3d:
    length = v.Length()
    if length < 1e-9:
        return fallback
    return v / length


def camera_look_quat(
    forward: chrono.ChVector3d,
    preferred_up: chrono.ChVector3d = chrono.ChVector3d(0.0, 1.0, 0.0),
) -> chrono.ChQuaterniond:
    """Return a parent-frame rotation whose camera optical +X points forward."""
    x_axis = safe_unit(forward, chrono.ChVector3d(0.0, 0.0, 1.0))
    up = safe_unit(preferred_up, chrono.ChVector3d(0.0, 1.0, 0.0))

    # Project the preferred up vector into the plane perpendicular to the view
    # direction. If it is nearly parallel, use a stable fallback.
    z_axis = up - x_axis * up.Dot(x_axis)
    if z_axis.Length() < 1e-6:
        up = chrono.ChVector3d(0.0, 0.0, 1.0)
        z_axis = up - x_axis * up.Dot(x_axis)
    if z_axis.Length() < 1e-6:
        up = chrono.ChVector3d(1.0, 0.0, 0.0)
        z_axis = up - x_axis * up.Dot(x_axis)
    z_axis = safe_unit(z_axis, chrono.ChVector3d(0.0, 1.0, 0.0))
    y_axis = z_axis.Cross(x_axis)

    # Rotation matrix columns are the parent-frame images of camera-local
    # +X, +Y, +Z. Convert that matrix to Chrono's [w, x, y, z] quaternion.
    m00, m01, m02 = x_axis.x, y_axis.x, z_axis.x
    m10, m11, m12 = x_axis.y, y_axis.y, z_axis.y
    m20, m21, m22 = x_axis.z, y_axis.z, z_axis.z
    trace = m00 + m11 + m22

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s

    return chrono.ChQuaterniond(qw, qx, qy, qz)


def make_vehicle_rear_camera_pose(
    vehicle,
    gripper,
    config: CameraConfig,
) -> tuple[chrono.ChFramed, chrono.ChVector3d, chrono.ChVector3d]:
    """Build a chassis-local rear-looking camera pose from the arm-base center."""
    chassis = vehicle.GetChassisBody()
    base_local = chassis.TransformPointParentToLocal(gripper.base.GetPos())
    camera_pos = base_local + chrono.ChVector3d(
        config.base_offset_x,
        config.base_offset_y,
        config.base_offset_z,
    )

    pitch = math.radians(config.backward_pitch_deg)
    look_dir = safe_unit(
        chrono.ChVector3d(-math.cos(pitch), 0.0, -math.sin(pitch)),
        chrono.ChVector3d(-1.0, 0.0, 0.0),
    )
    camera_rot = camera_look_quat(look_dir, chrono.ChVector3d(0.0, 0.0, 1.0))
    return chrono.ChFramed(camera_pos, camera_rot), base_local, look_dir


def sample_reachable_action(
    gripper,
    vehicle,
    terrain,
    ik_solver: RobotArmInverseKinematicsSolver,
    config: EpisodeConfig,
) -> tuple[list[float], chrono.ChVector3d, dict]:
    """Sample a random reachable rock grab point and solve the 4D arm action."""
    base = gripper.base.GetPos()
    vir_base = chrono.ChBody()
    vir_base.SetPos(base)
    vir_base.SetRot(vehicle.GetChassisBody().GetRot())

    angle_min = min(config.target_angle_min_deg, config.target_angle_max_deg)
    angle_max = max(config.target_angle_min_deg, config.target_angle_max_deg)
    radius_min = min(config.target_radius_min, config.target_radius_max)
    radius_max = max(config.target_radius_min, config.target_radius_max)

    for attempt in range(20):
        polar_angle_rad = math.radians(random.uniform(angle_min, angle_max))
        radius = random.uniform(radius_min, radius_max)
        tx = base.x + radius * math.cos(polar_angle_rad)
        ty = base.y + radius * math.sin(polar_angle_rad)
        ground_z = terrain.GetHeight(chrono.ChVector3d(tx, ty, base.z + 5.0))
        target = chrono.ChVector3d(tx, ty, ground_z + GRAB_HEIGHT)
        des_loc = vir_base.TransformPointParentToLocal(target)
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                theta = ik_solver.inverse_kinematics_solver(
                    [des_loc.x, des_loc.y, des_loc.z], elbow_up=True
                )
            sample = {
                "attempts": attempt + 1,
                "radius_m": float(radius),
                "polar_angle_rad": float(polar_angle_rad),
                "polar_angle_deg": math.degrees(polar_angle_rad),
                "arm_base_world": vec_to_list(base),
                "target_local_to_arm_base": [float(des_loc.x), float(des_loc.y), float(des_loc.z)],
            }
            print(f"  IK target (world): {target}")
            return [float(x) for x in theta], target, sample
        except ValueError:
            continue

    raise RuntimeError(
        "No reachable target found for "
        f"r in [{radius_min}, {radius_max}] m, "
        f"theta in [{angle_min}, {angle_max}] deg"
    )


class GripperCameraRig:
    """Chrono Sensor camera attached to the LRV chassis near the arm base."""

    def __init__(self, system, vehicle, gripper, episode_dir: str, config: CameraConfig):
        if sens is None:
            raise RuntimeError("pychrono.sensor is not available in this Python environment.")

        self.config = config
        self.parent_body = vehicle.GetChassisBody()
        self.rgb_dir = os.path.join(episode_dir, "rgb")
        if config.save_images:
            os.makedirs(self.rgb_dir, exist_ok=True)

        self.manager = sens.ChSensorManager(system)
        if hasattr(self.manager, "SetRayRecursions"):
            self.manager.SetRayRecursions(2)

        intensity = 0.6
        self.manager.scene.SetAmbientLight(chrono.ChVector3f(0.03, 0.03, 0.03))
        self.manager.scene.AddPointLight(
            chrono.ChVector3f(-85.0, -85.0, 8.0),
            chrono.ChColor(intensity, intensity, intensity),
            80.0,
        )
        # self.manager.scene.AddPointLight(
        #     chrono.ChVector3f(-89.0, -82.0, 5.0),
        #     chrono.ChColor(0.7, 0.7, 0.7),
        #     40.0,
        # )

        offset_pose, base_local, look_dir = make_vehicle_rear_camera_pose(vehicle, gripper, config)
        self.offset_pose = offset_pose
        self.base_local = base_local
        self.look_dir_local = look_dir
        self.last_frame = -1

        self.camera = sens.ChCameraSensor(
            self.parent_body,
            config.update_rate,
            offset_pose,
            config.width,
            config.height,
            math.radians(config.fov_deg),
        )
        self.camera.SetName("vehicle_rear_gripper_camera")
        self.camera.SetLag(config.lag)
        self.camera.SetCollectionWindow(config.exposure_time)

        if config.visualize:
            self.camera.PushFilter(
                sens.ChFilterVisualize(config.width, config.height, "Gripper Camera")
            )
        if config.save_images:
            self.camera.PushFilter(sens.ChFilterSave(self.rgb_dir + os.sep))

        # Keep an access filter in the graph so the pipeline can write metadata
        # only when a new frame is actually available.
        self.camera.PushFilter(sens.ChFilterRGBA8Access())
        self.manager.AddSensor(self.camera)

    def update(self):
        self.manager.Update()

    def saved_image_path(self, frame: int) -> Optional[str]:
        if not self.config.save_images:
            return None
        # ChFilterSave names the first saved image frame_0.png while the access
        # buffer's launched count reports that frame as 1.
        return os.path.join(self.rgb_dir, f"frame_{max(frame - 1, 0)}.png")

    def write_new_frame_metadata(self, sink, sim_time: float, phase: str, gripper, rock):
        rgba = self.camera.GetMostRecentRGBA8Buffer()
        if not rgba.HasData():
            return None

        frame = int(rgba.LaunchedCount)
        if frame == self.last_frame:
            return None
        self.last_frame = frame

        camera_pos_abs = self.parent_body.TransformPointLocalToParent(self.offset_pose.GetPos())
        camera_rot_abs = self.parent_body.GetRot() * self.offset_pose.GetRot()
        image_path = self.saved_image_path(frame)
        row = {
            "frame": frame,
            "sensor_time": float(rgba.TimeStamp),
            "sim_time": float(sim_time),
            "phase": phase,
            "camera_name": "vehicle_rear_gripper_camera",
            "camera_parent_body": self.parent_body.GetName(),
            "camera_mount_reference_body": gripper.base.GetName(),
            "arm_base_pos_parent_local": vec_to_list(self.base_local),
            "camera_look_dir_parent_local": vec_to_list(self.look_dir_local),
            "camera_pos_world": vec_to_list(camera_pos_abs),
            "camera_rot_world_wxyz": quat_to_list(camera_rot_abs),
            "gripper_center_world": vec_to_list(gripper_center(gripper)),
            "rock_pos_world": vec_to_list(rock.GetPos()) if rock else None,
            "image_stream": "rgb",
            "image_directory": self.rgb_dir if self.config.save_images else None,
            "image_path": image_path,
        }
        sink.write(json.dumps(row) + "\n")
        sink.flush()
        return row


class KinematicCameraRig:
    """Camera-pose metadata only; used when Chrono Sensor is disabled."""

    def __init__(self, vehicle, gripper, config: CameraConfig):
        self.config = config
        self.parent_body = vehicle.GetChassisBody()
        self.offset_pose, self.base_local, self.look_dir_local = make_vehicle_rear_camera_pose(
            vehicle, gripper, config
        )
        self.frame = 0
        self.next_sample_time = 0.0
        self.rgb_dir = None

    def update(self):
        pass

    def write_new_frame_metadata(self, sink, sim_time: float, phase: str, gripper, rock):
        sample_period = 1.0 / max(self.config.update_rate, 1e-9)
        if sim_time + 1e-12 < self.next_sample_time:
            return

        camera_pos_abs = self.parent_body.TransformPointLocalToParent(self.offset_pose.GetPos())
        camera_rot_abs = self.parent_body.GetRot() * self.offset_pose.GetRot()
        row = {
            "frame": self.frame,
            "sensor_time": float(sim_time),
            "sim_time": float(sim_time),
            "phase": phase,
            "camera_name": "vehicle_rear_gripper_camera",
            "camera_parent_body": self.parent_body.GetName(),
            "camera_mount_reference_body": gripper.base.GetName(),
            "arm_base_pos_parent_local": vec_to_list(self.base_local),
            "camera_look_dir_parent_local": vec_to_list(self.look_dir_local),
            "camera_pos_world": vec_to_list(camera_pos_abs),
            "camera_rot_world_wxyz": quat_to_list(camera_rot_abs),
            "gripper_center_world": vec_to_list(gripper_center(gripper)),
            "rock_pos_world": vec_to_list(rock.GetPos()) if rock else None,
            "image_stream": None,
            "image_directory": None,
            "image_path": None,
            "sensor_enabled": False,
        }
        sink.write(json.dumps(row) + "\n")
        sink.flush()
        self.frame += 1
        self.next_sample_time += sample_period
        return row


def chrono_sensor_preflight_error() -> Optional[str]:
    """Return an error string if Chrono's OptiX camera path is unavailable."""
    if sens is None:
        return "pychrono.sensor could not be imported."

    probe = r"""
import pychrono as chrono
import pychrono.sensor as sens

system = chrono.ChSystemNSC()
body = chrono.ChBody()
body.SetFixed(True)
system.Add(body)
manager = sens.ChSensorManager(system)
camera = sens.ChCameraSensor(
    body,
    1.0,
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    16,
    16,
    1.0,
)
manager.AddSensor(camera)
print("OK")
"""
    result = subprocess.run(
        [sys.executable, "-c", probe],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=30,
    )
    if result.returncode == 0:
        return None

    output = (result.stdout + result.stderr).strip()
    hint = (
        "Chrono Sensor's ChCameraSensor is OptiX-backed in this PyChrono build. "
        "The installed NVIDIA driver/OptiX runtime is not compatible with the "
        "PyChrono package, so camera images cannot be rendered in this environment."
    )
    if "OPTIX_ERROR_UNSUPPORTED_ABI_VERSION" in output:
        hint += (
            " The specific failure is OPTIX_ERROR_UNSUPPORTED_ABI_VERSION, which "
            "usually means the NVIDIA driver is too old for the OptiX ABI used by "
            "the installed PyChrono build."
        )
    return f"{hint}\n\nChrono Sensor preflight output:\n{output}"


def sim_phase(time: float, gripped: bool, lift_angle: Optional[float]) -> str:
    if time < T_CLOSE:
        return "replay"
    if not gripped:
        return "close"
    if time < T_LIFT:
        return "locked_wait"
    if lift_angle != LIFT_THETA2:
        return "lift"
    return "hold"


def drive_arm_collect(
    system,
    vehicle,
    terrain,
    gripper,
    theta,
    camera_rig: GripperCameraRig,
    episode_dir: str,
    duration: float,
    vis=None,
):
    """Replay the demo's sim2 arm sequence while updating sensors and metadata."""
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_throttle = 0.0
    driver_inputs.m_steering = 0.0
    driver_inputs.m_braking = 1.0

    render_steps = math.ceil((1.0 / 30.0) / STEP_SIZE)
    moved_arm = bent_arm = gripped = False
    close_pos = 0.0
    lift_angle = None
    next_tick = 0.0
    steps = 0
    stopped_by_window = False

    if vis is not None:
        vehicle.EnableRealtime(True)

    rock = system.SearchBody("rock")
    frames_path = os.path.join(episode_dir, "frames.jsonl")
    initial_frame = None
    with open(frames_path, "w", encoding="utf-8") as frame_sink:
        camera_rig.update()
        initial_frame = camera_rig.write_new_frame_metadata(
            frame_sink,
            system.GetChTime(),
            "initial",
            gripper,
            rock,
        )

        while True:
            if vis is not None and not vis.Run():
                stopped_by_window = True
                break

            time = system.GetChTime()
            if duration > 0.0 and time >= duration:
                break

            system.DoStepDynamics(STEP_SIZE)

            if vis is not None and steps % render_steps == 0:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()

            if time > 1.0 and not moved_arm:
                gripper.open()
                gripper.rotate_motor(gripper.motor_base_shoulder, theta[0])
                gripper.rotate_motor(gripper.motor_shoulder_biceps, theta[1])
                moved_arm = True

            if time > 2.0 and not bent_arm:
                gripper.rotate_motor(gripper.motor_biceps_elbow, theta[2])
                gripper.rotate_motor(gripper.motor_elbow_eef, theta[3])
                bent_arm = True

            if time > T_CLOSE and time >= next_tick:
                next_tick = time + CTRL_DT
                if not gripped:
                    close_pos = min(close_pos + FINGER_CLOSE_SPEED * CTRL_DT, FINGER_CLOSE_POS)
                    gripper.move_linear_motor(gripper.motor_endoffactor_finger_1, -close_pos)
                    gripper.move_linear_motor(gripper.motor_endoffactor_finger_2, close_pos)
                    actual_sep = (gripper.finger_1.GetPos() - gripper.finger_2.GetPos()).Length()
                    commanded_sep = FINGER_OPEN_SEP - 2.0 * close_pos
                    stalled = actual_sep - commanded_sep > GRIP_STALL_TOL

                    if stalled or close_pos >= FINGER_CLOSE_POS:
                        if stalled:
                            close_pos = (FINGER_OPEN_SEP - actual_sep) / 2.0 + 0.002
                            gripper.move_linear_motor(
                                gripper.motor_endoffactor_finger_1, -close_pos
                            )
                            gripper.move_linear_motor(
                                gripper.motor_endoffactor_finger_2, close_pos
                            )
                        gripped = True
                        gripper.add_lock()
                        if gripper.cur_lock:
                            print(
                                f"  gripper closed (sep {actual_sep:.3f} m), "
                                f"'{gripper.cur_object}' locked to the end-effector "
                                f"(t={time:.2f} s)"
                            )
                        else:
                            print(f"  gripper closed but no object in range to lock (t={time:.2f} s)")

                elif time > T_LIFT and lift_angle != LIFT_THETA2:
                    if lift_angle is None:
                        lift_angle = theta[1]
                        print(
                            f"  lifting: theta 2 {math.degrees(theta[1]):.1f} -> "
                            f"{math.degrees(LIFT_THETA2):.1f} deg"
                        )
                    dtheta = LIFT_THETA2 - lift_angle
                    step = LIFT_SPEED * CTRL_DT
                    if abs(dtheta) <= step:
                        lift_angle = LIFT_THETA2
                    else:
                        lift_angle += math.copysign(step, dtheta)
                    gripper.rotate_motor(gripper.motor_shoulder_biceps, lift_angle)

            vehicle.Synchronize(time, driver_inputs, terrain)
            terrain.Synchronize(time)
            if vis is not None:
                vis.Synchronize(time, driver_inputs)

            camera_rig.update()
            phase = "initial" if initial_frame is None else sim_phase(time, gripped, lift_angle)
            frame_row = camera_rig.write_new_frame_metadata(
                frame_sink, system.GetChTime(), phase, gripper, rock
            )
            if initial_frame is None and frame_row is not None:
                initial_frame = frame_row

            vehicle.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            if vis is not None:
                vis.Advance(STEP_SIZE)

            steps += 1

    initial_rgb_path = None
    if initial_frame and initial_frame.get("image_path"):
        source_path = initial_frame["image_path"]
        if os.path.exists(source_path):
            initial_rgb_path = os.path.join(episode_dir, "initial_rgb.png")
            shutil.copyfile(source_path, initial_rgb_path)
            initial_frame["initial_rgb_path"] = initial_rgb_path
        else:
            initial_frame["initial_rgb_copy_error"] = f"saved image not found: {source_path}"

    finger_1_pos = gripper.finger_1.GetPos()
    finger_2_pos = gripper.finger_2.GetPos()
    return {
        "frames_metadata": frames_path,
        "initial_frame": initial_frame,
        "initial_rgb_path": initial_rgb_path,
        "stopped_by_window": stopped_by_window,
        "sim_time": float(system.GetChTime()),
        "final_gripper_center_world": vec_to_list(gripper_center(gripper)),
        "final_finger_1_pos_world": vec_to_list(finger_1_pos),
        "final_finger_2_pos_world": vec_to_list(finger_2_pos),
        "final_finger_separation_m": float((finger_1_pos - finger_2_pos).Length()),
        "final_rock_pos_world": vec_to_list(rock.GetPos()) if rock else None,
        "lock_name": gripper.cur_lock,
        "locked_object": gripper.cur_object,
    }


def write_json(path: str, payload) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
        f.write("\n")


def write_dataset_index(output_dir: str, episodes: list[dict]) -> str:
    index_path = os.path.join(output_dir, "index.jsonl")
    with open(index_path, "w", encoding="utf-8") as sink:
        for episode in episodes:
            sink.write(json.dumps(episode["sample_record"]) + "\n")
    return index_path


def run_episode(
    episode_idx: int,
    output_dir: str,
    episode_config: EpisodeConfig,
    camera_config: CameraConfig,
    debug_irrlicht: bool,
    use_sensors: bool,
):
    episode_seed = episode_config.seed + episode_idx
    random.seed(episode_seed)

    episode_dir = os.path.join(output_dir, f"episode_{episode_idx:06d}")
    os.makedirs(episode_dir, exist_ok=True)

    print(f"=== EPISODE {episode_idx:06d} / SIM 1: target sampling and record ===")
    system, vehicle, terrain, gripper = build_scene()
    ik_solver = RobotArmInverseKinematicsSolver()
    final_theta, ik_target, target_sample = sample_reachable_action(
        gripper, vehicle, terrain, ik_solver, episode_config
    )
    ee_pos = drive_arm(
        system,
        vehicle,
        terrain,
        gripper,
        final_theta,
        settle_time=episode_config.sim1_settle_time,
    )
    sim1_record = {
        "episode": episode_idx,
        "seed": episode_seed,
        "joint_angles_rad": [float(x) for x in final_theta],
        "action_theta_rad": [float(x) for x in final_theta],
        "action_theta_deg": [math.degrees(float(x)) for x in final_theta],
        "sampled_ik_target_world": vec_to_list(ik_target),
        "sampled_target": target_sample,
        "recorded_gripper_center_world": vec_to_list(ee_pos),
        "sim1_settle_time": episode_config.sim1_settle_time,
    }
    write_json(os.path.join(episode_dir, "sim1_record.json"), sim1_record)

    print(f"  recorded joint angles  : {final_theta}")
    print(f"  recorded gripper center: {ee_pos}")
    print(f"=== EPISODE {episode_idx:06d} / SIM 2: replay with gripper camera ===")

    system2, vehicle2, terrain2, gripper2 = build_scene(rock_pos=ee_pos)
    rock2 = system2.SearchBody("rock")
    initial_rock_pos_world = vec_to_list(rock2.GetPos()) if rock2 else None
    initial_rock_rot_world = quat_to_list(rock2.GetRot()) if rock2 else None
    gripper2.add_object("rock")

    vis = None
    if debug_irrlicht:
        vis = make_vis(vehicle2, "LRV Gripper Data Collection Debug")

    if use_sensors:
        camera_rig = GripperCameraRig(system2, vehicle2, gripper2, episode_dir, camera_config)
    else:
        camera_rig = KinematicCameraRig(vehicle2, gripper2, camera_config)
    print(f"  camera parent          : {camera_rig.parent_body.GetName()}")
    print(f"  arm base local         : {camera_rig.base_local}")
    print(f"  camera offset pos local: {camera_rig.offset_pose.GetPos()}")
    print(f"  camera look dir local  : {camera_rig.look_dir_local}")
    if use_sensors and camera_config.save_images:
        print(f"  RGB output             : {camera_rig.rgb_dir}")
    elif not use_sensors:
        print("  Chrono Sensor          : disabled (pose metadata only)")

    sim2_record = drive_arm_collect(
        system2,
        vehicle2,
        terrain2,
        gripper2,
        final_theta,
        camera_rig,
        episode_dir,
        episode_config.sim2_duration,
        vis=vis,
    )
    final_rock_pos = sim2_record["final_rock_pos_world"]
    final_gripper_center = sim2_record["final_gripper_center_world"]
    rock_lift_delta_m = (
        final_rock_pos[2] - initial_rock_pos_world[2]
        if final_rock_pos is not None and initial_rock_pos_world is not None
        else None
    )
    final_rock_to_gripper_m = (
        math.sqrt(sum((a - b) ** 2 for a, b in zip(final_rock_pos, final_gripper_center)))
        if final_rock_pos is not None and final_gripper_center is not None
        else None
    )
    picked_up = (
        sim2_record["locked_object"] == "rock"
        and rock_lift_delta_m is not None
        and rock_lift_delta_m > 0.25
    )
    sim2_record.update(
        {
            "initial_rock_pos_world": initial_rock_pos_world,
            "initial_rock_rot_world_wxyz": initial_rock_rot_world,
            "rock_lift_delta_m": rock_lift_delta_m,
            "final_rock_to_gripper_center_m": final_rock_to_gripper_m,
            "picked_up": picked_up,
        }
    )
    write_json(os.path.join(episode_dir, "sim2_record.json"), sim2_record)

    sample_record = {
        "episode": episode_idx,
        "seed": episode_seed,
        "episode_dir": relpath_or_none(episode_dir, output_dir),
        "sample_json": relpath_or_none(os.path.join(episode_dir, "sample.json"), output_dir),
        "image": {
            "initial_rgb": relpath_or_none(sim2_record["initial_rgb_path"], output_dir),
            "initial_frame": frame_record_with_relative_paths(
                sim2_record["initial_frame"], output_dir
            ),
            "width": camera_config.width,
            "height": camera_config.height,
            "fov_deg": camera_config.fov_deg,
        },
        "action": {
            "theta_rad": [float(x) for x in final_theta],
            "theta_deg": [math.degrees(float(x)) for x in final_theta],
        },
        "rock": {
            "sampled_ik_target_world": vec_to_list(ik_target),
            "recorded_gripper_center_world": vec_to_list(ee_pos),
            "initial_body_pos_world": initial_rock_pos_world,
            "initial_body_rot_world_wxyz": initial_rock_rot_world,
            "final_body_pos_world": final_rock_pos,
            "lift_delta_m": rock_lift_delta_m,
        },
        "gripper_final": {
            "center_world": final_gripper_center,
            "finger_1_pos_world": sim2_record["final_finger_1_pos_world"],
            "finger_2_pos_world": sim2_record["final_finger_2_pos_world"],
            "finger_separation_m": sim2_record["final_finger_separation_m"],
            "rock_to_gripper_center_m": final_rock_to_gripper_m,
        },
        "success": {
            "picked_up": picked_up,
            "locked_object": sim2_record["locked_object"],
            "lock_name": sim2_record["lock_name"],
        },
    }
    write_json(os.path.join(episode_dir, "sample.json"), sample_record)

    return {
        "episode": episode_idx,
        "seed": episode_seed,
        "episode_dir": episode_dir,
        "sim1_record": sim1_record,
        "sim2_record": sim2_record,
        "sample_record": sample_record,
    }


def parse_args():
    parser = argparse.ArgumentParser(
        description="Collect vehicle-mounted rear camera data for the LRV rock pickup scenario."
    )
    parser.add_argument("--episodes", type=int, default=1, help="Number of episodes to collect.")
    parser.add_argument("--seed", type=int, default=7, help="Base random seed.")
    parser.add_argument(
        "--output-dir",
        default=DEFAULT_OUTPUT_DIR,
        help="Dataset output directory.",
    )
    parser.add_argument(
        "--sim1-settle-time",
        type=float,
        default=4.0,
        help="Headless sim1 settling time before recording the gripper center.",
    )
    parser.add_argument(
        "--sim2-duration",
        type=float,
        default=10.0,
        help="Finite sim2 collection duration. Use 0 to run until the debug window closes.",
    )
    parser.add_argument(
        "--target-radius-min",
        type=float,
        default=2.0,
        help="Minimum random rock/grasp radius from the arm base, in meters.",
    )
    parser.add_argument(
        "--target-radius-max",
        type=float,
        default=3.0,
        help="Maximum random rock/grasp radius from the arm base, in meters.",
    )
    parser.add_argument(
        "--target-angle-min-deg",
        type=float,
        default=90.0,
        help="Minimum random rock/grasp polar angle around the arm base, in degrees.",
    )
    parser.add_argument(
        "--target-angle-max-deg",
        type=float,
        default=270.0,
        help="Maximum random rock/grasp polar angle around the arm base, in degrees.",
    )
    parser.add_argument(
        "--debug-irrlicht",
        action="store_true",
        help="Open the Irrlicht vehicle viewer during sim2.",
    )
    parser.add_argument(
        "--sensor-visualize",
        action="store_true",
        help="Open Chrono Sensor's camera preview window.",
    )
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=360)
    parser.add_argument("--camera-update-rate", type=float, default=15.0)
    parser.add_argument("--camera-fov-deg", type=float, default=90.0)
    parser.add_argument(
        "--camera-base-offset-x",
        type=float,
        default=0.9,
        help="Camera x offset from the arm-base center, in chassis-local meters.",
    )
    parser.add_argument(
        "--camera-base-offset-y",
        type=float,
        default=0.0,
        help="Camera y offset from the arm-base center, in chassis-local meters.",
    )
    parser.add_argument(
        "--camera-base-offset-z",
        type=float,
        default=2.6,
        help="Camera z offset from the arm-base center, in chassis-local meters.",
    )
    parser.add_argument(
        "--camera-backward-pitch-deg",
        type=float,
        default=50.0,
        help="Downward pitch while looking backward along chassis -X.",
    )
    parser.add_argument(
        "--no-save-images",
        action="store_true",
        help="Disable RGB image saving while keeping metadata and sensor access active.",
    )
    parser.add_argument(
        "--no-sensors",
        action="store_true",
        help="Disable Chrono Sensor/OptiX camera rendering; run sim2 and write camera pose metadata only.",
    )
    parser.add_argument(
        "--require-sensors",
        action="store_true",
        help="Fail if Chrono Sensor/OptiX camera rendering is unavailable instead of falling back to pose metadata.",
    )
    args = parser.parse_args()
    if args.episodes < 0:
        parser.error("--episodes must be nonnegative.")
    if args.target_radius_min <= 0.0 or args.target_radius_max <= 0.0:
        parser.error("--target-radius-min and --target-radius-max must be positive.")
    if args.sim2_duration <= 0.0 and not args.debug_irrlicht:
        parser.error("--sim2-duration must be positive unless --debug-irrlicht is set.")
    if args.no_sensors and args.sensor_visualize:
        parser.error("--sensor-visualize requires Chrono Sensor; remove it or omit --no-sensors.")
    if args.no_sensors and args.require_sensors:
        parser.error("--no-sensors and --require-sensors are mutually exclusive.")
    return args


def main():
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    use_sensors = not args.no_sensors
    sensor_preflight = None
    if use_sensors and args.episodes > 0:
        sensor_preflight = chrono_sensor_preflight_error()
        if sensor_preflight:
            if args.require_sensors:
                raise SystemExit(
                    sensor_preflight
                    + "\n\nRGB collection is required for this run. Update the NVIDIA "
                    + "driver or use a PyChrono build compatible with the installed "
                    + "driver/OptiX runtime."
                )

            print(sensor_preflight, file=sys.stderr)
            print(
                "\nWARNING: Falling back to pose-only camera metadata. "
                "No RGB images or Chrono Sensor preview will be produced. "
                "Use --require-sensors to make this a hard failure.",
                file=sys.stderr,
            )
            use_sensors = False

    episode_config = EpisodeConfig(
        sim1_settle_time=args.sim1_settle_time,
        sim2_duration=args.sim2_duration,
        seed=args.seed,
        target_radius_min=args.target_radius_min,
        target_radius_max=args.target_radius_max,
        target_angle_min_deg=args.target_angle_min_deg,
        target_angle_max_deg=args.target_angle_max_deg,
    )
    camera_config = CameraConfig(
        width=args.camera_width,
        height=args.camera_height,
        update_rate=args.camera_update_rate,
        fov_deg=args.camera_fov_deg,
        base_offset_x=args.camera_base_offset_x,
        base_offset_y=args.camera_base_offset_y,
        base_offset_z=args.camera_base_offset_z,
        backward_pitch_deg=args.camera_backward_pitch_deg,
        visualize=use_sensors and args.sensor_visualize,
        save_images=use_sensors and not args.no_save_images,
    )

    manifest = {
        "scenario": "LRV gripper rock pickup",
        "flow": "sim1 headless record, sim2 replay with vehicle-mounted rear camera",
        "output_dir": args.output_dir,
        "index_file": os.path.join(args.output_dir, "index.jsonl"),
        "episode_config": asdict(episode_config),
        "camera_config": asdict(camera_config),
        "debug_irrlicht": bool(args.debug_irrlicht),
        "sensors_enabled": use_sensors,
        "sensor_preflight": (
            "ok"
            if use_sensors and args.episodes > 0
            else "failed_fallback" if sensor_preflight else "skipped"
        ),
        "sensor_preflight_error": sensor_preflight,
        "episodes": [],
    }
    write_json(os.path.join(args.output_dir, "manifest.json"), manifest)
    write_dataset_index(args.output_dir, manifest["episodes"])

    for episode_idx in range(args.episodes):
        result = run_episode(
            episode_idx,
            args.output_dir,
            episode_config,
            camera_config,
            args.debug_irrlicht,
            use_sensors,
        )
        manifest["episodes"].append(result)
        write_dataset_index(args.output_dir, manifest["episodes"])
        write_json(os.path.join(args.output_dir, "manifest.json"), manifest)

    print(f"Dataset manifest: {os.path.join(args.output_dir, 'manifest.json')}")
    print(f"Dataset index   : {os.path.join(args.output_dir, 'index.jsonl')}")


if __name__ == "__main__":
    main()
