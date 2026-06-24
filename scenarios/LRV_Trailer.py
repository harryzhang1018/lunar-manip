"""LRV + dumping trailer + rock -- static scene (brakes held, arm idle).

A wheeled lunar rover (LRV / Polaris) with the gripper arm welded to the back of
the chassis AND a wagon (a Chrono `WheeledTrailer`) hitched behind it, standing
(braked) on flat terrain with a rock resting on the ground nearby.

The scene is set up like `LRV_Arm.py`, with these differences: a trailer is hitched
at the back, the arm is *not* moved, and the trailer bed is mounted on the trailer
chassis by a powered hinge instead of a rigid weld. The rock is spawned at a random
ground position in polar coordinates around the arm base (the rear-left/rear-right
wedges, skipping straight behind where the trailer is), like `LRV_Arm.py`'s
reachable sampling. The vehicle just holds its brakes while the bed runs one
`dump()` cycle -- tilting up about the trailer's lateral (Y) axis and back -- to
exercise the hinge. (No IK, no grab; the arm is idle, and the rock is not yet
loaded onto the bed.)

The trailer (data/vehicle/LRV_Wagon/Polaris.json) is a rear chassis (a primitive
bed on a leafspring axle) connected to the rover's chassis through a
`ChassisConnectorHitch`. The collidable bed surface (`trailer_chassis_open.obj`) is
the `TrailerDumpBed` below: a mesh body joined to the trailer chassis by a
`ChLinkMotorRotationAngle` revolute motor about the chassis Y axis (X forward, Z
up), so `dump()` raises it.

`LRV_Arm` comes from the `model` package; the rock recipe (`place_rock`) is shared
with `LRV_Arm.py`.

Run with the project's conda env:

    conda run -n chrono python scenarios/LRV_Trailer.py

Add `--headless` to step the simulation without opening a render window (used for
smoke tests). Add `--vsg` to render with the VSG (Vulkan) backend instead of the
default Irrlicht (OpenGL) one.
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
# Reuse the rock recipe (scaled Curiosity mesh, sphere-swept collision) from the
# arm scenario so the rock here is identical.
from LRV_Arm import place_rock

# Initial vehicle pose and integration step.
INIT_LOC = chrono.ChVector3d(-83, -85, 0.5)
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)
STEP_SIZE = 1e-3

# Uniform geometric scale of the gripper arm (1.0 = as exported). Forwarded to
# both LRV_Arm and its IK solver; the grasp tuning and rock-spawn radius below
# scale with it. Mass/inertia stay at 1x values (geometry-only scaling).
ARM_SCALE = 1.0

# Trailer (WheeledTrailer) and the dumping bed mounted on top of it.
TRAILER_FILE = "LRV_Wagon/Polaris.json"
TRAILER_BED_MESH = "LRV_Wagon/trailer_chassis_open.obj"
TRAILER_BED_OFFSET = chrono.ChVector3d(0, 0, 0.05)  # bed sits just above the chassis
# Rock spawn: a random ground position in polar coordinates around the arm base
# (like LRV_Arm.py's reachable sampling). Theta covers the rear-left and rear-right
# wedges -- [90, 90+60] and [270-60, 270] deg -- skipping straight behind, where
# the trailer sits; r is the distance from the base.
ROCK_THETA_RANGES_DEG = ((70.0, 120.0), (240.0, 290.0))
ROCK_R_RANGE = (2.0 * ARM_SCALE, 2.75 * ARM_SCALE)

# Grasp-and-place sequence: hold the brakes, let the scene settle, then pick the
# rocks up one at a time and drop each on the trailer.
NUM_ROCKS = 3         # how many rocks to spawn and load
ROCK_MIN_SEP = 0.8 * ARM_SCALE    # keep spawned rocks at least this far apart (m)
T_GRASP_START = 1.0   # begin the first grasp once the rover and rocks have settled
# Drop point for each rock: the gripper is moved to the trailer chassis center plus
# this offset, then released, so the rock falls onto the bed. Successive rocks are
# spread laterally (PLACE_SPREAD_Y) so they don't pile on the same spot.
PLACE_OFFSET = chrono.ChVector3d(0.0, 0, 0.5)
PLACE_SPREAD_Y = 0.4

# After every rock is loaded, the rover drives off, stops, and dumps the load:
# drive forward for DRIVE_TIME s (throttle DRIVE_THROTTLE, steering DRIVE_STEERING),
# brake to a stop for STOP_TIME s, then run one dump() cycle to unload the bed.
# The throttle is eased in over DRIVE_RAMP s (rather than slammed to full at t=0) and
# kept gentle so the rocks ride along in the shallow tub instead of tumbling out, so
# the bed still has its load when dump() tips it off.
DRIVE_TIME = 6.0
DRIVE_THROTTLE = 0.2
DRIVE_STEERING = 0.1
DRIVE_RAMP = 3.0
STOP_TIME = 1.0

# Headless smoke-test duration (seconds of sim time): long enough to cover NUM_ROCKS
# pick-and-place cycles (~15 s each, worst case), the drive-off + stop + dump cycle,
# and settle. With a render window the scene instead holds until the window is closed.
HEADLESS_RUN_TIME = 80.0


class TrailerDumpBed:
    """Collidable trailer bed mounted on the trailer chassis by a Y-axis hinge.

    Instead of rigidly welding the bed to the trailer chassis, the bed is joined
    by a `ChLinkMotorRotationAngle` revolute motor whose axis is the chassis
    lateral (Y) axis (X forward, Z up). One call to `dump()` runs the whole cycle:
    from horizontal the bed tilts up to DUMP_MAX (over RISE_TIME), holds there so
    the load slides off (HOLD_TIME), then returns to horizontal (over LOWER_TIME).
    The motor follows a time-based angle profile, so the cycle plays out on its own
    as the simulation advances -- no per-step calls needed.
    """

    DUMP_MAX = math.radians(55.0)   # fully-raised bed angle
    RISE_TIME = 1.0                 # horizontal -> dumped (s)
    HOLD_TIME = 2.0                 # hold at the dump angle (s)
    LOWER_TIME = 1.0                # dumped -> horizontal (s)
    BED_WALL_THICK = 0.03           # thickness of the bed's box-primitive floor/walls (m)

    def __init__(self, system, chassis_body, mesh_file, offset, material):
        self.system = system
        # The bed's *visual* is the open-tub mesh (a floor plus three walls: front
        # +x, left +y, right -y; open top and open -x rear). Its *collision* is
        # built from box primitives tracing those same faces -- a thin floor box
        # plus the three side walls -- because box-vs-mesh contact is robust in NSC,
        # whereas a triangle-mesh collision lets the (mesh) rock tunnel through.
        mesh = chrono.ChTriangleMeshConnected.CreateFromWavefrontFile(mesh_file, False, True)
        bb = mesh.GetBoundingBox()
        ex, ey, ez = bb.max.x - bb.min.x, bb.max.y - bb.min.y, bb.max.z - bb.min.z

        self.bed = chrono.ChBody()
        self.bed.SetName("trailer_bed")
        self.bed.SetPos(chassis_body.GetPos() + offset)
        # Mass/inertia of an equivalent solid slab over the mesh bounding box.
        mass = 1000.0 * ex * ey * ez
        self.bed.SetMass(mass)
        self.bed.SetInertiaXX(chrono.ChVector3d(mass / 12.0 * (ey * ey + ez * ez),
                                                mass / 12.0 * (ex * ex + ez * ez),
                                                mass / 12.0 * (ex * ex + ey * ey)))

        bed_vis = chrono.ChVisualShapeTriangleMesh()
        bed_vis.SetMesh(mesh)
        bed_vis.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
        self.bed.AddVisualShape(bed_vis)

        # Box-primitive collision tracing the tub: a floor (its top at the mesh
        # floor) plus the three walls (centered on the mesh wall faces).
        t = self.BED_WALL_THICK
        self._add_box(material, ex, ey, t, 0.0, 0.0, bb.min.z - t / 2.0)  # floor
        self._add_box(material, t, ey, ez, bb.max.x, 0.0, 0.0)           # +x front wall
        self._add_box(material, ex, t, ez, 0.0, bb.max.y, 0.0)           # +y left wall
        self._add_box(material, ex, t, ez, 0.0, bb.min.y, 0.0)           # -y right wall
        self.bed.EnableCollision(True)
        system.Add(self.bed)

        # Revolute motor about the chassis Y axis. A Chrono rotation motor turns
        # about the Z axis of its frame, so rotate the frame +90 deg about X to
        # land that Z on the chassis -Y axis (composed with the chassis rotation so
        # the hinge stays lateral whatever way the trailer is pointing). With this
        # sense a positive dump angle lifts the front of the bed and drops the
        # rear, so the load slides off the back -- away from the rover and arm.
        pivot = chassis_body.GetPos() + offset
        frame_rot = chassis_body.GetRot() * chrono.QuatFromAngleX(math.pi / 2)
        self.motor = chrono.ChLinkMotorRotationAngle()
        self.motor.SetName("trailer_bed_dump_motor")
        self.motor.Initialize(self.bed, chassis_body, chrono.ChFramed(pivot, frame_rot))
        self.motor.SetAngleFunction(chrono.ChFunctionConst(0.0))  # start horizontal
        system.Add(self.motor)

    def _add_box(self, material, sx, sy, sz, cx, cy, cz):
        """Add a box collision shape (full lengths sx,sy,sz) at local center (cx,cy,cz)."""
        shape = chrono.ChCollisionShapeBox(material, sx, sy, sz)
        self.bed.AddCollisionShape(
            shape, chrono.ChFramed(chrono.ChVector3d(cx, cy, cz), chrono.QUNIT))

    def dump(self):
        """Run one dump cycle: horizontal -> dumped -> hold -> back to horizontal.

        Installs a time-based angle profile on the motor starting at the current
        simulation time, so the bed rises to DUMP_MAX over RISE_TIME, holds for
        HOLD_TIME (load slides off), then returns to horizontal over LOWER_TIME --
        all driven by the solver as the sim advances.
        """
        a = self.DUMP_MAX
        profile = chrono.ChFunctionSequence()
        profile.InsertFunct(chrono.ChFunctionRamp(0.0, a / self.RISE_TIME),
                            self.RISE_TIME, 1, True)        # rise: 0 -> a
        profile.InsertFunct(chrono.ChFunctionConst(a),
                            self.HOLD_TIME, 1, True)        # hold at a
        profile.InsertFunct(chrono.ChFunctionRamp(a, -a / self.LOWER_TIME),
                            self.LOWER_TIME, 1, True)       # lower: a -> 0
        profile.InsertFunct(chrono.ChFunctionConst(0.0),
                            1.0e6, 1, True)                 # stay horizontal
        profile.SetStartArg(self.system.GetChTime())        # phase to "now"
        profile.Setup()
        self.motor.SetAngleFunction(profile)

    def reset(self):
        """Force the bed back to flat (cancels any running dump cycle)."""
        self.motor.SetAngleFunction(chrono.ChFunctionConst(0.0))

    @property
    def angle(self):
        """Current commanded hinge angle (rad)."""
        return self.motor.GetMotorAngle()


class TrailerArm(LRV_Arm):
    """LRV gripper arm with a one-call grasp-and-lift sequence.

    `grasp(rock, place_pos=...)` drives the whole sequence -- inverse kinematics to
    the rock, approach, close the fingers, lock the rock to the end-effector, lift,
    and (if a `place_pos` is given) carry it to that world point, release it (open
    the fingers and drop the lock), and finally return the arm to a home pose
    (STOW_THETA) so the next grasp starts from a known configuration -- mirroring
    the staged grab in `LRV_Arm.py`. It is a *non-blocking* state machine: call it every simulation
    step (the scenario loop already steps the world). The timeline is keyed on the
    time since grasp() was first called for a rock, so passing a *new* rock restarts
    the sequence -- the same gripper can pick and place one rock after another in a
    single run. `grasp()` returns True once the sequence is finished (rock lifted,
    or released at `place_pos`).
    """

    # Grasp target: aim the gripper GRAB_HEIGHT above the ground under the rock
    # (the terrain here is flat at z = 0, as in LRV_Arm.py).
    GRAB_HEIGHT = 0.22

    # Staged-grasp timeline (seconds since grasp() began, unless noted otherwise).
    T_APPROACH_2 = 1.0      # bend the elbow/wrist after the base/shoulder swing
    GRASP_REACH_TOL = 0.15  # start closing once the gripper reaches the grab point...
    T_CLOSE_TIMEOUT = 8.0   # ...or after this long (fallback if it never quite arrives)
    LIFT_DELAY = 1.5        # lift this long after the rock is gripped
    PLACE_TOL = 0.15        # release once the gripper is this close to the drop point
    T_PLACE_MOVE = 6.0      # ...or after this long, whichever comes first (timeout)
    STOW_THETA = (-math.pi, math.pi / 5, math.pi / 4 , 0.0)  # home pose [t1,t2,t3,t4] after a place
    STOW_DELAY = 1.0        # hold over the bed this long after releasing (rock settles) before stowing
    T_STOW = 2.0            # hold the stow pose this long, then the grasp is done

    # Finger-close + lift tuning (identical to LRV_Arm.py).
    CTRL_DT = 0.01
    FINGER_OPEN_SEP = 0.388            # finger COM separation at open (motor pos 0)
    FINGER_CLOSE_POS = 0.145           # max finger travel from open
    FINGER_CLOSE_SPEED = 0.1           # m/s per finger
    GRIP_STALL_TOL = 0.012             # command-vs-actual lag that means "pads on rock"
    LIFT_THETA2 = math.radians(60.0)   # shoulder (theta 2) target for the lift
    LIFT_SPEED = 0.5                   # rad/s shoulder ramp

    def __init__(self, system, pos, attached_vehicle=None, ik_solver=None, scale=1.0):
        super().__init__(system, pos, attached_vehicle, scale=scale)
        self.attached_vehicle = attached_vehicle
        self.ik_solver = ik_solver or RobotArmInverseKinematicsSolver(scale=scale)
        # Geometric tuning scales with the arm; angles, times, and rad/s speeds do
        # not. These shadow the class constants of the same name.
        for attr in ("GRAB_HEIGHT", "GRASP_REACH_TOL", "PLACE_TOL",
                     "FINGER_OPEN_SEP", "FINGER_CLOSE_POS",
                     "FINGER_CLOSE_SPEED", "GRIP_STALL_TOL"):
            setattr(self, attr, getattr(self, attr) * scale)
        self._grasp_target = None
        self._grasp_done = True
        self._place_pos = None

    def _ik_to(self, world_point):
        """IK joint angles [theta1..4] that put the gripper at world `world_point`.

        Solves in the arm-base frame (oriented with the chassis). Raises ValueError
        if the point is out of reach.
        """
        base = self.base.GetPos()
        vir_base = chrono.ChBody()
        vir_base.SetPos(base)
        vir_base.SetRot(self.attached_vehicle.GetChassisBody().GetRot())  # arm-base frame
        des = vir_base.TransformPointParentToLocal(world_point)
        with contextlib.redirect_stdout(io.StringIO()):  # hush solver chatter
            return self.ik_solver.inverse_kinematics_solver([des.x, des.y, des.z],
                                                            elbow_up=True)

    def plan_grasp(self, x, y):
        """IK angles to bring the gripper GRAB_HEIGHT above (x, y); may raise ValueError.

        Used both to pre-check that a rock is reachable (at scene build) and to
        drive grasp().
        """
        return self._ik_to(chrono.ChVector3d(x, y, self.GRAB_HEIGHT))

    def grasp(self, rock, place_pos=None):
        """Advance the pick-and-place sequence for `rock`; return True when finished.

        Call every simulation step. With `place_pos` (a world `ChVector3d`), the
        rock is carried there after the lift and released; without it, the sequence
        stops after the lift. Passing a rock different from the one in progress
        (re)starts the sequence for it.
        """
        if rock is not self._grasp_target:
            self._begin_grasp(rock)
        self._place_pos = place_pos
        self._advance_grasp()
        return self._grasp_done

    # ---- internals ----
    def _begin_grasp(self, rock):
        """Plan the grasp for `rock` and reset the staged sequence to its start."""
        self._grasp_target = rock
        self._grasp_t0 = self.system.GetChTime()
        self._moved_arm = self._bent_arm = self._closing = self._gripped = False
        self._lifted = self._placed = False
        self._grasp_done = False
        self._close_pos = 0.0
        self._grip_time = None
        self._lift_angle = None
        self._place_theta = None
        self._place_t0 = None
        self._release_time = None
        self._stow_t0 = None
        self._next_tick = 0.0
        # World point the gripper must reach before closing (GRAB_HEIGHT above the rock).
        self._grab_target = chrono.ChVector3d(rock.GetPos().x, rock.GetPos().y, self.GRAB_HEIGHT)
        if rock.GetName() not in self.objects:
            self.add_object(rock.GetName())  # register with the lock logic
        self.open()  # release anything held and open the fingers
        try:
            self._theta = self.plan_grasp(rock.GetPos().x, rock.GetPos().y)
            print(f"  grasp '{rock.GetName()}': IK theta = "
                  f"{[round(float(t), 3) for t in self._theta]}")
        except ValueError:
            print(f"  grasp '{rock.GetName()}': out of reach -- skipping")
            self._theta = None
            self._grasp_done = True

    def _advance_grasp(self):
        """One step of the staged sequence (approach -> close -> lock -> lift -> place)."""
        if self._grasp_done or self._theta is None:
            return  # nothing to do; the motors hold their last command

        t = self.system.GetChTime() - self._grasp_t0

        # ---- approach: swing base/shoulder, then bend elbow/wrist (staggered) ----
        if not self._moved_arm:
            self.rotate_motor(self.motor_base_shoulder, self._theta[0])
            self.rotate_motor(self.motor_shoulder_biceps, self._theta[1])
            self._moved_arm = True
        if t > self.T_APPROACH_2 and not self._bent_arm:
            self.rotate_motor(self.motor_biceps_elbow, self._theta[2])
            self.rotate_motor(self.motor_elbow_eef, self._theta[3])
            self._bent_arm = True

        # ---- last mile: once the elbow/wrist are commanded, close when the gripper
        # has actually reached the grab point (or a timeout), lock, then lift. The
        # reach gate matters because a grasp may start from anywhere -- e.g. the
        # elevated pose left after placing the previous rock -- so a fixed delay
        # would close on empty air before the arm arrives. ----
        if not self._lifted and self._bent_arm and self.system.GetChTime() >= self._next_tick:
            self._next_tick = self.system.GetChTime() + self.CTRL_DT
            if not self._gripped:
                if not self._closing:
                    gripper_center = (self.finger_1.GetPos() + self.finger_2.GetPos()) * 0.5
                    if ((gripper_center - self._grab_target).Length() < self.GRASP_REACH_TOL
                            or t > self.T_CLOSE_TIMEOUT):
                        self._closing = True  # latched: keep closing once started
                if self._closing:
                    self._close_step()
            elif self.system.GetChTime() - self._grip_time > self.LIFT_DELAY:
                self._lift_step()

        # ---- place: once lifted, carry the rock to place_pos and release it ----
        if self._lifted and not self._placed:
            self._place_step()

        # ---- stow: after releasing, return to the home pose and hold, then finish ----
        if self._placed and not self._grasp_done:
            self._stow_step()

    def _close_step(self):
        """Close the fingers a notch; lock the rock once the pads stall on it."""
        self._close_pos = min(self._close_pos + self.FINGER_CLOSE_SPEED * self.CTRL_DT,
                              self.FINGER_CLOSE_POS)
        self.move_linear_motor(self.motor_endoffactor_finger_1, -self._close_pos)
        self.move_linear_motor(self.motor_endoffactor_finger_2, self._close_pos)
        actual_sep = (self.finger_1.GetPos() - self.finger_2.GetPos()).Length()
        stalled = actual_sep - (self.FINGER_OPEN_SEP - 2 * self._close_pos) > self.GRIP_STALL_TOL
        if stalled or self._close_pos >= self.FINGER_CLOSE_POS:
            if stalled:
                # Park the command where the fingers actually are (plus a light 2 mm
                # bite) so the position motors hold the rock instead of crushing it.
                self._close_pos = (self.FINGER_OPEN_SEP - actual_sep) / 2.0 + 0.002
                self.move_linear_motor(self.motor_endoffactor_finger_1, -self._close_pos)
                self.move_linear_motor(self.motor_endoffactor_finger_2, self._close_pos)
            self._gripped = True
            self._grip_time = self.system.GetChTime()
            self.add_lock()  # lock the registered rock within range to the end-effector
            if self.cur_lock:
                print(f"  grasp: '{self.cur_object}' locked (sep {actual_sep:.3f} m)")
            else:
                print("  grasp: fingers closed but nothing in range to lock")

    def _lift_step(self):
        """Ramp the shoulder (theta 2) up to LIFT_THETA2 to lift the rock."""
        if self._lift_angle is None:
            self._lift_angle = self._theta[1]
            print(f"  grasp: lifting (theta 2 -> {math.degrees(self.LIFT_THETA2):.0f} deg)")
        d = self.LIFT_THETA2 - self._lift_angle
        if abs(d) <= self.LIFT_SPEED * self.CTRL_DT:
            self._lift_angle = self.LIFT_THETA2
            self._lifted = True
            print("  grasp: lift complete")
        else:
            self._lift_angle += math.copysign(self.LIFT_SPEED * self.CTRL_DT, d)
        self.rotate_motor(self.motor_shoulder_biceps, self._lift_angle)

    def _place_step(self):
        """Carry the rock to `place_pos` (IK once), then open the fingers and release."""
        if self._place_pos is None:
            self._grasp_done = True  # no place target: stop after the lift
            return
        if self._place_theta is None:
            # Plan the move to the drop point and command all four joints toward it.
            try:
                self._place_theta = self._ik_to(self._place_pos)
            except ValueError:
                print("  place: drop point out of reach -- releasing in place")
                self.open()
                self._placed = True  # stow phase runs next
                self._release_time = self.system.GetChTime()
                return
            self._place_t0 = self.system.GetChTime()
            self.rotate_motor(self.motor_base_shoulder, self._place_theta[0])
            self.rotate_motor(self.motor_shoulder_biceps, self._place_theta[1])
            self.rotate_motor(self.motor_biceps_elbow, self._place_theta[2])
            self.rotate_motor(self.motor_elbow_eef, self._place_theta[3])
            print(f"  place: moving gripper to {self._place_pos}")
        else:
            # Release once the gripper actually reaches the drop point (the base may
            # have to swing most of the way around), or after a timeout.
            gripper_center = (self.finger_1.GetPos() + self.finger_2.GetPos()) * 0.5
            reached = (gripper_center - self._place_pos).Length() < self.PLACE_TOL
            if reached or self.system.GetChTime() - self._place_t0 > self.T_PLACE_MOVE:
                self.open()  # releases the fingers AND removes the lock constraint
                self._placed = True  # stow phase runs next
                self._release_time = self.system.GetChTime()
                print(f"  place: released ({'reached' if reached else 'timed out'}) "
                      f"-- fingers open, lock removed")

    def _stow_step(self):
        """Hold over the bed briefly so the rock settles, then return the arm to the
        home pose (STOW_THETA), hold T_STOW, and finish."""
        if self.system.GetChTime() - self._release_time < self.STOW_DELAY:
            return  # keep holding the place pose so the dropped rock settles first
        if self._stow_t0 is None:
            self._stow_t0 = self.system.GetChTime()
            self.rotate_motor(self.motor_base_shoulder, self.STOW_THETA[0])
            self.rotate_motor(self.motor_shoulder_biceps, self.STOW_THETA[1])
            self.rotate_motor(self.motor_biceps_elbow, self.STOW_THETA[2])
            self.rotate_motor(self.motor_elbow_eef, self.STOW_THETA[3])
            print("  stow: returning to home pose "
                  f"{[round(math.degrees(a)) for a in self.STOW_THETA]} deg")
        elif self.system.GetChTime() - self._stow_t0 > self.T_STOW:
            self._grasp_done = True
            print("  stow: home pose held -- grasp complete")


def sample_rock_position(base, terrain):
    """Random ground position for the rock, in polar coordinates around `base`.

    Mirrors `LRV_Arm.py`'s reachable sampling, but theta is drawn from the
    rear-left and rear-right wedges (ROCK_THETA_RANGES_DEG) -- avoiding straight
    behind, where the trailer sits -- and r spans ROCK_R_RANGE. Returns a world
    `ChVector3d` resting on the terrain. The polar angle is measured in world axes;
    at the initial (identity) heading that matches the chassis frame (X forward).
    """
    lo, hi = random.choice(ROCK_THETA_RANGES_DEG)
    theta = math.radians(random.uniform(lo, hi))
    r = random.uniform(*ROCK_R_RANGE)
    tx = base.x + r * math.cos(theta)
    ty = base.y + r * math.sin(theta)
    ground_z = terrain.GetHeight(chrono.ChVector3d(tx, ty, base.z + 5.0))
    return chrono.ChVector3d(tx, ty, ground_z)


def build_scene():
    """Create a system with the LRV vehicle, welded arm, dumping trailer, terrain, rocks.

    Returns (system, vehicle, trailer, terrain, gripper, dump_bed, rocks).
    """
    data_vehicle = os.path.join(project_root, "data", "vehicle") + os.sep
    veh.SetVehicleDataPath(data_vehicle)
    # The vehicle/trailer JSONs reference their meshes relative to the Chrono
    # data root.
    chrono.SetChronoDataPath(data_vehicle)

    # ---- Rover ----
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

    # ---- Gripper arm welded to the back of the chassis ----
    arm_offset = vehicle.GetChassis().GetPos() + chrono.ChVector3d(-1.1, 0, 0.1)
    gripper = TrailerArm(system, arm_offset, vehicle, scale=ARM_SCALE)

    # ---- Towed trailer hitched behind the rover ----
    trailer = veh.WheeledTrailer(system, veh.GetVehicleDataFile(TRAILER_FILE))
    trailer.Initialize(vehicle.GetChassis())  # hitches to the rover's chassis
    trailer.GetChassis().SetFixed(False)
    # The trailer chassis itself is not drawn; the dumping bed (TrailerDumpBed)
    # provides the visible/collidable deck on top of it.
    trailer.SetChassisVisualizationType(chrono.VisualizationType_NONE)
    trailer.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    for axle in trailer.GetAxles():
        for wheel in axle.GetWheels():
            tire = veh.ReadTireJSON(veh.GetVehicleDataFile("LRV/Polaris_RigidTire.json"))
            trailer.InitializeTire(tire, wheel, chrono.VisualizationType_MESH)

    # The collidable bed surface (the trailer JSON's chassis visualization is
    # primitives only), mounted on a Y-axis dump hinge.
    bed_mat = chrono.ChContactMaterialNSC()
    bed_mat.SetFriction(0.7)
    dump_bed = TrailerDumpBed(system, trailer.GetChassis().GetBody(),
                              veh.GetVehicleDataFile(TRAILER_BED_MESH),
                              TRAILER_BED_OFFSET, bed_mat)

    # ---- Flat rigid patch (100 x 100 m) at ground level under the vehicle ----
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

    # ---- Rocks dropped on the ground at random *reachable* spots (polar around
    # the arm base), each uniquely named. Re-sample each until the gripper's IK can
    # reach it (r near 3 m can be out of the arm's ~2.7 m span, as in LRV_Arm.py)
    # and it is clear (ROCK_MIN_SEP) of the rocks already placed. ----
    rocks = []
    placed_xy = []
    for i in range(NUM_ROCKS):
        for _attempt in range(80):
            rock_pos = sample_rock_position(gripper.base.GetPos(), terrain)
            try:
                gripper.plan_grasp(rock_pos.x, rock_pos.y)
            except ValueError:
                continue  # out of reach -> resample
            if all(math.hypot(rock_pos.x - px, rock_pos.y - py) > ROCK_MIN_SEP
                   for px, py in placed_xy):
                break     # reachable and well clear of the other rocks
        else:
            raise RuntimeError("no reachable, well-separated rock position found")
        rock = place_rock(system, rock_pos, ground_z=rock_pos.z)
        rock.SetName(f"rock_{i}")  # unique name so the gripper locks the right one
        rocks.append(rock)
        placed_xy.append((rock_pos.x, rock_pos.y))

    return system, vehicle, trailer, terrain, gripper, dump_bed, rocks


def make_vis(vehicle, title, use_vsg=False):
    """Create and initialize a visualization attached to `vehicle`.

    Defaults to Irrlicht (OpenGL); pass `use_vsg=True` for the VSG (Vulkan) backend.
    Both expose the same run/render/step interface, so only the setup differs: VSG
    has its own lighting/sky API and wants `AttachVehicle()` before `Initialize()`.
    """
    if use_vsg:
        vis = veh.ChWheeledVehicleVisualSystemVSG()
        vis.SetWindowTitle(title)
        vis.SetWindowSize(1280, 1024)
        # vis.EnableSkyTexture()
        vis.SetLightIntensity(1.0)
        vis.SetLightDirection(2.0, 0.75)
        vis.EnableShadows()
        vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 2.75), 15.0, 1.5)
        vis.AttachVehicle(vehicle)
        vis.Initialize()
        return vis

    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle(title)
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 2.0, 2.75), 10.0, 1.5)
    vis.Initialize()
    # Light above the (stationary) vehicle, aimed at it, so the work area is lit.
    vis.AddLightWithShadow(INIT_LOC + chrono.ChVector3d(0, 0, 30), INIT_LOC,
                           20, 1, 60, 50)
    vis.AddTypicalLights()
    vis.AttachVehicle(vehicle)
    return vis


def place_point(trailer, idx, n):
    """Drop point on the bed for rock `idx` of `n`: chassis center + PLACE_OFFSET,
    spread laterally so successive rocks land in different spots."""
    lateral = (idx - (n - 1) / 2.0) * PLACE_SPREAD_Y
    return (trailer.GetChassis().GetBody().GetPos()
            + PLACE_OFFSET + chrono.ChVector3d(0, lateral, 0))


def brake_and_grasp(system, vehicle, trailer, terrain, gripper, dump_bed, rocks,
                    vis=None, run_time=HEADLESS_RUN_TIME):
    """Hold the rover's brakes; after the scene settles, pick each rock up in turn
    and drop it on the trailer; then drive off, stop, and dump the load.

    Once `T_GRASP_START` has elapsed, `gripper.grasp(rock, place_pos)` is called
    every step for the current rock; when it returns True (rock released on the
    bed) the loop advances to the next rock -- the gripper's state machine restarts
    automatically for the new target. After every rock is loaded the rover releases
    the brakes and drives forward for DRIVE_TIME s, brakes to a stop for STOP_TIME s,
    then runs one `dump_bed.dump()` cycle to unload. With `vis`, render until the
    window is closed; otherwise step headless until `run_time` seconds (smoke test).
    """
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_throttle = 0.0
    driver_inputs.m_steering = 0.0
    driver_inputs.m_braking = 1.0  # hold the vehicle still

    render_steps = math.ceil((1.0 / 30) / STEP_SIZE)
    steps = 0
    idx = 0           # index of the rock currently being picked and placed
    t_loaded = None   # sim time at which the last rock was loaded (drive-off start)
    dumped = False    # whether the dump cycle has been triggered yet
    if vis is not None:
        vehicle.EnableRealtime(True)

    while True:
        if vis is not None and not vis.Run():
            break
        time = system.GetChTime()
        if vis is None and time >= run_time:
            break

        system.DoStepDynamics(STEP_SIZE)
        if vis is not None and steps % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        if idx < len(rocks):
            # ---- still loading: hold the brakes and pick/place each rock in turn ----
            driver_inputs.m_throttle = 0.0
            driver_inputs.m_steering = 0.0
            driver_inputs.m_braking = 1.0
            if time > T_GRASP_START:
                if gripper.grasp(rocks[idx], place_point(trailer, idx, len(rocks))):
                    print(f"  -> rock {idx} placed on the trailer")
                    idx += 1  # advance to the next rock (grasp() restarts on the new one)
        else:
            # ---- all rocks loaded: drive off, brake to a stop, then dump ----
            if t_loaded is None:
                t_loaded = time
                print(f"  all {len(rocks)} rocks loaded -- driving off")
            since = time - t_loaded
            if since < DRIVE_TIME:
                # ease the throttle in over DRIVE_RAMP s so the rocks don't lurch off
                driver_inputs.m_throttle = DRIVE_THROTTLE * min(1.0, since / DRIVE_RAMP)
                driver_inputs.m_steering = DRIVE_STEERING
                driver_inputs.m_braking = 0.0
            else:
                driver_inputs.m_throttle = 0.0
                driver_inputs.m_steering = 0.0
                driver_inputs.m_braking = 1.0  # brake to a stop, then hold
                if since >= DRIVE_TIME + STOP_TIME and not dumped:
                    print("  stopped -- dumping the load")
                    dump_bed.dump()
                    dumped = True

        # Keep the vehicle, trailer, terrain, and visual modules in sync.
        vehicle.Synchronize(time, driver_inputs, terrain)
        trailer.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        if vis is not None:
            vis.Synchronize(time, driver_inputs)
        vehicle.Advance(STEP_SIZE)
        trailer.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        if vis is not None:
            vis.Advance(STEP_SIZE)
        steps += 1


def main():
    headless = "--headless" in sys.argv
    use_vsg = "--vsg" in sys.argv  # render with VSG (Vulkan) instead of Irrlicht
    title = f"LRV + trailer: pick up {NUM_ROCKS} rocks and load them onto the trailer"
    print(f"=== {title} (headless smoke test) ===" if headless else f"=== {title} ===")
    system, vehicle, trailer, terrain, gripper, dump_bed, rocks = build_scene()
    arm_base = gripper.base.GetPos()
    print(f"  arm base   : {arm_base}")
    for rock in rocks:
        rel = rock.GetPos() - arm_base
        print(f"  {rock.GetName()} start: {rock.GetPos()} "
              f"(r={math.hypot(rel.x, rel.y):.2f} m, "
              f"theta={math.degrees(math.atan2(rel.y, rel.x)) % 360:.0f} deg)")

    vis = None if headless else make_vis(vehicle, "LRV With Trailer -- Pick and Place",
                                         use_vsg=use_vsg)
    brake_and_grasp(system, vehicle, trailer, terrain, gripper, dump_bed, rocks, vis=vis)

    if headless:
        bed_pos = trailer.GetChassis().GetBody().GetPos()
        print(f"  trailer at : {bed_pos}")
        for rock in rocks:
            on_bed = (rock.GetPos() - bed_pos).Length() < 1.0 and rock.GetPos().z > 0.4
            print(f"  {rock.GetName()} end  : {rock.GetPos()} -> on the trailer: {on_bed}")


if __name__ == "__main__":
    main()
