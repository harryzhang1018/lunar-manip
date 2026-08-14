"""LRV + gripper arm — record/replay rock placement (two-simulation demo).

A wheeled lunar rover (LRV / Polaris) with a gripper arm welded to the back of
the chassis, standing (braked) on flat terrain.

The demo runs two simulations:

  * SIM 1 (headless): pick a random target behind the arm (at the fixed grab
    height GRAB_HEIGHT above the ground), drive the end-effector there with
    inverse kinematics, let it settle, and *record* the four joint angles and
    the gripper-center position (the midpoint of the two fingers' centers of
    mass).

  * SIM 2 (visualized): rebuild the same scene, replay the recorded joint
    angles, and drop a rock at the recorded gripper-center position -- i.e.
    right where the gripper actually ends up. Because the scene and inputs are
    identical, the gripper settles at the same spot, so the rock lands at the
    gripper. (Recording the *actual* settled pose, rather than the IK target,
    accounts for IK residual error and dynamic sag.) Then the last mile: the
    fingers close around the rock, the rock is locked to the end-effector, and
    the arm lifts it by ramping the shoulder joint (theta 2) up to 60 deg.

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
import shutil

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.postprocess as postprocess

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

# The rock: the Curiosity rock2 mesh (from chrono's data, copied into
# data/rocks). At ROCK_SCALE it measures 0.182 x 0.227 x 0.182 m and its widest
# horizontal span is 0.182-0.227 m depending on the yaw it lands at -- between the
# pads' closed gap (0.098 m) and their open gap (0.388 m) whichever way it faces,
# which is what makes one mesh scale fit the gripper.
#
# Note that GRAB_HEIGHT is *above* the rock, not a cross-section through it: the
# arm drives the gripper centre to 0.22 m and the pads, which hang below it, meet
# the rock around its waist. The lock separations the runs record (0.184-0.206 m)
# are that waist, and they are what the shape library's size budget is checked
# against below -- not the nominal grab height.
ROCK_MESH = os.path.join("data", "rocks", "rock2.obj")
ROCK_SCALE = 0.16
ROCK_ROT_X = math.radians(90.0)    # initial rock rotation about its local x axis
ROCK_DENSITY = 1500                # kg/m^3
GRAB_HEIGHT = 0.22                 # gripper-center target height above ground

# ---- the rock shape library ----
# One mesh stamped out a thousand times reads as tiling: every rock in a wall has
# the same silhouette, and the eye picks that up well before it picks up the wall.
# So the rock is a small *library* instead -- ROCK_VARIANTS meshes derived once
# from rock2.obj, each with its own lumps, its own proportions and its own size,
# and then shared by every body that draws it. A body picks a variant index;
# nothing is built per rock, which is what keeps a 2000-rock wall cheap (and is
# actually faster than the old path, which re-read the .obj for every rock).
#
# What the sizes may not do is break the grasp. The base mesh at ROCK_SCALE is
# 0.182 x 0.227 x 0.182 m, and it is *not* grabbed at GRAB_HEIGHT: 0.22 m is over
# the top of it, so the bite is made by the pads hanging below the gripper centre.
# The lock separations the runs record (0.184-0.206 m) match the mesh's own widest
# span (0.182-0.237 m, at z ~ 0.08-0.11), so the pads meet the rock around its
# waist. Scaling the rock moves that waist:
#
#     size   height   widest span   pads (0.098 closed, 0.388 open)
#     0.80    0.146   0.146-0.190   grips, 0.05 m of travel to spare
#     1.00    0.182   0.182-0.237   as shipped
#     1.20    0.218   0.218-0.284   grips, 0.10 m of jaw to spare
#
# and the lock gate wants the rock's centre within 0.27 m of each finger, which a
# 0.146 m rock (centre 0.019 m lower than the base rock's) still clears by 0.09 m.
# 0.8-1.2 has room at both ends; much outside it does not, which is what the
# caller's 0.8-1.2 budget buys. Sizes are drawn over ROCK_SIZE_RANGE.
ROCK_VARIANTS = 12
ROCK_SIZE_RANGE = (0.8, 1.2)
# Proportions: each variant's axes are stretched by up to this fraction and then
# renormalised so their product is 1, i.e. the stretch changes the shape without
# changing the volume the size factor asked for. Kept modest, because a rock that
# is much wider than it is deep presents a very different span depending on the
# yaw it happens to land at, and the pads have to close on all of them.
ROCK_ASPECT = 0.16
# The geometry itself: each vertex is pushed in or out along its own direction
# from the mesh centre by a sum of ROCK_LOBES smooth bumps -- random directions,
# random signs, and a random falloff power so some are broad swells and some are
# tighter knobs. This is what makes the variants different *rocks* rather than
# the same rock at different sizes. The total displacement is clamped to
# +/-ROCK_LOBE_GAIN of the radius: past ~0.25 the low-poly mesh starts folding
# through itself and the sphere-swept collision hull stops matching what is drawn.
ROCK_LOBES = 4
ROCK_LOBE_GAIN = 0.22
ROCK_SHAPE_SEED = 8071             # fixed: the library is the same every run

# Grab-and-lift sequence (sim 2). The fingers close until the pads stall on
# the rock (where they meet it depends on the rock yaw), or at worst to
# FINGER_CLOSE_POS, just before the finger motors' travel stop. Over the
# whole sampling domain the settled theta 2 stays within [-16, 19] deg, so a
# 60 deg shoulder target always lifts.
CTRL_DT = 0.01                     # control tick for the staged grab/lift
T_CLOSE = 4.5                      # close once the replayed pose has settled
T_LIFT = 6.5                       # lift after the lock is in place
FINGER_OPEN_SEP = 0.388            # finger COM separation at open (motor pos 0)
FINGER_CLOSE_POS = 0.145           # max finger travel from open
FINGER_CLOSE_SPEED = 0.1           # m/s per finger
GRIP_STALL_TOL = 0.012             # command-vs-actual lag that means "pads on rock"
LIFT_THETA2 = math.radians(60.0)   # shoulder (theta 2) target for the lift
LIFT_SPEED = 0.5                   # rad/s shoulder ramp


class RockShape:
    """One geometry from the rock library: the mesh, its mass properties, its size.

    Built once by `rock_shape` and then shared by every body that uses it. The
    mesh is static geometry, so a single `ChTriangleMeshConnected` can back any
    number of sphere-swept collision shapes and a single
    `ChVisualShapeTriangleMesh` any number of bodies -- which is what makes a
    thousand-rock wall affordable to spawn. Mass and inertia are taken off the
    *deformed* mesh, so a lumpy or stretched variant gets the inertia it actually
    has rather than the base rock's.
    """

    __slots__ = ("index", "size", "axes", "mesh", "visual", "mass", "com",
                 "principal_I", "principal_rot", "bbox")

    def __init__(self, index, size, axes, mesh):
        self.index = index
        self.size = size
        self.axes = axes
        self.mesh = mesh
        props = mesh.ComputeMassProperties(True)  # unit-density mass = volume
        principal_I = chrono.ChVector3d()
        principal_rot = chrono.ChMatrix33d()
        chrono.ChInertiaUtils.PrincipalInertia(props.inertia, principal_I,
                                               principal_rot)
        # Copies, not references into the temporary the mass-properties call
        # returned -- these outlive it by the whole run.
        self.mass = props.mass
        self.com = chrono.ChVector3d(props.com)
        self.principal_I = chrono.ChVector3d(principal_I)
        self.principal_rot = principal_rot
        self.bbox = mesh.GetBoundingBox()
        self.visual = chrono.ChVisualShapeTriangleMesh()
        self.visual.SetMesh(mesh)
        self.visual.SetColor(chrono.ChColor(0.45, 0.35, 0.28))  # rock-brown

    @property
    def height(self):
        return self.bbox.max.z - self.bbox.min.z

    @property
    def span(self):
        """(min, max) horizontal extent -- the widths the gripper has to close on."""
        w = self.bbox.max.x - self.bbox.min.x
        d = self.bbox.max.y - self.bbox.min.y
        return min(w, d), max(w, d)

    def describe(self):
        lo, hi = self.span
        return (f"variant {self.index}: size {self.size:.2f}, axes "
                f"({self.axes[0]:.2f}, {self.axes[1]:.2f}, {self.axes[2]:.2f}), "
                f"{self.height:.3f} m tall, span {lo:.3f}-{hi:.3f} m, "
                f"{self.mass * ROCK_DENSITY:.1f} kg")


def _rock_lobes(rng, count):
    """`count` smooth surface bumps as (direction, gain, falloff) -- see ROCK_LOBES.

    Each is a uniformly random direction on the sphere with a signed gain, so a
    lobe either swells the rock out on that side or scoops it in, and a falloff
    power that decides how broad it is: 1 is a whole-hemisphere swell, 3 a tight
    knob. Summing a handful gives a surface that is lumpy but still smooth, which
    a 56-vertex mesh needs -- per-vertex noise on a mesh this coarse reads as
    spikes rather than as rock.
    """
    lobes = []
    for _ in range(count):
        z = rng.uniform(-1.0, 1.0)
        phi = rng.uniform(0.0, 2.0 * math.pi)
        rho = math.sqrt(max(0.0, 1.0 - z * z))
        lobes.append(((rho * math.cos(phi), rho * math.sin(phi), z),
                      rng.uniform(-1.0, 1.0), rng.choice((1, 2, 3))))
    return lobes


def _build_rock_shape(index):
    """Build library variant `index` (None -> the undeformed base rock)."""
    mesh = chrono.ChTriangleMeshConnected.CreateFromWavefrontFile(
        os.path.join(project_root, ROCK_MESH), False, True)
    mesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(ROCK_SCALE))
    mesh.Transform(chrono.ChVector3d(0, 0, 0),
                   chrono.ChMatrix33d(chrono.QuatFromAngleX(ROCK_ROT_X)))
    if index is None:
        return RockShape(None, 1.0, (1.0, 1.0, 1.0), mesh)

    # Its own RNG, seeded from the index: variant 7 is the same rock in every run
    # and in every scenario, and asking for it does not disturb the caller's
    # `random` stream (the wedge sampler and the wall sampler both use theirs).
    rng = random.Random(ROCK_SHAPE_SEED + index)
    size = rng.uniform(*ROCK_SIZE_RANGE)
    axes = [math.exp(rng.uniform(-ROCK_ASPECT, ROCK_ASPECT)) for _ in range(3)]
    # Renormalise the stretch to unit volume before applying the size, so
    # proportions and size are independent: `size` alone says how big the rock is.
    norm = size / (axes[0] * axes[1] * axes[2]) ** (1.0 / 3.0)
    axes = [a * norm for a in axes]
    lobes = _rock_lobes(rng, ROCK_LOBES)

    # The mesh carries no normals (loaded with load_normals=False, and the render
    # backend derives face normals itself), so moving vertices needs no fixup.
    verts = mesh.GetCoordsVertices()
    bb = mesh.GetBoundingBox()
    cx, cy, cz = ((bb.min.x + bb.max.x) / 2.0, (bb.min.y + bb.max.y) / 2.0,
                  (bb.min.z + bb.max.z) / 2.0)
    for i in range(len(verts)):
        v = verts[i]
        dx, dy, dz = v.x - cx, v.y - cy, v.z - cz
        radius = math.sqrt(dx * dx + dy * dy + dz * dz)
        if radius < 1e-9:
            continue
        nx, ny, nz = dx / radius, dy / radius, dz / radius
        bump = sum(gain * max(0.0, nx * d[0] + ny * d[1] + nz * d[2]) ** falloff
                   for d, gain, falloff in lobes)
        # Clamped, not scaled: a variant that happens to draw four lobes pointing
        # the same way must not turn inside out.
        grow = 1.0 + ROCK_LOBE_GAIN * max(-1.0, min(1.0, bump))
        verts[i] = chrono.ChVector3d(cx + dx * grow * axes[0],
                                     cy + dy * grow * axes[1],
                                     cz + dz * grow * axes[2])
    size *= _fit_rock_to_budget(mesh)
    return RockShape(index, size, tuple(axes), mesh)


def _rock_extents(mesh):
    """(widest horizontal extent, height) of `mesh` -- what the gripper sees."""
    bb = mesh.GetBoundingBox()
    return (max(bb.max.x - bb.min.x, bb.max.y - bb.min.y),
            bb.max.z - bb.min.z)


def _fit_rock_to_budget(mesh, _base=[]):
    """Rescale `mesh` in place until it fits ROCK_SIZE_RANGE; return the factor.

    The size a variant draws is only a request: the lobes and the axis stretch
    both move the extents on top of it, and they can conspire (a variant that
    draws 1.2 and then swells its long axis ends up half again as wide as the base
    rock, which is outside the range the pads were checked over). So the finished
    mesh is measured and, if either its widest horizontal extent or its height is
    outside 0.8-1.2 of the base rock's, uniformly rescaled to the boundary it
    broke -- making ROCK_SIZE_RANGE an invariant of the library rather than a
    distribution it is drawn from. Width wins a tie: the jaw opening is the hard
    limit, the height only decides where on the rock the pads bite.
    """
    if not _base:                      # the undeformed rock, measured once
        base = chrono.ChTriangleMeshConnected.CreateFromWavefrontFile(
            os.path.join(project_root, ROCK_MESH), False, True)
        base.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(ROCK_SCALE))
        base.Transform(chrono.ChVector3d(0, 0, 0),
                       chrono.ChMatrix33d(chrono.QuatFromAngleX(ROCK_ROT_X)))
        _base.append(_rock_extents(base))
    base_span, base_height = _base[0]
    lo, hi = ROCK_SIZE_RANGE

    span, height = _rock_extents(mesh)
    ratios = (span / base_span, height / base_height)
    if max(ratios) > hi:
        factor = hi / max(ratios)
    elif min(ratios) < lo:
        factor = lo / min(ratios)
    else:
        return 1.0
    mesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(factor))
    return factor


_ROCK_SHAPES = {}


def rock_shape(index=None):
    """The library's shape `index`, built on first use and cached for the run.

    `index` is taken modulo ROCK_VARIANTS, so a caller can pass any integer -- a
    rock number, a hash of a body name -- without knowing the library size.
    `None` is the undeformed base rock: what every scenario got before the library
    existed, and still what `place_rock` uses when it is not given a shape.
    """
    if isinstance(index, RockShape):
        return index
    key = None if index is None else int(index) % ROCK_VARIANTS
    if key not in _ROCK_SHAPES:
        _ROCK_SHAPES[key] = _build_rock_shape(key)
    return _ROCK_SHAPES[key]


def place_rock(system, pos, ground_z, contact_method=chrono.ChContactMethod_NSC,
               shape=None):
    """Spawn the rock mesh resting on the ground beneath the gripper center `pos`.

    Follows the rock recipe from chrono's Curiosity demos: a scaled
    ChTriangleMeshConnected used both as visual and (sphere-swept) collision
    shape on a ChBodyAuxRef, with mass/inertia computed from the mesh. The
    body is placed so the mesh bounding box is centered (x, y) under `pos`
    with its bottom on the ground; the gripper, at GRAB_HEIGHT, then bites
    the rock around its waist (see the rock shape library above).

    `contact_method` selects the rock's contact material type (NSC by default;
    pass ChContactMethod_SMC to match an SMC system).

    `shape` picks the geometry: a `RockShape`, a variant index, or None for the
    base rock. Every body sharing a variant shares its mesh and its visual shape,
    so spawning is a body allocation rather than a mesh load -- which is what lets
    a caller give every rock in a wall its own shape without paying for it.
    """
    shape = rock_shape(shape)

    rock = chrono.ChBodyAuxRef()
    rock.SetName("rock")
    rock.SetFixed(False)
    bb = shape.bbox
    # Rest the bottom on the ground, lifted by the sphere-swept collision
    # radius so the inflated collision shape starts free of penetration.
    rock.SetFrameRefToAbs(chrono.ChFramed(
        chrono.ChVector3d(pos.x - (bb.min.x + bb.max.x) / 2.0,
                          pos.y - (bb.min.y + bb.max.y) / 2.0,
                          ground_z - bb.min.z + 0.005), chrono.QUNIT))
    rock.SetFrameCOMToRef(chrono.ChFramed(shape.com, shape.principal_rot))
    rock.SetMass(shape.mass * ROCK_DENSITY)
    rock.SetInertiaXX(shape.principal_I * ROCK_DENSITY)

    if contact_method == chrono.ChContactMethod_SMC:
        mat = chrono.ChContactMaterialSMC()
        mat.SetYoungModulus(2e7)
        mat.SetRestitution(0.01)
    else:
        mat = chrono.ChContactMaterialNSC()
    mat.SetFriction(0.7)
    rock.AddCollisionShape(
        chrono.ChCollisionShapeTriangleMesh(mat, shape.mesh, False, False, 0.005))
    rock.EnableCollision(True)
    rock.AddVisualShape(shape.visual)

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
    the vehicle, which sits toward +x), r in [2, 3] m, and GRAB_HEIGHT above
    the ground (fixed, so the rock mesh always fits the gripper). Re-samples if
    the point is out of reach (the arm spans ~2.7 m, so r near 3 m can be
    unreachable). Returns the four joint angles.
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
        target = chrono.ChVector3d(tx, ty, ground_z + GRAB_HEIGHT)
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


def drive_arm(system, vehicle, terrain, gripper, theta, vis=None, settle_time=4.0,
              grab=False, blender_exporter=None):
    """Brake the vehicle and drive the arm to `theta` (staggered to avoid a snap).

    With `grab=True`, once the pose has settled the gripper finishes the last
    mile: it closes the fingers (t > T_CLOSE), locks the registered object to
    the end-effector via `gripper.add_lock()`, and lifts it by ramping the
    shoulder joint (theta 2) to LIFT_THETA2 (t > T_LIFT).

    With `vis`, render until the window is closed; otherwise step headless until
    `settle_time` seconds. With `blender_exporter`, export a frame at `fps`
    while 0.05 s <= t < 20 s. Returns the final gripper-center position.
    """
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_throttle = 0.0
    driver_inputs.m_steering = 0.0
    driver_inputs.m_braking = 1.0  # hold the vehicle still

    render_steps = math.ceil((1.0 / 30) / STEP_SIZE)
    moved_arm = bent_arm = gripped = False
    close_pos = 0.0    # current finger-motor target (0 = open)
    lift_angle = None  # current shoulder target while lifting
    next_tick = 0.0    # next control-tick time for the staged grab/lift
    steps = 0
    fps = 60
    frame_interval = 1.0 / fps  # seconds between exported Blender frames
    next_export_time = 0.0
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

        if blender_exporter is not None and 0.05 <= time < 20:
            if time >= next_export_time:
                blender_exporter.ExportData()
                next_export_time += frame_interval

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

        # ---- last mile: close the gripper, lock the rock, lift it ----
        # Ticks are keyed on sim time (not the loop counter) because each loop
        # iteration advances the system twice: DoStepDynamics here plus
        # vehicle.Advance (the vehicle owns the system).
        if grab and time > T_CLOSE and time >= next_tick:
            next_tick = time + CTRL_DT
            if not gripped:
                # Close the fingers gradually until the pads stall on the rock
                # (or, with nothing in the way, until the travel cap).
                close_pos = min(close_pos + FINGER_CLOSE_SPEED * CTRL_DT,
                                FINGER_CLOSE_POS)
                gripper.move_linear_motor(gripper.motor_endoffactor_finger_1, -close_pos)
                gripper.move_linear_motor(gripper.motor_endoffactor_finger_2, close_pos)
                actual_sep = (gripper.finger_1.GetPos()
                              - gripper.finger_2.GetPos()).Length()
                stalled = actual_sep - (FINGER_OPEN_SEP - 2 * close_pos) > GRIP_STALL_TOL
                if stalled or close_pos >= FINGER_CLOSE_POS:
                    if stalled:
                        # Park the command where the fingers actually are (plus
                        # a light 2 mm bite) so the position motors hold the
                        # rock instead of crushing into it.
                        close_pos = (FINGER_OPEN_SEP - actual_sep) / 2.0 + 0.002
                        gripper.move_linear_motor(gripper.motor_endoffactor_finger_1, -close_pos)
                        gripper.move_linear_motor(gripper.motor_endoffactor_finger_2, close_pos)
                    gripped = True
                    gripper.add_lock()  # lock the grabbed object to the end-effector
                    if gripper.cur_lock:
                        print(f"  gripper closed (sep {actual_sep:.3f} m), "
                              f"'{gripper.cur_object}' locked to the end-effector "
                              f"(t={time:.2f} s)")
                    else:
                        print(f"  gripper closed but no object in range to lock "
                              f"(t={time:.2f} s)")
            elif time > T_LIFT and lift_angle != LIFT_THETA2:
                # Ramp the shoulder (theta 2) up to the lift angle.
                if lift_angle is None:
                    lift_angle = theta[1]
                    print(f"  lifting: theta 2 {math.degrees(theta[1]):.1f} -> "
                          f"{math.degrees(LIFT_THETA2):.1f} deg")
                d = LIFT_THETA2 - lift_angle
                if abs(d) <= LIFT_SPEED * CTRL_DT:
                    lift_angle = LIFT_THETA2
                else:
                    lift_angle += math.copysign(LIFT_SPEED * CTRL_DT, d)
                gripper.rotate_motor(gripper.motor_shoulder_biceps, lift_angle)

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

    # ---- SIM 2 (visualized): rock at the recorded spot; replay, grab, lift. ----
    print("=== SIM 2: rock at recorded gripper center -- replay, grab, and lift ===")
    system2, vehicle2, terrain2, gripper2 = build_scene(rock_pos=ee_pos)
    gripper2.add_object("rock")  # register the rock with the gripper's lock logic
    vis = make_vis(vehicle2, "LRV Gripper -- replay, grab, and lift")

    out_dir = chrono.GetChronoOutputPath() + "BLENDER"
    if os.path.exists(out_dir):
        shutil.rmtree(out_dir)
    os.makedirs(out_dir)
    blender_exporter = postprocess.ChBlender(system2)
    blender_exporter.SetBasePath(out_dir)
    blender_exporter.SetBlenderUp_is_ChronoZ()
    blender_exporter.AddAll()
    blender_exporter.ExportScript()

    drive_arm(system2, vehicle2, terrain2, gripper2, final_theta, vis=vis, grab=True,
              blender_exporter=blender_exporter)


if __name__ == "__main__":
    main()
