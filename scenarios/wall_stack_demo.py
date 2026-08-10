"""The stand-in wall on its own: no vehicle, no gripper, no real rocks.

`TrackedVeh_OrbitBuilder` spends a minute of sim driving a gripper through five
pick-and-place cycles before its wall first appears, and its later dumps are
further minutes in. This is the wall by itself -- the same
`orbit_planning.SlopedWallStacker` geometry spawned by the same
`TrackedVeh_OrbitBuilder.spawn_wall_rocks`, on a bare terrain patch -- so the
shape can be looked at and tuned in seconds.

What it shows is the thing that makes this wall different from a stack of layers:
it stands at a constant slope. Dumped rubble cannot stand steeper than its slip
angle, so the cross-section is a triangle, not a rectangle -- a 0.6 m crest needs
a 2.0 m base at WALL_SLOPE = 0.6. And it grows: each stage here is one more
`grow()` call, raising the crest and widening the base by the same ratio, so
H / R never changes and every stage adds only the shell over what is already
standing. The default three stages take the wall to 0.6, 0.9 and 1.2 m.

    conda run -n chrono python scenarios/wall_stack_demo.py

Options:
  --stages 0.6,0.9,1.2   crest height (m) after each dump, lowest first
  --snapshots [DIR]      write PNGs of each stage (default: DEFAULT_SNAPSHOT_DIR)
  --drop-test            drop a rock on the finished crest and report where it
                         stops -- only the courses near the crest are given
                         collision (the rest is out of reach, and paying the
                         broadphase for it costs 8x the step time), so this is
                         what catches the surface silently not being solid
  --headless             no window: print the course tables (and run any drop
                         test) without rendering
  --hold                 keep the window open after the last stage (default when
                         rendering interactively without --snapshots)
"""

import os
import sys
import math

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_root)
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from LRV_Arm import place_rock
# The scenario owns the wall's numbers and the body-spawning recipe (collision
# families, the centre-of-mass lift, the mid-run collision re-bind). Importing
# them rather than copying them is the point: what this demo renders is what the
# scenario builds.
import TrackedVeh_OrbitBuilder as builder

# Three dumps by default, so the constant slope shows up as the wall growing
# rather than as a single silhouette.
DEFAULT_STAGE_HEIGHTS = (0.6, 0.9, 1.2)
DEFAULT_SNAPSHOT_DIR = os.path.join(project_root, "artifacts", "snapshots",
                                    "wall_stack")
STEP_SIZE = 1e-3
DROP_HEIGHT = 0.4      # m above the crest surface to release the probe rock from
DROP_SETTLE_TIME = 3.0  # s to let it land

# Vertical field of view (rad) for the snapshots. Narrower than Irrlicht's own
# default, which is wide enough for a driving scene and leaves a 1 m wall as a
# speck in the middle of a lot of terrain.
SNAPSHOT_FOV = 0.60
SNAPSHOT_ASPECT = 16.0 / 9.0
WINDOW_SIZE = (1280, 720)


def wall_local(profile, u, s, z):
    """World point at radial offset `u`, arc offset `s` (m), height `z`."""
    return builder.planner.to_world(
        profile.r_center + u,
        profile.theta_center_deg + math.degrees(s / profile.r_center), z)


def fit_distance(width, height, margin=1.25):
    """How far back (m) a subject `width` x `height` fills the snapshot frame."""
    half_v = math.tan(0.5 * SNAPSHOT_FOV)
    return margin * max(0.5 * height / half_v,
                        0.5 * width / (half_v * SNAPSHOT_ASPECT))


def views(profile):
    """Camera stations for the snapshots, sized to the wall as it stands now.

    Each is `(name, eye, target)` in wall-local metres -- radial offset from the
    centerline, arc offset along the wall, height -- and each is set back from the
    wall's own footprint by whatever it takes to frame it, so a stage that widens
    the base does not grow out of the shot.
    """
    height, half_width = profile.height, profile.half_width
    half_length = profile.base_half_length
    end_on = fit_distance(2.0 * half_width, height)      # the triangular section
    broadside = fit_distance(2.0 * half_length, height)  # the whole length of it
    return (
        # Along the ridge from beyond one end: the end hip is the cross-section.
        ("section", (0.0, -(half_length + end_on), 0.55 * height + 0.25),
         (0.0, 0.0, 0.45 * height)),
        # Three-quarters from outside the orbit, kept low: a high camera looks
        # straight down onto the crest and flattens the faces out of the shot.
        ("face", (half_width + 0.55 * broadside, -(half_length + 0.55 * broadside),
                  0.3 * broadside), (0.0, 0.0, 0.5 * height)),
        # Square on and low, where height reads against length.
        ("elevation", (half_width + broadside, 0.0, 0.35 * height + 0.4),
         (0.0, 0.0, 0.5 * height)),
    )


def build_scene():
    """An empty site: SMC system, solver, and the same flat terrain patch.

    No vehicle and no rocks -- just the ground the wall stands on, so the scene
    costs nothing to set up and holds nothing that could move.
    """
    system = chrono.ChSystemSMC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    solver = chrono.ChSolverBB()
    solver.SetMaxIterations(100)
    solver.SetOmega(0.8)
    solver.SetSharpnessLambda(1.0)
    system.SetSolver(solver)
    system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

    veh.SetVehicleDataPath(os.path.join(chrono.GetChronoDataPath(), "vehicle")
                           + os.sep)
    terrain = veh.RigidTerrain(system)
    minfo = chrono.ChContactMaterialData()
    minfo.mu, minfo.cr, minfo.Y = 0.9, 0.2, 2e7
    patch = terrain.AddPatch(
        minfo.CreateMaterial(chrono.ChContactMethod_SMC),
        chrono.ChCoordsysd(chrono.ChVector3d(builder.ORBIT_CENTER[0],
                                             builder.ORBIT_CENTER[1], 0),
                           chrono.QUNIT),
        builder.TERRAIN_SIZE, builder.TERRAIN_SIZE)
    patch.SetColor(chrono.ChColor(0.45, 0.45, 0.45))
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/dirt.jpg"),
                     builder.TERRAIN_SIZE / 4.0, builder.TERRAIN_SIZE / 4.0)
    terrain.Initialize()
    return system, terrain


def make_vis(system, profile):
    """A render window framed on the wall's site."""
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(*WINDOW_SIZE)
    vis.SetWindowTitle("Sloped wall stacking -- constant slope, growing profile")
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.Initialize()
    vis.AddLogo()
    vis.AddSkyBox()
    # The site's own fill lights, as the scenario uses them -- see its `make_vis`.
    vis.AddTypicalLights()
    # Framed on the wall's final size from the start, so the view does not jump
    # between stages: the "face" station of the tallest stage.
    _, eye, target = views(profile.grown(height=DEFAULT_STAGE_HEIGHTS[-1]))[1]
    vis.AddCamera(wall_local(profile, *eye), wall_local(profile, *target))
    vis.GetActiveCamera().setFOV(SNAPSHOT_FOV)
    return vis


def render(vis, frames=3):
    """Draw `frames` frames, so the window is up to date before a snapshot."""
    for _ in range(frames):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()


def snapshot(vis, profile, path_prefix):
    """Write one PNG per camera station, framed on the wall as it stands now."""
    written = []
    for name, eye, target in views(profile):
        vis.UpdateCamera(wall_local(profile, *eye), wall_local(profile, *target))
        render(vis)
        path = f"{path_prefix}_{name}.png"
        vis.WriteImageToFile(path)
        written.append(path)
    return written


def drop_test(system, terrain, profile, layers, vis=None):
    """Drop a real rock on the crest and report where it comes to rest.

    The wall is stand-in rocks, but they are solid stand-ins -- that is what lets
    the scenario lay its last rocks on the finished crest. A wall that is standing
    and visible but not registered with the collision system looks identical from
    the outside and swallows anything set on it, so this measures it instead: the
    probe should stop within a rock of the crest surface, not on the ground.
    """
    surface = builder.wall_top_z(profile, layers)
    drop = wall_local(profile, 0.0, 0.0, surface + DROP_HEIGHT)
    probe = place_rock(system, drop, ground_z=drop.z - builder.ROCK_HEIGHT / 2.0,
                       contact_method=chrono.ChContactMethod_SMC)
    probe.SetName("probe")
    system.GetCollisionSystem().BindAll()
    if vis is not None:
        vis.BindItem(probe)

    for step in range(int(DROP_SETTLE_TIME / STEP_SIZE)):
        terrain.Synchronize(system.GetChTime())
        terrain.Advance(STEP_SIZE)
        system.DoStepDynamics(STEP_SIZE)
        if vis is not None and step % 33 == 0:   # ~30 frames per simulated second
            render(vis, frames=1)

    z = probe.GetPos().z
    landed = z > surface - builder.ROCK_HEIGHT
    print(f"\n  drop test: released at z={drop.z:.2f} m over a "
          f"{surface:.2f} m crest -> came to rest at z={z:.2f} m "
          f"({'ON THE WALL' if landed else 'FELL THROUGH -- the wall is phantom'})")
    return landed


def parse_stage_heights(argv):
    """`--stages 0.6,0.9,1.2` -> (0.6, 0.9, 1.2); default DEFAULT_STAGE_HEIGHTS."""
    if "--stages" not in argv:
        return DEFAULT_STAGE_HEIGHTS
    raw = argv[argv.index("--stages") + 1]
    return tuple(float(part) for part in raw.replace(",", " ").split())


def parse_snapshot_dir(argv):
    """`--snapshots [DIR]` -> the directory to write to, or None if not asked."""
    if "--snapshots" not in argv:
        return None
    idx = argv.index("--snapshots") + 1
    if idx < len(argv) and not argv[idx].startswith("--"):
        return argv[idx]
    return DEFAULT_SNAPSHOT_DIR


def main():
    headless = "--headless" in sys.argv
    stage_heights = parse_stage_heights(sys.argv)
    snapshot_dir = parse_snapshot_dir(sys.argv)
    want_drop_test = "--drop-test" in sys.argv
    hold = "--hold" in sys.argv or (not headless and snapshot_dir is None)

    stacker = builder.wall_stacker()
    print("=== sloped wall stacking (no vehicle, no real rocks) ===")
    print(f"  site      : wall orbit r={stacker.r_center:.1f} m, "
          f"station {stacker.theta_center_deg:+.2f} deg, "
          f"{2 * stacker.profile.ridge_half_length:.2f} m of ridge")
    print(f"  slope     : {stacker.slope:.2f} = "
          f"{math.degrees(math.atan(stacker.slope)):.1f} deg faces "
          f"(crest height / base half-width, held constant)")
    print(f"  courses   : {stacker.rise:.2f} m apart, "
          f"{stacker.density:.0f} rocks per m^2 of footprint")
    print(f"  stages    : {', '.join(f'{h:.2f} m' for h in stage_heights)}")

    if headless:
        # Geometry only. Every number the render would show, without a window.
        for stage, height in enumerate(stage_heights, start=1):
            profile, rows = stacker.plan(height=height)
            print(f"\n  stage {stage} -> base {2 * profile.half_width:.2f} m, "
                  f"crest {profile.height:.2f} m, "
                  f"H/R={profile.height / profile.half_width:.3f}, "
                  f"+{sum(n for _, _, _, n in rows)} rocks")
            for z, half_w, half_l, count in rows:
                print(f"    course z={z:.2f}: {2 * half_w:5.2f} m wide x "
                      f"{2 * half_l:5.2f} m long, +{count:4d} rocks")
            stacker.grow(height=height)
        print()
        print(stacker.describe())
        if want_drop_test:
            system, terrain = build_scene()
            layers = []
            probe_stacker = builder.wall_stacker()
            for height in stage_heights:
                profile, courses = probe_stacker.grow(height=height)
                layers += builder.spawn_wall_rocks(
                    system, courses, stage=probe_stacker.stage,
                    crest_z=profile.crest_z)
            drop_test(system, terrain, probe_stacker.profile, layers)
        return

    system, terrain = build_scene()
    vis = make_vis(system, stacker.profile)
    if snapshot_dir:
        os.makedirs(snapshot_dir, exist_ok=True)
        print(f"  snapshots : {snapshot_dir}")

    layers = []
    for stage, height in enumerate(stage_heights, start=1):
        profile, courses = stacker.grow(height=height)
        # Spawned with the render window already up, exactly as the scenario does
        # it mid-run: every body has to be bound to the visual system to show up,
        # and re-bound to the collision system to be solid.
        spawned = builder.spawn_wall_rocks(system, courses, stage=stacker.stage,
                                           crest_z=profile.crest_z, vis=vis)
        layers += spawned
        print(f"\n  stage {stage}: +{sum(len(c) for c in spawned)} rocks "
              f"-> base {2 * profile.half_width:.2f} m, "
              f"crest {profile.height:.2f} m, "
              f"H/R={profile.height / profile.half_width:.3f} "
              f"({stacker.rock_count} rocks standing)")
        render(vis, frames=10)
        if snapshot_dir:
            for path in snapshot(vis, profile,
                                 os.path.join(snapshot_dir, f"stage{stage}")):
                print(f"    wrote {path}")

    print()
    print(stacker.describe())
    if want_drop_test:
        drop_test(system, terrain, stacker.profile, layers, vis=vis)

    if hold:
        print("\n  close the window to finish")
        while vis.Run():
            render(vis, frames=1)


if __name__ == "__main__":
    main()
