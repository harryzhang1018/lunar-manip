"""Orbit planning for the circular wall-building site.

The site is a set of concentric orbits about a single center (see the concept
sketch: a wall ring, a tracked-vehicle ring outside it, and a rock-unloading ring
outside that):

    wall orbit     r = WALL_RADIUS      -- the wall being built; PLACE_POINTs land here
    vehicle orbit  r = VEHICLE_RADIUS   -- the ring the builders drive/park on
    unload orbit   r = UNLOAD_RADIUS    -- where the fetchers drop rocks off

`OrbitPlanner` owns that geometry and turns it into the two things a builder
scenario needs: where a builder starts (a pose on the vehicle orbit, by default
headed *along* it, so the wall runs alongside the builder on its left and the
unloading side on its right), and the discrete wall points that builder is
responsible for -- the sequence of `PLACE_POINT`s its gripper drops rocks on.

Place-point assignment: a builder parked at station angle `theta_v` is given
`num_points` points on the wall orbit spanning `theta_v +/- half_span_deg`, both
endpoints included. With the defaults, a builder at (33, 0) -- i.e. station
theta_v = 0 deg -- gets 10 points at polar (r, theta) = (30, -5 deg) through
(30, +5 deg), stepping 10/9 deg each. Angles are degrees CCW from +X throughout.

The caller picks which station angle to center that arc on. It is not always the
chassis's own: `TrackedVeh_OrbitBuilder` centers on the station of its *arm base*,
which rides a couple of metres along the orbit from the chassis reference, since
that is where the reach is centered.

The half span is what one *parked* builder can cover: the gripper arm reaches
~2-4 m, and 1 deg of the 30 m wall orbit is 0.52 m of wall, so a parked builder
covers roughly +/-5 deg. A builder that drives along its orbit between place
points can be given a much wider span (the sketch's +/-15 deg and beyond) -- the
planner does not care, it just hands out the points.

Stacking layers without building them: the gripper takes ~14 s of sim per rock,
so laying every course of a wall for real costs a full pick-and-place cycle per
rock per layer. `measure_belt` + `layer_poses` skip that. A run builds its ground
layer for real and leaves `rock_state.csv` (where every rock ended up) and
`site_plan.csv` (the layout it used) behind -- see
`TrackedVeh_OrbitBuilder.write_rock_state`. `measure_belt` reads that back as the
round belt the rocks occupy: the radial band they scattered over, the arc they
span, and the height their centres sit at. `layer_poses` then re-samples that
same (r, theta) distribution `n` times over, one rock height apart in z, and
hands back rock poses for those upper layers -- scenery the caller spawns fixed
and collision-free rather than rocks the gripper carries.
`plan_layers_from_state` is the two of them in one call, straight off the CSVs.

`SlopedWallStacker` is the other way to get a wall, and the one the scenarios use
now. It does not measure anything: the wall is laid out where it is told, on the
wall orbit, so no rocks need to have been placed first. What it adds is the shape
-- dumped rubble stands at its slip angle, so the wall it builds has a triangular
cross-section (`WallProfile`) rather than the straight-sided stack `layer_poses`
makes, and every course it lays is narrower than the one below. `grow()` is
callable over and over: each call raises the crest and widens the base by the
same ratio, holding the slope constant, and hands back just the rocks that new
profile adds -- a shell over what is already standing. A wall therefore comes up
in dumps, at a fixed slope, exactly as tipping more rock onto a pile would build
it.

Standalone (prints the plan for the default site, no simulation):

    conda run -n chrono python scenarios/orbit_planning.py

Pass a run's `rock_state.csv` (and, optionally, how many layers to add -- default
2) to print the measured belt and the layers planned on top of it instead. The
sibling `site_plan.csv` is picked up automatically if it sits next to it:

    conda run -n chrono python scenarios/orbit_planning.py \\
        artifacts/states/trackedveh_orbitbuilder/rock_state.csv 3
"""

import os
import csv
import sys
import math
import random

import pychrono as chrono

# Default site geometry (m). The wall is the ring being built; the builders ride
# an orbit 3.4 m outside it, and the fetchers unload rocks a further ~1.6 m out.
# The builder separation is set by the widest wall the site builds: a finished
# crest on a constant slope spreads its base half its height / slope either side of
# the wall orbit, and the builder has to park clear of that with its tracks. See
# TrackedVeh_OrbitBuilder.VEHICLE_RADIUS for the measurements behind 3.4.
WALL_RADIUS = 30.0
VEHICLE_RADIUS = 33.4
UNLOAD_RADIUS = 35.0

# Default place-point assignment for one parked builder.
NUM_PLACE_POINTS = 10
HALF_SPAN_DEG = 5.0

# Height (m above ground) the gripper releases a rock at over a wall point.
PLACE_HEIGHT = 0.35

# Height (m) of one rock, i.e. how much a layer raises the belt it is laid on.
# The rocks the scenarios use stand ~0.26 m tall; `site_plan.csv` records the
# value the run itself used (`course_rise`), which overrides this when available.
LAYER_RISE = 0.26

# Default file names a scenario run writes its end state to.
ROCK_STATE_FILE = "rock_state.csv"
SITE_PLAN_FILE = "site_plan.csv"

# ---- sloped wall stacking (see WallProfile / SlopedWallStacker) ----
# Slope of the wall's faces, as the ratio height / base-half-width. It is the
# tangent of the angle the face makes with the ground, so 0.8 is a 38.7 deg face
# -- about what dumped angular rubble stands at. This ratio is the invariant the
# stack is built around: every call that grows the wall raises the crest and
# widens the base together, so H/R never changes and the cross-section stays the
# same triangle, just bigger.
WALL_SLOPE = 0.8
# Vertical spacing (m) between courses of rock centres. Well under a rock height
# (~0.26 m) on purpose, so consecutive courses overlap and knit into a solid face
# instead of reading as separate shelves.
WALL_COURSE_RISE = 0.10
# Rocks per m^2 of course footprint. With the rise above this works out at ~240
# rocks/m^3, i.e. about one rock volume per rock -- dense enough that the faces
# read solid.
WALL_COURSE_DENSITY = 24.0
# Re-sample a rock landing within this distance (m) of one already in the same
# course. Under the ~0.2 m rock width, so rocks knit rather than tile.
WALL_MIN_SEP = 0.08
WALL_TILT_DEG = 15.0      # max tilt off vertical of a stand-in rock
WALL_GROWTH_RISE = 0.30   # default crest rise (m) added by one grow() call


def _number(value):
    """A CSV cell as an int or float when it reads as one, else the raw string."""
    for cast in (int, float):
        try:
            return cast(value)
        except (TypeError, ValueError):
            continue
    return value


def _median(values):
    """Median of a non-empty sequence of numbers."""
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def read_site_plan(path):
    """A run's `site_plan.csv` (key,value rows) as a dict, numbers parsed."""
    with open(path, newline="") as f:
        return {row["key"]: _number(row["value"]) for row in csv.DictReader(f)}


def read_rock_state(path):
    """A run's `rock_state.csv` as a list of per-rock dicts, numbers parsed.

    Column names are whatever the writing scenario used; `measure_belt` needs the
    rock's world position (`com_x/com_y/com_z`, or `ref_*` as a fallback) and uses
    `placed`, `place_point` and `course` when they are there.
    """
    with open(path, newline="") as f:
        return [{key: _number(value) for key, value in row.items()}
                for row in csv.DictReader(f)]


class RockBelt:
    """The round belt a set of placed rocks occupies, in site polar coordinates.

    Built by `OrbitPlanner.measure_belt` from a run's rock positions: the rocks
    laid along a wall arc form an annular band, and this is its measurement --
    the radial band (`r_min`..`r_max`, i.e. how far the rocks scattered off the
    wall orbit), the arc it covers (`theta_min_deg`..`theta_max_deg`), and how
    high it stands (`top_z`, the top *surface*, which is what a new layer is laid
    on). `samples` are the (r, theta_deg, z) triples it was measured from.

    `r_med` -- the median radius -- is the belt's centerline: unlike the mean it
    ignores the odd rock that got punted a metre off the wall, which is exactly
    what a stray rock does to `r_min`/`r_max`.

    `points_per_layer` and `layers` are read off the build-plan columns of the
    CSV (`place_point`, `course`) and describe how the run was *planned*, not what
    it produced. A run planned as two courses can very easily end up as one belt
    of rubble on the ground -- which is why `layer_poses` sizes its layers from
    `count`, the rocks actually measured.
    """

    def __init__(self, samples, rock_height=LAYER_RISE, points_per_layer=None,
                 layers=1):
        if not samples:
            raise ValueError("no rocks to measure a belt from")
        self.samples = list(samples)
        self.count = len(self.samples)
        radii = [s[0] for s in self.samples]
        thetas = [s[1] for s in self.samples]
        zs = [s[2] for s in self.samples]
        self.r_min, self.r_max = min(radii), max(radii)
        self.r_mean = sum(radii) / self.count
        self.r_med = _median(radii)
        self.width = self.r_max - self.r_min
        self.theta_min_deg, self.theta_max_deg = min(thetas), max(thetas)
        self.theta_span_deg = self.theta_max_deg - self.theta_min_deg
        self.z_min, self.z_max = min(zs), max(zs)
        self.z_mean = sum(zs) / self.count
        self.rock_height = rock_height
        # Top surface of what is built: the highest rock's centre sits about half a
        # rock below its own top, and a new layer rests on that surface.
        self.top_z = self.z_max + rock_height / 2.0
        self.points_per_layer = points_per_layer
        self.layers = layers

    def arc_length(self, radius=None):
        """Length (m) of the belt's arc, on `r_med` unless told otherwise."""
        radius = self.r_med if radius is None else radius
        return math.radians(self.theta_span_deg) * radius

    def describe(self):
        """Multi-line summary of the measured belt."""
        return "\n".join([
            f"rock belt: {self.count} rocks"
            + (f" (planned as {self.layers} course(s) of {self.points_per_layer})"
               if self.points_per_layer else ""),
            f"  radial : r={self.r_med:.2f} m (median), {self.r_min:.2f}..{self.r_max:.2f} m "
            f"-> {self.width:.2f} m wide, mean {self.r_mean:.2f} m",
            f"  arc    : theta {self.theta_min_deg:+.2f}..{self.theta_max_deg:+.2f} deg "
            f"({self.theta_span_deg:.2f} deg = {self.arc_length():.2f} m of wall)",
            f"  height : rock centres {self.z_min:.2f}..{self.z_max:.2f} m, "
            f"top surface {self.top_z:.2f} m (rock height {self.rock_height:.2f} m)",
        ])


class OrbitPlanner:
    """Concentric build-site orbits, and the wall points assigned to each builder.

    Parameters mirror the module constants above. `center` is the (x, y) center of
    all the orbits and `ground_z` the terrain height they sit on. Place points are
    returned `place_height` above that ground.
    """

    def __init__(self, center=(0.0, 0.0), wall_radius=WALL_RADIUS,
                 vehicle_radius=VEHICLE_RADIUS, unload_radius=UNLOAD_RADIUS,
                 num_points=NUM_PLACE_POINTS, half_span_deg=HALF_SPAN_DEG,
                 place_height=PLACE_HEIGHT, ground_z=0.0):
        self.center = (float(center[0]), float(center[1]))
        self.wall_radius = wall_radius
        self.vehicle_radius = vehicle_radius
        self.unload_radius = unload_radius
        self.num_points = num_points
        self.half_span_deg = half_span_deg
        self.place_height = place_height
        self.ground_z = ground_z

    @classmethod
    def from_site_plan(cls, plan):
        """Rebuild the planner a run used, from its `site_plan.csv`.

        `plan` is the path to that file or an already-read dict. Anything the file
        does not carry falls back to this module's defaults, so a plan written by
        a different scenario still loads.
        """
        if not isinstance(plan, dict):
            plan = read_site_plan(plan)
        return cls(center=(plan.get("orbit_center_x", 0.0),
                           plan.get("orbit_center_y", 0.0)),
                   wall_radius=plan.get("wall_radius", WALL_RADIUS),
                   vehicle_radius=plan.get("vehicle_radius", VEHICLE_RADIUS),
                   num_points=int(plan.get("num_place_points", NUM_PLACE_POINTS)),
                   half_span_deg=plan.get("half_span_deg", HALF_SPAN_DEG),
                   place_height=plan.get("place_height", PLACE_HEIGHT),
                   ground_z=plan.get("ground_z", 0.0))

    # ---- polar <-> world ----
    def to_world(self, r, theta_deg, z=None):
        """World `ChVector3d` at polar (r, theta_deg) about the site center."""
        theta = math.radians(theta_deg)
        return chrono.ChVector3d(self.center[0] + r * math.cos(theta),
                                 self.center[1] + r * math.sin(theta),
                                 self.ground_z if z is None else z)

    def to_polar(self, pos):
        """(r, theta_deg) of a world point (a `ChVector3d` or an (x, y) pair)."""
        x, y = (pos.x, pos.y) if hasattr(pos, "x") else (pos[0], pos[1])
        dx, dy = x - self.center[0], y - self.center[1]
        return math.hypot(dx, dy), math.degrees(math.atan2(dy, dx))

    def station_of(self, pos):
        """Station angle (deg) of a builder at world `pos` -- its polar angle."""
        return self.to_polar(pos)[1]

    # ---- builder placement ----
    def builder_stations(self, num_builders, first_deg=90.0):
        """Station angles (deg) for `num_builders` builders, evenly spaced.

        Builder 1 starts at `first_deg` (90 deg = the +Y side of the site, near
        (0, VEHICLE_RADIUS)); the rest are spread around the orbit so they build
        disjoint arcs.
        """
        return [first_deg + i * 360.0 / num_builders for i in range(num_builders)]

    def heading_deg(self, station_deg, facing="tangent"):
        """Heading (deg) of a builder at `station_deg`.

        "tangent" (default) points the builder *along* its orbit, in the CCW
        direction of travel -- the wall then runs alongside it, on its left, and
        the rock-unloading orbit alongside on its right, as in the concept sketch.
        At station 0 deg -- a builder at (`vehicle_radius`, 0) -- that heading is
        +Y. "outward" instead points it radially away from the wall.
        """
        if facing == "tangent":
            return station_deg + 90.0
        if facing == "outward":
            return station_deg
        raise ValueError(f"unknown facing {facing!r} (use 'tangent' or 'outward')")

    def vehicle_pose(self, station_deg, facing="tangent", ref_offset=0.0, height=0.0):
        """(location, rotation) for a builder parked at `station_deg`.

        The builder sits on the vehicle orbit at `station_deg`, headed per
        `facing` (see `heading_deg`). `ref_offset` shifts the returned point
        radially outward -- vehicle poses are set on the chassis *reference*
        frame, which need not be the middle of the hull, so use it to nudge the
        hull onto the orbit. `height` is the ride height above the ground.
        """
        loc = self.to_world(self.vehicle_radius + ref_offset, station_deg,
                            self.ground_z + height)
        return loc, chrono.QuatFromAngleZ(math.radians(self.heading_deg(station_deg, facing)))

    # ---- wall place points ----
    def place_angles(self, station_deg):
        """The `num_points` wall angles (deg) assigned to a builder at `station_deg`.

        Evenly spaced over [station - half_span, station + half_span], endpoints
        included -- so the step is 2 * half_span / (num_points - 1).
        """
        n = self.num_points
        if n == 1:
            return [station_deg]
        step = 2.0 * self.half_span_deg / (n - 1)
        return [station_deg - self.half_span_deg + i * step for i in range(n)]

    def place_points_polar(self, station_deg):
        """The assigned wall points as (r, theta_deg) pairs, in build order."""
        return [(self.wall_radius, a) for a in self.place_angles(station_deg)]

    def place_points(self, station_deg):
        """The assigned wall points as world `ChVector3d`s, in build order.

        These are the `PLACE_POINT`s a builder's gripper drops rocks on, one per
        rock, `place_height` above the ground on the wall orbit.
        """
        z = self.ground_z + self.place_height
        return [self.to_world(r, a, z) for r, a in self.place_points_polar(station_deg)]

    def place_points_for(self, vehicle_pos):
        """Place points for a builder that is *at* `vehicle_pos` (a world point)."""
        return self.place_points(self.station_of(vehicle_pos))

    # ---- resuming: measure what was built, then plan more on top ----
    def measure_belt(self, rows, only_placed=True, rock_height=LAYER_RISE,
                     skip_fake=True):
        """Measure the `RockBelt` the rocks in `rows` occupy.

        `rows` comes from `read_rock_state`. Each rock's world position is taken
        from its `com_*` columns (falling back to `ref_*`) and converted to this
        planner's polar frame, so the belt is measured about the site center
        whatever coordinates the CSV was written in.

        `only_placed` keeps just the rocks the run actually laid on the wall (the
        `placed` column). Leave it on: rocks a run never got to are still sitting
        in the spawn wedge next to the vehicle, several metres off the wall orbit,
        and they would stretch the measured band right across the site. It is
        ignored for CSVs with no `placed` column.

        `skip_fake` drops rows a previous run's stand-in layers wrote (`fake`),
        so re-running this against a stacked site measures the ground layer again
        rather than the stack standing on it -- otherwise each pass would measure
        its own last output and march the belt upward.

        `rock_height` is how tall one rock is, which sets the belt's top surface
        and how much each new layer rises; `plan_layers_from_state` passes the
        `course_rise` the run itself used when a site plan is available.
        """
        rocks = [row for row in rows
                 if (not only_placed or int(row.get("placed", 1)))
                 and not (skip_fake and int(row.get("fake", 0)))]
        if not rocks:
            raise ValueError("no placed rocks in the rock state "
                             f"({len(rows)} row(s) read)")

        samples = []
        for row in rocks:
            x = row.get("com_x", row.get("ref_x"))
            y = row.get("com_y", row.get("ref_y"))
            z = row.get("com_z", row.get("ref_z"))
            if x is None or y is None or z is None:
                raise ValueError("rock state rows need com_x/com_y/com_z "
                                 "(or ref_x/ref_y/ref_z) columns")
            r, theta = self.to_polar((x, y))
            samples.append((r, theta, z))

        # atan2 splits at +/-180 deg, so a belt straddling that cut reads as one
        # spanning nearly the whole circle. Unwrap it onto 0..360 instead; every
        # angle here feeds back through to_world, which does not care which turn
        # it is on.
        thetas = [s[1] for s in samples]
        if max(thetas) - min(thetas) > 180.0:
            samples = [(r, t + 360.0 if t < 0.0 else t, z) for r, t, z in samples]

        place_points = {row["place_point"] for row in rocks if "place_point" in row}
        courses = {row["course"] for row in rocks if "course" in row}
        return RockBelt(samples, rock_height=rock_height,
                        points_per_layer=len(place_points) or None,
                        layers=len(courses) or 1)

    def layer_poses(self, belt, num_layers, rocks_per_layer=None, rise=None,
                    base_z=None, r_range=None, min_sep=0.12, tilt_deg=15.0,
                    seed=None, max_attempts=200):
        """Poses for `num_layers` of *stand-in* rocks stacked on top of `belt`.

        Returns a list of layers, each a list of `(position, rotation)` pairs --
        world `ChVector3d` / `ChQuaterniond` for rock bodies to be dropped into a
        scene as-is. This is how a scenario gets its upper courses without driving
        the gripper through another N pick-and-place cycles per layer: the builder
        lays the ground layer for real, this generates the layers above it, and the
        caller spawns them fixed and collision-free (see
        `TrackedVeh_OrbitBuilder.spawn_wall_rocks`). They are scenery, not physics.

        Each layer re-samples the *measured distribution* rather than copying the
        rocks below: `rocks_per_layer` (default: as many as the belt holds)
        positions drawn uniformly from the belt's own radial band
        (`r_min`..`r_max`, or `r_range` to override) and arc (`theta_min_deg`..
        `theta_max_deg`), so an upper layer scatters across the same ring of ground
        the built layer covers. Narrowing `r_range` is what turns the stack from a
        bank into a wall: the measured belt is as wide as the rocks scattered, and
        a stack that wide is a berm unless it is taller than it is broad. Layer k sits `k * rise` above the built layer's
        centre height -- one rock height per layer, matching how the ground layer's
        own centres sit half a rock over the ground. Rotations are a random yaw
        plus a tilt of up to `tilt_deg`, so the layers do not read as one rock
        stamped out N times.

        Both knobs matter for whether the result reads as a wall or as scattered
        rubble. A layer of `belt.count` rocks a full rock height apart reproduces
        the density of the built layer, which is a sparse scatter -- fine as a
        measurement, hollow as a wall. Stacking solid wants the opposite of what
        physics would give you: several times more rocks per layer than the ground
        layer holds, and a `rise` well under one rock height so consecutive layers
        overlap rather than sit on each other. Nothing enforces separation between
        layers, so the overlap is free.

        `min_sep` re-samples positions landing within that distance of one already
        in the same layer. It defaults *below* the ~0.2 m rock width on purpose:
        these bodies never collide, so letting them interpenetrate is what closes
        the gaps between them. Since a sample that cannot be separated after
        `max_attempts` tries is kept rather than raising, `min_sep` degrades to a
        spacing preference under crowding instead of a hard limit. `seed` makes the
        scatter reproducible; the draw uses its own RNG, so it does not disturb the
        caller's `random` stream.
        """
        # As many rocks as the measured layer holds, so an upper layer matches the
        # density of the one below rather than the plan's place-point count (the
        # ground layer of `TrackedVeh_OrbitBuilder` is 20 rocks over 10 points).
        n = rocks_per_layer or belt.count
        rise = belt.rock_height if rise is None else rise
        base_z = belt.z_mean if base_z is None else base_z
        r_lo, r_hi = (belt.r_min, belt.r_max) if r_range is None else r_range
        rng = random.Random(seed)

        layers = []
        for k in range(1, num_layers + 1):
            z = base_z + k * rise
            placed = []
            for _ in range(n):
                for attempt in range(max_attempts):
                    r = rng.uniform(r_lo, r_hi)
                    theta = rng.uniform(belt.theta_min_deg, belt.theta_max_deg)
                    pos = self.to_world(r, theta, z)
                    if all((pos - p).Length() > min_sep for p, _ in placed):
                        break  # clear of the rocks already in this layer
                # Random yaw, then a small tilt about a random horizontal axis.
                yaw = chrono.QuatFromAngleZ(rng.uniform(-math.pi, math.pi))
                axis_angle = rng.uniform(0.0, 2.0 * math.pi)
                axis = chrono.ChVector3d(math.cos(axis_angle),
                                         math.sin(axis_angle), 0.0)
                tilt = chrono.QuatFromAngleAxis(
                    math.radians(rng.uniform(0.0, tilt_deg)), axis)
                placed.append((pos, tilt * yaw))
            layers.append(placed)
        return layers

    # ---- rendering helpers ----
    def ring_segments(self, radius, num_segments=72, z=None):
        """Chord segments approximating the orbit at `radius`, for drawing it.

        Returns (midpoint, yaw_rad, length) per chord -- enough to lay a thin box
        along each one. Rings are drawn as a chain of boxes rather than as a
        polyline because `ChVisualShapeLine` does not show up in the Irrlicht
        backend. At the default 72 segments a 30 m ring bows 3 cm off true, which
        does not read at scene scale.
        """
        z = self.ground_z if z is None else z
        step = 2.0 * math.pi / num_segments
        length = 2.0 * radius * math.sin(step / 2.0)
        mid_radius = radius * math.cos(step / 2.0)
        segments = []
        for i in range(num_segments):
            angle = (i + 0.5) * step
            mid = chrono.ChVector3d(self.center[0] + mid_radius * math.cos(angle),
                                    self.center[1] + mid_radius * math.sin(angle), z)
            segments.append((mid, angle + math.pi / 2.0, length))
        return segments

    # ---- reporting ----
    def describe(self, station_deg):
        """Multi-line summary of the wall points assigned around `station_deg`."""
        lines = [f"orbit plan: center ({self.center[0]:.1f}, {self.center[1]:.1f}), "
                 f"wall r={self.wall_radius:.1f} m, vehicle r={self.vehicle_radius:.1f} m",
                 f"  build station: {station_deg:.2f} deg, {self.num_points} place points over "
                 f"{station_deg - self.half_span_deg:.1f}..{station_deg + self.half_span_deg:.1f} deg "
                 f"(step {2 * self.half_span_deg / max(self.num_points - 1, 1):.3f} deg = "
                 f"{math.radians(2 * self.half_span_deg / max(self.num_points - 1, 1)) * self.wall_radius:.2f} m of wall)"]
        for i, ((r, a), p) in enumerate(zip(self.place_points_polar(station_deg),
                                            self.place_points(station_deg))):
            lines.append(f"    place {i}: polar (r={r:.1f}, theta={a:+.2f} deg) "
                         f"-> ({p.x:+.2f}, {p.y:+.2f}, {p.z:.2f})")
        return "\n".join(lines)


class WallProfile:
    """The cross-section of a length of wall standing at a constant slope.

    A dumped-rock wall does not stand up as a box; it stands at its slip angle,
    so its cross-section is a triangle. This is that triangle, in the plane cut
    radially through the site: a base of half-width `half_width` centred on the
    wall orbit (`r_center`), faces rising at `slope`, and therefore a crest at

        height = slope * half_width

    `slope` is the invariant. It is height over base half-width -- the tangent of
    the face angle -- and it is what `SlopedWallStacker.grow` holds fixed: a wall
    that gets taller gets proportionally wider, because rubble cannot do anything
    else. Pick `half_width` or `height`; the slope fixes the other.

    Along the wall the shape is a hipped ridge, not a prism. `ridge_half_length`
    is the half-length (m of arc) of the crest line, which is the wall being
    built and does not move; the base runs `half_width` beyond it at each end,
    since the ends slump at the same slope as the faces. So at height z above the
    base every horizontal dimension is pulled in by the same `inset(z)`, and the
    footprint is a rectangle in (radial offset, arc offset) shrinking uniformly
    from base to ridge.

    Heights are of rock *centres*, which is what `SlopedWallStacker` samples and
    what `OrbitPlanner.measure_belt` measures. Actual rock bodies stick out about
    half a rock beyond the profile in every direction.
    """

    def __init__(self, r_center, theta_center_deg, ridge_half_length,
                 half_width, slope=WALL_SLOPE, base_z=0.0):
        if half_width < 0.0:
            raise ValueError(f"half_width must be >= 0 (got {half_width})")
        if slope <= 0.0:
            raise ValueError(f"slope must be > 0 (got {slope})")
        self.r_center = r_center
        self.theta_center_deg = theta_center_deg
        self.ridge_half_length = ridge_half_length
        self.half_width = half_width
        self.slope = slope
        self.base_z = base_z

    # ---- derived geometry ----
    @property
    def height(self):
        """Crest height (m) above the base -- `slope` * `half_width`."""
        return self.slope * self.half_width

    @property
    def crest_z(self):
        """World height (m) of the crest line."""
        return self.base_z + self.height

    @property
    def face_angle_deg(self):
        """Angle (deg) the faces make with the ground."""
        return math.degrees(math.atan(self.slope))

    @property
    def base_half_length(self):
        """Half-length (m of arc) of the footprint at the base."""
        return self.ridge_half_length + self.half_width

    def inset(self, z):
        """How far in (m) the wall is pulled from its base outline at height `z`."""
        return (z - self.base_z) / self.slope

    def footprint_at(self, z):
        """(radial half-width, arc half-length) in m at height `z`, or (0, 0).

        Both shrink by `inset(z)`; above the crest there is no wall left.
        """
        half_width = self.half_width - self.inset(z)
        if half_width < 0.0:
            return 0.0, 0.0
        return half_width, self.ridge_half_length + half_width

    def theta_span_deg(self, z=None):
        """Angular span (deg) of the footprint at height `z` (default: the base)."""
        half_length = (self.base_half_length if z is None
                       else self.footprint_at(z)[1])
        return 2.0 * math.degrees(half_length / self.r_center)

    def volume(self):
        """Volume (m^3) enclosed by the profile -- a hipped wedge.

        A prism of triangular section along the ridge, plus the two end hips,
        which together make a pyramid: 2*L*R*H/2 + 4*R*R*H/3... in the flat
        approximation that ignores the arc's curvature, which is under 1% here.
        """
        r, h, ell = self.half_width, self.height, self.ridge_half_length
        return 2.0 * ell * r * h + 4.0 * r * r * h / 3.0

    def contains(self, u, s, z):
        """Is the point at radial offset `u`, arc offset `s`, height `z` inside?"""
        half_width, half_length = self.footprint_at(z)
        return abs(u) <= half_width and abs(s) <= half_length

    def grown(self, half_width=None, height=None):
        """A copy of this profile scaled up to `half_width` (or to `height`).

        The slope, the ridge line and the base plane all carry over, so the new
        profile is the same triangle enlarged about the base of the ridge.
        """
        if (half_width is None) == (height is None):
            raise ValueError("pass exactly one of half_width, height")
        if height is not None:
            half_width = height / self.slope
        return WallProfile(self.r_center, self.theta_center_deg,
                           self.ridge_half_length, half_width, self.slope,
                           self.base_z)

    def describe(self):
        """Multi-line summary of the profile."""
        return "\n".join([
            f"wall profile: r={self.r_center:.2f} m, theta={self.theta_center_deg:+.2f} deg, "
            f"slope {self.slope:.2f} ({self.face_angle_deg:.1f} deg faces)",
            f"  section: {2 * self.half_width:.2f} m wide at the base -> "
            f"{self.height:.2f} m high (crest z={self.crest_z:.2f} m)",
            f"  along  : {2 * self.ridge_half_length:.2f} m of ridge, "
            f"{2 * self.base_half_length:.2f} m of base "
            f"({self.theta_span_deg():.2f} deg of orbit)",
            f"  volume : {self.volume():.2f} m^3",
        ])


class SlopedWallStacker:
    """Builds a constant-slope wall out of stand-in rocks, one dump at a time.

    The wall is *prescribed*, not measured: it is laid out on the wall orbit at
    the station and arc it is told, so it does not depend on anything having been
    built first (contrast `OrbitPlanner.measure_belt` + `layer_poses`, which fit
    the stack to a belt of real rocks a gripper laid). Nothing here touches a
    simulation -- it hands back rock poses, and the caller spawns bodies at them.

    `grow()` is callable as many times as you like, and that is the point. Each
    call enlarges the wall to a new `WallProfile` -- crest higher, base wider, the
    slope between them unchanged -- and returns *only the rocks that new profile
    needs and the old one did not*, grouped by course. So a wall is built up in
    dumps:

        stacker = SlopedWallStacker(planner, theta_center_deg=90.0,
                                    ridge_half_length=2.6)
        profile, courses = stacker.grow(height=0.6)   # -> R=0.75 m, H=0.60 m
        profile, courses = stacker.grow()             # -> R=1.12 m, H=0.90 m
        profile, courses = stacker.grow(height=1.2)   # -> R=1.50 m, H=1.20 m

    and each call's rocks are laid on and around what is already standing rather
    than replacing it. Since a bigger triangle contains the smaller one, the new
    rocks are exactly the shell between the two profiles: a skin over the faces
    and a cap over the crest. That is also what makes repeated growth cheap --
    only the first call fills a solid section.

    Courses are horizontal planes of rock centres `rise` apart, starting half a
    rise above the base. Each gets `density` rocks per m^2 of its own footprint,
    so the rock is spread evenly through the section and the courses narrow as
    they climb. Rotations are a random yaw plus up to `tilt_deg` of tilt, so the
    faces do not read as one rock stamped out N times.

    `seed` makes a whole build reproducible; the draw uses its own RNG so it does
    not disturb the caller's `random` stream. Note that the stream carries across
    calls -- growing in two steps does not give the same rocks as growing to the
    same profile in one.
    """

    def __init__(self, planner, theta_center_deg, ridge_half_length=None,
                 ridge_span_deg=None, r_center=None, base_z=None,
                 slope=WALL_SLOPE, rise=WALL_COURSE_RISE,
                 density=WALL_COURSE_DENSITY, min_sep=WALL_MIN_SEP,
                 tilt_deg=WALL_TILT_DEG, growth_rise=WALL_GROWTH_RISE,
                 seed=None, max_attempts=200):
        self.planner = planner
        self.r_center = planner.wall_radius if r_center is None else r_center
        self.theta_center_deg = theta_center_deg
        if (ridge_half_length is None) == (ridge_span_deg is None):
            raise ValueError("pass exactly one of ridge_half_length, ridge_span_deg")
        if ridge_half_length is None:
            ridge_half_length = 0.5 * math.radians(ridge_span_deg) * self.r_center
        self.rise = rise
        self.density = density
        self.min_sep = min_sep
        self.tilt_deg = tilt_deg
        self.growth_rise = growth_rise
        self.max_attempts = max_attempts
        self._rng = random.Random(seed)
        # Stage 0: the wall line is laid out, but nothing is standing on it yet.
        self.profile = WallProfile(self.r_center, theta_center_deg,
                                   ridge_half_length, 0.0, slope,
                                   planner.ground_z if base_z is None else base_z)
        self.stage = 0
        self.rock_count = 0
        self.course_z = []     # centre height of every course laid so far

    @property
    def slope(self):
        return self.profile.slope

    # ---- growth ----
    def grow(self, half_width=None, height=None, add_half_width=None,
             add_height=None):
        """Enlarge the wall and return `(profile, courses)` for the rocks to add.

        Give the new size as a base `half_width` or a crest `height`, or as an
        increment on the current one (`add_half_width` / `add_height`); with no
        argument the crest rises by `growth_rise`. Whichever you give, the slope
        fixes the rest of the profile -- that is the constant this class keeps.

        `courses` is a list of courses (lowest first), each a list of
        `(position, rotation)` pairs: world `ChVector3d` / `ChQuaterniond` ready
        to spawn rock bodies at, and *only* the rocks this call adds. Courses the
        call does not touch are left out, so the list is not a full section of the
        wall -- `profile` is the description of the whole thing.
        """
        target = self._target_half_width(half_width, height, add_half_width,
                                         add_height)
        if target <= self.profile.half_width + 1e-9:
            raise ValueError(
                f"the wall is already {self.profile.half_width:.3f} m half-width "
                f"({self.profile.height:.3f} m high); grow() only builds up, and "
                f"{target:.3f} m is not bigger")
        old, new = self.profile, self.profile.grown(half_width=target)

        courses = []
        for z in self._course_heights(new):
            poses = self._sample_course(z, new, old)
            if poses:
                courses.append(poses)
                if z not in self.course_z:
                    self.course_z.append(z)
        self.profile = new
        self.stage += 1
        self.rock_count += sum(len(c) for c in courses)
        return new, courses

    def _target_half_width(self, half_width, height, add_half_width, add_height):
        """Resolve the four mutually exclusive size arguments to a half-width."""
        given = [x for x in (half_width, height, add_half_width, add_height)
                 if x is not None]
        if len(given) > 1:
            raise ValueError("grow() takes at most one of half_width, height, "
                             "add_half_width, add_height")
        if half_width is not None:
            return half_width
        if height is not None:
            return height / self.slope
        if add_half_width is not None:
            return self.profile.half_width + add_half_width
        rise = self.growth_rise if add_height is None else add_height
        return self.profile.half_width + rise / self.slope

    def _course_heights(self, profile):
        """Centre heights of every course the profile holds, lowest first.

        The first sits half a rise above the base, so a course of rocks is bedded
        on the ground rather than half-buried in it, and they step up by `rise`
        until the next one would poke out of the crest.
        """
        n = int(math.floor(profile.height / self.rise + 1e-9))
        return [profile.base_z + (k + 0.5) * self.rise for k in range(n)]

    def _sample_course(self, z, new, old):
        """Poses for the rocks course `z` gains going from profile `old` to `new`.

        The course's footprint is a rectangle in (radial offset, arc offset) and
        the old wall's is a smaller rectangle inside it, so the rocks to add are
        the frame between them -- sampled by rejection, which is cheap because
        the frame is never a thin sliver of the outer rectangle at the heights
        that hold most of the rock.
        """
        half_width, half_length = new.footprint_at(z)
        if half_width <= 0.0:
            return []
        old_half_width, old_half_length = old.footprint_at(z)
        area = 4.0 * (half_width * half_length
                      - old_half_width * old_half_length)
        count = int(round(self.density * area))
        if count <= 0:
            return []

        poses = []
        for _ in range(count):
            pos = None
            for _ in range(self.max_attempts):
                u = self._rng.uniform(-half_width, half_width)
                s = self._rng.uniform(-half_length, half_length)
                if abs(u) < old_half_width and abs(s) < old_half_length:
                    continue          # inside what is already standing
                candidate = self._to_world(u, s, z)
                if pos is None:
                    pos = candidate   # usable; keep looking for a roomier spot
                if all((candidate - p).Length() > self.min_sep for p, _ in poses):
                    pos = candidate
                    break
            if pos is None:
                break                 # this course is full of the old wall
            poses.append((pos, self._random_rotation()))
        return poses

    def _to_world(self, u, s, z):
        """World point at radial offset `u`, arc offset `s` (m), height `z`."""
        return self.planner.to_world(
            self.r_center + u,
            self.theta_center_deg + math.degrees(s / self.r_center), z)

    def _random_rotation(self):
        """A random yaw, then a small tilt about a random horizontal axis."""
        yaw = chrono.QuatFromAngleZ(self._rng.uniform(-math.pi, math.pi))
        axis_angle = self._rng.uniform(0.0, 2.0 * math.pi)
        axis = chrono.ChVector3d(math.cos(axis_angle), math.sin(axis_angle), 0.0)
        tilt = chrono.QuatFromAngleAxis(
            math.radians(self._rng.uniform(0.0, self.tilt_deg)), axis)
        return tilt * yaw

    # ---- reporting ----
    def plan(self, half_width=None, height=None, add_half_width=None,
             add_height=None):
        """The course table `grow()` would produce, without producing it.

        Returns `(profile, rows)` where each row is
        `(z, half_width, half_length, rocks)` for one course of the enlarged
        wall -- including the courses that gain nothing, so it reads as a section
        of the whole wall. Rock counts are the same integers `grow` uses.
        """
        target = self._target_half_width(half_width, height, add_half_width,
                                         add_height)
        old, new = self.profile, self.profile.grown(half_width=target)
        rows = []
        for z in self._course_heights(new):
            half_w, half_l = new.footprint_at(z)
            old_w, old_l = old.footprint_at(z)
            count = int(round(self.density * 4.0
                              * (half_w * half_l - old_w * old_l)))
            rows.append((z, half_w, half_l, max(count, 0)))
        return new, rows

    def describe(self):
        """Multi-line summary of what is standing after the calls made so far."""
        return "\n".join([
            f"sloped wall: stage {self.stage}, {self.rock_count} stand-in rocks "
            f"in {len(self.course_z)} course(s) {self.rise:.2f} m apart",
            self.profile.describe(),
        ])


def plan_layers_from_state(rock_state_csv, num_layers, site_plan_csv=None,
                           planner=None, only_placed=True, skip_fake=True,
                           rocks_per_layer=None, rise=None, base_z=None, seed=None):
    """Read a run's end state and stack `num_layers` of stand-in rocks on it.

    The whole path in one call: read `rock_state_csv` (and the site plan beside
    it), rebuild the planner that run used, measure the belt its rocks ended up
    forming, and sample that many more layers over the same ring. Returns
    `(planner, belt, layers)` -- the belt is the measurement (print
    `belt.describe()`), and `layers` is a list of `num_layers` lists of
    `(position, rotation)` pairs to spawn rocks at.

    `site_plan_csv` defaults to a `site_plan.csv` sitting next to the rock state,
    and is optional: without one, the site geometry falls back to this module's
    defaults and the rock height to `LAYER_RISE`. Pass `planner` to measure
    against a site you have already built. The remaining arguments are the
    `measure_belt`/`layer_poses` overrides.
    """
    if site_plan_csv is None:
        beside = os.path.join(os.path.dirname(os.path.abspath(rock_state_csv)),
                              SITE_PLAN_FILE)
        site_plan_csv = beside if os.path.exists(beside) else None
    plan = read_site_plan(site_plan_csv) if site_plan_csv else {}

    if planner is None:
        planner = OrbitPlanner.from_site_plan(plan) if plan else OrbitPlanner()
    # The run's own rock height is what produced this belt. Older plans recorded
    # it as the rise between courses of real rocks, which was the same number.
    rock_height = plan.get("rock_height", plan.get("course_rise", LAYER_RISE))

    rocks = read_rock_state(rock_state_csv)
    belt = planner.measure_belt(rocks, only_placed=only_placed,
                                rock_height=rock_height, skip_fake=skip_fake)
    layers = planner.layer_poses(belt, num_layers, rocks_per_layer=rocks_per_layer,
                                 rise=rise, base_z=base_z, seed=seed)
    return planner, belt, layers


if __name__ == "__main__":
    if len(sys.argv) > 1:
        # A run's rock_state.csv: measure what it built and plan more on top.
        rock_state_csv = sys.argv[1]
        num_layers = int(sys.argv[2]) if len(sys.argv) > 2 else 2
        planner, belt, layers = plan_layers_from_state(rock_state_csv, num_layers,
                                                       seed=0)
        print(f"read {rock_state_csv}")
        print(belt.describe())
        print(f"  {num_layers} stand-in layer(s) on top, {len(layers[0])} rocks each:")
        for k, layer in enumerate(layers, start=1):
            zs = [p.z for p, _ in layer]
            rs = [planner.to_polar(p)[0] for p, _ in layer]
            print(f"    layer {k}: z={zs[0]:.2f} m, r {min(rs):.2f}..{max(rs):.2f} m, "
                  f"{len(layer)} rocks")
            for i, (p, _) in enumerate(layer):
                r, theta = planner.to_polar(p)
                print(f"      rock {i}: polar (r={r:.2f}, theta={theta:+.2f} deg) "
                      f"-> ({p.x:+.2f}, {p.y:+.2f}, {p.z:.2f})")
    else:
        planner = OrbitPlanner()
        station = planner.builder_stations(1)[0]  # builder 1: the +Y side of the site
        print(planner.describe(station))
        # ...and the wall those points are on the line of, grown in three dumps.
        # Nothing is measured here: the wall is prescribed, so this needs no run.
        stacker = SlopedWallStacker(planner, station,
                                    ridge_span_deg=2 * planner.half_span_deg, seed=0)
        print()
        for height in (0.6, 0.9, 1.2):
            profile, courses = stacker.grow(height=height)
            print(f"  dump {stacker.stage}: +{sum(len(c) for c in courses)} rocks -> "
                  f"base {2 * profile.half_width:.2f} m, crest {profile.height:.2f} m, "
                  f"H/R={profile.height / profile.half_width:.2f}")
        print(stacker.describe())
