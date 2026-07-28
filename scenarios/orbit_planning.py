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

Standalone (prints the plan for the default site, no simulation):

    conda run -n chrono python scenarios/orbit_planning.py
"""

import math

import pychrono as chrono

# Default site geometry (m). The wall is the ring being built; the builders ride
# an orbit 3 m outside it, and the fetchers unload rocks a further ~2 m out.
WALL_RADIUS = 30.0
VEHICLE_RADIUS = 33.0
UNLOAD_RADIUS = 35.0

# Default place-point assignment for one parked builder.
NUM_PLACE_POINTS = 10
HALF_SPAN_DEG = 5.0

# Height (m above ground) the gripper releases a rock at over a wall point.
PLACE_HEIGHT = 0.35


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


if __name__ == "__main__":
    planner = OrbitPlanner()
    station = planner.builder_stations(1)[0]  # builder 1: the +Y side of the site
    print(planner.describe(station))
