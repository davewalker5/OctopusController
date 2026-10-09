"""Local obstacle steering and conservative checks of the resulting arm motion.

The steering rule sees only one segment's attachment and requested endpoint.
It follows nearby circular boundaries clockwise, without a map or a planned
whole-arm route. A separate check enforces obstacle clearance during movement;
it can reject a pose, but never invents a different route or changes the target.
"""

from math import atan2, ceil, cos, hypot, sin

from octopus_controller.geometry import Point, closest_point, pose_points
from octopus_controller.segment_control import wrap_angle
from octopus_controller.sensing import WorldObject

ARM_CLEARANCE = 3.0  # Effective arm radius, independent of decorative line widths.
STEERING_MARGIN = 8.0  # Begin steering before the hard movement limit is reached.
WAYPOINT_MARGIN = 5.0  # Extra room for the short chord towards the boundary waypoint.
BOUNDARY_STEP = 0.4  # Radians to aim ahead around a circular boundary.
LOOKAHEAD_SEGMENTS = 3.0
MAXIMUM_SAMPLE_TRAVEL = 2.0  # Pixels; sample spacing is tightened for fast turns.
GEOMETRY_TOLERANCE = 1e-8


def steer_around_obstacles(
    start: Point, goal: Point, length: float, obstacles: tuple[WorldObject, ...]
) -> tuple[Point, bool]:
    """Redirect a local request around the first nearby blocking boundary.

    :param start: This segment's attachment in world pixels.
    :param goal: Endpoint requested by its neighbour, or the tip's target.
    :param length: Positive segment length, used to set the local lookahead distance.
    :param obstacles: Validated obstacle circles; food must be filtered out by the caller.
    :return: Steering endpoint and whether a nearby obstacle changed the request.
    """
    dx, dy = goal[0] - start[0], goal[1] - start[1]
    distance = hypot(dx, dy)
    if distance <= GEOMETRY_TOLERANCE:
        return goal, False

    # Inspect only the next few segment lengths. A distant obstacle does not
    # affect this segment until the neighbour request brings it within this area.
    lookahead = min(distance, length * LOOKAHEAD_SEGMENTS)
    end = (start[0] + dx / distance * lookahead, start[1] + dy / distance * lookahead)
    nearest: WorldObject | None = None
    nearest_distance = float("inf")
    for obstacle in obstacles:
        closest = closest_point(start, end, obstacle.centre)
        separation = hypot(closest[0] - obstacle.centre[0], closest[1] - obstacle.centre[1])
        steering_radius = obstacle.radius + ARM_CLEARANCE + STEERING_MARGIN
        if separation >= steering_radius:
            continue
        distance_from_start = hypot(start[0] - obstacle.centre[0], start[1] - obstacle.centre[1])
        if distance_from_start < nearest_distance:
            nearest, nearest_distance = obstacle, distance_from_start

    if nearest is None:
        return goal, False

    # Always prefer clockwise travel. This deterministic convention prevents
    # tiny numerical changes from making a segment alternate between two sides.
    # Neighbours share that convention, but calculate their own local requests.
    radial_heading = atan2(start[1] - nearest.centre[1], start[0] - nearest.centre[0])
    heading = radial_heading + BOUNDARY_STEP
    radius = nearest.radius + ARM_CLEARANCE + STEERING_MARGIN + WAYPOINT_MARGIN
    return (
        nearest.centre[0] + radius * cos(heading),
        nearest.centre[1] + radius * sin(heading),
    ), True


def safe_motion(
    base: Point,
    before: list[float],
    after: list[float],
    length: float,
    obstacles: tuple[WorldObject, ...],
) -> bool:
    """Reject obstacle crossings anywhere along a proposed angular movement.

    :param base: Fixed arm attachment in pixels.
    :param before: Current relative joint angles in radians.
    :param after: Proposed angles, already respecting bend and speed limits.
    :param length: Fixed segment length in pixels.
    :param obstacles: Validated obstacle circles; food is excluded.
    :return: Whether the movement maintains clearance or avoids worsening overlap.
    """
    if not obstacles:
        return True
    previous_points = pose_points(base, before, length)

    # Only the base wraps freely. Relative bends must interpolate directly:
    # taking a shortcut across ±pi there could pass outside the permitted range.
    changes = [new - old for old, new in zip(before, after)]
    changes[0] = wrap_angle(changes[0])
    heading_change = 0.0
    travel = 0.0
    travel_bounds = []
    for change in changes:
        heading_change += change

        # Arc length bounds how far each link's endpoint can travel. Adding the
        # upstream links also covers motion inherited from their rotations.
        travel += length * abs(heading_change)
        travel_bounds.append(travel)
    samples = max(1, ceil(travel / MAXIMUM_SAMPLE_TRAVEL))

    for sample in range(samples):
        fraction = (sample + 0.5) / samples
        angles = [old + fraction * change for old, change in zip(before, changes)]
        points = pose_points(base, angles, length)
        for index, (start, end) in enumerate(zip(points, points[1:])):
            # Between a sample and either edge of its time interval, any point
            # of this segment moves by at most this padding. Inflating the circle
            # by that bound prevents a thin obstacle being skipped between samples.
            padding = travel_bounds[index] / (2 * samples)
            for obstacle in obstacles:
                old_closest = closest_point(
                    previous_points[index], previous_points[index + 1], obstacle.centre
                )
                old_distance = hypot(
                    old_closest[0] - obstacle.centre[0], old_closest[1] - obstacle.centre[1]
                )
                minimum_distance = min(obstacle.radius + ARM_CLEARANCE, old_distance)
                closest = closest_point(start, end, obstacle.centre)
                distance = hypot(closest[0] - obstacle.centre[0], closest[1] - obstacle.centre[1])

                # A dragged obstacle may already overlap the arm. Allow a safe
                # retreat, but never teleport the arm or accept a deeper overlap.
                if distance - padding < minimum_distance - GEOMETRY_TOLERANCE:
                    return False
    return True
