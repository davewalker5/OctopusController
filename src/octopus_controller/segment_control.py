"""Local movement rules shared by segments; no rule can inspect the whole arm.

Positions are screen pixels, headings are clockwise radians from the positive
x axis, and bends are angles relative to the preceding segment. A request is a
message to a neighbour, not an instruction to move a joint there immediately.
"""

from dataclasses import dataclass
from math import atan2, cos, exp, hypot, pi, sin

Point = tuple[float, float]
DIRECTION_TOLERANCE = 1e-9
STEERING_RESPONSE = 20.0  # Per second: ease small angular errors towards zero.


@dataclass(frozen=True)
class SegmentRequest:
    """Where a segment would like its start, and its proposed absolute heading."""

    start: Point
    heading: float


def wrap_angle(angle: float) -> float:
    """Choose the equivalent angle in [-pi, pi), avoiding unnecessary full turns.

    :param angle: Angle in radians.
    :return: The same direction represented within one turn.
    """
    return (angle + pi) % (2 * pi) - pi


def request_start(
    start: Point,
    end_goal: Point,
    heading: float,
    neighbour_heading: float | None,
    length: float,
    maximum_bend: float,
) -> SegmentRequest:
    """Ask the preceding neighbour for a useful attachment position.

    Inputs come from one segment and its immediate tip-side neighbour. The last
    segment receives the target as its end goal instead. The owning arm validates
    geometry and parameters before these internal numerical rules are called.

    :param start: This segment's current attachment position in pixels.
    :param end_goal: End position requested by the tip-side neighbour, or target.
    :param heading: This segment's current absolute heading in radians.
    :param neighbour_heading: Neighbour's proposed heading, or None at the tip.
    :param length: Positive fixed segment length in pixels.
    :param maximum_bend: Permitted bend between neighbouring segments in radians.
    :return: A request for this segment's start and its associated heading.
    """
    # Aim from our own attachment towards the neighbour's requested position.
    # If they coincide, there is no direction to measure; keep the current one
    # rather than letting atan2(0, 0) introduce an arbitrary rightward turn.
    dx, dy = end_goal[0] - start[0], end_goal[1] - start[1]
    direction = atan2(dy, dx) if hypot(dx, dy) > DIRECTION_TOLERANCE else heading

    if neighbour_heading is not None:
        # A request must allow the next segment to join within its bend limit.
        # Without this check, neighbours can keep asking for an impossible fold,
        # leaving the visible arm pressed against its limit indefinitely.
        difference = wrap_angle(direction - neighbour_heading)
        direction = neighbour_heading + max(-maximum_bend, min(maximum_bend, difference))

    # Walk one segment length backwards from the requested endpoint. This is
    # where our attachment would need to be for that endpoint to be achievable.
    # We only send the request; the base and motion limits may prevent fulfilment.
    requested_start = (
        end_goal[0] - length * cos(direction),
        end_goal[1] - length * sin(direction),
    )
    return SegmentRequest(requested_start, wrap_angle(direction))


def turn_segment(
    start: Point,
    end_goal: Point,
    parent_heading: float,
    angle: float,
    maximum_bend: float | None,
    turning_speed: float,
    elapsed_seconds: float,
) -> float:
    """Choose a bounded turn using only this segment and its adjacent attachment.

    :param start: Current attachment position, supplied by the base-side neighbour.
    :param end_goal: Endpoint requested by the tip-side neighbour, or target.
    :param parent_heading: Base-side neighbour's absolute heading in radians.
    :param angle: Current bend relative to that heading, in radians.
    :param maximum_bend: Allowed relative bend, or None for a freely rotating base.
    :param turning_speed: Maximum bend change in radians per second.
    :param elapsed_seconds: Validated non-negative update duration in seconds.
    :return: Updated relative angle; no positions or inputs are modified.
    """
    dx, dy = end_goal[0] - start[0], end_goal[1] - start[1]
    if hypot(dx, dy) <= DIRECTION_TOLERANCE:
        return angle

    # Convert the desired world heading into a bend relative to our parent.
    # The shortest angular difference handles headings across the ±pi boundary.
    desired_angle = atan2(dy, dx) - parent_heading
    difference = wrap_angle(desired_angle - angle)

    # Requests take time to travel through the arm. Ease towards small errors
    # so a segment does not repeatedly overshoot while its neighbours catch up.
    # Exponential easing uses seconds, rather than assuming a display frame rate.
    change = difference * (1 - exp(-STEERING_RESPONSE * elapsed_seconds))
    maximum_change = turning_speed * elapsed_seconds
    updated_angle = angle + max(-maximum_change, min(maximum_change, change))
    if maximum_bend is None:
        return wrap_angle(updated_angle)

    return max(-maximum_bend, min(maximum_bend, updated_angle))
