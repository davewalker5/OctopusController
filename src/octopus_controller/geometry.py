"""Small geometry operations shared by sensing and movement, using pixel coordinates."""

from math import cos, isfinite, sin

Point = tuple[float, float]


def validate_point(point: Point) -> None:
    """Check that a coordinate pair is finite.

    :param point: Two coordinates in world pixels.
    :raises ValueError: If the point has the wrong size or non-finite coordinates.
    """
    if len(point) != 2 or not all(isfinite(value) for value in point):
        raise ValueError("A point must contain two finite coordinates")


def closest_point(start: Point, end: Point, point: Point) -> Point:
    """Find the nearest position on a finite segment, including its endpoints.

    :param start: Validated segment start in pixels.
    :param end: Validated segment end in pixels.
    :param point: Validated position to project onto the segment.
    :return: Closest position, or start when the segment has zero length.
    """
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_squared = dx * dx + dy * dy
    fraction = 0.0
    if length_squared > 0:
        fraction = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared
    # Restrict the projection to the actual segment, not an infinite line.
    fraction = max(0.0, min(1.0, fraction))
    return start[0] + fraction * dx, start[1] + fraction * dy


def pose_points(base: Point, angles: list[float], length: float) -> list[Point]:
    """Rebuild a candidate pose without modifying the live arm.

    :param base: Fixed attachment in world pixels.
    :param angles: Relative joint angles in radians; the first is an absolute heading.
    :param length: Fixed segment length in pixels.
    :return: Connected joint positions including the tip.
    """
    points = [base]
    heading = 0.0
    for angle in angles:
        heading += angle
        points.append(
            (points[-1][0] + length * cos(heading), points[-1][1] + length * sin(heading))
        )
    return points
