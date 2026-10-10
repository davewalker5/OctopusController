"""Emit deterministic Python reference traces for the browser model tests."""

import json
import sys
from pathlib import Path

from octopus_controller.avoidance import safe_motion, steer_around_obstacles
from octopus_controller.grasping import Grip
from octopus_controller.model import Arm, ArmParameters, ReachController
from octopus_controller.organism import CentralController
from octopus_controller.scenario import _unique_keys, load_scenario, parse_scenario
from octopus_controller.segment_control import request_start, turn_segment, wrap_angle
from octopus_controller.sensing import Environment, ObjectKind


def snapshot(central, environment):
    """Keep numerical state, local messages and discrete events observable."""
    return {
        "seconds": central.simulation_seconds,
        "count": central.capture_count,
        "reports": [
            [r.arm_index, r.object_id, r.simulation_seconds] for r in central.capture_reports
        ],
        "objects": [
            [o.identifier, list(o.centre), o.kind.value, o.radius] for o in environment.objects
        ],
        "arms": [
            {
                "angles": a.controller.arm.angles,
                "points": a.controller.arm.points,
                "target": a.controller.target,
                "active": a.active,
                "state": a.state.value,
                "status": a.controller.status,
                "grip": None
                if a.grip is None
                else [a.grip.object_id, a.grip.segment_index, a.grip.offset],
                "contacts": [
                    [c.segment_index, c.object_id, c.kind.value, c.position] for c in a.contacts
                ],
                "requests": [[r.start, r.heading] for r in a.controller._requests],
            }
            for a in central.arms
        ],
    }


def trace(name):
    """Run file scenarios or a controlled ownership/carry experiment."""
    if name != "grasp":
        central, environment = load_scenario(Path("scenarios") / f"{name}.json").build()
    else:
        central = CentralController((400, 460))
        environment = Environment()
        for arm in central.arms:
            arm.idle()
        for arm in central.arms[:2]:
            arm.controller = ReachController(
                Arm((400, 460), ArmParameters(segment_count=4, segment_length=12)), (440, 470)
            )
            arm.active = True
        environment.add(central.arm(0).controller.arm.points[1], ObjectKind.FOOD, 4)
        central.refresh_sensing(environment.objects)
    result = [snapshot(central, environment)]
    for tick in range(1, 601):
        if name == "grasp":
            if tick == 31:
                central.arm(0).assign_reach((425, 435))
            elif tick == 80:
                environment.add((430, 442), ObjectKind.OBSTACLE, 9)
            elif tick == 100:
                environment.remove(2)
            elif tick == 160:
                central.arm(0).retract()
            elif tick == 220:
                environment.move(1, (450, 420))
                central.refresh_sensing(environment.objects)
            elif tick == 300:
                environment.remove(1)
        central.step(1 / 120, environment)
        if tick in (1, 29, 30, 31, 60, 81, 101, 161, 221, 301, 600):
            result.append(snapshot(central, environment))
    return result


def geometry_cases():
    """Boundary, swept collision and attachment samples independent of long traces."""
    environment = Environment()
    environment.add((15, 5), ObjectKind.OBSTACLE, 2)
    obstacles = environment.objects
    arm = Arm((0, 0), ArmParameters(segment_count=3, segment_length=10))
    food = Environment()
    food.add(arm.points[1], ObjectKind.FOOD, 2)
    grip = Grip.attach(food.objects[0], 1, arm.points)
    return {
        "wrap": [wrap_angle(v) for v in (-100, -3.141592653589793, 0, 3.141592653589793, 100)],
        "steer": [
            steer_around_obstacles((0, 0), goal, 10, obstacles)
            for goal in ((0, 0), (30, 10), (-20, 0))
        ],
        "safe": [
            safe_motion((0, 0), [0, 0, 0], after, 10, obstacles)
            for after in ([1.5, 0, 0], [0, 0, 0], [-0.1, 0.02, 0.02])
        ],
        "request": [
            [r.start, r.heading]
            for r in [
                request_start((0, 0), goal, 0.2, neighbour, 10, 1)
                for goal, neighbour in [((0, 0), None), ((-20, 3), -3), ((2, 1), 2)]
            ]
        ],
        "turn": [
            turn_segment((0, 0), goal, 0.2, -0.1, bend, 1.5, 1 / 120)
            for goal, bend in [((0, 0), None), ((-20, 3), 1), ((2, 1), None)]
        ],
        "payload": [
            grip.safe_motion((0, 0), arm.angles, after, 10, 2, obstacles)
            for after in ([1.5, 0, 0], arm.angles, [-0.2, 0.06, 0.06])
        ],
    }


if len(sys.argv) > 1 and sys.argv[1] == "validate":
    results = []
    for text in json.load(sys.stdin):
        try:
            scenario = parse_scenario(json.loads(text, object_pairs_hook=_unique_keys))
            central, environment = scenario.build()
            results.append(
                {"valid": True, "name": scenario.name, "snapshot": snapshot(central, environment)}
            )
        except ValueError, OverflowError, TypeError:
            results.append({"valid": False})
    print(json.dumps(results))
else:
    print(
        json.dumps(
            {
                "traces": {
                    name: trace(name) for name in ("two-targets", "behaviour-showcase", "grasp")
                },
                "geometry": geometry_cases(),
            }
        )
    )
