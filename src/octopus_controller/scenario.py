"""Validate versioned JSON starting conditions and build fresh simulation scenes."""

import json
from dataclasses import dataclass, replace
from math import isfinite, radians
from pathlib import Path

from octopus_controller.geometry import Point
from octopus_controller.model import ArmParameters
from octopus_controller.organism import ARM_COUNT, DEFAULT_PARAMETERS, CentralController
from octopus_controller.sensing import OBJECT_RADIUS, Environment, ObjectKind, WorldObject

DEFAULT_BODY = (400.0, 460.0)


@dataclass(frozen=True)
class ArmSetup:
    """Resolved settings and optional starting objective for one arm."""

    parameters: ArmParameters
    target: Point | None = None


@dataclass(frozen=True)
class Scenario:
    """Immutable initial conditions, retained independently of a running scene."""

    name: str
    body: Point
    objects: tuple[WorldObject, ...]
    arms: tuple[ArmSetup, ...]

    def build(self) -> tuple[CentralController, Environment]:
        """Return a fresh scene with reset poses, sensing, time and capture state."""
        # Rebuild mutable runtime state rather than reusing a previous run: poses,
        # grips and capture history must never leak into a restart.
        central = CentralController(self.body)
        environment = Environment()
        for obj in self.objects:
            environment.add(obj.centre, obj.kind, obj.radius)
        for arm, setup in zip(central.arms, self.arms):
            arm.reset_pose(setup.parameters)
            if setup.target is None:
                arm.idle()
            else:
                arm.assign_reach(setup.target)

        # Populate contact indicators immediately, even though playback starts paused.
        central.refresh_sensing(environment.objects)
        return central, environment


def _mapping(value: object, allowed: set[str], location: str) -> dict:
    """Require an object and reject unknown keys so misspellings are visible."""
    if not isinstance(value, dict):
        raise ValueError(f"{location} must be an object")
    unknown = value.keys() - allowed
    if unknown:
        raise ValueError(f"{location}: unknown field {sorted(unknown)[0]!r}")
    return value


def _array(value: object, location: str) -> list:
    """Require a JSON array at the given field path."""
    if not isinstance(value, list):
        raise ValueError(f"{location} must be an array")
    return value


def _number(value: object, location: str) -> float:
    """Require a finite JSON number, excluding booleans and overflowing integers."""
    try:
        # bool is an int subclass in Python, but JSON true/false are not measurements.
        if type(value) in (int, float) and isfinite(value):
            return float(value)
    except OverflowError:
        # Huge JSON integers can overflow the float conversion used by isfinite;
        # report the same field-specific error as other invalid numbers.
        pass
    raise ValueError(f"{location} must be a finite number")


def _point(value: object, location: str) -> Point:
    """Read exactly two finite coordinates in screen pixels."""
    values = _array(value, location)
    if len(values) != 2:
        raise ValueError(f"{location} must contain exactly two coordinates")
    return _number(values[0], location), _number(values[1], location)


def _parameters(value: object, base: ArmParameters, location: str) -> ArmParameters:
    """Resolve partial overrides, converting explicitly named degree fields."""
    # Keep human-readable degree units at the file boundary; the model uses radians.
    fields = {
        "segment_count": "segment_count",
        "segment_length": "segment_length",
        "maximum_bend_degrees": "maximum_bend",
        "turning_speed_degrees": "turning_speed",
    }

    data = _mapping(value, set(fields), location)
    overrides = {}
    for key, raw in data.items():
        if key == "segment_count":
            if type(raw) is not int or not 2 <= raw <= 100:
                raise ValueError(f"{location}.{key} must be an integer between 2 and 100")
            overrides[fields[key]] = raw
        else:
            number = _number(raw, f"{location}.{key}")
            if number <= 0 or (key == "maximum_bend_degrees" and number > 180):
                raise ValueError(
                    f"{location}.{key} must be positive"
                    + (" and at most 180" if key == "maximum_bend_degrees" else "")
                )
            overrides[fields[key]] = radians(number) if key.endswith("_degrees") else number

    # Replacing only supplied fields preserves inherited settings. Construction
    # also runs the model validation after conversion (including underflow to zero).
    try:
        return replace(base, **overrides)
    except ValueError as error:
        raise ValueError(f"{location}: {error}") from error


def parse_scenario(value: object) -> Scenario:
    """Validate a decoded version-1 document without modifying any running scene.

    :param value: JSON document returned by json.loads.
    :return: Immutable, fully resolved starting conditions.
    :raises ValueError: If a field, parameter or food reference is invalid.
    """
    # Validate the envelope before interpreting nested fields. An exact integer
    # check prevents True or 1.0 from being accepted as format version 1.
    data = _mapping(
        value, {"version", "name", "body", "defaults", "food", "obstacles", "arms"}, "Scenario"
    )
    if type(data.get("version")) is not int or data["version"] != 1:
        raise ValueError("Scenario.version must be 1")

    name = data.get("name", "Untitled scenario")
    if not isinstance(name, str) or not name.strip():
        raise ValueError("Scenario.name must be a non-empty string")

    body = _point(data.get("body", list(DEFAULT_BODY)), "Scenario.body")

    # Resolve shared overrides once; each arm later inherits this complete set
    # before applying its own partial overrides.
    defaults = _parameters(data.get("defaults", {}), DEFAULT_PARAMETERS, "defaults")

    # Read objects before arm assignments so food references work regardless of
    # top-level JSON field order. Named food IDs are lookup labels, separate from
    # the numeric identities used by the simulation for contacts and grips.
    objects = []
    food_positions = {}
    for collection, kind in (("food", ObjectKind.FOOD), ("obstacles", ObjectKind.OBSTACLE)):
        for index, raw in enumerate(_array(data.get(collection, []), collection)):
            # Include the collection and index in errors to identify the bad entry.
            location = f"{collection}[{index}]"
            allowed = {"position", "radius"} | ({"id"} if kind is ObjectKind.FOOD else set())
            obj = _mapping(raw, allowed, location)
            position = _point(obj.get("position"), f"{location}.position")
            radius = _number(obj.get("radius", OBJECT_RADIUS), f"{location}.radius")
            if radius <= 0:
                raise ValueError(f"{location}.radius must be positive")
            if "id" in obj:
                identifier = obj["id"]
                if not isinstance(identifier, str) or not identifier.strip():
                    raise ValueError(f"{location}.id must be a non-empty string")
                if identifier in food_positions:
                    raise ValueError(f"{location}: duplicate food id {identifier!r}")

                # Retain the exact ID: validation rejects blank labels but does not
                # trim whitespace or change case when matching references.
                food_positions[identifier] = position

            # Stable insertion order reproduces these numeric IDs when build()
            # adds the objects to a fresh environment.
            objects.append(WorldObject(len(objects) + 1, position, kind, radius))

    # All eight arms exist even if omitted from the file. Seed idle setups, then
    # replace the explicitly numbered entries without depending on their order.
    arms = [ArmSetup(defaults) for _ in range(ARM_COUNT)]
    seen = set()
    for raw in _array(data.get("arms", []), "arms"):
        arm = _mapping(raw, {"number", "parameters", "target"}, "Arm")
        number = arm.get("number")
        if type(number) is not int or not 1 <= number <= ARM_COUNT:
            raise ValueError("Arm.number must be an integer between 1 and 8")

        # Reject repeated assignments instead of silently letting the last win.
        if number in seen:
            raise ValueError(f"Duplicate arm number {number}")

        seen.add(number)
        parameters = _parameters(arm.get("parameters", {}), defaults, f"Arm {number}.parameters")

        # Omission means idle. An explicit null or empty target is an error, so
        # check presence rather than treating false-like values as omission.
        target = None
        if "target" in arm:
            assignment = _mapping(arm["target"], {"food", "position"}, f"Arm {number}.target")

            # The allowed-key check above plus this count guarantees one target
            # form, avoiding ambiguous food and coordinate assignments.
            if len(assignment) != 1:
                raise ValueError(
                    f"Arm {number}.target must specify exactly one of food or position"
                )

            if "food" in assignment:
                food = assignment["food"]
                if not isinstance(food, str) or food not in food_positions:
                    raise ValueError(f"Arm {number} references unknown food {food!r}")
                # Resolve to the initial position, not a live object reference:
                # moving or capturing food later does not retarget other arms.
                target = food_positions[food]
            else:
                target = _point(assignment["position"], f"Arm {number}.target.position")

        # File/UI arm numbers are one-based; the controller tuple is zero-based.
        arms[number - 1] = ArmSetup(parameters, target)

    # Publish only after every entry is valid. Tuples and frozen records detach
    # restart conditions from both the decoded JSON and the mutable running scene.
    return Scenario(name, body, tuple(objects), tuple(arms))


def _unique_keys(pairs: list[tuple[str, object]]) -> dict:
    """Reject duplicate JSON keys instead of silently overwriting a setting."""
    # The JSON decoder supplies pairs before building a dict, while duplicate
    # keys are still observable. This hook runs for nested objects too.
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate JSON field {key!r}")
        result[key] = value
    return result


def load_scenario(path: str | Path) -> Scenario:
    """Read a UTF-8 scenario file and report file or validation errors with its path.

    :param path: JSON file to load.
    :return: Validated starting conditions.
    :raises ValueError: If the file cannot be read or does not describe a scenario.
    """
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"), object_pairs_hook=_unique_keys)
        return parse_scenario(value)

    # Present file access, JSON syntax and scenario validation failures through
    # one public error type, retaining the source path and original exception.
    except (OSError, ValueError) as error:
        raise ValueError(f"{path}: {error}") from error


def save_scenario(path: str | Path, central: CentralController, environment: Environment) -> None:
    """Export supported starting conditions to a new file, never overwriting one.

    Idle arms omit targets. A carrying arm exports its payload destination, not
    the solver's temporary tip correction. Poses, grips and history are outside
    version 1; food is exported at its current location as an unheld object.
    """
    from math import degrees

    path = Path(path)
    data = {
        "version": 1,
        "name": path.stem,
        "body": list(central.centre),
        "food": [],
        "obstacles": [],
        "arms": [],
    }
    for obj in environment.objects:
        entry = {"position": list(obj.centre), "radius": obj.radius}
        if obj.kind is ObjectKind.FOOD:
            entry["id"] = f"food-{obj.identifier}"
        data["food" if obj.kind is ObjectKind.FOOD else "obstacles"].append(entry)
    for number, arm in enumerate(central.arms, 1):
        parameters = arm.controller.arm.parameters
        entry = {
            "number": number,
            "parameters": {
                "segment_count": parameters.segment_count,
                "segment_length": parameters.segment_length,
                "maximum_bend_degrees": degrees(parameters.maximum_bend),
                "turning_speed_degrees": degrees(parameters.turning_speed),
            },
        }
        if arm.active:
            target = arm.carry_goal if arm.grip is not None else arm.controller.target
            if target is not None:
                entry["target"] = {"position": list(target)}
        data["arms"].append(entry)
    # Validate before creating a file, using the same strict contract as loading.
    parse_scenario(data)
    content = json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    with path.open("x", encoding="utf-8") as output:
        output.write(content)
