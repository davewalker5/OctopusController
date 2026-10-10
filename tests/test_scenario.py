"""Scenario validation and repeatable playback through the application's controls."""

import json
from math import pi, radians
from pathlib import Path

import pytest

from octopus_controller.organism import DEFAULT_PARAMETERS
from octopus_controller.scenario import load_scenario, parse_scenario

EXAMPLE = Path(__file__).resolve().parents[1] / "scenarios" / "two-targets.json"


def test_example_resolves_parameters_objects_and_independent_targets() -> None:
    """The documented example maps one-based arms and converts degrees once."""
    scenario = load_scenario(EXAMPLE)
    central, environment = scenario.build()
    assert central.centre == (400, 460)
    assert [arm.active for arm in central.arms] == [
        True,
        False,
        True,
        False,
        True,
        False,
        False,
        False,
    ]
    assert central.arm(0).controller.target == (400, 240)
    assert central.arm(2).controller.target == (620, 460)
    assert central.arm(4).controller.target == (400, 650)
    assert central.arm(2).controller.arm.parameters.segment_count == 24
    assert central.arm(1).controller.arm.parameters.segment_count == 20
    assert central.arm(2).controller.arm.parameters.maximum_bend == pytest.approx(pi / 3)
    assert central.arm(2).controller.arm.parameters.turning_speed == pytest.approx(radians(14))
    assert [obj.radius for obj in environment.objects] == [12, 12, 25]
    environment.move(1, (20, 20))
    assert central.arm(0).controller.target == (400, 240)
    assert scenario.build()[1].objects[0].centre == (400, 240)


def test_minimal_scenario_and_parameters_without_target() -> None:
    """Omitted collections are empty and even configured arms idle without a target."""
    scenario = parse_scenario(
        {"version": 1, "arms": [{"number": 8, "parameters": {"segment_length": 15}}]}
    )
    central, environment = scenario.build()
    assert not environment.objects
    assert not any(arm.active for arm in central.arms)
    assert central.arm(0).controller.arm.parameters == DEFAULT_PARAMETERS
    assert central.arm(7).controller.arm.parameters.segment_length == 15


@pytest.mark.parametrize(
    ("patch", "message"),
    [
        ({"version": 2}, "version"),
        ({"version": True}, "version"),
        ({"name": None}, "name"),
        ({"typo": 1}, "unknown field"),
        ({"body": [1]}, "two coordinates"),
        ({"body": [True, 1]}, "finite number"),
        ({"body": [float("inf"), 1]}, "finite number"),
        ({"body": [10**400, 1]}, "finite number"),
        ({"food": {}}, "array"),
        ({"food": [{"position": [1, 2], "radius": 0}]}, "radius"),
        ({"food": [{"position": [1, 2], "id": "a"}] * 2}, "duplicate food"),
        ({"defaults": {"segment_count": 2.5}}, "integer"),
        ({"defaults": {"segment_count": 101}}, "integer"),
        ({"defaults": {"segment_count": True}}, "integer"),
        ({"defaults": {"maximum_bend_degrees": 181}}, "at most 180"),
        ({"defaults": {"turning_speed_degrees": -1}}, "positive"),
        ({"defaults": {"segment_length": "12"}}, "finite number"),
        ({"defaults": {"segment_lenght": 12}}, "unknown field"),
        ({"arms": [{"number": 0}]}, "between 1 and 8"),
        ({"arms": [{"number": True}]}, "between 1 and 8"),
        ({"arms": [{"number": 1}] * 2}, "Duplicate arm"),
        ({"arms": [{"number": 3, "target": {"food": "missing"}}]}, "Arm 3.*unknown food"),
        ({"arms": [{"number": 3, "target": {"food": []}}]}, "unknown food"),
        ({"arms": [{"number": 1, "target": {}}]}, "exactly one"),
        ({"arms": [{"number": 1, "target": {"food": "a", "position": [1, 2]}}]}, "exactly one"),
    ],
)
def test_reject_invalid_scenarios(patch: dict, message: str) -> None:
    """Invalid types, ambiguous assignments and misspelled settings fail clearly."""
    with pytest.raises(ValueError, match=message):
        parse_scenario({"version": 1, **patch})


@pytest.mark.parametrize(
    "contents", ["{", '{"version":1,"version":1}', "[]", '{"version":1,"body":[NaN,0]}']
)
def test_file_errors_include_path(tmp_path: Path, contents: str) -> None:
    """Malformed files and duplicate keys report the source file."""
    path = tmp_path / "bad.json"
    path.write_text(contents)
    with pytest.raises(ValueError, match="bad.json"):
        load_scenario(path)


def test_missing_file(tmp_path: Path) -> None:
    """Unreadable input produces the same public validation error type."""
    with pytest.raises(ValueError, match="missing.json"):
        load_scenario(tmp_path / "missing.json")


def test_parser_does_not_retain_mutable_input() -> None:
    """Editing the original document cannot change stored restart conditions."""
    data = json.loads(EXAMPLE.read_text())
    scenario = parse_scenario(data)
    data["body"][0] = 10
    data["food"][0]["position"][0] = 10
    assert scenario.body == (400, 460)
    assert scenario.arms[0].target == (400, 240)
