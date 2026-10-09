"""Exercise UI input and drawing with SDL's in-memory display driver."""

import os
from pathlib import Path

# Set SDL drivers before importing Pygame so these tests never open a real window.
os.environ["SDL_VIDEODRIVER"] = "dummy"
os.environ["SDL_AUDIODRIVER"] = "dummy"

import pygame
import pytest

from octopus_controller.app import SIMULATION_STEP, Application
from octopus_controller.view import WINDOW_SIZE, WORLD, draw_scene


def press(application: Application, key: int) -> bool:
    """Send one key-down event and return the application's continuation flag.

    :param application: Experiment receiving the event.
    :param key: Pygame keyboard constant.
    :return: Whether the application should remain running.
    """
    return application.handle_event(pygame.event.Event(pygame.KEYDOWN, key=key))


def test_pause_step_resume_and_reset() -> None:
    """Pause freezes motion, stepping advances once and reset restores the pose."""
    application = Application()
    original = application.controller.arm.points
    press(application, pygame.K_SPACE)
    application.advance(0.1)
    assert application.controller.arm.points == original
    press(application, pygame.K_n)
    assert application.controller.arm.points != original
    press(application, pygame.K_r)
    assert application.controller.arm.points == original
    press(application, pygame.K_SPACE)
    application.advance(SIMULATION_STEP)
    assert application.controller.arm.points != original
    assert not press(application, pygame.K_ESCAPE)


def test_drag_target_and_release() -> None:
    """Dragging clamps to the world and release stops further target movement."""
    application = Application()
    application.handle_event(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=(500, 200)))
    assert application.controller.target == (500, 200)
    application.handle_event(pygame.event.Event(pygame.MOUSEMOTION, pos=(-100, 900)))
    assert application.controller.target == (WORLD.left, WORLD.bottom - 1)
    application.handle_event(pygame.event.Event(pygame.MOUSEBUTTONUP, button=1))
    application.handle_event(pygame.event.Event(pygame.MOUSEMOTION, pos=(100, 100)))
    assert application.controller.target == (WORLD.left, WORLD.bottom - 1)


def test_parameter_changes_and_defaults() -> None:
    """Geometry resets preserve the target; speed changes preserve the pose."""
    application = Application()
    application.controller.set_target((420, 200))
    press(application, pygame.K_RIGHTBRACKET)
    assert application.controller.arm.parameters.segment_count == 21
    assert application.controller.target == (420, 200)
    application.advance(0.1)
    before = application.controller.arm.points
    press(application, pygame.K_QUOTE)
    assert application.controller.arm.parameters.turning_speed == 1.75
    assert application.controller.arm.points == before
    for _ in range(100):
        press(application, pygame.K_LEFTBRACKET)
        press(application, pygame.K_COMMA)
    assert application.controller.arm.parameters.segment_count == 2
    assert application.controller.arm.parameters.maximum_bend > 0
    press(application, pygame.K_d)
    assert application.controller.arm.parameters.segment_count == 20


def test_fixed_steps_do_not_depend_on_render_frequency() -> None:
    """Different frame groupings produce the same one-second simulation."""
    slow, fast = Application(), Application()
    for _ in range(30):
        slow.advance(1 / 30)
    for _ in range(120):
        fast.advance(1 / 120)
    assert slow.controller.arm.angles == pytest.approx(fast.controller.arm.angles)


def test_renderer_and_packaged_icon() -> None:
    """Render a complete frame and load the icon without requiring a desktop."""
    pygame.init()
    try:
        screen = pygame.display.set_mode(WINDOW_SIZE)
        font = pygame.font.Font(None, 22)
        application = Application()
        from octopus_controller import app

        icon = pygame.image.load(Path(app.__file__).parent / "assets" / "octopus.png")
        header_icon = pygame.transform.smoothscale(icon, (48, 48))
        draw_scene(
            screen, application.central, False, font, pygame.font.Font(None, 38), header_icon
        )
        assert icon.get_width() > 0
        assert screen.get_at((0, 0))[:3] == (16, 26, 39)
    finally:
        pygame.quit()


def test_default_experiment_captures_food() -> None:
    """The opening food is captured once enough adjacent sensors establish a grip."""
    application = Application()
    for _ in range(600):
        application.advance(1 / 60)
    assert application.central.arm(0).grip is not None
    assert application.central.capture_count == 1


def test_main_starts_renders_and_quits(monkeypatch: pytest.MonkeyPatch) -> None:
    """Exercise the actual entry point, including icon loading and SDL cleanup.

    :param monkeypatch: Pytest fixture replacing the external event source.
    """
    from octopus_controller.app import main

    batches = iter([[], [pygame.event.Event(pygame.QUIT)]])

    def next_events() -> list[pygame.event.Event]:
        """Return one idle frame followed by a deterministic close request."""
        return next(batches)

    monkeypatch.setattr(pygame.event, "get", next_events)
    main()
    assert not pygame.get_init()


def test_selection_assignments_and_drag_do_not_leak_between_arms() -> None:
    """Switching selection ends the previous drag and assignments stay arm-specific."""
    application = Application()
    original = application.central.arm(0).controller.target
    press(application, pygame.K_3)
    application.handle_event(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=(500, 200)))
    assert application.central.arm(2).controller.target == (500, 200)
    assert application.central.arm(0).controller.target == original
    press(application, pygame.K_TAB)
    target = application.controller.target
    application.handle_event(pygame.event.Event(pygame.MOUSEMOTION, pos=(600, 200)))
    assert application.controller.target == target
    press(application, pygame.K_x)
    assert not application.central.arm(3).active
    application.handle_event(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=(450, 300)))
    assert application.central.arm(3).active
    assert application.controller.target == (450, 300)


def test_paused_step_updates_all_and_selected_parameters_only() -> None:
    """Pause and step are global; changing geometry is local to the selected arm."""
    application = Application()
    press(application, pygame.K_SPACE)
    before = [arm.controller.arm.points for arm in application.central.arms]
    application.advance(0.1)
    assert [arm.controller.arm.points for arm in application.central.arms] == before
    press(application, pygame.K_n)
    assert all(
        arm.controller.arm.points != pose for arm, pose in zip(application.central.arms, before)
    )
    press(application, pygame.K_8)
    press(application, pygame.K_RIGHTBRACKET)
    assert [arm.controller.arm.parameters.segment_count for arm in application.central.arms] == [
        20
    ] * 7 + [21]
    press(application, pygame.K_d)
    assert application.selected_index == 0
    assert all(
        arm.controller.arm.parameters.segment_count == 20 for arm in application.central.arms
    )


def test_place_move_and_delete_objects_while_paused() -> None:
    """Scene editing refreshes local sensors without moving arms or their targets."""
    from octopus_controller.sensing import ObjectKind

    application = Application()
    press(application, pygame.K_SPACE)
    before = application.controller.arm.points
    target = application.controller.target
    position = tuple(round(value) for value in before[10])
    press(application, pygame.K_f)
    application.handle_event(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=position))
    assert application.controller.target == target
    assert application.central.arm(0).contacts
    assert application.environment.objects[-1].kind is ObjectKind.FOOD
    application.handle_event(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=3, pos=position))
    application.handle_event(pygame.event.Event(pygame.MOUSEMOTION, pos=(40, 110)))
    application.handle_event(pygame.event.Event(pygame.MOUSEBUTTONUP, button=3))
    assert application.environment.objects[-1].centre == (40, 110)
    assert not application.central.arm(0).contacts
    assert application.controller.arm.points == before
    press(application, pygame.K_DELETE)
    assert len(application.environment.objects) == 2
    press(application, pygame.K_d)
    assert application.selected_object is None
    assert application.placement is None


def test_placement_tool_can_be_cancelled() -> None:
    """Repeating the placement key returns a click to normal target assignment."""
    application = Application()
    press(application, pygame.K_o)
    assert application.placement is not None
    press(application, pygame.K_o)
    assert application.placement is None
    application.handle_event(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=(600, 500)))
    assert application.controller.target == (600, 500)
    assert len(application.environment.objects) == 2


def test_default_obstacle_demonstrates_avoidance_and_reaches() -> None:
    """The opening obstacle scene produces a visible detour without user setup."""
    application = Application()
    saw_avoidance = False
    for _ in range(600):
        application.advance(1 / 60)
        saw_avoidance |= application.central.arm(2).controller.avoiding
    assert saw_avoidance
    assert application.central.arm(2).controller.status == "Reached"
    assert all(
        arm.controller.distance <= 2 or arm.grip is not None for arm in application.central.arms
    )


def test_grasp_controls_release_and_paused_delete() -> None:
    """Keyboard carry controls and scene editing preserve consistent attachment state."""
    application = Application()
    for _ in range(120):
        application.advance(1 / 60)
    arm = application.central.arm(0)
    assert arm.grip is not None
    press(application, pygame.K_t)
    assert arm.retracting
    press(application, pygame.K_x)
    assert arm.grip is not None and not arm.active
    press(application, pygame.K_g)
    assert arm.grip is None
    original = application.environment.objects[0].centre
    application.central.assign_reach(0, original)
    for _ in range(120):
        application.advance(1 / 60)
    assert arm.grip is not None
    press(application, pygame.K_SPACE)
    application.selected_object = arm.grip.object_id
    press(application, pygame.K_DELETE)
    assert arm.grip is None
    assert all(obj.identifier != 1 for obj in application.environment.objects)
