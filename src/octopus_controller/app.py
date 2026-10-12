"""Application input and fixed-step timing for the eight-arm experiment."""

import argparse
from dataclasses import replace
from math import pi
from pathlib import Path

import pygame

from octopus_controller.activity import ActivityHistory
from octopus_controller.dialogs import ScenarioNamePrompt, ScenarioPicker
from octopus_controller.model import ReachController
from octopus_controller.organism import ARM_COUNT, CentralController
from octopus_controller.scenario import DEFAULT_BODY, Scenario, load_scenario, save_scenario
from octopus_controller.sensing import Environment, ObjectKind
from octopus_controller.view import (
    CLEAR_SCENARIO_BUTTON,
    HEADER_ICON_SIZE,
    HISTORY_ARM,
    HISTORY_BUTTON,
    HISTORY_CATEGORY,
    HISTORY_CLOSE,
    HISTORY_LATEST,
    HISTORY_ROWS,
    LOAD_SCENARIO_BUTTON,
    MESSAGE_PANEL,
    SAVE_SCENARIO_BUTTON,
    SCENARIO_NAME_BUTTON,
    WINDOW_SIZE,
    WORLD,
    SceneCamera,
    draw_history,
    draw_scene,
    history_lines,
)

SIMULATION_STEP = 1 / 120
MAXIMUM_FRAME_TIME = 0.1
BODY_CENTRE = DEFAULT_BODY


class Application:
    """Own UI state and translate user input into simulation commands."""

    def __init__(self, scenario: Scenario | None = None) -> None:
        """Create an empty setup or a paused scenario without opening a display."""
        self.scenario = scenario
        self.history_open = False
        self.history_arm = None
        self.history_category = "All"
        self.history_was_paused = True
        self.message_scroll: int | None = None
        self.camera = SceneCamera()
        self.name_override: str | None = None
        self.name_prompt: ScenarioNamePrompt | None = None
        self.save_notice: str | None = None
        self.load_error: str | None = None
        self.scenario_directory = Path.cwd() / "scenarios"
        if not self.scenario_directory.is_dir():
            self.scenario_directory = Path.cwd()
        self.picker: ScenarioPicker | None = None
        self.central = CentralController(BODY_CENTRE)
        self.selected_index = 0
        self.playback_speed = 1
        self.paused = False
        self.dragging = False
        self.accumulator = 0.0
        self.environment = Environment()
        self.placement: ObjectKind | None = None
        self.selected_object: int | None = None
        self.dragging_object = False
        self.central.refresh_sensing(self.environment.objects)
        self.restart()

    @property
    def scenario_name(self) -> str:
        """Give the current setup a label even when no scenario is loaded."""
        return self.name_override or (
            self.scenario.name if self.scenario is not None else "Empty scenario"
        )

    def open_scenario(self, *, saving: bool = False) -> None:
        """Open a modal picker without blocking the Pygame event loop."""
        self.dragging = self.dragging_object = False
        self.load_error = None
        self.save_notice = None
        self.picker = ScenarioPicker(self.scenario_directory, saving=saving)

    def load_selected_scenario(self, path: Path) -> None:
        """Validate and build before replacing the current scene or restart setup."""
        try:
            scenario = load_scenario(path)
            central, environment = scenario.build()
        except ValueError as error:
            self.load_error = str(error)
            return
        self.scenario = scenario
        self.name_override = None
        self.central, self.environment = central, environment
        self.camera = SceneCamera.fit(self.central)
        self.message_scroll = None
        self.history_open = False
        self.activity = ActivityHistory(self.central)
        self.scenario_directory = path.parent
        self.paused = True
        self.selected_index = 0
        self.selected_object = None
        self.placement = None
        self.accumulator = 0.0

    def _history_event(self, event):
        """Consume inspection input without leaking mouse or keyboard edits to the scene.

        :param event: Pygame input received while the history overlay is open.
        :return: None; closing restores the playback state from before inspection.
        """
        close = event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE
        rows = 0
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                if HISTORY_CLOSE.collidepoint(event.pos):
                    close = True
                elif HISTORY_ARM.collidepoint(event.pos):
                    self.history_arm = (
                        0
                        if self.history_arm is None
                        else (self.history_arm + 1 if self.history_arm < 7 else None)
                    )
                    self.message_scroll = None
                elif HISTORY_CATEGORY.collidepoint(event.pos):
                    choices = ("All", "Food", "Safety", "Blocked", "Conflict")
                    self.history_category = choices[
                        (choices.index(self.history_category) + 1) % len(choices)
                    ]
                    self.message_scroll = None
                elif HISTORY_LATEST.collidepoint(event.pos):
                    self.message_scroll = None
            elif event.button in (4, 5):
                rows = -3 if event.button == 4 else 3
        elif event.type == pygame.MOUSEWHEEL:
            rows = event.y * (3 if getattr(event, "flipped", False) else -3)
        elif event.type == pygame.KEYDOWN:
            rows = {
                pygame.K_UP: -1,
                pygame.K_DOWN: 1,
                pygame.K_PAGEUP: -HISTORY_ROWS,
                pygame.K_PAGEDOWN: HISTORY_ROWS,
            }.get(event.key, 0)
            if event.key == pygame.K_END:
                self.message_scroll = None
        if rows:
            pygame.font.init()
            lines = history_lines(
                self.activity, self.history_arm, self.history_category, pygame.font.Font(None, 18)
            )
            maximum = max(0, len(lines) - HISTORY_ROWS)
            current = maximum if self.message_scroll is None else self.message_scroll
            self.message_scroll = max(0, min(maximum, current + rows))
        if close:
            self.history_open = False
            self.paused = self.history_was_paused
            self.accumulator = 0.0

    def restart(self) -> None:
        """Restore initial conditions; loaded scenarios always restart paused."""
        if self.scenario is None:
            name = self.name_override
            self.clear()
            self.name_override = name
            return
        else:
            self.central, self.environment = self.scenario.build()
            self.camera = SceneCamera.fit(self.central)
            self.paused = True
        self.selected_object = None
        self.placement = None
        self.dragging_object = False
        self.selected_index = 0
        self.dragging = False
        self.accumulator = 0.0
        self.message_scroll = None
        self.history_open = False
        self.activity = ActivityHistory(self.central)
        self.central.refresh_sensing(self.environment.objects)

    def clear(self) -> None:
        """Return to the empty startup setup with default, idle arms and no history."""
        self.scenario = None
        self.message_scroll = None
        self.history_open = False
        self.name_override = None
        self.name_prompt = None
        self.central = CentralController(BODY_CENTRE)
        # Empty startup, Clear and Restart share the same framing as loaded scenes.
        self.camera = SceneCamera.fit(self.central)
        for arm in self.central.arms:
            arm.idle()
        self.environment = Environment()
        self.activity = ActivityHistory(self.central)
        self.paused = True
        self.selected_index = 0
        self.selected_object = None
        self.placement = None
        self.dragging = self.dragging_object = False
        self.accumulator = 0.0
        self.picker = None
        self.load_error = self.save_notice = None

    @property
    def controller(self) -> ReachController:
        """Return the selected local controller for measurements and parameter editing."""
        return self.central.arm(self.selected_index).controller

    def handle_event(self, event: pygame.event.Event) -> bool:
        """Apply one input event and report whether the application should continue.

        :param event: Pygame window, keyboard or mouse event.
        :return: False when the user requests exit; otherwise True.
        """
        if event.type == pygame.QUIT:
            return False
        if self.history_open:
            self._history_event(event)
            return True
        if self.name_prompt is not None:
            self.name_prompt.handle_event(event)
            if self.name_prompt.cancelled:
                self.name_prompt = None
            elif self.name_prompt.result is not None:
                self.name_override = self.name_prompt.result
                self.name_prompt = None
            return True
        if self.picker is not None:
            self.picker.handle_event(event)
            if self.picker.cancelled:
                self.picker = None
            elif self.picker.result is not None:
                path = self.picker.result
                if self.picker.saving:
                    try:
                        save_scenario(path, self.central, self.environment, name=self.scenario_name)
                    except (OSError, ValueError) as error:
                        self.picker.error = str(error)
                        self.picker.result = None
                    else:
                        self.scenario_directory = path.parent
                        self.save_notice = f"Saved {path.name}"
                        self.picker = None
                else:
                    self.picker = None
                    self.load_selected_scenario(path)
            return True
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_o and getattr(event, "mod", 0) & (
                pygame.KMOD_CTRL | pygame.KMOD_GUI
            ):
                self.open_scenario()
            elif event.key == pygame.K_s and getattr(event, "mod", 0) & (
                pygame.KMOD_CTRL | pygame.KMOD_GUI
            ):
                self.open_scenario(saving=True)
            elif event.key == pygame.K_ESCAPE and self.load_error is not None:
                self.load_error = None
            elif event.key == pygame.K_ESCAPE:
                return False
            elif pygame.K_1 <= event.key <= pygame.K_8 or event.key == pygame.K_TAB:
                # End any drag before changing selection so one gesture cannot
                # accidentally move a second arm's target halfway through.
                self.dragging = False
                self.selected_index = (
                    (self.selected_index + 1) % ARM_COUNT
                    if event.key == pygame.K_TAB
                    else event.key - pygame.K_1
                )
            elif event.key in (pygame.K_f, pygame.K_o, pygame.K_h):
                # Placement is a one-click tool, avoiding accidental target changes
                # when the user intends to add something for an arm to feel.
                kind = {
                    pygame.K_f: ObjectKind.FOOD,
                    pygame.K_o: ObjectKind.OBSTACLE,
                    pygame.K_h: ObjectKind.HAZARD,
                }[event.key]
                self.placement = None if self.placement is kind else kind
                self.dragging = False
                self.dragging_object = False
            elif event.key in (pygame.K_DELETE, pygame.K_BACKSPACE):
                if self.selected_object is not None:
                    self.central.release_object(self.selected_object)
                    self.environment.remove(self.selected_object)
                    self.selected_object = None
                    self.dragging_object = False
            elif event.key == pygame.K_g:
                self.dragging = False
                self.central.arm(self.selected_index).release()
            elif event.key == pygame.K_t:
                self.dragging = False
                self.central.arm(self.selected_index).retract()
            elif event.key == pygame.K_x:
                self.dragging = False
                self.central.assign_idle(self.selected_index)
            elif event.key == pygame.K_SPACE:
                self.paused = not self.paused
                self.accumulator = 0.0
            elif event.key == pygame.K_v:
                speeds = (1, 2, 4, 8)
                self.playback_speed = speeds[(speeds.index(self.playback_speed) + 1) % len(speeds)]
            elif event.key == pygame.K_n and self.paused:
                self.central.step(SIMULATION_STEP, self.environment)
                self.activity.observe(self.central)
            elif event.key == pygame.K_d:
                self.restart()
            else:
                self._adjust_parameters(event.key)
        elif (
            event.type == pygame.MOUSEBUTTONDOWN
            and event.button == 1
            and LOAD_SCENARIO_BUTTON.collidepoint(event.pos)
        ):
            self.open_scenario()
        elif (
            event.type == pygame.MOUSEBUTTONDOWN
            and event.button == 1
            and SAVE_SCENARIO_BUTTON.collidepoint(event.pos)
        ):
            self.open_scenario(saving=True)
        elif (
            event.type == pygame.MOUSEBUTTONDOWN
            and event.button == 1
            and CLEAR_SCENARIO_BUTTON.collidepoint(event.pos)
        ):
            self.clear()
        elif (
            event.type == pygame.MOUSEBUTTONDOWN
            and event.button == 1
            and SCENARIO_NAME_BUTTON.collidepoint(event.pos)
        ):
            self.dragging = self.dragging_object = False
            self.name_prompt = ScenarioNamePrompt(self.scenario_name)
        elif event.type == pygame.MOUSEBUTTONDOWN and MESSAGE_PANEL.collidepoint(event.pos):
            if event.button == 1 and HISTORY_BUTTON.collidepoint(event.pos):
                self.history_was_paused = self.paused
                self.paused = True
                self.history_open = True
                self.dragging = self.dragging_object = False
                self.accumulator = 0.0
                self.message_scroll = None
        elif event.type == pygame.MOUSEWHEEL:
            pass  # The stable summary cannot be scrolled out of view.
        elif event.type == pygame.MOUSEBUTTONDOWN and WORLD.collidepoint(event.pos):
            point = self.camera.unproject(event.pos)
            if event.button == 1:
                self.dragging_object = False
                if self.placement is not None:
                    self.selected_object = self.environment.add(point, self.placement)
                    self.placement = None
                else:
                    self.dragging = True
                    self.central.assign_reach(self.selected_index, point)
            elif event.button == 3:
                self.selected_object = self.environment.pick(point)
                self.dragging_object = self.selected_object is not None
                self.dragging = False
        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                self.dragging = False
            elif event.button == 3:
                self.dragging_object = False
        elif event.type == pygame.WINDOWFOCUSLOST:
            self.dragging = False
            self.dragging_object = False
        elif event.type == pygame.MOUSEMOTION:
            # Keep target and object centres visible when a drag leaves the world.
            x = max(WORLD.left, min(WORLD.right - 1, event.pos[0]))
            y = max(WORLD.top, min(WORLD.bottom - 1, event.pos[1]))
            point = self.camera.unproject((x, y))
            if self.dragging_object and self.selected_object is not None:
                self.central.release_object(self.selected_object)
                self.environment.move(self.selected_object, point)
            elif self.dragging:
                self.central.assign_reach(self.selected_index, point)

        # Scene and geometry edits should be inspectable even while paused. This
        # refresh reads sensors only; it never advances the reaching controller.
        self.central.refresh_sensing(self.environment.objects)
        self.activity.observe(self.central)
        return True

    def _adjust_parameters(self, key: int) -> None:
        """Apply bounded keyboard adjustments, rebuilding geometry when necessary.

        :param key: Pygame key constant from a key-down event.
        """
        parameters = self.controller.arm.parameters
        adjustments = {
            pygame.K_LEFTBRACKET: ("segment_count", -1, 2, 40),
            pygame.K_RIGHTBRACKET: ("segment_count", 1, 2, 40),
            pygame.K_MINUS: ("segment_length", -2, 6, 30),
            pygame.K_EQUALS: ("segment_length", 2, 6, 30),
            pygame.K_COMMA: ("maximum_bend", -pi / 36, pi / 36, pi),
            pygame.K_PERIOD: ("maximum_bend", pi / 36, pi / 36, pi),
            pygame.K_SEMICOLON: ("turning_speed", -0.25, 0.25, 4.0),
            pygame.K_QUOTE: ("turning_speed", 0.25, 0.25, 4.0),
        }
        if key in adjustments:
            name, change, lower, upper = adjustments[key]
            value = max(lower, min(upper, getattr(parameters, name) + change))
            parameters = replace(parameters, **{name: value})
            if name == "turning_speed":
                self.controller.arm.parameters = parameters
                return
        elif key != pygame.K_r:
            return
        self.central.arm(self.selected_index).reset_pose(parameters)
        self.accumulator = 0.0

    def advance(self, elapsed_seconds: float) -> None:
        """Advance fixed simulation ticks, discarding excessive time after a stall.

        :param elapsed_seconds: Real frame duration in seconds from the display clock.
        """
        if (
            self.paused
            or self.history_open
            or self.picker is not None
            or self.name_prompt is not None
        ):
            return

        # Fixed ticks make the solver independent of rendering frequency. Limiting
        # catch-up avoids a sudden jump after moving the window or a debugger pause.
        # Scale wall time, never the solver tick: hazard dwell, arbitration and
        # collision checks therefore follow exactly the same sequence at any speed.
        self.accumulator += min(elapsed_seconds, MAXIMUM_FRAME_TIME) * self.playback_speed
        while self.accumulator >= SIMULATION_STEP:
            self.central.step(SIMULATION_STEP, self.environment)
            self.activity.observe(self.central)
            self.accumulator -= SIMULATION_STEP


def main(argv: list[str] | None = None) -> None:
    """Run an optional scenario; invalid files fail before opening a window.

    :param argv: Command-line arguments, or None to read the process arguments.
    """
    parser = argparse.ArgumentParser(description="Distributed Octopus Controller")
    parser.add_argument(
        "scenario", nargs="?", type=Path, help="JSON scenario to load paused; press Space to play"
    )
    args = parser.parse_args(argv)
    scenario = None
    if args.scenario is not None:
        try:
            scenario = load_scenario(args.scenario)
        except ValueError as error:
            parser.error(str(error))
    application = Application(scenario)
    if args.scenario is not None:
        application.scenario_directory = args.scenario.resolve().parent
    pygame.init()
    try:
        pygame.display.set_caption(f"Distributed Octopus Controller — {application.scenario_name}")
        icon_path = Path(__file__).parent / "assets" / "octopus.png"
        icon = pygame.image.load(icon_path)
        pygame.display.set_icon(icon)
        screen = pygame.display.set_mode(WINDOW_SIZE)

        # Scale once at startup, preserving proportions rather than resizing every frame.
        header_icon = pygame.transform.smoothscale(
            icon.convert_alpha(), icon.get_rect().fit((0, 0, *HEADER_ICON_SIZE)).size
        )
        font = pygame.font.Font(None, 22)
        heading_font = pygame.font.Font(None, 38)
        shortcuts_font = pygame.font.Font(None, 18)
        clock = pygame.time.Clock()
        running = True
        while running:
            elapsed_seconds = clock.tick(60) / 1000
            for event in pygame.event.get():
                if not application.handle_event(event):
                    running = False
            if running:
                pygame.display.set_caption(
                    f"Distributed Octopus Controller — {application.scenario_name}"
                )
                application.advance(elapsed_seconds)
                draw_scene(
                    screen,
                    application.central,
                    application.paused,
                    font,
                    heading_font,
                    header_icon,
                    application.selected_index,
                    application.environment.objects,
                    application.selected_object,
                    application.placement,
                    shortcuts_font,
                    application.scenario_name,
                    application.load_error,
                    application.message_scroll,
                    application.camera,
                    application.playback_speed,
                    application.activity,
                )
                if application.save_notice:
                    screen.set_clip(WORLD)
                    notice = font.render(application.save_notice, True, (79, 212, 184))
                    screen.blit(notice, (WORLD.left + 16, WORLD.bottom - 28))
                    screen.set_clip(None)
                if application.picker is not None:
                    application.picker.draw(screen, font)
                if application.name_prompt is not None:
                    application.name_prompt.draw(screen, font)
                if application.history_open:
                    draw_history(
                        screen,
                        application.central,
                        application.activity,
                        shortcuts_font,
                        application.history_arm,
                        application.history_category,
                        application.message_scroll,
                    )
                pygame.display.flip()
    finally:
        # Release SDL resources even if an input or drawing error is raised.
        pygame.quit()
