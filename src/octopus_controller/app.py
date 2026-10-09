"""Application input and fixed-step timing for the eight-arm experiment."""

from dataclasses import replace
from math import pi
from pathlib import Path

import pygame

from octopus_controller.model import ReachController
from octopus_controller.organism import ARM_COUNT, CentralController
from octopus_controller.sensing import Environment, ObjectKind
from octopus_controller.view import HEADER_ICON_SIZE, WINDOW_SIZE, WORLD, draw_scene

SIMULATION_STEP = 1 / 120
MAXIMUM_FRAME_TIME = 0.1
BODY_CENTRE = (400.0, 460.0)


class Application:
    """Own UI state and translate user input into simulation commands."""

    def __init__(self) -> None:
        """Create the initial experiment without opening a display."""
        self.central = CentralController(BODY_CENTRE)
        self.selected_index = 0
        self.paused = False
        self.dragging = False
        self.accumulator = 0.0
        self.environment = self._initial_environment()
        self.placement: ObjectKind | None = None
        self.selected_object: int | None = None
        self.dragging_object = False
        self.central.refresh_sensing(self.environment.objects)

    def _initial_environment(self) -> Environment:
        """Place food and a blocking circle so the default run demonstrates a detour."""
        environment = Environment()
        environment.add(self.central.arm(0).controller.target, ObjectKind.FOOD)
        # Arm 3 starts to the right of the body. Put an obstacle across its
        # curved approach, with a reachable target beyond it rather than inside it.
        base = self.central.arm(2).controller.arm.base
        self.central.assign_reach(2, (base[0] + 100, base[1]))
        environment.add((base[0] + 60, base[1] + 67), ObjectKind.OBSTACLE)
        return environment

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
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                return False
            if pygame.K_1 <= event.key <= pygame.K_8 or event.key == pygame.K_TAB:
                # End any drag before changing selection so one gesture cannot
                # accidentally move a second arm's target halfway through.
                self.dragging = False
                self.selected_index = (
                    (self.selected_index + 1) % ARM_COUNT
                    if event.key == pygame.K_TAB
                    else event.key - pygame.K_1
                )
            elif event.key in (pygame.K_f, pygame.K_o):
                # Placement is a one-click tool, avoiding accidental target changes
                # when the user intends to add something for an arm to feel.
                kind = ObjectKind.FOOD if event.key == pygame.K_f else ObjectKind.OBSTACLE
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
            elif event.key == pygame.K_n and self.paused:
                self.central.step(SIMULATION_STEP, self.environment)
            elif event.key == pygame.K_d:
                self.central = CentralController(BODY_CENTRE)
                self.environment = self._initial_environment()
                self.selected_object = None
                self.placement = None
                self.dragging_object = False
                self.selected_index = 0
                self.dragging = False
                self.accumulator = 0.0
            else:
                self._adjust_parameters(event.key)
        elif event.type == pygame.MOUSEBUTTONDOWN and WORLD.collidepoint(event.pos):
            if event.button == 1:
                self.dragging_object = False
                if self.placement is not None:
                    self.selected_object = self.environment.add(event.pos, self.placement)
                    self.placement = None
                else:
                    self.dragging = True
                    self.central.assign_reach(self.selected_index, event.pos)
            elif event.button == 3:
                self.selected_object = self.environment.pick(event.pos)
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
            if self.dragging_object and self.selected_object is not None:
                self.central.release_object(self.selected_object)
                self.environment.move(self.selected_object, (x, y))
            elif self.dragging:
                self.central.assign_reach(self.selected_index, (x, y))

        # Scene and geometry edits should be inspectable even while paused. This
        # refresh reads sensors only; it never advances the reaching controller.
        self.central.refresh_sensing(self.environment.objects)
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
        if self.paused:
            return

        # Fixed ticks make the solver independent of rendering frequency. Limiting
        # catch-up avoids a sudden jump after moving the window or a debugger pause.
        self.accumulator += min(elapsed_seconds, MAXIMUM_FRAME_TIME)
        while self.accumulator >= SIMULATION_STEP:
            self.central.step(SIMULATION_STEP, self.environment)
            self.accumulator -= SIMULATION_STEP


def main() -> None:
    """Run the graphical experiment until the user closes its window."""
    pygame.init()
    try:
        pygame.display.set_caption("Distributed Octopus Controller")
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
        application = Application()
        running = True
        while running:
            elapsed_seconds = clock.tick(60) / 1000
            for event in pygame.event.get():
                if not application.handle_event(event):
                    running = False
            if running:
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
                )
                pygame.display.flip()
    finally:
        # Release SDL resources even if an input or drawing error is raised.
        pygame.quit()
