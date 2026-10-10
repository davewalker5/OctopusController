"""A file picker driven by the existing Pygame event loop, with no native windows."""

from pathlib import Path

import pygame

PANEL = pygame.Rect(200, 100, 720, 580)
PATH_FIELD = pygame.Rect(220, 150, 680, 32)
UP_BUTTON = pygame.Rect(220, 195, 90, 30)
HOME_BUTTON = pygame.Rect(320, 195, 90, 30)
FILE_LIST = pygame.Rect(220, 238, 680, 330)
CANCEL_BUTTON = pygame.Rect(660, 630, 110, 30)
OPEN_BUTTON = pygame.Rect(790, 630, 110, 30)
ROW_HEIGHT = 30
VISIBLE_ROWS = FILE_LIST.height // ROW_HEIGHT


class ScenarioPicker:
    """Browse directories and JSON files without taking over the application loop."""

    def __init__(self, directory: Path, *, saving: bool = False) -> None:
        self.saving = saving
        self.directory = directory
        self.entries: list[Path] = []
        self.selected = 0
        self.offset = 0
        self.path_text = str(directory)
        self.path_focus = False
        self.select_all = False
        self.error = ""
        self.cancelled = False
        self.result: Path | None = None
        self.browse(directory)

    def browse(self, directory: Path) -> None:
        """Replace the listing only after a directory can be read successfully."""
        try:
            entries = [p for p in directory.iterdir() if p.is_dir() or p.suffix.lower() == ".json"]
            entries.sort(key=lambda p: (not p.is_dir(), p.name.casefold()))
        except OSError as error:
            self.error = str(error)
            return
        self.directory = directory
        self.entries = entries
        self.path_text = str(directory)
        self.selected = self.offset = 0
        self.error = ""
        self.path_focus = self.select_all = self.saving
        if self.saving:
            self.path_text = str(directory / "new-scenario.json")

    def activate(self) -> None:
        """Enter a directory or return a selected file for scenario validation."""
        if self.path_focus:
            path = Path(self.path_text).expanduser()
            if not path.is_absolute():
                path = self.directory / path
        elif self.entries:
            path = self.entries[self.selected]
        else:
            return
        try:
            if path.is_dir():
                self.browse(path)
            elif self.saving:
                if path.suffix.lower() != ".json":
                    path = path.with_name(path.name + ".json")
                if path.exists():
                    self.error = "A file with that name exists. Choose a new filename."
                elif not path.parent.is_dir():
                    self.error = "Choose an existing folder for the new scenario."
                else:
                    self.result = path
            elif path.is_file():
                self.result = path
            else:
                self.error = "That file or folder does not exist."
        except OSError as error:
            self.error = str(error)

    def handle_event(self, event: pygame.event.Event) -> None:
        """Consume modal input while the parent continues rendering and handling Quit."""
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                self.cancelled = True
            elif event.key == pygame.K_TAB:
                self.path_focus = not self.path_focus
                self.select_all = self.path_focus
            elif event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                self.activate()
            elif self.path_focus:
                if event.key == pygame.K_a and getattr(event, "mod", 0) & (
                    pygame.KMOD_CTRL | pygame.KMOD_GUI
                ):
                    self.select_all = True
                elif event.key == pygame.K_BACKSPACE:
                    self.path_text = "" if self.select_all else self.path_text[:-1]
                    self.select_all = False
            elif event.key in (pygame.K_UP, pygame.K_DOWN) and self.entries:
                self.selected = max(
                    0,
                    min(
                        len(self.entries) - 1,
                        self.selected + (1 if event.key == pygame.K_DOWN else -1),
                    ),
                )
                self.offset = max(0, min(self.offset, self.selected))
                if self.selected >= self.offset + VISIBLE_ROWS:
                    self.offset = self.selected - VISIBLE_ROWS + 1
        elif event.type == pygame.TEXTINPUT and self.path_focus:
            self.path_text = ("" if self.select_all else self.path_text) + event.text
            self.select_all = False
        elif event.type == pygame.MOUSEWHEEL:
            self.offset = max(
                0, min(max(0, len(self.entries) - VISIBLE_ROWS), self.offset - event.y)
            )
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if CANCEL_BUTTON.collidepoint(event.pos):
                self.cancelled = True
            elif OPEN_BUTTON.collidepoint(event.pos):
                self.activate()
            elif UP_BUTTON.collidepoint(event.pos):
                self.browse(self.directory.parent)
            elif HOME_BUTTON.collidepoint(event.pos):
                self.browse(Path.home())
            elif PATH_FIELD.collidepoint(event.pos):
                self.path_focus = self.select_all = True
            elif FILE_LIST.collidepoint(event.pos):
                index = self.offset + (event.pos[1] - FILE_LIST.top) // ROW_HEIGHT
                if index < len(self.entries):
                    self.selected = index
                    self.path_focus = False

    def draw(self, screen: pygame.Surface, font: pygame.font.Font) -> None:
        """Draw a modal panel in the same window as the simulation."""
        text, muted, accent = (220, 231, 239), (143, 164, 181), (79, 212, 184)
        shade = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
        shade.fill((0, 0, 0, 150))
        screen.blit(shade, (0, 0))
        pygame.draw.rect(screen, (22, 37, 51), PANEL, border_radius=10)
        screen.blit(
            font.render("Save scenario" if self.saving else "Load scenario", True, text), (220, 118)
        )

        def label(value: str, rect: pygame.Rect, colour=text, tail=False) -> None:
            # Clip names/paths without allowing them to cover adjacent controls.
            screen.set_clip(rect)
            rendered = font.render(value, True, colour)
            x = min(rect.left, rect.right - rendered.get_width()) if tail else rect.left
            screen.blit(rendered, (x, rect.top + 6))
            screen.set_clip(None)

        pygame.draw.rect(screen, (39, 61, 77), PATH_FIELD, border_radius=4)
        pygame.draw.rect(screen, accent if self.path_focus else muted, PATH_FIELD, 1, 4)
        label(self.path_text, PATH_FIELD.inflate(-12, 0), tail=True)
        if self.path_focus and self.select_all:
            pygame.draw.line(screen, accent, (226, 178), (894, 178), 2)
        for rect, title in [
            (UP_BUTTON, "Up"),
            (HOME_BUTTON, "Home"),
            (CANCEL_BUTTON, "Cancel"),
            (OPEN_BUTTON, "Save" if self.saving and self.path_focus else "Open"),
        ]:
            pygame.draw.rect(screen, (39, 61, 77), rect, border_radius=4)
            label(title, rect.inflate(-16, 0))
        for row, path in enumerate(self.entries[self.offset : self.offset + VISIBLE_ROWS]):
            rect = pygame.Rect(
                FILE_LIST.left, FILE_LIST.top + row * ROW_HEIGHT, FILE_LIST.width, ROW_HEIGHT
            )
            if self.offset + row == self.selected:
                pygame.draw.rect(screen, (39, 61, 77), rect)
            label(("[Folder] " if path.is_dir() else "") + path.name, rect.inflate(-8, 0))
        if not self.entries:
            label("No JSON files or folders here.", FILE_LIST, muted)
        label(
            f"{len(self.entries)} items · Scroll or use arrow keys · Enter to open",
            pygame.Rect(220, 575, 680, 26),
            muted,
        )
        label(
            self.error
            or (
                "Enter a new filename above · Esc to cancel"
                if self.saving
                else "Tab to edit a path · Esc to cancel"
            ),
            pygame.Rect(220, 601, 680, 26),
            (245, 160, 140) if self.error else muted,
        )


class ScenarioNamePrompt:
    """Edit a scenario label within the existing responsive Pygame window."""

    panel = pygame.Rect(240, 260, 640, 220)
    field = pygame.Rect(260, 310, 600, 36)
    cancel_button = pygame.Rect(620, 428, 110, 30)
    rename_button = pygame.Rect(750, 428, 110, 30)

    def __init__(self, name: str) -> None:
        self.text = name
        self.cursor = len(name)
        self.select_all = True
        self.cancelled = False
        self.result: str | None = None
        self.error = ""

    def accept(self) -> None:
        """Reject blank labels before publishing the edited name."""
        if not self.text.strip():
            self.error = "Please enter a scenario name."
        else:
            self.result = self.text.strip()

    def handle_event(self, event: pygame.event.Event) -> None:
        """Consume editing keys so they cannot also control the simulation."""
        if event.type == pygame.TEXTINPUT:
            if self.select_all:
                self.text, self.cursor = "", 0
            self.text = self.text[: self.cursor] + event.text + self.text[self.cursor :]
            self.cursor += len(event.text)
            self.select_all = False
            self.error = ""
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                self.cancelled = True
            elif event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                self.accept()
            elif event.key == pygame.K_a and getattr(event, "mod", 0) & (
                pygame.KMOD_CTRL | pygame.KMOD_GUI
            ):
                self.select_all = True
            elif event.key in (pygame.K_BACKSPACE, pygame.K_DELETE):
                if self.select_all:
                    self.text, self.cursor = "", 0
                elif event.key == pygame.K_BACKSPACE and self.cursor:
                    self.text = self.text[: self.cursor - 1] + self.text[self.cursor :]
                    self.cursor -= 1
                elif event.key == pygame.K_DELETE:
                    self.text = self.text[: self.cursor] + self.text[self.cursor + 1 :]
                self.select_all = False
            elif event.key in (pygame.K_LEFT, pygame.K_RIGHT, pygame.K_HOME, pygame.K_END):
                if event.key == pygame.K_HOME:
                    self.cursor = 0
                elif event.key == pygame.K_END:
                    self.cursor = len(self.text)
                else:
                    self.cursor = max(
                        0,
                        min(
                            len(self.text), self.cursor + (1 if event.key == pygame.K_RIGHT else -1)
                        ),
                    )
                self.select_all = False
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.cancel_button.collidepoint(event.pos):
                self.cancelled = True
            elif self.rename_button.collidepoint(event.pos):
                self.accept()
            elif self.field.collidepoint(event.pos):
                self.select_all = True

    def draw(self, screen: pygame.Surface, font: pygame.font.Font) -> None:
        """Display the full editable name, scrolling horizontally with the caret."""
        text, accent = (220, 231, 239), (79, 212, 184)
        shade = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
        shade.fill((0, 0, 0, 150))
        screen.blit(shade, (0, 0))
        pygame.draw.rect(screen, (22, 37, 51), self.panel, border_radius=10)
        screen.blit(font.render("Rename scenario", True, text), (260, 278))
        pygame.draw.rect(screen, (39, 61, 77), self.field, border_radius=4)
        pygame.draw.rect(screen, accent, self.field, 1, 4)
        screen.set_clip(self.field.inflate(-12, -4))
        caret_width = font.size(self.text[: self.cursor])[0]
        x = self.field.left + 8 - max(0, caret_width - self.field.width + 20)
        if self.select_all:
            pygame.draw.rect(
                screen, (55, 95, 110), (x, self.field.top + 6, font.size(self.text)[0], 24)
            )
        screen.blit(font.render(self.text, True, text), (x, self.field.top + 8))
        pygame.draw.line(
            screen,
            accent,
            (x + caret_width, self.field.top + 6),
            (x + caret_width, self.field.bottom - 6),
        )
        screen.set_clip(None)
        screen.blit(
            font.render(self.error or "Enter to rename · Esc to cancel", True, text), (260, 364)
        )
        for rect, label in [(self.cancel_button, "Cancel"), (self.rename_button, "Rename")]:
            pygame.draw.rect(screen, (39, 61, 77), rect, border_radius=4)
            rendered = font.render(label, True, text)
            screen.blit(rendered, rendered.get_rect(center=rect.center))
