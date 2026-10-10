"""Exercise the in-window picker with real filesystem entries and Pygame events."""

from pathlib import Path

import pygame

from octopus_controller.dialogs import (
    CANCEL_BUTTON,
    FILE_LIST,
    OPEN_BUTTON,
    PATH_FIELD,
    UP_BUTTON,
    ScenarioPicker,
)


def click(picker, rect):
    picker.handle_event(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=rect.center))


def test_browse_select_open_and_cancel(tmp_path):
    folder = tmp_path / "folder"
    folder.mkdir()
    meal = folder / "a reef.json"
    meal.write_text('{"version": 1}')
    (tmp_path / "ignore.txt").write_text("text")
    picker = ScenarioPicker(tmp_path)
    assert picker.entries == [folder]
    click(picker, OPEN_BUTTON)
    assert picker.directory == folder
    assert picker.result is None
    click(picker, OPEN_BUTTON)
    assert picker.result == meal
    picker = ScenarioPicker(folder)
    click(picker, UP_BUTTON)
    assert picker.directory == tmp_path
    click(picker, CANCEL_BUTTON)
    assert picker.cancelled


def test_path_entry_and_errors(tmp_path):
    picker = ScenarioPicker(tmp_path)
    click(picker, PATH_FIELD)
    picker.handle_event(pygame.event.Event(pygame.TEXTINPUT, text=str(tmp_path / "missing")))
    picker.activate()
    assert picker.error and picker.result is None
    assert picker.directory == tmp_path
    file = tmp_path / "reef.json"
    file.write_text("{}")
    click(picker, PATH_FIELD)
    picker.handle_event(pygame.event.Event(pygame.TEXTINPUT, text=str(file)))
    click(picker, OPEN_BUTTON)
    assert picker.result == file


def test_scroll_and_keyboard_selection(tmp_path):
    for index in range(20):
        (tmp_path / f"{index:02}.json").write_text("{}")
    picker = ScenarioPicker(tmp_path)
    for _ in range(19):
        picker.handle_event(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_DOWN))
    assert picker.selected == 19 and picker.offset == 9
    picker.handle_event(pygame.event.Event(pygame.MOUSEWHEEL, y=100))
    assert picker.offset == 0
    picker.handle_event(
        pygame.event.Event(
            pygame.MOUSEBUTTONDOWN, button=1, pos=(FILE_LIST.left + 5, FILE_LIST.top + 5)
        )
    )
    picker.activate()
    assert picker.result.name == "00.json"


def test_unreadable_folder_is_recoverable(monkeypatch, tmp_path):
    picker = ScenarioPicker(tmp_path)

    def denied(path):
        raise PermissionError("Access denied")

    monkeypatch.setattr(Path, "iterdir", denied)
    picker.browse(tmp_path / "private")
    assert picker.directory == tmp_path
    assert "Access denied" in picker.error
