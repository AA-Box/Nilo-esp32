#!/usr/bin/env python3
"""Install the Nilo robot board into an ESP-IDF voice-assistant firmware checkout.

    python3 scripts/install.py ../firmware          # install or update
    python3 scripts/install.py ../firmware --check  # report drift, change nothing

It copies `board/` to `main/boards/nilo-robot/` and applies the two build-system hooks a
custom board needs: a `BOARD_TYPE` entry in `main/Kconfig.projbuild`, and a branch in the
board-type chain in `main/CMakeLists.txt`.

Idempotent by construction: every edit is checked for before it is made, so running this
twice is the same as running it once, and running it after a firmware update re-applies
only what the update removed.

It never edits an existing board. The identity in `config.json` is unique and stays that
way after release, because OTA updates are keyed on the reported board type — a board that
borrowed another's identity would be handed the other board's firmware.
"""
from __future__ import annotations

import argparse
import difflib
import shutil
import sys
from pathlib import Path

BOARD_NAME = "nilo-robot"
KCONFIG_SYMBOL = "BOARD_TYPE_NILO_ROBOT"

KCONFIG_ENTRY = """    config BOARD_TYPE_NILO_ROBOT
        bool "Nilo Robot"
        depends on IDF_TARGET_ESP32S3
"""

CMAKE_ENTRY = """elseif(CONFIG_BOARD_TYPE_NILO_ROBOT)
    set(BOARD_DIR "nilo-robot")
    set(BUILTIN_TEXT_FONT font_puhui_basic_16_4)
    set(BUILTIN_ICON_FONT font_awesome_16_4)
"""

#: Files that are ours to install. Everything else in `board/` is documentation.
SOURCE_SUFFIXES = {".h", ".cc", ".c", ".json"}


class InstallError(RuntimeError):
    """Something about the target tree is not what this script expects."""


def board_files(root: Path) -> list[Path]:
    return sorted(path for path in (root / "board").iterdir() if path.suffix in SOURCE_SUFFIXES)


def copy_board(root: Path, firmware: Path, check: bool) -> list[str]:
    target = firmware / "main" / "boards" / BOARD_NAME
    changes: list[str] = []
    if not check:
        target.mkdir(parents=True, exist_ok=True)
    for source in board_files(root):
        destination = target / source.name
        existing = destination.read_bytes() if destination.exists() else None
        wanted = source.read_bytes()
        if existing == wanted:
            continue
        changes.append(f"{'update' if existing else 'add'} main/boards/{BOARD_NAME}/{source.name}")
        if not check:
            shutil.copy2(source, destination)
    # A file that used to be ours and is not any more would keep compiling forever.
    if target.exists():
        ours = {source.name for source in board_files(root)}
        for stale in sorted(target.iterdir()):
            if stale.is_file() and stale.name not in ours:
                changes.append(f"remove main/boards/{BOARD_NAME}/{stale.name}")
                if not check:
                    stale.unlink()
    return changes


def patch_kconfig(firmware: Path, check: bool) -> list[str]:
    path = firmware / "main" / "Kconfig.projbuild"
    text = _read(path)
    if KCONFIG_SYMBOL in text:
        return []
    # The *board* choice block, not the first `endchoice` in the file: there are several
    # choice blocks and the assets one comes first. Anchoring on the wrong one produces a
    # board entry that is offered under "flash assets", which nobody would ever find.
    start = text.find("choice BOARD_TYPE")
    if start < 0:
        raise InstallError(f"{path} has no `choice BOARD_TYPE` block")
    end = text.find("endchoice", start)
    if end < 0:
        raise InstallError(f"{path}: the `choice BOARD_TYPE` block is not closed")
    patched = text[:end] + KCONFIG_ENTRY + text[end:]
    if not check:
        path.write_text(patched, encoding="utf-8")
    return [f"add {KCONFIG_SYMBOL} to main/Kconfig.projbuild"]


def patch_cmake(firmware: Path, check: bool) -> list[str]:
    path = firmware / "main" / "CMakeLists.txt"
    text = _read(path)
    if f'set(BOARD_DIR "{BOARD_NAME}")' in text:
        return []
    # Inside the chain, before the `endif()` that closes it. Appending after that `endif()`
    # would be an `elseif` with no `if`, which is a CMake syntax error rather than a board
    # that quietly does not appear — but it is still a broken tree, so anchor properly.
    last_board = text.rfind('set(BOARD_DIR "')
    if last_board < 0:
        raise InstallError(f"{path} has no board-type chain; the build system has moved")
    end = text.find("\nendif()", last_board)
    if end < 0:
        raise InstallError(f"{path}: the board-type chain is not closed")
    insertion = end + 1  # just before `endif()`
    patched = text[:insertion] + CMAKE_ENTRY + text[insertion:]
    if not check:
        path.write_text(patched, encoding="utf-8")
    return [f"add the {BOARD_NAME} branch to main/CMakeLists.txt"]


def _read(path: Path) -> str:
    if not path.exists():
        raise InstallError(f"{path} does not exist; is this a firmware checkout?")
    return path.read_text(encoding="utf-8")


def verify(firmware: Path) -> None:
    for required in ("main/CMakeLists.txt", "main/Kconfig.projbuild", "main/boards", "main/mcp_server.h"):
        if not (firmware / required).exists():
            raise InstallError(
                f"{firmware} does not look like a firmware checkout: {required} is missing"
            )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("firmware", type=Path, help="path to the firmware checkout")
    parser.add_argument("--check", action="store_true", help="report what would change, change nothing")
    arguments = parser.parse_args()

    root = Path(__file__).resolve().parent.parent
    firmware = arguments.firmware.resolve()
    try:
        verify(firmware)
        changes = copy_board(root, firmware, arguments.check)
        changes += patch_kconfig(firmware, arguments.check)
        changes += patch_cmake(firmware, arguments.check)
    except InstallError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    if not changes:
        print(f"{firmware}: already up to date")
        return 0
    for change in changes:
        print(("would " if arguments.check else "") + change)
    if arguments.check:
        return 1
    print()
    print("Now build it:")
    print(f"    cd {firmware} && python scripts/build.py {BOARD_NAME}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
