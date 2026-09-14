#!/usr/bin/env python3
"""Tests for the installer, against a firmware tree small enough to write down.

The installer edits two files it does not own, and both edits have an anchor that is easy
to get subtly wrong: a `Kconfig.projbuild` with several `choice` blocks, and a
`CMakeLists.txt` whose board chain has to be extended *inside* its `if`/`endif`. Both
mistakes produce a tree that looks installed and is not — one puts the board under the
wrong menu, the other is a CMake syntax error.

    python3 scripts/test_install.py
"""
from __future__ import annotations

import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# A firmware tree with the shape that matters: an earlier choice block the board must not
# land in, and a board chain with a terminator after it.
KCONFIG = """menu "Assistant Configuration"

choice ASSETS_TYPE
    prompt "Assets"
    config FLASH_DEFAULT_ASSETS
        bool "Flash Default Assets"
endchoice

choice BOARD_TYPE
    prompt "Board Type"
    default BOARD_TYPE_EXAMPLE
    config BOARD_TYPE_EXAMPLE
        bool "Example"
        depends on IDF_TARGET_ESP32S3
endchoice

endmenu
"""

CMAKE = """set(SOURCES "application.cc")

if(CONFIG_BOARD_TYPE_EXAMPLE)
    set(BOARD_DIR "example")
    set(BUILTIN_TEXT_FONT font_example)
endif()

if(NOT BOARD_DIR AND NOT CMAKE_BUILD_EARLY_EXPANSION)
    message(FATAL_ERROR "The selected board does not define BOARD_DIR")
endif()
"""


def build_tree(root: Path) -> None:
    (root / "main" / "boards").mkdir(parents=True)
    (root / "main" / "Kconfig.projbuild").write_text(KCONFIG, encoding="utf-8")
    (root / "main" / "CMakeLists.txt").write_text(CMAKE, encoding="utf-8")
    (root / "main" / "mcp_server.h").write_text("// stand-in\n", encoding="utf-8")


def install(root: Path, *arguments: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "install.py"), str(root), *arguments],
        capture_output=True,
        text=True,
        check=False,
    )


def check(condition: bool, message: str) -> int:
    if condition:
        return 0
    print(f"FAIL {message}", file=sys.stderr)
    return 1


def main() -> int:
    failures = 0
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        build_tree(root)

        first = install(root)
        failures += check(first.returncode == 0, f"the first install failed: {first.stderr}")

        board = root / "main" / "boards" / "nilo-robot"
        failures += check(board.is_dir(), "the board directory was not created")
        for name in ("config.h", "config.json", "nilo_robot_board.cc", "robot_service.cc"):
            failures += check((board / name).exists(), f"{name} was not installed")
        # The build system globs *.cc at the top level of the board directory only.
        failures += check(
            not any(path.is_dir() for path in board.iterdir()),
            "the board has a subdirectory; the build system's glob would miss it",
        )

        kconfig = (root / "main" / "Kconfig.projbuild").read_text(encoding="utf-8")
        board_block = kconfig.index("choice BOARD_TYPE")
        entry = kconfig.index("BOARD_TYPE_NILO_ROBOT")
        closing = kconfig.index("endchoice", board_block)
        failures += check(
            board_block < entry < closing,
            "the board entry landed outside the BOARD_TYPE choice block",
        )

        cmake = (root / "main" / "CMakeLists.txt").read_text(encoding="utf-8")
        branch = cmake.index("elseif(CONFIG_BOARD_TYPE_NILO_ROBOT)")
        chain_end = cmake.index("endif()")
        failures += check(
            branch < chain_end,
            "the CMake branch landed after the chain closed; that is a syntax error",
        )

        again = install(root, "--check")
        failures += check(again.returncode == 0, "a second install reported drift; it is not idempotent")
        failures += check("already up to date" in again.stdout, "--check did not report a clean tree")

        # A firmware update that reverts the build-system edits is re-applied, and the
        # board files that are already correct are left alone.
        (root / "main" / "CMakeLists.txt").write_text(CMAKE, encoding="utf-8")
        repair = install(root)
        failures += check(repair.returncode == 0, "re-applying after an update failed")
        failures += check(
            "CMakeLists.txt" in repair.stdout and "config.h" not in repair.stdout,
            "a repair rewrote files that had not changed",
        )

    missing = install(Path("/nonexistent-firmware"), "--check")
    failures += check(missing.returncode == 2, "a tree that is not a firmware checkout was accepted")

    if failures:
        print(f"FAIL install ({failures} failure{'s' if failures != 1 else ''})")
        return 1
    print("ok   install")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
