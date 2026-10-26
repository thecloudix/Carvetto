#!/usr/bin/env python3
"""Regenerate Carvetto's golden files and the canonical starter tool database.

What it rewrites, from the current sources:
  - core/tests/golden/post/<post>_fixture.nc for every shipped post (posts/*.pp listed in
    core/tests/post/shipped_posts_test.cpp), from the shared fixture in post_fixture.hpp;
  - core/tests/golden/post/grbl_square.nc;
  - core/tests/golden/toolpath/profile_rect_tabs.moves and profile_rect_tabs_grbl.nc (T-0120; hand-check
    against the notes in docs/adr/0016-profile-toolpath.md);
  - data/tools/starter_tools.json, re-exported in the exporter's canonical form (sorted keys,
    id order, rounded numbers), so hand edits to the database become byte-stable.

It does this by running the hidden Catch2 test cases tagged [.regen] in the already-built test
binary. These tests are never run by ctest. Build first, for example:

    cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCARVETTO_BUILD_APP=OFF
    cmake --build build
    python3 tools/regen_golden.py            # or: --build-dir <dir>

IMPORTANT: regenerated goldens are NOT automatically correct. Every changed golden must be
hand-reviewed against the machine-safety checklist in posts/README.md, and every change to
starter_tools.json must be reviewed for plausible, conservative values, before committing.
Afterwards, run the full ctest suite. The safety and chip-load tests run on the regenerated
files too.
"""

import argparse
import pathlib
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
OUTPUTS = ["core/tests/golden/post", "core/tests/golden/toolpath", "data/tools/starter_tools.json"]


def find_test_binary(build_dir: pathlib.Path) -> pathlib.Path:
    candidates = [
        build_dir / "core" / "tests" / "carvetto_tests",
        build_dir / "core" / "tests" / "carvetto_tests.exe",
    ]
    candidates += sorted(build_dir.glob("core/tests/*/carvetto_tests.exe"))  # multi-config builds
    for c in candidates:
        if c.is_file():
            return c
    sys.exit(f"carvetto_tests not found under {build_dir}; build the project first")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--build-dir", default=str(REPO / "build"), help="CMake build directory")
    args = parser.parse_args()

    binary = find_test_binary(pathlib.Path(args.build_dir).resolve())
    result = subprocess.run([str(binary), "[.regen]"], cwd=REPO)
    if result.returncode != 0:
        print("regeneration failed; nothing should be committed", file=sys.stderr)
        return result.returncode

    diff = subprocess.run(["git", "status", "--short", "--", *OUTPUTS], cwd=REPO,
                          capture_output=True, text=True)
    changed = diff.stdout.strip()
    if changed:
        print("Changed files (HAND-REVIEW each one against posts/README.md before committing):")
        print(changed)
    else:
        print("No golden or starter-database changes.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
