#!/usr/bin/env python3
"""Regenerate Carvetto's golden files and the canonical starter tool database.

What it rewrites, from the current sources:
  - core/tests/golden/post/<post>_fixture.nc for every shipped post (posts/*.pp listed in
    core/tests/post/shipped_posts_test.cpp), from the shared fixture in post_fixture.hpp;
  - core/tests/golden/post/<post>_drag.nc for every shipped post, from the spindle-off (diamond
    drag) fixture in post_fixture.hpp (T-0154; hand-check against posts/README.md point 11);
  - core/tests/golden/post/grbl_square.nc;
  - core/tests/golden/toolpath/profile_rect_tabs.moves and profile_rect_tabs_grbl.nc (T-0120; hand-check
    against the notes in docs/adr/0016-profile-toolpath.md);
  - core/tests/golden/toolpath/pocket_rect_grbl.nc (T-0121 pocket through the Grbl post);
  - core/tests/golden/toolpath/drill_holes_grbl.nc (T-0150 drilling fixture through the Grbl post);
  - core/tests/golden/toolpath/engrave_grbl.nc (T-0151 quick engrave fixture through the Grbl post);
  - core/tests/golden/cli/part_grbl.nc (T-0122 carvetto-cli end to end: app/cli/tests/data/part.dxf
    -> pocket.json -> profile.json with tabs -> posts/grbl_mm.pp; the ctest cli.end_to_end_grbl and
    CI compare the real executable's output with it by SHA-256);
  - core/tests/golden/cli/part_drill_grbl.nc (T-0150: part.dxf -> drill.json -> profile.json -> grbl_mm.pp;
    the ctest cli.end_to_end_drill_grbl and CI compare it the same way);
  - core/tests/golden/cli/part_engrave_grbl.nc (T-0151: part.dxf -> engrave_fill.json -> engrave_outline.json
    -> grbl_mm.pp; the ctest cli.end_to_end_engrave_grbl and CI compare it the same way);
  - data/tools/starter_tools.json, re-exported in the exporter's canonical form (sorted keys,
    id order, rounded numbers), so hand edits to the database become byte-stable.

It does this by running the hidden Catch2 test cases tagged [.regen] in the already-built test
binaries (carvetto_tests, and carvetto_cli_tests when the CLI is built). These tests are never run by ctest. Build first, for example:

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
OUTPUTS = ["core/tests/golden/post", "core/tests/golden/toolpath", "core/tests/golden/cli",
           "data/tools/starter_tools.json"]


def find_binary(build_dir: pathlib.Path, subdir: str, name: str):
    candidates = [build_dir / subdir / name, build_dir / subdir / f"{name}.exe"]
    candidates += sorted(build_dir.glob(f"{subdir}/*/{name}.exe"))  # multi-config builds
    for c in candidates:
        if c.is_file():
            return c
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--build-dir", default=str(REPO / "build"), help="CMake build directory")
    args = parser.parse_args()

    build_dir = pathlib.Path(args.build_dir).resolve()
    core_tests = find_binary(build_dir, "core/tests", "carvetto_tests")
    if core_tests is None:
        sys.exit(f"carvetto_tests not found under {build_dir}; build the project first")
    cli_tests = find_binary(build_dir, "app/cli/tests", "carvetto_cli_tests")
    if cli_tests is None:
        print("warning: carvetto_cli_tests not found (CARVETTO_BUILD_CLI=OFF?); "
              "the CLI golden is not regenerated", file=sys.stderr)
    for binary in [b for b in (core_tests, cli_tests) if b is not None]:
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
