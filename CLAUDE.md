# Carvetto — agent instructions

Open-source CNC CAD/CAM (C++20 core, Qt 6 app). Read `docs/WORKFLOW.md` for roles (director / coder / verifier), `docs/ARCHITECTURE.md` for binding design rules, `docs/BACKLOG.md` for task cards, `CONTRIBUTING.md` for the clean-room policy and code conventions.

Build & test (headless, used by coders and verifiers):
```
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCARVETTO_BUILD_APP=OFF
cmake --build build && ctest --test-dir build --output-on-failure
```
The headless build above covers `core` and the Qt-free `app/viewmodel`. To build and run the real Qt app
in the cloud workspace (download.qt.io is blocked there), install Qt 6.7 from conda-forge once (~30 s):
```
tools/setup_qt.sh <dir>        # e.g. build-verify/qt-prefix (git-ignored)
cmake -S . -B build-qt -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=<dir>/env
cmake --build build-qt
LD_LIBRARY_PATH=<dir>/env/lib QT_PLUGIN_PATH=<dir>/env/lib/qt6/plugins QT_QPA_PLATFORM=offscreen build-qt/app/carvetto --self-test
```
CI builds and self-tests the app on Linux and Windows (MSVC) for every PR.

Hard rules:
- `core/` never includes Qt. Units in core are mm and radians.
- No code or assets from proprietary CAM products. `docs/reference/` describes behaviour only.
- One task per branch (`task/T-XXXX-slug`); tests ship with the code; `main` stays green.
- Product decisions go to the project owner through the director, never guessed.
