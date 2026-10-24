# Carvetto — agent instructions

Open-source CNC CAD/CAM (C++20 core, Qt 6 app). Read `docs/WORKFLOW.md` for roles (director / coder / verifier), `docs/ARCHITECTURE.md` for binding design rules, `docs/BACKLOG.md` for task cards, `CONTRIBUTING.md` for the clean-room policy and code conventions.

Build & test (headless, used by coders and verifiers):
```
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCARVETTO_BUILD_APP=OFF
cmake --build build && ctest --test-dir build --output-on-failure
```
Qt is not available in the cloud workspace; the app is built by the Windows CI job.

Hard rules:
- `core/` never includes Qt. Units in core are mm and radians.
- No code or assets from proprietary CAM products. `docs/reference/` describes behaviour only.
- One task per branch (`task/T-XXXX-slug`); tests ship with the code; `main` stays green.
- Product decisions go to the project owner through the director, never guessed.
