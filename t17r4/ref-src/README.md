# Carvetto

Open-source CNC CAD/CAM for 2D/2.5D machining and 3D relief carving. Windows first, Linux next.

Carvetto is a clean-room re-implementation of the workflow found in commercial relief-carving CAM packages: draw or import vectors, build 3D reliefs from stacked components, generate toolpaths, preview the cut, and post G-code through plain-text post-processor templates.

**Status:** pre-alpha, Phase 0 (foundations). Nothing is usable yet.

## Goals

- 2D drawing, text, and vector editing on layers and sheets
- 2.5D toolpaths: profile, pocket, drill, V-carve, fluting, chamfer, inlay, texture, and more
- 3D relief modeling (components, levels, combine modes, sweeps, sculpting) and 3D rough/finish toolpaths
- Material-removal simulation and time estimation
- `.pp`-compatible post-processor templates so existing community posts work unchanged
- Lua scripting for add-ons

See [docs/PLAN.md](docs/PLAN.md) for the roadmap and [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the design.

## Building

Requirements: CMake 3.24+, a C++20 compiler (MSVC 2022, GCC 13, Clang 17), Ninja. Qt 6.5+ is required only for the desktop app; the `carvetto_core` library and its tests build without Qt.

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

Pass `-DCARVETTO_BUILD_APP=OFF` to skip the Qt app.

## Development model

Carvetto is developed by a director/coder/verifier agent workflow coordinated by a human project owner. See [docs/WORKFLOW.md](docs/WORKFLOW.md) and [CONTRIBUTING.md](CONTRIBUTING.md).

## License

GPL-3.0-or-later. See [LICENSE](LICENSE).
