# ADR-0001: C++20 core with Qt 6 desktop UI

Date: 2026-10-24 · Status: accepted · Decided by: project owner

## Decision
The engine (`carvetto_core`) is C++20 with no UI dependency. The desktop application uses Qt 6 (Widgets, OpenGL via QRhi/QOpenGLWidget). Windows is the first supported platform; Linux is kept building in CI from the start so it is a packaging task later, not a port.

## Alternatives considered
- Rust: safer, but thinner CAD/UI ecosystem; slower start.
- C#/.NET + WPF/Avalonia: comfortable on Windows, weak geometry libraries, Linux UI story weaker.

## Consequences
- Heavy geometry (offsetting, V-carve medial axis, 3D raster) can use mature C/C++ libraries (Clipper2, FreeType, HarfBuzz, potrace).
- Qt is LGPL-3; dynamic linking keeps us compliant and is compatible with GPL-3.0-or-later.
- Contributors need MSVC 2022 or GCC 13+/Clang 17+.
