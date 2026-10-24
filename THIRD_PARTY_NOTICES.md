# Third-party notices

Carvetto is GPL-3.0-or-later. It uses the following third-party components, each under its own
GPL-compatible licence. Their full licence texts are fetched with the sources at build time
(see `core/CMakeLists.txt`) and must be shipped alongside any binary distribution.

| Component | Version | Licence | Used for |
|---|---|---|---|
| Qt | 6.x | LGPL-3.0 (dynamically linked) | Desktop UI (`app/`) |
| Clipper2 | 1.4.0 | Boost Software License 1.0 | Polygon offsets and booleans |
| nlohmann/json | 3.11.3 | MIT | JSON import/export |
| miniz | 3.0.2 | MIT | `.carv` zip container |
| Catch2 | 3.7.1 | Boost Software License 1.0 | Unit tests only (not distributed) |

When adding a dependency: pin it (tag or URL + SHA-256), confirm GPL compatibility, add a row here.
