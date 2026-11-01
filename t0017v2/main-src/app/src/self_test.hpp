#pragma once

namespace carvetto::app {

/// Headless smoke test of the application shell (`carvetto --self-test`). Requires a live
/// `QApplication`. Creates a job, adds a circle, renders the 2D canvas to an image and checks
/// that the circle drew inside the material, undoes the add and checks that it is gone, and
/// round-trips the dock layout through a temporary INI file. Returns 0 when every check passes;
/// otherwise prints one line with the reason to stderr and returns 1.
int runSelfTest();

}  // namespace carvetto::app
