#pragma once

#include <string>

namespace carvetto::app {

class MainWindow;

/// Self-test of rotary jobs (T-0319): Job Setup page, Material Setup page, Edit Sheet and a .carv round
/// trip, through the real dialogs and panels. Empty on success, else the failure.
std::string rotarySelfTest(MainWindow& window);

}  // namespace carvetto::app
