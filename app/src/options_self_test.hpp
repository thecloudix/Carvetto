#pragma once

#include <string>

namespace carvetto::app {

class MainWindow;

/// Self-test of Setup > Options (T-0348): the dialog edits every preference, refuses bad values, restores
/// defaults, applies to the live window and survives a save / load round trip. "" on success, else why not.
std::string optionsSelfTest(MainWindow& window);

}  // namespace carvetto::app
