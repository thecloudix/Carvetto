#pragma once

#include <string>

namespace carvetto::app {

class MainWindow;

/// Self-test of autosave and crash recovery (T-0357): a window with unsaved work autosaves on the preference's
/// timer, the "killed" session's snapshot is recovered into a second window state, a clean exit leaves nothing,
/// and the Options autosave preferences drive the timer. "" on success, else why not.
std::string recoverySelfTest(MainWindow& window);

}  // namespace carvetto::app
