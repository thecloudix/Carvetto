#pragma once

#include <string>

namespace carvetto::app {

class MainWindow;

/// Self-test of the localization plumbing (T-0350): the pseudo-locale transforms the menu bar and the
/// Options dialog, a missing catalog falls back to English, the sample German catalog loads when it was
/// built. "" on success, else why not.
std::string l10nSelfTest(MainWindow& window);

}  // namespace carvetto::app
