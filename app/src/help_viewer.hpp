#pragma once

#include <filesystem>
#include <string>

class QMenu;
class QWidget;

namespace carvetto::app {

/// Where the bundled user guide lives (docs/user): beside the executable, in the installed share/carvetto
/// tree, or in the source tree for development builds. Empty when it is not found.
std::filesystem::path userDocsDir();

/// Adds "Quick Start" and "User Guide" to the Help menu (offline viewer on the bundled Markdown pages, T-0355).
void installHelpActions(QMenu* helpMenu, QWidget* parent);

/// Opens the viewer on a page of the guide ("README.md" = the index, "quick-start.md").
/// Returns false (and shows a message) when the guide is not installed.
bool showUserGuide(QWidget* parent, const std::string& page);

/// Self-test: the Help menu entries exist, the index and quick start load, every local link of the
/// guide resolves. "" on success, else why not.
std::string helpSelfTest(QWidget* window);

}  // namespace carvetto::app
