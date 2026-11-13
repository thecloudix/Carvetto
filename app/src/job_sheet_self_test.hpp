#pragma once

#include <string>

namespace carvetto::app {

class MainWindow;

/// Self-test of File > Create Job Sheet (T-0259): HTML, text and PDF are written, equal to
/// makeJobSheet's output, hostile paths are refused. Empty on success, else the failure.
std::string jobSheetSelfTest(MainWindow& window);

}  // namespace carvetto::app
