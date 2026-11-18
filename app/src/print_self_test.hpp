#pragma once

#include <string>

namespace carvetto::app {

class MainWindow;

/// Self-test of File > Print / Print Preview / Print to PDF (T-0349): the actions exist, fit-to-page
/// and 1:1 tiled PDFs have the page count of the layout, the job sheet prints, the print paints white
/// paper, hostile paths are refused. Empty on success, else the failure.
std::string printSelfTest(MainWindow& window);

}  // namespace carvetto::app
