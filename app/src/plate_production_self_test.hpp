#pragma once

#include <string>

namespace carvetto::app {

class MainWindow;

/// Self-test of Plate Production / data merge (T-0347): the Edit > Copies entry, the dialog's variable
/// assignment and preview, a 20-row CSV merged through the window as one undo step, and the refusal of a
/// malformed file with line-numbered diagnostics. Empty on success, else the failure.
std::string plateProductionSelfTest(MainWindow& window);

}  // namespace carvetto::app
