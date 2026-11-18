#pragma once

#include <string>

namespace carvetto::app {

class MainWindow;

/// Self-test of the Wrapped Fluting Layout and Wrapped Spiral Layout dialogs (T-0340): the Setup menu entries
/// (rotary jobs only), the dialog widgets, the wrapped preview in the Model view, one undo step per layout and
/// the Fluting toolpath form taking its depth from the layout. Empty on success, else the failure.
std::string rotaryLayoutSelfTest(MainWindow& window);

}  // namespace carvetto::app
