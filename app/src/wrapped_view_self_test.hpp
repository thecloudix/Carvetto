#pragma once

#include <string>

namespace carvetto::app {

class MainWindow;

/// Self-test of the Auto-Wrapping View (T-0334): the View > Wrapped View toggle (rotary jobs only), the Model
/// view drawn wrapped or flat with vectors and toolpaths, the simulation view played wrapped, and the rotary post
/// refusal/warning dialog, through the real widgets offscreen. Empty on success, else the failure.
std::string wrappedViewSelfTest(MainWindow& window);

}  // namespace carvetto::app
