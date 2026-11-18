#pragma once

#include <string>

namespace carvetto::app {

class MainWindow;

/// Self-test of drawing in the 3D view (T-0345): a rectangle and a circle drawn with mouse events on the zero plane
/// become document vectors at the picked coordinates, one undo step each; picks in the wrapped rotary view or from
/// an edge-on camera are refused. Empty on success, else the failure.
std::string draw3dSelfTest(MainWindow& window);

}  // namespace carvetto::app
