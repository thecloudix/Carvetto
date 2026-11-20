#pragma once
// T-0363: the application icon (own artwork, packaging/icons/README.md), embedded as a Qt resource.
#include <QIcon>
#include <string>

namespace carvetto::app {

// The multi-size application icon: 16, 24, 32, 48, 64, 128 and 256 px (16 and 24 px use the simplified variant).
QIcon applicationIcon();

// Empty when the icon loads at every size from 16 to 256 px; otherwise the reason.
std::string appIconSelfTest();

}  // namespace carvetto::app
