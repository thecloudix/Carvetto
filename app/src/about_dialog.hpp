#pragma once

#include <QString>
#include <string>

class QDialog;
class QWidget;

namespace carvetto::app {

/// The program version: the CMake project VERSION, the single source shared with the CLI, the .carv
/// manifest, the installers and the Linux packages (T-0358).
QString appVersion();

/// First line printed by `carvetto --version`: "Carvetto 0.1.0".
QString versionLine();

/// Build information: compiler, build type, Qt version the binary was built against and runs with.
QString buildInfo();

/// The About text (plain text lines): version, licence, build info and the bundled dependencies.
QString aboutText();

/// The About dialog (not shown, not executed); the caller owns it via the parent.
QDialog* createAboutDialog(QWidget* parent);

/// Help > About Carvetto.
void showAboutDialog(QWidget* parent);

/// Self-test: the version matches the build, the text names licence, build info and dependencies, the
/// dialog shows them. "" on success, else why not.
std::string aboutSelfTest(QWidget* window);

}  // namespace carvetto::app
