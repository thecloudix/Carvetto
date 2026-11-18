#pragma once

#include <QDialog>
#include <filesystem>
#include <optional>
#include <string>

#include "carvetto/viewmodel/rotary_tools.hpp"

class QAction;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QMenu;

// UI of the rotary tools of T-0321 (ADR-0114): the Vector Unwrapper form and the rotary 3D import dialog, with the
// Setup menu entries that open them. The logic is in `viewmodel/rotary_tools.hpp`.

namespace carvetto::app {

class MainWindow;

/// Vector Unwrapper form: centre of the unwrapping (contour, workspace, custom point) and the refine tolerance.
class UnwrapperDialog : public QDialog {
   public:
    explicit UnwrapperDialog(QWidget* parent = nullptr);
    viewmodel::UnwrapperSettings settings() const;
    void setSettings(const viewmodel::UnwrapperSettings& s);

   private:
    void sync();
    QComboBox* centre_;
    QDoubleSpinBox *centreX_, *centreY_, *tolerance_;
    QCheckBox* refine_;
};

/// Import 3D Model into a Rotary job: model type, orientation, fit to material, axis movement and placement.
class RotaryImportDialog : public QDialog {
   public:
    explicit RotaryImportDialog(QWidget* parent = nullptr);
    viewmodel::RotaryImportSettings settings() const;
    void setSettings(const viewmodel::RotaryImportSettings& s);

   private:
    void sync();
    QComboBox *units_, *type_, *top_, *rotation_;
    QCheckBox *fit_, *moveAxis_, *centreAlong_;
    QDoubleSpinBox *shiftA_, *shiftB_, *along_, *cell_;
};

/// Adds "Vector Unwrapper (rotary)..." and "Import 3D Model into Rotary Job..." to `menu` (the Setup menu); both are
/// enabled for rotary jobs only (checked when the menu opens).
void addRotaryToolsActions(MainWindow& window, QMenu* menu);

/// Unwraps the selected vectors; with no `settings` the form is shown. Reports a refusal in a message box unless the
/// window is quiet; the reason is returned in `error` either way.
bool unwrapSelection(MainWindow& window, const std::optional<viewmodel::UnwrapperSettings>& settings,
                     std::string* error = nullptr);

/// Reads the 3D model file and imports it onto the active sheet of the rotary job; with no `settings` the dialog is
/// shown. Same error reporting as `unwrapSelection`.
bool importRotaryMeshFile(MainWindow& window, const std::filesystem::path& path,
                          const std::optional<viewmodel::RotaryImportSettings>& settings,
                          std::string* error = nullptr);

/// Self-test: Wrapping toggle, Vector Unwrapper and rotary import through the real dialogs; empty on success.
std::string rotaryToolsSelfTest(MainWindow& window);

}  // namespace carvetto::app
