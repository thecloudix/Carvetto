#pragma once

#include <QDialog>
#include <QString>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "carvetto/scripting/gadget_api.hpp"
#include "carvetto/viewmodel/gadget_manager.hpp"

class QComboBox;
class QLabel;
class QPushButton;
class QVBoxLayout;
class QWidget;

namespace carvetto::app {

/// Layers and tools a gadget form may offer (`layer` and `tool` fields).
struct GadgetFormChoices {
    std::vector<std::pair<std::uint64_t, std::string>> layers;  ///< id, name
    std::vector<std::pair<std::string, std::string>> tools;     ///< id, name
};

/// The modal input form of `carvetto.ui.form` (T-0331): one widget per field of the gadget's
/// `FormSpec` (spin box, line edit, drop-down, check box, layer and tool drop-downs). Values are
/// handed back in the types the script was promised; OK is the only way to get them.
class GadgetFormDialog : public QDialog {
   public:
    GadgetFormDialog(const scripting::FormSpec& spec, const GadgetFormChoices& choices, QWidget* parent = nullptr);

    /// Called when the user presses "Abort gadget" (the dialog then closes as cancelled).
    std::function<void()> onAbort;
    /// The "Abort gadget" button (scripted entry for the self-test).
    QPushButton* abortButton() const { return abort_; }

    /// Name -> value of every field, as the spec promised (numbers as double, layers as the layer id).
    scripting::ScriptValue::Map values() const;
    /// Scripted entry for the self-test: sets the widget of field `name`; false for an unknown name
    /// or a value of the wrong type.
    bool setValue(const std::string& name, const scripting::ScriptValue& value);

   private:
    struct Row {
        scripting::FormField field;
        QWidget* widget{nullptr};
    };
    std::vector<Row> rows_;
    QPushButton* abort_{nullptr};
};

/// Shows the error dialog of a failed run: title, summary, and the script's message and traceback
/// under "Show Details".
void showGadgetError(QWidget* parent, const viewmodel::GadgetErrorReport& report);

/// Gadget Shortcuts (T-0331): every gadget with a drop-down of Ctrl+F-keys; a choice another gadget
/// owns is refused inline and nothing changes. `onChanged` runs after an accepted change (the
/// window rebuilds the menu and stores the choices).
class GadgetShortcutsDialog : public QDialog {
   public:
    GadgetShortcutsDialog(viewmodel::GadgetManager& manager, QWidget* parent = nullptr);

    std::function<void()> onChanged;
    /// Scripted entry: picks `shortcut` ("" = none) for gadget row `row`. Returns "" or the refusal.
    std::string setShortcut(std::size_t row, const std::string& shortcut);
    /// The text of the inline message.
    QString message() const;
    std::size_t rowCount() const { return combos_.size(); }
    /// The shortcut shown for row `row` ("" = none).
    std::string shortcutOf(std::size_t row) const;

   private:
    void reload();
    viewmodel::GadgetManager& manager_;
    std::vector<QComboBox*> combos_;
    QLabel* message_{nullptr};
    QWidget* body_{nullptr};
    QVBoxLayout* top_{nullptr};
    bool loading_{false};
};

/// Install New Gadget: shows the manifest found in `folder` and asks to install. Returns the
/// manager's result, or nullopt when the user declined or the folder is no gadget (then the problem
/// was shown). `ask` replaces the confirmation boxes in the self-test (true = go on).
std::optional<scripting::InstallResult> installGadgetDialog(QWidget* parent, viewmodel::GadgetManager& manager,
                                                            const std::filesystem::path& folder,
                                                            const std::function<bool(const QString& text)>& ask = {});

}  // namespace carvetto::app
