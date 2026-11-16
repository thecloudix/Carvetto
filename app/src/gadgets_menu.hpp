#pragma once

#include <QString>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/scripting/gadget_api.hpp"
#include "carvetto/viewmodel/gadget_manager.hpp"

class QAction;
class QMenu;

namespace carvetto::app {

class MainWindow;

/// The Gadgets menu (T-0331, ADR-0108; feature-inventory section 6): the installed gadgets (built
/// from the user's gadgets folder when the window is created and again after an install or a
/// shortcut change), Install New Gadget... and Gadget Shortcuts.... Running a gadget is ONE undo
/// step; any failure leaves the drawing untouched and shows the error dialog (unless the window is
/// quiet: the report stays in `lastReport()`). Only hosts widgets: logic is `viewmodel::GadgetManager`.
class GadgetsMenu {
   public:
    explicit GadgetsMenu(MainWindow& window);
    ~GadgetsMenu();
    GadgetsMenu(const GadgetsMenu&) = delete;
    GadgetsMenu& operator=(const GadgetsMenu&) = delete;

    QMenu* menu() const { return menu_; }
    QAction* installAction() const { return installAction_; }
    QAction* shortcutsAction() const { return shortcutsAction_; }
    viewmodel::GadgetManager& manager() { return manager_; }

    /// Uses `folder` as the gadgets folder and rebuilds the menu.
    void setFolder(const std::filesystem::path& folder);
    /// Rebuilds the menu from the manager's current gadget list.
    void rebuild();
    /// Stores and restores the user's shortcut choices (QSettings); the window turns it on for real
    /// runs only, so tests never touch the user's settings.
    void setPersist(bool persist);

    /// The gadget actions of the menu, in menu order (submenus flattened).
    std::vector<QAction*> gadgetActions() const;

    /// What choosing gadget `index` does: runs it against the open document (refused while a
    /// toolpath calculation runs), refreshes the window, and shows the error dialog on failure.
    viewmodel::GadgetRunReport runGadget(std::size_t index);
    const viewmodel::GadgetRunReport& lastReport() const { return last_; }
    /// Replaces the dialogs a gadget run shows (forms, messages) with `ui`; the self-test scripts the
    /// answers. Without it the real dialogs are used.
    void setUiOverride(std::optional<scripting::GadgetUi> ui) { uiOverride_ = std::move(ui); }
    /// Limits of a gadget run (default: the host's defaults).
    void setLimits(scripting::ScriptLimits limits) { limits_ = std::move(limits); }

    /// Opens the Install New Gadget flow for `folder` (a folder picker when empty).
    std::optional<scripting::InstallResult> installGadget(const std::filesystem::path& folder = {});
    /// Opens the Gadget Shortcuts dialog (modal).
    void shortcutsDialog();

   private:
    scripting::GadgetUi realUi();
    void addItems(QMenu* into, const std::vector<viewmodel::GadgetMenuItem>& items);
    void loadOverrides();
    void storeOverrides();

    MainWindow& window_;
    QMenu* menu_{nullptr};
    QAction* installAction_{nullptr};
    QAction* shortcutsAction_{nullptr};
    std::vector<QAction*> gadgetActions_;
    viewmodel::GadgetManager manager_;
    viewmodel::GadgetRunReport last_;
    std::optional<scripting::GadgetUi> uiOverride_;
    scripting::ScriptLimits limits_;
    bool persist_{false};
};

/// The default gadgets folder: <AppData>/gadgets (created on first install).
std::filesystem::path defaultGadgetsFolder();

/// Self-test of the Gadgets menu (T-0331): installs a sample gadget folder, builds the menu, runs the
/// sample gadget (vectors and toolpaths in ONE undo step), hostile and failing gadgets, forms,
/// shortcuts and install. Returns "" or the first failure.
std::string gadgetsSelfTest(MainWindow& window);

}  // namespace carvetto::app
