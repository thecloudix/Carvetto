#pragma once

#include <atomic>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/scripting/gadget_api.hpp"
#include "carvetto/scripting/gadget_manifest.hpp"
#include "carvetto/viewmodel/toolpath_session.hpp"

// The gadget manager of the app (T-0331, ADR-0108): Qt-free logic behind the Gadgets menu, Install
// New Gadget, Gadget Shortcuts and the error dialog. It scans the user's gadgets folder, builds the
// menu model, runs a gadget against the document (ONE undo step, rolled back on any failure) with
// the app's tool database and toolpath session, and words failures for the dialog.
namespace carvetto::viewmodel {

/// One entry of the Gadgets menu: a submenu (`gadget` < 0, children set) or a gadget.
struct GadgetMenuItem {
    std::string label;
    /// "Ctrl+F5" or empty.
    std::string shortcut;
    /// Index into `GadgetManager::gadgets()`, or -1 for a submenu.
    int gadget{-1};
    /// False for a gadget that cannot run; `tooltip` says why.
    bool enabled{true};
    std::string tooltip;
    std::vector<GadgetMenuItem> children;
};

/// What the error dialog shows.
struct GadgetErrorReport {
    std::string title;
    /// One or two sentences a user can act on.
    std::string summary;
    /// The script's own message with chunk and line, and the traceback; may be empty.
    std::string detail;
    scripting::ScriptStatus status{scripting::ScriptStatus::RuntimeError};
    /// Line in `gadget.lua`, 0 if unknown.
    int line{0};
};

/// A manifest menu string as menu text: every '&' doubled so Qt shows it instead of reading it as a
/// mnemonic marker ("&Hack" must show as "&Hack", not underline the H).
std::string escapeMenuMnemonics(const std::string& label);

/// Words a failed run for the dialog. A Timeout is not a script bug: it says the gadget took too long
/// on this machine (ADR-0102 question 4). Always states that the drawing was not changed.
GadgetErrorReport describeFailure(const scripting::GadgetRunResult& run, const std::string& gadgetName);

/// Outcome of `GadgetManager::run`.
struct GadgetRunReport {
    bool ok{false};
    scripting::GadgetRunResult run;
    /// Set when `!ok`.
    std::optional<GadgetErrorReport> error;
    /// One line for the status bar when ok ("Weave creator: 12 objects, 1 toolpath").
    std::string summary;
};

/// The services of a gadget run bound to the app: tool database, toolpath calculation through the
/// `ToolpathSession`, snapshots for custom toolpaths, and `ui` as given. `session` must be bound to
/// `doc` and hold the tool database.
scripting::GadgetServices makeGadgetServices(document::Document& doc, ToolpathSession& session, scripting::GadgetUi ui);

class GadgetManager {
   public:
    explicit GadgetManager(std::filesystem::path folder);

    const std::filesystem::path& folder() const { return folder_; }
    /// Scans the folder again (after an install, or on request).
    void rescan();
    const std::vector<scripting::GadgetInfo>& gadgets() const { return discovery_.gadgets; }
    /// Folders skipped because their manifest was unusable, "folder: reason".
    const std::vector<std::string>& skipped() const { return discovery_.skipped; }

    /// The Gadgets menu: gadgets grouped by their manifest's menu path, entries sorted by label
    /// (case-insensitive, then folder name), submenus before gadgets at each level.
    std::vector<GadgetMenuItem> menu() const;
    /// The gadget a shortcut ("Ctrl+F5") belongs to; only runnable gadgets own shortcuts.
    std::optional<std::size_t> byShortcut(const std::string& shortcut) const;
    /// Gadget Shortcuts dialog rows: every gadget with its effective shortcut ("" = none) and notes.
    struct ShortcutRow {
        std::size_t gadget{0};
        std::string name;
        std::string shortcut;
        std::string note;
    };
    std::vector<ShortcutRow> shortcuts() const;

    /// Gives gadget `index` the shortcut `shortcut` ("Ctrl+F5", any case; "" = none), as the Gadget
    /// Shortcuts dialog does. Returns "" or why not: not a Ctrl+F1..F12 shortcut (optionally with
    /// Shift), or another gadget has it. Rescans.
    std::string assignShortcut(std::size_t index, const std::string& shortcut);
    /// The user's choices by gadget folder name, for saving; `setOverrides` restores them (before
    /// the first scan, or followed by `rescan`).
    const scripting::ShortcutOverrides& overrides() const { return overrides_; }
    void setOverrides(scripting::ShortcutOverrides overrides) { overrides_ = std::move(overrides); }

    /// What the Install dialog shows before the user confirms.
    scripting::InstallResult inspect(const std::filesystem::path& folder) const;
    /// Copies the gadget into the gadgets folder and rescans.
    scripting::InstallResult install(const std::filesystem::path& folder, bool replace = false);

    /// Runs gadget `index` against `doc`. The script is read from disk now (an edited script runs
    /// as saved). Nothing changes in `doc` unless the run succeeds; success is ONE undo step named
    /// after the gadget.
    GadgetRunReport run(std::size_t index, document::Document& doc, ToolpathSession& session, scripting::GadgetUi ui,
                        scripting::ScriptLimits limits = {}, const std::atomic<bool>* cancel = nullptr) const;
    /// Runs script text directly (tests and the self-test): same rules, undo text `name`.
    static GadgetRunReport runSource(const std::string& source, const std::string& name, document::Document& doc,
                                     ToolpathSession& session, scripting::GadgetUi ui,
                                     scripting::ScriptLimits limits = {}, const std::atomic<bool>* cancel = nullptr);

   private:
    std::filesystem::path folder_;
    scripting::GadgetDiscovery discovery_;
    scripting::ShortcutOverrides overrides_;
};

}  // namespace carvetto::viewmodel
