#pragma once

#include <cstddef>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/viewmodel/toolpath_session.hpp"

// Edit > Document Variables (T-0257, feature-inventory §1 "Document Variables"): the Qt-free model
// behind the dialog. It lists the document's variables and creates, changes, deletes, imports and
// exports them. Every edit is ONE undo step (a macro of the core's variable commands plus the
// recalculation of the toolpaths whose expressions name a changed variable); a refused edit changes
// nothing. A variable a toolpath expression (or an open toolpath form) names cannot be deleted.

namespace carvetto::viewmodel {

struct VariableRow {
    std::string name;
    double value{0.0};
    /// `value` as number text.
    std::string valueText;
    /// Number of users (toolpaths, open forms) holding the variable.
    std::size_t references{0};
};

/// Result of one edit.
struct VariableEdit {
    bool ok{false};
    /// What to show: the reason when `!ok`, else a short confirmation (with a warning when a
    /// dependent toolpath could not be recalculated).
    std::string message;
    /// Toolpaths recalculated because of the edit / that could not be.
    std::size_t recalculated{0};
    std::size_t failed{0};
    /// Deferred mode only: toolpaths hold expressions, so the caller should now run
    /// `ToolpathSession::prepareRecalculateBound` off the UI thread (the edit itself is done).
    bool recalculationPending{false};
    /// Variables created or overwritten by an import; lines it rejected (1-based).
    std::size_t imported{0};
    std::vector<std::size_t> rejectedLines;
};

class VariablesModel {
   public:
    /// `session` may be null (no toolpath bookkeeping); both must outlive the model.
    explicit VariablesModel(document::Document* doc, ToolpathSession* session = nullptr);

    /// Deferred recalculation (T-0271): an edit changes the variables only (one undo step) and sets
    /// `recalculationPending`; the caller recalculates the dependent toolpaths in the background (a
    /// second undo step). Off (default): recalculated inside the edit's undo step, on this thread.
    void setDeferRecalculation(bool defer) { defer_ = defer; }
    /// While it returns true every edit is refused ("wait ...") - a background recalculation runs.
    void setBusyCheck(std::function<bool()> busy) { busy_ = std::move(busy); }

    std::vector<VariableRow> rows() const;
    /// Names of the toolpaths whose expressions name `name`.
    std::vector<std::string> toolpathsUsing(const std::string& name) const;

    /// Creates `name` or changes its value. `valueText` is a number or a calculation (`=...`,
    /// which may name other variables); the name must follow `DocumentVariables::isValidName`.
    VariableEdit set(const std::string& name, const std::string& valueText);
    /// Deletes `name`; refused while a toolpath or an open toolpath form uses it.
    VariableEdit remove(const std::string& name);
    /// Renames `name` to `newName` (create + delete in one step). Refused when `name` is used or
    /// `newName` exists.
    VariableEdit rename(const std::string& name, const std::string& newName);
    /// Reads `Name = value` lines; same-named variables are overwritten. One undo step.
    VariableEdit importText(const std::string& text);
    VariableEdit importFile(const std::filesystem::path& path);
    /// The `Name = value` text of all variables.
    std::string exportText() const;
    VariableEdit exportFile(const std::filesystem::path& path) const;

    /// Longest import file read (bytes).
    static constexpr std::size_t kMaxImportBytes = 1u << 20;

   private:
    /// Runs `body` (variable commands) and the dependent recalculation as one undo step.
    template <class Body>
    VariableEdit edit(const char* text, Body&& body);

    document::Document* doc_;
    ToolpathSession* session_;
    bool defer_{false};
    std::function<bool()> busy_;
};

}  // namespace carvetto::viewmodel
