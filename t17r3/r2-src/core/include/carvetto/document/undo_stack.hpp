#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "carvetto/document/command.hpp"

namespace carvetto::document {

class Document;

/// Linear undo/redo history of `Command`s. Pushing applies the command, drops any redo tail, and
/// may coalesce it into the previous command (see `Command::mergeWith`) while a coalescing run is
/// open. Undo, redo and `breakCoalescing()` close the run; a typical UI calls `breakCoalescing()`
/// on mouse release or when a form closes so one drag or one spin-box session is one undo step.
class UndoStack {
   public:
    /// `limit` = 0 means unbounded; otherwise the oldest steps are dropped beyond `limit`.
    explicit UndoStack(std::size_t limit = 0) : limit_(limit) {}

    /// Applies `cmd` to `doc` and records it. Returns false (discarding `cmd`) when `apply`
    /// reported a no-op. Inside a macro the command is collected instead of recorded.
    bool push(std::unique_ptr<Command> cmd, Document& doc);
    /// Reverts the latest applied step. Returns false when there is nothing to undo.
    bool undo(Document& doc);
    /// Re-applies the latest undone step. Returns false when there is nothing to redo.
    bool redo(Document& doc);

    bool canUndo() const { return index_ > 0 && !inMacro(); }
    bool canRedo() const { return index_ < commands_.size() && !inMacro(); }
    /// Description of the step `undo()` would revert ("" when none).
    std::string undoText() const;
    /// Description of the step `redo()` would re-apply ("" when none).
    std::string redoText() const;

    /// Starts collecting pushes into one step; nests. The macro is recorded by `endMacro()`.
    void beginMacro(std::string text);
    /// Closes the innermost macro; the outermost close records it (if it collected anything).
    void endMacro();
    bool inMacro() const { return macroDepth_ > 0; }

    /// Ends the current coalescing run so the next push starts a new undo step.
    void breakCoalescing() { coalesceOpen_ = false; }

    /// Number of recorded steps (undone ones included).
    std::size_t count() const { return commands_.size(); }
    /// Number of steps currently applied.
    std::size_t index() const { return index_; }
    std::size_t limit() const { return limit_; }
    /// Forgets all history (does not touch the document).
    void clear();

    /// Marks the current state as saved.
    void setClean();
    /// True when the document state equals the last `setClean()` point.
    bool isClean() const { return cleanIndex_ >= 0 && static_cast<std::size_t>(cleanIndex_) == index_; }

   private:
    void record(std::unique_ptr<Command> cmd);

    std::vector<std::unique_ptr<Command>> commands_;
    std::size_t index_{0};
    std::size_t limit_;
    bool coalesceOpen_{false};
    std::unique_ptr<MacroCommand> macro_;
    int macroDepth_{0};
    /// Index matching the clean state, or -1 when it is no longer reachable.
    long long cleanIndex_{0};
};

}  // namespace carvetto::document
