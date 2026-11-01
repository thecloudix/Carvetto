#pragma once

#include <memory>
#include <string>
#include <vector>

namespace carvetto::document {

class Document;

/// One undoable edit. `apply` performs it (returning false when it had nothing to do, in which
/// case the stack discards it), `revert` restores the exact prior state. A command is applied
/// again on redo, so it must keep whatever it needs (allocated ids, removed objects) to reproduce
/// the same result.
class Command {
   public:
    virtual ~Command() = default;

    /// Performs the edit. Returns false when the command is a no-op and should not be recorded.
    virtual bool apply(Document& doc) = 0;
    /// Undoes the edit; only called after a successful `apply`.
    virtual void revert(Document& doc) = 0;
    /// Short user-facing description, e.g. "Move objects".
    virtual std::string text() const = 0;

    /// Coalescing hook: absorb `next` (already applied to the document) so that one undo reverts
    /// both edits. Return false to leave `next` as its own undo step. The default never merges.
    virtual bool mergeWith(const Command& next) {
        (void)next;
        return false;
    }
};

/// A sequence of commands undone and redone as one step.
class MacroCommand : public Command {
   public:
    explicit MacroCommand(std::string text) : text_(std::move(text)) {}

    void add(std::unique_ptr<Command> cmd) { children_.push_back(std::move(cmd)); }
    bool empty() const { return children_.empty(); }
    std::size_t size() const { return children_.size(); }

    /// Applies children in order; returns false only when the macro is empty.
    bool apply(Document& doc) override;
    /// Reverts children in reverse order.
    void revert(Document& doc) override;
    std::string text() const override { return text_; }

   private:
    std::string text_;
    std::vector<std::unique_ptr<Command>> children_;
};

}  // namespace carvetto::document
