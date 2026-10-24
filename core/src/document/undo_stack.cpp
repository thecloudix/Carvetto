#include "carvetto/document/undo_stack.hpp"

#include <utility>

namespace carvetto::document {

// --- MacroCommand ------------------------------------------------------------------------------

bool MacroCommand::apply(Document& doc) {
    for (auto& c : children_) {
        c->apply(doc);
    }
    return !children_.empty();
}

void MacroCommand::revert(Document& doc) {
    for (auto it = children_.rbegin(); it != children_.rend(); ++it) {
        (*it)->revert(doc);
    }
}

// --- UndoStack ---------------------------------------------------------------------------------

bool UndoStack::push(std::unique_ptr<Command> cmd, Document& doc) {
    if (!cmd || !cmd->apply(doc)) {
        return false;
    }
    if (inMacro()) {
        macro_->add(std::move(cmd));
        return true;
    }
    record(std::move(cmd));
    return true;
}

void UndoStack::record(std::unique_ptr<Command> cmd) {
    // Any redo tail is discarded by a new edit.
    if (index_ < commands_.size()) {
        commands_.erase(commands_.begin() + static_cast<std::ptrdiff_t>(index_), commands_.end());
        if (cleanIndex_ > static_cast<long long>(index_)) {
            cleanIndex_ = -1;
        }
    }
    if (coalesceOpen_ && !commands_.empty() && commands_.back()->mergeWith(*cmd)) {
        // The clean point can't sit on a merged step: the document has changed since it.
        if (cleanIndex_ == static_cast<long long>(index_)) {
            cleanIndex_ = -1;
        }
        return;
    }
    commands_.push_back(std::move(cmd));
    ++index_;
    coalesceOpen_ = true;
    if (limit_ > 0 && commands_.size() > limit_) {
        commands_.erase(commands_.begin());
        --index_;
        cleanIndex_ = cleanIndex_ < 0 ? -1 : cleanIndex_ - 1;
    }
}

bool UndoStack::undo(Document& doc) {
    if (!canUndo()) {
        return false;
    }
    commands_[--index_]->revert(doc);
    coalesceOpen_ = false;
    return true;
}

bool UndoStack::redo(Document& doc) {
    if (!canRedo()) {
        return false;
    }
    commands_[index_++]->apply(doc);
    coalesceOpen_ = false;
    return true;
}

std::string UndoStack::undoText() const {
    return canUndo() ? commands_[index_ - 1]->text() : std::string{};
}

std::string UndoStack::redoText() const {
    return canRedo() ? commands_[index_]->text() : std::string{};
}

void UndoStack::beginMacro(std::string text) {
    if (macroDepth_++ == 0) {
        macro_ = std::make_unique<MacroCommand>(std::move(text));
    }
}

void UndoStack::endMacro() {
    if (macroDepth_ == 0) {
        return;
    }
    if (--macroDepth_ > 0) {
        return;
    }
    std::unique_ptr<MacroCommand> macro = std::move(macro_);
    if (macro && !macro->empty()) {
        // Children are already applied; a macro never coalesces with its predecessor.
        coalesceOpen_ = false;
        record(std::move(macro));
    }
}

void UndoStack::clear() {
    commands_.clear();
    index_ = 0;
    coalesceOpen_ = false;
    macro_.reset();
    macroDepth_ = 0;
    cleanIndex_ = 0;
}

void UndoStack::setClean() {
    cleanIndex_ = static_cast<long long>(index_);
    coalesceOpen_ = false;
}

}  // namespace carvetto::document
