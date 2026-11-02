#include "carvetto/document/undo_stack.hpp"

#include <catch2/catch_test_macros.hpp>
#include <memory>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"

using namespace carvetto::document;

namespace {

// Appends a character to a shared log; coalesces with the next AppendCommand when `mergeable`.
class AppendCommand : public Command {
   public:
    AppendCommand(std::string& log, char c, bool mergeable = false)
        : log_(log), c_(c), mergeable_(mergeable) {}
    bool apply(Document&) override {
        if (c_ == '\0') {
            return false;  // deliberate no-op
        }
        log_ += text_;
        return true;
    }
    void revert(Document&) override { log_.erase(log_.size() - text_.size()); }
    std::string text() const override { return "append " + text_; }
    bool mergeWith(const Command& next) override {
        const auto* other = dynamic_cast<const AppendCommand*>(&next);
        if (!mergeable_ || !other || !other->mergeable_) {
            return false;
        }
        text_ += other->text_;
        return true;
    }

   private:
    std::string& log_;
    char c_;
    bool mergeable_;
    std::string text_ = std::string(1, c_);
};

std::unique_ptr<AppendCommand> append(std::string& log, char c, bool mergeable = false) {
    return std::make_unique<AppendCommand>(log, c, mergeable);
}

}  // namespace

TEST_CASE("undo stack applies, undoes and redoes in order", "[document][undo]") {
    Document doc;
    UndoStack stack;
    std::string log;

    CHECK_FALSE(stack.canUndo());
    CHECK_FALSE(stack.undo(doc));
    CHECK(stack.push(append(log, 'a'), doc));
    CHECK(stack.push(append(log, 'b'), doc));
    CHECK(stack.push(append(log, 'c'), doc));
    CHECK(log == "abc");
    CHECK(stack.count() == 3);
    CHECK(stack.index() == 3);
    CHECK(stack.undoText() == "append c");
    CHECK(stack.redoText().empty());

    CHECK(stack.undo(doc));
    CHECK(log == "ab");
    CHECK(stack.redoText() == "append c");
    CHECK(stack.undo(doc));
    CHECK(log == "a");
    CHECK(stack.redo(doc));
    CHECK(log == "ab");
    CHECK(stack.redo(doc));
    CHECK(log == "abc");
    CHECK_FALSE(stack.redo(doc));
}

TEST_CASE("a new push discards the redo tail", "[document][undo]") {
    Document doc;
    UndoStack stack;
    std::string log;
    stack.push(append(log, 'a'), doc);
    stack.push(append(log, 'b'), doc);
    stack.undo(doc);
    stack.push(append(log, 'c'), doc);
    CHECK(log == "ac");
    CHECK(stack.count() == 2);
    CHECK_FALSE(stack.canRedo());
}

TEST_CASE("a no-op command is not recorded", "[document][undo]") {
    Document doc;
    UndoStack stack;
    std::string log;
    CHECK_FALSE(stack.push(append(log, '\0'), doc));
    CHECK_FALSE(stack.push(nullptr, doc));
    CHECK(stack.count() == 0);
    CHECK_FALSE(stack.canUndo());
}

TEST_CASE("mergeable commands coalesce into one step until the run is broken", "[document][undo]") {
    Document doc;
    UndoStack stack;
    std::string log;
    stack.push(append(log, 'a', true), doc);
    stack.push(append(log, 'b', true), doc);
    stack.push(append(log, 'c', true), doc);
    CHECK(log == "abc");
    CHECK(stack.count() == 1);
    CHECK(stack.undoText() == "append abc");

    stack.breakCoalescing();
    stack.push(append(log, 'd', true), doc);
    CHECK(stack.count() == 2);

    // Undo closes the run too: the redo tail is dropped and the new push stands alone.
    stack.undo(doc);
    CHECK(log == "abc");
    stack.push(append(log, 'e', true), doc);
    CHECK(stack.count() == 2);
    CHECK(log == "abce");
    stack.undo(doc);
    CHECK(log == "abc");
    stack.undo(doc);
    CHECK(log.empty());

    // Non-mergeable commands never coalesce.
    stack.push(append(log, 'x'), doc);
    stack.push(append(log, 'y'), doc);
    CHECK(stack.count() == 2);
}

TEST_CASE("macros group several pushes into one undo step", "[document][undo]") {
    Document doc;
    UndoStack stack;
    std::string log;
    stack.push(append(log, 'a'), doc);

    stack.beginMacro("Two edits");
    CHECK(stack.inMacro());
    CHECK_FALSE(stack.canUndo());
    stack.push(append(log, 'b'), doc);
    stack.beginMacro("nested");
    stack.push(append(log, 'c'), doc);
    stack.endMacro();
    CHECK(stack.inMacro());
    stack.push(append(log, '\0'), doc);  // ignored no-op
    stack.endMacro();
    CHECK_FALSE(stack.inMacro());
    CHECK(log == "abc");
    CHECK(stack.count() == 2);
    CHECK(stack.undoText() == "Two edits");

    CHECK(stack.undo(doc));
    CHECK(log == "a");
    CHECK(stack.redo(doc));
    CHECK(log == "abc");

    // An empty macro records nothing.
    stack.beginMacro("empty");
    stack.endMacro();
    CHECK(stack.count() == 2);
    stack.endMacro();  // unbalanced close is harmless
}

TEST_CASE("undo limit drops the oldest steps", "[document][undo]") {
    Document doc;
    UndoStack stack(2);
    std::string log;
    stack.push(append(log, 'a'), doc);
    stack.push(append(log, 'b'), doc);
    stack.push(append(log, 'c'), doc);
    CHECK(stack.count() == 2);
    CHECK(stack.undo(doc));
    CHECK(stack.undo(doc));
    CHECK_FALSE(stack.undo(doc));
    CHECK(log == "a");
}

TEST_CASE("clean state tracks the saved point", "[document][undo]") {
    Document doc;
    UndoStack stack;
    std::string log;
    CHECK(stack.isClean());
    stack.push(append(log, 'a'), doc);
    CHECK_FALSE(stack.isClean());
    stack.setClean();
    CHECK(stack.isClean());
    stack.push(append(log, 'b'), doc);
    CHECK_FALSE(stack.isClean());
    stack.undo(doc);
    CHECK(stack.isClean());
    stack.undo(doc);
    CHECK_FALSE(stack.isClean());
    stack.redo(doc);
    CHECK(stack.isClean());

    // Editing past an undone clean point makes it unreachable.
    stack.undo(doc);
    stack.push(append(log, 'z'), doc);
    CHECK_FALSE(stack.isClean());
    stack.undo(doc);
    CHECK_FALSE(stack.isClean());

    stack.clear();
    CHECK(stack.count() == 0);
    CHECK(stack.isClean());
}
