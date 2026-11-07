#include "carvetto/document/undo_stack.hpp"

#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "carvetto/document/commands.hpp"
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

TEST_CASE("undo stack generation never repeats for different content", "[document][undo]") {
    using namespace carvetto::document;
    Document a;
    Document b;
    const std::uint64_t g0 = a.undoStack().generation();
    CHECK(g0 != b.undoStack().generation());  // distinct stacks
    REQUIRE(a.execute(std::make_unique<AddObjectCommand>(
        Object::vector(carvetto::geometry::Contour::circle({0, 0}, 1), a.activeLayer(), a.activeSheet()))));
    const std::uint64_t g1 = a.undoStack().generation();
    REQUIRE(a.undo());
    const std::uint64_t g2 = a.undoStack().generation();
    REQUIRE(a.redo());
    const std::uint64_t g3 = a.undoStack().generation();
    for (std::uint64_t g : {g1, g2, g3}) {
        CHECK(g != g0);
    }
    CHECK(g1 != g2);
    CHECK(g2 != g3);
    CHECK(g1 != g3);  // same undo index and count as g1, different generation
    a.undoStack().clear();
    CHECK(a.undoStack().generation() != g3);
}

TEST_CASE("undo stack generation changes on coalesced pushes and pushes inside a macro", "[document][undo]") {
    using namespace carvetto::document;
    using carvetto::geometry::Transform2;
    using carvetto::geometry::Vec2;
    Document d;
    REQUIRE(d.execute(std::make_unique<AddObjectCommand>(
        Object::vector(carvetto::geometry::Contour::circle({0, 0}, 1), d.activeLayer(), d.activeSheet()))));
    const ObjectId id = d.objects().begin()->first;
    d.undoStack().breakCoalescing();

    // First drag step opens a coalescing run; the second merges into it (one undo entry).
    std::uint64_t before = d.undoStack().generation();
    REQUIRE(d.execute(std::make_unique<TransformObjectsCommand>(std::vector<ObjectId>{id},
                                                                Transform2::translation({1, 0}))));
    CHECK(d.undoStack().generation() != before);
    before = d.undoStack().generation();
    const std::size_t steps = d.undoStack().count();
    REQUIRE(d.execute(std::make_unique<TransformObjectsCommand>(std::vector<ObjectId>{id},
                                                                Transform2::translation({1, 0}))));
    CHECK(d.undoStack().count() == steps);  // it coalesced
    CHECK(d.undoStack().generation() != before);

    // Pushes inside a macro change the document before the macro is recorded.
    d.undoStack().beginMacro("two");
    before = d.undoStack().generation();
    REQUIRE(d.execute(std::make_unique<TransformObjectsCommand>(std::vector<ObjectId>{id},
                                                                Transform2::translation({0, 1}))));
    CHECK(d.undoStack().generation() != before);
    before = d.undoStack().generation();
    REQUIRE(d.execute(std::make_unique<TransformObjectsCommand>(std::vector<ObjectId>{id},
                                                                Transform2::translation({0, 1}))));
    CHECK(d.undoStack().generation() != before);
    d.undoStack().endMacro();

    // Each of undo, redo and clear moves it on again.
    before = d.undoStack().generation();
    REQUIRE(d.undo());
    CHECK(d.undoStack().generation() != before);
    before = d.undoStack().generation();
    REQUIRE(d.redo());
    CHECK(d.undoStack().generation() != before);
    before = d.undoStack().generation();
    d.undoStack().clear();
    CHECK(d.undoStack().generation() != before);
}

TEST_CASE("undoAndDiscard reverts and forgets the latest step", "[document][undo]") {
    Document doc;
    std::string log;
    UndoStack& stack = doc.undoStack();
    REQUIRE(doc.execute(std::make_unique<AppendCommand>(log, 'a')));
    stack.setClean();
    REQUIRE(doc.execute(std::make_unique<AppendCommand>(log, 'b', true)));
    REQUIRE(doc.execute(std::make_unique<AppendCommand>(log, 'c')));
    CHECK_FALSE(stack.isClean());

    const std::uint64_t before = stack.generation();
    REQUIRE(stack.undoAndDiscard(doc));
    CHECK(log == "ab");
    CHECK(stack.generation() != before);
    CHECK(stack.count() == 2);
    CHECK(stack.index() == 2);
    CHECK_FALSE(stack.canRedo());
    CHECK(stack.canUndo());
    CHECK(stack.undoText() == "append b");

    // Discarding back to the saved state is clean again; the saved step itself is not.
    REQUIRE(stack.undoAndDiscard(doc));
    CHECK(log == "a");
    CHECK(stack.isClean());
    CHECK(stack.count() == 1);
    const std::uint64_t g2 = stack.generation();
    REQUIRE(stack.undoAndDiscard(doc));
    CHECK(log.empty());
    CHECK_FALSE(stack.isClean());
    CHECK(stack.generation() != g2);
    CHECK_FALSE(stack.canUndo());
    CHECK_FALSE(stack.undoAndDiscard(doc));  // nothing left
    // A new step must not look like the saved state just because the index came back to it.
    REQUIRE(doc.execute(std::make_unique<AppendCommand>(log, 'z')));
    CHECK_FALSE(stack.isClean());
    REQUIRE(stack.undoAndDiscard(doc));

    // Earlier steps still undo and redo normally afterwards.
    REQUIRE(doc.execute(std::make_unique<AppendCommand>(log, 'x')));
    REQUIRE(doc.execute(std::make_unique<AppendCommand>(log, 'y')));
    REQUIRE(stack.undo(doc));
    REQUIRE(stack.redo(doc));
    CHECK(log == "xy");
    CHECK(stack.undoText() == "append y");
}

TEST_CASE("undoAndDiscard: generations stay unique and the next push does not coalesce", "[document][undo]") {
    Document a;
    Document b;
    std::string logA;
    std::string logB;
    REQUIRE(a.execute(std::make_unique<AppendCommand>(logA, 'p', true)));
    REQUIRE(b.execute(std::make_unique<AppendCommand>(logB, 'p', true)));
    REQUIRE(a.execute(std::make_unique<AppendCommand>(logA, 'q', true)));  // coalesced into 'p'
    CHECK(a.undoStack().count() == 1);
    REQUIRE(a.undoStack().undoAndDiscard(a));
    CHECK(logA.empty());
    const std::uint64_t afterDiscard = a.undoStack().generation();
    CHECK(afterDiscard != b.undoStack().generation());
    // A mergeable push right after a discard starts a new step (it must not merge into the
    // discarded one, nor into anything older).
    REQUIRE(a.execute(std::make_unique<AppendCommand>(logA, 'r', true)));
    REQUIRE(a.execute(std::make_unique<AppendCommand>(logA, 's', true)));
    CHECK(a.undoStack().count() == 1);
    CHECK(a.undoStack().generation() != afterDiscard);
    REQUIRE(a.undoStack().undoAndDiscard(a));
    REQUIRE(a.execute(std::make_unique<AppendCommand>(logA, 't', true)));
    REQUIRE(a.execute(std::make_unique<AppendCommand>(logA, 'u')));
    REQUIRE(a.undoStack().undoAndDiscard(a));
    REQUIRE(a.execute(std::make_unique<AppendCommand>(logA, 'v', true)));
    CHECK(a.undoStack().count() == 2);  // 't' stays its own step: 'v' did not merge into it
}

TEST_CASE("undoAndDiscard refuses with a redo tail or inside a macro", "[document][undo]") {
    Document doc;
    std::string log;
    UndoStack& stack = doc.undoStack();
    REQUIRE(doc.execute(std::make_unique<AppendCommand>(log, 'a')));
    REQUIRE(doc.execute(std::make_unique<AppendCommand>(log, 'b')));
    REQUIRE(stack.undo(doc));
    const std::uint64_t g = stack.generation();
    CHECK_FALSE(stack.undoAndDiscard(doc));
    CHECK(log == "a");
    CHECK(stack.count() == 2);
    CHECK(stack.index() == 1);
    CHECK(stack.canRedo());
    CHECK(stack.generation() == g);

    REQUIRE(stack.redo(doc));
    stack.beginMacro("m");
    REQUIRE(doc.execute(std::make_unique<AppendCommand>(log, 'c')));
    const std::uint64_t inMacro = stack.generation();
    CHECK_FALSE(stack.undoAndDiscard(doc));
    CHECK(log == "abc");
    CHECK(stack.generation() == inMacro);
    stack.endMacro();
    CHECK(stack.count() == 3);
    CHECK(stack.undoAndDiscard(doc));
    CHECK(log == "ab");
}
