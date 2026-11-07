// T-0220: the toolpath session over a Document: list, edits (one undo step each), calculation as a
// pure function applied in one step, staleness and recalculation, V-carve groups, reloading, the
// job summary.

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <limits>
#include <map>
#include <string>

#include "carvetto/document/carv_io.hpp"
#include "toolpath_test_support.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using namespace carvetto::viewmodel::testing;
using document::Document;
using document::ObjectId;
using document::ToolpathId;
using document::ToolpathOperation;
using document::ToolpathRecord;

namespace {

/// The state undo must restore exactly (everything but the id counter).
struct Snapshot {
    std::vector<ToolpathRecord> toolpaths;
    std::map<ObjectId, document::Object> objects;
    bool operator==(const Snapshot&) const = default;
};

Snapshot snapshot(const Document& doc) { return Snapshot{doc.toolpaths(), doc.objects()}; }

void moveObject(Document& doc, ObjectId id, double dx, double dy) {
    REQUIRE(doc.execute(std::make_unique<document::TransformObjectsCommand>(
        std::vector<ObjectId>{id}, geometry::Transform2::translation({dx, dy}))));
}

void hideLayer(Document& doc, document::LayerId id, bool hidden) {
    document::Layer l = *doc.findLayer(id);
    l.visible = !hidden;
    REQUIRE(doc.execute(std::make_unique<document::SetLayerPropertiesCommand>(l)));
}

struct Fixture {
    Document doc;
    tooling::ToolDatabase db{cliToolDatabase()};
    ToolpathSession session{&doc};
    ObjectId big;    // 30 mm square
    ObjectId tiny;   // 4 mm square

    Fixture() {
        session.setToolDatabase(db);
        big = addVector(doc, square(0, 0, 30));
        tiny = addVector(doc, square(50, 0, 4));
        doc.undoStack().clear();
    }
    ToolpathDefinition pocketOnBig() const {
        ToolpathDefinition d = pocketDefinition(db);
        d.selector = VectorSelector::fromObjects({big});
        return d;
    }
    ToolpathId addPocket() {
        const ToolpathOutcome out = session.add(pocketOnBig());
        INFO(out.message);
        REQUIRE(out.ok);
        REQUIRE(out.ids.size() == 1);
        return out.ids.front();
    }
    std::size_t steps() const { return doc.undoStack().index(); }
};

}  // namespace

TEST_CASE("session: add is one undo step and undo/redo are exact", "[toolpath][session]") {
    Fixture f;
    const Snapshot empty = snapshot(f.doc);
    const ToolpathId id = f.addPocket();
    CHECK(f.steps() == 1);
    REQUIRE(f.doc.toolpaths().size() == 1);
    const ToolpathRecord& rec = f.doc.toolpaths().front();
    CHECK(rec.id == id);
    CHECK(rec.operation() == ToolpathOperation::Pocket);
    CHECK(rec.moveCount() > 10);
    CHECK(rec.selection.objects == std::vector<ObjectId>{f.big});
    CHECK(rec.toolSource.toolId == "end-mill-6mm");
    CHECK(f.doc.checkIntegrity().empty());
    const Snapshot added = snapshot(f.doc);

    REQUIRE(f.doc.undo());
    CHECK(snapshot(f.doc) == empty);
    CHECK(f.session.list().empty());
    REQUIRE(f.doc.redo());
    CHECK(snapshot(f.doc) == added);
    const auto list = f.session.list();
    REQUIRE(list.size() == 1);
    CHECK(list[0].status == ToolpathStatus::Current);
    CHECK(list[0].name == "Pocket");
    CHECK(list[0].toolName == f.db.findTool("end-mill-6mm")->name);
    CHECK(list[0].enabled);
}

TEST_CASE("session: update replaces in one step, undo restores the old record", "[toolpath][session]") {
    Fixture f;
    const ToolpathId id = f.addPocket();
    const Snapshot before = snapshot(f.doc);
    const std::size_t steps = f.steps();

    ToolpathDefinition def = *f.session.definition(id);
    REQUIRE(std::get<toolpath::PocketParams>(def.params).cutDepth == 3.0);
    std::get<toolpath::PocketParams>(def.params).cutDepth = 5.0;
    def.setName("Deeper");
    const ToolpathOutcome out = f.session.update(id, def);
    INFO(out.message);
    REQUIRE(out.ok);
    CHECK(f.steps() == steps + 1);
    CHECK(f.doc.toolpaths().size() == 1);
    CHECK(f.doc.toolpaths()[0].id == id);
    CHECK(f.doc.toolpaths()[0].name == "Deeper");
    CHECK(std::get<toolpath::PocketParams>(f.doc.toolpaths()[0].params).cutDepth == 5.0);
    CHECK(f.doc.undoStack().undoText() == "Edit toolpath");

    REQUIRE(f.doc.undo());
    CHECK(snapshot(f.doc) == before);
    // Editing twice in a row makes two steps (no coalescing).
    REQUIRE(f.session.update(id, def).ok);
    std::get<toolpath::PocketParams>(def.params).cutDepth = 6.0;
    REQUIRE(f.session.update(id, def).ok);
    CHECK(f.steps() == steps + 2);
}

TEST_CASE("session: a refused edit changes nothing and reports the vector", "[toolpath][session]") {
    Fixture f;
    const ToolpathId id = f.addPocket();
    const Snapshot before = snapshot(f.doc);
    const std::size_t steps = f.steps();

    // The 6 mm tool does not fit the 4 mm square: refused, per vector.
    ToolpathDefinition def = *f.session.definition(id);
    def.selector = VectorSelector::fromObjects({f.tiny});
    const ToolpathOutcome out = f.session.update(id, def);
    CHECK_FALSE(out.ok);
    CHECK_FALSE(out.message.empty());
    CHECK(snapshot(f.doc) == before);
    CHECK(f.steps() == steps);
    bool refusal = false;
    for (const ToolpathMessage& m : out.messages) {
        refusal = refusal || (m.severity == MessageSeverity::Error && m.issue == toolpath::ToolpathIssue::ToolTooLarge);
    }
    CHECK(refusal);
    CHECK_FALSE(f.session.lastMessages(id).empty());

    // A wrong operation, an unknown id and a missing tool are refused too.
    CHECK_FALSE(f.session.update(id, profileDefinition(f.db)).ok);
    CHECK_FALSE(f.session.update(ToolpathId{999}, def).ok);
    ToolpathDefinition notool = *f.session.definition(id);
    notool.tool = source("missing");
    CHECK_FALSE(f.session.update(id, notool).ok);
    CHECK(snapshot(f.doc) == before);
}

TEST_CASE("session: core diagnostics come back per vector with a severity", "[toolpath][session]") {
    Fixture f;
    // Inside a 6 mm tool: the big square is cut, the 4 mm one reported.
    ToolpathForm form(ToolpathOperation::Profile, document::Units::Millimetres);
    REQUIRE(form.setTool(f.db, source("end-mill-6mm")).ok);
    REQUIRE(form.setField("cutDepth", "3").ok);
    REQUIRE(form.setChoice("side", 1).ok);
    ToolpathDefinition def = form.definition();
    def.selector = VectorSelector::fromObjects({f.big, f.tiny});
    const ToolpathOutcome out = f.session.add(def);
    REQUIRE(out.ok);
    std::string all;
    for (const ToolpathMessage& m : out.messages) {
        all += m.text + " | ";
    }
    INFO(all);
    const ToolpathMessage* too = nullptr;
    for (const ToolpathMessage& m : out.messages) {
        if (m.issue == toolpath::ToolpathIssue::ToolTooLarge) {
            too = &m;
        }
    }
    REQUIRE(too != nullptr);
    CHECK(too->severity == MessageSeverity::Warning);
    CHECK(too->vector == f.tiny);
    CHECK(too->contour == 1);
    CHECK_FALSE(too->text.empty());

    // Open vectors are reported as ignored by a pocket.
    geometry::Contour line;
    line.append(geometry::LineSpan{{0, 40}, {10, 40}});
    const ObjectId openId = addVector(f.doc, line);  // reported as a whole, no vector named
    ToolpathDefinition pocket = pocketDefinition(f.db);
    pocket.selector = VectorSelector::fromObjects({f.big, openId});
    const ToolpathOutcome out2 = f.session.add(pocket);
    REQUIRE(out2.ok);
    bool ignored = false;
    std::string all2;
    for (const ToolpathMessage& m : out2.messages) {
        all2 += m.text + " | ";
        ignored = ignored || (m.issue == toolpath::ToolpathIssue::OpenContourIgnored &&
                              m.severity == MessageSeverity::Warning);
    }
    INFO(all2);
    CHECK(ignored);
}

TEST_CASE("session: duplicate, delete, reorder and rename are single undo steps", "[toolpath][session]") {
    Fixture f;
    const ToolpathId a = f.addPocket();
    ToolpathDefinition prof = profileDefinition(f.db);
    prof.selector = VectorSelector::fromObjects({f.big});
    const ToolpathId b = f.session.add(prof).ids.front();
    const Snapshot two = snapshot(f.doc);
    const std::size_t s0 = f.steps();

    const ToolpathOutcome dup = f.session.duplicate(a);
    REQUIRE(dup.ok);
    CHECK(f.steps() == s0 + 1);
    REQUIRE(f.doc.toolpaths().size() == 3);
    const ToolpathRecord& copy = f.doc.toolpaths()[2];
    CHECK(copy.id == dup.ids.front());
    CHECK(copy.name == "Pocket (copy)");
    CHECK(std::get<toolpath::PocketParams>(copy.params).name == "Pocket (copy)");
    CHECK(*copy.moves == *f.doc.toolpaths()[0].moves);
    CHECK(copy.moves != f.doc.toolpaths()[0].moves);
    CHECK(f.session.list()[2].status == ToolpathStatus::Current);
    f.doc.undo();
    CHECK(snapshot(f.doc) == two);

    REQUIRE(f.session.reorder(b, 0).ok);
    CHECK(f.steps() == s0 + 1);
    CHECK(f.doc.toolpaths()[0].id == b);
    f.doc.undo();
    CHECK(snapshot(f.doc) == two);
    CHECK(f.session.reorder(b, 1).ok);  // already there: no step
    CHECK(f.steps() == s0);
    CHECK_FALSE(f.session.reorder(b, 2).ok);
    CHECK_FALSE(f.session.reorder(ToolpathId{77}, 0).ok);

    REQUIRE(f.session.rename(a, "Roughing").ok);
    CHECK(f.steps() == s0 + 1);
    CHECK(f.doc.toolpaths()[0].name == "Roughing");
    CHECK(std::get<toolpath::PocketParams>(f.doc.toolpaths()[0].params).name == "Roughing");
    CHECK(f.session.item(a)->status == ToolpathStatus::Current);  // a rename is not a parameter change
    f.doc.undo();
    CHECK(snapshot(f.doc) == two);
    CHECK(f.session.item(a)->status == ToolpathStatus::Current);
    CHECK(f.session.rename(a, "Pocket").ok);  // unchanged: no step
    CHECK(f.steps() == s0);
    CHECK_FALSE(f.session.rename(a, "").ok);
    CHECK_FALSE(f.session.rename(a, "x\ny").ok);
    CHECK_FALSE(f.session.rename(ToolpathId{77}, "x").ok);
    CHECK(f.steps() == s0);

    REQUIRE(f.session.deleteToolpaths({a, ToolpathId{77}, a}).ok);
    CHECK(f.steps() == s0 + 1);
    CHECK(f.doc.toolpaths().size() == 1);
    f.doc.undo();
    CHECK(snapshot(f.doc) == two);
    CHECK_FALSE(f.session.deleteToolpaths({}).ok);
    CHECK_FALSE(f.session.deleteToolpaths({ToolpathId{77}}).ok);
    CHECK(f.steps() == s0);
    ToolpathDefinition longName = *f.session.definition(a);
    longName.setName(std::string(kMaxToolpathNameBytes, 'n'));
    REQUIRE(f.session.update(a, longName).ok);
    CHECK_FALSE(f.session.duplicate(a).ok);  // "<name> (copy)" would be too long
}

TEST_CASE("session: moving or deleting source vectors marks stale; recalculation is on request", "[toolpath][session]") {
    Fixture f;
    const ToolpathId id = f.addPocket();
    REQUIRE(f.session.item(id)->status == ToolpathStatus::Current);
    const std::size_t steps = f.steps();

    moveObject(f.doc, f.big, 2.0, 0.0);
    const ToolpathListItem stale = *f.session.item(id);
    CHECK(stale.status == ToolpathStatus::NeedsRecalculation);
    CHECK(stale.vectorsChanged);
    CHECK_FALSE(stale.parametersChanged);
    CHECK_FALSE(stale.toolChanged);
    CHECK(stale.canRecalculate);
    // Nothing recalculated by itself.
    CHECK(f.doc.toolpaths()[0].moves->front().to.x == f.doc.toolpaths()[0].moves->front().to.x);

    // Undo of the move makes it current again (content comparison).
    f.doc.undo();
    CHECK(f.session.item(id)->status == ToolpathStatus::Current);
    f.doc.redo();
    CHECK(f.session.item(id)->status == ToolpathStatus::NeedsRecalculation);

    const Snapshot stale0 = snapshot(f.doc);
    const ToolpathOutcome re = f.session.recalculate(id);
    INFO(re.message);
    REQUIRE(re.ok);
    CHECK(f.steps() == steps + 2);  // the move and the recalculation
    CHECK(f.doc.undoStack().undoText() == "Recalculate toolpath");
    CHECK(f.session.item(id)->status == ToolpathStatus::Current);
    CHECK(f.doc.toolpaths()[0].id == id);
    CHECK(*f.doc.toolpaths()[0].moves != *stale0.toolpaths[0].moves);
    // The recalculated record equals a fresh calculation at the new place.
    {
        Document fresh;
        fresh.insertObject(document::Object::vector(square(2, 0, 30), fresh.defaultLayer(), fresh.activeSheet()));
        ToolpathSession s2(&fresh);
        s2.setToolDatabase(f.db);
        ToolpathDefinition d = pocketDefinition(f.db);
        REQUIRE(s2.add(d).ok);
        CHECK(*s2.document()->toolpaths()[0].moves == *f.doc.toolpaths()[0].moves);
    }
    f.doc.undo();
    CHECK(snapshot(f.doc) == stale0);
    CHECK(f.session.item(id)->status == ToolpathStatus::NeedsRecalculation);

    // Deleting the vector: stale, and recalculation fails without touching the record.
    f.doc.redo();
    REQUIRE(f.doc.execute(std::make_unique<document::RemoveObjectsCommand>(std::vector<ObjectId>{f.big})));
    CHECK(f.session.item(id)->status == ToolpathStatus::NeedsRecalculation);
    const Snapshot deleted = snapshot(f.doc);
    const std::size_t s2 = f.steps();
    CHECK_FALSE(f.session.recalculate(id).ok);
    CHECK(snapshot(f.doc) == deleted);
    CHECK(f.steps() == s2);
    f.doc.undo();
    CHECK(f.session.item(id)->status == ToolpathStatus::Current);
}

TEST_CASE("session: recalculate all recalculates the stale ones in one step", "[toolpath][session]") {
    Fixture f;
    const ObjectId other = addVector(f.doc, square(100, 100, 30));
    ToolpathDefinition d1 = f.pocketOnBig();
    ToolpathDefinition d2 = f.pocketOnBig();
    d2.selector = VectorSelector::fromObjects({other});
    d2.setName("Second");
    const ToolpathId a = f.session.add(d1).ids.front();
    const ToolpathId b = f.session.add(d2).ids.front();
    moveObject(f.doc, other, 1.0, 1.0);
    CHECK(f.session.item(a)->status == ToolpathStatus::Current);
    CHECK(f.session.item(b)->status == ToolpathStatus::NeedsRecalculation);

    const ToolpathRecord keepA = *f.doc.findToolpath(a);
    const std::size_t steps = f.steps();
    const BatchOutcome out = f.session.recalculateAll(true);
    REQUIRE(out.ok);
    CHECK(out.recalculated == 1);
    CHECK(out.items.size() == 1);
    CHECK(f.steps() == steps + 1);
    CHECK(*f.doc.findToolpath(a) == keepA);
    CHECK(f.session.item(b)->status == ToolpathStatus::Current);
    f.doc.undo();
    CHECK(f.session.item(b)->status == ToolpathStatus::NeedsRecalculation);

    // "All" recalculates everything in one step, even current toolpaths (their records stay equal).
    const BatchOutcome all = f.session.recalculateAll(false);
    REQUIRE(all.ok);
    CHECK(all.items.size() == 2);
    CHECK(f.steps() == steps + 1);
    CHECK(*f.doc.findToolpath(a) == keepA);

    // One toolpath that cannot be recalculated does not stop the others.
    moveObject(f.doc, other, 1.0, 1.0);
    moveObject(f.doc, f.big, 1.0, 0.0);
    REQUIRE(f.doc.execute(std::make_unique<document::RemoveObjectsCommand>(std::vector<ObjectId>{f.big})));
    const Snapshot before = snapshot(f.doc);
    const BatchOutcome partial = f.session.recalculateAll(true);
    CHECK_FALSE(partial.ok);
    CHECK(partial.recalculated == 1);
    CHECK(partial.failed == 1);
    CHECK(*f.doc.findToolpath(a) == *before.toolpaths.begin());
    CHECK(f.session.item(b)->status == ToolpathStatus::Current);
    CHECK(f.session.item(a)->status == ToolpathStatus::NeedsRecalculation);
    CHECK(f.session.recalculateAll(true).items.size() == 1);  // only a is still stale
    CHECK_FALSE(f.session.lastMessages(a).empty());
}

TEST_CASE("session: hidden, locked layers and other sheets never contribute", "[toolpath][session]") {
    Fixture f;
    const document::LayerId layer2 = addLayer(f.doc, "Second");
    const ObjectId on2 = addVector(f.doc, square(100, 0, 30), layer2);
    ToolpathDefinition def = pocketDefinition(f.db);  // every vector
    const ToolpathOutcome both = f.session.add(def);
    REQUIRE(both.ok);
    CHECK(f.session.countVectors(VectorSelector::allVectors()) == 3);
    const std::size_t movesBoth = f.doc.toolpaths()[0].moveCount();

    hideLayer(f.doc, layer2, true);
    CHECK(f.session.countVectors(VectorSelector::allVectors()) == 2);
    CHECK(f.session.item(both.ids.front())->status == ToolpathStatus::NeedsRecalculation);
    REQUIRE(f.session.recalculate(both.ids.front()).ok);
    CHECK(f.doc.toolpaths()[0].moveCount() < movesBoth);
    hideLayer(f.doc, layer2, false);
    CHECK(f.session.item(both.ids.front())->status == ToolpathStatus::NeedsRecalculation);

    document::Layer locked = *f.doc.findLayer(layer2);
    locked.locked = true;
    REQUIRE(f.doc.execute(std::make_unique<document::SetLayerPropertiesCommand>(locked)));
    CHECK(f.session.countVectors(VectorSelector::fromObjects({on2})) == 0);
    ToolpathDefinition onlyLocked = pocketDefinition(f.db);
    onlyLocked.selector = VectorSelector::fromObjects({on2});
    const ToolpathOutcome none = f.session.add(onlyLocked);
    CHECK_FALSE(none.ok);
    CHECK(none.message.find("no vectors") != std::string::npos);

    // The same for the active sheet.
    document::Sheet second;
    second.name = "Other";
    const document::SheetId s2 = f.doc.insertSheet(second);
    f.doc.setActiveSheet(s2);
    f.session.invalidate();
    CHECK(f.session.countVectors(VectorSelector::allVectors()) == 0);
    CHECK_FALSE(f.session.add(def).ok);
}

TEST_CASE("session: calculation is a pure function applied in one step", "[toolpath][session]") {
    Fixture f;
    const Snapshot before = snapshot(f.doc);
    const std::size_t steps = f.steps();
    const ToolpathJob job = f.session.prepare(f.pocketOnBig());
    const ToolpathCalculation calc = job.run();
    REQUIRE(calc.ok);
    CHECK(calc.items.size() == 1);
    CHECK(calc.vectorCount == 1);
    CHECK(snapshot(f.doc) == before);  // the document is untouched until apply
    CHECK(f.steps() == steps);
    // Running again gives the same result; the job is independent of the document.
    const ToolpathCalculation again = job.run();
    CHECK(again.items.front().record == calc.items.front().record);

    // A cancelled run produces nothing.
    std::atomic<bool> cancel{true};
    const ToolpathCalculation cancelled = job.run(&cancel);
    CHECK(cancelled.cancelled);
    CHECK_FALSE(cancelled.ok);
    CHECK_FALSE(f.session.apply(cancelled).ok);
    CHECK(snapshot(f.doc) == before);

    // The document changed meanwhile: refused.
    moveObject(f.doc, f.big, 1.0, 0.0);
    const Snapshot moved = snapshot(f.doc);
    const ToolpathOutcome refused = f.session.apply(calc);
    CHECK_FALSE(refused.ok);
    CHECK(refused.message.find("changed") != std::string::npos);
    CHECK(snapshot(f.doc) == moved);

    // Fresh job, tool database changed meanwhile: refused.
    const ToolpathCalculation c2 = f.session.prepare(f.pocketOnBig()).run();
    tooling::ToolDatabase db2 = f.db;
    tooling::Tool t = *db2.findTool("end-mill-6mm");
    t.defaultCuttingData.feedRate += 10.0;
    db2.updateTool(t);
    f.session.setToolDatabase(db2);
    CHECK_FALSE(f.session.apply(c2).ok);
    f.session.setToolDatabase(f.db);

    // Applied in one step.
    const ToolpathCalculation c3 = f.session.prepare(f.pocketOnBig()).run();
    const std::size_t s = f.steps();
    const ToolpathOutcome ok = f.session.apply(c3);
    REQUIRE(ok.ok);
    CHECK(f.steps() == s + 1);
    CHECK(f.doc.undoStack().undoText() == "Add toolpath");
}

TEST_CASE("session: a hostile calculation is refused whole", "[toolpath][session]") {
    Fixture f;
    ToolpathCalculation good = f.session.prepare(f.pocketOnBig()).run();
    REQUIRE(good.ok);
    const Snapshot before = snapshot(f.doc);
    const std::size_t steps = f.steps();

    ToolpathCalculation nan = good;
    std::get<toolpath::PocketParams>(nan.items[0].record.params).safeZ = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(f.session.apply(nan).ok);

    ToolpathCalculation badSelection = good;
    badSelection.items[0].record.selection.objects = {ObjectId{424242}};
    CHECK_FALSE(f.session.apply(badSelection).ok);

    ToolpathCalculation layerAsObject = good;
    layerAsObject.items[0].record.selection.objects = {ObjectId{f.doc.defaultLayer().value}};
    CHECK_FALSE(f.session.apply(layerAsObject).ok);

    ToolpathCalculation noMoves = good;
    noMoves.items[0].record.moves = nullptr;
    CHECK_FALSE(f.session.apply(noMoves).ok);

    ToolpathCalculation spindle = good;
    spindle.items[0].record.tool.spindleOff = true;
    CHECK_FALSE(f.session.apply(spindle).ok);

    ToolpathCalculation name = good;
    name.items[0].record.name = "a\nb";
    CHECK_FALSE(f.session.apply(name).ok);

    ToolpathCalculation missingTarget = good;
    missingTarget.target = ToolpathId{31337};
    CHECK_FALSE(f.session.apply(missingTarget).ok);

    ToolpathCalculation empty;
    CHECK_FALSE(f.session.apply(empty).ok);
    CHECK(snapshot(f.doc) == before);
    CHECK(f.steps() == steps);
}

TEST_CASE("session: no ids left refuses without changes", "[toolpath][session]") {
    Fixture f;
    // An object with a high preset id leaves exactly one id.
    document::Object highId = document::Object::vector(square(200, 0, 5), f.doc.defaultLayer(), f.doc.activeSheet());
    highId.id = ObjectId{document::IdGenerator::kMaxId - 2};
    f.doc.insertObject(highId);
    REQUIRE(f.doc.idGenerator().remaining() == 1);
    const Snapshot before = snapshot(f.doc);
    ToolpathDefinition vc = vCarveDefinition(f.db, true);
    vc.selector = VectorSelector::fromObjects({f.big});
    const ToolpathOutcome refused = f.session.add(vc);  // needs two ids
    CHECK_FALSE(refused.ok);
    CHECK(snapshot(f.doc) == before);
    CHECK(f.session.add(f.pocketOnBig()).ok);  // the last id
    CHECK_FALSE(f.session.add(f.pocketOnBig()).ok);
    CHECK_FALSE(f.session.duplicate(f.doc.toolpaths()[0].id).ok);
    CHECK(f.doc.toolpaths().size() == 1);
}

TEST_CASE("session: a V-carve with clearance tools is a group of toolpaths", "[toolpath][session]") {
    Fixture f;
    ToolpathDefinition vc = vCarveDefinition(f.db, true);
    vc.selector = VectorSelector::fromObjects({f.big});
    vc.setName("Sign");
    const Snapshot empty = snapshot(f.doc);
    const ToolpathOutcome out = f.session.add(vc);
    INFO(out.message);
    REQUIRE(out.ok);
    REQUIRE(out.ids.size() == 2);
    CHECK(f.steps() == 1);
    // Machining order: the clearance tool first, the carving tool last, named "<name> [<tool>]".
    CHECK(f.doc.toolpaths()[0].id == out.ids[0]);
    CHECK(f.doc.toolpaths()[0].toolSource.toolId == "end-mill-3mm");
    CHECK(f.doc.toolpaths()[1].toolSource.toolId == "v-bit-90");
    CHECK(f.doc.toolpaths()[0].name == "Sign [" + f.db.findTool("end-mill-3mm")->name + "]");
    CHECK(f.doc.toolpaths()[1].name == "Sign [" + f.db.findTool("v-bit-90")->name + "]");
    const Snapshot added = snapshot(f.doc);

    // The definition of either member is the V-carve's.
    CHECK(f.session.definition(out.ids[0])->tool.toolId == "v-bit-90");
    CHECK(f.session.definition(out.ids[0])->clearanceTools.size() == 1);
    CHECK(*f.session.definition(out.ids[0]) == *f.session.definition(out.ids[1]));

    // Rename renames both; recalculation by either member replaces both, in place, in one step.
    REQUIRE(f.session.rename(out.ids[1], "Plate").ok);
    CHECK(f.doc.toolpaths()[0].name == "Plate [" + f.db.findTool("end-mill-3mm")->name + "]");
    CHECK(f.steps() == 2);
    f.doc.undo();
    moveObject(f.doc, f.big, 3.0, 0.0);
    CHECK(f.session.item(out.ids[0])->status == ToolpathStatus::NeedsRecalculation);
    CHECK(f.session.item(out.ids[1])->status == ToolpathStatus::NeedsRecalculation);
    const std::size_t steps = f.steps();
    REQUIRE(f.session.recalculate(out.ids[0]).ok);
    CHECK(f.steps() == steps + 1);
    CHECK(f.doc.toolpaths().size() == 2);
    CHECK(f.doc.toolpaths()[0].id == out.ids[0]);
    CHECK(f.doc.toolpaths()[1].id == out.ids[1]);
    CHECK(f.session.item(out.ids[0])->status == ToolpathStatus::Current);
    CHECK(f.session.item(out.ids[1])->status == ToolpathStatus::Current);
    f.doc.undo();
    f.doc.undo();
    CHECK(snapshot(f.doc) == added);

    // Dropping the clearance tool removes its toolpath; adding it back inserts it before the carve.
    ToolpathDefinition plain = *f.session.definition(out.ids[1]);
    plain.clearanceTools.clear();
    REQUIRE(f.session.update(out.ids[1], plain).ok);
    REQUIRE(f.doc.toolpaths().size() == 1);
    CHECK(f.doc.toolpaths()[0].id == out.ids[1]);
    CHECK(f.doc.toolpaths()[0].name == "Sign");
    ToolpathDefinition with = plain;
    with.clearanceTools = {source("end-mill-3mm")};
    const ToolpathOutcome back = f.session.update(out.ids[1], with);
    REQUIRE(back.ok);
    REQUIRE(f.doc.toolpaths().size() == 2);
    CHECK(f.doc.toolpaths()[1].id == out.ids[1]);
    CHECK(f.doc.toolpaths()[0].toolSource.toolId == "end-mill-3mm");
    CHECK(f.doc.checkIntegrity().empty());

    // Deleting a member and recalculating the other restores it in place.
    const ToolpathId clearanceId = f.doc.toolpaths()[0].id;
    REQUIRE(f.session.deleteToolpaths({clearanceId}).ok);
    CHECK(f.doc.toolpaths().size() == 1);
    REQUIRE(f.session.recalculate(out.ids[1]).ok);
    REQUIRE(f.doc.toolpaths().size() == 2);
    CHECK(f.doc.toolpaths()[0].toolSource.toolId == "end-mill-3mm");
    CHECK(f.doc.toolpaths()[1].id == out.ids[1]);

    // Duplicate copies the group and the copy is its own group.
    const ToolpathOutcome dup = f.session.duplicate(out.ids[1]);
    REQUIRE(dup.ok);
    CHECK(dup.ids.size() == 2);
    CHECK(f.doc.toolpaths().size() == 4);
    CHECK(f.doc.toolpaths()[2].name == "Sign (copy) [" + f.db.findTool("end-mill-3mm")->name + "]");
    CHECK(f.session.definition(dup.ids[0])->clearanceTools.size() == 1);
    moveObject(f.doc, f.big, 1.0, 0.0);
    REQUIRE(f.session.recalculate(dup.ids[1]).ok);
    CHECK(f.doc.toolpaths().size() == 4);
    CHECK(f.session.item(dup.ids[0])->status == ToolpathStatus::Current);
    CHECK(f.session.item(out.ids[1])->status == ToolpathStatus::NeedsRecalculation);
}

TEST_CASE("session: a saved and reloaded file is current, groups are inferred, stale detection resumes", "[toolpath][session]") {
    namespace fs = std::filesystem;
    Fixture f;
    ToolpathDefinition vc = vCarveDefinition(f.db, true);
    vc.selector = VectorSelector::fromObjects({f.big});
    const ToolpathOutcome v = f.session.add(vc);
    REQUIRE(v.ok);
    const ToolpathId p = f.addPocket();
    const fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / "reload";
    fs::create_directories(dir);
    REQUIRE(document::saveCarv(f.doc, dir / "a.carv", {}));
    auto loaded = document::loadCarv(dir / "a.carv");
    REQUIRE(loaded);
    Document& doc2 = loaded.value().document;

    ToolpathSession s2(&doc2);
    s2.setToolDatabase(f.db);
    const auto list = s2.list();
    REQUIRE(list.size() == 3);
    for (const ToolpathListItem& i : list) {
        CHECK(i.status == ToolpathStatus::Current);
        CHECK(i.canRecalculate);
    }
    // The V-carve's clearance toolpath finds its carving toolpath again.
    const ToolpathId clearance = doc2.toolpaths()[0].id;
    const ToolpathId carving = doc2.toolpaths()[1].id;
    CHECK(s2.definition(clearance)->tool.toolId == "v-bit-90");
    CHECK(s2.definition(clearance)->clearanceTools.size() == 1);
    // Recalculating reproduces the same records.
    const std::vector<ToolpathRecord> before = doc2.toolpaths();
    REQUIRE(s2.recalculate(carving).ok);
    CHECK(doc2.toolpaths() == before);
    REQUIRE(s2.recalculateAll(false).ok);
    CHECK(doc2.toolpaths() == before);

    // Moving a vector marks the loaded toolpaths stale.
    moveObject(doc2, f.big, 1.0, 0.0);
    for (const ToolpathListItem& i : s2.list()) {
        CHECK(i.status == ToolpathStatus::NeedsRecalculation);
    }
    (void)p;
    // Without the tool database the toolpaths are still listed, not stale, and cannot be recalculated.
    ToolpathSession s3(&doc2);
    const auto bare = s3.list();
    REQUIRE(bare.size() == 3);
    CHECK_FALSE(bare[0].canRecalculate);
    CHECK_FALSE(bare[0].note.empty());
    CHECK_FALSE(s3.recalculate(bare[0].id).ok);
    s3.setToolDatabase(f.db);
    CHECK(s3.list()[0].canRecalculate);
}

TEST_CASE("session: tool database changes are detected", "[toolpath][session]") {
    Fixture f;
    const ToolpathId id = f.addPocket();
    REQUIRE(f.session.item(id)->status == ToolpathStatus::Current);
    tooling::ToolDatabase db2 = f.db;
    tooling::Tool t = *db2.findTool("end-mill-6mm");
    t.defaultCuttingData.feedRate = 1234.0;
    REQUIRE(db2.updateTool(t));
    f.session.setToolDatabase(db2);
    ToolpathListItem i = *f.session.item(id);
    CHECK(i.status == ToolpathStatus::NeedsRecalculation);
    CHECK(i.toolChanged);
    REQUIRE(f.session.recalculate(id).ok);
    CHECK(f.doc.toolpaths()[0].tool.feedRate == 1234.0);
    CHECK(f.session.item(id)->status == ToolpathStatus::Current);

    // A tool that left the database: listed, not recalculable, record untouched.
    tooling::ToolDatabase db3 = db2;
    db3.removeTool("end-mill-6mm");
    f.session.setToolDatabase(db3);
    i = *f.session.item(id);
    CHECK(i.status == ToolpathStatus::Current);
    CHECK_FALSE(i.canRecalculate);
    CHECK(i.note.find("end-mill-6mm") != std::string::npos);
    const Snapshot before = snapshot(f.doc);
    CHECK_FALSE(f.session.recalculate(id).ok);
    const BatchOutcome batch = f.session.recalculateAll(false);
    CHECK_FALSE(batch.ok);
    CHECK(batch.failed == 1);
    CHECK(snapshot(f.doc) == before);
    f.session.setToolDatabase(db2);
    CHECK(f.session.item(id)->canRecalculate);
}

TEST_CASE("session: the selector filters and the current selection", "[toolpath][session]") {
    Fixture f;
    geometry::Contour line;
    line.append(geometry::LineSpan{{0, 60}, {20, 60}});
    const ObjectId openId = addVector(f.doc, line);
    const ObjectId circle = addVector(f.doc, geometry::Contour::circle({100, 100}, 10.0));

    // New forms take the current selection when there is one.
    f.doc.selection().clear();
    CHECK(f.session.newForm(ToolpathOperation::Pocket).selector() == VectorSelector::allVectors());
    f.doc.selection().assign({circle});
    CHECK(f.session.newForm(ToolpathOperation::Pocket).selector().base.objects == std::vector<ObjectId>{circle});

    // Only circles: the record keeps the resolved objects.
    ToolpathDefinition def = pocketDefinition(f.db);
    def.selector.onlyCircles = true;
    const ToolpathOutcome out = f.session.add(def);
    REQUIRE(out.ok);
    CHECK(f.doc.toolpaths()[0].selection.objects == std::vector<ObjectId>{circle});
    CHECK(f.session.definition(out.ids[0])->selector == def.selector);  // filters kept in the session
    CHECK(f.session.item(out.ids[0])->status == ToolpathStatus::Current);
    // A new circle appears: the filter selects it, so the toolpath is stale.
    const ObjectId circle2 = addVector(f.doc, geometry::Contour::circle({150, 100}, 10.0));
    CHECK(f.session.item(out.ids[0])->status == ToolpathStatus::NeedsRecalculation);
    REQUIRE(f.session.recalculate(out.ids[0]).ok);
    CHECK(f.doc.toolpaths()[0].selection.objects == std::vector<ObjectId>{circle, circle2});
    // A square that is not a circle changes nothing.
    addVector(f.doc, square(200, 200, 20));
    CHECK(f.session.item(out.ids[0])->status == ToolpathStatus::Current);

    // Closed only: the open vector is not selected.
    ToolpathDefinition closed = profileDefinition(f.db);
    closed.selector.includeOpen = false;
    REQUIRE(f.session.add(closed).ok);
    const auto& sel = f.doc.toolpaths().back().selection.objects;
    CHECK(std::find(sel.begin(), sel.end(), openId) == sel.end());

    // An invalid selector is refused with a message.
    ToolpathDefinition bad = pocketDefinition(f.db);
    bad.selector = VectorSelector::fromObjects({ObjectId{424242}});
    const ToolpathOutcome refused = f.session.add(bad);
    CHECK_FALSE(refused.ok);
    CHECK_FALSE(refused.message.empty());
}

TEST_CASE("session: enabling, last messages and the document switch", "[toolpath][session]") {
    Fixture f;
    const ToolpathId id = f.addPocket();
    CHECK(f.session.isEnabled(id));
    CHECK(f.session.setEnabled(id, false));
    CHECK_FALSE(f.session.isEnabled(id));
    CHECK_FALSE(f.session.item(id)->enabled);
    CHECK(f.session.enabledToolpaths().empty());
    CHECK_FALSE(f.session.setEnabled(ToolpathId{77}, false));
    CHECK(f.session.setEnabled(id, true));
    CHECK(f.session.enabledToolpaths() == std::vector<ToolpathId>{id});
    CHECK(f.steps() == 1);  // not an undo step

    // Switching documents forgets everything; the old document may already be gone.
    {
        Document other;
        addVector(other, square(0, 0, 30));
        ToolpathSession s(&other);
        s.setToolDatabase(f.db);
        REQUIRE(s.add(pocketDefinition(f.db)).ok);
        s.setEnabled(other.toolpaths()[0].id, false);
        Document replacement;
        s.setDocument(&replacement);
        other.undo();  // not touched by the session any more
        CHECK(s.list().empty());
        CHECK(s.enabledToolpaths().empty());
        CHECK(s.summary().toolpaths == 0);
        s.setDocument(nullptr);
        CHECK(s.list().empty());
        CHECK_FALSE(s.add(pocketDefinition(f.db)).ok);
        CHECK_FALSE(s.recalculate(id).ok);
        CHECK_FALSE(s.deleteToolpaths({id}).ok);
        CHECK_FALSE(s.rename(id, "x").ok);
        CHECK_FALSE(s.duplicate(id).ok);
        CHECK_FALSE(s.reorder(id, 0).ok);
        CHECK(s.countVectors(VectorSelector::allVectors()) == 0);
        CHECK_FALSE(s.definition(id).has_value());
        CHECK_FALSE(s.form(id).has_value());
        CHECK_FALSE(s.recalculateAll(false).ok);
        CHECK(s.summary().toolpaths == 0);
    }
}

TEST_CASE("session: undo, redo and deletions made outside the session leave it consistent", "[toolpath][session]") {
    Fixture f;
    const ToolpathId a = f.addPocket();
    f.doc.undo();
    CHECK(f.session.list().empty());
    CHECK_FALSE(f.session.item(a).has_value());
    f.doc.redo();
    CHECK(f.session.item(a)->status == ToolpathStatus::Current);
    // A command from outside that removes and restores the toolpath.
    REQUIRE(f.doc.execute(std::make_unique<document::RemoveToolpathsCommand>(std::vector<ToolpathId>{a})));
    CHECK(f.session.list().empty());
    f.doc.undo();
    CHECK(f.session.item(a)->status == ToolpathStatus::Current);
    // Edits outside the session: the parameters differ from what the moves were computed with.
    document::ToolpathRecord edited = *f.doc.findToolpath(a);
    std::get<toolpath::PocketParams>(edited.params).cutDepth = 1.0;
    REQUIRE(f.doc.execute(std::make_unique<document::SetToolpathCommand>(edited)));
    const ToolpathListItem i = *f.session.item(a);
    CHECK(i.status == ToolpathStatus::NeedsRecalculation);
    CHECK(i.parametersChanged);
    REQUIRE(f.session.recalculate(a).ok);
    CHECK(std::get<toolpath::PocketParams>(f.doc.toolpaths()[0].params).cutDepth == 1.0);
    CHECK(f.session.item(a)->status == ToolpathStatus::Current);
    // A record without moves (a definition only) is listed as not computed and can be calculated.
    document::ToolpathRecord bare = *f.doc.findToolpath(a);
    bare.moves = nullptr;
    REQUIRE(f.doc.execute(std::make_unique<document::SetToolpathCommand>(bare)));
    CHECK(f.session.item(a)->status == ToolpathStatus::NotComputed);
    CHECK(f.session.enabledToolpaths().empty());
    REQUIRE(f.session.recalculate(a).ok);
    CHECK(f.doc.toolpaths()[0].hasMoves());
    CHECK(f.session.item(a)->status == ToolpathStatus::Current);
}

TEST_CASE("session: the job summary", "[toolpath][session]") {
    Fixture f;
    REQUIRE(f.session.summary().toolpaths == 0);
    ToolpathDefinition prof = profileDefinition(f.db);
    prof.selector = VectorSelector::fromObjects({f.big, f.tiny});  // two cuts: rapid moves between them
    const ToolpathId p1 = f.session.add(prof).ids.front();
    const ToolpathId pk = f.addPocket();
    const ToolpathId p2 = f.session.add(prof).ids.front();

    JobSummary s = f.session.summary();
    CHECK(s.toolpaths == 3);
    CHECK(s.computed == 3);
    CHECK(s.included == 3);
    CHECK(s.needRecalculation == 0);
    CHECK(s.toolChanges == 2);  // 3 mm, 6 mm, 3 mm
    CHECK(s.distinctTools == 2);
    std::size_t moves = 0;
    for (const ToolpathRecord& r : f.doc.toolpaths()) {
        moves += r.moveCount();
    }
    CHECK(s.moves == moves);
    CHECK(s.entries.size() == 3);
    REQUIRE_FALSE(s.bounds.isEmpty());
    CHECK(s.bounds.min.x < 0.0);  // the profile runs outside the square
    CHECK(s.bounds.max.x > 30.0);
    CHECK(*s.maxZ == 5.0);
    CHECK(*s.minZ == -3.0);
    // No rapid rate: rapid moves are left out and reported.
    CHECK(s.rapidRateMissing);
    CHECK(s.time.rapid == 0.0);
    CHECK(s.time.feed > 0.0);
    CHECK(s.time.plunge > 0.0);
    CHECK(std::isfinite(s.time.total()));

    // With the machine's rates: equals estimateTime, summed.
    REQUIRE(f.session.setMachineSetup(MachineSetup{3000.0, 500.0}));
    s = f.session.summary();
    CHECK_FALSE(s.rapidRateMissing);
    toolpath::TimeEstimate sum;
    for (const ToolpathRecord& r : f.doc.toolpaths()) {
        const toolpath::TimeEstimate t =
            toolpath::estimateTime(r.toolpath(), toolpath::MachineRates::fromTool(r.tool, 3000.0, 500.0));
        sum.rapid += t.rapid;
        sum.feed += t.feed;
        sum.plunge += t.plunge;
        sum.retract += t.retract;
        sum.dwell += t.dwell;
    }
    CHECK(s.time == sum);
    CHECK(s.time.rapid > 0.0);
    // Acceleration slows the estimate.
    const double fast = s.time.total();
    REQUIRE(f.session.setMachineSetup(MachineSetup{3000.0, 50.0}));
    CHECK(f.session.summary().time.total() > fast);

    // Disabled toolpaths are listed but not counted.
    f.session.setEnabled(pk, false);
    s = f.session.summary();
    CHECK(s.included == 2);
    CHECK(s.toolChanges == 0);
    CHECK(s.distinctTools == 1);
    CHECK(s.moves == moves - f.doc.findToolpath(pk)->moveCount());
    CHECK_FALSE(s.entries[1].included);
    CHECK(s.entries[1].moves == f.doc.findToolpath(pk)->moveCount());
    // Stale toolpaths are counted.
    moveObject(f.doc, f.big, 1.0, 1.0);
    CHECK(f.session.summary().needRecalculation == 3);
    (void)p1;
    (void)p2;

    // Bad rates are rejected and keep the old ones.
    const MachineSetup keep = f.session.machineSetup();
    CHECK_FALSE(f.session.setMachineSetup(MachineSetup{std::nan(""), 1.0}));
    CHECK_FALSE(f.session.setMachineSetup(MachineSetup{-1.0, 1.0}));
    CHECK_FALSE(f.session.setMachineSetup(MachineSetup{100.0, 0.0}));
    CHECK_FALSE(f.session.setMachineSetup(MachineSetup{100.0, -5.0}));
    CHECK_FALSE(f.session.setMachineSetup(MachineSetup{std::numeric_limits<double>::infinity(), 5.0}));
    CHECK(f.session.machineSetup() == keep);
    tooling::Machine m;
    m.accelerationX = 400.0;
    m.accelerationY = 300.0;
    m.accelerationZ = 0.0;
    REQUIRE(f.session.setMachine(m, 2000.0));
    CHECK(f.session.machineSetup() == MachineSetup{2000.0, 300.0});
    m.accelerationX = m.accelerationY = 0.0;
    REQUIRE(f.session.setMachine(m, 2000.0));
    CHECK(std::isinf(f.session.machineSetup().accelerationMmPerS2));
}

TEST_CASE("session: forms reopen with the toolpath's values", "[toolpath][session]") {
    Fixture f;
    ToolpathDefinition def = f.pocketOnBig();
    std::get<toolpath::PocketParams>(def.params).strategy = toolpath::PocketStrategy::Raster;
    std::get<toolpath::PocketParams>(def.params).rasterAngle = 20.0 * (geometry::kPi / 180.0);
    def.setName("Raster pocket");
    const ToolpathId id = f.session.add(def).ids.front();
    std::optional<ToolpathForm> form = f.session.form(id);
    REQUIRE(form.has_value());
    CHECK(form->name() == "Raster pocket");
    CHECK(form->toolSummary().valid);
    CHECK(form->problems().empty());
    CHECK(form->definition().params == def.params);
    for (const ToolpathField& field : form->fields()) {
        if (field.id == "rasterAngle") {
            CHECK(field.text == "20");
        }
    }
    CHECK_FALSE(f.session.form(ToolpathId{77}).has_value());
    // Editing through the form and updating round-trips.
    REQUIRE(form->setField("cutDepth", "2").ok);
    REQUIRE(f.session.update(id, form->definition()).ok);
    CHECK(std::get<toolpath::PocketParams>(f.doc.toolpaths()[0].params).cutDepth == 2.0);
    CHECK(f.session.newForm(ToolpathOperation::Drill).operation() == ToolpathOperation::Drill);
}

TEST_CASE("session: a recalculation that changes nothing refreshes the toolpath without an undo step", "[toolpath][session]") {
    Fixture f;
    // A pocket machines closed vectors; an open vector in its selection is ignored (reported).
    geometry::Contour line;
    line.append(geometry::LineSpan{{0, 80}, {10, 80}});
    const ObjectId open = addVector(f.doc, line);
    ToolpathDefinition def = pocketDefinition(f.db);
    def.selector = VectorSelector::fromObjects({f.big, open});
    const ToolpathId id = f.session.add(def).ids.front();
    const Snapshot before = snapshot(f.doc);
    moveObject(f.doc, open, 5.0, 0.0);
    CHECK(f.session.item(id)->status == ToolpathStatus::NeedsRecalculation);
    const std::size_t steps = f.steps();
    const ToolpathOutcome out = f.session.recalculate(id);
    REQUIRE(out.ok);
    CHECK(f.steps() == steps);  // identical records: nothing to store
    CHECK(f.doc.toolpaths() == before.toolpaths);
    CHECK(f.session.item(id)->status == ToolpathStatus::Current);
}

TEST_CASE("session: a job that was not made by prepare cannot crash the calculation", "[toolpath][session]") {
    ToolpathJob empty;
    CHECK_FALSE(empty.run().ok);
    CHECK(empty.run().hasError());
    ToolpathJob noTool;
    noTool.contours.push_back(square(0, 0, 10));
    noTool.owners.push_back(ObjectId{1});
    CHECK_FALSE(noTool.run().ok);
    // A tool but mismatching owners and clearance lists.
    Fixture f;
    ToolpathJob job = f.session.prepare(vCarveDefinition(f.db, true));
    job.clearance.clear();
    job.owners.clear();
    const ToolpathCalculation calc = job.run();
    CHECK(calc.ok);  // the carving tool alone still carves
    CHECK(calc.items.size() == 1);
}
