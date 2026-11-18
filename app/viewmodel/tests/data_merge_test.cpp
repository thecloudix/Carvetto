#include "carvetto/viewmodel/data_merge.hpp"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <set>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/text_object.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/text/font.hpp"
#include "carvetto/viewmodel/starter_font.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using document::Document;
using document::Object;
using document::ObjectId;
using geometry::Box2;
using geometry::Contour;

namespace {

struct Fixture {
    Document doc;
    ObjectId rect;
    ObjectId label;
    std::string fontKey;

    explicit Fixture(const std::string& labelText = "!!NAME!! !!SERIAL!!", double sheetSize = 300.0) {
        const auto bytes = starterFontBytes();
        document::FontAsset asset;
        asset.bytes.assign(bytes.begin(), bytes.end());
        asset.key = document::fontKeyFor(asset.bytes);
        fontKey = asset.key;
        REQUIRE(doc.insertFont(asset));
        doc.findSheet(doc.activeSheet())->sizeMm.x = sheetSize;
        doc.findSheet(doc.activeSheet())->sizeMm.y = sheetSize;

        rect = doc.insertObject(Object::vector(Contour::rectangle(Box2{{10, 10}, {70, 40}}), doc.activeLayer(),
                                               doc.activeSheet()));
        document::TextParams params;
        params.text = labelText;
        params.fontKey = fontKey;
        params.sizeMm = 6.0;
        params.placement = geometry::Transform2::translation({14.0, 20.0});
        text::FontLoad load = text::Font::load(asset.bytes);
        REQUIRE(load.ok());
        document::TextBuild built = document::buildTextContours(load.font, params);
        REQUIRE(built.ok());
        label = doc.insertObject(
            Object::vector(std::move(built.contours), doc.activeLayer(), doc.activeSheet(), params));
        doc.selection().assign({rect, label});
    }
};

std::string csvRows(int n) {
    std::string s = "Name,Qty\n";
    for (int i = 1; i <= n; ++i) {
        s += "Person" + std::to_string(i) + "," + std::to_string(i) + "\n";
    }
    return s;
}

DataMergeOptions baseOptions() {
    DataMergeOptions o;
    o.assignments = {{"NAME", 0}};
    CounterSpec serial;
    serial.name = "SERIAL";
    serial.start = 1;
    serial.width = 3;
    serial.prefix = "SN";
    o.counters = {serial};
    return o;
}

std::string textOf(const Document& doc, ObjectId id) {
    const Object* obj = doc.findObject(id);
    REQUIRE(obj != nullptr);
    const auto* params = std::get_if<document::TextParams>(&obj->asVector()->params);
    REQUIRE(params != nullptr);
    return params->text;
}

}  // namespace

TEST_CASE("merge: a 20-row CSV makes 20 laid-out plates in one undo step", "[merge]") {
    Fixture f;
    const std::size_t objectsBefore = f.doc.objectCount();
    const std::size_t undoBefore = f.doc.undoStack().index();
    const DataMergeResult r = applyDataMerge(f.doc, csvRows(20), baseOptions());
    REQUIRE(r.ok);
    CHECK(r.plates == 20);
    CHECK(r.sheetsUsed == 1);
    CHECK(r.created.size() == 40);  // rectangle + label per plate
    CHECK(f.doc.objectCount() == objectsBefore + 40);
    CHECK(f.doc.undoStack().index() == undoBefore + 1);
    CHECK(f.doc.selection().ids() == r.created);

    // Plate k: [rect, label] in creation order.
    std::set<std::string> texts;
    const Box2 material = f.doc.findSheet(f.doc.activeSheet())->materialBoundsMm();
    for (std::size_t k = 0; k < 20; ++k) {
        const ObjectId rectId = r.created[2 * k];
        const ObjectId labelId = r.created[2 * k + 1];
        const std::string expected =
            "Person" + std::to_string(k + 1) + " SN" + std::string(k + 1 < 10 ? "00" : "0") + std::to_string(k + 1);
        CHECK(textOf(f.doc, labelId) == expected);
        texts.insert(expected);
        const Box2 plate = *f.doc.objectBounds(rectId);
        CHECK(plate.width() == 60.0);
        CHECK(plate.height() == 30.0);
        CHECK(plate.min.x >= material.min.x + 5.0 - 1e-9);
        CHECK(plate.max.x <= material.max.x - 5.0 + 1e-9);
        CHECK(plate.min.y >= material.min.y + 5.0 - 1e-9);
        CHECK(plate.max.y <= material.max.y - 5.0 + 1e-9);
        const Object* lab = f.doc.findObject(labelId);
        REQUIRE_FALSE(lab->asVector()->contours.empty());
        const Box2 labelBounds = *f.doc.objectBounds(labelId);
        CHECK(labelBounds.min.x >= plate.min.x);
        CHECK(labelBounds.max.y <= plate.max.y);
        CHECK(labelBounds.min.y >= plate.min.y);
    }
    CHECK(texts.size() == 20);

    // First plate at the top left inside the margins.
    const Box2 first = *f.doc.objectBounds(r.created[0]);
    CHECK(first.min.x == 5.0);
    CHECK(first.max.y == 295.0);

    CHECK(f.doc.undo());
    CHECK(f.doc.objectCount() == objectsBefore);
    CHECK(f.doc.sheets().size() == 1);
    CHECK(f.doc.layers().size() == 1);
}

TEST_CASE("merge: overflow goes to new sheets, one per sheet on request", "[merge]") {
    Fixture f("!!NAME!!", 160.0);  // avail 150: 2 columns of 60 + 5 gap = 125; rows 4 of 30
    DataMergeOptions o = baseOptions();
    const DataMergeResult r = applyDataMerge(f.doc, csvRows(20), o);
    REQUIRE(r.ok);
    CHECK(r.platesPerSheet == 2 * 4);
    CHECK(r.sheetsUsed == 3);
    CHECK(f.doc.sheets().size() == 3);
    std::set<std::uint64_t> sheetsSeen;
    for (const ObjectId id : r.created) {
        sheetsSeen.insert(f.doc.findObject(id)->sheet.value);
    }
    CHECK(sheetsSeen.size() == 3);
    f.doc.undo();
    CHECK(f.doc.sheets().size() == 1);

    f.doc.selection().assign({f.rect, f.label});
    o.layout = MergeLayout::OnePerSheet;
    const DataMergeResult single = applyDataMerge(f.doc, csvRows(4), o);
    REQUIRE(single.ok);
    CHECK(single.sheetsUsed == 4);
    CHECK(f.doc.sheets().size() == 4);
    for (std::size_t k = 0; k < 4; ++k) {
        const Box2 b = *f.doc.objectBounds(single.created[2 * k]);
        CHECK(b.min.x == 5.0);
        CHECK(b.max.y == 155.0);
    }
}

TEST_CASE("merge: malformed rows give line-numbered diagnostics and change nothing", "[merge]") {
    Fixture f;
    const std::size_t objects = f.doc.objectCount();
    const std::string csv = "Name,Qty\nA,1\nB\nC,3,extra\n\"open,4\nE,5\n";
    const DataMergeResult r = applyDataMerge(f.doc, csv, baseOptions());
    CHECK_FALSE(r.ok);
    REQUIRE(r.diagnostics.size() >= 3);
    CHECK(r.diagnostics[0].line == 3);
    CHECK(r.diagnostics[1].line == 4);
    CHECK(r.diagnostics[2].line == 5);
    CHECK_FALSE(r.message.empty());
    CHECK(f.doc.objectCount() == objects);
    CHECK_FALSE(f.doc.undoStack().canUndo());

    DataMergeOptions skip = baseOptions();
    skip.skipBadRows = true;
    const DataMergeResult s = applyDataMerge(f.doc, "Name,Qty\nA,1\nB\nC,3\n", skip);
    REQUIRE(s.ok);
    CHECK(s.plates == 2);
    CHECK(s.skippedRows == 1);
    CHECK(textOf(f.doc, s.created[1]) == "A SN001");
    CHECK(textOf(f.doc, s.created[3]) == "C SN002");  // counter follows the plates made
}

TEST_CASE("merge: refusals", "[merge]") {
    Fixture f;
    DataMergeOptions o = baseOptions();
    SECTION("no selection") {
        f.doc.selection().clear();
        CHECK_FALSE(applyDataMerge(f.doc, csvRows(2), o).ok);
        CHECK_FALSE(canDataMerge(f.doc));
    }
    SECTION("placeholder without assignment") {
        o.assignments.clear();
        const DataMergeResult r = applyDataMerge(f.doc, csvRows(2), o);
        CHECK_FALSE(r.ok);
        CHECK(r.message.find("NAME") != std::string::npos);
    }
    SECTION("column out of range") {
        o.assignments = {{"NAME", 5}};
        CHECK_FALSE(applyDataMerge(f.doc, csvRows(2), o).ok);
    }
    SECTION("too many rows") {
        CHECK_FALSE(applyDataMerge(f.doc, csvRows(1001), o).ok);
        CHECK(f.doc.undoStack().index() == 0);
        CHECK(applyDataMerge(f.doc, csvRows(1000), o).ok);
    }
    SECTION("header only and empty") {
        CHECK_FALSE(applyDataMerge(f.doc, "Name,Qty\n", o).ok);
        CHECK_FALSE(applyDataMerge(f.doc, "", o).ok);
    }
    SECTION("plate larger than the sheet") {
        f.doc.findSheet(f.doc.activeSheet())->sizeMm.x = 50.0;
        CHECK_FALSE(applyDataMerge(f.doc, csvRows(2), o).ok);
    }
    SECTION("malformed header") {
        CHECK_FALSE(applyDataMerge(f.doc, "\"Name,Qty\nA,1\n", o).ok);
    }
    SECTION("missing font") {
        f.doc.findObject(f.label);
        auto* params = std::get_if<document::TextParams>(&f.doc.findObject(f.label)->asVector()->params);
        params->fontKey = "0000000000000000";
        CHECK_FALSE(applyDataMerge(f.doc, csvRows(2), o).ok);
    }
}

TEST_CASE("merge: options are validated", "[merge]") {
    DataMergeOptions o = baseOptions();
    CHECK(validateDataMergeOptions(o).empty());
    o.counters[0].width = 0;
    CHECK_FALSE(validateDataMergeOptions(o).empty());
    o = baseOptions();
    o.assignments.push_back({"1bad", 0});
    CHECK_FALSE(validateDataMergeOptions(o).empty());
    o = baseOptions();
    o.assignments.push_back({"SERIAL", 1});
    CHECK_FALSE(validateDataMergeOptions(o).empty());
    o = baseOptions();
    o.marginLeftMm = -1.0;
    CHECK_FALSE(validateDataMergeOptions(o).empty());
    o = baseOptions();
    o.separator = '"';
    CHECK_FALSE(validateDataMergeOptions(o).empty());
    o = baseOptions();
    o.counters[0].start = 2'000'000'000'000LL;
    CHECK_FALSE(validateDataMergeOptions(o).empty());
}

TEST_CASE("merge: counter formatting", "[merge]") {
    CounterSpec c;
    c.name = "N";
    c.start = 8;
    c.step = 2;
    c.width = 4;
    c.prefix = "A-";
    c.suffix = "!";
    CHECK(formatCounter(c, 0) == "A-0008!");
    CHECK(formatCounter(c, 1) == "A-0010!");
    c.start = -3;
    c.step = 1;
    c.width = 2;
    CHECK(formatCounter(c, 0) == "A--03!");
    CHECK(formatCounter(c, 5) == "A-02!");
}

TEST_CASE("merge: placeholders, auto-assignment and CSV separators", "[merge]") {
    Fixture f("Hi !!name!! !!!!x!! !!1!! !!SERIAL!!");
    // "!!!!x!!" -> literal "!!" is not a placeholder, then !!x!!? The scan finds the real ones.
    const std::vector<std::string> names = templatePlaceholders(f.doc);
    CHECK(names == std::vector<std::string>{"SERIAL", "name", "x"});
    const auto headers = csvHeaders("NAME;Qty\nA;1\n", ';');
    CHECK(headers == std::vector<std::string>{"NAME", "Qty"});
    const auto assigned = autoAssign({"name", "x", "QTY"}, headers);
    REQUIRE(assigned.size() == 2);
    CHECK(assigned[0].column == 0);
    CHECK(assigned[1].name == "QTY");
    CHECK(assigned[1].column == 1);

    DataMergeOptions o = baseOptions();
    o.separator = ';';
    o.assignments = {{"name", 0}, {"x", 1}};
    const DataMergeResult r = applyDataMerge(f.doc, "NAME;Qty\nAnna;7\n", o);
    REQUIRE(r.ok);
    CHECK(textOf(f.doc, r.created[1]) == "Hi Anna !!7 !!1!! SN001");
}

TEST_CASE("merge: preview leaves the document alone; layer and template options", "[merge]") {
    Fixture f;
    DataMergeOptions o = baseOptions();
    const std::size_t objects = f.doc.objectCount();
    const DataMergeResult preview = previewDataMerge(f.doc, csvRows(5), o);
    REQUIRE(preview.ok);
    CHECK(preview.plates == 5);
    CHECK(preview.created.empty());
    CHECK(f.doc.objectCount() == objects);
    CHECK_FALSE(f.doc.undoStack().canUndo());

    o.newLayer = true;
    o.removeTemplate = true;
    const DataMergeResult r = applyDataMerge(f.doc, csvRows(5), o);
    REQUIRE(r.ok);
    CHECK(f.doc.layers().size() == 2);
    CHECK(f.doc.objectCount() == 10);
    CHECK(f.doc.findObject(f.rect) == nullptr);
    CHECK(f.doc.findObject(r.created[0])->layer == f.doc.layers().back().id);
    f.doc.undo();
    CHECK(f.doc.objectCount() == objects);
    CHECK(f.doc.findObject(f.rect) != nullptr);
}

TEST_CASE("merge: hidden or locked template layers are skipped, groups are copied whole", "[merge]") {
    Fixture f;
    Object group;
    group.layer = f.doc.activeLayer();
    group.sheet = f.doc.activeSheet();
    group.data = document::GroupObject{{f.rect, f.label}};
    const ObjectId gid = f.doc.insertObject(std::move(group));
    f.doc.findObject(f.rect)->parent = gid;
    f.doc.findObject(f.label)->parent = gid;
    f.doc.selection().assign({gid, f.label});  // the child is part of the selected group
    const DataMergeResult r = applyDataMerge(f.doc, csvRows(3), baseOptions());
    REQUIRE(r.ok);
    CHECK(r.created.size() == 3);  // three group copies
    CHECK(f.doc.findObject(r.created[0])->isGroup());
    CHECK(f.doc.findObject(r.created[0])->asGroup()->children.size() == 2);

    f.doc.findLayer(f.doc.activeLayer())->locked = true;
    f.doc.selection().assign({gid});
    CHECK_FALSE(canDataMerge(f.doc));
}
