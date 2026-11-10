// T-0261: Copy/Move to Other Side, switching back to single-sided, toolpath sheet links and the
// per-side Z zero of Save Toolpaths.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <string>

#include "carvetto/document/double_sided.hpp"
#include "carvetto/viewmodel/double_sided_ops.hpp"
#include "carvetto/viewmodel/layers_sheets.hpp"
#include "carvetto/viewmodel/save_toolpaths.hpp"
#include "toolpath_test_support.hpp"

namespace fs = std::filesystem;
using namespace carvetto;
using namespace carvetto::viewmodel;
using namespace carvetto::viewmodel::testing;
using Catch::Approx;

namespace {

struct Fixture {
    document::Document doc{document::Job::doubleSided(200, 100, 18, document::Units::Millimetres,
                                                       document::Datum::BottomLeft, document::ZZero::MachineBed,
                                                       document::FlipDirection::Horizontal)};
    tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session{&doc};
    document::SheetId front;
    document::SheetId back;

    explicit Fixture(bool zeroOffSame = false) {
        session.setToolDatabase(db);
        front = doc.activeSheet();
        REQUIRE(makeDoubleSided(doc, document::FlipDirection::Horizontal, zeroOffSame).ok);
        back = doc.findSheet(front)->partner;
    }
};

geometry::Box2 boundsOf(const document::Document& doc, document::ObjectId id) {
    const auto b = doc.objectBounds(id);
    REQUIRE(b.has_value());
    return *b;
}

}  // namespace

TEST_CASE("copy to other side mirrors the selection onto the partner sheet in one undo step",
          "[viewmodel][double-sided]") {
    Fixture f;
    const document::ObjectId a = addVector(f.doc, square(10, 10, 20));
    f.doc.selection().assign({a});
    const std::size_t steps = f.doc.undoStack().count();
    const SideCopyResult r = copySelectionToOtherSide(f.doc, false);
    REQUIRE(r.edit.ok);
    REQUIRE(r.objects.size() == 1);
    CHECK(f.doc.undoStack().count() == steps + 1);
    const document::Object* copy = f.doc.findObject(r.objects[0]);
    REQUIRE(copy != nullptr);
    CHECK(copy->sheet == f.back);
    const geometry::Box2 b = boundsOf(f.doc, r.objects[0]);
    CHECK(b.min.x == Approx(200 - 30));
    CHECK(b.max.x == Approx(200 - 10));
    CHECK(b.min.y == Approx(10));
    CHECK(f.doc.findObject(a)->sheet == f.front);  // the original stays
    CHECK(f.doc.validate().empty());
    f.doc.undo();
    CHECK(f.doc.objectsOnSheet(f.back).empty());
}

TEST_CASE("move to other side moves and mirrors; refused without a partner or on locked layers",
          "[viewmodel][double-sided]") {
    Fixture f;
    const document::ObjectId a = addVector(f.doc, square(10, 10, 20));
    f.doc.selection().assign({a});
    const SideCopyResult r = copySelectionToOtherSide(f.doc, true);
    REQUIRE(r.edit.ok);
    CHECK(f.doc.findObject(a)->sheet == f.back);
    CHECK(boundsOf(f.doc, a).min.x == Approx(170));
    f.doc.undo();
    CHECK(f.doc.findObject(a)->sheet == f.front);
    CHECK(boundsOf(f.doc, a).min.x == Approx(10));

    f.doc.selection().assign({a});
    f.doc.findLayer(f.doc.findObject(a)->layer)->locked = true;
    CHECK_FALSE(copySelectionToOtherSide(f.doc, false).edit.ok);

    document::Document single;
    const document::ObjectId s = addVector(single, square(0, 0, 5));
    single.selection().assign({s});
    CHECK_FALSE(copySelectionToOtherSide(single, false).edit.ok);
}

TEST_CASE("toolpaths know their sheet and copy to the other side with mirrored moves",
          "[viewmodel][double-sided]") {
    Fixture f;
    addVector(f.doc, square(10, 10, 40));
    const ToolpathOutcome o = f.session.add(profileDefinition(f.db));
    REQUIRE(o.ok);
    const document::ToolpathRecord* rec = f.doc.findToolpath(o.ids.front());
    REQUIRE(rec != nullptr);
    CHECK(rec->sheet == f.front);

    const SideCopyResult r = copyToolpathsToOtherSide(f.doc, {o.ids.front()}, false);
    REQUIRE(r.edit.ok);
    REQUIRE(r.toolpaths.size() == 1);
    const document::ToolpathRecord* copy = f.doc.findToolpath(r.toolpaths[0]);
    REQUIRE(copy != nullptr);
    CHECK(copy->sheet == f.back);
    CHECK(copy->name == rec->name + " (back)");
    REQUIRE(copy->hasMoves());
    REQUIRE(copy->moveCount() == rec->moveCount());
    const auto material = f.doc.findSheet(f.back)->materialBoundsMm();
    CHECK(*copy->moves == document::flipMoves(*rec->moves, material, document::FlipDirection::Horizontal));
    CHECK(f.doc.validate().empty());

    // Recalculating the back toolpath needs its side to be active.
    const ToolpathJob job = f.session.prepare(f.session.definition(copy->id).value(), copy->id);
    CHECK_FALSE(job.refusal.empty());

    // Move: the original record goes.
    const SideCopyResult mv = copyToolpathsToOtherSide(f.doc, {o.ids.front()}, true);
    REQUIRE(mv.edit.ok);
    CHECK(f.doc.findToolpath(o.ids.front()) == nullptr);
    f.doc.undo();
    CHECK(f.doc.findToolpath(o.ids.front()) != nullptr);
}

TEST_CASE("switching back to single-sided unpairs the sheets in one undo step", "[viewmodel][double-sided]") {
    Fixture f;
    const std::size_t steps = f.doc.undoStack().count();
    REQUIRE(makeSingleSided(f.doc).edit.ok == true);
    CHECK(f.doc.undoStack().count() == steps + 1);
    CHECK(f.doc.job().type == document::JobType::SingleSided);
    for (const document::Sheet& s : f.doc.sheets()) {
        CHECK(s.side == document::Side::Front);
        CHECK(s.partner.isNull());
    }
    CHECK(f.doc.sheets().size() == 2);
    CHECK(f.doc.validate().empty());
    CHECK_FALSE(makeSingleSided(f.doc).ok);
    f.doc.undo();
    CHECK(f.doc.job().type == document::JobType::DoubleSided);
    CHECK(f.doc.findSheet(f.back)->side == document::Side::Back);
}

TEST_CASE("save posts each side with its own Z zero in its own file", "[viewmodel][double-sided][save]") {
    Fixture f;  // machine-bed zero, 18 mm stock: both sides' cut face is at Z = 18
    addVector(f.doc, square(10, 10, 40));
    REQUIRE(f.session.add(profileDefinition(f.db)).ok);
    f.doc.selection().assign({f.doc.objectsOnSheet(f.front).front()});
    REQUIRE(copyToolpathsToOtherSide(f.doc, {f.doc.toolpaths().front().id}, false).edit.ok);
    REQUIRE(f.doc.toolpaths().size() == 2);

    const fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / "double_sided_save";
    fs::remove_all(dir);
    fs::create_directories(dir);
    const auto posts = discoverPosts(fs::path(CARVETTO_POSTS_DIR), {});
    const PostInfo* grbl = nullptr;
    for (const PostInfo& p : posts) {
        if (p.path.filename() == "grbl_mm.pp") {
            grbl = &p;
        }
    }
    REQUIRE(grbl != nullptr);
    SaveOptions opt;
    opt.directory = dir;
    opt.jobName = "part";
    const SaveSource src = makeSaveSource(f.doc, {});
    REQUIRE(src.sides.size() == 2);
    const SaveJob job = prepareSave(src, *grbl, opt);
    REQUIRE(job.error.empty());
    const SaveOutput out = generateOutput(job);
    INFO(out.error);
    REQUIRE(out.ok);
    REQUIRE(out.files.size() == 2);
    CHECK(out.files[0].path.filename() == "part_front.nc");
    CHECK(out.files[1].path.filename() == "part_back.nc");
    CHECK(out.files[0].toolpaths.size() == 1);

    // The same toolpath posted without a side is 18 mm lower than with the bed-zero shift.
    SaveSource plain = src;
    plain.sides.clear();
    const SaveOutput flat = generateOutput(prepareSave(plain, *grbl, opt));
    REQUIRE(flat.ok);
    REQUIRE(flat.files.size() == 1);
    CHECK(out.files[0].text != flat.files[0].text);
    CHECK(out.files[0].text.find("Z-3") == std::string::npos);  // cut depth 3 is Z15 with the 18 mm shift
    CHECK(out.files[0].text.find("Z15") != std::string::npos);
    CHECK(flat.files[0].text.find("Z-3") != std::string::npos);
}

TEST_CASE("Z zero follows the side: surface zero without zero-off-same-side posts both sides unshifted",
          "[viewmodel][double-sided][save]") {
    document::Document doc{document::Job::doubleSided(200, 100, 18, document::Units::Millimetres,
                                                      document::Datum::BottomLeft, document::ZZero::MaterialSurface,
                                                      document::FlipDirection::Horizontal)};
    REQUIRE(makeDoubleSided(doc, document::FlipDirection::Horizontal, true).ok);
    const document::SheetId front = doc.sheets()[0].id;
    const document::SheetId back = doc.findSheet(front)->partner;
    CHECK(sheetTopZ(doc, front) == Approx(0));
    CHECK(sheetTopZ(doc, back) == Approx(18));  // the shared zero face is the bed after the flip
    const SaveSource src = makeSaveSource(doc, {});
    REQUIRE(src.sides.size() == 2);
    CHECK(src.sides[0].topZ == Approx(0));
    CHECK(src.sides[1].topZ == Approx(18));
    CHECK(src.sides[1].side == document::Side::Back);
}
