#include "carvetto/viewmodel/file_session.hpp"

#include <atomic>
#include <chrono>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "carvetto/document/commands.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/transform2.hpp"
#include "carvetto/io/dxf.hpp"
#include "carvetto/viewmodel/dxf_exchange.hpp"

using namespace carvetto;
using namespace carvetto::document;
using carvetto::viewmodel::FileSession;
namespace fs = std::filesystem;

namespace {

/// A fresh temporary directory removed on destruction.
class TempDir {
   public:
    TempDir() {
        static std::atomic<int> counter{0};
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        dir_ = fs::temp_directory_path() / ("carvetto_t0202_" + std::to_string(stamp) + "_" + std::to_string(counter++));
        fs::create_directories(dir_);
    }
    ~TempDir() {
        std::error_code ignored;
        fs::remove_all(dir_, ignored);
    }
    TempDir(const TempDir&) = delete;
    TempDir& operator=(const TempDir&) = delete;
    fs::path operator/(const std::string& name) const { return dir_ / name; }

   private:
    fs::path dir_;
};

std::string slurp(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

void spit(const fs::path& p, const std::string& text) {
    std::ofstream out(p, std::ios::binary | std::ios::trunc);
    out << text;
}

ObjectId addCircle(Document& doc, geometry::Vec2 c = {50.0, 50.0}, double r = 10.0) {
    auto cmd = std::make_unique<AddObjectCommand>(
        Object::vector(geometry::Contour::circle(c, r), doc.activeLayer(), doc.activeSheet(), CircleParams{c, r}));
    const AddObjectCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    return raw->id();
}

constexpr const char* kSquareDxf =
    "0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n1\n0\nENDSEC\n"
    "0\nSECTION\n2\nTABLES\n0\nTABLE\n2\nLAYER\n"
    "0\nLAYER\n2\nCUT\n70\n0\n62\n1\n"
    "0\nENDTAB\n0\nENDSEC\n"
    "0\nSECTION\n2\nENTITIES\n"
    "0\nLINE\n8\nCUT\n10\n0\n20\n0\n11\n2\n21\n0\n"
    "0\nLINE\n8\nCUT\n10\n2\n20\n0\n11\n2\n21\n1\n"
    "0\nTEXT\n8\nCUT\n10\n0\n20\n0\n40\n1\n1\nhi\n"
    "0\nENDSEC\n0\nEOF\n";

}  // namespace

TEST_CASE("FileSession: dirty tracking follows the undo stack", "[file_session]") {
    TempDir dir;
    FileSession s;
    CHECK_FALSE(s.isDirty());
    CHECK(s.displayName() == "Untitled");
    CHECK(s.title("Carvetto") == "Untitled - Carvetto");
    addCircle(s.document());
    CHECK(s.isDirty());
    CHECK(s.title("Carvetto") == "Untitled* - Carvetto");

    REQUIRE(s.saveAs(dir / "a.carv"));
    CHECK_FALSE(s.isDirty());
    CHECK(s.title("Carvetto") == "a.carv - Carvetto");

    addCircle(s.document(), {10.0, 10.0}, 2.0);
    CHECK(s.isDirty());
    REQUIRE(s.document().undo());
    CHECK_FALSE(s.isDirty());  // undone back to the saved state
    REQUIRE(s.document().redo());
    CHECK(s.isDirty());
    REQUIRE(s.document().undo());
    REQUIRE(s.document().undo());  // before the saved state
    CHECK(s.isDirty());
    REQUIRE(s.document().redo());
    CHECK_FALSE(s.isDirty());
}

TEST_CASE("FileSession: an edit after saving never merges into the saved step", "[file_session]") {
    TempDir dir;
    FileSession s;
    const ObjectId id = addCircle(s.document());
    // Two transforms of the same objects coalesce into one step while a run is open.
    REQUIRE(s.document().execute(std::make_unique<TransformObjectsCommand>(
        std::vector<ObjectId>{id}, geometry::Transform2::translation({1.0, 0.0}))));
    REQUIRE(s.saveAs(dir / "a.carv"));
    REQUIRE(s.document().execute(std::make_unique<TransformObjectsCommand>(
        std::vector<ObjectId>{id}, geometry::Transform2::translation({1.0, 0.0}))));
    CHECK(s.isDirty());
    REQUIRE(s.document().undo());
    CHECK_FALSE(s.isDirty());  // back to the saved geometry, not past it
}

TEST_CASE("FileSession: save as changes the path and the recent list", "[file_session]") {
    TempDir dir;
    FileSession s;
    addCircle(s.document());
    CHECK_FALSE(s.save());  // untitled
    CHECK_FALSE(s.lastError().empty());
    REQUIRE(s.saveAs(dir / "one.carv"));
    CHECK(s.path() == dir / "one.carv");
    REQUIRE(s.saveAs(dir / "two.carv"));
    CHECK(s.path() == dir / "two.carv");
    CHECK(s.lastError().empty());
    CHECK(fs::exists(dir / "one.carv"));
    REQUIRE(s.recentFiles().size() == 2);
    CHECK(s.recentFiles()[0] == dir / "two.carv");
    CHECK(s.recentFiles()[1] == dir / "one.carv");
    CHECK(viewmodel::withCarvExtension("x") == fs::path("x.carv"));
    CHECK(viewmodel::withCarvExtension("x.cv") == fs::path("x.cv"));
}

TEST_CASE("FileSession: open, modify, save, reopen", "[file_session]") {
    TempDir dir;
    {
        FileSession s;
        addCircle(s.document());
        REQUIRE(s.saveAs(dir / "a.carv"));
    }
    FileSession s;
    REQUIRE(s.open(dir / "a.carv"));
    CHECK_FALSE(s.isDirty());
    CHECK(s.document().objectCount() == 1);
    addCircle(s.document(), {20.0, 20.0}, 3.0);
    REQUIRE(s.save());
    FileSession again;
    REQUIRE(again.open(dir / "a.carv"));
    CHECK(again.document().objectCount() == 2);
    CHECK(again.recentFiles().size() == 1);
}

TEST_CASE("FileSession: a failed save leaves the old file intact", "[file_session]") {
    TempDir dir;
    FileSession s;
    addCircle(s.document());
    REQUIRE(s.saveAs(dir / "a.carv"));
    const std::string before = slurp(dir / "a.carv");
    REQUIRE_FALSE(before.empty());

    // A layer name past the format's string limit cannot be saved.
    Layer layer = *s.document().findLayer(s.document().activeLayer());
    layer.name = std::string(std::size_t{1} << 21, 'x');
    REQUIRE(s.document().execute(std::make_unique<SetLayerPropertiesCommand>(layer)));
    CHECK(s.isDirty());
    CHECK_FALSE(s.save());
    CHECK_FALSE(s.lastError().empty());
    CHECK(s.isDirty());
    CHECK(slurp(dir / "a.carv") == before);
    CHECK_FALSE(fs::exists(dir / "a.carv.tmp"));

    // Saving into a directory that does not exist fails and keeps the old path.
    CHECK_FALSE(s.saveAs(dir / "missing" / "b.carv"));
    CHECK(s.path() == dir / "a.carv");
}

TEST_CASE("FileSession: recent list de-duplicates and caps", "[file_session]") {
    FileSession s;
    for (std::size_t i = 0; i < viewmodel::kMaxRecentFiles + 5; ++i) {
        s.addRecent(fs::path("/tmp") / ("f" + std::to_string(i) + ".carv"));
    }
    REQUIRE(s.recentFiles().size() == viewmodel::kMaxRecentFiles);
    CHECK(s.recentFiles().front() == fs::path("/tmp/f14.carv"));
    s.addRecent("/tmp/./f12.carv");  // same file spelled differently moves to the front
    CHECK(s.recentFiles().size() == viewmodel::kMaxRecentFiles);
    CHECK(s.recentFiles().front() == fs::path("/tmp/./f12.carv"));
    CHECK(s.recentFiles()[1] == fs::path("/tmp/f14.carv"));
    std::size_t n12 = 0;
    for (const auto& p : s.recentFiles()) {
        n12 += p.filename() == "f12.carv" ? 1U : 0U;
    }
    CHECK(n12 == 1);
    s.removeRecent("/tmp/f14.carv");
    CHECK(s.recentFiles().size() == viewmodel::kMaxRecentFiles - 1);
    s.setRecentFiles({"/a/x.carv", "/a/y.carv", "/a/x.carv"});
    CHECK(s.recentFiles().size() == 2);
    s.clearRecent();
    CHECK(s.recentFiles().empty());
}

TEST_CASE("FileSession: bad files report an error and keep the document", "[file_session]") {
    TempDir dir;
    FileSession s;
    addCircle(s.document());
    REQUIRE(s.saveAs(dir / "good.carv"));
    addCircle(s.document(), {1.0, 1.0}, 1.0);  // unsaved edit that must survive a failed open
    const std::string good = slurp(dir / "good.carv");

    spit(dir / "garbage.carv", "this is not a zip archive at all");
    spit(dir / "empty.carv", "");
    spit(dir / "truncated.carv", good.substr(0, good.size() / 2));
    std::string flipped = good;
    for (std::size_t i = 0; i < flipped.size(); i += 7) {
        flipped[i] = static_cast<char>(flipped[i] ^ 0x5A);
    }
    spit(dir / "flipped.carv", flipped);

    for (const char* name : {"garbage.carv", "empty.carv", "truncated.carv", "flipped.carv", "absent.carv"}) {
        INFO(name);
        s.addRecent(dir / name);
        CHECK_FALSE(s.open(dir / name));
        CHECK_FALSE(s.lastError().empty());
        CHECK(s.document().objectCount() == 2);
        CHECK(s.path() == dir / "good.carv");
        CHECK(s.isDirty());
        for (const auto& p : s.recentFiles()) {
            CHECK(p != dir / name);  // failed files drop out of the recent list
        }
    }
}

TEST_CASE("FileSession: DXF import is one undoable step", "[file_session][dxf]") {
    TempDir dir;
    spit(dir / "sq.dxf", kSquareDxf);
    FileSession s;
    addCircle(s.document());
    REQUIRE(s.saveAs(dir / "a.carv"));
    const std::size_t steps = s.document().undoStack().count();

    REQUIRE(s.importDxf(dir / "sq.dxf"));
    CHECK(s.isDirty());
    CHECK(s.path() == dir / "a.carv");
    CHECK(s.document().undoStack().count() == steps + 1);
    CHECK(s.lastImport().objectsAdded == 2);
    CHECK(s.lastImport().layersAdded == 1);
    CHECK(s.lastImport().textSkipped == 1);
    CHECK_FALSE(s.lastImport().warnings.empty());
    CHECK(s.document().objectCount() == 3);
    CHECK(s.document().selection().size() == 2);
    // Layers are kept: the DXF layer "CUT" was created in addition to the default one.
    REQUIRE(s.document().layers().size() == 2);
    CHECK(s.document().layers()[1].name == "CUT");
    CHECK(s.document().layers()[1].colour == Colour{255, 0, 0});
    // $INSUNITS = 1 (inches): the 2-unit line is 50.8 mm long.
    const Object* first = s.document().findObject(s.document().selection().ids().front());
    REQUIRE(first != nullptr);
    CHECK(first->asVector()->contours.front().length() == Catch::Approx(50.8));
    CHECK(s.document().checkIntegrity().empty());

    // Importing again reuses the layer by name.
    REQUIRE(s.importDxf(dir / "sq.dxf", 1.0));
    CHECK(s.document().layers().size() == 2);
    CHECK(s.document().findObject(s.document().selection().ids().front())->asVector()->contours.front().length() ==
          Catch::Approx(2.0));

    REQUIRE(s.document().undo());
    REQUIRE(s.document().undo());
    CHECK_FALSE(s.isDirty());
    CHECK(s.document().objectCount() == 1);
    CHECK(s.document().layers().size() == 1);
    REQUIRE(s.document().redo());
    CHECK(s.document().objectCount() == 3);
}

TEST_CASE("FileSession: unreadable DXF leaves the document untouched", "[file_session][dxf]") {
    TempDir dir;
    spit(dir / "bad.dxf", "not a dxf file\n");
    FileSession s;
    addCircle(s.document());
    const std::size_t steps = s.document().undoStack().count();
    CHECK_FALSE(s.importDxf(dir / "bad.dxf"));
    CHECK_FALSE(s.lastError().empty());
    CHECK_FALSE(s.importDxf(dir / "none.dxf"));
    CHECK(s.document().objectCount() == 1);
    CHECK(s.document().undoStack().count() == steps);

    spit(dir / "empty.dxf", "0\nSECTION\n2\nENTITIES\n0\nENDSEC\n0\nEOF\n");
    CHECK(s.importDxf(dir / "empty.dxf"));
    CHECK(s.lastImport().objectsAdded == 0);
    CHECK_FALSE(s.lastImport().warnings.empty());
    CHECK(s.document().undoStack().count() == steps);
}

TEST_CASE("FileSession: DXF export round-trips through importDxf", "[file_session][dxf]") {
    TempDir dir;
    FileSession s;
    Document& doc = s.document();
    const ObjectId circle = addCircle(doc, {40.0, 30.0}, 12.5);
    geometry::Contour poly;
    poly.spans.emplace_back(geometry::LineSpan{{0.0, 0.0}, {30.0, 0.0}});
    poly.spans.emplace_back(geometry::LineSpan{{30.0, 0.0}, {30.0, 20.0}});
    poly.spans.emplace_back(geometry::LineSpan{{30.0, 20.0}, {0.0, 0.0}});
    poly.closed = true;
    const geometry::Contour rect = geometry::Contour::rectangle(geometry::Box2{{5.0, 5.0}, {25.0, 15.0}});
    REQUIRE(doc.execute(std::make_unique<AddObjectCommand>(
        Object::vector({poly, rect}, doc.activeLayer(), doc.activeSheet()))));

    auto roundTrip = [&](bool selectionOnly) {
        const fs::path out = dir / (selectionOnly ? "sel.dxf" : "all.dxf");
        REQUIRE(s.exportDxf(out, selectionOnly));
        const std::string text = slurp(out);
        auto imported = io::importDxf(std::span<const std::uint8_t>(
            reinterpret_cast<const std::uint8_t*>(text.data()), text.size()));
        REQUIRE(imported.ok());
        return imported.value();
    };

    const io::DxfDrawing all = roundTrip(false);
    REQUIRE(all.contours.size() == 3);
    CHECK(all.unitScale == 1.0);
    const geometry::Contour& c0 = all.contours[0].contour;
    CHECK(c0.length() == Catch::Approx(geometry::Contour::circle({40.0, 30.0}, 12.5).length()).margin(1e-6));
    CHECK(c0.bounds().min.x == Catch::Approx(27.5).margin(1e-6));
    CHECK(c0.bounds().max.y == Catch::Approx(42.5).margin(1e-6));
    CHECK(all.contours[1].contour.closed);
    CHECK(all.contours[1].contour.length() == Catch::Approx(poly.length()).margin(1e-6));
    CHECK(all.contours[2].contour.length() == Catch::Approx(60.0).margin(1e-6));

    doc.selection().assign({circle});
    const io::DxfDrawing sel = roundTrip(true);
    REQUIRE(sel.contours.size() == 1);
    CHECK(sel.contours[0].contour.length() == Catch::Approx(c0.length()).margin(1e-6));

    // Exporting does not touch the path or the dirty state; nothing selected is an error.
    CHECK_FALSE(s.hasPath());
    doc.selection().clear();
    CHECK_FALSE(s.exportDxf(dir / "none.dxf", true));
    CHECK(s.lastError().find("selection") != std::string::npos);
    CHECK_FALSE(fs::exists(dir / "none.dxf"));

    // Export then import into a job: same geometry again.
    FileSession t;
    REQUIRE(t.importDxf(dir / "all.dxf"));
    CHECK(t.document().objectCount() == 3);
}

TEST_CASE("dxfUnitChoices: first is the file, the rest are forced scales", "[file_session][dxf]") {
    const auto& c = viewmodel::dxfUnitChoices();
    REQUIRE(c.size() >= 4);
    CHECK_FALSE(c[0].mmPerUnit.has_value());
    CHECK(c[1].mmPerUnit == 1.0);
    CHECK(c[4].mmPerUnit == 25.4);
}

TEST_CASE("FileSession: recent list collapses a symlinked folder and its target", "[file_session]") {
    TempDir dir;
    fs::create_directories(dir / "real");
    std::error_code ec;
    fs::create_directory_symlink(dir / "real", dir / "link", ec);
    if (ec) {
        SUCCEED("symlinks are not supported here; skipped");
        return;
    }
    FileSession s;
    s.addRecent(dir / "real" / "a.carv");
    s.addRecent(dir / "link" / "a.carv");
    CHECK(s.recentFiles().size() == 1);
    CHECK(s.recentFiles().front() == dir / "link" / "a.carv");
}

TEST_CASE("FileSession: opening a folder says so in plain words", "[file_session]") {
    TempDir dir;
    fs::create_directories(dir / "folder.carv");
    FileSession s;
    CHECK_FALSE(s.open(dir / "folder.carv"));
    CHECK(s.lastError().find("folder") != std::string::npos);
    CHECK(s.lastError().find("basic_filebuf") == std::string::npos);
    CHECK_FALSE(s.importDxf(dir / "folder.carv"));
    CHECK(s.lastError().find("folder") != std::string::npos);
    CHECK_FALSE(s.open(dir / "nothing.carv"));
    CHECK(s.lastError().find("does not exist") != std::string::npos);
}

TEST_CASE("FileSession: DXF export writes only layers that hold geometry", "[file_session][dxf]") {
    TempDir dir;
    FileSession s;
    Document& doc = s.document();
    Layer empty;
    empty.name = "EmptyA";
    REQUIRE(doc.execute(std::make_unique<AddLayerCommand>(empty)));
    Layer used;
    used.name = "Used";
    auto cmd = std::make_unique<AddLayerCommand>(used, true);
    const AddLayerCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    REQUIRE(doc.activeLayer() == raw->id());
    addCircle(doc);
    addCircle(doc, {5.0, 5.0}, 1.0);
    REQUIRE(s.exportDxf(dir / "l.dxf", false));
    const std::string text = slurp(dir / "l.dxf");
    CHECK(text.find("Used") != std::string::npos);
    CHECK(text.find("EmptyA") == std::string::npos);
    // One layer entry for two contours on the same layer.
    std::size_t at = 0;
    std::size_t entries = 0;
    while ((at = text.find("\nUsed\n", at)) != std::string::npos) {
        ++entries;
        ++at;
    }
    CHECK(entries >= 3);  // table entry + two entities
}
