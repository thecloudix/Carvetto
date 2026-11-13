// T-0275: the Image Engrave form, bitmap import and the session around the T-0250 core: the bitmap
// is read from the selection, the record names it, one undo step, save/load, staleness, cancel and
// the work cap.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "carvetto/document/bitmap_store.hpp"
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/geometry/transform2.hpp"
#include "carvetto/viewmodel/bitmap_import.hpp"
#include "carvetto/viewmodel/canvas_scene.hpp"
#include "toolpath_test_support.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using namespace carvetto::viewmodel::testing;
using Catch::Approx;
using document::Document;
using document::ToolpathOperation;
using document::Units;

namespace {

const ToolpathField* find(const std::vector<ToolpathField>& fields, const std::string& id) {
    for (const ToolpathField& f : fields) {
        if (f.id == id) {
            return &f;
        }
    }
    return nullptr;
}

/// A binary PGM of `w` x `h`, gray value `value(x, y)`.
template <class F>
std::vector<std::uint8_t> pgm(int w, int h, F value) {
    const std::string head = "P5\n" + std::to_string(w) + " " + std::to_string(h) + "\n255\n";
    std::vector<std::uint8_t> out(head.begin(), head.end());
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            out.push_back(static_cast<std::uint8_t>(value(x, y)));
        }
    }
    return out;
}

/// Left half black, right half white.
std::vector<std::uint8_t> halfAndHalf() {
    return pgm(8, 4, [](int x, int) { return x < 4 ? 0 : 255; });
}

document::ObjectId importHalf(Document& doc) {
    const BitmapImportOutcome out = importBitmapIntoDocument(doc, halfAndHalf(), "half.pgm", {});
    INFO(out.error);
    REQUIRE(out.ok);
    return out.object;
}

ToolpathDefinition photoFor(const tooling::ToolDatabase& db, const Document& doc) {
    ToolpathForm form(ToolpathOperation::ImageEngrave, Units::Millimetres);
    REQUIRE(form.setTool(db, source("v-bit-90")).ok);
    form.setSelector(VectorSelector::fromSelection(doc));
    return form.definition();
}

double minZ(const document::ToolpathRecord& rec) {
    double z = 0.0;
    for (const toolpath::Move& m : *rec.moves) {
        z = std::min(z, m.to.z);
    }
    return z;
}

}  // namespace

TEST_CASE("photo form: fields, choices and bounds", "[toolpath][form][photo]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::ImageEngrave, Units::Millimetres);
    CHECK(toolsFor(db, ToolpathOperation::ImageEngrave).size() >= 1);
    CHECK_FALSE(form.setTool(db, source("end-mill-3mm")).ok);
    CHECK_FALSE(form.setTool(db, source("drill-3mm")).ok);
    REQUIRE(form.setTool(db, source("v-bit-90")).ok);
    CHECK(form.problems().empty());

    for (const char* id : {"maxDepth", "stepoverRetract", "safeZ", "lineSpacing", "lineAngle"}) {
        CHECK(find(form.fields(), id) != nullptr);
    }
    CHECK(find(form.fields(), "chordal") == nullptr);  // Selected Vectors only
    const auto choices = form.choices();
    CHECK(std::any_of(choices.begin(), choices.end(), [](const ToolpathChoice& c) { return c.id == "strategy"; }));
    CHECK(std::any_of(choices.begin(), choices.end(), [](const ToolpathChoice& c) { return c.id == "invert" && c.isBool; }));

    REQUIRE(form.setField("maxDepth", "1.5").ok);
    REQUIRE(form.setField("stepoverRetract", "0").ok);
    REQUIRE(form.setField("lineSpacing", "35").ok);
    REQUIRE(form.setField("lineAngle", "45").ok);
    REQUIRE(form.setChoice("invert", 1).ok);
    const auto& p = std::get<toolpath::ImageEngraveParams>(form.definition().params);
    CHECK(p.maxDepth == Approx(1.5));
    CHECK(p.stepoverRetract == 0.0);
    CHECK(p.lineSpacingPercent == Approx(35.0));
    CHECK(p.lineAngle == Approx(0.7853981633974483));
    CHECK(p.invert);

    CHECK_FALSE(form.setField("maxDepth", "0").ok);
    CHECK_FALSE(form.setField("maxDepth", "-1").ok);
    CHECK_FALSE(form.setField("lineSpacing", "0").ok);
    CHECK_FALSE(form.setField("lineSpacing", "5000").ok);
    CHECK_FALSE(form.setField("stepoverRetract", "-0.1").ok);
    CHECK_FALSE(form.setField("lineAngle", "abc").ok);
    CHECK_FALSE(form.setChoice("strategy", 3).ok);

    REQUIRE(form.setChoice("strategy", 2).ok);  // Selected vectors
    CHECK(find(form.fields(), "lineAngle") == nullptr);
    CHECK(find(form.fields(), "chordal") != nullptr);
    REQUIRE(form.setChoice("strategy", 1).ok);  // Hatch
    CHECK(find(form.fields(), "lineAngle") != nullptr);
}

TEST_CASE("bitmap import: one undo step, embedded, fitted to the material, selected", "[bitmap][import]") {
    Document doc;
    doc.undoStack().clear();
    const document::ObjectId id = importHalf(doc);
    const document::Object* obj = doc.findObject(id);
    REQUIRE(obj != nullptr);
    REQUIRE(obj->asBitmap() != nullptr);
    const document::BitmapObject& b = *obj->asBitmap();
    CHECK(b.widthPx == 8);
    CHECK(b.heightPx == 4);
    CHECK(b.placementMm.width() / b.placementMm.height() == Approx(2.0));
    const geometry::Box2 material = doc.findSheet(doc.activeSheet())->materialBoundsMm();
    CHECK(b.placementMm.width() <= material.width() + 1e-9);
    CHECK(b.placementMm.height() <= material.height() + 1e-9);
    CHECK(doc.findBitmap(b.sourceRef) != nullptr);
    CHECK(doc.selection().ids() == std::vector<document::ObjectId>{id});
    CHECK(doc.checkIntegrity().empty());

    // The canvas scene lists it, with its placement in pixels.
    {
        ViewTransform view;
        view.setViewport(800.0, 600.0);
        view.zoomToFit(material);
        const CanvasScene scene = buildCanvasScene(doc, view);
        REQUIRE(scene.bitmaps.size() == 1);
        CHECK(scene.bitmaps[0].object == id);
        CHECK(scene.bitmaps[0].key == b.sourceRef);
        CHECK(scene.bitmaps[0].rectPx.width() / scene.bitmaps[0].rectPx.height() == Approx(2.0).epsilon(1e-6));
    }

    // The same image again shares the embedded bytes.
    const BitmapImportOutcome again = importBitmapIntoDocument(doc, halfAndHalf(), "again.pgm", {});
    REQUIRE(again.ok);
    CHECK(doc.bitmaps().size() == 1);

    REQUIRE(doc.undo());
    REQUIRE(doc.undo());
    CHECK(doc.findObject(id) == nullptr);
    CHECK(doc.bitmaps().empty());
}

TEST_CASE("bitmap import: refusals change nothing", "[bitmap][import]") {
    Document doc;
    const std::uint64_t generation = doc.undoStack().generation();
    CHECK_FALSE(importBitmapIntoDocument(doc, {}, "empty", {}).ok);
    const std::vector<std::uint8_t> junk{1, 2, 3, 4, 5};
    const BitmapImportOutcome noDecoder = importBitmapIntoDocument(doc, junk, "junk.png", {});
    CHECK_FALSE(noDecoder.ok);
    CHECK_FALSE(noDecoder.error.empty());
    const document::BitmapDecoder failing = [](std::span<const std::uint8_t>) -> geometry::ShapeResult<io::RasterImage> {
        return std::string("not an image");
    };
    CHECK_FALSE(importBitmapIntoDocument(doc, junk, "junk.png", failing).ok);
    const auto truncated = std::vector<std::uint8_t>{'P', '5', '\n', '9', '9', ' ', '9', '9', '\n', '2', '5', '5', '\n', 1};
    CHECK_FALSE(importBitmapIntoDocument(doc, truncated, "short.pgm", {}).ok);
    CHECK(doc.undoStack().generation() == generation);
    CHECK(doc.objectCount() == 0);

    // A hidden layer takes no import.
    document::Layer hidden;
    hidden.name = "hidden";
    hidden.visible = false;
    const document::LayerId layer = doc.insertLayer(hidden);
    doc.setActiveLayer(layer);
    CHECK_FALSE(importBitmapIntoDocument(doc, halfAndHalf(), "half.pgm", {}).ok);
}

TEST_CASE("photo session: carve the selected bitmap, one undo step, save, load, recalculate",
          "[toolpath][session][photo]") {
    namespace fs = std::filesystem;
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    const document::ObjectId bitmap = importHalf(doc);
    doc.undoStack().clear();

    ToolpathDefinition def = photoFor(db, doc);
    auto& params = std::get<toolpath::ImageEngraveParams>(def.params);
    params.maxDepth = 1.0;
    params.lineSpacingPercent = 100.0;
    const ToolpathOutcome out = session.add(def);
    INFO(out.message);
    REQUIRE(out.ok);
    REQUIRE(doc.toolpaths().size() == 1);
    const document::ToolpathRecord& rec = doc.toolpaths().front();
    CHECK(rec.operation() == ToolpathOperation::ImageEngrave);
    CHECK(rec.selection.objects == std::vector<document::ObjectId>{bitmap});
    REQUIRE(rec.hasMoves());
    CHECK(minZ(rec) == Approx(-1.0).margin(1e-6));

    // Dark left half cuts, white right half stays at the surface.
    const geometry::Box2 box = doc.findObject(bitmap)->asBitmap()->placementMm;
    const double splitX = 0.5 * (box.min.x + box.max.x);
    bool deepLeft = false;
    for (const toolpath::Move& m : *rec.moves) {
        if (m.kind == toolpath::MoveKind::Feed && m.to.x > splitX + 0.1 * box.width()) {
            CHECK(m.to.z > -1e-6);
        }
        if (m.kind == toolpath::MoveKind::Feed && m.to.x < splitX - 0.1 * box.width() && m.to.z < -0.9) {
            deepLeft = true;
        }
    }
    CHECK(deepLeft);

    // Invert swaps the halves.
    {
        ToolpathDefinition inv = def;
        std::get<toolpath::ImageEngraveParams>(inv.params).invert = true;
        const ToolpathCalculation calc = session.prepare(inv).run();
        REQUIRE(calc.ok);
        bool deepRight = false;
        for (const toolpath::Move& m : *calc.items.front().record.moves) {
            if (m.kind == toolpath::MoveKind::Feed && m.to.x > splitX + 0.1 * box.width() && m.to.z < -0.9) {
                deepRight = true;
            }
            if (m.kind == toolpath::MoveKind::Feed && m.to.x < splitX - 0.1 * box.width()) {
                CHECK(m.to.z > -1e-6);
            }
        }
        CHECK(deepRight);
    }

    CHECK(doc.checkIntegrity().empty());
    REQUIRE(session.list().size() == 1);
    CHECK(session.list().front().status == ToolpathStatus::Current);

    // Save, load, and the record and its staleness survive.
    const fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / "photo";
    fs::create_directories(dir);
    REQUIRE(document::saveCarv(doc, dir / "t.carv", {}));
    auto loaded = document::loadCarv(dir / "t.carv");
    REQUIRE(loaded);
    Document& doc2 = loaded.value().document;
    CHECK(doc2.toolpaths() == doc.toolpaths());
    ToolpathSession s2(&doc2);
    s2.setToolDatabase(db);
    const auto list = s2.list();
    REQUIRE(list.size() == 1);
    CHECK(list[0].status == ToolpathStatus::Current);
    const std::vector<document::ToolpathRecord> before = doc2.toolpaths();
    REQUIRE(s2.recalculate(list[0].id).ok);
    CHECK(doc2.toolpaths() == before);

    // One undo step removes the toolpath, the bitmap stays.
    REQUIRE(doc.undo());
    CHECK(doc.toolpaths().empty());
    CHECK(doc.findObject(bitmap) != nullptr);
    REQUIRE(doc.redo());
    CHECK(doc.toolpaths().size() == 1);
}

TEST_CASE("photo session: edit recalculates in one undo step; moving the bitmap makes it stale",
          "[toolpath][session][photo]") {
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    const document::ObjectId bitmap = importHalf(doc);
    doc.undoStack().clear();
    ToolpathDefinition def = photoFor(db, doc);
    std::get<toolpath::ImageEngraveParams>(def.params).maxDepth = 1.0;
    REQUIRE(session.add(def).ok);
    const document::ToolpathId id = doc.toolpaths().front().id;
    CHECK(session.list().front().status == ToolpathStatus::Current);

    // Recalculate with another depth (Edit): one step.
    const std::size_t steps = doc.undoStack().index();
    ToolpathDefinition edited = *session.definition(id);
    std::get<toolpath::ImageEngraveParams>(edited.params).maxDepth = 0.5;
    REQUIRE(session.update(id, edited).ok);
    CHECK(doc.undoStack().index() == steps + 1);
    CHECK(minZ(doc.toolpaths().front()) == Approx(-0.5).margin(1e-6));
    CHECK(session.list().front().status == ToolpathStatus::Current);

    // Moving the bitmap (an undoable edit) makes the toolpath stale; undo makes it current again.
    REQUIRE(doc.execute(std::make_unique<document::TransformObjectsCommand>(
        std::vector<document::ObjectId>{bitmap}, geometry::Transform2::translation({5.0, 0.0}))));
    CHECK(session.list().front().status == ToolpathStatus::NeedsRecalculation);
    CHECK(session.list().front().vectorsChanged);
    REQUIRE(session.recalculate(id).ok);
    CHECK(session.list().front().status == ToolpathStatus::Current);
    REQUIRE(doc.undo());
    REQUIRE(doc.undo());
    CHECK(session.list().front().status == ToolpathStatus::Current);
}

TEST_CASE("photo session: refusals say what is missing", "[toolpath][session][photo]") {
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    addVector(doc, square(10, 10, 20));
    doc.undoStack().clear();

    // No bitmap selected.
    ToolpathDefinition def = photoDefinition(db);
    def.selector = VectorSelector::allVectors();
    ToolpathOutcome out = session.add(def);
    CHECK_FALSE(out.ok);
    CHECK(out.message.find("bitmap") != std::string::npos);
    CHECK(doc.toolpaths().empty());

    // A bitmap with only a legacy reference (no pixels).
    document::Object legacy;
    legacy.layer = doc.defaultLayer();
    legacy.sheet = doc.activeSheet();
    document::BitmapObject b;
    b.widthPx = 2;
    b.heightPx = 2;
    b.placementMm = geometry::Box2{{0, 0}, {10, 10}};
    b.sourceRef = "legacy-name";
    legacy.data = b;
    const document::ObjectId legacyId = doc.insertObject(std::move(legacy));
    def.selector = VectorSelector::fromObjects({legacyId});
    out = session.add(def);
    CHECK_FALSE(out.ok);
    CHECK(out.message.find("embedded") != std::string::npos);

    // A hidden layer's bitmap.
    const document::ObjectId bitmap = importHalf(doc);
    doc.selection().assign({bitmap});
    const document::Object* obj = doc.findObject(bitmap);
    document::Layer l = *doc.findLayer(obj->layer);
    l.visible = false;
    REQUIRE(doc.execute(std::make_unique<document::SetLayerPropertiesCommand>(l)));
    def.selector = VectorSelector::fromObjects({bitmap});
    out = session.add(def);
    CHECK_FALSE(out.ok);
    CHECK(out.message.find("hidden") != std::string::npos);
    l.visible = true;
    REQUIRE(doc.execute(std::make_unique<document::SetLayerPropertiesCommand>(l)));

    // Selected Vectors needs vectors beside the bitmap.
    std::get<toolpath::ImageEngraveParams>(def.params).strategy = toolpath::PhotoStrategy::SelectedVectors;
    out = session.add(def);
    CHECK_FALSE(out.ok);
    CHECK(out.message.find("Selected Vectors") != std::string::npos);

    CHECK(doc.toolpaths().empty());
    CHECK(doc.checkIntegrity().empty());
}

TEST_CASE("photo session: Selected Vectors follows the chosen vectors; Hatch adds a second pass",
          "[toolpath][session][photo]") {
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    const document::ObjectId bitmap = importHalf(doc);
    const geometry::Box2 box = doc.findObject(bitmap)->asBitmap()->placementMm;
    const geometry::Vec2 mid{0.5 * (box.min.x + box.max.x), 0.5 * (box.min.y + box.max.y)};
    geometry::Contour line;
    line.closed = false;
    line.spans.push_back(geometry::LineSpan{{box.min.x + 0.05 * box.width(), mid.y}, {box.max.x - 0.05 * box.width(), mid.y}});
    const document::ObjectId vec = addVector(doc, line);
    doc.undoStack().clear();

    ToolpathDefinition def = photoDefinition(db);
    std::get<toolpath::ImageEngraveParams>(def.params).strategy = toolpath::PhotoStrategy::SelectedVectors;
    std::get<toolpath::ImageEngraveParams>(def.params).maxDepth = 1.0;
    def.selector = VectorSelector::fromObjects({bitmap, vec});
    const ToolpathOutcome out = session.add(def);
    INFO(out.message);
    REQUIRE(out.ok);
    const document::ToolpathRecord& rec = doc.toolpaths().front();
    CHECK(rec.selection.objects == std::vector<document::ObjectId>{bitmap, vec});
    CHECK(minZ(rec) == Approx(-1.0).margin(1e-6));
    for (const toolpath::Move& m : *rec.moves) {
        if (m.kind == toolpath::MoveKind::Feed) {
            CHECK(m.to.y == Approx(mid.y).margin(1e-9));  // only along the vector
        }
    }
    CHECK(session.list().front().status == ToolpathStatus::Current);

    // Hatch cuts more than Raster over the same image.
    ToolpathDefinition raster = photoDefinition(db);
    raster.selector = VectorSelector::fromObjects({bitmap});
    ToolpathDefinition hatch = raster;
    std::get<toolpath::ImageEngraveParams>(hatch.params).strategy = toolpath::PhotoStrategy::Hatch;
    const ToolpathCalculation r = session.prepare(raster).run();
    const ToolpathCalculation h = session.prepare(hatch).run();
    REQUIRE(r.ok);
    REQUIRE(h.ok);
    CHECK(h.items.front().record.moveCount() > r.items.front().record.moveCount());
}

TEST_CASE("photo session: cancel and work cap", "[toolpath][session][photo]") {
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    importHalf(doc);
    doc.undoStack().clear();

    ToolpathDefinition def = photoFor(db, doc);
    // Cancelled before and during: nothing produced, nothing stored.
    {
        const ToolpathJob job = session.prepare(def);
        std::atomic<bool> cancel{true};
        const ToolpathCalculation calc = job.run(&cancel);
        CHECK(calc.cancelled);
        CHECK_FALSE(calc.ok);
        CHECK(calc.items.empty());
        CHECK_FALSE(session.apply(calc).ok);
        CHECK(doc.toolpaths().empty());
    }
    // A job that decodes on the worker honours a flag set while it runs (via the decoder).
    {
        std::atomic<bool> cancel{false};
        ToolpathJob job = session.prepare(def);
        REQUIRE(job.photo.has_value());
        job.photo->decoder = [&cancel](std::span<const std::uint8_t>) -> geometry::ShapeResult<io::RasterImage> {
            cancel = true;
            io::RasterImage img;
            img.width = 1;
            img.height = 1;
            img.rgb = {0, 0, 0};
            return img;
        };
        // Netpbm bytes are read by core without the decoder: use bytes only the decoder can handle.
        job.photo->bytes = {1, 2, 3};
        const ToolpathCalculation calc = job.run(&cancel);
        CHECK(calc.cancelled);
        CHECK(calc.items.empty());
    }
    // The work cap: far more samples than the limit are refused before generating.
    {
        ToolpathDefinition big = def;
        auto& p = std::get<toolpath::ImageEngraveParams>(big.params);
        p.lineSpacingPercent = 0.01;
        ToolpathJob job = session.prepare(big);
        job.photo->bytes = pgm(1000, 1000, [](int x, int y) { return (x + y) % 256; });
        const ToolpathCalculation calc = job.run();
        CHECK_FALSE(calc.ok);
        CHECK(calc.hasError());
        CHECK(std::any_of(calc.messages.begin(), calc.messages.end(), [](const ToolpathMessage& m) {
            return m.issue == toolpath::ToolpathIssue::WorkLimitExceeded;
        }));
        CHECK(calc.items.empty());
    }
    CHECK(doc.toolpaths().empty());
}

TEST_CASE("photo session: a decoder supplies non-Netpbm images", "[toolpath][session][photo]") {
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    int calls = 0;
    const document::BitmapDecoder decoder = [&calls](std::span<const std::uint8_t> bytes) -> geometry::ShapeResult<io::RasterImage> {
        ++calls;
        io::RasterImage img;
        img.width = 4;
        img.height = 4;
        img.rgb.assign(48, static_cast<std::uint8_t>(bytes[0]));  // gray = first byte
        return img;
    };
    session.setBitmapDecoder(decoder);
    const std::vector<std::uint8_t> fake{0, 9, 9, 9};  // "0" = black everywhere
    const BitmapImportOutcome in = importBitmapIntoDocument(doc, fake, "fake.png", decoder);
    REQUIRE(in.ok);
    doc.undoStack().clear();
    calls = 0;
    ToolpathDefinition def = photoFor(db, doc);
    std::get<toolpath::ImageEngraveParams>(def.params).maxDepth = 1.0;
    REQUIRE(session.add(def).ok);
    CHECK(calls == 1);
    // A flat black image carves a constant depth.
    CHECK(minZ(doc.toolpaths().front()) == Approx(-1.0).margin(1e-6));
}
