// T-0275: photo V-carve toolpaths as native toolpath records (ADR-0021 schema, ADR-0047/ADR-0052): the
// PhotoVCarveParams variant alternative, its job.json params, exact round trips, non-finite refusal.

#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <variant>
#include <vector>

#include "carvetto/document/bitmap_store.hpp"
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/toolpath_record.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/photo_vcarve.hpp"

using namespace carvetto;
using namespace carvetto::document;

namespace {

toolpath::PhotoVCarveParams fullPhoto(toolpath::PhotoStrategy strategy) {
    toolpath::PhotoVCarveParams p;
    p.name = "Photo \"carve\" \xC3\xA9";
    p.maxDepth = 1.75;
    p.invert = true;
    p.stepoverRetract = 0.5;
    p.lineSpacingPercent = 12.5;
    p.lineAngle = 0.5235987755982988;
    p.strategy = strategy;
    p.safeZ = 7.0;
    p.depthTolerance = 0.02;
    p.tolerance = {0.002, 1.7453292519943296e-05, 0.02};
    return p;
}

tooling::Tool vBit() {
    tooling::Tool t;
    t.id = "vb";
    t.name = "V bit";
    t.type = tooling::ToolType::VBit;
    t.diameter = 6.0;
    t.includedAngle = 1.5707963267948966;
    t.defaultCuttingData = {1.0, 0.2, 0.0, 0.0, 18000.0, 1500.0, 500.0, 0.0, 1};
    return t;
}

/// A document holding an embedded 4x2 PGM bitmap and a photo toolpath over it.
Document photoDocument(const toolpath::PhotoVCarveParams& params) {
    Document doc;
    BitmapImport in;
    const std::string pgm = "P5\n4 2\n255\n";
    in.bytes.assign(pgm.begin(), pgm.end());
    for (int i = 0; i < 8; ++i) {
        in.bytes.push_back(static_cast<std::uint8_t>(i * 30));
    }
    in.widthPx = 4;
    in.heightPx = 2;
    in.placementMm = geometry::Box2{{0, 0}, {20, 10}};
    in.layer = doc.defaultLayer();
    in.sheet = doc.activeSheet();
    in.name = "photo.pgm";
    REQUIRE(doc.execute(makeImportBitmapCommand(doc, in)));
    ObjectId bitmap;
    for (const auto& [id, obj] : doc.objects()) {
        bitmap = id;
    }
    toolpath::GrayImage gray;
    gray.width = 4;
    gray.height = 2;
    gray.pixels = {0, 30, 60, 90, 120, 150, 180, 210};
    toolpath::PhotoVCarveResult r =
        toolpath::photoVCarve(gray, geometry::Box2{{0, 0}, {20, 10}}, vBit(), std::nullopt, params);
    REQUIRE(r.ok());
    ToolpathRecord rec;
    rec.name = r.toolpath.name;
    rec.params = params;
    rec.selection.objects = {bitmap};
    rec.tool = r.toolpath.tool;
    rec.toolSource = {"vb", "", ""};
    rec.moves = shareMoves(std::move(r.toolpath.moves));
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(rec)));
    return doc;
}

}  // namespace

TEST_CASE("T-0275: photo records save and load exactly; save -> load -> save is byte-identical",
          "[document][toolpath][carv][photo]") {
    for (const toolpath::PhotoStrategy strategy :
         {toolpath::PhotoStrategy::Raster, toolpath::PhotoStrategy::Hatch, toolpath::PhotoStrategy::SelectedVectors}) {
        const toolpath::PhotoVCarveParams params = fullPhoto(strategy);
        toolpath::PhotoVCarveParams run = params;
        run.strategy = toolpath::PhotoStrategy::Raster;  // the vectors of Selected Vectors are not needed to store it
        Document doc = photoDocument(run);
        ToolpathRecord edit = doc.toolpaths()[0];
        edit.params = params;
        if (!(edit == doc.toolpaths()[0])) {  // a command that changes nothing is refused
            REQUIRE(doc.execute(std::make_unique<SetToolpathCommand>(edit)));
        }
        REQUIRE(doc.toolpaths().size() == 1);
        CHECK(doc.toolpaths()[0].operation() == ToolpathOperation::PhotoVCarve);
        CHECK(doc.toolpaths()[0].safeZ() == 7.0);
        auto saved = saveCarv(doc, CarvExtras{});
        REQUIRE(saved.ok());
        auto back = loadCarv(std::span<const std::uint8_t>(saved.value()));
        REQUIRE(back.ok());
        REQUIRE(back.value().document.toolpaths().size() == 1);
        CHECK(back.value().document.toolpaths()[0] == doc.toolpaths()[0]);
        CHECK(std::get<toolpath::PhotoVCarveParams>(back.value().document.toolpaths()[0].params) == params);
        CHECK(back.value().document.bitmaps().size() == 1);
        auto again = saveCarv(back.value().document, back.value().extras);
        REQUIRE(again.ok());
        CHECK(again.value() == saved.value());
    }
}

TEST_CASE("T-0275: non-finite photo parameters are refused by the commands", "[document][toolpath][photo]") {
    Document doc = photoDocument(fullPhoto(toolpath::PhotoStrategy::Raster));
    ToolpathRecord edit = doc.toolpaths()[0];
    const double inf = std::numeric_limits<double>::infinity();
    auto& p = std::get<toolpath::PhotoVCarveParams>(edit.params);
    struct Field {
        double toolpath::PhotoVCarveParams::*member;
        const char* key;
    };
    const Field fields[] = {{&toolpath::PhotoVCarveParams::maxDepth, "max_depth"},
                            {&toolpath::PhotoVCarveParams::stepoverRetract, "stepover_retract"},
                            {&toolpath::PhotoVCarveParams::lineSpacingPercent, "line_spacing_percent"},
                            {&toolpath::PhotoVCarveParams::lineAngle, "line_angle_rad"},
                            {&toolpath::PhotoVCarveParams::safeZ, "safe_z"},
                            {&toolpath::PhotoVCarveParams::depthTolerance, "depth_tolerance"}};
    for (const Field& f : fields) {
        p = fullPhoto(toolpath::PhotoStrategy::Raster);
        p.*(f.member) = inf;
        CHECK(nonFiniteParam(edit.params) == f.key);
        CHECK_FALSE(doc.execute(std::make_unique<SetToolpathCommand>(edit)));
    }
    p = fullPhoto(toolpath::PhotoStrategy::Raster);
    p.tolerance.chordal = inf;
    CHECK_FALSE(nonFiniteParam(edit.params).empty());
    p = fullPhoto(toolpath::PhotoStrategy::Raster);
    CHECK(nonFiniteParam(edit.params).empty());
}
