// T-0265: V-carve inlay toolpaths as native toolpath records (ADR-0021 schema, ADR-0054): the
// VCarveInlayParams variant alternative, its job.json params, exact round trips, non-finite refusal.

#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <variant>
#include <vector>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/toolpath_record.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/v_carve_inlay.hpp"

using namespace carvetto;
using namespace carvetto::document;

namespace {

toolpath::VCarveInlayParams fullInlay() {
    toolpath::VCarveInlayParams p;
    p.name = "V inlay \"plug\" \xC3\xA9";
    p.piece = toolpath::VCarveInlayPiece::Plug;
    p.pocketDepth = 2.5;
    p.glueGap = 0.15;
    p.surfaceClearance = 0.3;
    p.plugSheet = 7;
    p.plugBoundary = toolpath::PlugBoundary::VectorOffset;
    p.boundaryOffset = 9.5;
    p.safeZ = 6.5;
    p.clearanceStrategy = toolpath::PocketStrategy::Raster;
    p.clearanceDirection = toolpath::CutDirection::Conventional;
    p.clearanceRasterAngle = 0.5;
    p.rampPlunge = true;
    p.rampDistance = 7.0;
    p.startPosition = {-3.5, 2.25};
    p.tolerance = {0.002, 1.7453292519943296e-05, 0.02};
    return p;
}

tooling::Tool vBit() {
    tooling::Tool t;
    t.id = "v";
    t.name = "V bit";
    t.type = tooling::ToolType::VBit;
    t.diameter = 6.0;
    t.includedAngle = 1.5707963267948966;
    t.defaultCuttingData = {1.0, 0.2, 0.0, 0.0, 18000.0, 1500.0, 500.0, 0.0, 4};
    t.defaultCuttingData.stepover = 0.5;
    t.defaultCuttingData.passDepth = 5.0;
    return t;
}

Document inlayDocument(const toolpath::VCarveInlayParams& params) {
    Document doc;
    const geometry::Contour c = geometry::Contour::circle({10, 10}, 8.0);
    const ObjectId a = doc.insertObject(Object::vector(c, doc.defaultLayer(), doc.activeSheet()));
    toolpath::VCarvingResult r = toolpath::vCarveInlayToolpath({c}, vBit(), std::nullopt, {}, params,
                                                               geometry::Box2{{0, 0}, {300, 300}});
    REQUIRE(r.ok());
    ToolpathRecord rec;
    rec.name = r.toolpath.name;
    rec.params = params;
    rec.selection.objects = {a};
    rec.tool = r.toolpath.tool;
    rec.toolSource = {"v", "", ""};
    rec.moves = shareMoves(std::move(r.toolpath.moves));
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(rec)));
    return doc;
}

}  // namespace

TEST_CASE("T-0265: V-carve inlay records save and load exactly; save -> load -> save is byte-identical",
          "[document][toolpath][carv][v_carve_inlay]") {
    for (const toolpath::VCarveInlayPiece piece : {toolpath::VCarveInlayPiece::Pocket, toolpath::VCarveInlayPiece::Plug}) {
        for (const std::uint64_t sheet : {std::uint64_t{0}, std::uint64_t{7}}) {
            toolpath::VCarveInlayParams params = fullInlay();
            params.piece = piece;
            params.plugSheet = sheet;
            params.plugBoundary = piece == toolpath::VCarveInlayPiece::Plug ? toolpath::PlugBoundary::BoundaryOffset
                                                                          : toolpath::PlugBoundary::SheetLimits;
            const Document doc = inlayDocument(params);
            REQUIRE(doc.toolpaths().size() == 1);
            CHECK(doc.toolpaths()[0].operation() == ToolpathOperation::VCarveInlay);
            CHECK(doc.toolpaths()[0].safeZ() == 6.5);
            auto saved = saveCarv(doc, CarvExtras{});
            REQUIRE(saved.ok());
            auto back = loadCarv(std::span<const std::uint8_t>(saved.value()));
            REQUIRE(back.ok());
            REQUIRE(back.value().document.toolpaths().size() == 1);
            CHECK(back.value().document.toolpaths()[0] == doc.toolpaths()[0]);
            CHECK(std::get<toolpath::VCarveInlayParams>(back.value().document.toolpaths()[0].params) == params);
            auto again = saveCarv(back.value().document, back.value().extras);
            REQUIRE(again.ok());
            CHECK(again.value() == saved.value());
        }
    }
}

TEST_CASE("T-0265: non-finite V-carve inlay parameters are refused by the commands",
          "[document][toolpath][v_carve_inlay]") {
    Document doc = inlayDocument(fullInlay());
    ToolpathRecord edit = doc.toolpaths()[0];
    const double inf = std::numeric_limits<double>::infinity();
    auto& p = std::get<toolpath::VCarveInlayParams>(edit.params);
    struct Field {
        double toolpath::VCarveInlayParams::*member;
        const char* key;
    };
    const Field fields[] = {{&toolpath::VCarveInlayParams::pocketDepth, "pocket_depth"},
                            {&toolpath::VCarveInlayParams::glueGap, "glue_gap"},
                            {&toolpath::VCarveInlayParams::surfaceClearance, "surface_clearance"},
                            {&toolpath::VCarveInlayParams::boundaryOffset, "boundary_offset"},
                            {&toolpath::VCarveInlayParams::safeZ, "safe_z"},
                            {&toolpath::VCarveInlayParams::clearanceRasterAngle, "clearance_raster_angle_rad"},
                            {&toolpath::VCarveInlayParams::rampDistance, "ramp_distance"}};
    for (const Field& f : fields) {
        p = fullInlay();
        p.*(f.member) = inf;
        CHECK(nonFiniteParam(edit.params) == f.key);
        CHECK_FALSE(doc.execute(std::make_unique<SetToolpathCommand>(edit)));
    }
    p = fullInlay();
    p.startPosition.y = inf;
    CHECK(nonFiniteParam(edit.params) == "start_position[1]");
    p = fullInlay();
    CHECK(nonFiniteParam(edit.params).empty());
}
