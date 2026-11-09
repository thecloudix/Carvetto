// T-0236: chamfer toolpaths as native toolpath records (ADR-0021 schema, ADR-0035): the
// ChamferParams variant alternative, its job.json params, exact round trips, non-finite refusal.

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
#include "carvetto/toolpath/chamfer.hpp"

using namespace carvetto;
using namespace carvetto::document;

namespace {

toolpath::ChamferParams fullChamfer() {
    toolpath::ChamferParams p;
    p.name = "Edge \"A\" \xC3\xA9";
    p.type = toolpath::ChamferType::Outside;
    p.slope = toolpath::ChamferSlope::Up;
    p.angle = 0.5235987755982988;
    p.width = 1.2000000000000002;
    p.overcut = 0.1;
    p.direction = toolpath::CutDirection::Conventional;
    p.safeZ = 7.5;
    p.startPosition = {-3.5, 2.25};
    p.tolerance = {0.002, 1.7453292519943296e-05, 0.02};
    return p;
}

tooling::Tool vBit() {
    tooling::Tool t;
    t.id = "v-bit-60";
    t.name = "V-bit 60";
    t.type = tooling::ToolType::VBit;
    t.diameter = 12.7;
    t.includedAngle = 1.0471975511965976;
    t.defaultCuttingData = {1.0, 0.2, 0.0, 0.0, 18000.0, 1500.0, 500.0, 0.0, 4};
    return t;
}

Document chamferDocument(const toolpath::ChamferParams& params) {
    Document doc;
    const ObjectId a = doc.insertObject(Object::vector(geometry::Contour::rectangle({{0, 0}, {10, 10}}),
                                                       doc.defaultLayer(), doc.activeSheet()));
    const geometry::Contour sq = geometry::Contour::rectangle({{0, 0}, {10, 10}});
    toolpath::ChamferResult r = toolpath::chamferToolpath({sq}, vBit(), std::nullopt, params);
    REQUIRE(r.ok());
    ToolpathRecord rec;
    rec.name = r.toolpath.name;
    rec.params = params;
    rec.selection.objects = {a};
    rec.tool = r.toolpath.tool;
    rec.toolSource = {"v-bit-60", "", ""};
    rec.moves = shareMoves(std::move(r.toolpath.moves));
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(rec)));
    return doc;
}

}  // namespace

TEST_CASE("T-0236: chamfer records save and load exactly; save -> load -> save is byte-identical",
          "[document][toolpath][carv][chamfer]") {
    const Document doc = chamferDocument(fullChamfer());
    REQUIRE(doc.toolpaths().size() == 1);
    CHECK(doc.toolpaths()[0].operation() == ToolpathOperation::Chamfer);
    CHECK(doc.toolpaths()[0].safeZ() == 7.5);
    auto saved = saveCarv(doc, CarvExtras{});
    REQUIRE(saved.ok());
    auto back = loadCarv(std::span<const std::uint8_t>(saved.value()));
    REQUIRE(back.ok());
    REQUIRE(back.value().document.toolpaths().size() == 1);
    CHECK(back.value().document.toolpaths()[0] == doc.toolpaths()[0]);
    CHECK(std::get<toolpath::ChamferParams>(back.value().document.toolpaths()[0].params) == fullChamfer());
    auto again = saveCarv(back.value().document, back.value().extras);
    REQUIRE(again.ok());
    CHECK(again.value() == saved.value());
}

TEST_CASE("T-0236: non-finite chamfer parameters are refused by the commands", "[document][toolpath][chamfer]") {
    Document doc = chamferDocument(fullChamfer());
    ToolpathRecord edit = doc.toolpaths()[0];
    const double inf = std::numeric_limits<double>::infinity();
    auto& p = std::get<toolpath::ChamferParams>(edit.params);
    p.width = inf;
    CHECK(nonFiniteParam(edit.params) == "width");
    CHECK_FALSE(doc.execute(std::make_unique<SetToolpathCommand>(edit)));
    p = fullChamfer();
    p.angle = std::numeric_limits<double>::quiet_NaN();
    CHECK(nonFiniteParam(edit.params) == "angle_rad");
    p = fullChamfer();
    p.overcut = inf;
    CHECK(nonFiniteParam(edit.params) == "overcut");
    p = fullChamfer();
    p.safeZ = inf;
    CHECK(nonFiniteParam(edit.params) == "safe_z");
    p = fullChamfer();
    p.startPosition.y = inf;
    CHECK(nonFiniteParam(edit.params) == "start_position[1]");
    p = fullChamfer();
    CHECK(nonFiniteParam(edit.params).empty());
}
