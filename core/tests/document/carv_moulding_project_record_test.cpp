// T-0311: Moulding and projected-onto-model toolpaths as native toolpath records (ADR-0091): the
// MouldingParams / Project3dParams variant alternatives, their job.json params, exact round trips,
// the "absent id = none" encoding and non-finite refusal.

#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <variant>
#include <vector>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/toolpath_record.hpp"

using namespace carvetto;
using namespace carvetto::document;

namespace {

toolpath::MouldingParams fullMoulding() {
    toolpath::MouldingParams p;
    p.name = "Moulding \xC3\xA9";
    p.mode = toolpath::MouldingMode::Rough;
    p.position = toolpath::MouldingPosition::Centre;
    p.reverseProfile = true;
    p.reverseRails = true;
    p.zOffset = -2.5;
    p.stockTop = 1.0;
    p.stockBottom = -14.0;
    p.safeZ = 7.0;
    p.allowance = 0.3;
    p.boundaryOffset = 1.25;
    p.autoBoundaryOffset = true;
    p.travel = toolpath::RasterTravel::OneWay;
    p.direction = toolpath::CutDirection::Conventional;
    p.reverseSteps = true;
    p.varyStepover = true;
    p.skipFlatRegions = true;
    p.machineFlatRegions = false;
    p.flatAngle = 0.0349065850398866;
    p.sharpCorners = true;
    p.rampPlunge = true;
    p.rampAngle = 0.2617993877991494;
    p.restTolerance = 0.02;
    p.tolerance = 0.005;
    p.profileObject = 42;
    return p;
}

toolpath::Project3dParams fullProject() {
    toolpath::Project3dParams p;
    p.name = "Projected";
    p.mode = toolpath::ProjectTipMode::Project;
    p.zOffset = -4.0;
    p.allowance = 0.2;
    p.outsideSurface = 0.5;
    p.stockBottom = -9.0;
    p.tolerance = 0.02;
    p.maxSegment = 0.75;
    p.tabs.enabled = true;
    p.tabs.length = 8.0;
    p.tabs.thickness = 1.5;
    p.tabs.spacing = toolpath::TabSpacing::ConstantDistance;
    p.tabs.count = 3;
    p.tabs.distance = 40.0;
    p.tabs.avoidCorners = true;
    p.tabs.threeD = true;
    p.sourceToolpath = 7;
    return p;
}

ToolpathRecord recordWith(ToolpathParams params) {
    ToolpathRecord rec;
    rec.name = "t";
    rec.params = std::move(params);
    rec.selection = VectorSelection::allVectors();
    rec.tool.name = "em";
    rec.tool.diameter = 6.0;
    rec.tool.toolNumber = 1;
    rec.tool.feedRate = 1500.0;
    rec.tool.plungeRate = 500.0;
    rec.tool.spindleSpeed = 18000.0;
    rec.toolSource = {"em", "", ""};
    std::vector<toolpath::Move> moves;
    moves.push_back(toolpath::Move::rapid({0, 0, 5}));
    moves.push_back(toolpath::Move::plunge({0, 0, -1.5}));
    moves.push_back(toolpath::Move::feed({10, 0, -1.5}));
    rec.moves = shareMoves(std::move(moves));
    return rec;
}

}  // namespace

TEST_CASE("T-0311: moulding and projected records save and load exactly; save -> load -> save is byte-identical",
          "[document][toolpath][carv][3d]") {
    Document doc;
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(recordWith(fullMoulding()))));
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(recordWith(fullProject()))));
    CHECK(doc.toolpaths()[0].operation() == ToolpathOperation::Moulding);
    CHECK(doc.toolpaths()[1].operation() == ToolpathOperation::Project3d);
    CHECK(doc.toolpaths()[0].safeZ() == 7.0);
    CHECK(doc.toolpaths()[1].safeZ() == 5.0);  // the highest point of the moves
    auto saved = saveCarv(doc, CarvExtras{});
    const std::string why = saved.ok() ? std::string() : saved.error().message;
    INFO(why);
    REQUIRE(saved.ok());
    auto back = loadCarv(std::span<const std::uint8_t>(saved.value()));
    REQUIRE(back.ok());
    REQUIRE(back.value().document.toolpaths().size() == 2);
    CHECK(back.value().document.toolpaths() == doc.toolpaths());
    CHECK(std::get<toolpath::MouldingParams>(back.value().document.toolpaths()[0].params) == fullMoulding());
    CHECK(std::get<toolpath::Project3dParams>(back.value().document.toolpaths()[1].params) == fullProject());
    auto again = saveCarv(back.value().document, back.value().extras);
    REQUIRE(again.ok());
    CHECK(again.value() == saved.value());
}

TEST_CASE("T-0311: an unset profile object and source toolpath survive a round trip", "[document][toolpath][carv][3d]") {
    Document doc;
    toolpath::MouldingParams m;
    toolpath::Project3dParams pr;
    REQUIRE(m.profileObject == 0);
    REQUIRE(pr.sourceToolpath == 0);
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(recordWith(m))));
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(recordWith(pr))));
    auto saved = saveCarv(doc, CarvExtras{});
    REQUIRE(saved.ok());
    auto back = loadCarv(std::span<const std::uint8_t>(saved.value()));
    REQUIRE(back.ok());
    CHECK(back.value().document.toolpaths() == doc.toolpaths());
}

TEST_CASE("T-0311: non-finite moulding and projection parameters are refused", "[document][toolpath][3d]") {
    const double inf = std::numeric_limits<double>::infinity();
    struct MField {
        double toolpath::MouldingParams::*member;
        const char* key;
    };
    const MField mFields[] = {{&toolpath::MouldingParams::zOffset, "z_offset"},
                              {&toolpath::MouldingParams::stockTop, "stock_top"},
                              {&toolpath::MouldingParams::stockBottom, "stock_bottom"},
                              {&toolpath::MouldingParams::safeZ, "safe_z"},
                              {&toolpath::MouldingParams::allowance, "allowance"},
                              {&toolpath::MouldingParams::boundaryOffset, "boundary_offset"},
                              {&toolpath::MouldingParams::flatAngle, "flat_angle_rad"},
                              {&toolpath::MouldingParams::rampAngle, "ramp_angle_rad"},
                              {&toolpath::MouldingParams::restTolerance, "rest_tolerance"},
                              {&toolpath::MouldingParams::tolerance, "tolerance"}};
    for (const MField& f : mFields) {
        toolpath::MouldingParams p = fullMoulding();
        CHECK(nonFiniteParam(ToolpathParams{p}).empty());
        p.*(f.member) = inf;
        CHECK(nonFiniteParam(ToolpathParams{p}) == f.key);
    }
    struct PField {
        double toolpath::Project3dParams::*member;
        const char* key;
    };
    const PField pFields[] = {{&toolpath::Project3dParams::zOffset, "z_offset"},
                              {&toolpath::Project3dParams::allowance, "allowance"},
                              {&toolpath::Project3dParams::outsideSurface, "outside_surface"},
                              {&toolpath::Project3dParams::stockBottom, "stock_bottom"},
                              {&toolpath::Project3dParams::tolerance, "tolerance"},
                              {&toolpath::Project3dParams::maxSegment, "max_segment"}};
    for (const PField& f : pFields) {
        toolpath::Project3dParams p = fullProject();
        CHECK(nonFiniteParam(ToolpathParams{p}).empty());
        p.*(f.member) = inf;
        CHECK(nonFiniteParam(ToolpathParams{p}) == f.key);
    }
    Document doc;
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(recordWith(fullMoulding()))));
    toolpath::MouldingParams bad = fullMoulding();
    bad.safeZ = inf;
    ToolpathRecord edit = doc.toolpaths()[0];
    edit.params = bad;
    CHECK_FALSE(doc.execute(std::make_unique<SetToolpathCommand>(edit)));
}
