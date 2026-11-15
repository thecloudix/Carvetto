// T-0303: 3D rough and 3D finish toolpaths as native toolpath records (ADR-0021 schema, ADR-0085): the
// Rough3dParams / Finish3dParams variant alternatives, their job.json params, exact round trips and
// non-finite refusal.

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
#include "carvetto/toolpath/finish3d.hpp"
#include "carvetto/toolpath/rough3d.hpp"

using namespace carvetto;
using namespace carvetto::document;

namespace {

toolpath::Rough3dParams fullRough() {
    toolpath::Rough3dParams p;
    p.name = "Rough \"3D\" \xC3\xA9";
    p.strategy = toolpath::Rough3dStrategy::Raster;
    p.tip = toolpath::Rough3dTipMode::Project;
    p.zOffset = -6.25;
    p.stockTop = 1.0;
    p.stockBottom = -17.5;
    p.allowance = 0.4;
    p.safeZ = 9.0;
    p.boundary = toolpath::Rough3dBoundary::Vectors;
    p.boundaryOffset = -1.5;
    p.axis = toolpath::RasterAxis::AlongY;
    p.travel = toolpath::RasterTravel::OneWay;
    p.direction = toolpath::CutDirection::Conventional;
    p.clearing = toolpath::Rough3dClearing::Offset;
    p.profile = toolpath::Rough3dProfile::Last;
    p.order = toolpath::Rough3dOrder::DepthFirst;
    p.avoidMachined = true;
    p.reverseSteps = true;
    p.rampPlunge = true;
    p.rampAngle = 0.3490658503988659;
    p.tolerance = 0.02;
    p.restTolerance = 0.03;
    return p;
}

toolpath::Finish3dParams fullFinish() {
    toolpath::Finish3dParams p;
    p.name = "Finish 3D";
    p.strategy = toolpath::Finish3dStrategy::Offset;
    p.zOffset = -3.0;
    p.stockTop = 0.5;
    p.stockBottom = -9.0;
    p.allowance = 0.1;
    p.safeZ = 4.0;
    p.boundary = toolpath::Rough3dBoundary::Model;
    p.boundaryOffset = 2.0;
    p.rasterAngle = 0.7853981633974483;
    p.travel = toolpath::RasterTravel::OneWay;
    p.direction = toolpath::CutDirection::Conventional;
    p.reverseSteps = true;
    p.tolerance = 0.005;
    p.restTolerance = 0.02;
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

TEST_CASE("T-0303: 3D records save and load exactly; save -> load -> save is byte-identical",
          "[document][toolpath][carv][3d]") {
    Document doc;
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(recordWith(fullRough()))));
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(recordWith(fullFinish()))));
    CHECK(doc.toolpaths()[0].operation() == ToolpathOperation::Rough3d);
    CHECK(doc.toolpaths()[1].operation() == ToolpathOperation::Finish3d);
    CHECK(doc.toolpaths()[0].safeZ() == 9.0);
    CHECK(doc.toolpaths()[1].safeZ() == 4.0);
    auto saved = saveCarv(doc, CarvExtras{});
    const std::string why = saved.ok() ? std::string() : saved.error().message;
    INFO(why);
    REQUIRE(saved.ok());
    auto back = loadCarv(std::span<const std::uint8_t>(saved.value()));
    REQUIRE(back.ok());
    REQUIRE(back.value().document.toolpaths().size() == 2);
    CHECK(back.value().document.toolpaths() == doc.toolpaths());
    CHECK(std::get<toolpath::Rough3dParams>(back.value().document.toolpaths()[0].params) == fullRough());
    CHECK(std::get<toolpath::Finish3dParams>(back.value().document.toolpaths()[1].params) == fullFinish());
    auto again = saveCarv(back.value().document, back.value().extras);
    REQUIRE(again.ok());
    CHECK(again.value() == saved.value());
}

TEST_CASE("T-0303: non-finite 3D parameters are refused", "[document][toolpath][3d]") {
    Document doc;
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(recordWith(fullRough()))));
    const double inf = std::numeric_limits<double>::infinity();
    struct Field {
        double toolpath::Rough3dParams::*member;
        const char* key;
    };
    const Field roughFields[] = {{&toolpath::Rough3dParams::zOffset, "z_offset"},
                                 {&toolpath::Rough3dParams::stockTop, "stock_top"},
                                 {&toolpath::Rough3dParams::stockBottom, "stock_bottom"},
                                 {&toolpath::Rough3dParams::allowance, "allowance"},
                                 {&toolpath::Rough3dParams::safeZ, "safe_z"},
                                 {&toolpath::Rough3dParams::boundaryOffset, "boundary_offset"},
                                 {&toolpath::Rough3dParams::rampAngle, "ramp_angle_rad"},
                                 {&toolpath::Rough3dParams::tolerance, "tolerance"},
                                 {&toolpath::Rough3dParams::restTolerance, "rest_tolerance"}};
    for (const Field& f : roughFields) {
        toolpath::Rough3dParams p = fullRough();
        p.*(f.member) = inf;
        ToolpathRecord edit = recordWith(p);
        CHECK(nonFiniteParam(edit.params) == f.key);
    }
    toolpath::Finish3dParams fin = fullFinish();
    CHECK(nonFiniteParam(ToolpathParams{fin}).empty());
    fin.rasterAngle = inf;
    CHECK(nonFiniteParam(ToolpathParams{fin}) == "raster_angle_rad");
    toolpath::Rough3dParams bad = fullRough();
    bad.safeZ = inf;
    ToolpathRecord edit = doc.toolpaths()[0];
    edit.params = bad;
    CHECK_FALSE(doc.execute(std::make_unique<SetToolpathCommand>(edit)));
}

TEST_CASE("T-0315: every T-0304 rough field survives save and load", "[document][toolpath][carv][3d]") {
    using toolpath::Rough3dClearing;
    using toolpath::Rough3dOrder;
    using toolpath::Rough3dProfile;
    const auto roundTrip = [](const toolpath::Rough3dParams& in) {
        Document doc;
        REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(recordWith(in))));
        auto saved = saveCarv(doc, CarvExtras{});
        REQUIRE(saved.ok());
        auto back = loadCarv(std::span<const std::uint8_t>(saved.value()));
        REQUIRE(back.ok());
        REQUIRE(back.value().document.toolpaths().size() == 1);
        return std::get<toolpath::Rough3dParams>(back.value().document.toolpaths()[0].params);
    };
    for (const auto c : {Rough3dClearing::Raster, Rough3dClearing::Offset}) {
        toolpath::Rough3dParams p;
        p.clearing = c;
        CHECK(roundTrip(p).clearing == c);
    }
    for (const auto v : {Rough3dProfile::None, Rough3dProfile::First, Rough3dProfile::Last}) {
        toolpath::Rough3dParams p;
        p.profile = v;
        CHECK(roundTrip(p).profile == v);
    }
    for (const auto v : {Rough3dOrder::LevelByLevel, Rough3dOrder::DepthFirst}) {
        toolpath::Rough3dParams p;
        p.order = v;
        CHECK(roundTrip(p).order == v);
    }
    for (const bool v : {false, true}) {
        toolpath::Rough3dParams p;
        p.avoidMachined = v;
        CHECK(roundTrip(p).avoidMachined == v);
    }
}
