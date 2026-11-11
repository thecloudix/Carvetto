// T-0239: fluting toolpaths as native toolpath records (ADR-0021 schema, ADR-0041): the
// FlutingParams variant alternative, its job.json params, exact round trips, non-finite refusal.

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
#include "carvetto/toolpath/fluting.hpp"

using namespace carvetto;
using namespace carvetto::document;

namespace {

toolpath::FlutingParams fullFluting() {
    toolpath::FlutingParams p;
    p.name = "Flute \"a\" \xC3\xA9";
    p.fluteDepth = 3.25;
    p.type = toolpath::FluteType::RampAtStartAndEnd;
    p.rampBasis = toolpath::FluteRampBasis::Percent;
    p.rampLength = 12.5;
    p.rampPercent = 17.5;
    p.rampType = toolpath::FluteRampType::Smooth;
    p.safeZ = 7.5;
    p.startPosition = {-3.5, 2.25};
    p.tolerance = {0.002, 1.7453292519943296e-05, 0.02};
    return p;
}

tooling::Tool endMill() {
    tooling::Tool t;
    t.id = "fl";
    t.name = "Flute mill";
    t.type = tooling::ToolType::BallNose;
    t.diameter = 3.0;
    t.defaultCuttingData = {1.0, 0.2, 0.0, 0.0, 18000.0, 1500.0, 500.0, 0.0, 4};
    return t;
}

Document flutingDocument(const toolpath::FlutingParams& params) {
    Document doc;
    geometry::Contour c;
    c.append(geometry::LineSpan{{0, 0}, {60, 10}});
    const ObjectId a = doc.insertObject(Object::vector(c, doc.defaultLayer(), doc.activeSheet()));
    toolpath::FlutingResult r = toolpath::flutingToolpath({c}, endMill(), std::nullopt, params);
    REQUIRE(r.ok());
    ToolpathRecord rec;
    rec.name = r.toolpath.name;
    rec.params = params;
    rec.selection.objects = {a};
    rec.tool = r.toolpath.tool;
    rec.toolSource = {"fl", "", ""};
    rec.moves = shareMoves(std::move(r.toolpath.moves));
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(rec)));
    return doc;
}

}  // namespace

TEST_CASE("T-0239: fluting records save and load exactly; save -> load -> save is byte-identical",
          "[document][toolpath][carv][fluting]") {
    const Document doc = flutingDocument(fullFluting());
    REQUIRE(doc.toolpaths().size() == 1);
    CHECK(doc.toolpaths()[0].operation() == ToolpathOperation::Fluting);
    CHECK(doc.toolpaths()[0].safeZ() == 7.5);
    auto saved = saveCarv(doc, CarvExtras{});
    REQUIRE(saved.ok());
    auto back = loadCarv(std::span<const std::uint8_t>(saved.value()));
    REQUIRE(back.ok());
    REQUIRE(back.value().document.toolpaths().size() == 1);
    CHECK(back.value().document.toolpaths()[0] == doc.toolpaths()[0]);
    CHECK(std::get<toolpath::FlutingParams>(back.value().document.toolpaths()[0].params) == fullFluting());
    auto again = saveCarv(back.value().document, back.value().extras);
    REQUIRE(again.ok());
    CHECK(again.value() == saved.value());
}

TEST_CASE("T-0239: every flute and ramp type survives a save and load", "[document][toolpath][carv][fluting]") {
    for (const auto ft : {toolpath::FluteType::RampOverLength, toolpath::FluteType::RampAtStart,
                          toolpath::FluteType::RampAtStartAndEnd}) {
        for (const auto rt : {toolpath::FluteRampType::Linear, toolpath::FluteRampType::Smooth}) {
            for (const auto rb : {toolpath::FluteRampBasis::Length, toolpath::FluteRampBasis::Percent}) {
                toolpath::FlutingParams p = fullFluting();
                p.type = ft;
                p.rampType = rt;
                p.rampBasis = rb;
                const Document doc = flutingDocument(p);
                auto saved = saveCarv(doc, CarvExtras{});
                REQUIRE(saved.ok());
                auto back = loadCarv(std::span<const std::uint8_t>(saved.value()));
                REQUIRE(back.ok());
                CHECK(std::get<toolpath::FlutingParams>(back.value().document.toolpaths()[0].params) == p);
            }
        }
    }
}

TEST_CASE("T-0239: non-finite fluting parameters are refused by the commands", "[document][toolpath][fluting]") {
    Document doc = flutingDocument(fullFluting());
    ToolpathRecord edit = doc.toolpaths()[0];
    const double inf = std::numeric_limits<double>::infinity();
    auto& p = std::get<toolpath::FlutingParams>(edit.params);
    struct Field {
        double toolpath::FlutingParams::*member;
        const char* key;
    };
    const Field fields[] = {{&toolpath::FlutingParams::fluteDepth, "flute_depth"},
                            {&toolpath::FlutingParams::rampLength, "ramp_length"},
                            {&toolpath::FlutingParams::rampPercent, "ramp_percent"},
                            {&toolpath::FlutingParams::safeZ, "safe_z"}};
    for (const Field& f : fields) {
        p = fullFluting();
        p.*(f.member) = inf;
        CHECK(nonFiniteParam(edit.params) == f.key);
        CHECK_FALSE(doc.execute(std::make_unique<SetToolpathCommand>(edit)));
    }
    p = fullFluting();
    p.startPosition.y = inf;
    CHECK(nonFiniteParam(edit.params) == "start_position[1]");
    p = fullFluting();
    p.tolerance.chordal = inf;
    CHECK_FALSE(nonFiniteParam(edit.params).empty());
    p = fullFluting();
    CHECK(nonFiniteParam(edit.params).empty());
}
