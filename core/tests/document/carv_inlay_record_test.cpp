// T-0238: inlay toolpaths as native toolpath records (ADR-0021 schema, ADR-0040): the InlayParams
// variant alternative, its job.json params, exact round trips, non-finite refusal.

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
#include "carvetto/toolpath/inlay.hpp"

using namespace carvetto;
using namespace carvetto::document;

namespace {

toolpath::InlayParams fullInlay() {
    toolpath::InlayParams p;
    p.name = "Inlay \"male\" \xC3\xA9";
    p.piece = toolpath::InlayPiece::Male;
    p.maleStyle = toolpath::InlayMaleStyle::Stepped;
    p.femaleStyle = toolpath::InlayFemaleStyle::Hole;
    p.startDepth = 0.5;
    p.cutDepth = 4.25;
    p.stepDepth = 1.5;
    p.stepWidth = 2.75;
    p.allowance = 0.1;
    p.safeZ = 6.5;
    p.direction = toolpath::CutDirection::Conventional;
    p.startPosition = {-3.5, 2.25};
    p.tolerance = {0.002, 1.7453292519943296e-05, 0.02};
    return p;
}

tooling::Tool endMill() {
    tooling::Tool t;
    t.id = "em";
    t.name = "End mill";
    t.type = tooling::ToolType::EndMill;
    t.diameter = 3.0;
    t.defaultCuttingData = {1.0, 0.2, 0.0, 0.0, 18000.0, 1500.0, 500.0, 0.0, 4};
    return t;
}

Document inlayDocument(const toolpath::InlayParams& params) {
    Document doc;
    const geometry::Contour c = geometry::Contour::circle({10, 10}, 8.0);
    const ObjectId a = doc.insertObject(Object::vector(c, doc.defaultLayer(), doc.activeSheet()));
    toolpath::InlayResult r = toolpath::inlayToolpath({c}, endMill(), std::nullopt, params);
    REQUIRE(r.ok());
    ToolpathRecord rec;
    rec.name = r.toolpath.name;
    rec.params = params;
    rec.selection.objects = {a};
    rec.tool = r.toolpath.tool;
    rec.toolSource = {"em", "", ""};
    rec.moves = shareMoves(std::move(r.toolpath.moves));
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(rec)));
    return doc;
}

}  // namespace

TEST_CASE("T-0238: inlay records save and load exactly; save -> load -> save is byte-identical",
          "[document][toolpath][carv][inlay]") {
    for (const toolpath::InlayPiece piece : {toolpath::InlayPiece::Male, toolpath::InlayPiece::Female}) {
        toolpath::InlayParams params = fullInlay();
        params.piece = piece;
        const Document doc = inlayDocument(params);
        REQUIRE(doc.toolpaths().size() == 1);
        CHECK(doc.toolpaths()[0].operation() == ToolpathOperation::Inlay);
        CHECK(doc.toolpaths()[0].safeZ() == 6.5);
        auto saved = saveCarv(doc, CarvExtras{});
        REQUIRE(saved.ok());
        auto back = loadCarv(std::span<const std::uint8_t>(saved.value()));
        REQUIRE(back.ok());
        REQUIRE(back.value().document.toolpaths().size() == 1);
        CHECK(back.value().document.toolpaths()[0] == doc.toolpaths()[0]);
        CHECK(std::get<toolpath::InlayParams>(back.value().document.toolpaths()[0].params) == params);
        auto again = saveCarv(back.value().document, back.value().extras);
        REQUIRE(again.ok());
        CHECK(again.value() == saved.value());
    }
}

TEST_CASE("T-0238: non-finite inlay parameters are refused by the commands", "[document][toolpath][inlay]") {
    Document doc = inlayDocument(fullInlay());
    ToolpathRecord edit = doc.toolpaths()[0];
    const double inf = std::numeric_limits<double>::infinity();
    auto& p = std::get<toolpath::InlayParams>(edit.params);
    struct Field {
        double toolpath::InlayParams::*member;
        const char* key;
    };
    const Field fields[] = {{&toolpath::InlayParams::startDepth, "start_depth"},
                            {&toolpath::InlayParams::cutDepth, "cut_depth"},
                            {&toolpath::InlayParams::stepDepth, "step_depth"},
                            {&toolpath::InlayParams::stepWidth, "step_width"},
                            {&toolpath::InlayParams::allowance, "allowance"},
                            {&toolpath::InlayParams::safeZ, "safe_z"}};
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
