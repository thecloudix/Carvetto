// T-0240: prism toolpaths as native toolpath records (ADR-0021 schema, ADR-0042): the
// PrismParams variant alternative, its job.json params, exact round trips, non-finite refusal.

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
#include "carvetto/toolpath/prism.hpp"

using namespace carvetto;
using namespace carvetto::document;

namespace {

toolpath::PrismParams fullPrism() {
    toolpath::PrismParams p;
    p.name = "Prism \"1\" \xC3\xA9";
    p.depth = 3.1000000000000005;
    p.fullPrism = true;
    p.stepover = 0.35;
    p.direction = toolpath::CutDirection::Conventional;
    p.safeZ = 7.5;
    p.startPosition = {-3.5, 2.25};
    p.tolerance = {0.002, 1.7453292519943296e-05, 0.02};
    return p;
}

tooling::Tool vBit() {
    tooling::Tool t;
    t.id = "vb";
    t.name = "V-bit";
    t.type = tooling::ToolType::VBit;
    t.diameter = 12.7;
    t.includedAngle = 1.5707963267948966;
    t.defaultCuttingData = {1.0, 0.2, 0.0, 0.0, 18000.0, 1500.0, 500.0, 0.0, 4};
    return t;
}

Document prismDocument(const toolpath::PrismParams& params) {
    Document doc;
    const geometry::Contour c = geometry::Contour::circle({10, 10}, 8.0);
    const ObjectId a = doc.insertObject(Object::vector(c, doc.defaultLayer(), doc.activeSheet()));
    toolpath::PrismResult r = toolpath::prismToolpath({c}, vBit(), std::nullopt, params);
    REQUIRE(r.ok());
    ToolpathRecord rec;
    rec.name = r.toolpath.name;
    rec.params = params;
    rec.selection.objects = {a};
    rec.tool = r.toolpath.tool;
    rec.toolSource = {"vb", "", ""};
    rec.moves = shareMoves(std::move(r.toolpath.moves));
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(rec)));
    return doc;
}

}  // namespace

TEST_CASE("T-0240: prism records save and load exactly; save -> load -> save is byte-identical",
          "[document][toolpath][carv][prism]") {
    const Document doc = prismDocument(fullPrism());
    REQUIRE(doc.toolpaths().size() == 1);
    CHECK(doc.toolpaths()[0].operation() == ToolpathOperation::Prism);
    CHECK(doc.toolpaths()[0].safeZ() == 7.5);
    auto saved = saveCarv(doc, CarvExtras{});
    REQUIRE(saved.ok());
    auto back = loadCarv(std::span<const std::uint8_t>(saved.value()));
    REQUIRE(back.ok());
    REQUIRE(back.value().document.toolpaths().size() == 1);
    CHECK(back.value().document.toolpaths()[0] == doc.toolpaths()[0]);
    CHECK(std::get<toolpath::PrismParams>(back.value().document.toolpaths()[0].params) == fullPrism());
    auto again = saveCarv(back.value().document, back.value().extras);
    REQUIRE(again.ok());
    CHECK(again.value() == saved.value());
}

TEST_CASE("T-0240: non-finite prism parameters are refused by the commands", "[document][toolpath][prism]") {
    Document doc = prismDocument(fullPrism());
    ToolpathRecord edit = doc.toolpaths()[0];
    const double inf = std::numeric_limits<double>::infinity();
    auto& p = std::get<toolpath::PrismParams>(edit.params);
    struct Field {
        double toolpath::PrismParams::*member;
        const char* key;
    };
    const Field fields[] = {{&toolpath::PrismParams::depth, "depth"},
                            {&toolpath::PrismParams::stepover, "stepover"},
                            {&toolpath::PrismParams::safeZ, "safe_z"}};
    for (const Field& f : fields) {
        p = fullPrism();
        p.*(f.member) = inf;
        CHECK(nonFiniteParam(edit.params) == f.key);
        CHECK_FALSE(doc.execute(std::make_unique<SetToolpathCommand>(edit)));
    }
    p = fullPrism();
    p.startPosition.y = inf;
    CHECK(nonFiniteParam(edit.params) == "start_position[1]");
    p = fullPrism();
    CHECK(nonFiniteParam(edit.params).empty());
}
