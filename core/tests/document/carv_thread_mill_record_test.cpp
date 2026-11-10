// T-0237: thread mill toolpaths as native toolpath records (ADR-0021 schema, ADR-0036): the
// ThreadMillParams variant alternative, its job.json params, exact round trips, non-finite refusal.

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
#include "carvetto/toolpath/thread_mill.hpp"

using namespace carvetto;
using namespace carvetto::document;

namespace {

toolpath::ThreadMillParams fullThread() {
    toolpath::ThreadMillParams p;
    p.name = "M6 \"thread\" \xC3\xA9";
    p.cutter = toolpath::ThreadCutter::MultiPoint;
    p.threadedLength = 4.5;
    p.type = toolpath::ThreadType::External;
    p.hand = toolpath::ThreadHand::Left;
    p.cutDirection = toolpath::ThreadCutDirection::Up;
    p.pitchPreset = toolpath::ThreadPitchPreset::UnifiedCoarse;
    p.pitch = 1.2700000000000002;
    p.diameter = 6.35;
    p.fitTolerance = -0.05;
    p.createCircles = true;
    p.startDepth = 1.5;
    p.threadLength = 9.0;
    p.maxDepth = 12.0;
    p.safeZ = 7.5;
    p.startPosition = {-3.5, 2.25};
    p.tolerance = {0.002, 1.7453292519943296e-05, 0.02};
    return p;
}

tooling::Tool endMill() {
    tooling::Tool t;
    t.id = "tm";
    t.name = "Thread mill";
    t.type = tooling::ToolType::EndMill;
    t.diameter = 3.0;
    t.defaultCuttingData = {1.0, 0.2, 0.0, 0.0, 18000.0, 1500.0, 500.0, 0.0, 4};
    return t;
}

Document threadDocument(const toolpath::ThreadMillParams& params) {
    Document doc;
    const geometry::Contour c = geometry::Contour::circle({10, 10}, 8.0);
    const ObjectId a = doc.insertObject(Object::vector(c, doc.defaultLayer(), doc.activeSheet()));
    toolpath::ThreadMillResult r = toolpath::threadMillToolpath({c}, endMill(), std::nullopt, params);
    REQUIRE(r.ok());
    ToolpathRecord rec;
    rec.name = r.toolpath.name;
    rec.params = params;
    rec.selection.objects = {a};
    rec.tool = r.toolpath.tool;
    rec.toolSource = {"tm", "", ""};
    rec.moves = shareMoves(std::move(r.toolpath.moves));
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(rec)));
    return doc;
}

toolpath::ThreadMillParams runnable() {
    toolpath::ThreadMillParams p = fullThread();
    p.type = toolpath::ThreadType::Internal;
    p.diameter = 6.35;
    p.createCircles = true;
    return p;
}

}  // namespace

TEST_CASE("T-0237: thread mill records save and load exactly; save -> load -> save is byte-identical",
          "[document][toolpath][carv][thread_mill]") {
    const Document doc = threadDocument(runnable());
    REQUIRE(doc.toolpaths().size() == 1);
    CHECK(doc.toolpaths()[0].operation() == ToolpathOperation::ThreadMill);
    CHECK(doc.toolpaths()[0].safeZ() == 7.5);
    auto saved = saveCarv(doc, CarvExtras{});
    REQUIRE(saved.ok());
    auto back = loadCarv(std::span<const std::uint8_t>(saved.value()));
    REQUIRE(back.ok());
    REQUIRE(back.value().document.toolpaths().size() == 1);
    CHECK(back.value().document.toolpaths()[0] == doc.toolpaths()[0]);
    CHECK(std::get<toolpath::ThreadMillParams>(back.value().document.toolpaths()[0].params) == runnable());
    auto again = saveCarv(back.value().document, back.value().extras);
    REQUIRE(again.ok());
    CHECK(again.value() == saved.value());
}

TEST_CASE("T-0237: non-finite thread mill parameters are refused by the commands",
          "[document][toolpath][thread_mill]") {
    Document doc = threadDocument(runnable());
    ToolpathRecord edit = doc.toolpaths()[0];
    const double inf = std::numeric_limits<double>::infinity();
    auto& p = std::get<toolpath::ThreadMillParams>(edit.params);
    struct Field {
        double toolpath::ThreadMillParams::*member;
        const char* key;
    };
    const Field fields[] = {{&toolpath::ThreadMillParams::threadedLength, "threaded_length"},
                            {&toolpath::ThreadMillParams::pitch, "pitch"},
                            {&toolpath::ThreadMillParams::diameter, "diameter"},
                            {&toolpath::ThreadMillParams::fitTolerance, "fit_tolerance"},
                            {&toolpath::ThreadMillParams::startDepth, "start_depth"},
                            {&toolpath::ThreadMillParams::threadLength, "thread_length"},
                            {&toolpath::ThreadMillParams::maxDepth, "max_depth"},
                            {&toolpath::ThreadMillParams::safeZ, "safe_z"}};
    for (const Field& f : fields) {
        p = runnable();
        p.*(f.member) = inf;
        CHECK(nonFiniteParam(edit.params) == f.key);
        CHECK_FALSE(doc.execute(std::make_unique<SetToolpathCommand>(edit)));
    }
    p = runnable();
    p.startPosition.y = inf;
    CHECK(nonFiniteParam(edit.params) == "start_position[1]");
    p = runnable();
    CHECK(nonFiniteParam(edit.params).empty());
}
