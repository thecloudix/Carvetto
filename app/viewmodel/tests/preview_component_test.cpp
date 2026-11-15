// T-0311: Create Component From Toolpath Preview (ADR-0091): the simulated stock surface becomes a relief component
// of the active sheet in one undo step; cut-only cells, the Z offset, refusals and cancelling.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <cmath>
#include <limits>
#include <string>

#include "carvetto/viewmodel/preview_component.hpp"
#include "toolpath_test_support.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using namespace carvetto::viewmodel::testing;
using Catch::Approx;
using document::Document;

namespace {

/// A document with one 3 mm deep pocket on a 20 mm square.
void addPocket(Document& doc, ToolpathSession& session, const tooling::ToolDatabase& db) {
    addVector(doc, square(40.0, 40.0, 20.0));
    ToolpathDefinition def = pocketDefinition(db, 3.0);
    def.selector = VectorSelector::allVectors();
    const ToolpathOutcome out = session.add(def);
    INFO(out.message);
    REQUIRE(out.ok);
    doc.undoStack().clear();
}

}  // namespace

TEST_CASE("Component from toolpath preview: the carved stock becomes a component, one undo step",
          "[toolpath][preview][model]") {
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    addPocket(doc, session, db);
    REQUIRE(doc.reliefModel(doc.activeSheet()).empty());

    PreviewComponentRequest request;
    request.quality = simulate::Quality::Coarse;
    request.zOffset = 0.0;
    const PreviewComponentOutcome out = createComponentFromToolpathPreview(doc, session, request);
    INFO(out.message);
    REQUIRE(out.ok);
    CHECK_FALSE(out.component.isNull());
    const std::vector<relief::Level> levels = doc.reliefModel(doc.activeSheet());
    REQUIRE(levels.size() == 1);
    REQUIRE(levels[0].components.size() == 1);
    const relief::Component& c = levels[0].components[0];
    CHECK(c.name == "Toolpath Preview");
    // With a zero offset the uncut stock is at its top (0) and the pocket floor at -3 (cells inside the pocket).
    const float inside = c.field.sample(50.0, 50.0);
    const float outside = c.field.sample(10.0, 10.0);
    CHECK(static_cast<double>(inside) == Approx(-3.0).margin(0.01));
    CHECK(static_cast<double>(outside) == Approx(0.0).margin(1e-6));

    // One undo step removes the component (and the level that was added with it).
    REQUIRE(doc.undo());
    CHECK(doc.reliefModel(doc.activeSheet()).empty());
    REQUIRE(doc.redo());
    CHECK(doc.reliefModel(doc.activeSheet()).size() == 1);
    CHECK(doc.checkIntegrity().empty());
}

TEST_CASE("Component from toolpath preview: the default offset stands the stock on height 0; cut-only cells",
          "[toolpath][preview][model]") {
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    addPocket(doc, session, db);

    PreviewComponentRequest request;
    request.quality = simulate::Quality::Coarse;
    request.cells = simulate::PreviewCells::CutOnly;
    request.name = "Carved";
    const PreviewComponentOutcome out = createComponentFromToolpathPreview(doc, session, request);
    INFO(out.message);
    REQUIRE(out.ok);
    const std::vector<relief::Level> levels = doc.reliefModel(doc.activeSheet());
    const relief::Component& c = levels[0].components[0];
    CHECK(c.name == "Carved");
    const double thickness = doc.job().sizeMm.z;
    const float inside = c.field.sample(50.0, 50.0);
    CHECK(static_cast<double>(inside) == Approx(thickness - 3.0).margin(0.01));
    CHECK(std::isnan(c.field.sample(10.0, 10.0)));  // uncut cells are transparent
}

TEST_CASE("Component from toolpath preview: refusals and cancelling change nothing", "[toolpath][preview][model]") {
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    // No toolpaths.
    PreviewComponentOutcome none = createComponentFromToolpathPreview(doc, session);
    CHECK_FALSE(none.ok);
    CHECK_FALSE(none.message.empty());
    CHECK(doc.reliefModel(doc.activeSheet()).empty());

    addPocket(doc, session, db);
    PreviewComponentRequest bad;
    bad.zOffset = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(createComponentFromToolpathPreview(doc, session, bad).ok);

    std::atomic<bool> stop{true};
    PreviewComponentRequest request;
    request.quality = simulate::Quality::Coarse;
    const PreviewComponentOutcome cancelled = createComponentFromToolpathPreview(doc, session, request, &stop);
    CHECK_FALSE(cancelled.ok);
    CHECK(cancelled.cancelled);
    CHECK(doc.reliefModel(doc.activeSheet()).empty());
    CHECK_FALSE(doc.undoStack().canUndo());
}
