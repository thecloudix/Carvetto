// T-0257: the document variables model, expression bindings and recalculation of dependent toolpaths.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>
#include <map>
#include <span>
#include <string>
#include <vector>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/viewmodel/calculation.hpp"
#include "carvetto/viewmodel/toolpath_session.hpp"
#include "carvetto/viewmodel/variables_model.hpp"
#include "toolpath_test_support.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using namespace carvetto::viewmodel::testing;
using Catch::Approx;

namespace {

struct Fixture {
    document::Document doc;
    tooling::ToolDatabase db{cliToolDatabase()};
    ToolpathSession session{&doc};
    VariablesModel model{&doc, &session};
    document::ObjectId big;

    Fixture() {
        session.setToolDatabase(db);
        big = addVector(doc, square(0, 0, 30));
        doc.undoStack().clear();
        setCalculationProvider(this, [this]() {
            return std::optional<CalculationEnv>(environmentOf(doc));
        });
    }
    ~Fixture() { clearCalculationProvider(this); }

    /// A pocket whose cut depth is `={Depth}`; returns its id.
    document::ToolpathId addBoundPocket() {
        ToolpathForm form(document::ToolpathOperation::Pocket, document::Units::Millimetres);
        REQUIRE(form.setTool(db, source("end-mill-6mm")).ok);
        REQUIRE(form.setField("cutDepth", "={Depth}").ok);
        ToolpathDefinition def = form.definition();
        def.selector = VectorSelector::fromObjects({big});
        const ToolpathOutcome out = session.add(def);
        REQUIRE(out.ok);
        session.bindExpressions(out.ids, form.expressions());
        return out.ids.front();
    }
    double depthOf(document::ToolpathId id) const {
        return std::get<toolpath::PocketParams>(doc.findToolpath(id)->params).cutDepth;
    }
};

}  // namespace

TEST_CASE("variables model: create, change, rename and delete are one undo step each", "[variables]") {
    Fixture f;
    REQUIRE(f.model.set("Depth", "3").ok);
    CHECK(f.doc.undoStack().index() == 1);
    REQUIRE(f.model.set("Depth", "=1.5*2 + 1").ok);  // a calculation may define a value
    CHECK(f.doc.variables().get("Depth") == Approx(4.0));
    CHECK(f.doc.undoStack().index() == 2);  // an edit after a pause is its own step
    f.doc.undo();
    CHECK(f.doc.variables().get("Depth") == Approx(3.0));
    REQUIRE(f.model.rename("Depth", "D2").ok);
    CHECK_FALSE(f.doc.variables().contains("Depth"));
    CHECK(f.doc.variables().contains("D2"));
    f.doc.undo();
    CHECK(f.doc.variables().contains("Depth"));
    CHECK_FALSE(f.doc.variables().contains("D2"));
    REQUIRE(f.model.remove("Depth").ok);
    f.doc.undo();
    CHECK(f.doc.variables().contains("Depth"));
    // Refusals change nothing and say why.
    const std::size_t steps = f.doc.undoStack().index();
    for (const char* bad : {"", "1x", "=1/0", "=", "abc", "inf", "nan"}) {
        const VariableEdit r = f.model.set("X1", bad);
        INFO(bad);
        CHECK_FALSE(r.ok);
        CHECK_FALSE(r.message.empty());
    }
    CHECK_FALSE(f.model.set("1bad", "1").ok);
    CHECK_FALSE(f.model.set("has space", "1").ok);
    CHECK_FALSE(f.model.remove("Missing").ok);
    CHECK_FALSE(f.model.set("Depth", "1e300").ok);
    CHECK(f.doc.undoStack().index() == steps);
}

TEST_CASE("variables model: a variable used by a toolpath or an open form cannot be deleted", "[variables]") {
    Fixture f;
    REQUIRE(f.model.set("Depth", "3").ok);
    const document::ToolpathId id = f.addBoundPocket();
    CHECK(f.depthOf(id) == Approx(3.0));
    CHECK(f.doc.variables().referenceCount("Depth") == 1);
    const VariableEdit r = f.model.remove("Depth");
    CHECK_FALSE(r.ok);
    CHECK(r.message.find("Pocket") != std::string::npos);
    CHECK(f.doc.variables().contains("Depth"));
    CHECK_FALSE(f.model.rename("Depth", "Other").ok);
    CHECK(f.model.toolpathsUsing("Depth").size() == 1);

    // Deleting the toolpath (or undoing its creation) frees the variable.
    REQUIRE(f.session.deleteToolpaths({id}).ok);
    CHECK(f.model.remove("Depth").ok);
    f.doc.undo();  // variable back
    f.doc.undo();  // toolpath back (its binding was dropped with it; a new bind restores the hold)
    CHECK(f.doc.variables().contains("Depth"));

    // An open form holds its variables.
    REQUIRE(f.model.set("Margin", "5").ok);
    f.session.setOpenFormVariables({"Margin"});
    CHECK_FALSE(f.model.remove("Margin").ok);
    f.session.setOpenFormVariables({});
    CHECK(f.model.remove("Margin").ok);
}

TEST_CASE("variables model: changing a variable recalculates dependent toolpaths in one undo step", "[variables]") {
    Fixture f;
    REQUIRE(f.model.set("Depth", "3").ok);
    const document::ToolpathId id = f.addBoundPocket();
    const auto movesBefore = f.doc.findToolpath(id)->moves;
    f.doc.undoStack().breakCoalescing();
    const std::size_t steps = f.doc.undoStack().index();

    const VariableEdit r = f.model.set("Depth", "5");
    REQUIRE(r.ok);
    CHECK(r.recalculated == 1);
    CHECK(f.depthOf(id) == Approx(5.0));
    CHECK(f.doc.findToolpath(id)->moves != movesBefore);
    CHECK(f.doc.undoStack().index() == steps + 1);
    CHECK(f.session.item(id)->status == ToolpathStatus::Current);

    f.doc.undo();  // variable and toolpath return together
    CHECK(f.doc.variables().get("Depth") == Approx(3.0));
    CHECK(f.depthOf(id) == Approx(3.0));
    f.doc.redo();
    CHECK(f.depthOf(id) == Approx(5.0));

    // A value the toolpath form refuses keeps the record and warns; the variable still changes.
    const VariableEdit z = f.model.set("Depth", "0");
    CHECK(z.ok);
    CHECK(z.failed == 1);
    CHECK(z.message.find("warning") != std::string::npos);
    CHECK(f.depthOf(id) == Approx(5.0));
    // Unrelated variables recalculate nothing.
    REQUIRE(f.model.set("Other", "1").ok);
    CHECK(f.model.set("Other", "2").recalculated == 0);
}

TEST_CASE("variables model: the edit form of a bound toolpath shows its expression", "[variables]") {
    Fixture f;
    REQUIRE(f.model.set("Depth", "3").ok);
    const document::ToolpathId id = f.addBoundPocket();
    const std::optional<ToolpathForm> form = f.session.form(id);
    REQUIRE(form.has_value());
    CHECK(form->expression("cutDepth") == "={Depth}");
    f.session.bindExpressions({id}, {});
    CHECK(f.doc.variables().referenceCount("Depth") == 0);
    CHECK(f.model.remove("Depth").ok);
}

TEST_CASE("variables model: import overwrites same-named variables, export round-trips", "[variables]") {
    Fixture f;
    REQUIRE(f.model.set("A", "1").ok);
    f.doc.undoStack().breakCoalescing();
    const std::size_t steps = f.doc.undoStack().index();
    const VariableEdit r = f.model.importText("# comment\nA = 2\nB = 3.5\nbad line\n9x = 1\n");
    REQUIRE(r.ok);
    CHECK(r.imported == 2);
    CHECK(r.rejectedLines.size() == 2);
    CHECK(f.doc.variables().get("A") == Approx(2.0));
    CHECK(f.doc.variables().get("B") == Approx(3.5));
    CHECK(f.doc.undoStack().index() == steps + 1);
    f.doc.undo();
    CHECK(f.doc.variables().get("A") == Approx(1.0));
    CHECK_FALSE(f.doc.variables().contains("B"));
    f.doc.redo();

    const std::filesystem::path path = std::filesystem::temp_directory_path() / "carvetto_t0257_vars.txt";
    REQUIRE(f.model.exportFile(path).ok);
    Fixture g;
    REQUIRE(g.model.importFile(path).ok);
    CHECK(g.doc.variables() == f.doc.variables());
    std::filesystem::remove(path);
    CHECK_FALSE(g.model.importFile(path).ok);  // gone
    CHECK_FALSE(g.model.importText("").ok);
    CHECK_FALSE(g.model.importText(std::string(VariablesModel::kMaxImportBytes + 1, 'x')).ok);
    CHECK_FALSE(g.model.exportFile("/nonexistent-dir-t0257/vars.txt").ok);
}

TEST_CASE("T-0271: a toolpath's expressions live in its record and survive save and load", "[variables]") {
    Fixture f;
    REQUIRE(f.model.set("Depth", "3").ok);
    const document::ToolpathId id = f.addBoundPocket();
    REQUIRE(f.doc.findToolpath(id) != nullptr);
    CHECK(f.doc.findToolpath(id)->expressions == std::map<std::string, std::string>{{"cutDepth", "={Depth}"}});
    CHECK(f.session.expressionsOf(id) == f.doc.findToolpath(id)->expressions);

    const auto bytes = document::saveCarv(f.doc);
    REQUIRE(bytes.ok());
    auto loaded = document::loadCarv(std::span<const std::uint8_t>(bytes.value()));
    REQUIRE(loaded.ok());
    document::Document& copy = loaded.value().document;
    ToolpathSession session(&copy);
    session.setToolDatabase(f.db);
    VariablesModel model(&copy, &session);
    setCalculationProvider(&copy, [&copy]() { return std::optional<CalculationEnv>(environmentOf(copy)); });
    struct Release {
        const void* owner;
        ~Release() { clearCalculationProvider(owner); }
    } release{&copy};
    CHECK(model.rows().front().references == 1);  // the loaded toolpath holds Depth
    const VariableEdit refused = model.remove("Depth");
    CHECK_FALSE(refused.ok);
    CHECK(refused.message.find("Pocket") != std::string::npos);
    CHECK(copy.variables().contains("Depth"));
    // Changing the variable recalculates the loaded toolpath: the expression came back.
    const VariableEdit r = model.set("Depth", "4");
    REQUIRE(r.ok);
    CHECK(r.recalculated == 1);
    CHECK(std::get<toolpath::PocketParams>(copy.toolpaths().front().params).cutDepth == Approx(4.0));
    // Editing the loaded toolpath shows the expression.
    CHECK(session.form(copy.toolpaths().front().id)->expression("cutDepth") == "={Depth}");
    // Undoing the toolpath's removal brings the expressions back with it.
    REQUIRE(f.session.deleteToolpaths({id}).ok);
    f.session.refreshReferences();
    CHECK(f.doc.variables().referenceCount("Depth") == 0);
    f.doc.undo();
    f.session.refreshReferences();
    CHECK(f.doc.variables().referenceCount("Depth") == 1);
}

TEST_CASE("T-0271: deferred recalculation edits the variable, then recalculates off the edit", "[variables]") {
    Fixture f;
    REQUIRE(f.model.set("Depth", "3").ok);
    const document::ToolpathId id = f.addBoundPocket();
    f.doc.undoStack().breakCoalescing();
    const std::size_t steps = f.doc.undoStack().index();
    f.model.setDeferRecalculation(true);
    bool busy = false;
    f.model.setBusyCheck([&busy]() { return busy; });

    const VariableEdit r = f.model.set("Depth", "5");
    REQUIRE(r.ok);
    CHECK(r.recalculationPending);
    CHECK(r.recalculated == 0);
    CHECK(f.depthOf(id) == Approx(3.0));  // not yet
    CHECK(f.doc.undoStack().index() == steps + 1);

    // The background half: jobs are prepared against the new value, run anywhere, then applied.
    std::vector<ToolpathOutcome> refused;
    std::vector<ToolpathJob> jobs = f.session.prepareRecalculateBound(&refused);
    REQUIRE(jobs.size() == 1);
    CHECK(refused.empty());
    std::vector<ToolpathCalculation> calcs;
    for (const ToolpathJob& job : jobs) {
        calcs.push_back(job.run(nullptr));
    }
    const BatchOutcome applied = f.session.applyAll(calcs);
    REQUIRE(applied.ok);
    CHECK(f.depthOf(id) == Approx(5.0));
    CHECK(f.doc.findToolpath(id)->expressions.size() == 1);  // still bound
    CHECK(f.doc.undoStack().index() == steps + 2);
    CHECK(f.session.prepareRecalculateBound().empty());  // nothing changed since

    // A busy check refuses every edit; an unrelated variable needs no recalculation.
    busy = true;
    CHECK_FALSE(f.model.set("Depth", "6").ok);
    CHECK_FALSE(f.model.remove("Depth").ok);
    busy = false;
    REQUIRE(f.model.set("Other", "1").ok);
    CHECK(f.model.set("Other", "2").recalculationPending);  // any bound toolpath: the job list is what filters
    CHECK(f.session.prepareRecalculateBound().empty());
}
