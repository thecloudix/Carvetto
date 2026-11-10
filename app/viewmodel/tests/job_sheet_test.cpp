// T-0244: job sheet / toolpath summary report: contents match the session summary and
// core/toolpath/time.hpp, deterministic output, hostile names escaped.

#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <string>

#include "carvetto/viewmodel/job_sheet.hpp"
#include "toolpath_test_support.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using namespace carvetto::viewmodel::testing;

namespace {

bool contains(const std::string& hay, const std::string& needle) { return hay.find(needle) != std::string::npos; }

struct Fixture {
    document::Document doc;
    tooling::ToolDatabase db{cliToolDatabase()};
    ToolpathSession session{&doc};
    document::ObjectId big, tiny;
    Fixture() {
        session.setToolDatabase(db);
        big = addVector(doc, square(0, 0, 30));
        tiny = addVector(doc, square(50, 0, 4));
        REQUIRE(session.setMachineSetup(MachineSetup{3000.0, 500.0}));
    }
    document::ToolpathId addProfile(const std::string& name) {
        ToolpathDefinition d = profileDefinition(db);
        d.selector = VectorSelector::fromObjects({big, tiny});
        const auto out = session.add(d);
        REQUIRE(out.ok);
        if (!name.empty()) {
            REQUIRE(session.rename(out.ids.front(), name).ok);
        }
        return out.ids.front();
    }
    document::ToolpathId addPocket() {
        ToolpathDefinition d = pocketDefinition(db);
        d.selector = VectorSelector::fromObjects({big});
        const auto out = session.add(d);
        REQUIRE(out.ok);
        return out.ids.front();
    }
};

}  // namespace

TEST_CASE("job sheet: lists every toolpath, tool and time of the core estimate", "[jobsheet]") {
    Fixture f;
    f.addProfile("Outline");
    f.addPocket();
    const JobSummary summary = f.session.summary();
    const JobSheet sheet = makeJobSheet(f.doc, summary, "Sign");
    REQUIRE(sheet.toolpaths.size() == 2);
    REQUIRE(sheet.tools.size() == 2);
    for (std::size_t i = 0; i < 2; ++i) {
        const document::ToolpathRecord& rec = f.doc.toolpaths()[i];
        const toolpath::MachineRates rates = toolpath::MachineRates::fromTool(rec.tool, 3000.0, 500.0);
        const toolpath::TimeEstimate expect = toolpath::estimateTime(
            toolpath::Toolpath{rec.name, rec.tool, *rec.moves}, rates);
        CHECK(sheet.toolpaths[i].name == rec.name);
        CHECK(sheet.toolpaths[i].time.total() == expect.total());
        CHECK(sheet.toolpaths[i].feedRateMmPerMin == rec.tool.feedRate);
        CHECK(sheet.toolpaths[i].plungeRateMmPerMin == rec.tool.plungeRate);
        CHECK(sheet.toolpaths[i].spindleRpm == rec.tool.spindleSpeed);
        CHECK(sheet.toolpaths[i].moves == rec.moveCount());
    }
    CHECK(sheet.tools[0].toolpaths == std::vector<std::string>{"Outline"});

    const std::string text = renderJobSheetText(sheet);
    CHECK(contains(text, "JOB SHEET: Sign"));
    CHECK(contains(text, "1. Outline [profile]"));
    CHECK(contains(text, "2. Pocket [pocket]"));
    CHECK(contains(text, formatDuration(summary.time.total())));
    CHECK(contains(text, "Single sided"));

    const std::string html = renderJobSheetHtml(sheet);
    CHECK(contains(html, "<h1>Job sheet: Sign</h1>"));
    CHECK(contains(html, "<svg"));
    CHECK(contains(html, "<td>Outline</td>"));
    CHECK(contains(html, formatDuration(sheet.toolpaths[0].time.total())));
}

TEST_CASE("job sheet: output is deterministic", "[jobsheet]") {
    Fixture f;
    f.addProfile("");
    const JobSheet a = makeJobSheet(f.doc, f.session.summary(), "J");
    const JobSheet b = makeJobSheet(f.doc, f.session.summary(), "J");
    CHECK(renderJobSheetHtml(a) == renderJobSheetHtml(b));
    CHECK(renderJobSheetText(a) == renderJobSheetText(b));
}

TEST_CASE("job sheet: hostile names are escaped", "[jobsheet]") {
    Fixture f;
    f.addProfile("<script>alert(1)</script>&\"'");
    const std::string title = "</title><img src=x onerror=1>";
    JobSheet sheet = makeJobSheet(f.doc, f.session.summary(), title);
    // The session refuses control characters in names, but a loaded file may carry them.
    sheet.toolpaths[0].name += "\nFAKE LINE";
    const std::string html = renderJobSheetHtml(sheet);
    CHECK_FALSE(contains(html, "<script"));
    CHECK_FALSE(contains(html, "<img"));
    CHECK(contains(html, "&lt;script&gt;alert(1)&lt;/script&gt;&amp;&quot;&#39; FAKE LINE"));
    CHECK(contains(html, "&lt;/title&gt;&lt;img src=x onerror=1&gt;"));
    const std::string text = renderJobSheetText(sheet);
    CHECK_FALSE(contains(text, "\nFAKE LINE"));
    CHECK(contains(text, "' FAKE LINE"));
    CHECK(escapeHtml("a\tb\x01") == "a b ");
}

TEST_CASE("job sheet: empty job, missing rapid rate, inches, unknown time", "[jobsheet]") {
    Fixture f;
    JobSheet empty = makeJobSheet(f.doc, f.session.summary());
    CHECK(empty.title == "Untitled job");
    CHECK(contains(renderJobSheetText(empty), "(none)"));
    CHECK(contains(renderJobSheetHtml(empty), "no toolpaths"));

    f.addProfile("");
    REQUIRE(f.session.setMachineSetup(MachineSetup{}));
    document::Job job = f.doc.job();
    job.displayUnits = document::Units::Inches;
    f.doc.setJob(job);
    const std::string text = renderJobSheetText(makeJobSheet(f.doc, f.session.summary(), "J"));
    CHECK(contains(text, "without rapids"));
    CHECK(contains(text, "rapid rate is not set"));
    CHECK(contains(text, " in/min"));
    CHECK(contains(text, " in x "));

    // A disabled toolpath is listed but not in the estimate.
    REQUIRE(f.session.setEnabled(f.doc.toolpaths().front().id, false));
    const JobSheet off = makeJobSheet(f.doc, f.session.summary(), "J");
    CHECK(off.tools.empty());
    CHECK(contains(renderJobSheetText(off), "(not in the estimate)"));
}
