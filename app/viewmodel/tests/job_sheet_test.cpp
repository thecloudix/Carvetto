// T-0244: job sheet / toolpath summary report: contents match the session summary and
// core/toolpath/time.hpp, deterministic output, hostile names escaped.

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <locale>
#include <string>
#include <utility>

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

namespace {

JobSheetToolpath fakeToolpath(const std::string& name, const std::string& tool) {
    JobSheetToolpath t;
    t.name = name;
    t.operation = "profile";
    t.toolName = tool;
    t.toolNumber = 1;
    t.toolDiameterMm = 3.0;
    t.feedRateMmPerMin = 1000.0;
    t.included = true;
    t.bounds = geometry::Box2{{0.0, 0.0}, {10.0, 10.0}};
    return t;
}

JobSheet bigSheet(std::size_t count, std::size_t nameBytes) {
    JobSheet s;
    s.title = "Big";
    JobSheetTool tool;
    tool.name = "Tool";
    tool.number = 1;
    for (std::size_t i = 0; i < count; ++i) {
        s.toolpaths.push_back(fakeToolpath(std::string(nameBytes, 'x'), "Tool"));
        tool.toolpaths.push_back(s.toolpaths.back().name);
    }
    s.tools.push_back(tool);
    return s;
}

struct CommaPunct : std::numpunct<char> {
    char do_decimal_point() const override { return ','; }
    char do_thousands_sep() const override { return '.'; }
    std::string do_grouping() const override { return "\3"; }
};

}  // namespace

TEST_CASE("job sheet: output size is bounded for large jobs and long names", "[jobsheet]") {
    for (const auto& dims : {std::pair<std::size_t, std::size_t>{2000, 100000}, {50000, 256}}) {
        const JobSheet s = bigSheet(dims.first, dims.second);
        const std::string html = renderJobSheetHtml(s);
        const std::string text = renderJobSheetText(s);
        CHECK(html.size() < 1000000);
        CHECK(text.size() < 500000);
        CHECK(contains(html, "more toolpaths"));
        CHECK(contains(text, "more toolpaths"));
        CHECK(contains(html, "... and " + std::to_string(dims.first - kJobSheetMaxToolpathsPerTool) + " more"));
        CHECK(contains(text, "... and " + std::to_string(dims.first - kJobSheetMaxToolpathsPerTool) + " more"));
        CHECK_FALSE(contains(text, std::string(kJobSheetMaxNameBytes + 1, 'x')));
        CHECK(contains(text, std::string(kJobSheetMaxNameBytes, 'x') + "..."));
    }
}

TEST_CASE("job sheet: clipping respects UTF-8 and limits tools and warnings", "[jobsheet]") {
    CHECK(clipJobSheetText("short") == "short");
    std::string name(kJobSheetMaxNameBytes - 1, 'a');
    name += "\xC3\xA9\xC3\xA9";  // two-byte characters straddling the limit
    const std::string clipped = clipJobSheetText(name);
    CHECK(clipped == std::string(kJobSheetMaxNameBytes - 1, 'a') + "...");

    JobSheet s;
    for (std::size_t i = 0; i < kJobSheetMaxToolRows + 5; ++i) {
        s.tools.push_back({"T", static_cast<int>(i), 1.0, {}});
    }
    for (std::size_t i = 0; i < kJobSheetMaxWarnings + 3; ++i) {
        s.readout.warnings.push_back("w");
    }
    CHECK(contains(renderJobSheetText(s), "... and 5 more tools"));
    CHECK(contains(renderJobSheetHtml(s), "... and 5 more tools"));
    CHECK(contains(renderJobSheetText(s), "... and 3 more warnings"));
    CHECK(contains(renderJobSheetHtml(s), "... and 3 more warnings"));
}

TEST_CASE("job sheet: non-finite numbers print n/a; extreme aspect ratios are clamped", "[jobsheet]") {
    JobSheet s = bigSheet(1, 5);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    s.widthMm = nan;
    s.thicknessMm = -inf;
    s.toolpaths[0].feedRateMmPerMin = nan;
    s.toolpaths[0].plungeRateMmPerMin = inf;
    s.toolpaths[0].spindleRpm = nan;
    s.toolpaths[0].toolDiameterMm = inf;
    const std::string text = renderJobSheetText(s);
    const std::string html = renderJobSheetHtml(s);
    CHECK(contains(text, "n/a mm x"));
    CHECK(contains(text, "Feed: n/a mm/min, plunge: n/a mm/min"));
    CHECK(contains(html, "n/a"));
    CHECK_FALSE(contains(text, "nan"));
    CHECK_FALSE(contains(text, "inf"));
    CHECK_FALSE(contains(html, "nan"));
    CHECK_FALSE(contains(html, "inf"));

    for (const double h : {1e-9, 1e9}) {
        JobSheet a = bigSheet(1, 3);
        a.material = geometry::Box2{{0.0, 0.0}, {100.0, h}};
        a.toolpaths.clear();
        const std::string svg = renderJobSheetHtml(a);
        const std::size_t p = svg.find("<svg");
        REQUIRE(p != std::string::npos);
        const std::size_t q = svg.find("height=\"", p) + 8;
        const double height = std::stod(svg.substr(q, svg.find('"', q) - q));
        CHECK(height >= 40.0);
        CHECK(height <= 4000.0);
    }
}

TEST_CASE("job sheet: control characters never reach the output", "[jobsheet]") {
    JobSheet s = bigSheet(1, 3);
    s.title = std::string("T\r\nitle\x1b[0m");
    s.toolpaths[0].name = "a\nb\rc\x01\x7f";
    s.toolpaths[0].operation = "op\nX";
    s.toolpaths[0].toolName = "tool\nY";
    s.tools[0].name = "tn\nZ";
    s.tools[0].toolpaths = {"p\nQ"};
    s.readout.breakdown = "br\nW";
    s.readout.headline = "hl\nH";
    s.readout.warnings = {"wa\nR"};
    const std::string html = renderJobSheetHtml(s);
    const std::string text = renderJobSheetText(s);
    for (const std::string* out : {&html, &text}) {
        for (const char c : *out) {
            const unsigned char u = static_cast<unsigned char>(c);
            CHECK((u >= 0x20 || c == '\n'));
            CHECK(u != 0x7f);
        }
    }
    for (const char* forged : {"\nitle", "\nX", "\nY", "\nZ", "\nQ", "\nW", "\nH", "\nR", "\nb"}) {
        CHECK_FALSE(contains(text, forged));
    }
}

TEST_CASE("job sheet: output does not depend on the global locale", "[jobsheet]") {
    Fixture f;
    f.addProfile("Outline");
    const JobSheet sheet = makeJobSheet(f.doc, f.session.summary(), "L");
    const std::string html = renderJobSheetHtml(sheet);
    const std::string text = renderJobSheetText(sheet);
    const std::locale old = std::locale::global(std::locale(std::locale::classic(), new CommaPunct));
    const std::string html2 = renderJobSheetHtml(sheet);
    const std::string text2 = renderJobSheetText(sheet);
    std::locale::global(old);
    CHECK(html == html2);
    CHECK(text == text2);
}

namespace {

JobSummary manyToolsSummary(std::size_t tools, std::size_t perTool, std::size_t nameBytes) {
    JobSummary sum;
    for (std::size_t i = 0; i < tools * perTool; ++i) {
        ToolpathSummaryEntry e;
        e.name = "tp" + std::to_string(i);
        e.included = true;
        e.toolNumber = static_cast<int>(i % tools);
        e.toolName = std::to_string(i % tools) + std::string(nameBytes, 'n');
        e.bounds = geometry::Box2{{0.0, 0.0}, {10.0, 10.0}};
        sum.entries.push_back(std::move(e));
    }
    sum.toolpaths = sum.entries.size();
    sum.included = sum.entries.size();
    return sum;
}

}  // namespace

TEST_CASE("job sheet: 20000 distinct tools build quickly and the stored sheet is capped", "[jobsheet]") {
    const document::Document doc;
    const auto t0 = std::chrono::steady_clock::now();
    const JobSheet s = makeJobSheet(doc, manyToolsSummary(20000, 1, 200), "big");
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    CHECK(secs < 1.0);
    CHECK(s.toolsTotal == 20000);
    CHECK(s.tools.size() == kJobSheetMaxToolRows);
    CHECK(s.toolpaths.size() == kJobSheetMaxToolpathRows);
    CHECK(s.toolpathsTotal == 20000);
    CHECK(s.thumbnailOmitted == 20000 - kJobSheetMaxToolpathRows);
    // Stored names are clipped.
    CHECK(s.tools[0].name.size() <= kJobSheetMaxNameBytes + 3);
    const std::string text = renderJobSheetText(s);
    CHECK(contains(text, "... and 19900 more tools"));
    CHECK(contains(text, "... and 19500 more toolpaths"));
    CHECK(contains(renderJobSheetHtml(s), "... and 19900 more tools"));
}

TEST_CASE("job sheet: tools are told apart by number, name and diameter; shared tools gather toolpaths", "[jobsheet]") {
    const document::Document doc;
    const JobSheet s = makeJobSheet(doc, manyToolsSummary(3, 50, 1), "t");
    REQUIRE(s.tools.size() == 3);
    CHECK(s.toolsTotal == 3);
    CHECK(s.tools[0].toolpaths.size() == kJobSheetMaxToolpathsPerTool);
    CHECK(s.tools[0].toolpathsTotal == 50);
    CHECK(contains(renderJobSheetText(s), "... and 30 more"));
}

TEST_CASE("job sheet: the thumbnail says when toolpaths are left out", "[jobsheet]") {
    const document::Document doc;
    const JobSheet few = makeJobSheet(doc, manyToolsSummary(1, 10, 1), "t");
    CHECK_FALSE(contains(renderJobSheetHtml(few), "are not drawn"));
    const JobSheet many = makeJobSheet(doc, manyToolsSummary(1, kJobSheetMaxToolpathRows + 7, 1), "t");
    CHECK(contains(renderJobSheetHtml(many), "7 more are not drawn"));
    // A hand-built sheet with more rows than the cap too.
    JobSheet hand = bigSheet(kJobSheetMaxToolpathRows + 3, 1);
    for (JobSheetToolpath& t : hand.toolpaths) {
        t.included = true;
        t.bounds = geometry::Box2{{0.0, 0.0}, {5.0, 5.0}};
    }
    CHECK(contains(renderJobSheetHtml(hand), "more are not drawn"));
}

TEST_CASE("job sheet: writing is atomic, safe and refuses hostile paths", "[jobsheet]") {
    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path() / "carvetto-jobsheet-test";
    fs::remove_all(dir);
    fs::create_directories(dir);
    Fixture f;
    f.addProfile("Outline");
    const JobSheet sheet = makeJobSheet(f.doc, f.session.summary(), "Sign");

    const auto read = [](const fs::path& p) {
        std::ifstream in(p, std::ios::binary);
        return std::string(std::istreambuf_iterator<char>(in), {});
    };
    const fs::path html = dir / "sheet.html";
    auto r = writeJobSheet(sheet, JobSheetFormat::Html, html, false);
    REQUIRE(r.ok);
    CHECK(read(html) == renderJobSheetHtml(sheet));
    const fs::path txt = dir / "sheet.txt";
    REQUIRE(writeJobSheet(sheet, JobSheetFormat::Text, txt, false).ok);
    CHECK(read(txt) == renderJobSheetText(sheet));

    // Existing file: refused unless overwrite; content untouched when refused.
    CHECK_FALSE(writeJobSheet(sheet, JobSheetFormat::Text, txt, false).ok);
    CHECK(writeJobSheet(sheet, JobSheetFormat::Text, txt, true).ok);

    // Hostile paths.
    CHECK_FALSE(writeJobSheet(sheet, JobSheetFormat::Html, {}, true).ok);
    CHECK_FALSE(writeJobSheet(sheet, JobSheetFormat::Html, dir / ".." / "x.html", true).ok);
    CHECK_FALSE(writeJobSheet(sheet, JobSheetFormat::Html, dir / "x.txt", true).ok);
    CHECK_FALSE(writeJobSheet(sheet, JobSheetFormat::Text, dir / "x.html", true).ok);
    CHECK_FALSE(writeJobSheet(sheet, JobSheetFormat::Html, dir / "a\nb.html", true).ok);
    CHECK_FALSE(writeJobSheet(sheet, JobSheetFormat::Html, dir / "CON.html", true).ok);
    CHECK_FALSE(writeJobSheet(sheet, JobSheetFormat::Html, dir / "x.html.", true).ok);
    CHECK_FALSE(writeJobSheet(sheet, JobSheetFormat::Html, dir / ".html", true).ok);
    CHECK_FALSE(writeJobSheet(sheet, JobSheetFormat::Html, dir / "sub" / "x.html", true).ok);
    fs::create_directories(dir / "folder.html");
    CHECK_FALSE(writeJobSheet(sheet, JobSheetFormat::Html, dir / "folder.html", true).ok);
    std::size_t files = 0;
    for (const auto& e : fs::directory_iterator(dir)) {
        (void)e;
        ++files;
    }
    CHECK(files == 3);  // sheet.html, sheet.txt, folder.html: no stray temp files

    CHECK(defaultJobSheetFileName("My/Sign: v2", JobSheetFormat::Html) == "My_Sign_ v2.html");
    CHECK(defaultJobSheetFileName("", JobSheetFormat::Text) == "job-sheet.txt");
    CHECK(checkJobSheetPath(dir / "ok.HTM", JobSheetFormat::Html).empty());
    fs::remove_all(dir);
}

TEST_CASE("job sheet: PDF names are checked but not written by the viewmodel", "[jobsheet]") {
    CHECK(checkJobSheetPath("/x/a.pdf", JobSheetFormat::Pdf).empty());
    CHECK_FALSE(checkJobSheetPath("/x/a.html", JobSheetFormat::Pdf).empty());
    CHECK(defaultJobSheetFileName("Sign", JobSheetFormat::Pdf) == "Sign.pdf");
    const document::Document doc;
    const JobSheet s = makeJobSheet(doc, JobSummary{}, "t");
    CHECK_FALSE(writeJobSheet(s, JobSheetFormat::Pdf, "/tmp/never.pdf", true).ok);
}
