// T-0332: the stock gadgets against the REAL toolpath session and tool database (the core tests use
// scripted services): the DXF batch gadget's profile toolpaths calculate, a drag-knife path is stored and
// survives a .carv round trip, the weave gadget is one undo step, and the job sheet obeys the editor.
#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <map>
#include <optional>
#include <fstream>
#include <string>
#include <vector>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/io/dxf.hpp"
#include "carvetto/viewmodel/gadget_manager.hpp"
#include "carvetto/viewmodel/job_sheet.hpp"
#include "toolpath_test_support.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using namespace carvetto::viewmodel::testing;
namespace fs = std::filesystem;

namespace {

bool contains(const std::string& s, const std::string& part) { return s.find(part) != std::string::npos; }

std::string stockSource(const std::string& folder) {
    std::ifstream in(fs::path(CARVETTO_STOCK_GADGETS_DIR) / folder / "gadget.lua", std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

std::string why(const GadgetRunReport& r) { return r.error ? r.error->summary + r.error->detail : std::string(); }

struct Stock {
    document::Document doc;
    ToolpathSession session{&doc};
    std::map<std::string, scripting::ScriptValue> answers;
    std::vector<std::string> messages;
    std::optional<std::string> folder;
    Stock() { session.setToolDatabase(cliToolDatabase()); }

    std::string firstTool() const { return session.toolDatabase().tools().begin()->first; }

    scripting::GadgetUi ui() {
        scripting::GadgetUi u;
        u.form = [this](const scripting::FormSpec& spec) -> std::optional<scripting::ScriptValue::Map> {
            scripting::ScriptValue::Map m;
            for (const auto& f : spec.fields) {
                const auto it = answers.find(f.name);
                if (it != answers.end()) {
                    m.set(f.name, it->second);
                } else if (f.kind == scripting::FormFieldKind::Tool) {
                    m.set(f.name, scripting::ScriptValue(firstTool()));
                } else if (f.kind == scripting::FormFieldKind::Layer) {
                    m.set(f.name, scripting::ScriptValue(static_cast<std::int64_t>(doc.activeLayer().value)));
                } else {
                    m.set(f.name, f.defaultValue);
                }
            }
            return m;
        };
        u.message = [this](const std::string& t) { messages.push_back(t); };
        u.chooseFolder = [this](const std::string&) { return folder; };
        return u;
    }

    GadgetRunReport run(const std::string& gadget) {
        scripting::ScriptLimits l;
        l.maxInstructions = 20'000'000;
        l.deadlineMillis = 0;
        return GadgetManager::runSource(stockSource(gadget), gadget, doc, session, ui(), l);
    }
    GadgetRunReport runSrc(const std::string& src) {
        scripting::ScriptLimits l;
        l.deadlineMillis = 0;
        return GadgetManager::runSource(src, "setup", doc, session, {}, l);
    }
};

}  // namespace

TEST_CASE("stock gadget: drag knife stores a path that survives save and load", "[gadget][viewmodel][t0332]") {
    Stock s;
    REQUIRE(s.runSrc("local id = carvetto.object.create_shape(nil, 'rect', {cx = 10, cy = 10, width = 20, height = 20}) "
                     "carvetto.selection.set({id})").ok);
    const GadgetRunReport r = s.run("drag-knife");
    INFO(why(r));
    REQUIRE(r.ok);
    REQUIRE(s.doc.toolpaths().size() == 1);
    CHECK(s.doc.toolpaths()[0].name == "Drag knife");
    CHECK(s.doc.undoStack().count() == 2);
    const auto bytes = document::saveCarv(s.doc);
    REQUIRE(bytes);
    const auto back = document::loadCarv(std::span<const std::uint8_t>(bytes.value()));
    REQUIRE(back);
    REQUIRE(back.value().document.toolpaths().size() == 1);
    CHECK(back.value().document.toolpaths()[0].moves != nullptr);
    CHECK(back.value().document.toolpaths()[0].moves->size() == s.doc.toolpaths()[0].moves->size());
}

TEST_CASE("stock gadget: DXF batch with real profile toolpaths", "[gadget][viewmodel][t0332]") {
    const fs::path dir = fs::temp_directory_path() / "carvetto-t0332vm-batch";
    std::error_code ec;
    fs::remove_all(dir, ec);
    fs::create_directories(dir);
    for (const char* name : {"plate.dxf", "bracket.dxf"}) {
        geometry::Contour c;
        c.closed = true;
        c.spans.push_back(geometry::LineSpan{{0, 0}, {30, 0}});
        c.spans.push_back(geometry::LineSpan{{30, 0}, {30, 20}});
        c.spans.push_back(geometry::LineSpan{{30, 20}, {0, 20}});
        c.spans.push_back(geometry::LineSpan{{0, 20}, {0, 0}});
        const std::vector<io::DxfLayer> layers{io::DxfLayer{}};
        const std::vector<io::DxfContour> cs{io::DxfContour{0, c}};
        const auto bytes = io::exportDxf(layers, cs);
        REQUIRE(bytes.ok());
        std::ofstream out(dir / name, std::ios::binary);
        out.write(reinterpret_cast<const char*>(bytes.value().data()), static_cast<std::streamsize>(bytes.value().size()));
    }
    Stock s;
    s.folder = dir.string();
    s.answers["profile"] = scripting::ScriptValue(true);
    s.answers["cut_depth"] = scripting::ScriptValue(3.0);
    const GadgetRunReport r = s.run("dxf-batch");
    INFO(why(r));
    REQUIRE(r.ok);
    CHECK(s.doc.toolpaths().size() == 2);
    CHECK(s.doc.objectCount() == 2);
    CHECK(s.doc.undoStack().count() == 1);
    for (const auto& tp : s.doc.toolpaths()) {
        REQUIRE(tp.moves != nullptr);
        CHECK_FALSE(tp.moves->empty());
    }
    s.doc.undoStack().undo(s.doc);
    CHECK(s.doc.objectCount() == 0);
    CHECK(s.doc.toolpaths().empty());
    fs::remove_all(dir, ec);
}

TEST_CASE("stock gadget: weave is one undo step, rollback leaves nothing", "[gadget][viewmodel][t0332]") {
    Stock s;
    const GadgetRunReport r = s.run("celtic-weave");
    REQUIRE(r.ok);
    CHECK(s.doc.undoStack().count() == 1);
    CHECK(s.doc.objectCount() == 25);
    // an invalid tool name fails the keyhole gadget after it created preview vectors: all rolled back
    REQUIRE(s.runSrc("carvetto.selection.set({carvetto.object.create_shape(nil, 'circle', {radius = 2})})").ok);
    s.answers["tool"] = scripting::ScriptValue(std::string("no-such-tool"));
    s.answers["preview"] = scripting::ScriptValue(true);
    const std::size_t objects = s.doc.objectCount();
    const std::size_t steps = s.doc.undoStack().count();
    const GadgetRunReport bad = s.run("keyhole");
    CHECK_FALSE(bad.ok);
    CHECK(s.doc.objectCount() == objects);
    CHECK(s.doc.undoStack().count() == steps);
}

TEST_CASE("job sheet: the editor's variables dress the sheet; defaults change nothing", "[gadget][viewmodel][t0332][jobsheet]") {
    Stock s;
    const JobSummary summary = s.session.summary();
    const std::string plainHtml = renderJobSheetHtml(makeJobSheet(s.doc, summary, "Gate"));
    const std::string plainText = renderJobSheetText(makeJobSheet(s.doc, summary, "Gate"));
    CHECK(jobSheetStyleFromVariables(s.doc.variables()) == JobSheetStyle{});
    CHECK_FALSE(contains(plainHtml, "background:"));
    CHECK(contains(plainHtml, "<h1>Job sheet: Gate</h1>"));

    s.answers["header"] = scripting::ScriptValue(std::string("#1A3C5E"));
    s.answers["footer"] = scripting::ScriptValue(std::string("#FFE080"));
    s.answers["layout"] = scripting::ScriptValue(false);
    s.answers["material"] = scripting::ScriptValue(false);
    REQUIRE(s.run("job-sheet-editor").ok);
    const JobSheetStyle st = jobSheetStyleFromVariables(s.doc.variables());
    CHECK(st.headerRgb == 0x1A3C5Eu);
    CHECK(st.footerRgb == 0xFFE080u);
    CHECK_FALSE(st.showLayout);
    CHECK_FALSE(st.showMaterial);
    CHECK(st.showSummary);
    const JobSheet sheet = makeJobSheet(s.doc, summary, "Gate");
    const std::string html = renderJobSheetHtml(sheet);
    CHECK(contains(html, "background:#1A3C5E;color:#FFFFFF"));   // dark header: white text
    CHECK(contains(html, "background:#FFE080;color:#000000"));   // light footer: black text
    CHECK_FALSE(contains(html, "Material setup"));
    CHECK_FALSE(contains(html, "Job layout"));
    CHECK(contains(html, "Toolpaths summary"));
    CHECK(contains(html, "Toolpaths list"));
    const std::string text = renderJobSheetText(sheet);
    CHECK_FALSE(contains(text, "Material setup"));
    CHECK(contains(text, "Toolpaths summary"));
    CHECK_FALSE(contains(text, "background"));

    // restore default: byte for byte what the sheet was before
    s.answers.clear();
    s.answers["restore"] = scripting::ScriptValue(true);
    REQUIRE(s.run("job-sheet-editor").ok);
    const JobSheet back = makeJobSheet(s.doc, summary, "Gate");
    CHECK(renderJobSheetHtml(back) == plainHtml);
    CHECK(renderJobSheetText(back) == plainText);
}

TEST_CASE("job sheet style: hostile variable values are bounded", "[gadget][viewmodel][t0332][jobsheet]") {
    document::DocumentVariables v;
    v.set("jobsheet_header_rgb", -5.0);
    v.set("jobsheet_footer_rgb", 1.0e300);
    CHECK(jobSheetStyleFromVariables(v).headerRgb == 0);
    CHECK(jobSheetStyleFromVariables(v).footerRgb == 0);
    v.set("jobsheet_header_rgb", 3.0e9);  // above 24 bits: masked, never markup
    const std::uint32_t masked = jobSheetStyleFromVariables(v).headerRgb;
    CHECK(masked <= 0xFFFFFFu);
    document::Document doc;
    doc.variables().set("jobsheet_header_rgb", 3.0e9);
    ToolpathSession session(&doc);
    const std::string html = renderJobSheetHtml(makeJobSheet(doc, session.summary(), "x"));
    CHECK(contains(html, "background:#"));
    CHECK_FALSE(contains(html, "background:#GG"));
}
