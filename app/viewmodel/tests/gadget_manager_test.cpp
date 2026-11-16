// T-0331: the gadget manager: menu model, shortcuts, install, and real runs against a document with
// the toolpath session (one undo step, rollback, toolpath parameters, failure wording).
#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <filesystem>
#include <fstream>
#include <string>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/viewmodel/gadget_manager.hpp"
#include "toolpath_test_support.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using namespace carvetto::viewmodel::testing;
namespace fs = std::filesystem;

namespace {

bool contains(const std::string& s, const char* part) {
    return s.find(part) != std::string::npos;
}

struct Scratch {
    fs::path root;
    explicit Scratch(const std::string& name) : root(fs::temp_directory_path() / ("carvetto-t0331vm-" + name)) {
        std::error_code ec;
        fs::remove_all(root, ec);
        fs::create_directories(root);
    }
    ~Scratch() {
        std::error_code ec;
        fs::remove_all(root, ec);
    }
    Scratch(const Scratch&) = delete;
    Scratch& operator=(const Scratch&) = delete;
};

void write(const fs::path& p, const std::string& text) {
    fs::create_directories(p.parent_path());
    std::ofstream out(p, std::ios::binary);
    out << text;
}

void makeGadget(const fs::path& root, const std::string& folder, const std::string& name, const std::string& menu = "",
                const std::string& shortcut = "", int api = 1, const std::string& script = "return 1") {
    std::string m = "{\"name\": \"" + name + "\", \"api_version\": " + std::to_string(api);
    if (!menu.empty()) m += ", \"menu\": \"" + menu + "\"";
    if (!shortcut.empty()) m += ", \"shortcut\": \"" + shortcut + "\"";
    m += "}";
    write(root / folder / "manifest.json", m);
    write(root / folder / "gadget.lua", script);
}

struct Fixture {
    document::Document doc;
    ToolpathSession session{&doc};
    Fixture() { session.setToolDatabase(cliToolDatabase()); }
    scripting::ScriptLimits limits() const {
        scripting::ScriptLimits l;
        l.maxInstructions = 5'000'000;
        l.deadlineMillis = 0;
        return l;
    }
    GadgetRunReport run(const std::string& src) {
        return GadgetManager::runSource(src, "Test gadget", doc, session, {}, limits());
    }
};

}  // namespace

TEST_CASE("gadget manager: menu model is grouped and sorted", "[gadget][viewmodel][t0331]") {
    Scratch s("menu");
    makeGadget(s.root, "z1", "Zeta");
    makeGadget(s.root, "a1", "alpha", "Weave/Celtic");
    makeGadget(s.root, "a2", "Beta", "Weave/Another", "Ctrl+F3");
    makeGadget(s.root, "a3", "Deep", "Weave/Sub/Deep");
    makeGadget(s.root, "f1", "Future", "", "Ctrl+F4", 9);
    write(s.root / "junk" / "manifest.json", "nope");

    GadgetManager m(s.root);
    REQUIRE(m.gadgets().size() == 5);
    REQUIRE(m.skipped().size() == 1);
    const auto menu = m.menu();
    // submenus first, then gadgets by label ignoring case
    REQUIRE(menu.size() == 3);
    CHECK(menu[0].label == "Weave");
    CHECK(menu[0].gadget == -1);
    CHECK(menu[1].label == "Future");
    CHECK_FALSE(menu[1].enabled);
    CHECK(contains(menu[1].tooltip, "version 9"));
    CHECK(menu[2].label == "Zeta");
    CHECK(menu[2].enabled);
    const auto& weave = menu[0].children;
    REQUIRE(weave.size() == 3);
    CHECK(weave[0].label == "Sub");  // submenu first
    CHECK(weave[0].children.size() == 1);
    CHECK(weave[0].children[0].label == "Deep");
    CHECK(weave[1].label == "Another");
    CHECK(weave[1].shortcut == "Ctrl+F3");
    CHECK(weave[2].label == "Celtic");
    CHECK(m.gadgets()[static_cast<std::size_t>(weave[2].gadget)].manifest.name == "alpha");

    CHECK(m.byShortcut("Ctrl+F3").has_value());
    CHECK_FALSE(m.byShortcut("Ctrl+F4").has_value());  // an unrunnable gadget owns none
    CHECK_FALSE(m.byShortcut("").has_value());

    const auto rows = m.shortcuts();
    REQUIRE(rows.size() == 5);
}

TEST_CASE("gadget manager: a missing folder gives an empty menu", "[gadget][viewmodel][t0331]") {
    GadgetManager m(fs::temp_directory_path() / "carvetto-t0331vm-nothing-here");
    CHECK(m.menu().empty());
    CHECK(m.gadgets().empty());
}

TEST_CASE("gadget manager: assigning shortcuts", "[gadget][viewmodel][t0331]") {
    Scratch s("shortcuts");
    makeGadget(s.root, "a", "A", "", "Ctrl+F1");
    makeGadget(s.root, "b", "B");
    GadgetManager m(s.root);
    REQUIRE(m.gadgets().size() == 2);
    CHECK(m.assignShortcut(1, "ctrl+f2").empty());
    CHECK(m.gadgets()[1].shortcut == "Ctrl+F2");
    CHECK(m.overrides().at("b") == "Ctrl+F2");
    CHECK(contains(m.assignShortcut(1, "Ctrl+F1"), "already used by \"A\""));
    CHECK(m.gadgets()[1].shortcut == "Ctrl+F2");  // unchanged by a refusal
    CHECK(contains(m.assignShortcut(1, "F5"), "Ctrl+F1 to Ctrl+F12"));
    CHECK(m.assignShortcut(0, "").empty());  // remove A's own
    CHECK(m.gadgets()[0].shortcut.empty());
    CHECK(m.overrides().at("a").empty());
    CHECK(m.assignShortcut(1, "Ctrl+F1").empty());  // now free
    CHECK(m.assignShortcut(0, "Ctrl+F1").size() > 0);
    CHECK(m.assignShortcut(1, "").empty());
    CHECK(m.assignShortcut(0, "Ctrl+F1").empty());  // A's manifest value again: the override goes away
    CHECK(m.overrides().count("a") == 0);
    CHECK(contains(m.assignShortcut(9, "Ctrl+F1"), "no such gadget"));

    // overrides survive a restart: the app stores them and restores them before the scan
    GadgetManager again(s.root);
    again.setOverrides(m.overrides());
    again.rescan();
    CHECK(again.gadgets()[0].shortcut == "Ctrl+F1");
    CHECK(again.gadgets()[1].shortcut.empty());
}

TEST_CASE("gadget manager: install copies, rescans and shows in the menu", "[gadget][viewmodel][t0331]") {
    Scratch src("install-src");
    Scratch dst("install-dst");
    makeGadget(src.root, "pkg", "Fresh One", "Tools/Fresh", "Ctrl+Shift+F2");
    GadgetManager m(dst.root / "gadgets");
    CHECK(m.menu().empty());
    const auto seen = m.inspect(src.root / "pkg");
    REQUIRE(seen.ok);
    CHECK(seen.gadget.manifest.name == "Fresh One");
    const auto r = m.install(src.root / "pkg");
    REQUIRE(r.ok);
    REQUIRE(m.gadgets().size() == 1);
    CHECK(m.gadgets()[0].shortcut == "Ctrl+Shift+F2");
    CHECK(m.menu().size() == 1);
    CHECK_FALSE(m.install(src.root / "pkg").ok);  // already installed
    CHECK(m.install(src.root / "pkg", true).ok);
}

TEST_CASE("gadget run: vectors and toolpaths in ONE undo step", "[gadget][viewmodel][t0331]") {
    Fixture f;
    const GadgetRunReport r = f.run(R"(
        local c = carvetto
        local layer = c.layer.create("Gadget layer")
        local box = c.object.create_shape(layer, "rect", {width = 40, height = 30})
        local hole = c.object.create_shape(layer, "circle", {radius = 4})
        local pocket = c.toolpath.create("pocket", {box}, {tool = c.tool.find("end-mill-6mm"), cutDepth = 3,
                                                         strategy = "offset", name = "Gadget pocket"})
        local profile = c.toolpath.create("profile", {box}, {tool = "end-mill-3mm", cutDepth = 4, side = "outside",
                                                            direction = "conventional"})
        local drill = c.toolpath.create("drill", {hole}, {tool = "drill-3mm", cutDepth = 5})
        return pocket[1], profile[1], drill[1]
    )");
    INFO((r.error ? r.error->summary + r.error->detail : std::string()));
    REQUIRE(r.ok);
    CHECK(r.run.committed);
    CHECK(contains(r.summary, "3 objects") == false);  // layer is not an object: 2 objects
    CHECK(contains(r.summary, "2 objects, 3 toolpaths"));
    CHECK(f.doc.undoStack().count() == 1);
    CHECK(f.doc.undoStack().undoText() == "Test gadget");
    REQUIRE(f.doc.toolpaths().size() == 3);
    CHECK(f.doc.toolpaths()[0].name == "Gadget pocket");
    CHECK(f.doc.toolpaths()[0].operation() == document::ToolpathOperation::Pocket);
    CHECK(f.doc.toolpaths()[0].hasMoves());
    CHECK(f.doc.toolpaths()[0].moveCount() > 10);
    const auto* profile = std::get_if<toolpath::ProfileParams>(&f.doc.toolpaths()[1].params);
    REQUIRE(profile != nullptr);
    CHECK(profile->side == toolpath::ProfileSide::Outside);
    CHECK(profile->direction == toolpath::CutDirection::Conventional);
    CHECK(f.doc.toolpaths()[2].operation() == document::ToolpathOperation::Drill);
    CHECK(f.doc.checkIntegrity().empty());

    REQUIRE(f.doc.undo());
    CHECK(f.doc.toolpaths().empty());
    CHECK(f.doc.objectCount() == 0);
    CHECK(f.doc.layers().size() == 1);
    REQUIRE(f.doc.redo());
    CHECK(f.doc.toolpaths().size() == 3);
    CHECK(f.doc.objectCount() == 2);
    CHECK(f.doc.checkIntegrity().empty());
    // the session sees the redone toolpaths as current records
    CHECK(f.session.list().size() == 3);
}

TEST_CASE("gadget run: a toolpath that cannot be made rolls everything back", "[gadget][viewmodel][t0331]") {
    Fixture f;
    const auto expectFail = [&](const std::string& src, const char* text) {
        INFO(src);
        const GadgetRunReport r = f.run(src);
        REQUIRE_FALSE(r.ok);
        REQUIRE(r.error.has_value());
        INFO(r.error->summary);
        CHECK(contains(r.error->summary, text));
        CHECK(contains(r.error->summary, "Your drawing was not changed"));
        CHECK(f.doc.objectCount() == 0);
        CHECK(f.doc.toolpaths().empty());
        CHECK(f.doc.layers().size() == 1);
        CHECK(f.doc.undoStack().count() == 0);
    };
    const std::string prefix = "local b = carvetto.object.create_shape(nil, 'rect', {width = 40, height = 30}) ";
    expectFail(prefix + "carvetto.toolpath.create('pocket', {b}, {cutDepth = 3})", "needs a tool");
    expectFail(prefix + "carvetto.toolpath.create('pocket', {b}, {tool = 'no-such-tool', cutDepth = 3})", "tool:");
    expectFail(prefix + "carvetto.toolpath.create('pocket', {b}, {tool = 'end-mill-6mm'})", "");
    expectFail(prefix + "carvetto.toolpath.create('pocket', {b}, {tool = 'end-mill-6mm', cutDepth = 3, cutdepth = 2})",
               "unknown parameter 'cutdepth'");
    expectFail(prefix + "carvetto.toolpath.create('pocket', {b}, {tool = 'end-mill-6mm', cutDepth = 'deep'})",
               "must be a finite number");
    expectFail(prefix + "carvetto.toolpath.create('pocket', {b}, {tool = 'end-mill-6mm', cutDepth = -3})", "cutDepth");
    expectFail(prefix + "carvetto.toolpath.create('pocket', {b}, {tool = 'end-mill-6mm', cutDepth = 3, strategy = 'spiral'})",
               "must be one of: offset, raster");
    // a toolpath whose depth the tool cannot do is not made, and the vectors made before it go too
    expectFail(prefix + "carvetto.toolpath.create('pocket', {b}, {tool = 'end-mill-6mm', cutDepth = 3, name = ''})", "name");
}

TEST_CASE("gadget run: custom toolpaths use the tool database snapshot", "[gadget][viewmodel][t0331]") {
    Fixture f;
    const GadgetRunReport r = f.run(R"(
        local id = carvetto.toolpath.create_custom("Knife", "end-mill-3mm", {
            {kind = "rapid", x = 0, y = 0, z = 5},
            {kind = "plunge", z = -1},
            {kind = "feed", x = 20, y = 0},
        })
        return id
    )");
    INFO((r.error ? r.error->summary : std::string()));
    REQUIRE(r.ok);
    REQUIRE(f.doc.toolpaths().size() == 1);
    const document::ToolpathRecord& rec = f.doc.toolpaths()[0];
    CHECK(rec.name == "Knife");
    CHECK(rec.tool.diameter == 3.0);
    CHECK(rec.tool.feedRate > 0.0);
    CHECK(rec.tool.spindleSpeed > 0.0);
    CHECK(rec.toolSource.toolId == "end-mill-3mm");
    CHECK(rec.moveCount() == 3);
    CHECK(f.doc.undoStack().count() == 1);
    // the record is storable: it survives a save and load
    const auto bytes = document::saveCarv(f.doc);
    REQUIRE(bytes);
    const auto back = document::loadCarv(std::span<const std::uint8_t>(bytes.value()));
    REQUIRE(back);
    CHECK(back.value().document.toolpaths().size() == 1);
}

TEST_CASE("gadget run: forms receive the host's answers", "[gadget][viewmodel][t0331]") {
    Fixture f;
    scripting::GadgetUi ui;
    ui.form = [](const scripting::FormSpec& spec) -> std::optional<scripting::ScriptValue::Map> {
        scripting::ScriptValue::Map m;
        for (const auto& fld : spec.fields) {
            m.set(fld.name, fld.defaultValue);
        }
        m.set("size", scripting::ScriptValue(25.0));
        return m;
    };
    const GadgetRunReport r = GadgetManager::runSource(
        "local v = carvetto.ui.form({title = 'Size', fields = {{name = 'size', default = 10}}}) "
        "carvetto.object.create_shape(nil, 'circle', {radius = v.size})",
        "Circle gadget", f.doc, f.session, ui, f.limits());
    REQUIRE(r.ok);
    const auto bounds = f.doc.objectBounds(f.doc.objectIds().front());
    REQUIRE(bounds.has_value());
    CHECK(bounds->width() > 49.99);
}

TEST_CASE("gadget run: failures are worded for the dialog", "[gadget][viewmodel][t0331]") {
    using scripting::ScriptStatus;
    scripting::GadgetRunResult run;
    run.script.line = 7;
    run.script.message = "gadget.lua:7: boom";
    run.script.traceback = "stack traceback:\n\t[C]: in ?";
    const auto say = [&](ScriptStatus st) {
        run.script.status = st;
        return describeFailure(run, "Weave");
    };
    for (ScriptStatus st : {ScriptStatus::SyntaxError, ScriptStatus::RuntimeError, ScriptStatus::InstructionLimit,
                            ScriptStatus::MemoryLimit, ScriptStatus::OutputLimit, ScriptStatus::Cancelled,
                            ScriptStatus::Timeout, ScriptStatus::LimitsInvalid}) {
        const GadgetErrorReport e = say(st);
        INFO(static_cast<int>(st));
        CHECK(contains(e.title, "\"Weave\""));
        CHECK(contains(e.summary, "Your drawing was not changed"));
        CHECK(e.status == st);
        CHECK(e.line == 7);
        CHECK(contains(e.detail, "gadget.lua:7: boom"));
        CHECK(contains(e.detail, "stack traceback"));
    }
    CHECK(contains(say(ScriptStatus::RuntimeError).summary, "gadget.lua:7: boom"));
    // a Timeout is not blamed on the script
    const GadgetErrorReport t = say(ScriptStatus::Timeout);
    CHECK(contains(t.title, "took too long on this machine"));
    CHECK(contains(t.summary, "not necessarily a mistake"));
    CHECK_FALSE(contains(say(ScriptStatus::InstructionLimit).title, "machine"));
    CHECK(contains(say(ScriptStatus::SyntaxError).summary, "line 7"));
}

TEST_CASE("gadget run: hostile scripts through the manager", "[gadget][viewmodel][t0331]") {
    Fixture f;
    // an infinite loop
    scripting::ScriptLimits l = f.limits();
    l.maxInstructions = 200'000;
    GadgetRunReport r = GadgetManager::runSource("carvetto.object.create_shape(nil,'circle',{radius=1}) while true do end",
                                                 "Loop", f.doc, f.session, {}, l);
    REQUIRE_FALSE(r.ok);
    CHECK(r.run.script.status == scripting::ScriptStatus::InstructionLimit);
    CHECK(f.doc.objectCount() == 0);
    // the wall-clock deadline, with a fake clock
    std::uint64_t t = 0;
    l = f.limits();
    l.maxInstructions = 1'000'000'000;
    l.deadlineMillis = 20;
    l.clockNanos = [&t] { return t += 1'000'000; };
    r = GadgetManager::runSource("carvetto.layer.create('x') while true do end", "Slow", f.doc, f.session, {}, l);
    REQUIRE_FALSE(r.ok);
    CHECK(r.run.script.status == scripting::ScriptStatus::Timeout);
    CHECK(contains(r.error->title, "took too long on this machine"));
    CHECK(f.doc.layers().size() == 1);
    // a script that does not compile
    r = GadgetManager::runSource("x = = 1", "Broken", f.doc, f.session, {}, f.limits());
    REQUIRE_FALSE(r.ok);
    CHECK(r.run.script.status == scripting::ScriptStatus::SyntaxError);
}

TEST_CASE("gadget run: from the gadgets folder", "[gadget][viewmodel][t0331]") {
    Scratch s("run");
    makeGadget(s.root, "circle", "Circle", "", "", 1, "carvetto.object.create_shape(nil, 'circle', {radius = 3})");
    makeGadget(s.root, "future", "Future", "", "", 9);
    makeGadget(s.root, "bad", "Bad", "", "", 1, "error('nope')");
    GadgetManager m(s.root);
    Fixture f;
    REQUIRE(m.gadgets().size() == 3);
    const auto indexOf = [&](const char* folder) {
        for (std::size_t i = 0; i < m.gadgets().size(); ++i) {
            if (m.gadgets()[i].folderName == folder) {
                return i;
            }
        }
        return std::size_t{99};
    };
    GadgetRunReport r = m.run(indexOf("circle"), f.doc, f.session, {}, f.limits());
    REQUIRE(r.ok);
    CHECK(f.doc.objectCount() == 1);
    CHECK(f.doc.undoStack().undoText() == "Circle");

    r = m.run(indexOf("future"), f.doc, f.session, {}, f.limits());
    CHECK_FALSE(r.ok);
    CHECK(contains(r.error->summary, "version 9"));
    r = m.run(indexOf("bad"), f.doc, f.session, {}, f.limits());
    CHECK_FALSE(r.ok);
    CHECK(contains(r.error->detail, "gadget.lua:1: nope"));
    r = m.run(42, f.doc, f.session, {}, f.limits());
    CHECK_FALSE(r.ok);
    CHECK(f.doc.objectCount() == 1);
    CHECK(f.doc.undoStack().count() == 1);

    // the script is read when it runs: an edit takes effect without a rescan
    write(s.root / "circle" / "gadget.lua", "carvetto.object.create_shape(nil, 'circle', {radius = 9})");
    r = m.run(indexOf("circle"), f.doc, f.session, {}, f.limits());
    REQUIRE(r.ok);
    CHECK(f.doc.objectCount() == 2);
}

TEST_CASE("gadget manager: menu strings are escaped for Qt mnemonics", "[gadget][viewmodel][t0331]") {
    CHECK(escapeMenuMnemonics("Plain") == "Plain");
    CHECK(escapeMenuMnemonics("&Hack") == "&&Hack");
    CHECK(escapeMenuMnemonics("Fish & Chips") == "Fish && Chips");
    CHECK(escapeMenuMnemonics("a&&b") == "a&&&&b");
    CHECK(escapeMenuMnemonics("") == "");
}

TEST_CASE("gadget manager: an Abort pressed in a dialog cancels the run and rolls it back", "[gadget][viewmodel][t0331]") {
    Fixture f;
    std::atomic<bool> abort{false};
    scripting::GadgetUi ui;
    ui.message = [&abort](const std::string&) { abort = true; };
    const GadgetRunReport r = GadgetManager::runSource("carvetto.layer.create('x') carvetto.ui.message('hi')", "Dialogs",
                                                       f.doc, f.session, ui, f.limits(), &abort);
    CHECK_FALSE(r.ok);
    CHECK(r.run.script.status == scripting::ScriptStatus::Cancelled);
    REQUIRE(r.error.has_value());
    CHECK(contains(r.error->summary, "not changed"));
    CHECK(f.doc.layers().size() == 1);
}
