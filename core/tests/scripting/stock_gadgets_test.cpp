// T-0332: the stock gadgets (gadgets/stock/*, ADR-0113) run against a document with scripted
// services: documented output on known inputs, one undo step, rollback, refusals; plus the API
// additions they need (ui.choose_folder, file.read_dxf_folder).
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/io/dxf.hpp"
#include "carvetto/scripting/gadget_api.hpp"
#include "carvetto/scripting/gadget_manifest.hpp"

using namespace carvetto;
using namespace carvetto::scripting;
using document::Document;
using Catch::Matchers::WithinAbs;
namespace fs = std::filesystem;

namespace {

std::string readFile(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

std::string stockSource(const std::string& folder) {
    return readFile(fs::path(CARVETTO_STOCK_GADGETS_DIR) / folder / "gadget.lua");
}

bool contains(const std::string& s, const char* part) {
    return s.find(part) != std::string::npos;
}

/// Scripted services: forms answer with the field defaults, overridden by `answers`; tools and layers
/// are filled in; every message is recorded.
struct Harness {
    Document doc;
    std::map<std::string, ScriptValue> answers;
    std::vector<std::string> messages;
    std::vector<std::string> formKeys;
    bool cancelForm{false};
    std::optional<std::string> folder;
    GadgetServices sv;

    Harness() {
        sv.ui.form = [this](const FormSpec& spec) -> std::optional<ScriptValue::Map> {
            if (cancelForm) {
                return std::nullopt;
            }
            ScriptValue::Map m;
            for (const FormField& f : spec.fields) {
                formKeys.push_back(f.name);
                const auto it = answers.find(f.name);
                if (it != answers.end()) {
                    m.items.emplace_back(f.name, it->second);
                } else if (f.kind == FormFieldKind::Tool) {
                    m.items.emplace_back(f.name, ScriptValue(std::string("em3")));
                } else if (f.kind == FormFieldKind::Layer) {
                    m.items.emplace_back(f.name, ScriptValue(static_cast<std::int64_t>(doc.activeLayer().value)));
                } else {
                    m.items.emplace_back(f.name, f.defaultValue);
                }
            }
            return m;
        };
        sv.ui.message = [this](const std::string& t) { messages.push_back(t); };
        sv.ui.confirm = [](const std::string&) { return true; };
        sv.ui.chooseFolder = [this](const std::string&) { return folder; };
        sv.toolSnapshot = [](const std::string& id) -> std::optional<toolpath::ToolSnapshot> {
            if (id != "em3") {
                return std::nullopt;
            }
            toolpath::ToolSnapshot s;
            s.name = "3 mm end mill";
            s.diameter = 3.0;
            return s;
        };
        sv.tools = [] { return std::vector<GadgetTool>{{"em3", "3 mm end mill", 3.0, "end_mill"}}; };
        sv.createToolpath = [this](const ToolpathRequest& req) {
            ToolpathCreated out;
            document::ToolpathRecord rec;
            rec.name = "T " + req.kind;
            rec.moves = document::shareMoves({toolpath::Move::rapid({0, 0, 5})});
            rec.selection.objects = req.objects;
            auto cmd = std::make_unique<document::AddToolpathCommand>(rec);
            const document::AddToolpathCommand* raw = cmd.get();
            requests.push_back(req);
            out.ok = doc.execute(std::move(cmd));
            out.ids.push_back(raw->id());
            return out;
        };
    }

    std::vector<ToolpathRequest> requests;

    GadgetRunResult run(const std::string& gadget) {
        GadgetRunOptions o;
        o.limits.maxInstructions = 20'000'000;
        o.limits.deadlineMillis = 0;
        o.undoText = gadget;
        return runGadget(doc, sv, stockSource(gadget), gadget + "/gadget.lua", o);
    }

};

/// Makes shapes and selects them (the setup itself is one more undo step).
void setup(Harness& h, const std::string& src) {
    GadgetRunOptions o;
    o.limits.deadlineMillis = 0;
    const GadgetRunResult r = runGadget(h.doc, GadgetServices{}, src, "setup", o);
    INFO(r.script.message);
    REQUIRE(r.ok());
}

geometry::Box2 boundsOf(const Document& d, document::ObjectId id) {
    const auto b = d.objectBounds(id);
    REQUIRE(b.has_value());
    return *b;
}

ScriptValue num(double v) { return ScriptValue(v); }
ScriptValue txt(const char* v) { return ScriptValue(std::string(v)); }

const std::vector<toolpath::Move>& movesOf(const Document& d, std::size_t i) {
    REQUIRE(d.toolpaths().size() > i);
    REQUIRE(d.toolpaths()[i].moves != nullptr);
    return *d.toolpaths()[i].moves;
}

std::size_t countKind(const std::vector<toolpath::Move>& m, toolpath::MoveKind k) {
    return static_cast<std::size_t>(std::count_if(m.begin(), m.end(), [k](const toolpath::Move& x) { return x.kind == k; }));
}

}  // namespace

TEST_CASE("stock gadgets: the shipped folder is a valid set of gadgets", "[gadget][t0332]") {
    const GadgetDiscovery d = discoverGadgets(CARVETTO_STOCK_GADGETS_DIR);
    CHECK(d.skipped.empty());
    REQUIRE(d.gadgets.size() == 5);
    std::vector<std::string> names;
    for (const GadgetInfo& g : d.gadgets) {
        INFO(g.folderName << ": " << g.problem);
        CHECK(g.runnable());
        CHECK(g.manifest.apiVersion <= kGadgetApiVersion);
        CHECK_FALSE(g.manifest.description.empty());
        names.push_back(g.manifest.name);
    }
    std::sort(names.begin(), names.end());
    CHECK(names == std::vector<std::string>{"Celtic Weave Creator", "DXF Batch Processor", "Drag Knife Toolpath",
                                            "Job Setup Sheet Editor", "Keyhole Toolpath"});
}

TEST_CASE("celtic weave: alternating 4 x 4 grid", "[gadget][t0332]") {
    Harness h;
    const GadgetRunResult r = h.run("celtic-weave");
    INFO(r.script.message);
    REQUIRE(r.ok());
    CHECK(r.committed);
    CHECK(h.doc.undoStack().count() == 1);
    CHECK(h.doc.layers().size() == 3);
    // every strand (4 vertical + 4 horizontal) is cut twice: 3 pieces each, plus the cross section
    CHECK(r.createdObjects.size() == 25);
    CHECK(h.doc.selection().ids().size() == 24);
    // total strand length: 8 strands of 80 mm minus 2 gaps of 6 mm each
    double total = 0.0;
    for (const document::ObjectId id : h.doc.selection().ids()) {
        const geometry::Box2 b = boundsOf(h.doc, id);
        total += (b.max.x - b.min.x) + (b.max.y - b.min.y);
    }
    CHECK_THAT(total, WithinAbs(8 * (80.0 - 12.0), 1e-6));
    // one undo step takes everything away
    h.doc.undoStack().undo(h.doc);
    CHECK(h.doc.objectCount() == 0);
    CHECK(h.doc.layers().size() == 1);
}

TEST_CASE("celtic weave: patterns and cross sections", "[gadget][t0332]") {
    SECTION("cut neither leaves one piece per strand") {
        Harness h;
        h.answers["pattern"] = txt("Cut neither");
        h.answers["columns"] = num(3);
        h.answers["rows"] = num(2);
        const GadgetRunResult r = h.run("celtic-weave");
        REQUIRE(r.ok());
        CHECK(h.doc.selection().ids().size() == 5);
        CHECK(r.createdObjects.size() == 6);
    }
    SECTION("alternating starts with a cut vertical strand at the bottom left crossing") {
        Harness h;
        h.answers["columns"] = num(1);
        h.answers["rows"] = num(1);
        REQUIRE(h.run("celtic-weave").ok());
        // the vertical strand (x = 0, from -10 to 10) is cut at the crossing: two pieces of 7 mm;
        // the horizontal strand runs over, whole: 20 mm
        int vertical7 = 0;
        int horizontal20 = 0;
        for (const document::ObjectId id : h.doc.selection().ids()) {
            const geometry::Box2 b = boundsOf(h.doc, id);
            if (std::abs(b.max.x - b.min.x) < 1e-9 && std::abs((b.max.y - b.min.y) - 7.0) < 1e-9) {
                ++vertical7;
            }
            if (std::abs(b.max.y - b.min.y) < 1e-9 && std::abs((b.max.x - b.min.x) - 20.0) < 1e-9) {
                ++horizontal20;
            }
        }
        CHECK(vertical7 == 2);
        CHECK(horizontal20 == 1);
    }
    SECTION("all cut vertical breaks only the vertical strands") {
        Harness h;
        h.answers["pattern"] = txt("All cut vertical");
        h.answers["columns"] = num(2);
        h.answers["rows"] = num(3);
        REQUIRE(h.run("celtic-weave").ok());
        // 2 vertical strands cut 3 times = 4 pieces each; 3 horizontal strands whole
        CHECK(h.doc.selection().ids().size() == 2 * 4 + 3);
    }
    SECTION("custom pattern, top row first") {
        Harness h;
        h.answers["pattern"] = txt("Custom");
        h.answers["custom"] = txt("vn/hh");  // 2 x 2: top row "V N", bottom row "H H"
        h.answers["columns"] = num(2);
        h.answers["rows"] = num(2);
        REQUIRE(h.run("celtic-weave").ok());
        // vertical strand 0 cut at the top (y = 20): 2 pieces; vertical 1 whole.
        // horizontal bottom (y = 0) cut at both x: 3 pieces; horizontal top whole.
        CHECK(h.doc.selection().ids().size() == 2 + 1 + 3 + 1);
    }
    SECTION("a custom pattern of the wrong size is refused and changes nothing") {
        Harness h;
        h.answers["pattern"] = txt("Custom");
        h.answers["custom"] = txt("VH/HV/VH");
        const GadgetRunResult r = h.run("celtic-weave");
        REQUIRE(r.ok());
        CHECK_FALSE(r.committed);
        CHECK(h.doc.objectCount() == 0);
        REQUIRE(h.messages.size() == 1);
        CHECK(contains(h.messages[0], "4 rows"));
        h.messages.clear();
        h.answers["custom"] = txt("VH/HV/VH/HV");
        REQUIRE(h.run("celtic-weave").ok());
        CHECK(contains(h.messages.at(0), "4 letters"));
        h.messages.clear();
        h.answers["custom"] = txt("VHXV/HVHV/VHVH/HVHV");
        REQUIRE(h.run("celtic-weave").ok());
        CHECK(contains(h.messages.at(0), "Only V, H and N"));
        CHECK(h.doc.objectCount() == 0);
    }
    SECTION("a strand width that is not smaller than the spacing is refused") {
        Harness h;
        h.answers["strand_width"] = num(20);
        const GadgetRunResult r = h.run("celtic-weave");
        REQUIRE(r.ok());
        CHECK_FALSE(r.committed);
        CHECK(h.doc.objectCount() == 0);
        CHECK(h.messages.size() == 1);
    }
    SECTION("cross sections have the width and height asked for") {
        for (const char* kind : {"Round", "Flat", "Pointed"}) {
            INFO(kind);
            Harness h;
            h.answers["section"] = txt(kind);
            h.answers["section_height"] = num(4);
            h.answers["strand_width"] = num(8);
            h.answers["pattern"] = txt("Cut neither");
            h.answers["columns"] = num(1);
            h.answers["rows"] = num(1);
            const GadgetRunResult r = h.run("celtic-weave");
            REQUIRE(r.ok());
            const geometry::Box2 b = boundsOf(h.doc, r.createdObjects.back());
            CHECK_THAT(b.max.x - b.min.x, WithinAbs(8.0, 1e-6));
            CHECK_THAT(b.max.y - b.min.y, WithinAbs(4.0, 1e-6));
            CHECK_THAT(b.min.y, WithinAbs(0.0, 1e-6));
        }
    }
    SECTION("cancelling the form changes nothing") {
        Harness h;
        h.cancelForm = true;
        const GadgetRunResult r = h.run("celtic-weave");
        REQUIRE(r.ok());
        CHECK_FALSE(r.committed);
        CHECK(h.doc.objectCount() == 0);
        CHECK(h.doc.layers().size() == 1);
    }
}

TEST_CASE("celtic weave: the largest grid runs within the default script limits", "[gadget][t0332]") {
    Harness h;
    h.answers["columns"] = num(30);
    h.answers["rows"] = num(30);
    h.answers["spacing"] = num(10);
    h.answers["strand_width"] = num(3);
    const GadgetRunOptions defaults;  // the host's default instruction, memory and time limits
    const GadgetRunResult r = runGadget(h.doc, h.sv, stockSource("celtic-weave"), "celtic-weave", defaults);
    INFO(r.script.message);
    REQUIRE(r.ok());
    // 60 strands, each cut at half of its 30 crossings: 16 pieces; plus the cross section
    CHECK(r.createdObjects.size() == 60 * 16 + 1);
}

TEST_CASE("drag knife: square with swivels, passes and lead-in", "[gadget][t0332]") {
    Harness h;
    setup(h, "local id = carvetto.object.create_shape(nil, 'rect', {cx = 10, cy = 10, width = 20, height = 20}) "
             "carvetto.selection.set({id})");
    h.answers["offset"] = num(1);
    h.answers["cut_depth"] = num(2);
    h.answers["swivel_depth"] = num(0.5);
    h.answers["passes"] = num(2);
    const GadgetRunResult r = h.run("drag-knife");
    INFO(r.script.message);
    REQUIRE(r.ok());
    REQUIRE(h.doc.toolpaths().size() == 1);
    const auto& m = movesOf(h.doc, 0);
    // four corners (the start corner included) per pass, each a swivel arc of the blade offset
    const std::size_t arcs = countKind(m, toolpath::MoveKind::ArcCW) + countKind(m, toolpath::MoveKind::ArcCCW);
    CHECK(arcs == 8);
    CHECK(countKind(m, toolpath::MoveKind::ArcCW) + countKind(m, toolpath::MoveKind::ArcCCW) == arcs);
    double deepest = 0.0;
    for (std::size_t i = 0; i < m.size(); ++i) {
        const toolpath::Move& mv = m[i];
        deepest = std::min(deepest, mv.to.z);
        if (mv.kind == toolpath::MoveKind::ArcCW || mv.kind == toolpath::MoveKind::ArcCCW) {
            // radius = blade offset around a corner of the square, at the swivel depth (or the
            // pass depth when that is shallower)
            const double dx = mv.to.x - mv.centre.x;
            const double dy = mv.to.y - mv.centre.y;
            CHECK_THAT(std::sqrt(dx * dx + dy * dy), WithinAbs(1.0, 1e-9));
            const bool corner = (std::abs(mv.centre.x) < 1e-9 || std::abs(mv.centre.x - 20) < 1e-9) &&
                                (std::abs(mv.centre.y) < 1e-9 || std::abs(mv.centre.y - 20) < 1e-9);
            CHECK(corner);
            CHECK_THAT(mv.to.z, WithinAbs(-0.5, 1e-9));
            // the rectangle runs counter-clockwise, so the swivels turn counter-clockwise
            CHECK(mv.kind == toolpath::MoveKind::ArcCCW);
        }
    }
    CHECK_THAT(deepest, WithinAbs(-2.0, 1e-9));
    // first pass at -1: 1 mm; second at -2: nothing deeper
    CHECK(m.front().kind == toolpath::MoveKind::Rapid);
    CHECK(m.back().kind == toolpath::MoveKind::Retract);
    CHECK_THAT(m.back().to.z, WithinAbs(5.0, 1e-9));
    // lead-in: the first cutting feed runs one blade offset along the first side (rectangle start is a corner)
    const auto firstFeed = std::find_if(m.begin(), m.end(), [](const toolpath::Move& x) { return x.kind == toolpath::MoveKind::Feed; });
    REQUIRE(firstFeed != m.end());
    const double sx = m.front().to.x;
    const double sy = m.front().to.y;
    CHECK_THAT(std::hypot(firstFeed->to.x - sx, firstFeed->to.y - sy), WithinAbs(1.0, 1e-9));
    CHECK(h.doc.undoStack().count() == 2);  // setup + gadget
}

TEST_CASE("drag knife: a gentle bend is followed without a swivel; a sharp one swivels", "[gadget][t0332]") {
    for (const bool sharp : {false, true}) {
        INFO(sharp);
        Harness h;
        const std::string bend = sharp ? "{20, 20}" : "{40, 2}";  // 90 degrees or about 5.7 degrees
        setup(h, "local id = carvetto.object.create_contour(nil, {{0, 0}, {20, 0}, " + bend + "}, false) "
                 "carvetto.selection.set({id})");
        h.answers["tolerance"] = num(15);
        h.answers["offset"] = num(1);
        REQUIRE(h.run("drag-knife").ok());
        const auto& m = movesOf(h.doc, 0);
        const std::size_t arcs = countKind(m, toolpath::MoveKind::ArcCW) + countKind(m, toolpath::MoveKind::ArcCCW);
        CHECK(arcs == (sharp ? 1u : 0u));
        if (sharp) {
            // a left turn at (20, 0): swivel counter-clockwise from (21, 0) to (20, 1)
            const auto it = std::find_if(m.begin(), m.end(), [](const toolpath::Move& x) { return x.kind == toolpath::MoveKind::ArcCCW; });
            REQUIRE(it != m.end());
            CHECK_THAT(it->to.x, WithinAbs(20.0, 1e-9));
            CHECK_THAT(it->to.y, WithinAbs(1.0, 1e-9));
            CHECK_THAT(it->centre.x, WithinAbs(20.0, 1e-9));
            CHECK_THAT(it->centre.y, WithinAbs(0.0, 1e-9));
        }
        // the cut ends with the blade tip at the end point: pivot one offset past it
        const toolpath::Move& last = m[m.size() - 2];
        CHECK(last.kind == toolpath::MoveKind::Feed);
        if (sharp) {
            CHECK_THAT(last.to.x, WithinAbs(20.0, 1e-9));
            CHECK_THAT(last.to.y, WithinAbs(21.0, 1e-9));
        }
    }
}

TEST_CASE("drag knife: a clockwise turn swivels clockwise; arcs of a bulge are followed", "[gadget][t0332]") {
    Harness h;
    setup(h, "local id = carvetto.object.create_contour(nil, {{0, 0}, {20, 0}, {20, -20}}, false) "
             "carvetto.selection.set({id})");
    REQUIRE(h.run("drag-knife").ok());
    const auto& m = movesOf(h.doc, 0);
    CHECK(countKind(m, toolpath::MoveKind::ArcCW) == 1);
    CHECK(countKind(m, toolpath::MoveKind::ArcCCW) == 0);

    Harness c;
    // a semicircle of radius 10 is flattened into steps of at most 5 degrees: each smaller than the tolerance
    setup(c, "local id = carvetto.object.create_contour(nil, {{0, 0}, {20, 0}}, false, {1, 0}) carvetto.selection.set({id})");
    REQUIRE(c.run("drag-knife").ok());
    const auto& cm = movesOf(c.doc, 0);
    CHECK(countKind(cm, toolpath::MoveKind::ArcCW) + countKind(cm, toolpath::MoveKind::ArcCCW) == 0);
    CHECK(countKind(cm, toolpath::MoveKind::Feed) > 30);
    // the pivot never strays from the true arc by more than the blade offset
    for (const toolpath::Move& mv : cm) {
        if (mv.kind == toolpath::MoveKind::Feed) {
            const double rr = std::hypot(mv.to.x - 10.0, mv.to.y);
            CHECK(std::abs(rr - 10.0) <= 0.5 + 1e-9);
        }
    }
}

TEST_CASE("drag knife: without vector start points the contours are ordered and reversed", "[gadget][t0332]") {
    const char* src =
        "local a = carvetto.object.create_contour(nil, {{0, 0}, {10, 0}}, false) "
        "local b = carvetto.object.create_contour(nil, {{30, 0}, {11, 0}}, false) "
        "carvetto.selection.set({a, b})";
    for (const bool useStart : {true, false}) {
        INFO(useStart);
        Harness h;
        setup(h, src);
        h.answers["use_start"] = ScriptValue(useStart);
        REQUIRE(h.run("drag-knife").ok());
        const auto& m = movesOf(h.doc, 0);
        std::vector<double> rapidXs;
        for (const toolpath::Move& mv : m) {
            if (mv.kind == toolpath::MoveKind::Rapid) {
                rapidXs.push_back(mv.to.x);
            }
        }
        REQUIRE(rapidXs.size() == 2);
        CHECK_THAT(rapidXs[0], WithinAbs(0.0, 1e-9));
        // as drawn the second starts at x = 30; reordered it is entered from its near end, x = 11
        CHECK_THAT(rapidXs[1], WithinAbs(useStart ? 30.0 : 11.0, 1e-9));
    }
}

TEST_CASE("drag knife: refusals", "[gadget][t0332]") {
    SECTION("nothing selected") {
        Harness h;
        const GadgetRunResult r = h.run("drag-knife");
        REQUIRE(r.ok());
        CHECK_FALSE(r.committed);
        CHECK(h.messages.size() == 1);
    }
    SECTION("swivel deeper than the cut") {
        Harness h;
        setup(h, "carvetto.selection.set({carvetto.object.create_shape(nil, 'circle', {radius = 5})})");
        h.answers["swivel_depth"] = num(3);
        h.answers["cut_depth"] = num(1);
        const GadgetRunResult r = h.run("drag-knife");
        REQUIRE(r.ok());
        CHECK(h.doc.toolpaths().empty());
        CHECK(contains(h.messages.at(0), "swivel depth"));
    }
}

TEST_CASE("keyhole: moves and preview vectors", "[gadget][t0332]") {
    Harness h;
    setup(h, "local a = carvetto.object.create_shape(nil, 'circle', {cx = 10, cy = 10, radius = 2}) "
             "local b = carvetto.object.create_shape(nil, 'circle', {cx = 50, cy = 10, radius = 2}) "
             "carvetto.selection.set({a, b})");
    h.answers["direction"] = num(0);
    h.answers["length"] = num(12);
    h.answers["depth"] = num(4);
    h.answers["preview"] = ScriptValue(true);
    h.answers["entry_diameter"] = num(10);
    h.answers["slot_diameter"] = num(4);
    const std::size_t before = h.doc.objectCount();
    const GadgetRunResult r = h.run("keyhole");
    INFO(r.script.message);
    REQUIRE(r.ok());
    const auto& m = movesOf(h.doc, 0);
    REQUIRE(m.size() == 8);
    CHECK_THAT(m[0].to.x, WithinAbs(10.0, 1e-9));
    CHECK(m[1].kind == toolpath::MoveKind::Plunge);
    CHECK_THAT(m[1].to.z, WithinAbs(-4.0, 1e-9));
    CHECK_THAT(m[2].to.x, WithinAbs(22.0, 1e-9));
    CHECK_THAT(m[2].to.y, WithinAbs(10.0, 1e-9));
    CHECK(m[3].kind == toolpath::MoveKind::Retract);
    CHECK_THAT(m[4].to.x, WithinAbs(50.0, 1e-9));
    CHECK_THAT(m[6].to.x, WithinAbs(62.0, 1e-9));
    // per keyhole: an entry circle and a slot outline
    CHECK(h.doc.objectCount() == before + 4);
    const geometry::Box2 b = boundsOf(h.doc, r.createdObjects[1]);
    // from one slot radius behind the entry centre to one slot radius past the end; slot diameter tall
    CHECK_THAT(b.min.x, WithinAbs(10.0 - 2.0, 1e-6));
    CHECK_THAT(b.max.x, WithinAbs(22.0 + 2.0, 1e-6));
    CHECK_THAT(b.max.y - b.min.y, WithinAbs(4.0, 1e-6));
    const geometry::Box2 hb = boundsOf(h.doc, r.createdObjects[0]);
    CHECK_THAT(hb.max.x - hb.min.x, WithinAbs(10.0, 1e-6));
}

TEST_CASE("keyhole: slot direction and no preview", "[gadget][t0332]") {
    Harness h;
    setup(h, "carvetto.selection.set({carvetto.object.create_shape(nil, 'circle', {cx = 5, cy = 5, radius = 1})})");
    h.answers["direction"] = num(90);
    h.answers["length"] = num(7);
    const std::size_t before = h.doc.objectCount();
    REQUIRE(h.run("keyhole").ok());
    const auto& m = movesOf(h.doc, 0);
    CHECK_THAT(m[2].to.x, WithinAbs(5.0, 1e-9));
    CHECK_THAT(m[2].to.y, WithinAbs(12.0, 1e-9));
    CHECK(h.doc.objectCount() == before);
}

TEST_CASE("keyhole: nothing selected, or an unknown tool, changes nothing", "[gadget][t0332]") {
    Harness h;
    const GadgetRunResult r = h.run("keyhole");
    REQUIRE(r.ok());
    CHECK_FALSE(r.committed);
    CHECK(h.messages.size() == 1);

    Harness g;
    setup(g, "carvetto.selection.set({carvetto.object.create_shape(nil, 'circle', {radius = 1})})");
    g.answers["tool"] = txt("nope");
    g.answers["preview"] = ScriptValue(true);
    const std::size_t objects = g.doc.objectCount();
    const GadgetRunResult bad = g.run("keyhole");
    CHECK_FALSE(bad.ok());
    CHECK(g.doc.objectCount() == objects);
    CHECK(g.doc.toolpaths().empty());
}

TEST_CASE("job sheet editor: stores colours and sections, restores defaults", "[gadget][t0332]") {
    Harness h;
    h.answers["header"] = txt("#1a3c5e");
    h.answers["footer"] = txt("00ff00");
    h.answers["notes"] = ScriptValue(false);
    h.answers["toolpaths"] = ScriptValue(false);
    const GadgetRunResult r = h.run("job-sheet-editor");
    INFO(r.script.message);
    REQUIRE(r.ok());
    const auto& v = h.doc.variables();
    CHECK(v.get("jobsheet_header_rgb") == 0x1A3C5E);
    CHECK(v.get("jobsheet_footer_rgb") == 0x00FF00);
    CHECK(v.get("jobsheet_show_notes") == 0.0);
    CHECK(v.get("jobsheet_show_toolpaths") == 0.0);
    CHECK(v.get("jobsheet_show_layout") == 1.0);
    CHECK(h.doc.undoStack().count() == 1);

    // the form starts from what is stored
    Harness again;
    again.doc.variables().set("jobsheet_header_rgb", 255.0);
    GadgetRunOptions o;
    o.limits.deadlineMillis = 0;
    bool sawDefault = false;
    again.sv.ui.form = [&](const FormSpec& spec) -> std::optional<ScriptValue::Map> {
        for (const FormField& f : spec.fields) {
            if (f.name == "header") {
                sawDefault = f.defaultValue.isString() && f.defaultValue.asString() == "#0000FF";
            }
        }
        return std::nullopt;
    };
    REQUIRE(runGadget(again.doc, again.sv, stockSource("job-sheet-editor"), "g", o).ok());
    CHECK(sawDefault);

    // restore default
    Harness restore;
    restore.doc.variables().set("jobsheet_header_rgb", 255.0);
    restore.doc.variables().set("jobsheet_show_layout", 0.0);
    restore.answers["restore"] = ScriptValue(true);
    REQUIRE(restore.run("job-sheet-editor").ok());
    CHECK(restore.doc.variables().get("jobsheet_header_rgb") == 0.0);
    CHECK(restore.doc.variables().get("jobsheet_show_layout") == 1.0);
}

TEST_CASE("job sheet editor: a bad colour is refused and nothing is stored", "[gadget][t0332]") {
    Harness h;
    h.answers["header"] = txt("blue");
    const GadgetRunResult r = h.run("job-sheet-editor");
    REQUIRE(r.ok());
    CHECK_FALSE(r.committed);
    CHECK(h.doc.variables().empty());
    CHECK(h.messages.size() == 1);
}

namespace {

/// A temporary folder removed at the end of the test.
struct TempDir {
    fs::path path;
    explicit TempDir(const char* tag) {
        static int counter = 0;
        path = fs::temp_directory_path() / (std::string("carvetto-t0332-") + tag + "-" + std::to_string(++counter) + "-" +
                                            std::to_string(static_cast<long long>(std::hash<std::string>{}(__FILE__)) % 100000));
        fs::remove_all(path);
        fs::create_directories(path);
    }
    ~TempDir() {
        std::error_code ec;
        fs::remove_all(path, ec);
    }
};

geometry::Contour box(double x0, double y0, double x1, double y1) {
    geometry::Contour c;
    c.closed = true;
    c.spans.push_back(geometry::LineSpan{{x0, y0}, {x1, y0}});
    c.spans.push_back(geometry::LineSpan{{x1, y0}, {x1, y1}});
    c.spans.push_back(geometry::LineSpan{{x1, y1}, {x0, y1}});
    c.spans.push_back(geometry::LineSpan{{x0, y1}, {x0, y0}});
    return c;
}

void writeDxf(const fs::path& file, const std::vector<geometry::Contour>& contours) {
    const std::vector<io::DxfLayer> layers{io::DxfLayer{}};
    std::vector<io::DxfContour> cs;
    for (const geometry::Contour& c : contours) {
        cs.push_back(io::DxfContour{0, c});
    }
    const auto bytes = io::exportDxf(layers, cs);
    REQUIRE(bytes.ok());
    std::ofstream out(file, std::ios::binary);
    out.write(reinterpret_cast<const char*>(bytes.value().data()), static_cast<std::streamsize>(bytes.value().size()));
}

}  // namespace

TEST_CASE("dxf batch: imports every file onto its own layer in a row", "[gadget][t0332]") {
    TempDir dir("batch");
    writeDxf(dir.path / "b_part.dxf", {box(100, 50, 130, 60)});
    writeDxf(dir.path / "a_part.DXF", {box(5, 5, 15, 25), box(7, 7, 9, 9)});
    { std::ofstream(dir.path / "broken.dxf") << "this is not a dxf file"; }
    { std::ofstream(dir.path / "notes.txt") << "ignored"; }
    fs::create_directories(dir.path / "sub.dxf");  // a folder named like a drawing is not a file

    Harness h;
    h.folder = dir.path.string();
    h.answers["gap"] = num(10);
    const GadgetRunResult r = h.run("dxf-batch");
    INFO(r.script.message);
    REQUIRE(r.ok());
    CHECK(h.doc.undoStack().count() == 1);
    CHECK(h.doc.layers().size() == 3);
    CHECK(h.doc.layers()[1].name == "a_part");
    CHECK(h.doc.layers()[2].name == "b_part");
    REQUIRE(r.createdObjects.size() == 3);
    // a_part: 10 x 20 from x = 0, y = 0; b_part: 30 x 10, after the gap
    const geometry::Box2 a = boundsOf(h.doc, r.createdObjects[0]);
    CHECK_THAT(a.min.x, WithinAbs(0.0, 1e-6));
    CHECK_THAT(a.min.y, WithinAbs(0.0, 1e-6));
    CHECK_THAT(a.max.x, WithinAbs(10.0, 1e-6));
    const geometry::Box2 b = boundsOf(h.doc, r.createdObjects[2]);
    CHECK_THAT(b.min.x, WithinAbs(20.0, 1e-6));
    CHECK_THAT(b.min.y, WithinAbs(0.0, 1e-6));
    CHECK_THAT(b.max.x, WithinAbs(50.0, 1e-6));
    CHECK(h.doc.findObject(r.createdObjects[2])->layer == h.doc.layers()[2].id);
    // the broken file is reported, the others went in
    REQUIRE(h.messages.size() == 1);
    CHECK(contains(h.messages[0], "2 files imported"));
    CHECK(contains(h.messages[0], "broken.dxf"));
    CHECK(h.doc.toolpaths().empty());
}

TEST_CASE("dxf batch: optional profile toolpath per part", "[gadget][t0332]") {
    TempDir dir("profile");
    writeDxf(dir.path / "one.dxf", {box(0, 0, 10, 10)});
    writeDxf(dir.path / "two.dxf", {box(0, 0, 20, 10)});
    Harness h;
    h.folder = dir.path.string();
    h.answers["profile"] = ScriptValue(true);
    h.answers["cut_depth"] = num(2.5);
    REQUIRE(h.run("dxf-batch").ok());
    REQUIRE(h.requests.size() == 2);
    CHECK(h.requests[0].kind == "profile");
    CHECK(h.requests[0].objects.size() == 1);
    CHECK(h.requests[0].params.find("cutDepth")->asNumber() == 2.5);
    CHECK(h.requests[0].params.find("name")->asString() == "one");
    CHECK(h.doc.toolpaths().size() == 2);
}

TEST_CASE("dxf batch: cancelling, empty folders and refusals", "[gadget][t0332]") {
    SECTION("the folder dialog is cancelled") {
        Harness h;
        h.folder = std::nullopt;
        const GadgetRunResult r = h.run("dxf-batch");
        REQUIRE(r.ok());
        CHECK_FALSE(r.committed);
    }
    SECTION("a folder without DXF files") {
        TempDir dir("empty");
        Harness h;
        h.folder = dir.path.string();
        const GadgetRunResult r = h.run("dxf-batch");
        REQUIRE(r.ok());
        CHECK_FALSE(r.committed);
        CHECK(h.messages.size() == 1);
    }
    SECTION("cancelling the options form after choosing leaves the drawing alone") {
        TempDir dir("cancel");
        writeDxf(dir.path / "p.dxf", {box(0, 0, 1, 1)});
        Harness h;
        h.folder = dir.path.string();
        h.cancelForm = true;
        const GadgetRunResult r = h.run("dxf-batch");
        REQUIRE(r.ok());
        CHECK_FALSE(r.committed);
        CHECK(h.doc.objectCount() == 0);
    }
}

TEST_CASE("file.read_dxf_folder: only chosen folders, caps, symlinks", "[gadget][t0332][security]") {
    TempDir dir("api");
    writeDxf(dir.path / "a.dxf", {box(0, 0, 1, 1)});
    writeDxf(dir.path / "b.dxf", {box(0, 0, 1, 1)});
    Harness h;
    h.folder = dir.path.string();
    GadgetRunOptions o;
    o.limits.deadlineMillis = 0;
    const auto run = [&](const std::string& src) { return runGadget(h.doc, h.sv, src, "g", o); };

    SECTION("a path the user did not choose is refused") {
        const std::string p = dir.path.string();
        std::string esc;
        for (const char c : p) {
            esc += (c == '\\') ? std::string("\\\\") : std::string(1, c);
        }
        const GadgetRunResult r = run("carvetto.file.read_dxf_folder('" + esc + "')");
        CHECK_FALSE(r.ok());
        CHECK(contains(r.script.message, "chose"));
        const GadgetRunResult r2 = run("carvetto.ui.choose_folder('x') return carvetto.file.read_dxf_folder('/')");
        CHECK_FALSE(r2.ok());
        const GadgetRunResult r3 = run("local f = carvetto.ui.choose_folder('x') return carvetto.file.read_dxf_folder(f .. '/..')");
        CHECK_FALSE(r3.ok());
    }
    SECTION("the chosen folder is listed in name order") {
        const GadgetRunResult r = run("local f = carvetto.ui.choose_folder('x') local l = carvetto.file.read_dxf_folder(f) "
                                      "return #l, l[1].name, l[2].file, #l[1].contours, l[1].warnings");
        INFO(r.script.message);
        REQUIRE(r.ok());
        CHECK(r.script.returned[0].asInt() == 2);
        CHECK(r.script.returned[1].asString() == "a");
        CHECK(r.script.returned[2].asString() == "b.dxf");
        CHECK(r.script.returned[3].asInt() == 1);
    }
    SECTION("a cancelled chooser gives nil") {
        h.folder = std::nullopt;
        const GadgetRunResult r = run("return carvetto.ui.choose_folder('x')");
        REQUIRE(r.ok());
        CHECK(r.script.returned[0].isNil());
    }
    SECTION("no chooser service is a script error") {
        h.sv.ui.chooseFolder = nullptr;
        CHECK_FALSE(run("carvetto.ui.choose_folder('x')").ok());
    }
    SECTION("the file count cap is exact") {
        o.gadgetLimits.maxDxfFiles = 2;
        const std::string src = "local f = carvetto.ui.choose_folder('x') return #carvetto.file.read_dxf_folder(f)";
        REQUIRE(run(src).ok());
        writeDxf(dir.path / "c.dxf", {box(0, 0, 1, 1)});
        const GadgetRunResult r = run(src);
        CHECK_FALSE(r.ok());
        CHECK(contains(r.script.message, "more than 2"));
    }
    SECTION("a file over the size cap is reported per file, not fatal") {
        o.gadgetLimits.maxDxfFileBytes = 100;
        const GadgetRunResult r = run("local f = carvetto.ui.choose_folder('x') local l = carvetto.file.read_dxf_folder(f) "
                                      "return l[1].error, #l[1].contours");
        REQUIRE(r.ok());
        CHECK(r.script.returned[0].isString());
        CHECK(r.script.returned[1].asInt() == 0);
    }
    SECTION("the contour cap is exact") {
        o.gadgetLimits.maxDxfContours = 2;
        const std::string src = "local f = carvetto.ui.choose_folder('x') return #carvetto.file.read_dxf_folder(f)";
        REQUIRE(run(src).ok());  // 2 files x 1 contour
        o.gadgetLimits.maxDxfContours = 1;
        CHECK_FALSE(run(src).ok());
    }
    SECTION("a symbolic link named .dxf is not followed") {
        std::error_code ec;
        fs::create_symlink(dir.path / "a.dxf", dir.path / "link.dxf", ec);
        if (!ec) {
            const GadgetRunResult r = run("local f = carvetto.ui.choose_folder('x') return #carvetto.file.read_dxf_folder(f)");
            REQUIRE(r.ok());
            CHECK(r.script.returned[0].asInt() == 2);
        }
    }
}

TEST_CASE("installShippedGadgets: install, keep, update, leave strangers alone", "[gadget][t0332]") {
    TempDir user("ship-user");
    TempDir shipped("ship-src");
    const auto put = [](const fs::path& folder, const std::string& name, const std::string& version, const std::string& script) {
        fs::create_directories(folder);
        std::ofstream(folder / "manifest.json") << "{\"name\": \"" << name << "\", \"version\": \"" << version
                                                << "\", \"api_version\": 1}";
        std::ofstream(folder / "gadget.lua") << script;
    };
    put(shipped.path / "one", "Gadget One", "1", "return 1");
    put(shipped.path / "two", "Gadget Two", "1", "return 2");

    ShippedInstall r = installShippedGadgets(shipped.path, user.path / "gadgets", 1u << 20);
    CHECK(r.errors.empty());
    CHECK(r.installed.size() == 2);
    CHECK(discoverGadgets(user.path / "gadgets").gadgets.size() == 2);

    // second start: nothing to do
    r = installShippedGadgets(shipped.path, user.path / "gadgets", 1u << 20);
    CHECK(r.installed.empty());
    CHECK(r.errors.empty());

    // the user's edit of an installed gadget at the same version survives
    const fs::path installedOne = user.path / "gadgets" / gadgetFolderName("Gadget One") / "gadget.lua";
    { std::ofstream(installedOne) << "return 'edited'"; }
    r = installShippedGadgets(shipped.path, user.path / "gadgets", 1u << 20);
    CHECK(r.installed.empty());
    CHECK(readFile(installedOne) == "return 'edited'");

    // a new release (other version) replaces it
    put(shipped.path / "one", "Gadget One", "2", "return 'new'");
    r = installShippedGadgets(shipped.path, user.path / "gadgets", 1u << 20);
    REQUIRE(r.installed.size() == 1);
    CHECK(r.installed[0] == "Gadget One");
    CHECK(readFile(installedOne) == "return 'new'");

    // a different gadget sitting in the folder a shipped gadget would use is not overwritten
    put(shipped.path / "three", "Gadget Three", "1", "return 3");
    const fs::path squat = user.path / "gadgets" / gadgetFolderName("Gadget Three");
    put(squat, "Somebody Else", "1", "return 'mine'");
    r = installShippedGadgets(shipped.path, user.path / "gadgets", 1u << 20);
    CHECK(r.installed.empty());
    REQUIRE(r.errors.size() == 1);
    CHECK(readFile(squat / "gadget.lua") == "return 'mine'");

    // a shipped folder that does not exist installs nothing; a broken shipped gadget is reported
    CHECK(installShippedGadgets(shipped.path / "nope", user.path / "gadgets", 1u << 20).installed.empty());
    { std::ofstream(shipped.path / "two" / "manifest.json") << "not json"; }
    r = installShippedGadgets(shipped.path, user.path / "gadgets", 1u << 20);
    CHECK_FALSE(r.errors.empty());
}

TEST_CASE("stock gadgets: the shipped set installs into an empty gadgets folder", "[gadget][t0332]") {
    TempDir user("ship-stock");
    const ShippedInstall r = installShippedGadgets(CARVETTO_STOCK_GADGETS_DIR, user.path / "gadgets", 1u << 20);
    CHECK(r.errors.empty());
    CHECK(r.installed.size() == 5);
    const GadgetDiscovery d = discoverGadgets(user.path / "gadgets");
    CHECK(d.gadgets.size() == 5);
    CHECK(d.skipped.empty());
}
