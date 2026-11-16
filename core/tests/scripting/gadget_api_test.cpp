// T-0331: the gadget API binding (ADR-0102, ADR-0108): reads, edits through commands, ONE undo step
// per run, rollback on any failure, forms, toolpaths through a service, hostile gadgets.
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <thread>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/scripting/gadget_api.hpp"

using namespace carvetto;
using namespace carvetto::scripting;
using document::Document;
using document::ObjectId;
using Catch::Matchers::WithinAbs;

namespace {

bool contains(const std::string& s, const char* part) {
    return s.find(part) != std::string::npos;
}

GadgetRunOptions quick() {
    GadgetRunOptions o;
    o.limits.maxInstructions = 2'000'000;
    o.limits.deadlineMillis = 0;
    return o;
}

/// What a rolled-back run must restore.
struct State {
    std::size_t objects{0};
    std::size_t layers{0};
    std::size_t toolpaths{0};
    std::size_t steps{0};
    std::size_t index{0};
    std::size_t variables{0};
    bool canRedo{false};
    std::vector<ObjectId> selection;
    bool operator==(const State&) const = default;
};

State stateOf(const Document& d) {
    return State{d.objectCount(),        d.layers().size(), d.toolpaths().size(), d.undoStack().count(),
                 d.undoStack().index(),  d.variables().size(), d.undoStack().canRedo(), d.selection().ids()};
}

GadgetServices noServices() { return GadgetServices{}; }

/// A service that applies a trivial toolpath record through the undo stack, as the app's does.
GadgetServices fakeToolpathService(Document& doc, std::vector<ToolpathRequest>* seen = nullptr) {
    GadgetServices sv;
    sv.createToolpath = [&doc, seen](const ToolpathRequest& req) {
        if (seen != nullptr) {
            seen->push_back(req);
        }
        ToolpathCreated out;
        if (req.params.find("fail") != nullptr) {
            out.message = "the tool is too big";
            return out;
        }
        document::ToolpathRecord rec;
        rec.name = "T " + req.kind;
        rec.moves = document::shareMoves({toolpath::Move::rapid({0, 0, 5})});
        rec.selection.objects = req.objects;
        auto cmd = std::make_unique<document::AddToolpathCommand>(rec);
        const document::AddToolpathCommand* raw = cmd.get();
        if (!doc.execute(std::move(cmd))) {
            out.message = "refused";
            return out;
        }
        out.ok = true;
        out.ids.push_back(raw->id());
        return out;
    };
    sv.toolSnapshot = [](const std::string& id) -> std::optional<toolpath::ToolSnapshot> {
        if (id != "em3") {
            return std::nullopt;
        }
        toolpath::ToolSnapshot s;
        s.name = "3 mm end mill";
        s.toolNumber = 1;
        s.diameter = 3.0;
        s.feedRate = 800;
        s.plungeRate = 300;
        s.spindleSpeed = 18000;
        return s;
    };
    sv.tools = [] {
        return std::vector<GadgetTool>{{"em3", "3 mm end mill", 3.0, "end_mill"}, {"vb60", "60 deg V-bit", 6.0, "v_bit"}};
    };
    return sv;
}

}  // namespace

TEST_CASE("gadget: read functions", "[gadget][t0331]") {
    Document doc;
    const std::string src = R"(
        local c = carvetto
        local layers = c.doc.layers()
        local id = c.object.create_shape(layers[1], "rect", {cx = 10, cy = 20, width = 30, height = 40})
        c.doc.set_variable("Width", 12.5)
        local x0, y0, x1, y1 = c.object.bounds(id)
        local cs = c.object.contours(id)
        local w, h, t = c.doc.job_size()
        return c.api_version, c.doc.units(), #layers, c.layer.name(layers[1]), c.layer.visible(layers[1]),
               c.layer.locked(layers[1]), c.layer.colour(layers[1]), c.object.kind(id), x0, y0, x1, y1,
               #cs, #cs[1].points, cs[1].closed, c.doc.variable("Width"), c.doc.variable("Nope"), w, h, t,
               c.object.layer(id) == layers[1], c.doc.variables().Width, c.doc.active_layer() == layers[1]
    )";
    const GadgetRunResult r = runGadget(doc, noServices(), src, "g.lua", quick());
    INFO(r.script.message);
    REQUIRE(r.ok());
    const ScriptValues& v = r.script.returned;
    REQUIRE(v.size() == 23);
    CHECK(v[0].asInt() == kGadgetApiVersion);
    CHECK(v[1].asString() == "mm");
    CHECK(v[2].asInt() == 1);
    CHECK(v[3].asString() == "Layer 1");
    CHECK(v[4].asBool());
    CHECK_FALSE(v[5].asBool());
    CHECK(v[6].asString() == "#000000");
    CHECK(v[7].asString() == "contour");
    CHECK_THAT(v[8].asNumber(), WithinAbs(-5.0, 1e-9));
    CHECK_THAT(v[9].asNumber(), WithinAbs(0.0, 1e-9));
    CHECK_THAT(v[10].asNumber(), WithinAbs(25.0, 1e-9));
    CHECK_THAT(v[11].asNumber(), WithinAbs(40.0, 1e-9));
    CHECK(v[12].asInt() == 1);
    CHECK(v[13].asInt() == 4);
    CHECK(v[14].asBool());
    CHECK(v[15].asNumber() == 12.5);
    CHECK(v[16].isNil());
    CHECK(v[17].asNumber() == doc.job().sizeMm.x);
    CHECK(v[18].asNumber() == doc.job().sizeMm.y);
    CHECK(v[19].asNumber() == doc.job().sizeMm.z);
    CHECK(v[20].asBool());
    CHECK(v[21].asNumber() == 12.5);
    CHECK(v[22].asBool());
}

TEST_CASE("gadget: contours with bulges round-trip", "[gadget][t0331]") {
    Document doc;
    const std::string src = R"(
        local c = carvetto
        -- a half circle (bulge 1) closed by a straight line: a D shape
        local id = c.object.create_contour(nil, {{0, 0}, {10, 0}}, true, {1, 0})
        local cs = c.object.contours(id)
        local circle = c.object.create_shape(nil, "circle", {radius = 5})
        local ring = c.object.contours(circle)[1]
        return #cs[1].points, cs[1].bulges[1], cs[1].bulges[2], cs[1].closed, #ring.points, ring.bulges[1], ring.bulges[2]
    )";
    const GadgetRunResult r = runGadget(doc, noServices(), src, "g.lua", quick());
    INFO(r.script.message);
    REQUIRE(r.ok());
    const ScriptValues& v = r.script.returned;
    CHECK(v[0].asInt() == 2);
    CHECK_THAT(v[1].asNumber(), WithinAbs(1.0, 1e-9));
    CHECK(v[2].asNumber() == 0.0);
    CHECK(v[3].asBool());
    CHECK(v[4].asInt() == 2);  // a full circle is two half-circle pieces
    CHECK_THAT(v[5].asNumber(), WithinAbs(1.0, 1e-9));
    CHECK_THAT(v[6].asNumber(), WithinAbs(1.0, 1e-9));
    // the D shape: its arc bulges to the left of 0,0 -> 10,0 (counter-clockwise, below the line)
    const document::Object* d = doc.findObject(r.createdObjects.front());
    REQUIRE(d != nullptr);
    const auto b = doc.objectBounds(d->id);
    REQUIRE(b.has_value());
    CHECK_THAT(b->min.y, WithinAbs(-5.0, 1e-6));
    CHECK_THAT(b->max.y, WithinAbs(0.0, 1e-6));
}

TEST_CASE("gadget: bulge arcs have the right centre, side and size", "[gadget][t0331]") {
    Document doc;
    const std::string src = R"(
        local c = carvetto
        -- bulge 0.5 on (0,0)->(10,0): counter-clockwise, apex 2.5 below the chord (sagitta = bulge * chord / 2)
        local a = c.object.create_contour(nil, {{0, 0}, {10, 0}}, false, {0.5, 0})
        -- the same with a negative bulge: clockwise, apex 2.5 above
        local b = c.object.create_contour(nil, {{0, 0}, {10, 0}}, false, {-0.5, 0})
        -- bulge -2 is more than a half circle (253.7 degrees): read back as two pieces
        local d = c.object.create_contour(nil, {{0, 0}, {10, 0}}, false, {-2, 0})
        local cd = c.object.contours(d)[1]
        local ca = c.object.contours(a)[1]
        local ay0, ay1 = select(2, c.object.bounds(a)), select(4, c.object.bounds(a))
        local by0, by1 = select(2, c.object.bounds(b)), select(4, c.object.bounds(b))
        local dx0, dy0, dx1, dy1 = c.object.bounds(d)
        return ay0, ay1, by0, by1, #cd.points, cd.bulges[1], cd.bulges[2], cd.bulges[3], #ca.points, ca.bulges[1], dy0, dy1
    )";
    const GadgetRunResult r = runGadget(doc, noServices(), src, "g.lua", quick());
    INFO(r.script.message);
    REQUIRE(r.ok());
    const ScriptValues& v = r.script.returned;
    CHECK_THAT(v[0].asNumber(), WithinAbs(-2.5, 1e-9));
    CHECK_THAT(v[1].asNumber(), WithinAbs(0.0, 1e-9));
    CHECK_THAT(v[2].asNumber(), WithinAbs(0.0, 1e-9));
    CHECK_THAT(v[3].asNumber(), WithinAbs(2.5, 1e-9));
    CHECK(v[4].asInt() == 3);  // start, the piece boundary, the end
    CHECK_THAT(v[5].asNumber(), WithinAbs(-0.6180339887, 1e-6));
    CHECK_THAT(v[6].asNumber(), WithinAbs(-0.6180339887, 1e-6));
    CHECK(v[7].asNumber() == 0.0);
    CHECK(v[8].asInt() == 2);
    CHECK_THAT(v[9].asNumber(), WithinAbs(0.5, 1e-9));
    // the big clockwise arc bulges above the chord by its sagitta, bulge * chord / 2 = 10
    CHECK_THAT(v[11].asNumber(), WithinAbs(10.0, 1e-6));
}

TEST_CASE("gadget: curves are flattened when read", "[gadget][t0331]") {
    Document doc;
    const GadgetRunResult r = runGadget(doc, noServices(), R"(
        local id = carvetto.object.create_shape(nil, "ellipse", {width = 40, height = 20})
        local pts = carvetto.object.contours(id)[1].points
        local x0, y0, x1, y1 = 1e9, 1e9, -1e9, -1e9
        for _, p in ipairs(pts) do
            x0 = math.min(x0, p[1]) x1 = math.max(x1, p[1]) y0 = math.min(y0, p[2]) y1 = math.max(y1, p[2])
        end
        return #pts >= 8, x0, x1, y0, y1
    )", "g", quick());
    REQUIRE(r.ok());
    const ScriptValues& v = r.script.returned;
    CHECK(v[0].asBool());
    CHECK_THAT(v[1].asNumber(), WithinAbs(-20.0, 0.01));
    CHECK_THAT(v[2].asNumber(), WithinAbs(20.0, 0.01));
    CHECK_THAT(v[3].asNumber(), WithinAbs(-10.0, 0.01));
    CHECK_THAT(v[4].asNumber(), WithinAbs(10.0, 0.01));
}

TEST_CASE("gadget: per-run caps are exact", "[gadget][t0331]") {
    Document doc;
    GadgetRunOptions o = quick();
    o.gadgetLimits.maxCreatedObjects = 3;
    o.gadgetLimits.maxToolpaths = 2;
    const GadgetServices sv = fakeToolpathService(doc);
    SECTION("objects: the limit itself is allowed, one more is not") {
        const std::string three = "for i = 1, 3 do carvetto.object.create_shape(nil, 'circle', {radius = 1}) end";
        CHECK(runGadget(doc, sv, three, "g", o).ok());
        CHECK(doc.objectCount() == 3);
        const GadgetRunResult r = runGadget(doc, sv, three + " carvetto.layer.create('one more')", "g", o);
        CHECK_FALSE(r.ok());  // layers count as created objects
        CHECK(doc.objectCount() == 3);
    }
    SECTION("toolpaths: the limit itself is allowed, one more is not") {
        const std::string two =
            "local i = carvetto.object.create_shape(nil, 'circle', {radius = 1}) "
            "carvetto.toolpath.create('drill', {i}, {}) carvetto.toolpath.create('drill', {i}, {}) ";
        CHECK(runGadget(doc, sv, two, "g", o).ok());
        CHECK(doc.toolpaths().size() == 2);
        GadgetRunResult r = runGadget(doc, sv, two + "carvetto.toolpath.create('drill', {i}, {})", "g", o);
        CHECK_FALSE(r.ok());
        CHECK(contains(r.script.message, "too many toolpaths"));
        CHECK(doc.toolpaths().size() == 2);
        const std::string custom =
            "local t = {{kind = 'rapid', x = 0, y = 0, z = 0}} "
            "carvetto.toolpath.create_custom('a', 'em3', t) carvetto.toolpath.create_custom('b', 'em3', t) ";
        CHECK(runGadget(doc, sv, custom, "g", o).ok());
        CHECK(doc.toolpaths().size() == 4);
        r = runGadget(doc, sv, custom + "carvetto.toolpath.create_custom('c', 'em3', t)", "g", o);
        CHECK(contains(r.script.message, "too many toolpaths"));
        CHECK(doc.toolpaths().size() == 4);
        // the two kinds share one count
        r = runGadget(doc, sv, "local t = {{kind = 'rapid', x = 0, y = 0, z = 0}} carvetto.toolpath.create_custom('a', 'em3', t) "
                               "local i = carvetto.object.create_shape(nil, 'circle', {radius = 1}) "
                               "carvetto.toolpath.create('drill', {i}, {}) carvetto.toolpath.create('drill', {i}, {})", "g", o);
        CHECK(contains(r.script.message, "too many toolpaths"));
    }
    SECTION("custom moves: the limit itself is allowed, one more is not") {
        o.gadgetLimits.maxCustomMoves = 3;
        const auto withMoves = [](int n) {
            return "local t = {} for i = 1, " + std::to_string(n) +
                   " do t[i] = {kind = 'rapid', x = i, y = 0, z = 0} end "
                   "carvetto.toolpath.create_custom('m', 'em3', t)";
        };
        CHECK(runGadget(doc, sv, withMoves(3), "g", o).ok());
        const GadgetRunResult r = runGadget(doc, sv, withMoves(4), "g", o);
        CHECK_FALSE(r.ok());
        CHECK(doc.toolpaths().size() == 1);
    }
    SECTION("dialogs: the limit itself is allowed, one more is not") {
        o.gadgetLimits.maxDialogs = 3;
        GadgetServices ui;
        ui.ui.message = [](const std::string&) {};
        ui.ui.confirm = [](const std::string&) { return true; };
        ui.ui.form = [](const FormSpec&) { return std::optional<ScriptValue::Map>(ScriptValue::Map{}); };
        const std::string three =
            "carvetto.ui.message('a') carvetto.ui.confirm('b') carvetto.ui.form({title = 't', fields = {}}) ";
        CHECK(runGadget(doc, ui, three, "g", o).ok());  // the cap is per run, not per process
        GadgetRunResult r = runGadget(doc, ui, "carvetto.layer.create('x') " + three + "carvetto.ui.message('d')", "g", o);
        CHECK_FALSE(r.ok());
        CHECK(contains(r.script.message, "too many dialogs"));
        CHECK(doc.layers().size() == 1);  // rolled back
        // a loop of boxes is stopped by the cap, not only by the instruction budget
        o.gadgetLimits.maxDialogs = 100;
        int shown = 0;
        ui.ui.message = [&shown](const std::string&) { ++shown; };
        r = runGadget(doc, ui, "while true do carvetto.ui.message('x') end", "g", o);
        CHECK(contains(r.script.message, "too many dialogs"));
        CHECK(shown == 100);
        // a script cannot pcall its way out of the cap: the run still fails and rolls back
        shown = 0;
        r = runGadget(doc, ui, "carvetto.layer.create('x') for i = 1, 150 do pcall(carvetto.ui.message, 'x') end", "g", o);
        CHECK_FALSE(r.ok());
        CHECK_FALSE(r.committed);
        CHECK(contains(r.script.message, "too many dialogs"));
        CHECK(shown == 100);
        CHECK(doc.layers().size() == 1);
    }
    SECTION("layers and variable edits: the limit itself is allowed, one more is not") {
        o.gadgetLimits.maxCreatedLayers = 2;
        const std::string two = "carvetto.layer.create('a') carvetto.layer.create('b') ";
        CHECK(runGadget(doc, sv, two, "g", o).ok());
        CHECK(doc.layers().size() == 3);
        GadgetRunResult r = runGadget(doc, sv, two + "carvetto.layer.create('c')", "g", o);
        CHECK(contains(r.script.message, "too many layers"));
        CHECK(doc.layers().size() == 3);

        o.gadgetLimits.maxVariableEdits = 2;
        const std::string vars = "carvetto.doc.set_variable('A', 1) carvetto.doc.set_variable('B', 2) ";
        CHECK(runGadget(doc, sv, vars, "g", o).ok());
        // a set that changes nothing is not an edit
        CHECK(runGadget(doc, sv, vars + "carvetto.doc.set_variable('A', 1) carvetto.doc.set_variable('B', 2)", "g", o).ok());
        r = runGadget(doc, sv, "carvetto.doc.set_variable('A', 5) carvetto.doc.set_variable('B', 6) carvetto.doc.set_variable('C', 7)", "g", o);
        CHECK(contains(r.script.message, "too many variables"));
        CHECK(doc.variables().get("A") == 1.0);
        CHECK_FALSE(doc.variables().contains("C"));
    }
    SECTION("waiting: the combined paused time is capped, exactly") {
        std::uint64_t t = 0;
        o.limits.clockNanos = [&t] { return t; };
        o.limits.deadlineMillis = 0;
        o.gadgetLimits.maxPausedNanos = 3000;
        GadgetServices ui;
        ui.ui.message = [&t](const std::string&) { t += 1000; };
        const std::string two = "carvetto.ui.message('a') carvetto.ui.message('b') ";
        CHECK(runGadget(doc, ui, two + "carvetto.ui.message('c')", "g", o).ok());  // 3000 = the cap: allowed
        GadgetRunResult r = runGadget(doc, ui, two + "carvetto.layer.create('x') carvetto.ui.message('c') carvetto.ui.message('d')", "g", o);
        CHECK_FALSE(r.ok());
        CHECK(contains(r.script.message, "too long waiting"));
        CHECK(doc.layers().size() == 1);
        // a script cannot pcall its way out of the budget
        r = runGadget(doc, ui, "pcall(carvetto.ui.message, 'a') pcall(carvetto.ui.message, 'b') pcall(carvetto.ui.message, 'c') "
                               "pcall(carvetto.ui.message, 'd') carvetto.layer.create('x')", "g", o);
        CHECK_FALSE(r.ok());
        CHECK(doc.layers().size() == 1);
    }
    SECTION("toolpath calculation has its own, smaller budget, exactly") {
        std::uint64_t t = 0;
        o.limits.clockNanos = [&t] { return t; };
        o.limits.deadlineMillis = 0;
        o.gadgetLimits.maxPausedNanos = 1'000'000;  // dialogs: generous here
        o.gadgetLimits.maxCalculationNanos = 3000;
        o.gadgetLimits.maxToolpaths = 1000;  // the toolpath COUNT cap has its own test
        GadgetServices tp = fakeToolpathService(doc);
        const auto inner = tp.createToolpath;
        std::uint64_t cost = 1500;
        tp.createToolpath = [&t, inner, &cost](const ToolpathRequest& q) {
            t += cost;
            return inner(q);
        };
        tp.ui.message = [&t](const std::string&) { t += 5000; };  // dialog time never eats the calculation budget
        const std::string prefix = "local i = carvetto.object.create_shape(nil, 'circle', {radius = 1}) carvetto.ui.message('m') ";
        const std::string call = "carvetto.toolpath.create('drill', {i}, {}) ";
        const std::size_t before = doc.toolpaths().size();
        GadgetRunResult r = runGadget(doc, tp, prefix + call + call, "g", o);  // 3000 = the cap: allowed
        INFO(r.script.message);
        CHECK(r.ok());
        CHECK(doc.toolpaths().size() == before + 2);
        r = runGadget(doc, tp, prefix + call + call + call, "g", o);  // 4500: one over
        CHECK_FALSE(r.ok());
        INFO(r.script.message);
        CHECK(contains(r.script.message, "too long waiting for toolpath calculations"));
        CHECK(doc.toolpaths().size() == before + 2);  // rolled back
        // one call that alone costs 3001 is over, and pcall cannot hide it
        cost = 3001;
        r = runGadget(doc, tp, prefix + "pcall(carvetto.toolpath.create, 'drill', {i}, {})", "g", o);
        CHECK_FALSE(r.ok());
        CHECK(contains(r.script.message, "toolpath calculations"));
        // ~60 slow calculations cannot add up to half an hour any more
        cost = 29'000'000'000ull;
        o.gadgetLimits.maxCalculationNanos = GadgetLimits{}.maxCalculationNanos;
        std::string many = prefix;
        for (int k = 0; k < 63; ++k) {
            many += "pcall(carvetto.toolpath.create, 'drill', {i}, {}) ";
        }
        r = runGadget(doc, tp, many, "g", o);
        CHECK_FALSE(r.ok());
        CHECK(doc.toolpaths().size() == before + 2);
        CHECK(GadgetLimits{}.maxCalculationNanos <= 5ull * 60ull * 1'000'000'000ull);
    }
    SECTION("a running calculation is cut short through the cancel token") {
        o.gadgetLimits.maxCalculationNanos = 100'000'000;  // 100 ms of real time
        GadgetServices tp = fakeToolpathService(doc);
        bool sawToken = false;
        tp.createToolpath = [&sawToken](const ToolpathRequest& q) {
            sawToken = q.cancel != nullptr;
            const auto start = std::chrono::steady_clock::now();
            while (q.cancel != nullptr && !q.cancel->load() &&
                   std::chrono::steady_clock::now() - start < std::chrono::seconds(20)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            ToolpathCreated c;
            c.message = "the calculation was cancelled";
            return c;
        };
        const auto begin = std::chrono::steady_clock::now();
        const GadgetRunResult r = runGadget(doc, tp,
            "local i = carvetto.object.create_shape(nil, 'circle', {radius = 1}) pcall(carvetto.toolpath.create, 'drill', {i}, {})",
            "g", o);
        CHECK(std::chrono::steady_clock::now() - begin < std::chrono::seconds(15));
        CHECK(sawToken);
        CHECK_FALSE(r.ok());
        CHECK(contains(r.script.message, "toolpath calculations"));
        CHECK(doc.toolpaths().empty());
        // the user's Abort cuts a calculation short too
        std::atomic<bool> abort{false};
        o.gadgetLimits.maxCalculationNanos = GadgetLimits{}.maxCalculationNanos;
        o.cancel = &abort;
        const auto spin = tp.createToolpath;
        tp.createToolpath = [&abort, spin](const ToolpathRequest& q) {
            abort = true;
            return spin(q);
        };
        const auto abortBegin = std::chrono::steady_clock::now();
        const GadgetRunResult r2 = runGadget(doc, tp,
            "local i = carvetto.object.create_shape(nil, 'circle', {radius = 1}) carvetto.toolpath.create('drill', {i}, {})",
            "g", o);
        // The polling calculation would run its natural 20 s; the watchdog ends it on Abort.
        CHECK(std::chrono::steady_clock::now() - abortBegin < std::chrono::seconds(10));
        CHECK(r2.script.status == ScriptStatus::Cancelled);
        CHECK(doc.toolpaths().empty());
    }
    SECTION("the watchdog starts with what is LEFT of the calculation budget") {
        // Real clock, generous margins: budget 1000 ms; the first calculation really uses ~600 ms,
        // so the second one must be cut after ~400 ms (not after a fresh 1000 ms).
        o.limits.clockNanos = nullptr;
        o.gadgetLimits.maxCalculationNanos = 1'000'000'000;
        GadgetServices tp = fakeToolpathService(doc);
        int calls = 0;
        std::chrono::steady_clock::duration secondRan{};
        tp.createToolpath = [&](const ToolpathRequest& q) {
            const auto start = std::chrono::steady_clock::now();
            ++calls;
            if (calls == 1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(600));
            } else {
                while (q.cancel != nullptr && !q.cancel->load() &&
                       std::chrono::steady_clock::now() - start < std::chrono::seconds(20)) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
                secondRan = std::chrono::steady_clock::now() - start;
            }
            ToolpathCreated c;
            c.message = "not done";
            return c;
        };
        const GadgetRunResult r = runGadget(doc, tp,
            "local i = carvetto.object.create_shape(nil, 'circle', {radius = 1}) "
            "pcall(carvetto.toolpath.create, 'drill', {i}, {}) pcall(carvetto.toolpath.create, 'drill', {i}, {})",
            "g", o);
        CHECK(calls == 2);
        CHECK_FALSE(r.ok());
        CHECK(contains(r.script.message, "toolpath calculations"));
        CHECK(secondRan < std::chrono::milliseconds(800));  // a watchdog started with the full limit would take ~1000 ms
    }
    SECTION("the log keeps whole lines up to its byte cap") {
        o.gadgetLimits.maxLogBytes = 10;
        const GadgetRunResult r = runGadget(doc, sv, "carvetto.log('aaaa') carvetto.log('bbbb') carvetto.log('cccc')", "g", o);
        REQUIRE(r.ok());
        CHECK(r.log.size() == 2);  // 5 + 5 bytes reach the cap; the third line is dropped
    }
}

TEST_CASE("gadget: shapes keep their creation parameters", "[gadget][t0331]") {
    Document doc;
    const std::string src = R"(
        local c = carvetto
        c.object.create_shape(nil, "rect", {width = 20, height = 10, corner = "rounded", corner_size = 2})
        c.object.create_shape(nil, "circle", {cx = 5, radius = 3})
        c.object.create_shape(nil, "ellipse", {width = 8, height = 4, angle = 0.5})
        c.object.create_shape(nil, "polygon", {sides = 6, radius = 4})
        c.object.create_shape(nil, "star", {points = 5, outer = 6, inner = 3})
    )";
    const GadgetRunResult r = runGadget(doc, noServices(), src, "g.lua", quick());
    INFO(r.script.message);
    REQUIRE(r.ok());
    REQUIRE(r.createdObjects.size() == 5);
    const auto* rect = std::get_if<document::RectangleParams>(&doc.findObject(r.createdObjects[0])->asVector()->params);
    REQUIRE(rect != nullptr);
    CHECK(rect->corners == document::RectangleParams::CornerStyle::Rounded);
    CHECK(rect->cornerRadiusMm == 2.0);
    CHECK(std::holds_alternative<document::CircleParams>(doc.findObject(r.createdObjects[1])->asVector()->params));
    CHECK(std::holds_alternative<document::StarParams>(doc.findObject(r.createdObjects[4])->asVector()->params));
}

TEST_CASE("gadget: modify objects", "[gadget][t0331]") {
    Document doc;
    const std::string src = R"(
        local c = carvetto
        local a = c.object.create_shape(nil, "rect", {width = 10, height = 10})   -- -5..5
        local b = c.object.create_shape(nil, "rect", {width = 2, height = 2, cx = 50})
        c.object.move(a, 100, 0)
        local ax0, ay0, ax1, ay1 = c.object.bounds(a)
        c.object.scale(b, 2)               -- about its own centre
        local bx0, by0, bx1, by1 = c.object.bounds(b)
        c.object.rotate(a, math.pi / 2)    -- about its own centre: a square stays put
        local rx0, ry0, rx1, ry1 = c.object.bounds(a)
        local layer = c.layer.create("Second", "#ff8800")
        c.object.set_layer({a, b}, layer)
        local onLayer = c.object.layer(b) == layer
        c.object.delete(b)
        local gone, err = pcall(c.object.kind, b)
        return ax0, ax1, bx0, bx1, rx0, rx1, onLayer, gone, err, c.layer.colour(layer), by0, by1
    )";
    const GadgetRunResult r = runGadget(doc, noServices(), src, "g.lua", quick());
    INFO(r.script.message);
    REQUIRE(r.ok());
    const ScriptValues& v = r.script.returned;
    CHECK_THAT(v[0].asNumber(), WithinAbs(95.0, 1e-9));
    CHECK_THAT(v[1].asNumber(), WithinAbs(105.0, 1e-9));
    CHECK_THAT(v[2].asNumber(), WithinAbs(49.0 - 1.0, 1e-9));  // 2 mm square scaled x2 about 50: 48..52
    CHECK_THAT(v[3].asNumber(), WithinAbs(52.0, 1e-9));
    CHECK_THAT(v[4].asNumber(), WithinAbs(95.0, 1e-6));
    CHECK_THAT(v[5].asNumber(), WithinAbs(105.0, 1e-6));
    CHECK(v[6].asBool());
    CHECK_FALSE(v[7].asBool());  // the deleted id is a catchable error
    CHECK(contains(v[8].asString(), "no such object"));
    CHECK(v[9].asString() == "#ff8800");
    CHECK_THAT(v[10].asNumber(), WithinAbs(-2.0, 1e-9));  // a single factor scales both axes
    CHECK_THAT(v[11].asNumber(), WithinAbs(2.0, 1e-9));
}

TEST_CASE("gadget: a run is ONE undo step and redo restores it", "[gadget][t0331]") {
    Document doc;
    const State before = stateOf(doc);
    const std::string src = R"(
        local c = carvetto
        local layer = c.layer.create("Weave")
        for i = 1, 5 do c.object.create_shape(layer, "circle", {cx = i * 10, radius = 2}) end
        c.doc.set_variable("Count", 5)
        c.selection.set({c.object.create_contour(layer, {{0, 0}, {1, 1}}, false)})
    )";
    GadgetRunOptions o = quick();
    o.undoText = "Weave creator";
    const GadgetRunResult r = runGadget(doc, noServices(), src, "g.lua", o);
    INFO(r.script.message);
    REQUIRE(r.ok());
    CHECK(r.committed);
    CHECK(r.edits == 8);
    CHECK(doc.undoStack().count() == before.steps + 1);
    CHECK(doc.undoStack().undoText() == "Weave creator");
    CHECK(doc.objectCount() == 6);
    CHECK(doc.layers().size() == 2);
    CHECK(doc.selection().size() == 1);
    REQUIRE(doc.undo());
    CHECK(doc.objectCount() == 0);
    CHECK(doc.layers().size() == 1);
    CHECK_FALSE(doc.variables().contains("Count"));
    REQUIRE(doc.redo());
    CHECK(doc.objectCount() == 6);
    CHECK(doc.layers().size() == 2);
    CHECK(doc.variables().get("Count") == 5.0);
    CHECK(doc.checkIntegrity().empty());
    REQUIRE(doc.undo());
    CHECK(doc.undoStack().count() == before.steps + 1);  // still one step, now undone
    CHECK(doc.undoStack().canRedo());
}

TEST_CASE("gadget: a read-only run records no undo step", "[gadget][t0331]") {
    Document doc;
    const GadgetRunResult r = runGadget(doc, noServices(), "return carvetto.doc.units()", "g.lua", quick());
    REQUIRE(r.ok());
    CHECK_FALSE(r.committed);
    CHECK(doc.undoStack().count() == 0);
}

TEST_CASE("gadget: a failing run changes nothing", "[gadget][t0331]") {
    Document doc;
    REQUIRE(doc.execute(std::make_unique<document::SetDocumentVariableCommand>("Keep", 1.0)));
    doc.undo();  // a redo tail the failed run must not destroy
    const ObjectId pre = [&] {
        auto cmd = std::make_unique<document::AddObjectCommand>(document::Object::vector(
            geometry::Contour::circle({0, 0}, 1), doc.defaultLayer(), doc.activeSheet()));
        const auto* raw = cmd.get();
        // not through the undo stack: the redo tail stays
        cmd->apply(doc);
        return raw->id();
    }();
    doc.selection().assign({pre});
    const State before = stateOf(doc);
    REQUIRE(before.canRedo);

    const char* scripts[] = {
        "carvetto.object.create_shape(nil, 'circle', {radius = 2}) carvetto.doc.set_variable('X', 1) error('boom')",
        "carvetto.layer.create('L') carvetto.selection.set({}) carvetto.object.delete(carvetto.selection.objects()[1] or 0)",
        "carvetto.object.create_shape(nil, 'circle', {radius = 2}) carvetto.object.create_shape(nil, 'circle', {radius = -1})",
        "carvetto.object.create_shape(nil, 'circle', {radius = 2}) while true do end",
        "carvetto.object.create_shape(nil, 'circle', {radius = 2}) local t = {} for i = 1, 1e9 do t[i] = ('x'):rep(1000) .. i end",
        "carvetto.object.create_shape(nil, 'circle', {radius = 2}) x = = 1",
    };
    for (const char* src : scripts) {
        INFO(src);
        const GadgetRunResult r = runGadget(doc, noServices(), src, "g.lua", quick());
        CHECK_FALSE(r.ok());
        CHECK_FALSE(r.committed);
        CHECK(r.createdObjects.empty());
        CHECK(stateOf(doc) == before);
        CHECK(doc.checkIntegrity().empty());
    }
    // the selection a failed run changed is back, and the history still works
    CHECK(doc.selection().ids() == std::vector<ObjectId>{pre});
    REQUIRE(doc.redo());
}

TEST_CASE("gadget: errors name the function and the line, pcall works", "[gadget][t0331]") {
    Document doc;
    const GadgetRunResult r = runGadget(doc, noServices(), "\n\ncarvetto.object.move(99999, 1, 1)", "gadget.lua", quick());
    REQUIRE_FALSE(r.ok());
    CHECK(r.script.status == ScriptStatus::RuntimeError);
    CHECK(r.script.line == 3);
    CHECK(contains(r.script.message, "carvetto.object.move: no such object (id 99999)"));

    const GadgetRunResult p = runGadget(doc, noServices(),
                                        "local ok, e = pcall(carvetto.object.move, 7, 1, 1) return ok, e", "g", quick());
    REQUIRE(p.ok());
    CHECK_FALSE(p.script.returned[0].asBool());
    CHECK(contains(p.script.returned[1].asString(), "no such object"));
}

TEST_CASE("gadget: bad arguments are errors, never crashes", "[gadget][t0331]") {
    Document doc;
    const char* bad[] = {
        "carvetto.object.create_contour(nil, {{0,0}}, false)",
        "carvetto.object.create_contour(nil, {{0,0},{1,1}}, 'yes')",
        "carvetto.object.create_contour(nil, {{0,0},{0/0,1}}, false)",
        "carvetto.object.create_contour(nil, {{0,0},{1,1}}, false, {1})",
        "carvetto.object.create_contour(nil, {{0,0},{0,0}}, false, {1, 0})",
        "carvetto.object.create_contour(nil, {{0,0},{0,0}}, false)",
        "carvetto.object.create_contour(nil, {{0,0,0},{1,1}}, false)",
        "carvetto.object.create_contour(12345, {{0,0},{1,1}}, false)",
        "carvetto.object.create_shape(nil, 'hexagon', {})",
        "carvetto.object.create_shape(nil, 'circle', {radus = 3})",
        "carvetto.object.create_shape(nil, 'circle', {radius = 'big'})",
        "carvetto.object.create_shape(nil, 'circle', {})",
        "carvetto.object.create_shape(nil, 'polygon', {sides = 2.5, radius = 1})",
        "carvetto.object.create_shape(nil, 'rect', {width = 1, height = 1, corner = 'round'})",
        "carvetto.object.move(nil, 1, 1)",
        "carvetto.object.move({}, 1, 1)",
        "carvetto.object.scale(1, 0)",
        "carvetto.object.bounds('a')",
        "carvetto.doc.set_variable('1bad', 3)",
        "carvetto.doc.set_variable('ok', 1/0)",
        "carvetto.layer.create('')",
        "carvetto.layer.create('x', 'red')",
        "carvetto.layer.set(carvetto.doc.layers()[1], {colour = '#12'})",
        "carvetto.layer.set(carvetto.doc.layers()[1], {nme = 'x'})",
        "carvetto.selection.set({1, 2, 3})",
        "carvetto.toolpath.create('profile', {1})",
        "carvetto.toolpath.create_custom('x', 'em3', {})",
        "carvetto.ui.form({})",
        "carvetto.ui.message(5)",
    };
    for (const char* src : bad) {
        INFO(src);
        const GadgetRunResult r = runGadget(doc, noServices(), src, "g.lua", quick());
        CHECK_FALSE(r.ok());
        CHECK(r.script.status == ScriptStatus::RuntimeError);
        CHECK(doc.objectCount() == 0);
    }
}

TEST_CASE("gadget: locked layers cannot be drawn on, hidden ones can", "[gadget][t0331]") {
    Document doc;
    const GadgetRunResult r = runGadget(doc, noServices(), R"(
        local l = carvetto.doc.layers()[1]
        carvetto.layer.set(l, {visible = false})
        carvetto.object.create_shape(l, "circle", {radius = 1})
        carvetto.layer.set(l, {locked = true, name = "Frozen"})
        local ok, e = pcall(carvetto.object.create_shape, l, "circle", {radius = 1})
        return ok, e, carvetto.layer.name(l), carvetto.layer.visible(l)
    )", "g", quick());
    REQUIRE(r.ok());
    CHECK_FALSE(r.script.returned[0].asBool());
    CHECK(contains(r.script.returned[1].asString(), "is locked"));
    CHECK(r.script.returned[2].asString() == "Frozen");
    CHECK_FALSE(r.script.returned[3].asBool());
    CHECK(doc.objectCount() == 1);
}

TEST_CASE("gadget: objects on a locked layer cannot be edited and a gadget cannot unlock", "[gadget][t0331]") {
    Document doc;
    const GadgetRunResult setup = runGadget(doc, noServices(), R"(
        local l = carvetto.doc.layers()[1]
        local other = carvetto.layer.create("Other")
        local a = carvetto.object.create_shape(l, "rect", {width = 10, height = 10})
        local b = carvetto.object.create_shape(other, "rect", {width = 4, height = 4})
        carvetto.layer.set(l, {locked = true})
        return a, b
    )", "g", quick());
    REQUIRE(setup.ok());
    const std::string a = std::to_string(setup.script.returned[0].asInt());  // on the locked layer
    const std::string b = std::to_string(setup.script.returned[1].asInt());
    const State before = stateOf(doc);
    const std::string attempts[] = {
        "carvetto.object.move(" + a + ", 5, 5)",
        "carvetto.object.rotate({" + a + "}, 1)",
        "carvetto.object.scale(" + a + ", 2)",
        "carvetto.object.delete(" + a + ")",
        "carvetto.object.set_layer(" + a + ", carvetto.doc.layers()[2])",
        "carvetto.object.set_layer(" + b + ", carvetto.doc.layers()[1])",  // INTO the locked layer
        "carvetto.layer.set(carvetto.doc.layers()[1], {locked = false})",
        "carvetto.object.move({" + b + ", " + a + "}, 1, 1)",  // one locked object refuses the whole call
    };
    for (const std::string& src : attempts) {
        INFO(src);
        const GadgetRunResult r = runGadget(doc, noServices(), src, "g", quick());
        CHECK_FALSE(r.ok());
        CHECK((contains(r.script.message, "locked layer") || contains(r.script.message, "is locked") ||
               contains(r.script.message, "cannot unlock")));
        CHECK(stateOf(doc) == before);
    }
    // the unlocked layer, and moving onto/off nothing locked, still work; locking is allowed; so are
    // name/colour/visibility of a locked layer
    std::string okSrc = R"(
        local layers = carvetto.doc.layers()
        carvetto.object.move(%B%, 1, 0)
        carvetto.layer.set(layers[2], {locked = true})
        carvetto.layer.set(layers[1], {name = "Renamed", visible = false, locked = true})
    )";
    okSrc.replace(okSrc.find("%B%"), 3, b);
    const GadgetRunResult ok = runGadget(doc, noServices(), okSrc, "g", quick());
    INFO(ok.script.message);
    CHECK(ok.ok());
}

TEST_CASE("gadget: selection", "[gadget][t0331]") {
    Document doc;
    const GadgetRunResult r = runGadget(doc, noServices(), R"(
        local a = carvetto.object.create_shape(nil, "circle", {radius = 1})
        local b = carvetto.object.create_shape(nil, "circle", {radius = 2})
        local before = #carvetto.selection.objects()
        carvetto.selection.set({b, a, b})
        local s = carvetto.selection.objects()
        carvetto.selection.set({})
        return before, #s, s[1] == b, s[2] == a, #carvetto.selection.objects()
    )", "g", quick());
    REQUIRE(r.ok());
    CHECK(r.script.returned[0].asInt() == 0);
    CHECK(r.script.returned[1].asInt() == 2);  // duplicates collapse
    CHECK(r.script.returned[2].asBool());
    CHECK(r.script.returned[3].asBool());
    CHECK(r.script.returned[4].asInt() == 0);
}

TEST_CASE("gadget: toolpaths through the service join the one undo step", "[gadget][t0331]") {
    Document doc;
    std::vector<ToolpathRequest> seen;
    const GadgetServices sv = fakeToolpathService(doc, &seen);
    const GadgetRunResult r = runGadget(doc, sv, R"(
        local id = carvetto.object.create_shape(nil, "rect", {width = 20, height = 10})
        local tool = carvetto.tool.find("3 mm end mill")
        local ids = carvetto.toolpath.create("profile", {id}, {tool = tool, cutDepth = 3, side = "outside"})
        local custom = carvetto.toolpath.create_custom("Knife path", tool, {
            {kind = "rapid", x = 0, y = 0, z = 5},
            {kind = "plunge", z = -1},
            {kind = "feed", x = 10, y = 0, feed = 500},
            {kind = "arc_ccw", x = 10, y = 10, cx = 10, cy = 5},
            {kind = "dwell", seconds = 0.5},
            {kind = "retract", z = 5},
        })
        return #ids, ids[1], custom, #carvetto.tool.list(), carvetto.tool.find("nothing"), carvetto.tool.find("vb60")
    )", "g", quick());
    INFO(r.script.message);
    REQUIRE(r.ok());
    CHECK(r.committed);
    CHECK(doc.undoStack().count() == 1);
    CHECK(doc.toolpaths().size() == 2);
    CHECK(r.createdToolpaths.size() == 2);
    REQUIRE(seen.size() == 1);
    CHECK(seen[0].kind == "profile");
    CHECK(seen[0].objects.size() == 1);
    CHECK(seen[0].params.find("cutDepth")->asNumber() == 3.0);
    CHECK(seen[0].params.find("tool")->asString() == "em3");
    const ScriptValues& v = r.script.returned;
    CHECK(v[0].asInt() == 1);
    CHECK(v[3].asInt() == 2);
    CHECK(v[4].isNil());
    CHECK(v[5].asString() == "vb60");

    const document::ToolpathRecord* custom = doc.findToolpath(r.createdToolpaths[1]);
    REQUIRE(custom != nullptr);
    CHECK(custom->name == "Knife path");
    CHECK(custom->tool.name == "3 mm end mill");
    CHECK(custom->toolSource.toolId == "em3");
    REQUIRE(custom->moves != nullptr);
    REQUIRE(custom->moves->size() == 6);
    const auto& m = *custom->moves;
    CHECK(m[1].kind == toolpath::MoveKind::Plunge);
    CHECK(m[1].to.x == 0.0);  // x and y default to the previous position
    CHECK(m[1].to.z == -1.0);
    CHECK(m[2].feedOverride.value_or(0) == 500.0);
    CHECK(m[3].kind == toolpath::MoveKind::ArcCCW);
    CHECK(m[3].centre.y == 5.0);
    CHECK(m[4].kind == toolpath::MoveKind::Dwell);
    CHECK(m[4].dwellSeconds == 0.5);
    CHECK(m[4].to.y == 10.0);

    REQUIRE(doc.undo());
    CHECK(doc.toolpaths().empty());
    CHECK(doc.objectCount() == 0);
    REQUIRE(doc.redo());
    CHECK(doc.toolpaths().size() == 2);
}

TEST_CASE("gadget: a failing toolpath rolls the whole run back", "[gadget][t0331]") {
    Document doc;
    const GadgetServices sv = fakeToolpathService(doc);
    const State before = stateOf(doc);
    GadgetRunResult r = runGadget(doc, sv, R"(
        local id = carvetto.object.create_shape(nil, "rect", {width = 20, height = 10})
        carvetto.toolpath.create("pocket", {id}, {})
        carvetto.toolpath.create("pocket", {id}, {fail = true})
    )", "g", quick());
    REQUIRE_FALSE(r.ok());
    CHECK(contains(r.script.message, "the tool is too big"));
    CHECK(stateOf(doc) == before);

    SECTION("unknown kinds, no objects, no service") {
        r = runGadget(doc, sv, "carvetto.toolpath.create('lathe', {1}, {})", "g", quick());
        CHECK(contains(r.script.message, "unknown toolpath kind"));
        r = runGadget(doc, sv, "carvetto.toolpath.create('pocket', nil, {})", "g", quick());
        CHECK(contains(r.script.message, "nothing is selected"));
        r = runGadget(doc, noServices(), "local i = carvetto.object.create_shape(nil,'circle',{radius=1}) carvetto.toolpath.create('pocket', {i}, {})",
                      "g", quick());
        CHECK(contains(r.script.message, "cannot be created here"));
        r = runGadget(doc, sv, "carvetto.toolpath.create_custom('x', 'zzz', {{kind='rapid',x=0,y=0,z=0}})", "g", quick());
        CHECK(contains(r.script.message, "unknown tool"));
        r = runGadget(doc, sv, "carvetto.toolpath.create_custom('x', 'em3', {{kind='spin'}})", "g", quick());
        CHECK(contains(r.script.message, "unknown move kind"));
        r = runGadget(doc, sv, "carvetto.toolpath.create_custom('x', 'em3', {{kind='dwell', seconds=1}})", "g", quick());
        CHECK(contains(r.script.message, "needs x, y and z"));
        r = runGadget(doc, sv, "carvetto.toolpath.create_custom('x', 'em3', {{kind='arc_cw', x=1, y=1, z=0}})", "g", quick());
        CHECK(contains(r.script.message, "missing parameter 'cx'"));
        r = runGadget(doc, sv, "carvetto.toolpath.create_custom('x', 'em3', {{kind='feed', x=1, y=1, z=0, feed=0}})", "g", quick());
        CHECK(contains(r.script.message, "feed must be positive"));
        r = runGadget(doc, sv, "carvetto.toolpath.create_custom('x', 'em3', {{kind='feed', x=1, y=1, z=0, feed=-5}})", "g", quick());
        CHECK(contains(r.script.message, "feed must be positive"));
        r = runGadget(doc, sv, "carvetto.toolpath.create_custom('x', 'em3', {{kind='dwell', x=1, y=1, z=0, seconds=-1}})", "g", quick());
        CHECK(contains(r.script.message, "seconds must be between"));
        r = runGadget(doc, sv, "carvetto.toolpath.create_custom('x', 'em3', {{kind='dwell', x=1, y=1, z=0, seconds=0}})", "g", quick());
        CHECK(r.ok());  // a zero dwell is allowed
        CHECK(doc.toolpaths().size() == 1);
        doc.undo();
        CHECK(doc.toolpaths().empty());
    }
}

TEST_CASE("gadget: with no selection toolpath.create uses the selection", "[gadget][t0331]") {
    Document doc;
    std::vector<ToolpathRequest> seen;
    const GadgetServices sv = fakeToolpathService(doc, &seen);
    const GadgetRunResult r = runGadget(doc, sv, R"(
        local a = carvetto.object.create_shape(nil, "circle", {radius = 1})
        local b = carvetto.object.create_shape(nil, "circle", {radius = 2})
        carvetto.selection.set({b})
        carvetto.toolpath.create("drill", nil, {})
    )", "g", quick());
    REQUIRE(r.ok());
    REQUIRE(seen.size() == 1);
    REQUIRE(seen[0].objects.size() == 1);
    CHECK(seen[0].objects[0] == r.createdObjects[1]);
}

TEST_CASE("gadget: forms", "[gadget][t0331]") {
    Document doc;
    FormSpec lastSpec;
    GadgetServices sv;
    sv.ui.form = [&](const FormSpec& spec) -> std::optional<ScriptValue::Map> {
        lastSpec = spec;
        ScriptValue::Map m;
        for (const FormField& f : spec.fields) {
            m.set(f.name, f.defaultValue);
        }
        m.set("count", ScriptValue(7));
        return m;
    };
    const std::string src = R"(
        local v = carvetto.ui.form({
            title = "Weave",
            fields = {
                {name = "count", label = "Count", kind = "number", default = 3, min = 1, max = 10},
                {name = "style", kind = "choice", choices = {"over", "under"}, default = "under"},
                {name = "on", kind = "bool"},
                {name = "text", kind = "text", default = "hi"},
                {name = "layer", kind = "layer"},
                {name = "tool", kind = "tool"},
            }})
        return v.count, v.style, v.on, v.text, v.layer == carvetto.doc.layers()[1], v.tool
    )";
    GadgetRunResult r = runGadget(doc, sv, src, "g", quick());
    INFO(r.script.message);
    REQUIRE(r.ok());
    CHECK(lastSpec.title == "Weave");
    REQUIRE(lastSpec.fields.size() == 6);
    CHECK(lastSpec.fields[0].label == "Count");
    CHECK(lastSpec.fields[1].label == "style");  // defaults to the name
    CHECK(*lastSpec.fields[0].min == 1.0);
    CHECK(lastSpec.fields[1].defaultValue.asString() == "under");
    const ScriptValues& v = r.script.returned;
    CHECK(v[0].asInt() == 7);
    CHECK(v[1].asString() == "under");
    CHECK_FALSE(v[2].asBool());
    CHECK(v[3].asString() == "hi");
    CHECK(v[4].asBool());
    CHECK(v[5].asString().empty());

    SECTION("the limits of a number field are inclusive") {
        double give = 10.0;
        sv.ui.form = [&give](const FormSpec&) {
            ScriptValue::Map m;
            m.set("n", ScriptValue(give));
            return std::optional<ScriptValue::Map>(m);
        };
        const std::string form = "return carvetto.ui.form({fields = {{name = 'n', min = 1, max = 10}}}).n";
        CHECK(runGadget(doc, sv, form, "g", quick()).ok());
        give = 1.0;
        CHECK(runGadget(doc, sv, form, "g", quick()).ok());
        give = 0.999;
        CHECK_FALSE(runGadget(doc, sv, form, "g", quick()).ok());
        give = 10.001;
        CHECK_FALSE(runGadget(doc, sv, form, "g", quick()).ok());
    }
    SECTION("cancel returns nil") {
        sv.ui.form = [](const FormSpec&) { return std::optional<ScriptValue::Map>(); };
        r = runGadget(doc, sv, "return carvetto.ui.form({fields = {{name = 'a'}}}) == nil", "g", quick());
        REQUIRE(r.ok());
        CHECK(r.script.returned[0].asBool());
    }
    SECTION("a bad value from the host is an error, not a script value") {
        sv.ui.form = [](const FormSpec&) {
            ScriptValue::Map m;
            m.set("count", ScriptValue(99));  // above max
            return std::optional<ScriptValue::Map>(m);
        };
        r = runGadget(doc, sv, "carvetto.ui.form({fields = {{name = 'count', min = 1, max = 10}}})", "g", quick());
        REQUIRE_FALSE(r.ok());
        CHECK(contains(r.script.message, "invalid value for 'count'"));
        sv.ui.form = [](const FormSpec&) { return std::optional<ScriptValue::Map>(ScriptValue::Map{}); };
        r = runGadget(doc, sv, "carvetto.ui.form({fields = {{name = 'count'}}})", "g", quick());
        CHECK(contains(r.script.message, "invalid value for 'count'"));
    }
    SECTION("bad specifications") {
        const char* bad[] = {
            "carvetto.ui.form({fields = {{name = 'a'}, {name = 'a'}}})",
            "carvetto.ui.form({fields = {{kind = 'number'}}})",
            "carvetto.ui.form({fields = {{name = 'a', kind = 'slider'}}})",
            "carvetto.ui.form({fields = {{name = 'a', kind = 'choice'}}})",
            "carvetto.ui.form({fields = {{name = 'a', kind = 'choice', choices = {'x'}, default = 'y'}}})",
            "carvetto.ui.form({fields = {{name = 'a', default = 'x'}}})",
            "carvetto.ui.form({fields = {{name = 'a', colour = 1}}})",
            "carvetto.ui.form({title = 'x', fieldz = {}})",
            "carvetto.ui.form({fields = 3})",
        };
        for (const char* b : bad) {
            INFO(b);
            CHECK_FALSE(runGadget(doc, sv, b, "g", quick()).ok());
        }
    }
    SECTION("message and confirm") {
        std::string shown;
        sv.ui.message = [&](const std::string& t) { shown = t; };
        sv.ui.confirm = [](const std::string& t) { return t == "sure?"; };
        r = runGadget(doc, sv, "carvetto.ui.message('hello') return carvetto.ui.confirm('sure?'), carvetto.ui.confirm('x')", "g", quick());
        REQUIRE(r.ok());
        CHECK(shown == "hello");
        CHECK(r.script.returned[0].asBool());
        CHECK_FALSE(r.script.returned[1].asBool());
    }
    SECTION("without a user interface") {
        r = runGadget(doc, noServices(), "carvetto.ui.message('x')", "g", quick());
        CHECK(contains(r.script.message, "no user interface available"));
    }
}

TEST_CASE("gadget: log and print are captured", "[gadget][t0331]") {
    Document doc;
    GadgetRunOptions o = quick();
    o.gadgetLimits.maxLogLines = 3;
    const GadgetRunResult r = runGadget(doc, noServices(),
                                        "print('p') carvetto.log('a', 1, 2.5, true, nil) for i = 1, 10 do carvetto.log(i) end", "g", o);
    REQUIRE(r.ok());
    REQUIRE(r.script.output.size() == 1);
    REQUIRE(r.log.size() == 3);
    CHECK(r.log[0] == "a\t1\t2.5\ttrue\tnil");
    CHECK(r.log[1] == "1");
}

TEST_CASE("gadget: hostile gadgets are stopped and rolled back", "[gadget][t0331]") {
    Document doc;
    const State before = stateOf(doc);

    SECTION("an endless loop hits the instruction limit") {
        GadgetRunOptions o = quick();
        o.limits.maxInstructions = 100'000;
        const GadgetRunResult r = runGadget(doc, noServices(),
                                            "carvetto.object.create_shape(nil,'circle',{radius=1}) while true do end", "g", o);
        CHECK(r.script.status == ScriptStatus::InstructionLimit);
        CHECK(stateOf(doc) == before);
    }
    SECTION("an endless loop of edits hits the object cap") {
        GadgetRunOptions o = quick();
        o.gadgetLimits.maxCreatedObjects = 50;
        const GadgetRunResult r = runGadget(
            doc, noServices(), "while true do carvetto.object.create_shape(nil, 'circle', {radius = 1}) end", "g", o);
        CHECK(r.script.status == ScriptStatus::RuntimeError);
        CHECK(contains(r.script.message, "created too many objects"));
        CHECK(stateOf(doc) == before);
    }
    SECTION("pcall cannot swallow the object cap into a success") {
        GadgetRunOptions o = quick();
        o.gadgetLimits.maxCreatedObjects = 5;
        o.limits.maxInstructions = 200'000;
        const GadgetRunResult r = runGadget(
            doc, noServices(), "while true do pcall(carvetto.object.create_shape, nil, 'circle', {radius = 1}) end", "g", o);
        CHECK_FALSE(r.ok());  // the instruction budget ends the loop
        CHECK(stateOf(doc) == before);
    }
    SECTION("a memory bomb") {
        GadgetRunOptions o = quick();
        o.limits.maxMemoryBytes = 2u * 1024u * 1024u;
        const GadgetRunResult r = runGadget(
            doc, noServices(), "carvetto.layer.create('x') local t = {} for i = 1, 1e7 do t[i] = {i} end", "g", o);
        CHECK(r.script.status == ScriptStatus::MemoryLimit);
        CHECK(stateOf(doc) == before);
    }
    SECTION("cancellation") {
        std::atomic<bool> cancel{true};
        GadgetRunOptions o = quick();
        o.cancel = &cancel;
        const GadgetRunResult r = runGadget(doc, noServices(), "carvetto.layer.create('x')", "g", o);
        CHECK(r.script.status == ScriptStatus::Cancelled);
        CHECK(stateOf(doc) == before);
    }
    SECTION("the wall-clock deadline reports Timeout (fake clock) and rolls back") {
        std::uint64_t t = 0;
        GadgetRunOptions o = quick();
        o.limits.maxInstructions = 500'000'000;
        o.limits.deadlineMillis = 50;
        o.limits.clockNanos = [&t] { return t += 1'000'000; };  // every look at the clock costs 1 ms
        const GadgetRunResult r = runGadget(
            doc, noServices(), "carvetto.layer.create('x') while true do end", "g", o);
        CHECK(r.script.status == ScriptStatus::Timeout);
        CHECK(stateOf(doc) == before);
    }
    SECTION("time spent in a dialog does not count towards the deadline") {
        std::uint64_t t = 0;
        GadgetServices sv;
        sv.ui.message = [&t](const std::string&) { t += 3'600'000'000'000ull; };  // the user takes an hour
        sv.ui.confirm = [&t](const std::string&) {
            t += 3'600'000'000'000ull;
            return true;
        };
        sv.ui.form = [&t](const FormSpec&) {
            t += 3'600'000'000'000ull;
            return std::optional<ScriptValue::Map>(ScriptValue::Map{});
        };
        sv.toolSnapshot = [](const std::string&) { return std::optional<toolpath::ToolSnapshot>(); };
        sv.createToolpath = [&t](const ToolpathRequest&) {
            t += 3'600'000'000'000ull;  // a long calculation
            ToolpathCreated c;
            c.ok = true;
            return c;
        };
        GadgetRunOptions o = quick();
        o.limits.deadlineMillis = 1000;
        o.limits.clockNanos = [&t] { return t += 1'000; };
        o.gadgetLimits.maxPausedNanos = ~0ull;  // this case is about the deadline; the budgets have their own
        o.gadgetLimits.maxCalculationNanos = ~0ull;
        const GadgetRunResult r = runGadget(doc, sv,
                                            "carvetto.ui.message('a') for i = 1, 2000 do end carvetto.ui.confirm('b') "
                                            "carvetto.ui.form({title = 't', fields = {}}) for i = 1, 2000 do end "
                                            "local i = carvetto.object.create_shape(nil, 'circle', {radius = 1}) "
                                            "carvetto.toolpath.create('drill', {i}, {}) "
                                            "carvetto.layer.create('x') for i = 1, 2000 do end",
                                            "g", o);
        INFO(r.script.message);
        CHECK(r.ok());
        CHECK(doc.layers().size() == 2);
    }
    SECTION("Abort in a dialog cancels the run and rolls it back, even at the end of the script") {
        std::atomic<bool> abort{false};
        GadgetServices sv;
        sv.ui.message = [&abort](const std::string&) { abort = true; };
        GadgetRunOptions o = quick();
        o.cancel = &abort;
        GadgetRunResult r = runGadget(doc, sv, "carvetto.layer.create('x') carvetto.ui.message('a')", "g", o);
        CHECK(r.script.status == ScriptStatus::Cancelled);
        CHECK_FALSE(r.committed);
        CHECK(stateOf(doc) == before);
        abort = false;
        // a script that swallows the error and finishes is still cancelled
        r = runGadget(doc, sv, "carvetto.layer.create('x') pcall(carvetto.ui.message, 'a')", "g", o);
        CHECK(r.script.status == ScriptStatus::Cancelled);
        CHECK(stateOf(doc) == before);
    }
    SECTION("but time between dialogs does") {
        std::uint64_t t = 0;
        GadgetServices sv;
        sv.ui.message = [&t](const std::string&) { t += 3'600'000'000'000ull; };
        GadgetRunOptions o = quick();
        o.limits.maxInstructions = 500'000'000;
        o.limits.deadlineMillis = 100;
        o.limits.clockNanos = [&t] { return t += 1'000'000; };
        o.gadgetLimits.maxPausedNanos = ~0ull;  // the waiting budgets have their own tests
        o.gadgetLimits.maxCalculationNanos = ~0ull;
        const GadgetRunResult r = runGadget(doc, sv, "carvetto.ui.message('a') while true do end", "g", o);
        CHECK(r.script.status == ScriptStatus::Timeout);
    }
    SECTION("a gadget cannot run inside an open edit sequence") {
        doc.undoStack().beginMacro("outer");
        const GadgetRunResult r = runGadget(doc, noServices(), "carvetto.layer.create('x')", "g", quick());
        CHECK_FALSE(r.ok());
        CHECK(doc.layers().size() == 1);
        CHECK(doc.undoStack().inMacro());
        doc.undoStack().endMacro();
    }
}

TEST_CASE("gadget: the sandbox gives no file or process access", "[gadget][t0331]") {
    Document doc;
    const char* attempts[] = {"return io.open('/etc/passwd')", "return os.execute('true')", "return require('os')",
                              "return dofile('x')", "return package.loadlib('x', 'y')", "return debug.getinfo(1)"};
    for (const char* src : attempts) {
        INFO(src);
        CHECK_FALSE(runGadget(doc, noServices(), src, "g", quick()).ok());
    }
}
