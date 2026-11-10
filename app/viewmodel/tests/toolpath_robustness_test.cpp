// T-0220 AC2: hostile parameters and vectors neither crash nor hang, 10k vectors calculate within
// the core operation's own cost, and a scripted-event fuzz of the whole session keeps the document
// consistent. Seeds: CARVETTO_FUZZ_SEEDS (default 120) from CARVETTO_FUZZ_FIRST (default 1).

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "toolpath_test_support.hpp"
// Last: on Windows it includes <windows.h> (macros such as near/far; GDI names like Polyline).
#include "../../../core/tests/geometry/cpu_time.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using namespace carvetto::viewmodel::testing;
using document::Document;
using document::ObjectId;
using document::ToolpathId;
using document::ToolpathOperation;
using document::ToolpathRecord;

namespace {

/// An environment variable's value, "" when unset (getenv is deprecated, hence an error, on MSVC).
std::string envText(const char* name) {
#ifdef _WIN32
    char* buffer = nullptr;
    std::size_t length = 0;
    std::string value;
    if (_dupenv_s(&buffer, &length, name) == 0 && buffer != nullptr) {
        value = buffer;
        std::free(buffer);
    }
    return value;
#else
    const char* value = std::getenv(name);
    return value != nullptr ? value : "";
#endif
}

std::uint64_t envNumber(const char* name, std::uint64_t fallback) {
    const std::string text = envText(name);
    return text.empty() ? fallback : std::strtoull(text.c_str(), nullptr, 10);
}

bool tracing() { return !envText("CARVETTO_TRACE").empty(); }

struct Rng {
    std::uint64_t s;
    explicit Rng(std::uint64_t seed) : s(seed * 0x9E3779B97F4A7C15ULL + 0x1234567ULL) {}
    std::uint64_t next() {
        s += 0x9E3779B97F4A7C15ULL;
        std::uint64_t z = s;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }
    std::size_t below(std::size_t n) { return n == 0 ? 0 : static_cast<std::size_t>(next() % n); }
    bool chance(std::size_t percent) { return below(100) < percent; }
    double unit() { return static_cast<double>(next() >> 11) / 9007199254740992.0; }
    double range(double lo, double hi) { return lo + (hi - lo) * unit(); }
    template <class T>
    const T& pick(const std::vector<T>& v) {
        return v[below(v.size())];
    }
};

const ToolpathOperation kOps[] = {ToolpathOperation::Profile, ToolpathOperation::Pocket, ToolpathOperation::Drill,
                                  ToolpathOperation::Engrave, ToolpathOperation::VCarving};

std::string toolFor(ToolpathOperation op, Rng& rng) {
    switch (op) {
        case ToolpathOperation::Profile:
        case ToolpathOperation::Pocket:
            return rng.pick<std::string>({"end-mill-3mm", "end-mill-6mm", "v-bit-90"});
        case ToolpathOperation::Drill: return rng.pick<std::string>({"drill-3mm", "end-mill-3mm"});
        case ToolpathOperation::Engrave: return rng.pick<std::string>({"v-bit-90", "end-mill-3mm"});
        case ToolpathOperation::VCarving:
        case ToolpathOperation::Chamfer: return "v-bit-90";
        case ToolpathOperation::ThreadMill:
        case ToolpathOperation::Fluting: return "end-mill-3mm";
    }
    return "end-mill-3mm";
}

/// A form of `op` with random (in-range or hostile) entries; bad ones are simply rejected.
ToolpathDefinition randomDefinition(const tooling::ToolDatabase& db, ToolpathOperation op, Rng& rng, bool hostile) {
    ToolpathForm form(op, document::Units::Millimetres);
    form.setTool(db, source(toolFor(op, rng)));
    if (op == ToolpathOperation::VCarving && rng.chance(40)) {
        form.setClearanceTools(db, {source("end-mill-3mm")});
    }
    const std::vector<std::string> sane = {"0", "0.5", "1", "2", "3", "4", "6", "10", "45", "0.05", "0.2"};
    const std::vector<std::string> wild = {"0",    "1e-9",  "1e-6", "0.001", "1e3",   "1e6",    "1e9", "-1",
                                           "-1e9", "1e10",  "nan",  "inf",   "abc",   "123456", "0.01", "99999"};
    for (const std::string& id : form.allFieldIds()) {
        if (rng.chance(hostile ? 70 : 35)) {
            form.setField(id, hostile && rng.chance(60) ? rng.pick(wild) : rng.pick(sane));
        }
    }
    // The depth is the one field every cutting operation needs.
    for (const char* depth : {"cutDepth", "depth"}) {
        if (!rng.chance(10)) {
            for (const std::string& id : form.allFieldIds()) {
                if (id == depth) {
                    form.setField(id, rng.pick<std::string>({"1", "2", "3", "5"}));
                }
            }
        }
    }
    for (const std::string& id : form.allChoiceIds()) {
        if (rng.chance(40)) {
            form.setChoice(id, static_cast<int>(rng.below(4)));
        }
    }
    return form.definition();
}

/// A drawing of assorted shapes, some of them awkward, on layer 1.
void fillDocument(Document& doc, Rng& rng, std::size_t count) {
    for (std::size_t i = 0; i < count; ++i) {
        const double x = rng.range(-20, 120);
        const double y = rng.range(-20, 120);
        switch (rng.below(7)) {
            case 0: addVector(doc, square(x, y, rng.range(2, 40))); break;
            case 1: addVector(doc, geometry::Contour::circle({x, y}, rng.range(1, 25))); break;
            case 2: {
                geometry::Contour c;
                c.append(geometry::LineSpan{{x, y}, {x + rng.range(1, 30), y + rng.range(-10, 10)}});
                addVector(doc, c);
                break;
            }
            case 3: {
                geometry::Polyline pl;
                pl.closed = true;
                const std::size_t n = 3 + rng.below(6);
                for (std::size_t k = 0; k < n; ++k) {
                    pl.points.push_back({x + rng.range(0, 30), y + rng.range(0, 30)});  // may self-intersect
                }
                addVector(doc, geometry::Contour::fromPolyline(pl));
                break;
            }
            case 4: addVector(doc, square(x, y, 0.0)); break;  // zero-size
            case 5: {
                geometry::Contour c;  // an arc contour closed by a line
                c.append(geometry::ArcSpan{{x, y}, rng.range(1, 10), 0.0, geometry::kPi, geometry::ArcDirection::CCW});
                c.append(geometry::LineSpan{c.endPoint(), c.startPoint()});
                c.closed = true;
                addVector(doc, c);
                break;
            }
            default: addVector(doc, geometry::Contour{}); break;  // a vector without spans
        }
    }
}

double cpu() { return carvetto::test::processCpuSeconds(); }

struct Timed {
    double seconds{0.0};
    ToolpathOutcome outcome;
};

Timed timedAdd(ToolpathSession& session, const ToolpathDefinition& def) {
    const double t0 = cpu();
    Timed t;
    t.outcome = session.add(def);
    t.seconds = cpu() - t0;
    return t;
}

}  // namespace

TEST_CASE("robustness: hostile parameters are refused or calculated, never a crash or a hang", "[toolpath][robust]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    double worst = 0.0;
    std::size_t produced = 0;
    std::size_t refused = 0;
    const std::uint64_t only = envNumber("CARVETTO_HOSTILE_ONLY", 0);
#ifdef NDEBUG
    const std::uint64_t hostileSeeds = 150;
#else
    const std::uint64_t hostileSeeds = 60;
#endif
    for (std::uint64_t seed = 1; seed <= hostileSeeds; ++seed) {
        if (only != 0 && only != seed) {
            continue;
        }
        Rng rng(seed);
        Document doc;
        ToolpathSession session(&doc);
        session.setToolDatabase(db);
        fillDocument(doc, rng, 4 + rng.below(4));
        doc.undoStack().clear();
        const ToolpathOperation op = kOps[seed % 5];
        const ToolpathDefinition def = randomDefinition(db, op, rng, true);
        const std::vector<ToolpathRecord> before = doc.toolpaths();
        if (tracing()) {
            std::cerr << "hostile seed " << seed << " op " << static_cast<int>(op) << std::endl;
            ToolpathForm shown(def, document::Units::Millimetres);
            for (const ToolpathField& f : shown.fields()) {
                std::cerr << "   " << f.id << " = " << f.text << std::endl;
            }
            for (const ToolpathChoice& ch : shown.choices()) {
                std::cerr << "   " << ch.id << " : " << ch.options[static_cast<std::size_t>(ch.current)] << std::endl;
            }
        }
        const Timed t = timedAdd(session, def);
        worst = std::max(worst, t.seconds);
        if (t.seconds > 3.0) {
            std::cout << "[T-0220 slow hostile] seed " << seed << " op " << static_cast<int>(op) << " " << t.seconds
                      << " s CPU: " << t.outcome.message << "\n";
        }
        INFO("seed " << seed << " op " << static_cast<int>(op) << ": " << t.outcome.message);
        CHECK(t.seconds < 60.0);
        CHECK(doc.checkIntegrity().empty());
        if (t.outcome.ok) {
            ++produced;
            CHECK(doc.undoStack().index() == 1);
            for (const ToolpathRecord& r : doc.toolpaths()) {
                CHECK(document::nonFiniteParam(r.params).empty());
                CHECK(r.moveCount() > 0);
            }
        } else {
            ++refused;
            CHECK(doc.toolpaths() == before);
            CHECK(doc.undoStack().index() == 0);
            CHECK_FALSE(t.outcome.message.empty());
        }
    }
    std::cout << "[T-0220 hostile params] produced=" << produced << " refused=" << refused << " worst=" << worst
              << " s CPU\n";
    CHECK(produced > hostileSeeds / 10);
    CHECK(refused > hostileSeeds / 20);
}

TEST_CASE("robustness: definitions built in code with non-finite or absurd values are refused", "[toolpath][robust]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    Document doc;
    addVector(doc, square(0, 0, 30));
    doc.undoStack().clear();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const std::vector<double> bad = {nan, inf, -inf, 1e300, -1e300};
    for (const double v : bad) {
        {
            ToolpathDefinition d = pocketDefinition(db);
            std::get<toolpath::PocketParams>(d.params).safeZ = v;
            CHECK_FALSE(session.add(d).ok);
            d = pocketDefinition(db);
            std::get<toolpath::PocketParams>(d.params).tolerance.chordal = v;
            CHECK_FALSE(session.add(d).ok);
            d = pocketDefinition(db);
            std::get<toolpath::PocketParams>(d.params).rasterAngle = v;
            CHECK_FALSE(session.add(d).ok);
        }
        {
            ToolpathDefinition d = profileDefinition(db);
            std::get<toolpath::ProfileParams>(d.params).leads.sweep = v;
            CHECK_FALSE(session.add(d).ok);
            d = profileDefinition(db);
            std::get<toolpath::ProfileParams>(d.params).startPosition = {v, 0.0};
            CHECK_FALSE(session.add(d).ok);
            d = profileDefinition(db);
            std::get<toolpath::ProfileParams>(d.params).tolerance.angle = v;
            CHECK_FALSE(session.add(d).ok);
            d = profileDefinition(db);
            std::get<toolpath::ProfileParams>(d.params).tolerance.distance = v;
            CHECK_FALSE(session.add(d).ok);
        }
    }
    {
        ToolpathDefinition d = profileDefinition(db);
        std::get<toolpath::ProfileParams>(d.params).side = static_cast<toolpath::ProfileSide>(77);
        CHECK_FALSE(session.add(d).ok);
        d = engraveDefinition(db);
        std::get<toolpath::EngraveParams>(d.params).passes = std::numeric_limits<int>::max();
        CHECK_FALSE(session.add(d).ok);
        d = engraveDefinition(db);
        std::get<toolpath::EngraveParams>(d.params).passes = -5;
        CHECK_FALSE(session.add(d).ok);
        d = profileDefinition(db);
        std::get<toolpath::ProfileParams>(d.params).tabs.count = std::numeric_limits<int>::max();
        CHECK_FALSE(session.add(d).ok);
        d = pocketDefinition(db);
        d.setName(std::string("x\0y", 3));
        CHECK_FALSE(session.add(d).ok);
        d = pocketDefinition(db);
        d.tool = source("end-mill-6mm", "", "material-acrylic");
        CHECK_FALSE(session.add(d).ok);
    }
    CHECK(doc.toolpaths().empty());
    CHECK(doc.undoStack().index() == 0);
}

TEST_CASE("robustness: work the core has no limit for is refused up front", "[toolpath][robust]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    Document doc;
    addVector(doc, square(0, 0, 30));
    addVector(doc, geometry::Contour::circle({60, 15}, 10.0));
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    const auto refused = [&](const ToolpathDefinition& d, const char* word) {
        const double t0 = cpu();
        const ToolpathOutcome o = session.add(d);
        INFO(o.message);
        CHECK_FALSE(o.ok);
        CHECK(o.message.find(word) != std::string::npos);
        CHECK(cpu() - t0 < 5.0);
    };
    {
        ToolpathForm f(ToolpathOperation::Profile, document::Units::Millimetres);
        REQUIRE(f.setTool(db, source("end-mill-3mm")).ok);
        REQUIRE(f.setField("cutDepth", "5").ok);
        REQUIRE(f.setField("passDepth", "0.0001").ok);
        refused(f.definition(), "passes");
        REQUIRE(f.setField("passDepth", "1").ok);
        REQUIRE(f.setChoice("tabsEnabled", 1).ok);
        REQUIRE(f.setChoice("tabSpacing", 1).ok);
        REQUIRE(f.setField("tabDistance", "0.000000001").ok);
        refused(f.definition(), "moves");
        REQUIRE(f.setField("tabDistance", "50").ok);
        REQUIRE(f.setChoice("rampType", 2).ok);
        REQUIRE(f.setChoice("rampUseAngle", 1).ok);
        REQUIRE(f.setField("rampDistance", "0.000000001").ok);
        refused(f.definition(), "moves");
        // The form bounds the distances that cost time without being of use.
        CHECK_FALSE(f.setField("allowance", "5000").ok);
        CHECK_FALSE(f.setField("rampDistance", "1001").ok);
        REQUIRE(f.setChoice("leadType", 1).ok);
        CHECK_FALSE(f.setField("overcut", "1000000").ok);
        CHECK_FALSE(f.setField("leadLength", "1e6").ok);
        // A definition built in code is held to the same bounds.
        ToolpathDefinition d = profileDefinition(db);
        std::get<toolpath::ProfileParams>(d.params).allowance = 1e5;
        refused(d, "allowance");
        d = profileDefinition(db);
        std::get<toolpath::ProfileParams>(d.params).leads.type = toolpath::LeadType::Straight;
        std::get<toolpath::ProfileParams>(d.params).leads.overcut = 1e6;
        refused(d, "overcut");
    }
    {
        // Vectors that are long in all are refused before the core spends minutes on them.
        Document longDoc;
        for (int k = 0; k < 30; ++k) {
            longDoc.insertObject(document::Object::vector(square(-5e4, -5e4 + k, 1e5), longDoc.defaultLayer(),
                                                          longDoc.activeSheet()));
        }
        ToolpathSession s3(&longDoc);
        s3.setToolDatabase(db);
        const double t0 = cpu();
        const ToolpathOutcome o = s3.add(profileDefinition(db));
        CHECK_FALSE(o.ok);
        CHECK(o.message.find("too long") != std::string::npos);
        CHECK(cpu() - t0 < 5.0);
    }
    {
        // Quick engrave outlines multiply the vectors by the passes.
        Document big;
        for (int k = 0; k < 6000; ++k) {
            big.insertObject(document::Object::vector(square(k % 100 * 6.0, k / 100 * 6.0, 4.0), big.defaultLayer(),
                                                      big.activeSheet()));
        }
        ToolpathSession s2(&big);
        s2.setToolDatabase(db);
        ToolpathForm f(ToolpathOperation::Engrave, document::Units::Millimetres);
        REQUIRE(f.setTool(db, source("v-bit-90")).ok);
        REQUIRE(f.setField("depth", "1").ok);
        REQUIRE(f.setField("passes", "1000").ok);
        const ToolpathOutcome o = s2.add(f.definition());
        CHECK_FALSE(o.ok);
        CHECK(o.message.find("moves") != std::string::npos);
    }
}

TEST_CASE("robustness: degenerate and hostile vectors", "[toolpath][robust]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    struct Shape {
        const char* name;
        geometry::Contour contour;
    };
    std::vector<Shape> shapes;
    shapes.push_back({"no spans", geometry::Contour{}});
    shapes.push_back({"zero square", square(5, 5, 0.0)});
    {
        geometry::Contour c;
        c.append(geometry::LineSpan{{0, 0}, {0, 0}});
        c.closed = true;
        shapes.push_back({"zero line", c});
    }
    {
        geometry::Contour c;
        c.append(geometry::LineSpan{{0, 0}, {10, 0}});
        c.append(geometry::LineSpan{{20, 20}, {30, 0}});  // gap
        c.closed = true;
        shapes.push_back({"gap", c});
    }
    shapes.push_back({"huge square", square(-200, -200, 400)});
    shapes.push_back({"far square", square(9e4, 9e4, 10)});
    shapes.push_back({"beyond limit", square(2e5, 0, 10)});
    shapes.push_back({"beyond limit 1e9", square(2e9, 0, 10)});
    shapes.push_back({"nan", square(nan, 0, 10)});
    shapes.push_back({"inf", square(0, inf, 10)});
    shapes.push_back({"tiny circle", geometry::Contour::circle({0, 0}, 1e-9)});
    shapes.push_back({"zero circle", geometry::Contour::circle({0, 0}, 0.0)});
    shapes.push_back({"negative circle", geometry::Contour::circle({0, 0}, -5.0)});
    shapes.push_back({"huge circle", geometry::Contour::circle({0, 0}, 200)});
    {
        geometry::Contour c;  // a bow tie
        geometry::Polyline pl;
        pl.closed = true;
        pl.points = {{0, 0}, {20, 20}, {20, 0}, {0, 20}};
        shapes.push_back({"bow tie", geometry::Contour::fromPolyline(pl)});
    }
    {
        geometry::Contour c;  // a hairline sliver
        geometry::Polyline pl;
        pl.closed = true;
        pl.points = {{0, 0}, {50, 0.0001}, {50, 0}};
        shapes.push_back({"sliver", geometry::Contour::fromPolyline(pl)});
    }
    {
        geometry::Contour c;  // an arc with absurd angles
        c.append(geometry::ArcSpan{{0, 0}, 5.0, 0.0, 1e300, geometry::ArcDirection::CCW});
        c.closed = true;
        shapes.push_back({"absurd arc", c});
    }
    {
        geometry::Contour c;  // a bezier looping over itself
        c.append(geometry::BezierSpan{{0, 0}, {2, 10}, {-2, 10}, {10, 0}});
        c.append(geometry::LineSpan{{10, 0}, {0, 0}});
        c.closed = true;
        shapes.push_back({"looping bezier", c});
    }
    {
        geometry::Contour c;  // a bezier with handles beyond the coordinate limit
        c.append(geometry::BezierSpan{{0, 0}, {1e7, 5e7}, {-1e7, 5e7}, {10, 0}});
        c.append(geometry::LineSpan{{10, 0}, {0, 0}});
        c.closed = true;
        shapes.push_back({"wild bezier", c});
    }

    double worst = 0.0;
    for (const Shape& shape : shapes) {
        for (const ToolpathOperation op : kOps) {
            Document doc;
            addVector(doc, square(100, 100, 30));  // a good neighbour
            doc.insertObject(document::Object::vector(shape.contour, doc.defaultLayer(), doc.activeSheet()));
            doc.undoStack().clear();
            ToolpathSession session(&doc);
            session.setToolDatabase(db);
            ToolpathDefinition def;
            switch (op) {
                case ToolpathOperation::Profile: def = profileDefinition(db); break;
                case ToolpathOperation::Pocket: def = pocketDefinition(db); break;
                case ToolpathOperation::Drill: def = drillDefinition(db); break;
                case ToolpathOperation::Engrave: def = engraveDefinition(db); break;
                case ToolpathOperation::VCarving: def = vCarveDefinition(db, true); break;
                case ToolpathOperation::Chamfer: def = chamferDefinition(db); break;
                case ToolpathOperation::ThreadMill: def = threadMillDefinition(db); break;
                case ToolpathOperation::Fluting: def = flutingDefinition(db); break;
            }
            if (tracing()) {
                std::cerr << "vector " << shape.name << " op " << static_cast<int>(op) << std::endl;
            }
            const Timed t = timedAdd(session, def);
            worst = std::max(worst, t.seconds);
            if (t.seconds > 2.0) {
                std::cout << "[T-0220 slow vector] " << shape.name << " op " << static_cast<int>(op) << " "
                          << t.seconds << " s CPU" << std::endl;
            }
            INFO(shape.name << " op " << static_cast<int>(op) << ": " << t.outcome.message);
            CHECK(t.seconds < 60.0);
            CHECK(doc.checkIntegrity().empty());
            if (t.outcome.ok) {
                CHECK(doc.undoStack().index() == 1);
                CHECK(session.list().size() == doc.toolpaths().size());
                (void)session.summary();
            } else {
                CHECK(doc.toolpaths().empty());
                CHECK_FALSE(t.outcome.message.empty());
            }
            // The kinds of input the session itself refuses outright.
            for (const char* refusedShape : {"nan", "inf", "beyond limit", "beyond limit 1e9", "wild bezier"}) {
                if (std::string(shape.name) == refusedShape) {
                    CHECK_FALSE(t.outcome.ok);
                    CHECK(t.seconds < 1.0);
                }
            }
        }
    }
    std::cout << "[T-0220 degenerate vectors] worst=" << worst << " s CPU\n";
}

TEST_CASE("robustness: 10k vectors calculate within the cost of the core operation", "[toolpath][robust][scale]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    Document doc;
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
#ifdef NDEBUG
    std::size_t n = 10000;
#else
    std::size_t n = 2500;  // a Debug build runs the core 20 times slower; CARVETTO_SCALE_VECTORS=10000 for the full size
#endif
    n = static_cast<std::size_t>(envNumber("CARVETTO_SCALE_VECTORS", n));
    // Closed vectors on a 14 mm grid (10 mm squares and hexagons), like a page of text. Few
    // vertices each: the pocket refuses regions of more than 200000 polygon vertices.
    const auto cols = static_cast<std::size_t>(std::ceil(std::sqrt(static_cast<double>(n))));
    for (std::size_t k = 0; k < n; ++k) {
        const double x = static_cast<double>(k % cols) * 14.0;
        const double y = static_cast<double>(k / cols) * 14.0;
        geometry::Polyline hexagon;
        hexagon.closed = true;
        for (int corner = 0; corner < 6; ++corner) {
            const double a = corner * (geometry::kPi / 3.0);
            hexagon.points.push_back({x + 5.0 + 5.0 * std::cos(a), y + 5.0 + 5.0 * std::sin(a)});
        }
        doc.insertObject(document::Object::vector(
            k % 2 == 0 ? square(x, y, 10.0) : geometry::Contour::fromPolyline(hexagon), doc.defaultLayer(),
            doc.activeSheet()));
    }
    doc.undoStack().clear();
    REQUIRE(session.countVectors(VectorSelector::allVectors()) == n);

    for (const ToolpathOperation op : {ToolpathOperation::Profile, ToolpathOperation::Pocket}) {
        ToolpathDefinition def = op == ToolpathOperation::Profile ? profileDefinition(db, 2.0) : pocketDefinition(db, 2.0);
        INFO("vectors " << n);
        // The core operation alone on the same contours.
        const ResolvedVectors rv = resolveVectors(doc, def.selector);
        const std::vector<geometry::Contour> contours = rv.copyContours();
        const ResolvedTool tool = *resolveTool(db, def.tool).tool;
        const double c0 = cpu();
        if (op == ToolpathOperation::Profile) {
            toolpath::ProfileParams p = std::get<toolpath::ProfileParams>(def.params);
            p.cuttingData = tool.data;
            const toolpath::ProfileResult r = toolpath::profileToolpath(contours, tool.tool, p);
            REQUIRE(r.ok());
        } else {
            const toolpath::PocketResult r = toolpath::pocketToolpath(
                contours, {toolpath::PocketTool{tool.tool, tool.data}}, std::get<toolpath::PocketParams>(def.params));
            REQUIRE(r.ok());
        }
        const double core = cpu() - c0;

        const double s0 = cpu();
        const ToolpathOutcome out = session.add(def);
        const double total = cpu() - s0;
        INFO(out.message);
        REQUIRE(out.ok);
        std::cout << "[T-0220 10k " << (op == ToolpathOperation::Profile ? "profile" : "pocket") << "] core=" << core
                  << " s, session add=" << total << " s CPU\n";
        // The session adds copying, hashing and the record: a modest multiple, never another order.
        CHECK(total < 2.0 * core + 5.0);
        // Listing and the summary of a 10k-vector job are quick.
        const double l0 = cpu();
        const auto list = session.list();
        (void)session.summary();
        CHECK(cpu() - l0 < 0.5 * core + 5.0);
        CHECK(list.back().status == ToolpathStatus::Current);
        REQUIRE(session.deleteToolpaths({out.ids.front()}).ok);
    }
}

// --- The scripted-event fuzz --------------------------------------------------------------------

namespace {

struct World {
    std::unique_ptr<Document> doc;
    std::unique_ptr<ToolpathSession> session;
    tooling::ToolDatabase db;
    std::vector<ToolpathCalculation> pending;  // calculations waiting to be applied
};

std::vector<ObjectId> someObjects(const Document& doc, Rng& rng, std::size_t most) {
    std::vector<ObjectId> all = doc.objectIds();
    std::vector<ObjectId> out;
    for (const ObjectId id : all) {
        if (out.size() < most && rng.chance(40)) {
            out.push_back(id);
        }
    }
    if (rng.chance(10)) {
        out.push_back(ObjectId{rng.below(500) + 1});  // maybe stale, maybe a layer or sheet id
    }
    return out;
}

VectorSelector randomSelector(const Document& doc, Rng& rng) {
    VectorSelector s;
    switch (rng.below(20)) {
        case 0: case 1: case 2: case 3: case 4: case 5: case 6: case 7: case 8: case 9: case 10: break;
        case 11: case 12: case 13: s = VectorSelector::fromLayers({doc.layers()[rng.below(doc.layers().size())].id}); break;
        case 14: case 15: case 16: case 17: s = VectorSelector::fromObjects(someObjects(doc, rng, 6)); break;
        default: s = VectorSelector::fromSelection(doc); break;
    }
    if (rng.chance(20)) {
        s.includeOpen = rng.chance(50);
    }
    if (rng.chance(10)) {
        s.onlyCircles = true;
    }
    return s;
}

tooling::ToolDatabase mutatedDatabase(const tooling::ToolDatabase& base, Rng& rng) {
    tooling::ToolDatabase db = base;
    switch (rng.below(4)) {
        case 0: db.removeTool(rng.pick<std::string>({"end-mill-3mm", "end-mill-6mm", "v-bit-90", "drill-3mm"})); break;
        case 1:
            if (const tooling::Tool* found = db.findTool("end-mill-3mm")) {
                tooling::Tool t = *found;
                t.defaultCuttingData.feedRate = rng.range(100, 2000);
                db.updateTool(t);
            }
            break;
        case 2: db = tooling::ToolDatabase{}; break;
        default: break;
    }
    return db;
}

/// All invariants that must hold after every event.
void checkInvariants(World& w, std::uint64_t seed, int step, const char* what) {
    Document& doc = *w.doc;
    INFO("seed " << seed << " step " << step << " event " << what);
    // The fuzz selects stale ids on purpose (the session must cope); every other issue is a bug.
    for (const document::IntegrityIssue& issue : doc.checkIntegrity()) {
        INFO(issue.message);
        REQUIRE(issue.kind == document::IntegrityIssue::Kind::StaleSelection);
    }
    const auto list = w.session->list();
    REQUIRE(list.size() == doc.toolpaths().size());
    for (std::size_t i = 0; i < list.size(); ++i) {
        REQUIRE(list[i].id == doc.toolpaths()[i].id);
        REQUIRE(validateToolpathName(doc.toolpaths()[i].name).empty());
        REQUIRE(document::nonFiniteParam(doc.toolpaths()[i].params).empty());
    }
    const JobSummary s = w.session->summary();
    REQUIRE(s.toolpaths == list.size());
    REQUIRE(std::isfinite(s.time.total()));
}

struct Counters {
    std::map<std::string, std::size_t> reasons;
    std::map<std::string, double> cpuByEvent;
    std::size_t events{0};
    std::size_t addsOk{0};
    std::size_t editsOk{0};
    std::size_t refused{0};
    std::size_t undoChecks{0};
};

/// Runs one op that may edit the document; checks the one-undo-step and exact-undo contracts.
template <class F>
void checkedEdit(World& w, Counters& c, std::uint64_t seed, F&& op) {
    Document& doc = *w.doc;
    const std::size_t index = doc.undoStack().index();
    const std::vector<ToolpathRecord> before = doc.toolpaths();
    const auto objectsBefore = doc.objects();
    const ToolpathOutcome out = op();
    const std::size_t after = doc.undoStack().index();
    INFO("seed " << seed << ": " << out.message);
    if (!out.ok) {
        ++c.refused;
        ++c.reasons[out.message.substr(0, 60)];
        REQUIRE(after == index);
        REQUIRE(doc.toolpaths() == before);
        return;
    }
    REQUIRE(after <= index + 1);
    if (after == index + 1) {
        const std::vector<ToolpathRecord> mine = doc.toolpaths();
        REQUIRE(doc.undo());
        REQUIRE(doc.toolpaths() == before);
        REQUIRE(doc.objects() == objectsBefore);
        REQUIRE(doc.redo());
        REQUIRE(doc.toolpaths() == mine);
        ++c.undoChecks;
    } else {
        REQUIRE(doc.toolpaths() == before);
    }
}

void fuzzOne(std::uint64_t seed, Counters& c) {
    Rng rng(seed);
    World w;
    w.db = cliToolDatabase();
    w.doc = std::make_unique<Document>();
    w.session = std::make_unique<ToolpathSession>(w.doc.get());
    w.session->setToolDatabase(w.db);
    w.session->setMachineSetup(MachineSetup{3000.0, 800.0});
    addVector(*w.doc, square(0, 0, 30));
    addVector(*w.doc, geometry::Contour::circle({60, 15}, 12.0));
    fillDocument(*w.doc, rng, 1 + rng.below(4));
    w.doc->undoStack().clear();

    const int steps = 30;
    for (int step = 0; step < steps; ++step) {
        Document& doc = *w.doc;
        ToolpathSession& session = *w.session;
        const char* what = "?";
        const std::vector<ToolpathRecord>& recs = doc.toolpaths();
        const auto anyId = [&]() {
            return recs.empty() || rng.chance(4) ? ToolpathId{rng.below(40)} : recs[rng.below(recs.size())].id;
        };
        ++c.events;
        const double stepStart = cpu();
        std::size_t kind = rng.below(24);
        if (recs.size() < 2 && rng.chance(50)) {
            kind = 0;  // keep the list populated
        }
        switch (kind) {
            case 0:
            case 1:
            case 2: {
                what = "add";
                ToolpathDefinition def = randomDefinition(w.db, kOps[rng.below(5)], rng, rng.chance(20));
                def.selector = randomSelector(doc, rng);
                checkedEdit(w, c, seed, [&] {
                    ToolpathOutcome o = session.add(def);
                    c.addsOk += o.ok ? 1 : 0;
                    return o;
                });
                break;
            }
            case 3:
            case 4: {
                what = "update";
                const ToolpathId id = anyId();
                std::optional<ToolpathDefinition> cur = session.definition(id);
                ToolpathDefinition def = cur.has_value() ? *cur : randomDefinition(w.db, kOps[rng.below(5)], rng, false);
                if (cur.has_value() && rng.chance(70)) {
                    def = randomDefinition(w.db, cur->operation(), rng, rng.chance(15));
                    def.selector = rng.chance(50) ? cur->selector : randomSelector(doc, rng);
                }
                checkedEdit(w, c, seed, [&] {
                    ToolpathOutcome o = session.update(id, def);
                    c.editsOk += o.ok ? 1 : 0;
                    return o;
                });
                break;
            }
            case 5:
                what = "recalculate";
                checkedEdit(w, c, seed, [&] { return session.recalculate(anyId()); });
                break;
            case 6: {
                what = "recalculateAll";
                const bool only = rng.chance(50);
                checkedEdit(w, c, seed, [&] {
                    const BatchOutcome b = session.recalculateAll(only);
                    ToolpathOutcome o;
                    o.ok = b.ok || b.recalculated > 0;
                    o.message = b.message;
                    return o;
                });
                break;
            }
            case 7:
                what = "duplicate";
                checkedEdit(w, c, seed, [&] { return session.duplicate(anyId()); });
                break;
            case 8: {
                what = "delete";
                std::vector<ToolpathId> ids;
                for (const ToolpathRecord& r : recs) {
                    if (rng.chance(30)) {
                        ids.push_back(r.id);
                    }
                }
                if (rng.chance(10)) {
                    ids.push_back(ToolpathId{rng.below(900)});
                }
                checkedEdit(w, c, seed, [&] { return session.deleteToolpaths(ids); });
                break;
            }
            case 9:
                what = "reorder";
                checkedEdit(w, c, seed, [&] { return session.reorder(anyId(), rng.below(recs.size() + 1)); });
                break;
            case 10: {
                what = "rename";
                static const std::vector<std::string> names = {"A", "Plate", "x\ny", "", "tab\t", "Caf\xC3\xA9", "q"};
                checkedEdit(w, c, seed, [&] { return session.rename(anyId(), rng.pick(names)); });
                break;
            }
            case 11:
                what = "enable";
                session.setEnabled(anyId(), rng.chance(50));
                break;
            case 12:
                what = "undo";
                doc.undo();
                break;
            case 13:
                what = "redo";
                doc.redo();
                break;
            case 14: {
                what = "edit vectors";
                switch (rng.below(5)) {
                    case 0: addVector(doc, square(rng.range(0, 100), rng.range(0, 100), rng.range(3, 30))); break;
                    case 1:
                        if (!doc.objectIds().empty()) {
                            const auto ids = doc.objectIds();
                            doc.execute(std::make_unique<document::TransformObjectsCommand>(
                                std::vector<ObjectId>{ids[rng.below(ids.size())]},
                                geometry::Transform2::translation({rng.range(-5, 5), rng.range(-5, 5)})));
                        }
                        break;
                    case 2:
                        if (!doc.objectIds().empty()) {
                            const auto ids = doc.objectIds();
                            doc.execute(std::make_unique<document::RemoveObjectsCommand>(
                                std::vector<ObjectId>{ids[rng.below(ids.size())]}));
                        }
                        break;
                    case 3: fillDocument(doc, rng, 1); break;
                    default: break;
                }
                break;
            }
            case 15: {
                what = "layers";
                document::Layer l = doc.layers()[rng.below(doc.layers().size())];
                l.visible = rng.chance(70);
                l.locked = rng.chance(20);
                doc.execute(std::make_unique<document::SetLayerPropertiesCommand>(l));
                if (rng.chance(30)) {
                    addLayer(doc, "L" + std::to_string(step));
                }
                break;
            }
            case 16: {
                what = "selection";
                std::vector<ObjectId> ids = someObjects(doc, rng, 5);
                doc.selection().assign(ids);
                break;
            }
            case 17: {
                what = "sheets";
                if (rng.chance(50)) {
                    document::Sheet s;
                    s.name = "S" + std::to_string(step);
                    doc.insertSheet(s);
                }
                doc.setActiveSheet(doc.sheets()[rng.below(doc.sheets().size())].id);
                w.session->invalidate();
                break;
            }
            case 18:
                what = "database";
                w.db = rng.chance(60) ? cliToolDatabase() : mutatedDatabase(w.db, rng);
                session.setToolDatabase(w.db);
                break;
            case 19: {
                what = "prepare, edit, apply";
                ToolpathDefinition def = randomDefinition(w.db, kOps[rng.below(5)], rng, false);
                def.selector = randomSelector(doc, rng);
                const ToolpathJob job = session.prepare(def);
                ToolpathCalculation calc = job.run();
                w.pending.push_back(std::move(calc));
                if (rng.chance(60) && !doc.objectIds().empty()) {
                    const auto ids = doc.objectIds();
                    doc.execute(std::make_unique<document::TransformObjectsCommand>(
                        std::vector<ObjectId>{ids[rng.below(ids.size())]}, geometry::Transform2::translation({1.0, 0.0})));
                }
                if (!w.pending.empty() && rng.chance(80)) {
                    const std::size_t at = rng.below(w.pending.size());
                    ToolpathCalculation use = w.pending[at];
                    if (rng.chance(10)) {
                        use.target = anyId();
                    }
                    checkedEdit(w, c, seed, [&] { return session.apply(use); });
                    if (rng.chance(50)) {
                        w.pending.erase(w.pending.begin() + static_cast<std::ptrdiff_t>(at));
                    }
                }
                if (w.pending.size() > 6) {
                    w.pending.clear();
                }
                break;
            }
            case 20: {
                what = "cancel";
                const ToolpathJob job = session.prepare(randomDefinition(w.db, kOps[rng.below(5)], rng, false));
                std::atomic<bool> stop{true};
                const ToolpathCalculation calc = job.run(&stop);
                REQUIRE_FALSE(calc.ok);
                REQUIRE_FALSE(session.apply(calc).ok);
                break;
            }
            case 21: {
                what = "document replacement";
                if (rng.chance(50)) {
                    // A saved and reloaded copy of the document.
                    namespace fs = std::filesystem;
                    const fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / "fuzz";
                    fs::create_directories(dir);
                    const fs::path file = dir / ("f" + std::to_string(seed) + ".carv");
                    if (document::saveCarv(doc, file, {})) {
                        auto loaded = document::loadCarv(file);
                        if (loaded) {
                            auto fresh = std::make_unique<Document>(std::move(loaded.value().document));
                            session.setDocument(fresh.get());
                            w.doc = std::move(fresh);  // the old document is destroyed here
                        }
                        std::error_code ec;
                        fs::remove(file, ec);
                    }
                } else {
                    auto fresh = std::make_unique<Document>();
                    session.setDocument(fresh.get());
                    w.doc = std::move(fresh);
                    fillDocument(*w.doc, rng, 2 + rng.below(3));
                }
                w.pending.clear();
                break;
            }
            case 22:
                what = "forms";
                if (!recs.empty()) {
                    (void)session.form(anyId());
                    (void)session.definition(anyId());
                    (void)session.lastMessages(anyId());
                }
                (void)session.newForm(kOps[rng.below(5)]);
                break;
            default:
                what = "machine";
                session.setMachineSetup(MachineSetup{rng.range(0, 6000), rng.chance(50) ? rng.range(10, 2000)
                                                                                         : std::numeric_limits<double>::infinity()});
                break;
        }
        checkInvariants(w, seed, step, what);
        c.cpuByEvent[what] += cpu() - stepStart;
    }
}

}  // namespace

TEST_CASE("fuzz: scripted session events keep the document consistent", "[toolpath][fuzz]") {
#ifdef NDEBUG
    const std::uint64_t defaultSeeds = 120;
#else
    const std::uint64_t defaultSeeds = 24;
#endif
    const std::uint64_t seeds = envNumber("CARVETTO_FUZZ_SEEDS", defaultSeeds);
    const std::uint64_t first = envNumber("CARVETTO_FUZZ_FIRST", 1);
    Counters c;
    const double t0 = cpu();
    for (std::uint64_t seed = first; seed < first + seeds; ++seed) {
        fuzzOne(seed, c);
    }
    std::cout << "[T-0220 fuzz] seeds=" << seeds << " events=" << c.events << " adds ok=" << c.addsOk
              << " edits ok=" << c.editsOk << " refused=" << c.refused << " undo/redo checks=" << c.undoChecks
              << " cpu=" << (cpu() - t0) << " s\n";
    if (tracing()) {
        for (const auto& [why, n] : c.reasons) {
            std::cout << "   " << n << " x " << why << "\n";
        }
        for (const auto& [event, seconds] : c.cpuByEvent) {
            std::cout << "   cpu " << seconds << " s in " << event << "\n";
        }
    }
    CHECK(c.addsOk > seeds / 4);
    CHECK(c.undoChecks > seeds / 4);
}
