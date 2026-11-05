// T-0150 AC2: drilling toolpaths through every shipped post. The output is checked against the
// post-safety rules of T-0110/T-0133/T-0134 (units and absolute mode first, XY rapids only at safe
// Z, rapids below safe Z only as upward Z-only peck retracts, spindle on with a speed > 0 before
// any cutting move, tool changes with the spindle off at safe Z, dwells > 0 in the controller's
// unit, printable ASCII only, no canned drilling cycles), and the Grbl output against a golden
// (core/tests/golden/toolpath/drill_holes_grbl.nc, regenerated only by tools/regen_golden.py and
// hand-checked against the posts/README.md checklist and ADR-0022).

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/post/post_processor.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/drill.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using geometry::Box2;
using geometry::Contour;
using geometry::Vec2;

namespace {

constexpr double kSafeZ = 5.0;

std::string readFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        FAIL("cannot open " << path);
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

void writeFile(const std::string& path, const std::string& text) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    REQUIRE(out);
    out << text;
}

tooling::Tool drill3mm() {
    tooling::Tool t;
    t.id = "drill-3mm";
    t.name = "Drill 3 mm";
    t.type = tooling::ToolType::Drill;
    t.diameter = 3.0;
    t.includedAngle = 118.0 * geometry::kPi / 180.0;
    t.defaultCuttingData.toolNumber = 2;
    t.defaultCuttingData.feedRate = 600.0;
    t.defaultCuttingData.plungeRate = 250.0;
    t.defaultCuttingData.spindleSpeed = 10000.0;
    t.defaultCuttingData.passDepth = 2.5;
    return t;
}

/// The drilling fixture: three toolpaths over the same holes (two circles, a square, a slot,
/// a hole marker smaller than the drill) — peck above the start with a 0.5 s dwell, peck above
/// the previous pass from a start depth of 1 mm, and single plunges with a 0.25 s dwell.
std::vector<Toolpath> drillJob() {
    Contour slot;
    slot.append(geometry::LineSpan{{60, 50}, {80, 50}});
    slot.append(geometry::ArcSpan{{80, 54}, 4.0, -geometry::kPi / 2, geometry::kPi / 2, geometry::ArcDirection::CCW});
    slot.append(geometry::LineSpan{{80, 58}, {60, 58}});
    slot.append(geometry::ArcSpan{{60, 54}, 4.0, geometry::kPi / 2, 3 * geometry::kPi / 2, geometry::ArcDirection::CCW});
    slot.closed = true;
    const std::vector<Contour> holes{Contour::circle({20, 20}, 3.0), Contour::circle({60, 20}, 5.0),
                                     Contour::rectangle(Box2{Vec2{15, 45}, Vec2{25, 55}}), slot,
                                     Contour::circle({40, 35}, 0.5)};
    std::vector<Toolpath> out;

    DrillParams peck;
    peck.name = "Peck above start";
    peck.cutDepth = 6.0;
    peck.safeZ = kSafeZ;
    peck.peck = true;
    peck.retractGap = 1.0;
    peck.dwellTime = 0.5;
    const DrillResult a = drillToolpath(holes, drill3mm(), std::nullopt, peck);
    REQUIRE(a.ok());
    REQUIRE(a.diagnostics.size() == 1);  // the marker circle is narrower than the drill
    CHECK(a.diagnostics[0].issue == ToolpathIssue::HoleWiderThanVector);
    out.push_back(a.toolpath);

    DrillParams chip = peck;
    chip.name = "Peck above previous pass";
    chip.startDepth = 1.0;
    chip.peckRetract = PeckRetract::AbovePreviousPass;
    chip.retractGap = 0.5;
    chip.dwellTime = 0.0;
    chip.order.leftToRight = true;
    const DrillResult b = drillToolpath(holes, drill3mm(), std::nullopt, chip);
    REQUIRE(b.ok());
    out.push_back(b.toolpath);

    DrillParams single;
    single.name = "Single plunge";
    single.cutDepth = 3.0;
    single.safeZ = kSafeZ;
    single.dwellTime = 0.25;
    single.order.selectionOrder = true;
    const DrillResult c = drillToolpath(holes, drill3mm(), std::nullopt, single);
    REQUIRE(c.ok());
    out.push_back(c.toolpath);
    return out;
}

post::PostContext context(const std::string& name) {
    post::PostContext ctx;
    ctx.fileName = name;
    ctx.product = "Carvetto";
    ctx.home = {0, 0, 20};
    ctx.safeZ = kSafeZ;
    return ctx;
}

struct ShippedPost {
    const char* stem;
    bool inch;
    const char* lineEnding;
    const char* dwellHalf;     ///< the dwell word for 0.5 s
    const char* dwellQuarter;  ///< ... and for 0.25 s
};

const std::vector<ShippedPost> kPosts{
    {"grbl_mm", false, "\n", "P0.50", "P0.25"},        {"grbl_inch", true, "\n", "P0.50", "P0.25"},
    {"mach3_mm", false, "\r\n", "P0.50", "P0.25"},     {"mach3_inch", true, "\r\n", "P0.50", "P0.25"},
    {"mach4_mm", false, "\r\n", "P0.50", "P0.25"},     {"linuxcnc_mm", false, "\n", "P0.50", "P0.25"},
    {"linuxcnc_inch", true, "\n", "P0.50", "P0.25"},   {"marlin_mm", false, "\n", "P500", "P250"},
};

struct Word {
    char letter;
    std::string text;
    double value;
};

/// Words of a line without comments and line number (Marlin's `M0 <message>` keeps only M0).
std::vector<Word> words(const std::string& line) {
    std::string code;
    int depth = 0;
    for (const char c : line) {
        if (c == ';' && depth == 0) break;
        if (c == '(') {
            ++depth;
        } else if (c == ')') {
            --depth;
        } else if (depth == 0) {
            code += c;
        }
    }
    std::vector<Word> out;
    std::istringstream in(code);
    std::string tok;
    while (in >> tok) {
        if (tok[0] < 'A' || tok[0] > 'Z' || tok.size() < 2) break;
        Word w{tok[0], tok.substr(1), 0.0};
        std::istringstream num(w.text);
        if (!(num >> w.value)) break;
        out.push_back(w);
        if (w.letter == 'M' && w.text == "0") break;
    }
    if (!out.empty() && out.front().letter == 'N') {
        out.erase(out.begin());
    }
    return out;
}

/// The safety rules over one program; returns the problems found.
std::vector<std::string> problems(const ShippedPost& p, const std::string& program) {
    std::vector<std::string> out;
    const double unit = p.inch ? 25.4 : 1.0;
    const double safeZ = std::round(kSafeZ / unit * 1e4) / 1e4;
    for (const char c : program) {
        const auto b = static_cast<unsigned char>(c);
        if ((b < 0x20 || b > 0x7E) && c != '\n' && c != '\r') {
            out.push_back("byte " + std::to_string(b) + " is not printable ASCII");
            break;
        }
    }
    bool unitsSet = false;
    bool absolute = false;
    bool spindleOn = false;
    double speed = 0.0;
    int motion = -1;
    std::optional<double> x;
    std::optional<double> y;
    std::optional<double> z;
    std::vector<std::string> dwells;
    std::istringstream in(program);
    std::string line;
    std::size_t n = 0;
    while (std::getline(in, line)) {
        ++n;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const std::string where = "line " + std::to_string(n) + " '" + line + "': ";
        std::map<char, double> axis;
        bool dwell = false;
        bool toolChange = false;
        for (const Word& w : words(line)) {
            if (w.letter == 'G') {
                const int g = static_cast<int>(w.value);
                if (w.text == "20" || w.text == "21") unitsSet = true;
                if (w.text == "90") absolute = true;
                if (w.text == "91") absolute = false;
                if (g >= 0 && g <= 3 && w.text.find('.') == std::string::npos) motion = g;
                if (w.text == "4") dwell = true;
                if (w.text == "43") z.reset();
                if (w.text == "73" || w.text == "76" || (g >= 80 && g <= 89 && w.text != "80") || w.text == "98" ||
                    w.text == "99") {
                    out.push_back(where + "canned cycle word G" + w.text);
                }
            } else if (w.letter == 'M') {
                if (w.text == "3" || w.text == "4") spindleOn = true;
                if (w.text == "5") spindleOn = false;
                if (w.text == "0" || w.text == "6") toolChange = true;
            } else if (w.letter == 'S') {
                speed = w.value;
            } else if (w.letter == 'P' && dwell) {
                dwells.push_back("P" + w.text);
                if (!(w.value > 0.0)) out.push_back(where + "dwell of zero");
            } else if (w.letter == 'X' || w.letter == 'Y' || w.letter == 'Z') {
                axis[w.letter] = w.value;
            }
        }
        if (toolChange) {
            if (spindleOn) out.push_back(where + "tool change with the spindle on");
            if (z && *z < safeZ - 1e-9) out.push_back(where + "tool change below safe Z");
            z.reset();
        }
        if (axis.empty() || dwell) continue;
        if (!unitsSet || !absolute) out.push_back(where + "motion before the units code or G90");
        const std::optional<double> nx = axis.count('X') ? std::optional<double>(axis['X']) : x;
        const std::optional<double> ny = axis.count('Y') ? std::optional<double>(axis['Y']) : y;
        const std::optional<double> nz = axis.count('Z') ? std::optional<double>(axis['Z']) : z;
        const bool xy = (axis.count('X') && (!x || *x != axis['X'])) || (axis.count('Y') && (!y || *y != axis['Y']));
        if (motion == 0) {
            if (xy && (!z || *z < safeZ - 1e-9 || !nz || *nz < safeZ - 1e-9)) {
                out.push_back(where + "XY rapid while the tool is not known to be at safe Z");
            }
            if (nz && *nz < safeZ - 1e-9 && (xy || !z || !(*nz > *z))) {
                out.push_back(where + "rapid below safe Z that is not an upward peck retract");
            }
        } else if (motion >= 1) {
            if (!spindleOn || !(speed > 0.0)) out.push_back(where + "cutting with the spindle off");
            if (!x || !y || !z) out.push_back(where + "cutting from an unknown position");
            if (motion == 1 && xy) out.push_back(where + "drilling feeds sideways");
        } else {
            out.push_back(where + "motion without a motion mode");
        }
        x = nx;
        y = ny;
        z = nz;
    }
    if (spindleOn) out.push_back("program ends with the spindle on");
    if (!z || *z < safeZ - 1e-9) out.push_back("program does not end at safe Z");
    // 5 holes x 3 passes with 0.5 s, then 5 single plunges with 0.25 s.
    std::vector<std::string> want(15, p.dwellHalf);
    want.insert(want.end(), 5, p.dwellQuarter);
    if (dwells != want) out.push_back("dwell words differ from the expected 15 x 0.5 s and 5 x 0.25 s");
    return out;
}

std::string run(const std::string& stem, const std::vector<Toolpath>& job, const std::string& name) {
    auto pp = post::PostProcessor::parse(readFile(std::string(CARVETTO_POSTS_DIR) + "/" + stem + ".pp"));
    REQUIRE(pp.ok());
    const auto r = pp.value().run(job, context(name));
    if (!r.ok()) {
        FAIL(stem << ": " << r.error().describe());
    }
    CHECK(r.value().warnings.empty());
    return r.value().text;
}

const std::string kGolden = std::string(CARVETTO_GOLDEN_DIR) + "/toolpath/drill_holes_grbl.nc";

}  // namespace

TEST_CASE("AC2: drilling through every shipped post passes the safety rules", "[toolpath][drill][post][safety]") {
    const std::vector<Toolpath> job = drillJob();
    for (const ShippedPost& p : kPosts) {
        INFO(p.stem);
        const std::string program = run(p.stem, job, "drill_holes");
        for (const std::string& problem : problems(p, program)) {
            UNSCOPED_INFO(problem);
        }
        CHECK(problems(p, program).empty());
    }
}

TEST_CASE("AC2: the safety rules catch drilling hazards", "[toolpath][drill][post][safety]") {
    const ShippedPost& grbl = kPosts.front();
    const std::string ok = run(grbl.stem, drillJob(), "drill_holes");
    REQUIRE(problems(grbl, ok).empty());
    auto replaced = [&](const std::string& from, const std::string& to) {
        std::string s = ok;
        const std::size_t at = s.find(from);
        REQUIRE(at != std::string::npos);
        s.replace(at, from.size(), to);
        return s;
    };
    CHECK_FALSE(problems(grbl, replaced("G1 Z-2.500 F250", "G0 Z-2.500")).empty());     // rapid into the hole
    CHECK_FALSE(problems(grbl, replaced("G0 Z1.000", "G0 X1 Z1.000")).empty());         // sideways in the hole
    CHECK_FALSE(problems(grbl, replaced("G4 P0.50", "G4 P0.00")).empty());              // zero dwell
    CHECK_FALSE(problems(grbl, replaced("G1 Z-2.500 F250", "G83 Z-2.500 R1 Q2")).empty());  // canned cycle
    CHECK_FALSE(problems(grbl, replaced("M3 S10000", "M3 S0")).empty());                // spindle speed 0
}

TEST_CASE("AC2: the drilling fixture through the Grbl post matches the golden", "[toolpath][drill][post][golden]") {
    CHECK(run("grbl_mm", drillJob(), "drill_holes") == readFile(kGolden));
}

TEST_CASE("AC2: the drilling golden reads as the fixture says", "[toolpath][drill][post][golden]") {
    const std::string g = readFile(kGolden);
    CHECK(g.find("(Peck above start)") != std::string::npos);
    CHECK(g.find("(Peck above previous pass)") != std::string::npos);
    CHECK(g.find("(Single plunge)") != std::string::npos);
    CHECK(g.find("M3 S10000") != std::string::npos);
    CHECK(g.find("G0 Z1.000") != std::string::npos);    // peck retract: cutting start 0 + gap 1
    CHECK(g.find("G0 Z-3.000") != std::string::npos);   // above previous pass: -3.5 + 0.5
    CHECK(g.find("G1 Z-2.500 F250") != std::string::npos);  // plunge at the plunge rate
    CHECK(g.find("G1 Z-6.000\n") != std::string::npos);
    CHECK(g.find("G1 Z-7.000") != std::string::npos);   // start depth 1 + cut depth 6
    CHECK(g.find("G4 P0.50") != std::string::npos);
    CHECK(g.find("G4 P0.25") != std::string::npos);
    CHECK(g.find("G83") == std::string::npos);
    CHECK(g.find("G81") == std::string::npos);
}

// Hidden (tag [.regen], never run by ctest): rewrites the drilling golden. Run via
// tools/regen_golden.py and hand-check the diff against posts/README.md before committing.
TEST_CASE("Regenerate the drilling golden", "[.regen]") {
    writeFile(kGolden, run("grbl_mm", drillJob(), "drill_holes"));
}
