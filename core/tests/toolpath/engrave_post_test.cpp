// T-0151 AC1 (safety rules as T-0150): quick engrave toolpaths through every shipped post. The
// output is checked against the post-safety rules of T-0110/T-0133/T-0134 (units and absolute
// mode first, rapids only at safe Z, spindle on with a speed > 0 before any cutting move, tool
// changes with the spindle off at safe Z, nothing below the engrave depth, printable ASCII only),
// and the Grbl output against a golden (core/tests/golden/toolpath/engrave_grbl.nc, regenerated
// only by tools/regen_golden.py and hand-checked against the posts/README.md checklist and
// ADR-0024).

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
#include "carvetto/geometry/span.hpp"
#include "carvetto/post/post_processor.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/engrave.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using geometry::Box2;
using geometry::Contour;
using geometry::Vec2;

namespace {

constexpr double kSafeZ = 5.0;
constexpr double kDeepest = 0.6;  // the deepest engrave depth of the fixture

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

tooling::Tool vBit(const std::string& name, double angleDeg, int number) {
    tooling::Tool t;
    t.id = name;
    t.name = name;
    t.type = tooling::ToolType::VBit;
    t.diameter = 6.35;
    t.includedAngle = angleDeg * geometry::kPi / 180.0;
    t.defaultCuttingData.toolNumber = number;
    t.defaultCuttingData.feedRate = 1500.0;
    t.defaultCuttingData.plungeRate = 500.0;
    t.defaultCuttingData.spindleSpeed = 18000.0;
    t.defaultCuttingData.passDepth = 1.0;
    t.defaultCuttingData.stepover = 0.3;
    return t;
}

tooling::Tool endMill1mm() {
    tooling::Tool t;
    t.id = "em-1";
    t.name = "End Mill 1 mm";
    t.type = tooling::ToolType::EndMill;
    t.diameter = 1.0;
    t.defaultCuttingData.toolNumber = 5;
    t.defaultCuttingData.feedRate = 800.0;
    t.defaultCuttingData.plungeRate = 300.0;
    t.defaultCuttingData.spindleSpeed = 20000.0;
    t.defaultCuttingData.passDepth = 0.5;
    t.defaultCuttingData.stepover = 0.4;
    return t;
}

/// The quick engrave fixture: Outline with a 60-degree V-bit over a closed rectangle, a circle,
/// an open polyline and an open Bezier in two passes; Fill with a 90-degree V-bit (offset rings
/// round a circular island); Fill with a 1 mm end mill (raster at 30 degrees, two passes).
std::vector<Toolpath> engraveJob() {
    Contour zig;
    zig.append(geometry::LineSpan{{5, 40}, {15, 48}});
    zig.append(geometry::LineSpan{{15, 48}, {25, 40}});
    zig.closed = false;
    Contour wave;
    wave.append(geometry::BezierSpan{{30, 40}, {35, 50}, {40, 30}, {45, 40}});
    wave.closed = false;
    const std::vector<Contour> marks{Contour::rectangle(Box2{Vec2{5, 5}, Vec2{25, 15}}), Contour::circle({40, 10}, 5.0),
                                     zig, wave};
    std::vector<Toolpath> out;

    EngraveParams outline;
    outline.name = "Outline V-bit";
    outline.depth = 0.6;
    outline.passes = 2;
    outline.safeZ = kSafeZ;
    outline.tolerance.chordal = 0.05;
    const EngraveResult a = engraveToolpath(marks, vBit("V-bit 60", 60.0, 3), std::nullopt, outline);
    REQUIRE(a.ok());
    CHECK(a.diagnostics.empty());
    out.push_back(a.toolpath);

    const std::vector<Contour> plate{Contour::rectangle(Box2{Vec2{60, 5}, Vec2{72, 13}}), Contour::circle({66, 9}, 2.0)};
    EngraveParams offset;
    offset.name = "Fill offset";
    offset.depth = 0.5;
    offset.safeZ = kSafeZ;
    offset.strategy = EngraveStrategy::Fill;
    offset.fillPattern = PocketStrategy::Offset;
    offset.stepover = 0.6;
    const EngraveResult b = engraveToolpath(plate, vBit("V-bit 90", 90.0, 4), std::nullopt, offset);
    REQUIRE(b.ok());
    CHECK(b.diagnostics.empty());
    out.push_back(b.toolpath);

    EngraveParams raster = offset;
    raster.name = "Fill raster";
    raster.depth = 0.3;
    raster.passes = 2;
    raster.fillPattern = PocketStrategy::Raster;
    raster.stepover = 0.0;  // the cutting data's 0.4 mm
    raster.rasterAngle = 30.0 * geometry::kPi / 180.0;
    const std::vector<Contour> box{Contour::rectangle(Box2{Vec2{60, 30}, Vec2{70, 36}})};
    const EngraveResult c = engraveToolpath(box, endMill1mm(), std::nullopt, raster);
    REQUIRE(c.ok());
    CHECK(c.diagnostics.empty());
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
};

const std::vector<ShippedPost> kPosts{{"grbl_mm", false},     {"grbl_inch", true},     {"mach3_mm", false},
                                      {"mach3_inch", true},   {"mach4_mm", false},     {"linuxcnc_mm", false},
                                      {"linuxcnc_inch", true}, {"marlin_mm", false}};

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
    const double floorZ = -kDeepest / unit - (p.inch ? 1e-4 : 1e-3);  // one output digit of rounding
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
    std::size_t cuts = 0;
    std::optional<double> x;
    std::optional<double> y;
    std::optional<double> z;
    std::istringstream in(program);
    std::string line;
    std::size_t n = 0;
    while (std::getline(in, line)) {
        ++n;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const std::string where = "line " + std::to_string(n) + " '" + line + "': ";
        std::map<char, double> axis;
        bool toolChange = false;
        bool nonMotion = false;
        for (const Word& w : words(line)) {
            if (w.letter == 'G') {
                const int g = static_cast<int>(w.value);
                if (w.text == "20" || w.text == "21") unitsSet = true;
                if (w.text == "90") absolute = true;
                if (w.text == "91") absolute = false;
                if (g >= 0 && g <= 3 && w.text.find('.') == std::string::npos) motion = g;
                if (w.text == "4" || w.text == "28" || w.text == "53") nonMotion = true;
                if (w.text == "43") z.reset();
            } else if (w.letter == 'M') {
                if (w.text == "3" || w.text == "4") spindleOn = true;
                if (w.text == "5") spindleOn = false;
                if (w.text == "0" || w.text == "6") toolChange = true;
            } else if (w.letter == 'S') {
                speed = w.value;
            } else if (w.letter == 'X' || w.letter == 'Y' || w.letter == 'Z') {
                axis[w.letter] = w.value;
            }
        }
        if (toolChange) {
            if (spindleOn) out.push_back(where + "tool change with the spindle on");
            if (z && *z < safeZ - 1e-9) out.push_back(where + "tool change below safe Z");
            z.reset();
        }
        if (axis.empty() || nonMotion) continue;
        if (!unitsSet || !absolute) out.push_back(where + "motion before the units code or G90");
        const std::optional<double> nx = axis.count('X') ? std::optional<double>(axis['X']) : x;
        const std::optional<double> ny = axis.count('Y') ? std::optional<double>(axis['Y']) : y;
        const std::optional<double> nz = axis.count('Z') ? std::optional<double>(axis['Z']) : z;
        const bool xy = (axis.count('X') && (!x || *x != axis['X'])) || (axis.count('Y') && (!y || *y != axis['Y']));
        if (motion == 0) {
            if (xy && (!z || *z < safeZ - 1e-9 || !nz || *nz < safeZ - 1e-9)) {
                out.push_back(where + "XY rapid while the tool is not known to be at safe Z");
            }
            if (nz && *nz < safeZ - 1e-9) {
                out.push_back(where + "rapid below safe Z");
            }
        } else if (motion >= 1) {
            ++cuts;
            if (!spindleOn || !(speed > 0.0)) out.push_back(where + "cutting with the spindle off");
            if (!x || !y || !z) out.push_back(where + "cutting from an unknown position");
            if (nz && *nz < floorZ) out.push_back(where + "below the engrave depth");
        } else {
            out.push_back(where + "motion without a motion mode");
        }
        x = nx;
        y = ny;
        z = nz;
    }
    if (cuts == 0) out.push_back("no cutting moves");
    if (spindleOn) out.push_back("program ends with the spindle on");
    if (!z || *z < safeZ - 1e-9) out.push_back("program does not end at safe Z");
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

const std::string kGolden = std::string(CARVETTO_GOLDEN_DIR) + "/toolpath/engrave_grbl.nc";

}  // namespace

TEST_CASE("AC1: quick engraving through every shipped post passes the safety rules",
          "[toolpath][engrave][post][safety]") {
    const std::vector<Toolpath> job = engraveJob();
    for (const ShippedPost& p : kPosts) {
        INFO(p.stem);
        const std::string program = run(p.stem, job, "engrave");
        for (const std::string& problem : problems(p, program)) {
            UNSCOPED_INFO(problem);
        }
        CHECK(problems(p, program).empty());
    }
}

TEST_CASE("AC1: the safety rules catch engraving hazards", "[toolpath][engrave][post][safety]") {
    const ShippedPost& grbl = kPosts.front();
    const std::string ok = run(grbl.stem, engraveJob(), "engrave");
    REQUIRE(problems(grbl, ok).empty());
    auto replaced = [&](const std::string& from, const std::string& to) {
        std::string s = ok;
        const std::size_t at = s.find(from);
        REQUIRE(at != std::string::npos);
        s.replace(at, from.size(), to);
        return s;
    };
    CHECK_FALSE(problems(grbl, replaced("G1 Z-0.300 F500", "G0 Z-0.300")).empty());  // rapid into the material
    CHECK_FALSE(problems(grbl, replaced("G1 Z-0.600", "G1 Z-0.700")).empty());       // below the depth
    CHECK_FALSE(problems(grbl, replaced("M3 S18000", "M3 S0")).empty());             // spindle speed 0
    CHECK_FALSE(problems(grbl, replaced("M3 S18000", "M5")).empty());                // spindle off
}

TEST_CASE("AC1: the quick engrave fixture through the Grbl post matches the golden",
          "[toolpath][engrave][post][golden]") {
    CHECK(run("grbl_mm", engraveJob(), "engrave") == readFile(kGolden));
}

TEST_CASE("AC1: the quick engrave golden reads as the fixture says", "[toolpath][engrave][post][golden]") {
    const std::string g = readFile(kGolden);
    CHECK(g.find("(Outline V-bit)") != std::string::npos);
    CHECK(g.find("(Fill offset)") != std::string::npos);
    CHECK(g.find("(Fill raster)") != std::string::npos);
    CHECK(g.find("M3 S18000") != std::string::npos);
    CHECK(g.find("M3 S20000") != std::string::npos);
    CHECK(g.find("G1 Z-0.300 F500") != std::string::npos);  // first outline pass at the plunge rate
    CHECK(g.find("G1 Z-0.600") != std::string::npos);       // second outline pass
    CHECK(g.find("G1 Z-0.500") != std::string::npos);       // offset fill depth
    CHECK(g.find("G1 Z-0.150") != std::string::npos);       // first raster pass
    CHECK((g.find("G2 ") != std::string::npos || g.find("G3 ") != std::string::npos));  // the circle as arcs
    CHECK(g.find("G1 Z-0.700") == std::string::npos);
}

// Hidden (tag [.regen], never run by ctest): rewrites the quick engrave golden. Run via
// tools/regen_golden.py and hand-check the diff against posts/README.md before committing.
TEST_CASE("Regenerate the quick engrave golden", "[.regen]") {
    writeFile(kGolden, run("grbl_mm", engraveJob(), "engrave"));
}
