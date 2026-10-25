// Golden-file and machine-safety tests for every post shipped in posts/ (T-0110).
//
// AC1: each post parses with no warnings and reproduces core/tests/golden/post/<post>_fixture.nc
//      byte for byte for the shared fixture (post_fixture.hpp).
// AC2: the golden files were hand-reviewed against the checklist in posts/README.md; the same
//      checklist is also enforced mechanically here by a small G-code checker, so a later edit to
//      a post or golden cannot silently break it: units and absolute mode before the first move,
//      no XY rapid unless the tool is known to be at or above safe Z (a tool change or G43 makes
//      Z unknown), spindle on before any cutting move, spindle off before a tool change pause,
//      arcs whose start and end radii agree, dwell words in the controller's unit, and spindle off
//      plus the controller's program end at the end.

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "carvetto/post/post_processor.hpp"
#include "post_fixture.hpp"

using namespace carvetto::post;

namespace {

std::string readFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        FAIL("cannot open " << path);
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

/// What each shipped post is expected to look like.
struct ShippedPost {
    const char* stem;          ///< posts/<stem>.pp, golden <stem>_fixture.nc
    const char* postName;      ///< POST_NAME
    const char* extension;     ///< FILE_EXTENSION
    Units units;               ///< UNITS
    const char* lineEnding;    ///< LINE_ENDING, decoded
    const char* unitsCode;     ///< G20 / G21
    const char* toolChange;    ///< M0 (manual pause) or M6 (tool change)
    const char* programEnd;    ///< M30 / M2, or empty when the controller has none (Marlin)
    const char* dwellHalfSec;  ///< The dwell word the post writes for 0.5 s
    const char* dwellQuarter;  ///< ... and for 0.25 s
};

const std::vector<ShippedPost>& shippedPosts() {
    static const std::vector<ShippedPost> posts{
        {"grbl_mm", "Grbl (mm) (*.nc)", "nc", Units::Millimetres, "\n", "G21", "M0", "M30", "P0.50", "P0.25"},
        {"grbl_inch", "Grbl (inch) (*.nc)", "nc", Units::Inches, "\n", "G20", "M0", "M30", "P0.50", "P0.25"},
        {"mach3_mm", "Mach3 (mm) (*.tap)", "tap", Units::Millimetres, "\r\n", "G21", "M6", "M30", "P0.50",
         "P0.25"},
        {"mach3_inch", "Mach3 (inch) (*.tap)", "tap", Units::Inches, "\r\n", "G20", "M6", "M30", "P0.50",
         "P0.25"},
        {"mach4_mm", "Mach4 (mm) (*.tap)", "tap", Units::Millimetres, "\r\n", "G21", "M6", "M30", "P0.50",
         "P0.25"},
        {"linuxcnc_mm", "LinuxCNC (mm) (*.ngc)", "ngc", Units::Millimetres, "\n", "G21", "M6", "M2", "P0.50",
         "P0.25"},
        {"linuxcnc_inch", "LinuxCNC (inch) (*.ngc)", "ngc", Units::Inches, "\n", "G20", "M6", "M2", "P0.50",
         "P0.25"},
        {"marlin_mm", "Marlin (mm) (*.gcode)", "gcode", Units::Millimetres, "\n", "G21", "M0", "", "P500",
         "P250"},
    };
    return posts;
}

PostProcessor loadPost(const ShippedPost& p) {
    auto r = PostProcessor::parse(readFile(std::string(CARVETTO_POSTS_DIR) + "/" + p.stem + ".pp"));
    if (!r.ok()) {
        FAIL(p.stem << ".pp failed to parse: " << r.error().describe());
    }
    return r.value();
}

// ---- a minimal G-code reader for the safety checks ---------------------------------------------

/// One word of a G-code line, e.g. G1 -> {'G', "1", 1.0}.
struct Word {
    char letter;
    std::string text;
    double value;
};

/// Splits a line into words after removing `( … )` and `;` comments and the line number.
/// Marlin's `M0 <message>` keeps only the M0 word.
std::vector<Word> words(std::string line) {
    std::string code;
    int depth = 0;
    for (char c : line) {
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
        const char letter = tok[0];
        if (letter < 'A' || letter > 'Z' || tok.size() < 2) {
            break;  // message text after M0
        }
        Word w{letter, tok.substr(1), 0.0};
        std::istringstream num(w.text);
        if (!(num >> w.value)) {
            break;
        }
        out.push_back(w);
        if (letter == 'M' && w.text == "0") {
            break;
        }
    }
    if (!out.empty() && out.front().letter == 'N') {
        out.erase(out.begin());
    }
    return out;
}

std::vector<std::string> splitLines(const std::string& text, const std::string& ending) {
    std::vector<std::string> lines;
    std::size_t start = 0;
    while (start < text.size()) {
        const std::size_t end = text.find(ending, start);
        REQUIRE(end != std::string::npos);  // every line, the last included, is terminated
        lines.push_back(text.substr(start, end - start));
        start = end + ending.size();
    }
    return lines;
}

/// Runs the safety checklist over a program; returns the problems found (empty = safe).
std::vector<std::string> safetyProblems(const ShippedPost& p, const std::string& program) {
    std::vector<std::string> problems;
    const bool inch = p.units == Units::Inches;
    const double unit = inch ? 25.4 : 1.0;
    const double safeZ = std::round(carvetto::test::kFixtureSafeZ / unit * 1e4) / 1e4;
    const double arcTol = inch ? 0.0002 : 0.002;

    bool unitsSet = false;
    bool absolute = false;
    bool spindleOn = false;
    double spindleSpeed = 0.0;
    int motion = -1;
    std::optional<double> x;
    std::optional<double> y;
    std::optional<double> z;
    bool ended = false;
    bool cutAlready = false;
    std::vector<std::string> dwells;

    const std::vector<std::string> lines = splitLines(program, p.lineEnding);
    for (std::size_t i = 0; i < lines.size(); ++i) {
        const std::string where = "line " + std::to_string(i + 1) + " '" + lines[i] + "': ";
        const std::vector<Word> ws = words(lines[i]);
        if (ws.empty()) continue;
        if (ended) problems.push_back(where + "code after the program end");

        std::map<char, double> axis;
        bool dwell = false;
        bool toolChange = false;
        for (const Word& w : ws) {
            if (w.letter == 'G') {
                if (w.text == "20" || w.text == "21") {
                    unitsSet = true;
                    if (("G" + w.text) != p.unitsCode) problems.push_back(where + "wrong units code");
                } else if (w.text == "90") {
                    absolute = true;
                } else if (w.text == "91") {
                    absolute = false;
                } else if (w.text == "0" || w.text == "1" || w.text == "2" || w.text == "3") {
                    motion = static_cast<int>(w.value);
                } else if (w.text == "4") {
                    dwell = true;
                } else if (w.text == "43") {
                    z.reset();  // the tool length offset changes what Z means
                }
            } else if (w.letter == 'M') {
                if (w.text == "3" || w.text == "4") {
                    spindleOn = true;
                } else if (w.text == "5") {
                    spindleOn = false;
                } else if (w.text == "0" || w.text == "6") {
                    toolChange = true;
                    if ("M" + w.text != p.toolChange)
                        problems.push_back(where + "unexpected tool change code");
                } else if (w.text == "2" || w.text == "30") {
                    ended = true;
                    if (spindleOn) problems.push_back(where + "program ends with the spindle on");
                }
            } else if (w.letter == 'S') {
                spindleSpeed = w.value;
            } else if (w.letter == 'P' && dwell) {
                dwells.push_back("P" + w.text);
            } else if (w.letter == 'X' || w.letter == 'Y' || w.letter == 'Z' || w.letter == 'I' ||
                       w.letter == 'J') {
                axis[w.letter] = w.value;
            }
        }
        if (toolChange) {
            if (spindleOn) problems.push_back(where + "tool change with the spindle on");
            // Before the first cut the position may be unknown (program start); afterwards the
            // tool must have been lifted to safe Z.
            if ((z && *z < safeZ - 1e-9) || (!z && cutAlready)) {
                problems.push_back(where + "tool change below safe Z");
            }
            z.reset();  // the operator may re-zero Z
        }
        if (axis.empty() || dwell) continue;
        if (!unitsSet) problems.push_back(where + "motion before the units code");
        if (!absolute) problems.push_back(where + "motion before G90");

        const std::optional<double> nx = axis.count('X') ? std::optional<double>(axis['X']) : x;
        const std::optional<double> ny = axis.count('Y') ? std::optional<double>(axis['Y']) : y;
        const std::optional<double> nz = axis.count('Z') ? std::optional<double>(axis['Z']) : z;
        const bool xyChanges =
            (axis.count('X') && (!x || *x != axis['X'])) || (axis.count('Y') && (!y || *y != axis['Y']));
        if (motion == 0) {
            if (xyChanges && (!z || *z < safeZ - 1e-9 || !nz || *nz < safeZ - 1e-9)) {
                problems.push_back(where + "XY rapid while the tool is not known to be at safe Z");
            }
        } else if (motion >= 1) {
            if (!spindleOn || !(spindleSpeed > 0.0))
                problems.push_back(where + "cutting with the spindle off");
            if (!x || !y || !z) problems.push_back(where + "cutting move from an unknown position");
            cutAlready = true;
            if ((motion == 2 || motion == 3) && x && y && nx && ny) {
                const double cx = *x + (axis.count('I') ? axis['I'] : 0.0);
                const double cy = *y + (axis.count('J') ? axis['J'] : 0.0);
                const double r0 = std::hypot(*x - cx, *y - cy);
                const double r1 = std::hypot(*nx - cx, *ny - cy);
                if (std::fabs(r0 - r1) > arcTol) problems.push_back(where + "arc start/end radii differ");
            }
        } else {
            problems.push_back(where + "motion without a motion mode");
        }
        x = nx;
        y = ny;
        z = nz;
    }
    if (spindleOn) problems.push_back("program ends with the spindle on");
    if (p.programEnd[0] != '\0' && !ended) problems.push_back(std::string("missing ") + p.programEnd);
    if (dwells != std::vector<std::string>{p.dwellHalfSec, p.dwellQuarter}) {
        problems.push_back("dwell words differ from the controller's unit convention");
    }
    if (!z || *z < safeZ - 1e-9) problems.push_back("program does not end at safe Z");
    return problems;
}

}  // namespace

TEST_CASE("Shipped posts parse with no warnings and declare their dialect", "[post][shipped]") {
    for (const ShippedPost& p : shippedPosts()) {
        INFO(p.stem);
        const PostProcessor pp = loadPost(p);
        CHECK(pp.warnings().empty());
        CHECK(pp.settings().postName == p.postName);
        CHECK(pp.settings().fileExtension == p.extension);
        CHECK(pp.settings().units == p.units);
        CHECK(pp.settings().lineEnding == p.lineEnding);
        for (const char* b :
             {"HEADER", "FOOTER", "TOOLCHANGE", "SPINDLE_ON", "NEW_SEGMENT", "RAPID_MOVE", "FEED_MOVE",
              "PLUNGE_MOVE", "RETRACT_MOVE", "CW_ARC_MOVE", "CCW_ARC_MOVE", "DWELL_MOVE"}) {
            INFO(b);
            CHECK(pp.hasBlock(b));
        }
    }
}

TEST_CASE("AC1: every shipped post reproduces its golden output for the shared fixture",
          "[post][shipped][golden]") {
    for (const ShippedPost& p : shippedPosts()) {
        INFO(p.stem);
        const std::string golden =
            readFile(std::string(CARVETTO_GOLDEN_DIR) + "/post/" + p.stem + "_fixture.nc");
        const auto r = loadPost(p).run(carvetto::test::fixtureToolpaths(), carvetto::test::fixtureContext());
        REQUIRE(r.ok());
        CHECK(r.value().warnings.empty());
        CHECK(r.value().text == golden);
    }
}

TEST_CASE("AC2: every golden program passes the machine-safety checklist", "[post][shipped][safety]") {
    for (const ShippedPost& p : shippedPosts()) {
        INFO(p.stem);
        const std::string golden =
            readFile(std::string(CARVETTO_GOLDEN_DIR) + "/post/" + p.stem + "_fixture.nc");
        const std::vector<std::string> problems = safetyProblems(p, golden);
        for (const std::string& problem : problems) {
            UNSCOPED_INFO(problem);
        }
        CHECK(problems.empty());
    }
}

TEST_CASE("Safety checker catches the hazards it is meant to catch", "[post][shipped][safety]") {
    const ShippedPost& grbl = shippedPosts().front();
    const std::string good =
        "G90 G21\nG0 Z5.000\nM3 S1000\nG0 X1.000 Y1.000\nG1 Z-1.000 F100\nG0 Z5.000\nG4 P0.50\nG4 P0.25\nM5\n"
        "M30\n";
    CHECK(safetyProblems(grbl, good).empty());

    const auto fails = [&](std::string program) { return !safetyProblems(grbl, program).empty(); };
    // No units code / no absolute mode before motion.
    CHECK(
        fails("G90\nG0 Z5.000\nM3 S1000\nG0 X1.000 Y1.000\nG1 Z-1.000 F100\nG0 Z5.000\nG4 P0.50\nG4 "
              "P0.25\nM5\nM30\n"));
    CHECK(
        fails("G21\nG0 Z5.000\nM3 S1000\nG0 X1.000 Y1.000\nG1 Z-1.000 F100\nG0 Z5.000\nG4 P0.50\nG4 "
              "P0.25\nM5\nM30\n"));
    // XY rapid before reaching safe Z.
    CHECK(
        fails("G90 G21\nM3 S1000\nG0 X1.000 Y1.000\nG0 Z5.000\nG1 Z-1.000 F100\nG0 Z5.000\nG4 P0.50\nG4 "
              "P0.25\nM5\nM30\n"));
    // Plunge with the spindle off.
    CHECK(
        fails("G90 G21\nG0 Z5.000\nG0 X1.000 Y1.000\nG1 Z-1.000 F100\nG0 Z5.000\nG4 P0.50\nG4 P0.25\nM3 "
              "S1000\nM5\nM30\n"));
    // Spindle left on / no program end.
    CHECK(
        fails("G90 G21\nG0 Z5.000\nM3 S1000\nG0 X1.000 Y1.000\nG1 Z-1.000 F100\nG0 Z5.000\nG4 P0.50\nG4 "
              "P0.25\nM30\n"));
    CHECK(
        fails("G90 G21\nG0 Z5.000\nM3 S1000\nG0 X1.000 Y1.000\nG1 Z-1.000 F100\nG0 Z5.000\nG4 P0.50\nG4 "
              "P0.25\nM5\n"));
    // Dwell in the wrong unit.
    CHECK(
        fails("G90 G21\nG0 Z5.000\nM3 S1000\nG0 X1.000 Y1.000\nG1 Z-1.000 F100\nG0 Z5.000\nG4 P500\nG4 "
              "P250\nM5\nM30\n"));
    // XY rapid straight after a tool change pause (Z may have been re-zeroed).
    CHECK(
        fails("G90 G21\nG0 Z5.000\nM3 S1000\nG0 X1.000 Y1.000\nG1 Z-1.000 F100\nG0 Z5.000\nM5\nM0\nM3 "
              "S1000\nG0 X2.000 Y2.000 Z5.000\nG4 P0.50\nG4 P0.25\nM5\nM30\n"));
    // Arc whose end is not on the circle.
    CHECK(
        fails("G90 G21\nG0 Z5.000\nM3 S1000\nG0 X1.000 Y1.000\nG1 Z-1.000 F100\nG2 X3.000 Y1.500 I1.000 "
              "J0.000\nG0 Z5.000\nG4 P0.50\nG4 P0.25\nM5\nM30\n"));
}

// Hidden test (tag [.regen], never run by ctest): rewrites every <post>_fixture.nc golden from the
// current posts. Run through tools/regen_golden.py; the goldens must then be hand-reviewed against
// the checklist in posts/README.md before they are committed.
TEST_CASE("Regenerate shipped-post fixture goldens", "[.regen]") {
    for (const ShippedPost& p : shippedPosts()) {
        INFO(p.stem);
        const auto r = loadPost(p).run(carvetto::test::fixtureToolpaths(), carvetto::test::fixtureContext());
        REQUIRE(r.ok());
        CHECK(r.value().warnings.empty());
        std::ofstream out(std::string(CARVETTO_GOLDEN_DIR) + "/post/" + p.stem + "_fixture.nc",
                          std::ios::binary);
        REQUIRE(out);
        out << r.value().text;
    }
}
