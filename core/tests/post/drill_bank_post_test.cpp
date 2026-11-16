// T-0335: the post's drill bank blocks act (ADR-0111): SETUP_DRILL_BANK, DRILL_BANK_PATTERN, DRILL_BANK_EXTEND_DRILL,
// DRILL_BANK_RETRACT_DRILL and FINAL_DRILL_BANK_ACTION with the tags [DB] [DBX] [DBY] [DL] [CD], written from the
// documented sample post (posts/samples/drill_bank_linuxcnc_mm.pp) against a golden file; refusals when the post lacks
// the needed blocks or the toolpath is malformed; ordinary toolpaths are written as before.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/toolpath.hpp"
#include "post_fixture.hpp"

using namespace carvetto;
using post::PostProcessor;
using toolpath::Move;
using toolpath::Toolpath;
using Catch::Matchers::ContainsSubstring;

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

std::string samplePostText() {
    return readFile(std::string(CARVETTO_POSTS_DIR) + "/samples/drill_bank_linuxcnc_mm.pp");
}

PostProcessor parse(const std::string& text) {
    auto r = PostProcessor::parse(text);
    if (!r.ok()) {
        FAIL("post failed to parse: " << r.error().describe());
    }
    return r.value();
}

Move banked(Move m, std::uint64_t set) {
    m.bank = set;
    return m;
}

/// Removes the block `name` (its `begin` .. `end` lines) from a post text.
std::string withoutBlock(std::string text, const std::string& name) {
    const std::size_t begin = text.find("begin " + name + "\n");
    REQUIRE(begin != std::string::npos);
    const std::size_t end = text.find("\nend\n", begin);
    REQUIRE(end != std::string::npos);
    text.erase(begin, end + 5 - begin);
    return text;
}

/// A 3 mm drill with a peck: two strokes, the first with all three spindles of the head, the second with 1 and 3.
Toolpath bankToolpath() {
    Toolpath tp;
    tp.name = "Hinge bank";
    tp.tool = test::fixtureDrill();
    tp.bankHead = {{{-32.0, 0.0}, 3.0, 12.0}, {{0.0, 0.0}, 3.0, 8.0}, {{32.0, 0.0}, 5.0, 10.0}};
    const double s = test::kFixtureSafeZ;
    tp.moves = {banked(Move::rapid({40, 20, s}), 0b111),
                banked(Move::plunge({40, 20, -6}), 0b111),
                banked(Move::dwell({40, 20, -6}, 0.5), 0b111),
                banked(Move::retract({40, 20, -1}), 0b111),
                banked(Move::plunge({40, 20, -12}), 0b111),
                banked(Move::retract({40, 20, s}), 0b111),
                banked(Move::rapid({40, 60, s}), 0b101),
                banked(Move::plunge({40, 60, -10}), 0b101),
                banked(Move::retract({40, 60, s}), 0b101)};
    return tp;
}

std::string write(const PostProcessor& pp, const std::vector<Toolpath>& tps) {
    auto r = pp.run(tps, test::fixtureContext());
    if (!r.ok()) {
        FAIL("post failed: " << r.error().describe());
    }
    CHECK(r.value().warnings.empty());
    return r.value().text;
}

std::size_t lines(const std::string& text, const std::string& part) {
    std::size_t n = 0;
    std::size_t at = 0;
    while ((at = text.find(part, at)) != std::string::npos) {
        ++n;
        at += part.size();
    }
    return n;
}

std::string failure(const PostProcessor& pp, const std::vector<Toolpath>& tps) {
    auto r = pp.run(tps, test::fixtureContext());
    REQUIRE_FALSE(r.ok());
    return r.error().message;
}

}  // namespace

TEST_CASE("T-0335: the sample post writes the drill bank blocks in order (golden)", "[post][drill_bank]") {
    const PostProcessor pp = parse(samplePostText());
    const std::string out = write(pp, {bankToolpath()});
    const std::string golden = readFile(std::string(CARVETTO_GOLDEN_DIR) + "/post/drill_bank_sample.nc");
    CHECK(out == golden);
}

TEST_CASE("T-0335: block counts and order of a stroke", "[post][drill_bank]") {
    const PostProcessor pp = parse(samplePostText());
    const std::string out = write(pp, {bankToolpath()});
    // SETUP once per spindle that drills anywhere (3), PATTERN / EXTEND / RETRACT per spindle per stroke (3 + 2),
    // FINAL once.
    CHECK(lines(out, "M100 ") == 3);
    CHECK(lines(out, "M101 ") == 5);
    CHECK(lines(out, "M102 ") == 5);
    CHECK(lines(out, "M103 ") == 5);
    CHECK(lines(out, "M104") == 1);
    // Order inside the first stroke: rapid to the place, pattern, extend, plunge ... retract, retract blocks.
    const std::size_t rapid = out.find("G0 X40.000 Y20.000");
    REQUIRE(rapid != std::string::npos);
    const std::size_t pattern = out.find("M101 P1", rapid);
    const std::size_t extend = out.find("M102 P1", rapid);
    const std::size_t plunge = out.find("G1 Z-6.000", rapid);
    const std::size_t retract = out.find("M103 P1", rapid);
    CHECK(rapid < pattern);
    CHECK(pattern < extend);
    CHECK(extend < plunge);
    CHECK(plunge < retract);
    // The setup comes before the first move of the toolpath, the final action after the last stroke.
    CHECK(out.find("M100 ") < rapid);
    CHECK(out.rfind("M104") > out.rfind("M103 P3"));
    CHECK(out.find("M104") < out.find("M5\n", out.find("M104")));
    // The second stroke drills with spindles 1 and 3 only.
    const std::size_t second = out.find("G0 Y60.000");
    REQUIRE(second != std::string::npos);
    CHECK(out.find("M101 P2", second) == std::string::npos);
    CHECK(out.find("M101 P3", second) != std::string::npos);
}

TEST_CASE("T-0335: the spindle tags carry the head's data, converted for inch posts", "[post][drill_bank]") {
    const PostProcessor pp = parse(samplePostText());
    const std::string out = write(pp, {bankToolpath()});
    CHECK_THAT(out, ContainsSubstring("M100 P1 I-32.000 J0.000 D3.000 L12.000"));
    CHECK_THAT(out, ContainsSubstring("(Drill bank spindle P3 I32.000 J0.000 D5.000 L10.000)"));
    CHECK_THAT(out, ContainsSubstring("M100 P3 I32.000 J0.000 D5.000 L10.000"));
    CHECK_THAT(out, ContainsSubstring("M102 P2 L8.000"));
    // Inches: the lengths are converted, the number is not.
    std::string inch = samplePostText();
    const std::size_t at = inch.find("UNITS = \"MM\"");
    REQUIRE(at != std::string::npos);
    inch.replace(at, 12, "UNITS = \"INCH\"");
    const std::string out2 = write(parse(inch), {bankToolpath()});
    CHECK_THAT(out2, ContainsSubstring("M100 P3 I1.260 J0.000 D0.197 L0.394"));
    // Outside the blocks the tags are zero.
    std::string lax = samplePostText();
    const std::size_t feed = lax.find("begin FEED_MOVE\n\"");
    REQUIRE(feed != std::string::npos);
    lax.insert(lax.find('\n', feed + 16), "\n\"(outside[DB][DL])\"");
    const std::string out3 = write(parse(lax), {test::fixtureToolpaths()[0]});
    CHECK_THAT(out3, ContainsSubstring("(outside P0 L0.000)"));
}

TEST_CASE("T-0335: a post without the needed blocks refuses a drill bank toolpath", "[post][drill_bank]") {
    for (const char* block : {"DRILL_BANK_PATTERN", "DRILL_BANK_EXTEND_DRILL", "DRILL_BANK_RETRACT_DRILL"}) {
        INFO(block);
        const PostProcessor pp = parse(withoutBlock(samplePostText(), block));
        const std::string message = failure(pp, {bankToolpath()});
        CHECK_THAT(message, ContainsSubstring(std::string("the post has no ") + block + " block"));
        CHECK_THAT(message, ContainsSubstring("Hinge bank"));
    }
    // SETUP and FINAL are optional: without them the blocks are simply not written.
    std::string lean = withoutBlock(samplePostText(), "SETUP_DRILL_BANK");
    lean = withoutBlock(lean, "FINAL_DRILL_BANK_ACTION");
    const std::string out = write(parse(lean), {bankToolpath()});
    CHECK(lines(out, "M100 ") == 0);
    CHECK(lines(out, "M104") == 0);
    CHECK(lines(out, "M101 ") == 5);
    // A shipped post (no bank blocks) refuses too, and still writes ordinary toolpaths.
    const PostProcessor grbl = parse(readFile(std::string(CARVETTO_POSTS_DIR) + "/grbl_mm.pp"));
    CHECK_THAT(failure(grbl, {bankToolpath()}), ContainsSubstring("DRILL_BANK_PATTERN"));
    CHECK(write(grbl, test::fixtureToolpaths()).find("M101") == std::string::npos);
}

TEST_CASE("T-0335: malformed drill bank toolpaths are refused", "[post][drill_bank]") {
    const PostProcessor pp = parse(samplePostText());
    // A spindle the head does not have.
    Toolpath beyond = bankToolpath();
    for (Move& m : beyond.moves) {
        m.bank |= 0b1000;
    }
    CHECK_THAT(failure(pp, {beyond}), ContainsSubstring("does not have"));
    Toolpath noHead = bankToolpath();
    noHead.bankHead.clear();
    CHECK_THAT(failure(pp, {noHead}), ContainsSubstring("does not have"));
    // A stroke must start with its positioning rapid.
    Toolpath noRapid = bankToolpath();
    noRapid.moves.erase(noRapid.moves.begin());
    CHECK_THAT(failure(pp, {noRapid}), ContainsSubstring("starts a stroke but is not a rapid"));
    Toolpath change = bankToolpath();
    change.moves[7].bank = 0b001;  // the set changes in mid-stroke without a rapid
    CHECK_THAT(failure(pp, {change}), ContainsSubstring("move 8 starts a stroke but is not a rapid"));
    // Only rapid, plunge, retract and dwell moves may carry spindles.
    Toolpath feed = bankToolpath();
    feed.moves[1] = banked(Move::feed({40, 20, -6}), 0b111);
    CHECK_THAT(failure(pp, {feed}), ContainsSubstring("is not a rapid, plunge, retract or dwell"));
}

TEST_CASE("T-0335: a rapid with no spindle set inside a job does not end the pattern of the next stroke",
          "[post][drill_bank]") {
    // A detour rapid (no set) between two strokes: each stroke still gets its own blocks.
    const PostProcessor pp = parse(samplePostText());
    Toolpath tp = bankToolpath();
    tp.moves.insert(tp.moves.begin() + 6, Move::rapid({40, 40, test::kFixtureSafeZ}));
    const std::string out = write(pp, {tp});
    CHECK(lines(out, "M101 ") == 5);
    CHECK(lines(out, "M103 ") == 5);
    // The retract blocks of stroke one come before the detour's move.
    CHECK(out.find("M103 P3") < out.find("G0 Y40.000"));
}

TEST_CASE("T-0335: an ordinary toolpath after a drill bank toolpath is written without bank blocks", "[post][drill_bank]") {
    const PostProcessor pp = parse(samplePostText());
    const std::string out = write(pp, {bankToolpath(), test::fixtureToolpaths()[0]});
    CHECK(lines(out, "M104") == 1);
    CHECK(out.find("M104") < out.find("(Profile)"));  // the profile's own segment follows the final action
    CHECK(lines(out, "M101 ") == 5);
}

TEST_CASE("T-0335: SETUP is written for the spindles that drill, not for the whole head", "[post][drill_bank]") {
    const PostProcessor pp = parse(samplePostText());
    Toolpath tp = bankToolpath();
    tp.bankHead.push_back({{64.0, 0.0}, 3.0, 9.0});  // a fourth spindle that no stroke uses
    const std::string out = write(pp, {tp});
    CHECK(lines(out, "M100 ") == 3);
    CHECK(out.find("M100 P4") == std::string::npos);
    CHECK(out.find("M101 P4") == std::string::npos);
}

TEST_CASE("T-0335: the retract blocks come right after the stroke's last move, before a following move without spindles",
          "[post][drill_bank]") {
    const PostProcessor pp = parse(samplePostText());
    Toolpath tp = bankToolpath();
    // A feed move without a spindle set right after the first stroke (not a rapid): the stroke still ends before it.
    tp.moves.insert(tp.moves.begin() + 6, Move::feed({41, 20, test::kFixtureSafeZ}));
    const std::string out = write(pp, {tp});
    const std::size_t feed = out.find("G1 X41.000");
    REQUIRE(feed != std::string::npos);
    CHECK(out.rfind("M103 P3", feed) != std::string::npos);
    CHECK(out.find("M103 P1", feed) > feed);  // the second stroke's retracts come after it
    CHECK(lines(out, "M103 ") == 5);
}

TEST_CASE("T-0335: a drill bank toolpath without spindle sets is posted with a warning", "[post][drill_bank]") {
    const PostProcessor pp = parse(samplePostText());
    Toolpath old = bankToolpath();
    for (Move& m : old.moves) {
        m.bank = 0;
    }
    auto r = pp.run({old}, test::fixtureContext());
    REQUIRE(r.ok());
    REQUIRE(r.value().warnings.size() == 1);
    CHECK_THAT(r.value().warnings[0].message, Catch::Matchers::ContainsSubstring("without spindle sets"));
    CHECK_THAT(r.value().warnings[0].message, Catch::Matchers::ContainsSubstring("Hinge bank"));
    CHECK(lines(r.value().text, "M101 ") == 0);  // no stroke blocks
    // An ordinary toolpath (no head) and a drill bank with sets do not warn.
    auto plain = pp.run(test::fixtureToolpaths(), test::fixtureContext());
    REQUIRE(plain.ok());
    CHECK(plain.value().warnings.empty());
}
