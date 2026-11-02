// T-0122: interim toolpath members in .carv (ADR-0019).

#include "toolpath_store.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cstdint>
#include <string>
#include <vector>

#include "params.hpp"

using Catch::Matchers::ContainsSubstring;
using namespace carvetto;
using namespace carvetto::cli;
using toolpath::Move;

namespace {

std::vector<std::uint8_t> bytes(const std::string& s) {
    return {s.begin(), s.end()};
}

StoredToolpath sample() {
    StoredToolpath s;
    s.member = "toolpaths/0001.json";
    s.operation = "profile";
    s.paramsJson = R"({"tool":{"id":"end-mill-6mm"}})";
    s.safeZ = 7.25;
    s.toolpath.name = "Sample";
    s.toolpath.tool = {"End Mill 6 mm", 8, 6.0, 2300.0, 770.0, 18000.0};
    s.toolpath.moves = {
        Move::rapid({0.1, 0.2, 7.25}),
        Move::plunge({0.1, 0.2, -1.0 / 3.0}),
        Move::feed({10.0, 0.2, -1.0 / 3.0}, 1234.5),
        Move::arcCW({20.0, 10.2, -0.5}, {10.0, 10.2}),
        Move::arcCCW({10.0, 20.2, -0.5}, {10.0, 10.2}, 99.0),
        Move::dwell({10.0, 20.2, -0.5}, 1.5),
        Move::retract({10.0, 20.2, 7.25}),
    };
    return s;
}

std::string decodeError(const std::string& text) {
    try {
        decodeToolpath("toolpaths/0003.json", bytes(text));
    } catch (const CliError& e) {
        return e.what();
    }
    return "";
}

}  // namespace

TEST_CASE("toolpath members round-trip exactly", "[cli][store]") {
    const StoredToolpath s = sample();
    const std::vector<std::uint8_t> encoded = encodeToolpath(s);
    const StoredToolpath back = decodeToolpath(s.member, encoded);
    CHECK(back.member == s.member);
    CHECK(back.operation == s.operation);
    CHECK(back.paramsJson == s.paramsJson);
    CHECK(back.safeZ == s.safeZ);
    CHECK(back.toolpath == s.toolpath);  // bit-exact doubles, feed overrides, arcs, dwell
    CHECK(encodeToolpath(back) == encoded);
    const std::string text(encoded.begin(), encoded.end());
    CHECK_THAT(text, ContainsSubstring(R"("format":"carvetto-toolpath","format_version":1)"));
    CHECK_THAT(text, ContainsSubstring(R"(["feed",10.0,0.2,-0.3333333333333333,{"feed":1234.5}])"));
    CHECK_THAT(text, ContainsSubstring(R"(["dwell",10.0,20.2,-0.5,1.5])"));
}

TEST_CASE("malformed toolpath members are reported with the member name", "[cli][store][errors]") {
    const std::string head = R"("format":"carvetto-toolpath","format_version":1,"operation":"profile","name":"n",)"
                             R"("params":{},"safe_z":5,"tool":{"name":"t","tool_number":1,"diameter":6,)"
                             R"("feed_rate":1,"plunge_rate":1,"spindle_speed":1},)";
    CHECK(decodeError("{" + head + R"("moves":[]})").empty());
    CHECK_THAT(decodeError("nope"), ContainsSubstring("toolpath member 'toolpaths/0003.json': not valid JSON"));
    CHECK_THAT(decodeError("[]"), ContainsSubstring("must hold a JSON object"));
    CHECK_THAT(decodeError(R"({"format":"other"})"), ContainsSubstring("is not a Carvetto toolpath"));
    CHECK_THAT(decodeError(R"({"format":"carvetto-toolpath","format_version":2})"),
               ContainsSubstring("unsupported format_version"));
    CHECK_THAT(decodeError("{" + head + R"("moves":[["warp",0,0,0]]})"), ContainsSubstring("unknown kind 'warp'"));
    CHECK_THAT(decodeError("{" + head + R"("moves":[["feed",0,0]]})"), ContainsSubstring("move 1 has 3 elements"));
    CHECK_THAT(decodeError("{" + head + R"("moves":[["arc_cw",0,0,0]]})"), ContainsSubstring("has 4 elements"));
    CHECK_THAT(decodeError("{" + head + R"("moves":[["feed",0,"y",0]]})"), ContainsSubstring("move 1: y must be a number"));
    CHECK_THAT(decodeError("{" + head + R"("moves":[["rapid",0,0,0,{"feed":3}]]})"),
               ContainsSubstring("invalid feed override"));
    CHECK_THAT(decodeError("{" + head + R"("moves":{}})"), ContainsSubstring("'moves' must be an array"));
}

TEST_CASE("T-0134: crafted members with zero or negative tool sizes, feeds or dwell are refused",
          "[cli][store][errors]") {
    const auto member = [](const std::string& diameter, const std::string& feed, const std::string& plunge,
                           const std::string& moves) {
        return R"({"format":"carvetto-toolpath","format_version":1,"operation":"profile","name":"n",)"
               R"("params":{},"safe_z":5,"tool":{"name":"t","tool_number":1,"diameter":)" +
               diameter + R"(,"feed_rate":)" + feed + R"(,"plunge_rate":)" + plunge +
               R"(,"spindle_speed":1},"moves":)" + moves + "}";
    };
    CHECK(decodeError(member("6", "1000", "300", R"([["dwell",0,0,0,0.5],["feed",1,0,0,{"feed":5}]])")).empty());
    // Move numbers are 1-based, as in the post's errors.
    CHECK_THAT(decodeError(member("6", "1000", "300", R"([["feed",1,0,0],["dwell",1,0,0,0]])")),
               ContainsSubstring("toolpath member 'toolpaths/0003.json': move 2: dwell seconds must be greater than zero"));
    for (const char* bad : {"0", "-6", "-0.0", "-1e-300"}) {
        INFO(bad);
        CHECK_THAT(decodeError(member(bad, "1000", "300", "[]")),
                   ContainsSubstring("toolpath member 'toolpaths/0003.json': 'tool.diameter' must be greater than zero"));
        CHECK_THAT(decodeError(member("6", bad, "300", "[]")),
                   ContainsSubstring("'tool.feed_rate' must be greater than zero"));
        CHECK_THAT(decodeError(member("6", "1000", bad, "[]")),
                   ContainsSubstring("'tool.plunge_rate' must be greater than zero"));
        CHECK_THAT(decodeError(member("6", "1000", "300", std::string(R"([["dwell",0,0,0,)") + bad + "]]")),
                   ContainsSubstring("move 1: dwell seconds must be greater than zero"));
        CHECK_THAT(decodeError(member("6", "1000", "300", std::string(R"([["feed",1,0,0,{"feed":)") + bad + "}]]")),
                   ContainsSubstring("move 1: feed override must be greater than zero"));
    }
}

TEST_CASE("toolpath member names are numbered from 0001", "[cli][store]") {
    CHECK(isToolpathMember("toolpaths/0001.json"));
    CHECK(isToolpathMember("toolpaths/9999.json"));
    CHECK_FALSE(isToolpathMember("toolpaths/001.json"));
    CHECK_FALSE(isToolpathMember("toolpaths/00a1.json"));
    CHECK_FALSE(isToolpathMember("toolpaths/0001.bin"));
    CHECK_FALSE(isToolpathMember("other/0001.json"));

    document::CarvExtras e;
    CHECK(nextToolpathMember(e) == "toolpaths/0001.json");
    e = e.withMember("toolpaths/0001.json", {}).withMember("bitmaps/a.png", {}).withMember("toolpaths/zz.bin", {});
    CHECK(nextToolpathMember(e) == "toolpaths/0002.json");
    e = e.withMember("toolpaths/0041.json", {});
    CHECK(nextToolpathMember(e) == "toolpaths/0042.json");
    e = e.withMember("toolpaths/9999.json", {});
    CHECK_THROWS_AS(nextToolpathMember(e), CliError);
}

TEST_CASE("storedToolpaths decodes every toolpath member in name order", "[cli][store]") {
    StoredToolpath a = sample();
    StoredToolpath b = sample();
    b.toolpath.name = "Second";
    const document::CarvExtras e = document::CarvExtras{}
                                       .withMember("toolpaths/0002.json", encodeToolpath(b))
                                       .withMember("toolpaths/0001.json", encodeToolpath(a))
                                       .withMember("notes.txt", bytes("x"));
    const auto stored = storedToolpaths(e);
    REQUIRE(stored.size() == 2);
    CHECK(stored[0].member == "toolpaths/0001.json");
    CHECK(stored[0].toolpath.name == "Sample");
    CHECK(stored[1].toolpath.name == "Second");
}
