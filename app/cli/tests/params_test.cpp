// T-0122: params.json parsing for `carvetto-cli toolpath` (schema: owner decision 2026-11-01).

#include "params.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <algorithm>
#include <ctime>
#include <cstddef>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "carvetto/geometry/vec2.hpp"

using Catch::Approx;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::StartsWith;
using namespace carvetto;
using namespace carvetto::cli;

namespace {

constexpr double kDeg = geometry::kPi / 180.0;

/// Every profile key with a non-default value.
const char* kFullProfile = R"({
  "name": "Full", "side": "inside", "direction": "conventional", "allowance": 0.25,
  "passes": {"start_depth": 1, "cut_depth": 9, "pass_depth": 2.5, "maintain_exact_step": false,
             "last_pass_thickness": 0.5},
  "safe_z": 12, "separate_last_pass": true, "last_pass_allowance": 0.3, "last_pass_reverse": true,
  "tabs": {"enabled": true, "length": 7, "thickness": 1.5, "spacing": "constant_distance", "count": 6,
           "distance": 80, "avoid_corners": true, "three_d": true},
  "ramp": {"type": "zig_zag", "distance": 15, "use_angle": true, "angle": 10, "on_lead_in": true},
  "leads": {"type": "circular", "length": 4, "angle": 30, "radius": 6, "sweep": 60, "lead_out": true, "overcut": 1},
  "order": {"selection_order": true, "left_to_right": true, "bottom_to_top": true, "grid": true},
  "start_at": "closest_on_bounding_box", "start_anchor": "top_right", "start_position": [10, -20],
  "corners": {"sharp_external": true, "sharp_internal": true, "mitre_limit": 3},
  "tolerance": {"distance": 0.002, "angle": 0.001, "chordal": 0.02},
  "tool": {"database": "db.json", "id": "v90", "machine": "m", "material": "w"},
  "layers": ["A", "B"]
})";

/// Every pocket key with a non-default value.
const char* kFullPocket = R"({
  "name": "P", "start_depth": 0.5, "cut_depth": 6, "maintain_exact_step": false, "last_pass_thickness": 0.4,
  "safe_z": 8, "strategy": "raster", "direction": "conventional", "raster_angle": 45, "profile_pass": "first",
  "allowance": 0.1, "ramp_plunge": true, "ramp_distance": 12,
  "tolerance": {"distance": 0.002, "angle": 0.001, "chordal": 0.02},
  "tool": {"id": "end-mill-3mm"}, "layers": ["Pockets"]
})";

std::string errorOf(const std::string& text, bool pocket = false) {
    try {
        if (pocket) {
            parsePocketParams(text);
        } else {
            parseProfileParams(text);
        }
    } catch (const CliError& e) {
        return e.what();
    }
    return "";
}

/// All key paths of a JSON object ("tabs.length", ...), for the help cross-check.
void keyPaths(const nlohmann::json& j, const std::string& prefix, std::vector<std::string>& out) {
    for (auto it = j.begin(); it != j.end(); ++it) {
        out.push_back(it.key());
        if (it->is_object()) {
            keyPaths(*it, prefix + it.key() + ".", out);
        }
    }
}

}  // namespace

TEST_CASE("profile params: every key maps to ProfileParams, angles in degrees", "[cli][params]") {
    const ProfileJob job = parseProfileParams(kFullProfile);
    const toolpath::ProfileParams& p = job.params;
    CHECK(p.name == "Full");
    CHECK(p.side == toolpath::ProfileSide::Inside);
    CHECK(p.direction == toolpath::CutDirection::Conventional);
    CHECK(p.allowance == 0.25);
    CHECK(p.passes.startDepth == 1.0);
    CHECK(p.passes.cutDepth == 9.0);
    CHECK(p.passes.passDepth == 2.5);
    CHECK_FALSE(p.passes.maintainExactStep);
    CHECK(p.passes.lastPassThickness == 0.5);
    CHECK(p.safeZ == 12.0);
    CHECK(p.separateLastPass);
    CHECK(p.lastPassAllowance == 0.3);
    CHECK(p.lastPassReverse);
    CHECK(p.tabs.enabled);
    CHECK(p.tabs.length == 7.0);
    CHECK(p.tabs.thickness == 1.5);
    CHECK(p.tabs.spacing == toolpath::TabSpacing::ConstantDistance);
    CHECK(p.tabs.count == 6);
    CHECK(p.tabs.distance == 80.0);
    CHECK(p.tabs.avoidCorners);
    CHECK(p.tabs.threeD);
    CHECK(p.ramp.type == toolpath::RampType::ZigZag);
    CHECK(p.ramp.distance == 15.0);
    CHECK(p.ramp.useAngle);
    CHECK(p.ramp.angle == Approx(10 * kDeg));
    CHECK(p.ramp.onLeadIn);
    CHECK(p.leads.type == toolpath::LeadType::Circular);
    CHECK(p.leads.length == 4.0);
    CHECK(p.leads.angle == Approx(30 * kDeg));
    CHECK(p.leads.radius == 6.0);
    CHECK(p.leads.sweep == Approx(60 * kDeg));
    CHECK(p.leads.leadOut);
    CHECK(p.leads.overcut == 1.0);
    CHECK(p.order.selectionOrder);
    CHECK(p.order.leftToRight);
    CHECK(p.order.bottomToTop);
    CHECK(p.order.grid);
    CHECK(p.startAt == toolpath::StartAt::ClosestOnBoundingBox);
    CHECK(p.startAnchor == toolpath::BoxAnchor::TopRight);
    CHECK(p.startPosition == geometry::Vec2{10, -20});
    CHECK(p.corners.sharpExternal);
    CHECK(p.corners.sharpInternal);
    CHECK(p.corners.mitreLimit == 3.0);
    CHECK(p.tolerance.distance == 0.002);
    CHECK(p.tolerance.angle == Approx(0.001 * kDeg));
    CHECK(p.tolerance.chordal == 0.02);
    CHECK(job.common.tool.database == "db.json");
    CHECK(job.common.tool.id == "v90");
    CHECK(job.common.tool.machine == "m");
    CHECK(job.common.tool.material == "w");
    CHECK(job.common.layers == std::vector<std::string>{"A", "B"});
}

TEST_CASE("pocket params: every key maps to PocketParams", "[cli][params]") {
    const PocketJob job = parsePocketParams(kFullPocket);
    const toolpath::PocketParams& p = job.params;
    CHECK(p.name == "P");
    CHECK(p.startDepth == 0.5);
    CHECK(p.cutDepth == 6.0);
    CHECK_FALSE(p.maintainExactStep);
    CHECK(p.lastPassThickness == 0.4);
    CHECK(p.safeZ == 8.0);
    CHECK(p.strategy == toolpath::PocketStrategy::Raster);
    CHECK(p.direction == toolpath::CutDirection::Conventional);
    CHECK(p.rasterAngle == Approx(45 * kDeg));
    CHECK(p.profilePass == toolpath::ProfilePass::First);
    CHECK(p.allowance == 0.1);
    CHECK(p.rampPlunge);
    CHECK(p.rampDistance == 12.0);
    CHECK(p.tolerance.chordal == 0.02);
    CHECK(job.common.tool.id == "end-mill-3mm");
    CHECK(job.common.tool.database.empty());
    CHECK(job.common.layers == std::vector<std::string>{"Pockets"});
}

TEST_CASE("omitted keys keep the core defaults", "[cli][params]") {
    const ProfileJob a = parseProfileParams(R"({"tool": {"id": "t"}})");
    toolpath::ProfileParams defaults;
    CHECK(a.params.name == defaults.name);
    CHECK(a.params.tabs == defaults.tabs);
    CHECK(a.params.ramp == defaults.ramp);
    CHECK(a.params.leads == defaults.leads);
    CHECK(a.params.passes == defaults.passes);
    CHECK(a.params.safeZ == defaults.safeZ);
    CHECK_FALSE(a.params.cuttingData.has_value());
    CHECK(a.common.layers.empty());
    const PocketJob b = parsePocketParams(R"({"tool": {"id": "t"}})");
    CHECK(b.params.profilePass == toolpath::PocketParams{}.profilePass);
    CHECK(b.params.rasterAngle == 0.0);
}

TEST_CASE("T-0134: duplicate keys and number overflow are refused", "[cli][params][errors]") {
    CHECK(errorOf(R"({"tool": {"id": "t"}, "safe_z": 5, "safe_z": 50})") == "params: duplicate key 'safe_z'");
    CHECK(errorOf(R"({"tool": {"id": "t", "id": "u"}})") == "params: duplicate key 'tool.id'");
    CHECK(errorOf(R"({"tool": {"id": "t"}, "tabs": {"count": 2, "count": 4}})") ==
          "params: duplicate key 'tabs.count'");
    CHECK(errorOf(R"({"tool": {"id": "t"}, "tolerance": {"distance": 0.01, "distance": 0.01}})", true) ==
          "params: duplicate key 'tolerance.distance'");
    // The same key in different objects is fine.
    CHECK(errorOf(R"({"tool": {"id": "end-mill-6mm"}, "tabs": {"enabled": true}, "passes": {"enabled": true}})")
              .find("duplicate") == std::string::npos);
    for (const char* big : {"1e400", "-1e400", "1e309"}) {
        INFO(big);
        const std::string e = errorOf(std::string(R"({"tool": {"id": "t"}, "safe_z": )") + big + "}");
        CHECK_THAT(e, StartsWith("params: not valid JSON: "));
        CHECK_THAT(e, ContainsSubstring("overflow"));
    }
}

TEST_CASE("T-0134: duplicate keys inside arrays are named with their path", "[cli][params][errors]") {
    CHECK(errorOf(R"({"tool": {"id": "t"}, "layers": [{"a": 1, "a": 2}]})") ==
          "params: duplicate key 'layers[].a'");
    CHECK(errorOf(R"({"tool": {"id": "t"}, "x": {"y": [[{"z": {"k": 1, "k": 2}}]]}})") ==
          "params: duplicate key 'x.y[][].z.k'");
}

TEST_CASE("T-0134: deep nesting is refused early with a clear error", "[cli][params][errors][limits]") {
    const auto nested = [](char open, char close, std::size_t n) {
        std::string s = R"({"tool": {"id": "t"}, "zz": )";
        for (std::size_t i = 0; i < n; ++i) {
            s += open;
            if (open == '{') {
                s += R"("k": )";
            }
        }
        s += "1";
        s.append(n, close);
        return s + "}";
    };
    const std::string tooDeep = "params: objects and arrays are nested more than 32 levels deep";
    // 31 levels under the root object are allowed (the schema then rejects the unknown key)...
    CHECK_THAT(errorOf(nested('[', ']', 31)), ContainsSubstring("unknown key 'zz'"));
    CHECK_THAT(errorOf(nested('{', '}', 31)), ContainsSubstring("unknown key 'zz'"));
    // ...one more is refused, and so is a crafted file of a million levels (about 2 MB), without
    // building anything for the levels past the cap (the T-0134 verifier saw gigabytes otherwise).
    for (std::size_t n : {std::size_t{32}, std::size_t{60'000}, std::size_t{1'000'000}}) {
        INFO(n);
        CHECK(errorOf(nested('[', ']', n)) == tooDeep);
        CHECK(errorOf(nested('{', '}', n)) == tooDeep);
    }
}

namespace {

/// Process CPU time (immune to other processes competing for the cores; on MSVC std::clock() is
/// wall-clock, and the Windows CI runs ctest serially).
double cpuSeconds() {
    return static_cast<double>(std::clock()) / static_cast<double>(CLOCKS_PER_SEC);
}

/// {"tool": …, "x": [{"a": 1}, … n times]} or {"tool": …, "x": {"k0": {}, … n times}}.
std::string wide(bool arrayOfObjects, std::size_t n) {
    std::string s = R"({"tool": {"id": "t"}, "x": )";
    s += arrayOfObjects ? '[' : '{';
    for (std::size_t i = 0; i < n; ++i) {
        s += i == 0 ? "" : ",";
        s += arrayOfObjects ? std::string(R"({"a":1})") : "\"k" + std::to_string(i) + "\":{}";
    }
    s += arrayOfObjects ? ']' : '}';
    return s + "}";
}

/// CPU seconds to run `f`, best of three (the minimum filters out scheduling noise).
template <class F>
double timed(F&& f) {
    double best = 1e300;
    for (int k = 0; k < 3; ++k) {
        const double t0 = cpuSeconds();
        f();
        best = std::min(best, cpuSeconds() - t0);
    }
    return best;
}

}  // namespace

TEST_CASE("T-0134: params parsing stays linear for wide arrays of objects and objects of objects",
          "[cli][params][limits]") {
#ifdef NDEBUG
    const double absolute = 1.0;
#else
    const double absolute = 10.0;
#endif
    for (bool arrayOfObjects : {true, false}) {
        INFO((arrayOfObjects ? "array of objects" : "object of objects"));
        const std::string small = wide(arrayOfObjects, 50'000);
        const std::string large = wide(arrayOfObjects, 100'000);
        std::string e;
        const double tSmall = timed([&] { e = errorOf(small); });
        CHECK_THAT(e, ContainsSubstring("unknown key 'x'"));  // parsed through, then the schema
        const double tLarge = timed([&] { e = errorOf(large); });
        CHECK_THAT(e, ContainsSubstring("unknown key 'x'"));
        // Baseline: the plain JSON parse the checks come on top of.
        std::size_t plainSize = 0;
        const double tPlain = timed([&] { plainSize = nlohmann::json::parse(large).size(); });
        CHECK(plainSize == 2);
        INFO("50k " << tSmall << " s, 100k " << tLarge << " s, plain parse of 100k " << tPlain << " s");
        CHECK(tLarge < absolute);
        // Quadratic work would take 4x when the input doubles (the T-0134 verifier measured
        // 2.7 s at 10k and 10.8 s at 20k with nlohmann's callback parser).
        CHECK(tLarge <= 3.0 * tSmall + 0.02);
        CHECK(tLarge <= 4.0 * tPlain + 0.02);
    }
}

TEST_CASE("every params key is documented in the toolpath help", "[cli][params][help]") {
    const std::string help = paramsHelp();
    for (const char* text : {kFullProfile, kFullPocket}) {
        std::vector<std::string> keys;
        keyPaths(nlohmann::json::parse(text), "", keys);
        for (const std::string& key : keys) {
            INFO(key);
            CHECK_THAT(help, ContainsSubstring("\"" + key + "\""));
        }
    }
}

TEST_CASE("params errors name the key", "[cli][params][errors]") {
    CHECK_THAT(errorOf(R"({"tool": {"id": "t"}, "sid": "inside"})"), ContainsSubstring("unknown key 'sid'"));
    CHECK_THAT(errorOf(R"({"tool": {"id": "t"}, "sid": "inside"})"), ContainsSubstring("side"));  // known keys listed
    CHECK_THAT(errorOf(R"({"tool": {"id": "t"}, "tabs": {"count": 2, "x": 1}})"),
               ContainsSubstring("unknown key 'tabs.x' (known keys in 'tabs': "));
    CHECK_THAT(errorOf(R"({"tool": {"id": "t", "diameter": 6}})"), ContainsSubstring("unknown key 'tool.diameter'"));
    CHECK_THAT(errorOf(R"({"tool": {"id": "t"}, "side": "middle"})"),
               ContainsSubstring("'side' must be one of: outside, inside, on, right, left (got 'middle')"));
    CHECK_THAT(errorOf(R"({"tool": {"id": "t"}, "allowance": "1"})"), ContainsSubstring("'allowance' must be a number"));
    CHECK_THAT(errorOf(R"({"tool": {"id": "t"}, "tabs": {"count": 2.5}})"),
               ContainsSubstring("'tabs.count' must be a whole number"));
    CHECK_THAT(errorOf(R"({"tool": {"id": "t"}, "tabs": {"enabled": 1}})"),
               ContainsSubstring("'tabs.enabled' must be true or false"));
    CHECK_THAT(errorOf(R"({"tool": {"id": "t"}, "passes": 3})"), ContainsSubstring("'passes' must be a JSON object"));
    CHECK_THAT(errorOf(R"({"tool": {"id": "t"}, "start_position": [1]})"),
               ContainsSubstring("'start_position' must be an array of two numbers"));
    CHECK_THAT(errorOf(R"({"tool": {"id": "t"}, "layers": "A"})"), ContainsSubstring("'layers' must be an array"));
    CHECK_THAT(errorOf(R"({"tool": {"id": "t"}, "layers": [1]})"), ContainsSubstring("'layers' must be an array"));
    CHECK_THAT(errorOf(R"({"passes": {"cut_depth": 3}})"), ContainsSubstring("missing key 'tool'"));
    CHECK_THAT(errorOf(R"({"tool": {}})"), ContainsSubstring("missing key 'tool.id'"));
    CHECK_THAT(errorOf(R"({"tool": {"id": "t", "machine": "m"}})"),
               ContainsSubstring("'tool.machine' and 'tool.material' must be given together"));
    CHECK_THAT(errorOf("[1, 2]"), ContainsSubstring("must hold a JSON object"));
    CHECK_THAT(errorOf("{"), ContainsSubstring("not valid JSON"));
    CHECK_THAT(errorOf(R"({"tool": {"id": "t"}, "tabs": {}})", true), ContainsSubstring("unknown key 'tabs'"));
    CHECK_THAT(errorOf(R"({"tool": {"id": "t"}, "strategy": "spiral"})", true),
               ContainsSubstring("'strategy' must be one of: offset, raster"));
    CHECK_THAT(errorOf(R"({"tool": {"id": "t"}, "tabs": {"count": 1e10}})"),
               ContainsSubstring("'tabs.count' must be a whole number"));
    CHECK_THAT(errorOf(R"({"tool": {"id": "t"}, "tabs": {"count": 10000000000}})"),
               ContainsSubstring("'tabs.count' is out of range"));
}

// ------------------------------------------------------------------------------- T-0150 drill

namespace {

/// Every drill key with a non-default value.
const char* kFullDrill = R"({
  "name": "Holes", "start_depth": 0.5, "cut_depth": 12, "safe_z": 8, "peck": true,
  "peck_retract": "above_previous_pass", "retract_gap": 0.4, "dwell_time": 0.75,
  "order": {"selection_order": true, "left_to_right": true, "bottom_to_top": true, "grid": true},
  "start_position": [10, -20],
  "tolerance": {"distance": 0.002, "angle": 0.001, "chordal": 0.02},
  "tool": {"database": "db.json", "id": "drill-6mm", "machine": "m", "material": "w"},
  "layers": ["Holes"]
})";

std::string drillErrorOf(const std::string& text) {
    try {
        parseDrillParams(text);
    } catch (const CliError& e) {
        return e.what();
    }
    return "";
}

}  // namespace

TEST_CASE("drill params: every key maps to DrillParams", "[cli][params][drill]") {
    const DrillJob job = parseDrillParams(kFullDrill);
    const toolpath::DrillParams& p = job.params;
    CHECK(p.name == "Holes");
    CHECK(p.startDepth == 0.5);
    CHECK(p.cutDepth == 12.0);
    CHECK(p.safeZ == 8.0);
    CHECK(p.peck);
    CHECK(p.peckRetract == toolpath::PeckRetract::AbovePreviousPass);
    CHECK(p.retractGap == 0.4);
    CHECK(p.dwellTime == 0.75);
    CHECK(p.order == toolpath::OrderOptions{true, true, true, true});
    CHECK(p.startPosition == geometry::Vec2{10, -20});
    CHECK(p.tolerance.distance == 0.002);
    CHECK(p.tolerance.angle == Approx(0.001 * kDeg));
    CHECK(p.tolerance.chordal == 0.02);
    CHECK(job.common.tool.database == "db.json");
    CHECK(job.common.tool.id == "drill-6mm");
    CHECK(job.common.tool.machine == "m");
    CHECK(job.common.tool.material == "w");
    CHECK(job.common.layers == std::vector<std::string>{"Holes"});

    const DrillJob defaults = parseDrillParams(R"({"tool": {"id": "t"}})");
    CHECK(defaults.params == toolpath::DrillParams{});
    CHECK(parseDrillParams(R"({"tool": {"id": "t"}, "peck_retract": "above_start"})").params.peckRetract ==
          toolpath::PeckRetract::AboveStart);
}

TEST_CASE("drill params: every key is documented in the toolpath help", "[cli][params][drill][help]") {
    const std::string help = paramsHelp();
    std::vector<std::string> keys;
    keyPaths(nlohmann::json::parse(kFullDrill), "", keys);
    for (const std::string& key : keys) {
        INFO(key);
        CHECK_THAT(help, ContainsSubstring("\"" + key + "\""));
    }
    CHECK_THAT(help, ContainsSubstring("above_start"));
    CHECK_THAT(help, ContainsSubstring("above_previous_pass"));
}

TEST_CASE("drill params: unknown keys and bad values are errors naming the key", "[cli][params][drill][errors]") {
    CHECK_THAT(drillErrorOf(R"({"tool": {"id": "t"}, "side": "inside"})"),
               ContainsSubstring("unknown key 'side' (known keys: "));
    CHECK_THAT(drillErrorOf(R"({"tool": {"id": "t"}, "pass_depth": 2})"), ContainsSubstring("unknown key 'pass_depth'"));
    CHECK_THAT(drillErrorOf(R"({"tool": {"id": "t"}, "order": {"nearest": true}})"),
               ContainsSubstring("unknown key 'order.nearest' (known keys in 'order': "));
    CHECK_THAT(drillErrorOf(R"({"tool": {"id": "t"}, "peck_retract": "full"})"),
               ContainsSubstring("'peck_retract' must be one of: above_start, above_previous_pass (got 'full')"));
    CHECK_THAT(drillErrorOf(R"({"tool": {"id": "t"}, "peck": "yes"})"), ContainsSubstring("'peck' must be true or false"));
    CHECK_THAT(drillErrorOf(R"({"tool": {"id": "t"}, "dwell_time": "1"})"),
               ContainsSubstring("'dwell_time' must be a number"));
    CHECK_THAT(drillErrorOf(R"({"tool": {"id": "t"}, "start_position": [1]})"),
               ContainsSubstring("'start_position' must be an array of two numbers"));
    CHECK_THAT(drillErrorOf(R"({"cut_depth": 3})"), ContainsSubstring("missing key 'tool'"));
    CHECK_THAT(drillErrorOf(R"({"tool": {"id": "t"}, "retract_gap": 1, "retract_gap": 2})"),
               ContainsSubstring("duplicate key 'retract_gap'"));
}
