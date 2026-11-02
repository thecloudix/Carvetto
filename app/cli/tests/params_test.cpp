// T-0122: params.json parsing for `carvetto-cli toolpath` (schema: owner decision 2026-11-01).

#include "params.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
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
