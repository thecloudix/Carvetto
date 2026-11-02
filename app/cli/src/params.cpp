#include "params.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <initializer_list>
#include <ios>
#include <iterator>
#include <limits>
#include <nlohmann/json.hpp>
#include <set>
#include <sstream>
#include <string>
#include <utility>

#include "carvetto/geometry/vec2.hpp"
#include "carvetto/tooling/json_io.hpp"
#include "carvetto/tooling/tool_database.hpp"
#include "starter_tools.hpp"

namespace carvetto::cli {

namespace {

using json = nlohmann::json;

constexpr double kRadPerDeg = geometry::kPi / 180.0;

/// Reads the keys of one JSON object, remembering which ones were asked for, so `finish` can
/// reject every key the schema does not know.
class Fields {
   public:
    Fields(const json& j, std::string path) : j_(j), path_(std::move(path)) {
        if (!j_.is_object()) {
            throw CliError(path_.empty() ? std::string("params: the file must hold a JSON object")
                                         : "params: '" + path_ + "' must be a JSON object");
        }
    }

    /// The value of `key`, or null when absent.
    const json* get(const std::string& key) {
        known_.insert(key);
        const auto it = j_.find(key);
        return it == j_.end() ? nullptr : &*it;
    }

    std::string keyPath(const std::string& key) const { return path_.empty() ? key : path_ + "." + key; }

    [[noreturn]] void fail(const std::string& key, const std::string& what) const {
        throw CliError("params: '" + keyPath(key) + "' " + what);
    }

    void number(const std::string& key, double& out) {
        if (const json* v = get(key)) {
            if (!v->is_number()) {
                fail(key, "must be a number");
            }
            const double d = v->get<double>();
            if (!std::isfinite(d)) {
                fail(key, "must be a finite number");
            }
            out = d;
        }
    }

    /// An angle given in degrees, stored in radians.
    void angle(const std::string& key, double& outRadians) {
        double deg = outRadians / kRadPerDeg;
        number(key, deg);
        if (j_.contains(key)) {
            outRadians = deg * kRadPerDeg;
        }
    }

    void boolean(const std::string& key, bool& out) {
        if (const json* v = get(key)) {
            if (!v->is_boolean()) {
                fail(key, "must be true or false");
            }
            out = v->get<bool>();
        }
    }

    void integer(const std::string& key, int& out) {
        if (const json* v = get(key)) {
            if (!v->is_number_integer()) {
                fail(key, "must be a whole number");
            }
            const auto i = v->get<long long>();
            if (i < std::numeric_limits<int>::min() || i > std::numeric_limits<int>::max()) {
                fail(key, "is out of range");
            }
            out = static_cast<int>(i);
        }
    }

    void string(const std::string& key, std::string& out) {
        if (const json* v = get(key)) {
            if (!v->is_string()) {
                fail(key, "must be a string");
            }
            out = v->get<std::string>();
        }
    }

    template <class E>
    void choice(const std::string& key, E& out, std::initializer_list<std::pair<const char*, E>> options) {
        const json* v = get(key);
        if (!v) {
            return;
        }
        std::string names;
        for (const auto& [name, value] : options) {
            if (v->is_string() && v->get<std::string>() == name) {
                out = value;
                return;
            }
            names += names.empty() ? "" : ", ";
            names += name;
        }
        fail(key, "must be one of: " + names + (v->is_string() ? " (got '" + v->get<std::string>() + "')" : ""));
    }

    /// The nested object at `key`, or null when absent.
    const json* object(const std::string& key) {
        const json* v = get(key);
        if (v && !v->is_object()) {
            fail(key, "must be a JSON object");
        }
        return v;
    }

    /// Rejects keys that were never asked for.
    void finish() const {
        for (auto it = j_.begin(); it != j_.end(); ++it) {
            if (!known_.contains(it.key())) {
                std::string list;
                for (const std::string& k : known_) {
                    list += list.empty() ? "" : ", ";
                    list += k;
                }
                throw CliError("params: unknown key '" + keyPath(it.key()) + "' (known keys" +
                               (path_.empty() ? "" : " in '" + path_ + "'") + ": " + list + ")");
            }
        }
    }

   private:
    const json& j_;
    std::string path_;
    std::set<std::string> known_;
};

json parseJson(std::string_view text) {
    try {
        return json::parse(text.begin(), text.end());
    } catch (const json::parse_error& e) {
        throw CliError(std::string("params: not valid JSON: ") + e.what());
    }
}

void readTolerance(Fields& f, geometry::Tolerance& tol) {
    if (const json* t = f.object("tolerance")) {
        Fields g(*t, f.keyPath("tolerance"));
        g.number("distance", tol.distance);
        g.angle("angle", tol.angle);
        g.number("chordal", tol.chordal);
        g.finish();
    }
}

void readCommon(Fields& f, CommonParams& common) {
    const json* t = f.object("tool");
    if (!t) {
        throw CliError("params: missing key 'tool' (e.g. \"tool\": {\"id\": \"end-mill-6mm\"})");
    }
    Fields g(*t, "tool");
    g.string("database", common.tool.database);
    g.string("id", common.tool.id);
    g.string("machine", common.tool.machine);
    g.string("material", common.tool.material);
    g.finish();
    if (common.tool.id.empty()) {
        throw CliError("params: missing key 'tool.id' (the id of a tool in the tool database)");
    }
    if (common.tool.machine.empty() != common.tool.material.empty()) {
        throw CliError("params: 'tool.machine' and 'tool.material' must be given together");
    }
    if (const json* layers = f.get("layers")) {
        if (!layers->is_array()) {
            f.fail("layers", "must be an array of layer names");
        }
        for (const json& name : *layers) {
            if (!name.is_string()) {
                f.fail("layers", "must be an array of layer names");
            }
            common.layers.push_back(name.get<std::string>());
        }
    }
}

}  // namespace

ProfileJob parseProfileParams(std::string_view text) {
    using namespace toolpath;
    const json root = parseJson(text);
    ProfileJob job;
    ProfileParams& p = job.params;
    Fields f(root, "");
    f.string("name", p.name);
    f.choice("side", p.side,
             {{"outside", ProfileSide::Outside},
              {"inside", ProfileSide::Inside},
              {"on", ProfileSide::On},
              {"right", ProfileSide::Right},
              {"left", ProfileSide::Left}});
    f.choice("direction", p.direction, {{"climb", CutDirection::Climb}, {"conventional", CutDirection::Conventional}});
    f.number("allowance", p.allowance);
    if (const json* j = f.object("passes")) {
        Fields g(*j, "passes");
        g.number("start_depth", p.passes.startDepth);
        g.number("cut_depth", p.passes.cutDepth);
        g.number("pass_depth", p.passes.passDepth);
        g.boolean("maintain_exact_step", p.passes.maintainExactStep);
        g.number("last_pass_thickness", p.passes.lastPassThickness);
        g.finish();
    }
    f.number("safe_z", p.safeZ);
    f.boolean("separate_last_pass", p.separateLastPass);
    f.number("last_pass_allowance", p.lastPassAllowance);
    f.boolean("last_pass_reverse", p.lastPassReverse);
    if (const json* j = f.object("tabs")) {
        Fields g(*j, "tabs");
        g.boolean("enabled", p.tabs.enabled);
        g.number("length", p.tabs.length);
        g.number("thickness", p.tabs.thickness);
        g.choice("spacing", p.tabs.spacing,
                 {{"constant_number", TabSpacing::ConstantNumber}, {"constant_distance", TabSpacing::ConstantDistance}});
        g.integer("count", p.tabs.count);
        g.number("distance", p.tabs.distance);
        g.boolean("avoid_corners", p.tabs.avoidCorners);
        g.boolean("three_d", p.tabs.threeD);
        g.finish();
    }
    if (const json* j = f.object("ramp")) {
        Fields g(*j, "ramp");
        g.choice("type", p.ramp.type,
                 {{"none", RampType::None},
                  {"smooth", RampType::Smooth},
                  {"zig_zag", RampType::ZigZag},
                  {"spiral", RampType::Spiral}});
        g.number("distance", p.ramp.distance);
        g.boolean("use_angle", p.ramp.useAngle);
        g.angle("angle", p.ramp.angle);
        g.boolean("on_lead_in", p.ramp.onLeadIn);
        g.finish();
    }
    if (const json* j = f.object("leads")) {
        Fields g(*j, "leads");
        g.choice("type", p.leads.type,
                 {{"none", LeadType::None}, {"straight", LeadType::Straight}, {"circular", LeadType::Circular}});
        g.number("length", p.leads.length);
        g.angle("angle", p.leads.angle);
        g.number("radius", p.leads.radius);
        g.angle("sweep", p.leads.sweep);
        g.boolean("lead_out", p.leads.leadOut);
        g.number("overcut", p.leads.overcut);
        g.finish();
    }
    if (const json* j = f.object("order")) {
        Fields g(*j, "order");
        g.boolean("selection_order", p.order.selectionOrder);
        g.boolean("left_to_right", p.order.leftToRight);
        g.boolean("bottom_to_top", p.order.bottomToTop);
        g.boolean("grid", p.order.grid);
        g.finish();
    }
    f.choice("start_at", p.startAt,
             {{"vector_start", StartAt::VectorStart},
              {"optimise", StartAt::Optimise},
              {"closest_on_bounding_box", StartAt::ClosestOnBoundingBox}});
    f.choice("start_anchor", p.startAnchor,
             {{"bottom_left", BoxAnchor::BottomLeft},
              {"bottom_centre", BoxAnchor::BottomCentre},
              {"bottom_right", BoxAnchor::BottomRight},
              {"centre_left", BoxAnchor::CentreLeft},
              {"centre", BoxAnchor::Centre},
              {"centre_right", BoxAnchor::CentreRight},
              {"top_left", BoxAnchor::TopLeft},
              {"top_centre", BoxAnchor::TopCentre},
              {"top_right", BoxAnchor::TopRight}});
    if (const json* j = f.get("start_position")) {
        if (!j->is_array() || j->size() != 2 || !(*j)[0].is_number() || !(*j)[1].is_number() ||
            !std::isfinite((*j)[0].get<double>()) || !std::isfinite((*j)[1].get<double>())) {
            f.fail("start_position", "must be an array of two numbers [x, y]");
        }
        p.startPosition = geometry::Vec2{(*j)[0].get<double>(), (*j)[1].get<double>()};
    }
    if (const json* j = f.object("corners")) {
        Fields g(*j, "corners");
        g.boolean("sharp_external", p.corners.sharpExternal);
        g.boolean("sharp_internal", p.corners.sharpInternal);
        g.number("mitre_limit", p.corners.mitreLimit);
        g.finish();
    }
    readTolerance(f, p.tolerance);
    readCommon(f, job.common);
    f.finish();
    return job;
}

PocketJob parsePocketParams(std::string_view text) {
    using namespace toolpath;
    const json root = parseJson(text);
    PocketJob job;
    PocketParams& p = job.params;
    Fields f(root, "");
    f.string("name", p.name);
    f.number("start_depth", p.startDepth);
    f.number("cut_depth", p.cutDepth);
    f.boolean("maintain_exact_step", p.maintainExactStep);
    f.number("last_pass_thickness", p.lastPassThickness);
    f.number("safe_z", p.safeZ);
    f.choice("strategy", p.strategy, {{"offset", PocketStrategy::Offset}, {"raster", PocketStrategy::Raster}});
    f.choice("direction", p.direction, {{"climb", CutDirection::Climb}, {"conventional", CutDirection::Conventional}});
    f.angle("raster_angle", p.rasterAngle);
    f.choice("profile_pass", p.profilePass,
             {{"none", ProfilePass::None}, {"first", ProfilePass::First}, {"last", ProfilePass::Last}});
    f.number("allowance", p.allowance);
    f.boolean("ramp_plunge", p.rampPlunge);
    f.number("ramp_distance", p.rampDistance);
    readTolerance(f, p.tolerance);
    readCommon(f, job.common);
    f.finish();
    return job;
}

ResolvedTool resolveTool(const ToolChoice& choice, const std::filesystem::path& baseDir) {
    std::string text;
    std::string dbName;
    if (choice.database.empty()) {
        text = std::string(starterToolsJson());
        dbName = "the built-in starter tool database";
    } else {
        std::filesystem::path path(choice.database);
        if (path.is_relative()) {
            path = baseDir / path;
        }
        dbName = "tool database '" + path.generic_string() + "'";
        std::ifstream in(path, std::ios::binary);
        if (!in) {
            throw CliError("cannot read " + dbName);
        }
        std::ostringstream ss;
        ss << in.rdbuf();
        text = ss.str();
    }
    tooling::ImportResult db = tooling::importToolDatabaseJson(text);
    if (!db) {
        throw CliError(dbName + ": " + db.error);
    }
    const tooling::Tool* tool = db.database.findTool(choice.id);
    if (!tool) {
        throw CliError(dbName + " has no tool with id '" + choice.id + "'");
    }
    ResolvedTool out;
    out.tool = *tool;
    if (!choice.machine.empty()) {
        if (!db.database.findMachine(choice.machine)) {
            throw CliError(dbName + " has no machine with id '" + choice.machine + "'");
        }
        if (!db.database.findMaterial(choice.material)) {
            throw CliError(dbName + " has no material with id '" + choice.material + "'");
        }
        const tooling::CuttingDataLookup lookup = tool->cuttingDataFor(choice.machine, choice.material);
        if (lookup.isFallback) {
            out.warnings.push_back("tool '" + choice.id + "' has no cutting data for machine '" + choice.machine +
                                   "' and material '" + choice.material + "'; using its default cutting data");
        }
        out.cuttingData = *lookup.data;
    }
    return out;
}

std::string paramsHelp() {
    return R"(params.json reference (lengths in mm, angles in degrees; every key is optional
unless marked "required"; unknown keys are errors):

Both operations:
  "tool": {                      required
    "id": "end-mill-6mm",        required: tool id in the tool database
    "database": "tools.json",    tool database file, relative to the params file
                                 (default: the starter database built into carvetto-cli)
    "machine": "machine-...",    machine id   } choose the tool's cutting data (feeds,
    "material": "material-..."   material id  } speeds, pass depth, stepover); both or neither
  }                              (default: the tool's default cutting data)
  "layers": ["Layer 1", ...]     machine only the vectors on these layers (default: all)
  "name": "..."                  toolpath name
  "safe_z": 5                    absolute Z of moves between cuts
  "direction": "climb"           climb | conventional
  "allowance": 0                 material left on the vectors (profile: negative cuts in)
  "tolerance": {"distance": 0.001, "angle": 5.7e-8, "chordal": 0.01}

profile:
  "side": "outside"              outside | inside | on | right (= outside) | left (= inside)
  "passes": {"start_depth": 0, "cut_depth": 0, "pass_depth": 0,
             "maintain_exact_step": true, "last_pass_thickness": 0}
                                 pass_depth <= 0 takes the cutting data's pass depth
  "separate_last_pass": false, "last_pass_allowance": 0.2, "last_pass_reverse": false
  "tabs": {"enabled": false, "length": 10, "thickness": 2,
           "spacing": "constant_number" | "constant_distance", "count": 4,
           "distance": 100, "avoid_corners": false, "three_d": false}
  "ramp": {"type": "none" | "smooth" | "zig_zag" | "spiral", "distance": 10,
           "use_angle": false, "angle": 5, "on_lead_in": false}
  "leads": {"type": "none" | "straight" | "circular", "length": 5, "angle": 45,
            "radius": 5, "sweep": 90, "lead_out": false, "overcut": 0}
  "order": {"selection_order": false, "left_to_right": false,
            "bottom_to_top": false, "grid": false}
  "start_at": "vector_start"     vector_start | optimise | closest_on_bounding_box
  "start_anchor": "bottom_left"  bottom_left | bottom_centre | bottom_right | centre_left |
                                 centre | centre_right | top_left | top_centre | top_right
  "start_position": [0, 0]       tool XY before the first cut
  "corners": {"sharp_external": false, "sharp_internal": false, "mitre_limit": 2}

pocket:
  "start_depth": 0, "cut_depth": 0 (> 0 required for a toolpath)
  "maintain_exact_step": true, "last_pass_thickness": 0
  "strategy": "offset"           offset | raster
  "raster_angle": 0
  "profile_pass": "last"         none | first | last
  "ramp_plunge": false, "ramp_distance": 10
)";
}

}  // namespace carvetto::cli
