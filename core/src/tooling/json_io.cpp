#include "carvetto/tooling/json_io.hpp"

#include <cmath>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>

#include "carvetto/geometry/vec2.hpp"

namespace carvetto::tooling {

namespace {

using json = nlohmann::json;  // std::map-backed: keys serialise sorted.

constexpr std::string_view kFormatTag = "carvetto-tool-database";
constexpr double kQuantumInverse = 1e6;
constexpr double kRadToDeg = 180.0 / geometry::kPi;
constexpr double kDegToRad = geometry::kPi / 180.0;

/// Rounds to the 1e-6 grid and normalises -0 to +0 so output is stable. Dividing the rounded
/// integer by 1e6 yields the correctly rounded double for that decimal, which is exactly what
/// parsing the printed decimal gives back, so quantise(parse(print(quantise(v)))) is a fixed point.
double quantise(double v) {
    double r = std::round(v * kQuantumInverse) / kQuantumInverse;
    if (r == 0.0) r = 0.0;
    return r;
}

double toDeg(double rad) {
    return quantise(rad * kRadToDeg);
}

// ------------------------------------------------------------------ export

json cuttingDataToJson(const CuttingData& d) {
    return json{
        {"pass_depth_mm", quantise(d.passDepth)},
        {"stepover_mm", quantise(d.stepover)},
        {"final_pass_stepover_mm", quantise(d.finalPassStepover)},
        {"clearance_pass_stepover_mm", quantise(d.clearancePassStepover)},
        {"spindle_rpm", quantise(d.spindleSpeed)},
        {"feed_mm_min", quantise(d.feedRate)},
        {"plunge_mm_min", quantise(d.plungeRate)},
        {"max_burn_rate_mm_min", quantise(d.maxBurnRate)},
        {"tool_number", d.toolNumber},
    };
}

json toolToJson(const Tool& t) {
    json profile = json::array();
    for (const auto& p : t.profile) profile.push_back(json::array({quantise(p.x), quantise(p.y)}));
    json cutting = json::array();
    for (const auto& [key, data] : t.cuttingData) {
        cutting.push_back(json{
            {"machine_id", key.machineId},
            {"material_id", key.materialId},
            {"data", cuttingDataToJson(data)},
        });
    }
    return json{
        {"id", t.id},
        {"name", t.name},
        {"type", std::string(toString(t.type))},
        {"notes", t.notes},
        {"group_id", t.groupId},
        {"diameter_mm", quantise(t.diameter)},
        {"included_angle_deg", toDeg(t.includedAngle)},
        {"tip_diameter_mm", quantise(t.tipDiameter)},
        {"corner_radius_mm", quantise(t.cornerRadius)},
        {"profile_mm", std::move(profile)},
        {"flutes", t.flutes},
        {"default_cutting_data", cuttingDataToJson(t.defaultCuttingData)},
        {"cutting_data", std::move(cutting)},
    };
}

json machineToJson(const Machine& m) {
    return json{
        {"id", m.id},
        {"name", m.name},
        {"acceleration_mm_s2",
         json::array({quantise(m.accelerationX), quantise(m.accelerationY), quantise(m.accelerationZ)})},
        {"acceleration_angular_deg_s2", toDeg(m.accelerationAngular)},
        {"associated_posts", m.associatedPosts},
        {"default_post", m.defaultPost},
    };
}

// ------------------------------------------------------------------ import

struct ImportError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

[[noreturn]] void fail(const std::string& where, const std::string& what) {
    throw ImportError(where + ": " + what);
}

const json& require(const json& obj, const char* key, const std::string& where) {
    const auto it = obj.find(key);
    if (it == obj.end()) fail(where, std::string("missing key '") + key + "'");
    return *it;
}

std::string getString(const json& obj, const char* key, const std::string& where, bool required) {
    const auto it = obj.find(key);
    if (it == obj.end()) {
        if (required) fail(where, std::string("missing key '") + key + "'");
        return {};
    }
    if (!it->is_string()) fail(where, std::string("'") + key + "' must be a string");
    return it->get<std::string>();
}

double getNumber(const json& obj, const char* key, const std::string& where, double fallback = 0.0) {
    const auto it = obj.find(key);
    if (it == obj.end()) return fallback;
    if (!it->is_number()) fail(where, std::string("'") + key + "' must be a number");
    return it->get<double>();
}

int getInt(const json& obj, const char* key, const std::string& where, int fallback) {
    const auto it = obj.find(key);
    if (it == obj.end()) return fallback;
    if (!it->is_number_integer()) fail(where, std::string("'") + key + "' must be an integer");
    return it->get<int>();
}

const json& requireArray(const json& obj, const char* key, const std::string& where) {
    const json& v = require(obj, key, where);
    if (!v.is_array()) fail(where, std::string("'") + key + "' must be an array");
    return v;
}

const json* optionalArray(const json& obj, const char* key, const std::string& where) {
    const auto it = obj.find(key);
    if (it == obj.end()) return nullptr;
    if (!it->is_array()) fail(where, std::string("'") + key + "' must be an array");
    return &*it;
}

void requireObject(const json& v, const std::string& where) {
    if (!v.is_object()) fail(where, "must be an object");
}

CuttingData cuttingDataFromJson(const json& j, const std::string& where) {
    requireObject(j, where);
    CuttingData d;
    d.passDepth = getNumber(j, "pass_depth_mm", where);
    d.stepover = getNumber(j, "stepover_mm", where);
    d.finalPassStepover = getNumber(j, "final_pass_stepover_mm", where);
    d.clearancePassStepover = getNumber(j, "clearance_pass_stepover_mm", where);
    d.spindleSpeed = getNumber(j, "spindle_rpm", where);
    d.feedRate = getNumber(j, "feed_mm_min", where);
    d.plungeRate = getNumber(j, "plunge_mm_min", where);
    d.maxBurnRate = getNumber(j, "max_burn_rate_mm_min", where);
    d.toolNumber = getInt(j, "tool_number", where, 0);
    return d;
}

Tool toolFromJson(const json& j, const std::string& where) {
    requireObject(j, where);
    Tool t;
    t.id = getString(j, "id", where, true);
    const std::string here = where + " '" + t.id + "'";
    t.name = getString(j, "name", here, false);
    const std::string typeName = getString(j, "type", here, true);
    const auto type = parseToolType(typeName);
    if (!type) fail(here, "unknown tool type '" + typeName + "'");
    t.type = *type;
    t.notes = getString(j, "notes", here, false);
    t.groupId = getString(j, "group_id", here, false);
    t.diameter = getNumber(j, "diameter_mm", here);
    t.includedAngle = getNumber(j, "included_angle_deg", here) * kDegToRad;
    t.tipDiameter = getNumber(j, "tip_diameter_mm", here);
    t.cornerRadius = getNumber(j, "corner_radius_mm", here);
    t.flutes = getInt(j, "flutes", here, 1);
    if (const json* profile = optionalArray(j, "profile_mm", here)) {
        for (const auto& p : *profile) {
            if (!p.is_array() || p.size() != 2 || !p[0].is_number() || !p[1].is_number()) {
                fail(here, "'profile_mm' entries must be [radius, height] number pairs");
            }
            t.profile.emplace_back(p[0].get<double>(), p[1].get<double>());
        }
    }
    if (const auto it = j.find("default_cutting_data"); it != j.end()) {
        t.defaultCuttingData = cuttingDataFromJson(*it, here + " default_cutting_data");
    }
    if (const json* cutting = optionalArray(j, "cutting_data", here)) {
        for (const auto& entry : *cutting) {
            const std::string entryWhere = here + " cutting_data";
            requireObject(entry, entryWhere);
            CuttingDataKey key{getString(entry, "machine_id", entryWhere, true),
                               getString(entry, "material_id", entryWhere, true)};
            if (t.cuttingData.contains(key)) {
                fail(entryWhere, "duplicate entry for machine '" + key.machineId + "', material '" +
                                     key.materialId + "'");
            }
            t.cuttingData[key] = cuttingDataFromJson(require(entry, "data", entryWhere), entryWhere);
        }
    }
    if (const auto problem = validate(t)) fail(here, *problem);
    return t;
}

Machine machineFromJson(const json& j, const std::string& where) {
    requireObject(j, where);
    Machine m;
    m.id = getString(j, "id", where, true);
    const std::string here = where + " '" + m.id + "'";
    m.name = getString(j, "name", here, false);
    if (const json* accel = optionalArray(j, "acceleration_mm_s2", here)) {
        if (accel->size() != 3 || !(*accel)[0].is_number() || !(*accel)[1].is_number() ||
            !(*accel)[2].is_number()) {
            fail(here, "'acceleration_mm_s2' must be three numbers");
        }
        m.accelerationX = (*accel)[0].get<double>();
        m.accelerationY = (*accel)[1].get<double>();
        m.accelerationZ = (*accel)[2].get<double>();
    }
    m.accelerationAngular = getNumber(j, "acceleration_angular_deg_s2", here) * kDegToRad;
    if (const json* posts = optionalArray(j, "associated_posts", here)) {
        for (const auto& p : *posts) {
            if (!p.is_string()) fail(here, "'associated_posts' entries must be strings");
            m.associatedPosts.push_back(p.get<std::string>());
        }
    }
    m.defaultPost = getString(j, "default_post", here, false);
    return m;
}

ToolDatabase databaseFromJson(const json& root) {
    const std::string where = "document";
    requireObject(root, where);
    const std::string format = getString(root, "format", where, true);
    if (format != kFormatTag) fail(where, "unrecognised format '" + format + "'");
    const int version = getInt(root, "version", where, 0);
    if (version < 1 || version > kToolDatabaseFormatVersion) {
        fail(where, "unsupported version " + std::to_string(version));
    }

    ToolDatabase db;
    // Groups may appear in any order; add parents first by iterating until no progress.
    std::vector<ToolGroup> pending;
    for (const auto& g : requireArray(root, "groups", where)) {
        const std::string gw = "group";
        requireObject(g, gw);
        ToolGroup group;
        group.id = getString(g, "id", gw, true);
        group.name = getString(g, "name", gw + " '" + group.id + "'", false);
        group.parentId = getString(g, "parent_id", gw + " '" + group.id + "'", false);
        pending.push_back(std::move(group));
    }
    while (!pending.empty()) {
        std::vector<ToolGroup> next;
        for (auto& g : pending) {
            if (!db.addGroup(g)) next.push_back(std::move(g));
        }
        if (next.size() == pending.size()) {
            fail("group '" + next.front().id + "'",
                 "duplicate id or unknown parent '" + next.front().parentId + "'");
        }
        pending = std::move(next);
    }
    for (const auto& m : requireArray(root, "materials", where)) {
        const std::string mw = "material";
        requireObject(m, mw);
        Material material;
        material.id = getString(m, "id", mw, true);
        material.name = getString(m, "name", mw + " '" + material.id + "'", false);
        if (!db.addMaterial(material)) fail(mw + " '" + material.id + "'", "duplicate id");
    }
    for (const auto& m : requireArray(root, "machines", where)) {
        Machine machine = machineFromJson(m, "machine");
        if (!db.addMachine(machine)) fail("machine '" + machine.id + "'", "duplicate id");
    }
    for (const auto& t : requireArray(root, "tools", where)) {
        Tool tool = toolFromJson(t, "tool");
        if (!db.addTool(tool))
            fail("tool '" + tool.id + "'", "duplicate id or unknown group '" + tool.groupId + "'");
    }
    return db;
}

}  // namespace

std::string exportToolDatabaseJson(const ToolDatabase& db) {
    json groups = json::array();
    for (const auto& [id, g] : db.groups()) {
        groups.push_back(json{{"id", g.id}, {"name", g.name}, {"parent_id", g.parentId}});
    }
    json materials = json::array();
    for (const auto& [id, m] : db.materials()) {
        materials.push_back(json{{"id", m.id}, {"name", m.name}});
    }
    json machines = json::array();
    for (const auto& [id, m] : db.machines()) machines.push_back(machineToJson(m));
    json tools = json::array();
    for (const auto& [id, t] : db.tools()) tools.push_back(toolToJson(t));

    const json root{
        {"format", std::string(kFormatTag)}, {"version", kToolDatabaseFormatVersion},
        {"groups", std::move(groups)},       {"materials", std::move(materials)},
        {"machines", std::move(machines)},   {"tools", std::move(tools)},
    };
    return root.dump(2) + "\n";
}

ImportResult importToolDatabaseJson(std::string_view text) {
    ImportResult result;
    try {
        const json root = json::parse(text);
        result.database = databaseFromJson(root);
        result.ok = true;
    } catch (const ImportError& e) {
        result.error = e.what();
        result.database = ToolDatabase{};
    } catch (const json::exception& e) {
        result.error = std::string("invalid JSON: ") + e.what();
        result.database = ToolDatabase{};
    }
    return result;
}

}  // namespace carvetto::tooling
