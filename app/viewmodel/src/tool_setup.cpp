#include "carvetto/viewmodel/tool_setup.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <system_error>

#include "carvetto/geometry/vec2.hpp"
#include "carvetto/viewmodel/file_session.hpp"

namespace carvetto::viewmodel {

namespace fs = std::filesystem;
using tooling::CuttingData;
using tooling::Machine;
using tooling::Tool;
using tooling::ToolDatabase;
using tooling::ToolType;

namespace {

bool sane(double v) {
    return std::isfinite(v) && std::fabs(v) <= kMaxSetupMagnitude;
}

bool saneNonNegative(double v) {
    return sane(v) && v >= 0.0;
}

std::optional<std::string> checkCuttingData(const CuttingData& d, const std::string& what) {
    const struct {
        double value;
        const char* name;
    } fields[] = {{d.passDepth, "pass depth"},
                  {d.stepover, "stepover"},
                  {d.finalPassStepover, "final pass stepover"},
                  {d.clearancePassStepover, "clearance pass stepover"},
                  {d.spindleSpeed, "spindle speed"},
                  {d.feedRate, "feed rate"},
                  {d.plungeRate, "plunge rate"},
                  {d.maxBurnRate, "maximum burn rate"}};
    for (const auto& f : fields) {
        if (!saneNonNegative(f.value)) {
            return what + ": " + f.name + " must be a number from 0 to " + std::to_string(static_cast<long long>(kMaxSetupMagnitude));
        }
    }
    if (d.toolNumber < 0 || d.toolNumber > 1000000) {
        return what + ": tool number out of range";
    }
    return std::nullopt;
}

std::string idKind(const ToolDatabase& db, const std::string& prefix, int kind) {
    for (int n = 1;; ++n) {
        const std::string id = prefix + "-" + std::to_string(n);
        bool taken = false;
        switch (kind) {
            case 0:
                taken = db.findTool(id) != nullptr;
                break;
            case 1:
                taken = db.findMachine(id) != nullptr;
                break;
            case 2:
                taken = db.findMaterial(id) != nullptr;
                break;
            default:
                taken = db.findGroup(id) != nullptr;
                break;
        }
        if (!taken) {
            return id;
        }
    }
}

}  // namespace

std::optional<std::string> validateMachine(const Machine& m) {
    const std::string what = "machine '" + m.name + "'";
    if (m.name.empty()) {
        return std::string("machine '") + m.id + "' needs a name";
    }
    const struct {
        double value;
        const char* name;
    } fields[] = {{m.accelerationX, "X acceleration"}, {m.accelerationY, "Y acceleration"},
                  {m.accelerationZ, "Z acceleration"}, {m.accelerationAngular, "angular acceleration"},
                  {m.rapidRate, "rapid rate"},         {m.workAreaX, "work area X"},
                  {m.workAreaY, "work area Y"},        {m.workAreaZ, "work area Z"},
                  {m.maxSpindleSpeed, "spindle speed limit"}, {m.maxFeedRate, "feed rate limit"}};
    for (const auto& f : fields) {
        if (!saneNonNegative(f.value)) {
            return what + ": " + f.name + " must be a number from 0 to " + std::to_string(static_cast<long long>(kMaxSetupMagnitude));
        }
    }
    if (!m.defaultPost.empty() &&
        std::find(m.associatedPosts.begin(), m.associatedPosts.end(), m.defaultPost) == m.associatedPosts.end()) {
        return what + ": the default post is not among the associated posts";
    }
    return std::nullopt;
}

std::optional<std::string> validateToolForEdit(const Tool& t) {
    const std::string what = "tool '" + t.name + "'";
    if (t.name.empty()) {
        return std::string("tool '") + t.id + "' needs a name";
    }
    for (const double v : {t.diameter, t.tipDiameter, t.cornerRadius, t.shankDiameter, t.toothHeight,
                           t.toothOffset, t.threadedLength}) {
        if (!saneNonNegative(v)) {
            return what + ": a dimension is not a usable number";
        }
    }
    if (!std::isfinite(t.includedAngle)) {
        return what + ": the angle is not a number";
    }
    for (const geometry::Vec2& p : t.profile) {
        if (!sane(p.x) || !sane(p.y)) {
            return what + ": a profile point is not a usable number";
        }
    }
    if (t.flutes > 1000) {
        return what + ": too many flutes";
    }
    if (const auto problem = tooling::validate(t)) {
        return what + ": " + *problem;
    }
    if (const auto problem = checkCuttingData(t.defaultCuttingData, what + " defaults")) {
        return problem;
    }
    for (const auto& [key, data] : t.cuttingData) {
        if (const auto problem = checkCuttingData(data, what + " (" + key.machineId + " / " + key.materialId + ")")) {
            return problem;
        }
    }
    return std::nullopt;
}

std::optional<std::string> validateSetup(const ToolDatabase& db) {
    for (const auto& [id, m] : db.machines()) {
        if (const auto problem = validateMachine(m)) {
            return problem;
        }
    }
    for (const auto& [id, m] : db.materials()) {
        if (m.name.empty()) {
            return "material '" + id + "' needs a name";
        }
    }
    for (const auto& [id, g] : db.groups()) {
        if (g.name.empty()) {
            return "group '" + id + "' needs a name";
        }
    }
    for (const auto& [id, t] : db.tools()) {
        if (const auto problem = validateToolForEdit(t)) {
            return problem;
        }
    }
    return std::nullopt;
}

tooling::ImportResult readToolDatabaseFile(const fs::path& path) {
    tooling::ImportResult result;
    const std::string shown = pathToUtf8(path);
    std::error_code ec;
    const fs::file_status st = fs::status(path, ec);
    if (ec || !fs::exists(st)) {
        result.error = "'" + shown + "' does not exist";
        return result;
    }
    if (!fs::is_regular_file(st)) {
        result.error = "'" + shown + "' is not a regular file";
        return result;
    }
    const std::uintmax_t size = fs::file_size(path, ec);
    if (ec) {
        result.error = "cannot read the size of '" + shown + "'";
        return result;
    }
    if (size > kMaxSetupFileBytes) {
        result.error = "'" + shown + "' is too large for a tool database (" + std::to_string(size / (1024 * 1024)) +
                       " MiB)";
        return result;
    }
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        result.error = "cannot open '" + shown + "'";
        return result;
    }
    std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (in.bad()) {
        result.error = "cannot read '" + shown + "'";
        return result;
    }
    // A hostile file may grow between the size check and the read; re-check what was read.
    if (text.size() > kMaxSetupFileBytes) {
        result.error = "'" + shown + "' is too large for a tool database";
        return result;
    }
    result = tooling::importToolDatabaseJson(text);
    if (!result.ok) {
        result.error = "'" + shown + "': " + result.error;
        return result;
    }
    if (const auto problem = validateSetup(result.database)) {
        result.ok = false;
        result.error = "'" + shown + "': " + *problem;
        result.database = ToolDatabase{};
    }
    return result;
}

SetupLoadResult loadToolSetup(const fs::path& path, const ToolDatabase& defaults) {
    SetupLoadResult out;
    out.database = defaults;
    std::error_code ec;
    if (!fs::exists(path, ec)) {
        out.status = SetupLoadStatus::NotFound;
        return out;
    }
    tooling::ImportResult r = readToolDatabaseFile(path);
    if (!r.ok) {
        out.status = SetupLoadStatus::Refused;
        out.message = "The tool setup file was not used (" + r.error + "). The built-in tools are shown instead.";
        return out;
    }
    out.status = SetupLoadStatus::Loaded;
    out.database = std::move(r.database);
    return out;
}

bool writeToolDatabaseFile(const fs::path& path, const ToolDatabase& db, std::string& error) {
    if (const auto problem = validateSetup(db)) {
        error = *problem;
        return false;
    }
    const std::string text = tooling::exportToolDatabaseJson(db);
    std::error_code ec;
    if (path.has_parent_path()) {
        fs::create_directories(path.parent_path(), ec);
        if (ec) {
            error = "cannot create the folder '" + pathToUtf8(path.parent_path()) + "': " + ec.message();
            return false;
        }
    }
    fs::path tmp = path;
    tmp += ".tmp";
    {
        std::ofstream file(tmp, std::ios::binary | std::ios::trunc);
        if (!file) {
            error = "cannot open '" + pathToUtf8(tmp) + "' for writing";
            return false;
        }
        file.write(text.data(), static_cast<std::streamsize>(text.size()));
        file.close();
        if (!file) {
            std::error_code ignored;
            fs::remove(tmp, ignored);
            error = "cannot write '" + pathToUtf8(tmp) + "'";
            return false;
        }
    }
    fs::rename(tmp, path, ec);
    if (ec) {
        std::error_code ignored;
        fs::remove(tmp, ignored);
        error = "cannot replace '" + pathToUtf8(path) + "': " + ec.message();
        return false;
    }
    return true;
}

std::string newToolId(const ToolDatabase& db) {
    return idKind(db, "tool", 0);
}
std::string newMachineId(const ToolDatabase& db) {
    return idKind(db, "machine", 1);
}
std::string newMaterialId(const ToolDatabase& db) {
    return idKind(db, "material", 2);
}
std::string newGroupId(const ToolDatabase& db) {
    return idKind(db, "group", 3);
}

std::string copyNameForTool(const ToolDatabase& db, const std::string& base) {
    const auto used = [&db](const std::string& name) {
        return std::any_of(db.tools().begin(), db.tools().end(),
                           [&name](const auto& entry) { return entry.second.name == name; });
    };
    std::string name = base + " copy";
    for (int n = 2; used(name); ++n) {
        name = base + " copy " + std::to_string(n);
    }
    return name;
}

Tool defaultTool(ToolType type) {
    Tool t;
    t.type = type;
    t.diameter = 6.0;
    t.flutes = 2;
    switch (type) {
        case ToolType::VBit:
        case ToolType::Drill:
            t.includedAngle = 60.0 * geometry::kPi / 180.0;
            break;
        case ToolType::Engraving:
            t.includedAngle = 30.0 * geometry::kPi / 180.0;
            t.tipDiameter = 0.1;
            break;
        case ToolType::TaperedBallNose:
            t.includedAngle = 10.0 * geometry::kPi / 180.0;
            t.tipDiameter = 1.0;
            break;
        case ToolType::RadiusedEndMill:
            t.cornerRadius = 1.0;
            break;
        case ToolType::ThreadMill:
            t.diameter = 4.0;
            t.includedAngle = 60.0 * geometry::kPi / 180.0;
            t.shankDiameter = 3.8;
            t.toothHeight = 0.6;
            break;
        case ToolType::FormTool:
            t.profile = {{0.0, 0.0}, {3.0, 6.0}};
            break;
        default:
            break;
    }
    if (type == ToolType::Drill || type == ToolType::DiamondDrag) {
        t.flutes = 1;
    }
    return t;
}

Machine defaultMachine(const std::string& name) {
    Machine m;
    m.name = name;
    return m;
}

ToolFields toolFields(ToolType type) {
    ToolFields f;
    switch (type) {
        case ToolType::VBit:
        case ToolType::Drill:
            f.includedAngle = true;
            break;
        case ToolType::Engraving:
        case ToolType::TaperedBallNose:
            f.includedAngle = true;
            f.tipDiameter = true;
            break;
        case ToolType::RadiusedEndMill:
            f.cornerRadius = true;
            break;
        case ToolType::ThreadMill:
            f.includedAngle = true;
            f.threadMill = true;
            break;
        case ToolType::FormTool:
            f.profile = true;
            f.diameter = false;
            break;
        default:
            break;
    }
    return f;
}

std::string toolTypeLabel(ToolType type) {
    switch (type) {
        case ToolType::EndMill:
            return "End mill";
        case ToolType::BallNose:
            return "Ball nose";
        case ToolType::VBit:
            return "V-bit";
        case ToolType::Engraving:
            return "Engraving";
        case ToolType::TaperedBallNose:
            return "Tapered ball nose";
        case ToolType::RadiusedEndMill:
            return "Radiused end mill";
        case ToolType::FormTool:
            return "Form tool";
        case ToolType::DiamondDrag:
            return "Diamond drag";
        case ToolType::Drill:
            return "Drill";
        case ToolType::ThreadMill:
            return "Thread mill";
    }
    return "Tool";
}

std::vector<ToolpathUse> toolpathsUsingTool(const document::Document& doc, const std::string& toolId) {
    std::vector<ToolpathUse> out;
    for (const document::ToolpathRecord& rec : doc.toolpaths()) {
        if (rec.toolSource.toolId == toolId) {
            out.push_back({rec.id, rec.name});
        }
    }
    return out;
}

const Machine* activeMachine(const ToolDatabase& db, const std::string& preferredId) {
    if (const Machine* m = db.findMachine(preferredId)) {
        return m;
    }
    return db.machines().empty() ? nullptr : &db.machines().begin()->second;
}

}  // namespace carvetto::viewmodel
