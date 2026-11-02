#include "carvetto/tooling/tool.hpp"

#include <algorithm>
#include <cmath>

namespace carvetto::tooling {

namespace {

struct TypeName {
    ToolType type;
    std::string_view name;
};

constexpr TypeName kTypeNames[] = {
    {ToolType::EndMill, "end_mill"},
    {ToolType::BallNose, "ball_nose"},
    {ToolType::VBit, "v_bit"},
    {ToolType::Engraving, "engraving"},
    {ToolType::TaperedBallNose, "tapered_ball_nose"},
    {ToolType::RadiusedEndMill, "radiused_end_mill"},
    {ToolType::FormTool, "form_tool"},
    {ToolType::DiamondDrag, "diamond_drag"},
    {ToolType::Drill, "drill"},
};

/// Chord width of a circle of radius r at `depth` below its lowest point (0 when depth <= 0,
/// the full diameter once depth >= r).
double sphereChord(double r, double depth) {
    if (depth <= 0.0 || r <= 0.0) return 0.0;
    if (depth >= r) return 2.0 * r;
    return 2.0 * std::sqrt(2.0 * r * depth - depth * depth);
}

/// Width of a V of included angle `angle` at `depth` (0 at the tip).
double vWidth(double angle, double depth) {
    if (depth <= 0.0) return 0.0;
    return 2.0 * depth * std::tan(angle * 0.5);
}

/// Radius of a form-tool profile at `height` above the tip (linear interpolation; clamped).
double profileRadiusAt(const std::vector<geometry::Vec2>& profile, double height) {
    if (profile.empty()) return 0.0;
    if (height <= profile.front().y) return profile.front().x;
    for (std::size_t i = 1; i < profile.size(); ++i) {
        const geometry::Vec2 a = profile[i - 1];
        const geometry::Vec2 b = profile[i];
        if (height <= b.y) {
            const double span = b.y - a.y;
            if (span <= 0.0) return b.x;
            const double t = (height - a.y) / span;
            return a.x + (b.x - a.x) * t;
        }
    }
    return profile.back().x;
}

double maxProfileRadius(const std::vector<geometry::Vec2>& profile) {
    double m = 0.0;
    for (const auto& p : profile) m = std::max(m, p.x);
    return m;
}

}  // namespace

std::string_view toString(ToolType type) {
    for (const auto& tn : kTypeNames) {
        if (tn.type == type) return tn.name;
    }
    return "end_mill";
}

std::optional<ToolType> parseToolType(std::string_view text) {
    for (const auto& tn : kTypeNames) {
        if (tn.name == text) return tn.type;
    }
    return std::nullopt;
}

const std::vector<ToolType>& allToolTypes() {
    static const std::vector<ToolType> types = [] {
        std::vector<ToolType> v;
        for (const auto& tn : kTypeNames) v.push_back(tn.type);
        return v;
    }();
    return types;
}

double chipLoad(const CuttingData& data, int flutes) {
    if (flutes <= 0 || data.spindleSpeed <= 0.0) return 0.0;
    return data.feedRate / (data.spindleSpeed * static_cast<double>(flutes));
}

// ------------------------------------------------------------------- Tool

const CuttingData* Tool::findCuttingData(std::string_view machineId, std::string_view materialId) const {
    const auto it = cuttingData.find(CuttingDataKey{std::string(machineId), std::string(materialId)});
    return it == cuttingData.end() ? nullptr : &it->second;
}

CuttingDataLookup Tool::cuttingDataFor(std::string_view machineId, std::string_view materialId) const {
    if (const CuttingData* exact = findCuttingData(machineId, materialId)) return {exact, false};
    return {&defaultCuttingData, true};
}

void Tool::setCuttingData(std::string machineId, std::string materialId, const CuttingData& data) {
    cuttingData[CuttingDataKey{std::move(machineId), std::move(materialId)}] = data;
}

bool Tool::removeCuttingData(std::string_view machineId, std::string_view materialId) {
    return cuttingData.erase(CuttingDataKey{std::string(machineId), std::string(materialId)}) > 0;
}

double cuttingWidthAtDepth(const Tool& tool, double depth) {
    const double d = std::max(0.0, depth);
    const double diameter = std::max(0.0, tool.diameter);
    switch (tool.type) {
        case ToolType::EndMill:
        case ToolType::DiamondDrag:
            return diameter;
        case ToolType::BallNose:
            return sphereChord(diameter * 0.5, d);
        case ToolType::VBit:
            return std::min(diameter, vWidth(tool.includedAngle, d));
        case ToolType::Engraving:
            return std::min(diameter, std::max(0.0, tool.tipDiameter) + vWidth(tool.includedAngle, d));
        case ToolType::TaperedBallNose: {
            const double r = std::max(0.0, tool.tipDiameter) * 0.5;
            const double halfAngle = tool.includedAngle * 0.5;
            const double sinA = std::sin(halfAngle);
            const double cosA = std::cos(halfAngle);
            // The taper is tangent to the ball at height h0 above the tip.
            const double h0 = r * (1.0 - sinA);
            double width;
            if (d <= h0) {
                width = sphereChord(r, d);
            } else {
                width = 2.0 * r * cosA + 2.0 * (d - h0) * std::tan(halfAngle);
            }
            return std::min(diameter, width);
        }
        case ToolType::RadiusedEndMill: {
            const double c = std::clamp(tool.cornerRadius, 0.0, diameter * 0.5);
            const double core = diameter - 2.0 * c;
            return std::min(diameter, core + sphereChord(c, d));
        }
        case ToolType::FormTool:
            return 2.0 * profileRadiusAt(tool.profile, d);
        case ToolType::Drill:
            return std::min(diameter, vWidth(tool.includedAngle, d));
    }
    return diameter;
}

std::optional<std::string> validate(const Tool& tool) {
    using namespace std::string_literals;
    if (tool.flutes < 1) return "flutes must be at least 1"s;
    const bool needsAngle = tool.type == ToolType::VBit || tool.type == ToolType::Engraving ||
                            tool.type == ToolType::TaperedBallNose || tool.type == ToolType::Drill;
    if (needsAngle && !(tool.includedAngle > 0.0 && tool.includedAngle < geometry::kPi)) {
        return "included angle must be in (0, pi) radians"s;
    }
    if (tool.type == ToolType::FormTool) {
        if (tool.profile.size() < 2) return "form tool profile needs at least two points"s;
        if (tool.profile.front().y != 0.0) return "form tool profile must start at height 0"s;
        for (std::size_t i = 0; i < tool.profile.size(); ++i) {
            if (tool.profile[i].x < 0.0) return "form tool profile radii must be >= 0"s;
            if (i > 0 && !(tool.profile[i].y > tool.profile[i - 1].y)) {
                return "form tool profile heights must be strictly ascending"s;
            }
        }
        if (maxProfileRadius(tool.profile) <= 0.0) return "form tool profile must have a positive radius"s;
        return std::nullopt;
    }
    if (!(tool.diameter > 0.0)) return "diameter must be positive"s;
    switch (tool.type) {
        case ToolType::Engraving:
            if (tool.tipDiameter < 0.0 || tool.tipDiameter >= tool.diameter) {
                return "engraving tip diameter must be in [0, diameter)"s;
            }
            break;
        case ToolType::TaperedBallNose:
            if (!(tool.tipDiameter > 0.0) || tool.tipDiameter > tool.diameter) {
                return "tapered ball nose tip diameter must be in (0, diameter]"s;
            }
            break;
        case ToolType::RadiusedEndMill:
            if (!(tool.cornerRadius > 0.0) || tool.cornerRadius > tool.diameter * 0.5) {
                return "corner radius must be in (0, diameter/2]"s;
            }
            break;
        default:
            break;
    }
    return std::nullopt;
}

}  // namespace carvetto::tooling
