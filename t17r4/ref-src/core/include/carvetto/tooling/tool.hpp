#pragma once

#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "carvetto/geometry/vec2.hpp"
#include "carvetto/tooling/cutting_data.hpp"

namespace carvetto::tooling {

/// Kind of cutter. Determines which geometry fields of `Tool` are meaningful and how the
/// cutting width varies with depth.
enum class ToolType {
    EndMill,
    BallNose,
    VBit,
    Engraving,
    TaperedBallNose,
    RadiusedEndMill,
    FormTool,
    DiamondDrag,
    Drill,
};

/// Stable lower-snake identifier used in files, e.g. "ball_nose".
std::string_view toString(ToolType type);
/// Inverse of `toString`; nullopt for an unknown identifier.
std::optional<ToolType> parseToolType(std::string_view text);
/// All tool types in declaration order.
const std::vector<ToolType>& allToolTypes();

/// Result of looking up cutting data for a (machine, material) pair.
struct CuttingDataLookup {
    /// The data to use. Never null when obtained from `Tool::cuttingDataFor`.
    const CuttingData* data{nullptr};
    /// True when no entry existed for the pair and the tool's default data was returned instead.
    bool isFallback{false};
};

/// A cutter definition. Lengths in mm, angles in radians. Which geometry fields apply:
///   EndMill, DiamondDrag: diameter.
///   BallNose:             diameter (ball diameter).
///   VBit:                 diameter, includedAngle.
///   Engraving:            diameter, includedAngle, tipDiameter (flat tip).
///   TaperedBallNose:      diameter (shank/max), includedAngle (taper), tipDiameter (ball diameter).
///   RadiusedEndMill:      diameter, cornerRadius.
///   FormTool:             profile (right-hand half profile), diameter derived from it.
///   Drill:                diameter, includedAngle (point angle).
struct Tool {
    /// Stable unique id (any non-empty string chosen by the creator; never regenerated).
    std::string id;
    std::string name;
    ToolType type{ToolType::EndMill};
    std::string notes;
    /// Id of the containing `ToolGroup`; empty = database root.
    std::string groupId;

    /// Largest cutting diameter (mm).
    double diameter{0.0};
    /// Included angle of V-bits, engraving cutters, tapered ball noses and drill points (radians).
    double includedAngle{0.0};
    /// Flat tip diameter (engraving) or ball diameter (tapered ball nose), mm.
    double tipDiameter{0.0};
    /// Corner radius of a radiused end mill (mm).
    double cornerRadius{0.0};
    /// Form tool half profile: points (radius, height-above-tip) in mm, heights strictly
    /// ascending, first height 0. The width at a height is interpolated linearly.
    std::vector<geometry::Vec2> profile;
    int flutes{1};

    /// Cutting data used when no (machine, material) entry exists.
    CuttingData defaultCuttingData;
    /// Cutting data per (machine, material).
    std::map<CuttingDataKey, CuttingData> cuttingData;

    bool operator==(const Tool&) const = default;

    /// Finds cutting data for the pair, falling back to `defaultCuttingData` and saying so.
    CuttingDataLookup cuttingDataFor(std::string_view machineId, std::string_view materialId) const;
    /// Finds the exact entry for the pair; nullptr when absent (no fallback).
    const CuttingData* findCuttingData(std::string_view machineId, std::string_view materialId) const;
    /// Inserts or replaces the entry for the pair.
    void setCuttingData(std::string machineId, std::string materialId, const CuttingData& data);
    /// Removes the entry for the pair; returns true when one existed.
    bool removeCuttingData(std::string_view machineId, std::string_view materialId);
};

/// Width of the cut (mm) that the tool leaves at `depth` mm below the surface when plunged
/// straight down. Depth is clamped to >= 0; widths never exceed the tool's largest diameter.
///
/// Per type: end mill / diamond drag are constant; V-bit = 2·d·tan(a/2); engraving adds its flat
/// tip; ball nose = chord of a sphere of radius D/2; tapered ball nose follows the ball until it
/// meets the tangent taper; radiused end mill = flat core plus the torus chord; form tool
/// interpolates its profile; drill follows its point angle until full diameter.
double cuttingWidthAtDepth(const Tool& tool, double depth);

/// Checks the geometry fields for the tool's type; returns a message describing the first
/// problem, or nullopt when the tool is well-formed.
std::optional<std::string> validate(const Tool& tool);

}  // namespace carvetto::tooling
