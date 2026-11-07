#pragma once

#include <array>
#include <cstddef>
#include <limits>
#include <optional>

#include "carvetto/tooling/tool.hpp"

// Tool shapes for the material-removal simulation (T-0222, ADR-0032). A shape is a surface of
// revolution described by its height profile h(r): the height above the tool tip of the cutting
// surface at radius r from the axis, for 0 <= r <= radius(). The cutter removes every point of
// the stock above that surface. Pieces are a flat annulus, a sphere cap or a cone; every shape
// built here has a convex, non-decreasing profile (the exact sweep formulas rely on it).

namespace carvetto::simulate {

/// One piece of a tool's height profile over the radial range [r0, r1].
struct ProfilePiece {
    enum class Kind { Flat, Cone, Sphere };
    Kind kind{Kind::Flat};
    double r0{0.0};
    double r1{0.0};
    /// Flat: height of the piece. Cone: height at r0. Sphere: height of the lowest point of the
    /// sphere (its centre is `base + sphereRadius` above the tip).
    double base{0.0};
    /// Cone: dh/dr (= 1 / tan(half angle)).
    double slope{0.0};
    /// Sphere: radius of the sphere.
    double sphereRadius{0.0};

    bool operator==(const ProfilePiece&) const = default;
    /// Profile height at radius r (r is not clamped to [r0, r1]).
    double heightAt(double r) const;
};

/// Kind of cutter shape (for display and tests; the geometry is in the pieces).
enum class ShapeKind { None, FlatEndMill, BallNose, VBit, TaperedBallNose };

/// A tool for the simulator: height profile plus the length of its flutes (the shank above them
/// must not enter material). Build with the factories; a default-constructed shape is invalid.
class ToolShape {
public:
    static constexpr std::size_t kMaxPieces = 3;
    /// Largest accepted tool diameter and smallest accepted included angle.
    static constexpr double kMaxDiameter = 10000.0;
    static constexpr double kMinIncludedAngle = 1e-3;

    ToolShape() = default;

    /// Flat end mill (also a diamond drag point): constant height 0 over the full radius.
    static std::optional<ToolShape> flatEndMill(double diameter);
    /// Ball nose of the given ball diameter.
    static std::optional<ToolShape> ballNose(double diameter);
    /// V-bit: cone of included angle `includedAngle` (radians) up to `diameter`; an optional flat
    /// tip of diameter `tipDiameter` (engraving cutters) in [0, diameter).
    static std::optional<ToolShape> vBit(double diameter, double includedAngle, double tipDiameter = 0.0);
    /// Tapered ball nose: ball of diameter `ballDiameter` continued by the tangent cone of included
    /// angle `includedAngle` up to `diameter` (ADR-0010 geometry, as `tooling::cuttingWidthAtDepth`).
    static std::optional<ToolShape> taperedBallNose(double diameter, double includedAngle, double ballDiameter);
    /// Shape of a tool-database tool: EndMill, DiamondDrag, BallNose, VBit, Engraving,
    /// TaperedBallNose and Drill (its point as a V). nullopt for the other types and for tools that
    /// fail `tooling::validate`. `fluteLength` (mm) is not part of the database tool.
    static std::optional<ToolShape> fromTool(const tooling::Tool& tool,
                                             std::optional<double> fluteLength = std::nullopt);

    bool valid() const { return count_ > 0; }
    ShapeKind kind() const { return kind_; }
    double diameter() const { return 2.0 * radius_; }
    double radius() const { return radius_; }
    /// Length of the cutting flutes measured up from the tip; infinity = unlimited.
    double fluteLength() const { return fluteLength_; }
    /// Returns a copy with the given flute length (mm, > 0, or infinity).
    ToolShape withFluteLength(double length) const;
    std::size_t pieceCount() const { return count_; }
    const ProfilePiece& piece(std::size_t i) const { return pieces_[i]; }
    /// Height of the profile at radius r in [0, radius()]; +infinity beyond the radius.
    double heightAt(double r) const;
    /// Largest radius at which the profile is at most `gap` above the tip (gap >= 0): the part of the
    /// tool that can reach a surface `gap` above its tip. Used to bound the cells a move can touch.
    double reachAtGap(double gap) const;
    /// The cylinder above the flutes: one flat piece at the flute length (for shank collisions).
    ToolShape shank() const;

    bool operator==(const ToolShape&) const = default;

private:
    ShapeKind kind_{ShapeKind::None};
    std::array<ProfilePiece, kMaxPieces> pieces_{};
    std::size_t count_{0};
    double radius_{0.0};
    double fluteLength_{std::numeric_limits<double>::infinity()};
};

}  // namespace carvetto::simulate
