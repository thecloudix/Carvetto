#pragma once

#include <vector>

#include "carvetto/document/job.hpp"
#include "carvetto/document/sheet.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/toolpath/move.hpp"

// Double-sided job maths (T-0243, ADR-0039): how a design on one face of the stock maps onto the
// other face, and where Z = 0 sits for each side. Millimetres; no document state.

namespace carvetto::document {

inline constexpr Side otherSide(Side s) {
    return s == Side::Front ? Side::Back : Side::Front;
}

/// Mirrors `p` about the centre line of `material`: a horizontal flip (the stock is turned over
/// about its vertical axis) negates X about the centre, a vertical flip negates Y. Involution.
geometry::Vec2 flipPoint(const geometry::Box2& material, FlipDirection direction, geometry::Vec2 p);

/// The box covering the mirror image of `box`.
geometry::Box2 flipBox(const geometry::Box2& material, FlipDirection direction, const geometry::Box2& box);

/// Machine Z of the top face of the stock while machining `side`, with the side's Z = 0 where the
/// job puts it. `MaterialSurface`: the face being cut is at 0. `MachineBed`: the face against the
/// bed is at 0, so the cut face is at `thicknessMm`. With `zeroOffSameSide` both sides share one
/// physical zero face, the one the front side used: after the flip it is the bed face (surface
/// zero) or the cut face (bed zero). `CylinderAxis` is treated as the surface.
double materialTopZ(ZZero zero, bool zeroOffSameSide, double thicknessMm, Side side);

/// `moves` with `dz` added to every Z (toolpaths are computed with the cut face at Z = 0).
std::vector<toolpath::Move> shiftMovesZ(const std::vector<toolpath::Move>& moves, double dz);

}  // namespace carvetto::document
