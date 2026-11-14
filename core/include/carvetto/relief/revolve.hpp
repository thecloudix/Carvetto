#pragma once

#include <atomic>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/relief/relief.hpp"

// Turn and Spin (T-0290, ADR-0067): a relief component from a revolved profile. Units are mm.

namespace carvetto::relief {

enum class RevolveKind {
    Turn,  ///< Revolve about the line from the profile's first to its last point (a lathe part).
    Spin,  ///< Revolve about the vertical axis through the profile's left end (a circular component).
};

enum class RevolveStatus {
    Ok,
    BadGrid,       ///< Invalid or oversized grid / cell size.
    BadProfile,    ///< Fewer than 2 points, non-finite coordinate, too many points, or not monotone.
    Degenerate,    ///< Start equals end (Turn), or the profile has no extent / no radius.
    BadParameter,  ///< Base, centre or exact height not finite, or exact height not reachable.
    Cancelled,     ///< The cancel flag was raised; no result.
};

const char* revolveStatusMessage(RevolveStatus s);

struct RevolveParams {
    RevolveKind kind = RevolveKind::Spin;
    double cellSize = 1.0;
    std::optional<Grid> grid;             ///< Empty: the footprint of the part at `cellSize`.
    Point center;                         ///< Spin: model position of the axis.
    std::optional<double> exactHeight;    ///< Scale to Exact Height: total height = base + raw*scale.
    double baseHeight = 0.0;
    CombineMode mode = CombineMode::Add;
    std::string name;
};

/// Turn: profile points are model positions; the radius at a point is its distance to the axis
/// and the position along the axis must not decrease. Surface height is sqrt(r^2 - w^2) at lateral
/// offset w. Spin: x is the radius from the left end (the first point, the minimum x, which must
/// not decrease), y the height. Where the profile steps (equal position) the larger value wins.
/// Cells outside the part are transparent.
struct Revolve {
    RevolveParams params;
    std::vector<Point> profile;
};

/// Deterministic; `cancel` (may be null) is polled once per row. Exact height rescales the
/// component's shape height so the analytic maximum (not the sampled one) reaches the total.
RevolveStatus buildRevolve(const Revolve& r, const std::atomic<bool>* cancel, Component& out);

}  // namespace carvetto::relief
