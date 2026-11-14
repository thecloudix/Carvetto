#pragma once

#include <atomic>
#include <string>
#include <vector>

#include "carvetto/geometry/polyline.hpp"
#include "carvetto/relief/relief.hpp"

// Modify operations 2 (T-0293, ADR-0069): Slice Model, Add Draft, Split Component, Create Vector
// Boundary, Create Cross Section Vector and Distort Component. Pure functions over the relief core;
// nothing here mutates its inputs. Units are mm and radians.

namespace carvetto::relief {

enum class DeriveStatus {
    Ok,
    EmptyModel,    ///< Every cell is transparent (or nothing was given).
    BadParameter,  ///< Non-finite or out-of-range number (thickness, angle, spacing, ...).
    BadVector,     ///< Vector too short, degenerate or non-finite.
    BadGrid,       ///< Resulting grid invalid or larger than kMaxCells.
    TooMany,       ///< More slices or samples than the bounded work allows.
    Cancelled,     ///< The cancel flag was raised; no result.
};

const char* deriveStatusMessage(DeriveStatus s);

/// Most slices one call may produce, and most samples of one cross section.
inline constexpr std::size_t kMaxSlices = 10000;
inline constexpr std::size_t kMaxProfileSamples = std::size_t{1} << 22;

// ---- Contours of an opaque footprint --------------------------------------------------------

/// Closed contours of the opaque region of `field`, in model coordinates: marching squares on the
/// opaque mask (cell centres are the lattice, edge crossings at the midpoints), outer contours
/// counter-clockwise and holes clockwise. Collinear vertices are removed. Deterministic order
/// (by lowest start key). A cell on the grid border is closed off by an outer empty ring.
std::vector<geometry::Polyline> opaqueContours(const HeightField& field);

// ---- Slice Model ----------------------------------------------------------------------------

struct SliceParams {
    double thickness = 1.0;      ///< Slice thickness, > 0.
    bool fromTop = true;         ///< Bands anchored at the model top (going down) or bottom (going up).
    std::size_t count = 0;       ///< Number of slices; 0 = as many as cover the whole model.
    bool boundaryVectors = false;  ///< Also return the footprint contours of each slice.
};

struct Slice {
    Component component;                       ///< MergeHigh; field = min(h, high) - low, base = low.
    double low = 0.0;
    double high = 0.0;
    std::vector<geometry::Polyline> boundary;  ///< Footprint of the slice, when requested.
};

/// Cuts the (composed) model into horizontal bands of equal thickness, one component per slice,
/// ordered from the anchor outwards. Cells below a band's low edge are transparent in it, cells above
/// its high edge are clamped to it, so merging the slices (MergeHigh, or Add on the clamped
/// remainders) rebuilds the model. With a count that leaves part of the model uncovered, cells below
/// the lowest band (from top) are lost, cells above the highest band (from bottom) are clamped.
DeriveStatus sliceModel(const HeightField& model, const SliceParams& params, const std::atomic<bool>* cancel,
                        std::vector<Slice>& out);

// ---- Add Draft ------------------------------------------------------------------------------

/// Bakes a drafted copy of the model into a new component (Add mode, name "Draft"): material is added
/// at the foot of every wall so no wall is steeper than `minDraftRad` from vertical, i.e. the gradient is at
/// most 1/tan(minDraftRad). Tops are never lowered. Angle in [0, pi/2); 0 copies the model. Only
/// opaque cells change; transparent cells stay transparent and block propagation.
/// The bound holds exactly between neighbours in a 16-point stencil (axes, diagonals, knight moves);
/// measured against the Euclidean distance it may be exceeded by about 3% in the worst direction.
DeriveStatus addDraft(const HeightField& model, double minDraftRad, const std::atomic<bool>* cancel, Component& out);

// ---- Split Component ------------------------------------------------------------------------

/// Splits with one vector. Closed: A is inside, B outside. Open: the end segments are extended
/// to infinity; A is the left side of the vector (direction of travel), B the right; a cell on the line
/// goes to A. Both results keep the grid and all dynamic properties and are named "<name> - A"/" - B"
/// (en dash); cells that belong to the other half are transparent.
DeriveStatus splitComponent(const Component& c, const geometry::Polyline& cutter, Component& a, Component& b);

// ---- Create Vector Boundary -----------------------------------------------------------------

/// Silhouette contours of the union of the visible footprints of `components` (every opaque
/// cell of any of them, sampled on the finest cell size). Several components give one group of
/// contours; holes are clockwise contours.
DeriveStatus vectorBoundary(const std::vector<const Component*>& components, std::vector<geometry::Polyline>& out);

// ---- Create Cross Section Vector ------------------------------------------------------------

/// Profile of the model along the segment a-b as open polylines in (distance along, height)
/// coordinates, one per opaque run. Samples every `spacing` mm along the line plus the end
/// point (bilinear). When `simplifyTol` > 0, vertices within that distance of the chord of their
/// neighbours are dropped (a straight ramp becomes two points). No opaque sample = EmptyModel.
DeriveStatus crossSection(const HeightField& model, Point a, Point b, double spacing, double simplifyTol,
                          std::vector<geometry::Polyline>& out);

// ---- Distort Component ----------------------------------------------------------------------

/// Bounding-box distortion: the field's cell-centre rectangle is mapped bilinearly onto the quad
/// p00 (left-bottom), p10 (right-bottom), p11 (right-top), p01 (left-top), which must be convex and
/// non-degenerate. Heights are unchanged; the result keeps the cell size on the original lattice
/// (origin snapped to it, so an identity quad gives back the same cells) and all properties.
struct Quad {
    Point p00, p10, p11, p01;
};

DeriveStatus distortComponent(const Component& c, const Quad& quad, const std::atomic<bool>* cancel, Component& out);

}  // namespace carvetto::relief
