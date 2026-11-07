#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/toolpath/box_index.hpp"
#include "carvetto/viewmodel/view_transform.hpp"

// Qt-free snapping maths (T-0203, ADR-0014). Given a cursor in screen pixels, the snappable
// geometry of a document and the settings, `SnapIndex` returns the snap candidates within a pixel
// tolerance, best first. Everything is computed in millimetres; the view only converts the cursor
// and the tolerance.
//
// Snappable geometry: vector objects on the active sheet whose layer (and every ancestor group's
// layer) is visible and, unless `includeLocked` is set, unlocked: the same notion of "pickable"
// as the selection index (T-0201). Bitmaps have no outline and are not snapped to. The document
// stores objects already transformed, so object transforms need no handling here.
//
// Kinds and their meaning:
//   Endpoint      start/end point of any span.
//   Intersection  crossing or touch of two spans (of one or two objects), as reported by
//                 `geometry::intersect` for the settings' tolerance. The shared junction of two
//                 consecutive spans of one contour is an endpoint, not an intersection. A guide
//                 crossing another guide is a Guide snap.
//   Midpoint      point halfway along the length of a span (arc length, also for Beziers).
//   Centre        centre of an arc span (so of circles and arcs), and the stored centre of
//                 parametric rectangles, circles, ellipses, polygons and stars.
//   Quadrant      the 0/90/180/270 degree points of an arc span that lie on the arc, and the four
//                 axis end points of a parametric ellipse (rotated with it).
//   Perpendicular (needs `pendingStartMm`) a point Q of a span where the segment pending-start to Q
//                 is perpendicular to the span (the foot of the perpendicular). Lines, arcs and
//                 Beziers. Roots of the Bezier condition are found by sampling 32 intervals per span,
//                 so a pair of roots closer than 1/32 of the parameter range can be missed.
//   Tangent       (needs `pendingStartMm`) a point Q of an arc or Bezier where the line pending-start
//                 to Q touches the curve. Lines have none (a line is only tangent to itself).
//   Guide         the point of a user guide line nearest to the cursor, and crossings of two guides.
//   Nearest       the point of any span nearest to the cursor.
//   Grid          the grid node nearest to the cursor.
//
// Priority (documented, deterministic): the candidates within the tolerance are ordered by kind in
// the order listed in `SnapKind` (Endpoint first, Grid last), then by distance to the cursor
// (quantised to 1e-6 px), then by the smallest source object ids, then by position (x, then y)
// and guide index. So a point snap within the tolerance beats a nearer edge snap, and ties never
// depend on container or hash order.
//
// Performance: only spans and points whose boxes touch the cursor's tolerance box are examined
// (a uniform grid, `toolpath::BoxIndex`). At most `kMaxNearSpans` spans (the nearest ones, ties
// by index) take part in the intersection / perpendicular / tangent searches of one query.

namespace carvetto::viewmodel {

/// Snap kinds, in priority order (lowest value wins).
enum class SnapKind : int {
    Endpoint = 0,
    Intersection,
    Midpoint,
    Centre,
    Quadrant,
    Perpendicular,
    Tangent,
    Guide,
    Nearest,
    Grid
};
inline constexpr std::size_t kSnapKindCount = 10;

/// Infinite user guide: the line through `pointMm` with direction `direction` (need not be unit).
struct Guide {
    geometry::Vec2 pointMm{};
    geometry::Vec2 direction{1.0, 0.0};

    static Guide horizontal(double yMm) { return {{0.0, yMm}, {1.0, 0.0}}; }
    static Guide vertical(double xMm) { return {{xMm, 0.0}, {0.0, 1.0}}; }
};

struct SnapSettings {
    /// Pick radius in screen pixels.
    double tolerancePx{10.0};
    /// Per-kind enable flags indexed by `SnapKind`; all kinds are on by default.
    std::array<bool, kSnapKindCount> enabled{true, true, true, true, true, true, true, true, true, true};
    /// Grid pitch in millimetres (<= 0 disables the grid) and the position of one grid node.
    double gridSpacingMm{10.0};
    geometry::Vec2 gridOriginMm{};
    std::vector<Guide> guides;
    /// Tolerances handed to `geometry::intersect` (and used to drop degenerate hits).
    geometry::Tolerance tolerance{};

    bool isEnabled(SnapKind kind) const { return enabled[static_cast<std::size_t>(kind)]; }
    void setEnabled(SnapKind kind, bool on) { enabled[static_cast<std::size_t>(kind)] = on; }
    /// Turns every kind off.
    void disableAll() { enabled.fill(false); }
};

/// What the caller knows about the current interaction.
struct SnapQuery {
    /// Start of the segment being drawn; enables Perpendicular and Tangent.
    std::optional<geometry::Vec2> pendingStartMm;
    /// Objects that never snap (e.g. the ones being dragged), in any order.
    std::vector<document::ObjectId> exclude;
};

struct Snap {
    SnapKind kind{SnapKind::Endpoint};
    geometry::Vec2 pointMm{};
    /// Distance from the cursor to `pointMm`, in screen pixels (<= the tolerance).
    double distancePx{0.0};
    /// Objects the snap comes from, ascending and unique (empty for grid and guides).
    std::vector<document::ObjectId> objects;
    /// Index into `SnapSettings::guides` for a Guide snap (the lower one for a crossing), else -1.
    int guide{-1};
    /// Second guide of a guide crossing, else -1.
    int guide2{-1};
};

/// Spatial index over the snappable geometry of a document's active sheet. Build once per document
/// state; any edit, layer flag change or sheet switch needs a new index. The document must outlive
/// the index and stay unchanged while it is used.
class SnapIndex {
   public:
    static constexpr std::size_t kMaxNearSpans = 256;

    explicit SnapIndex(const document::Document& doc, bool includeLocked = false);

    /// Number of indexed spans / static snap points (endpoints, midpoints, centres, quadrants).
    std::size_t spanCount() const { return spans_.size(); }
    std::size_t pointCount() const { return points_.size(); }

    /// All snap candidates within the tolerance, best first (see the priority rule above).
    /// Candidates of one kind at the same position (within 1e-7 mm) are merged, their objects united.
    std::vector<Snap> candidates(const ViewTransform& view, geometry::Vec2 cursorPx,
                                 const SnapSettings& settings, const SnapQuery& query = {}) const;

    /// The best candidate, or nullopt.
    std::optional<Snap> best(const ViewTransform& view, geometry::Vec2 cursorPx, const SnapSettings& settings,
                             const SnapQuery& query = {}) const;

   private:
    struct SpanEntry {
        document::ObjectId object{};
        const geometry::Contour* contour{nullptr};
        std::size_t spanIndex{0};
    };
    struct PointEntry {
        SnapKind kind{SnapKind::Endpoint};
        geometry::Vec2 point{};
        document::ObjectId object{};
    };
    std::vector<SpanEntry> spans_;
    std::vector<PointEntry> points_;
    toolpath::BoxIndex spanIndex_;
    toolpath::BoxIndex pointIndex_;
};

/// Convenience: builds an index and runs one query. Prefer a retained `SnapIndex`.
std::optional<Snap> snapAt(const document::Document& doc, const ViewTransform& view, geometry::Vec2 cursorPx,
                           const SnapSettings& settings, const SnapQuery& query = {});

}  // namespace carvetto::viewmodel
