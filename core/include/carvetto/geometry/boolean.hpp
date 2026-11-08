#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/geometry/fixed_point.hpp"
#include "carvetto/geometry/tolerance.hpp"

namespace carvetto::geometry {

/// Boolean operation between a subject set and a clip set of closed contours.
enum class BooleanOp { Union, Difference, Intersection, Xor };

/// Result of a boolean operation. Closed results are polygonal contours of line spans: outers
/// are counter-clockwise (positive area), holes clockwise. Open results come from open subject
/// contours clipped by the operation and keep their direction of travel.
struct BooleanResult {
    std::vector<Contour> closed;
    std::vector<Contour> open;
    /// Input contours (subject or clip) left out because their polygon could not meet `tol.chordal`
    /// within the work bounds (T-0182, ADR-0005 A2: an arc needing more than `kMaxArcSegments`
    /// chords, a Bezier past its split cap, a non-finite span). They contribute nothing, as
    /// degenerate input does; a non-zero count tells the caller the result is incomplete.
    std::size_t refusedContours{0};

    bool isEmpty() const { return closed.empty() && open.empty(); }
};

/// Performs `op` with `subject` against `clip` (Union ignores the subject/clip distinction).
///
/// Approach: every contour is polygonised at `tol.chordal`, scaled by `kFixedPointScale` onto
/// Clipper2's integer grid, clipped with the Vatti-based Clipper2 engine, and converted back.
/// Closed input contours become closed polygons; open input contours in `subject` are clipped
/// as open paths (open contours in `clip` are ignored). Degenerate input (fewer than three
/// distinct vertices for a closed contour) contributes nothing. Arcs are not refitted.
///
/// When many far-apart groups of contours sit side by side (at least 32 groups, judged by their
/// bounding boxes, across one horizontal line), each group is clipped in its own Clipper2 call
/// and the results are concatenated group by group (T-0142, ADR-0006 §10): Clipper2's sweep
/// costs O(n²) for n shapes side by side. Groups never touch, so every winding number and
/// every result region is the same; the result vertices can differ from a single call by a few
/// grid units (Clipper2 rounds intersections per scanbeam) and the order differs. Below that
/// threshold the result is that of one call.
BooleanResult booleanOp(BooleanOp op, const std::vector<Contour>& subject, const std::vector<Contour>& clip,
                        const Tolerance& tol = Tolerance::defaults(), FillRule fill = FillRule::NonZero);

/// Convenience: union of one set of closed contours with itself (resolves overlaps and holes).
std::vector<Contour> unionContours(const std::vector<Contour>& contours,
                                   const Tolerance& tol = Tolerance::defaults(),
                                   FillRule fill = FillRule::NonZero);

/// Clips open contours against one fixed set of closed contours, batch after batch (T-0142,
/// ADR-0006 §10), e.g. the raster lines of a pocket, each batch lying in a narrow strip across
/// the clip.
///
/// `clip(op, subject)` returns the open result of `booleanOp(op, subject, clip, tol, fill)` (no
/// closed result). The clip is polygonised once. When it has at least `kCropMinEdges` edges,
/// each call only gives Clipper2 the clip edges that reach the strip `subject` spans along
/// `across` (from its least to its greatest extent, plus a few grid units), the rest of each
/// ring replaced by straight segments outside the strip, so a call costs about the clip edges
/// in its strip instead of the whole clip (a 90-degree raster on a comb of n teeth was O(n²)).
/// Winding numbers inside the strip are unchanged, so the pieces are the same. Their ends come
/// from the same edges in other scanbeams: where a subject crosses a clip edge at a near-parallel
/// angle an end can move by tens of grid units (the whole-clip call is then the less accurate
/// one), and a stretch lying exactly along a clip edge may be decided the other way (ADR-0006
/// §7). Below the threshold every call is `booleanOp` itself.
class OpenPathClipper {
public:
    /// Clips with this many edges or more are cropped to each call's strip.
    static constexpr std::size_t kCropMinEdges = 1024;

    OpenPathClipper(const std::vector<Contour>& clip, Vec2 across, const Tolerance& tol = Tolerance::defaults(),
                    FillRule fill = FillRule::NonZero);
    ~OpenPathClipper();
    OpenPathClipper(OpenPathClipper&&) noexcept;
    OpenPathClipper& operator=(OpenPathClipper&&) noexcept;
    OpenPathClipper(const OpenPathClipper&) = delete;
    OpenPathClipper& operator=(const OpenPathClipper&) = delete;

    /// The open contours of `subject` clipped by `op` (closed contours of `subject` are ignored).
    std::vector<Contour> clip(BooleanOp op, const std::vector<Contour>& subject) const;
    /// Contours (clip ones at construction, subjects over all `clip` calls) left out for the reason
    /// given at `BooleanResult::refusedContours`. Not thread-safe against concurrent `clip` calls.
    std::size_t refusedContours() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace carvetto::geometry
