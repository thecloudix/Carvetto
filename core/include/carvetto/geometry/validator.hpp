#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/tolerance.hpp"

/// Vector Validator (T-0249): finds, and for most kinds repairs, problems in the contours of a set
/// of vector objects. Pure geometry: units mm / radians, no document types.
///
/// The kinds, each checked per contour (indices are into the object list / contour list given):
///  - `ZeroLengthSpan`: a span no longer than `tol.distance`. Fix: drop the span.
///  - `TinyContour`: a contour whose bounding box is at most `tinySizeMm` in both directions. Fix:
///    delete the contour (area lost <= tinySizeMm^2).
///  - `OpenContour`: a contour that is not closed. Fix: closed when the end lies within
///    `closeGapMm` of the start (a short bridging line is added when the gap exceeds
///    `tol.distance`); wider gaps are reported as not fixable.
///  - `DuplicateContour`: a contour identical (same spans within tolerance, same or reversed
///    direction, any starting span) to an earlier one, in the same or another object. Fix: delete the
///    later copy.
///  - `SelfIntersecting`: a contour crossing, touching or overlapping itself. Never fixed
///    automatically (the right repair is a design decision).
///  - `WrongDirection`: a closed, simple contour whose winding does not follow the nesting
///    convention within its object (outer contours counter-clockwise, holes clockwise, alternating
///    with depth). Fix: reverse it (no area change).
///  - `InvalidGeometry`: a non-finite or absurdly large coordinate. Reported, never modified, and
///    skipped by every other check.
namespace carvetto::geometry::validator {

enum class IssueKind : std::size_t {
    ZeroLengthSpan,
    TinyContour,
    OpenContour,
    DuplicateContour,
    SelfIntersecting,
    WrongDirection,
    InvalidGeometry,
};
inline constexpr std::size_t kIssueKindCount = 7;

struct Options {
    Tolerance tol{Tolerance::defaults()};
    double tinySizeMm{0.01};
    double closeGapMm{0.01};
};

/// Hard bounds so hostile input cannot run away.
inline constexpr std::size_t kMaxSpans = 2'000'000;            ///< Total spans examined; more: nothing is analysed.
inline constexpr std::size_t kMaxSelfIntersectSpans = 20'000;  ///< Longer contours skip the crossing check.
inline constexpr std::size_t kMaxNestingContours = 1'000;      ///< Objects with more contours skip the direction check.
inline constexpr std::size_t kMaxDuplicateComparisons = 5'000'000;
inline constexpr double kMaxCoordinateMm = 1.0e9;

struct Issue {
    IssueKind kind{IssueKind::ZeroLengthSpan};
    std::size_t object{0};
    std::size_t contour{0};
    std::size_t span{0};  ///< ZeroLengthSpan only.
    /// DuplicateContour only: the earlier contour this one repeats.
    std::size_t otherObject{0};
    std::size_t otherContour{0};
    bool fixable{false};
};

struct Report {
    std::vector<Issue> issues;
    std::array<std::size_t, kIssueKindCount> counts{};
    /// Contours/objects a check could not examine (too large, or a work budget ran out). When
    /// non-zero or `truncated`, absence of an issue is not a guarantee.
    std::size_t skipped{0};
    bool truncated{false};

    std::size_t count(IssueKind k) const { return counts[static_cast<std::size_t>(k)]; }
    std::size_t total() const { return issues.size(); }
    std::size_t fixableCount() const;
};

/// One object's contours.
using ObjectContours = std::vector<Contour>;

Report validate(const std::vector<const ObjectContours*>& objects, const Options& options = {},
                const std::vector<bool>& skipIntersections = {});

struct FixFlags {
    bool zeroLength{true};
    bool tiny{true};
    bool open{true};
    bool duplicates{true};
    bool direction{true};
};

struct FixSummary {
    std::size_t spansRemoved{0};
    std::size_t tinyRemoved{0};
    std::size_t contoursClosed{0};
    std::size_t duplicatesRemoved{0};
    std::size_t contoursReversed{0};

    std::size_t total() const {
        return spansRemoved + tinyRemoved + contoursClosed + duplicatesRemoved + contoursReversed;
    }
};

/// Repairs `objects` in place (order: zero-length, tiny, open, duplicate, direction; each step looks
/// at the result of the previous). A contour left without spans is removed. Objects may end up with
/// no contours; the caller decides what to do with them.
FixSummary fix(std::vector<ObjectContours>& objects, const Options& options = {}, const FixFlags& flags = {});

}  // namespace carvetto::geometry::validator
