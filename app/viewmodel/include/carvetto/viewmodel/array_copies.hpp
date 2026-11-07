#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/geometry/vec2.hpp"

// Qt-free copy operations on the current selection (T-0209, ADR-0014, ADR-0007): Array Copy,
// Circular Copy and Copy Along Vectors. Each is ONE undo step (a macro of `AddObjectCommand`s,
// plus one `AddLayerCommand` when copies go to a new layer), is all-or-nothing (the whole plan is
// built and checked before the document is touched; an error leaves document, undo history and
// selection unchanged) and leaves the new copies selected. Selection is not part of undo.
//
// Sources: the selected objects on the active sheet whose layer, and every ancestor group's layer,
// is visible and unlocked. Others are ignored (never copied, never modified). A selected object
// whose ancestor group is also selected is copied as part of that group. Groups are copied whole
// (same structure; descendants on hidden or locked layers are left out; a group with nothing left
// is skipped). Copies of a selected object that sits inside an unselected group become top-level
// objects. Copies keep layer, sheet (the active sheet), name and payload; the original selection
// is never changed. Creation parameters stay parametric where the transform keeps them so (a
// translation, or a rotation by a multiple of 90 degrees without mirroring); otherwise the copy
// is a plain vector (`NoParams`).
//
// Document ids are never reused (ADR-0007), so undo restores the document except for the id
// counter.
//
// Work cap: (copies) x (objects + spans per copy) must not exceed `kMaxCopyWork` (500000).

namespace carvetto::viewmodel {

/// Most (copies x (objects + spans)) a copy operation accepts.
inline constexpr std::size_t kMaxCopyWork = 500'000;
/// True when `copies` x `unitCost` (objects + spans of one copy) is over `kMaxCopyWork` (or NaN).
bool copyWorkExceeded(double copies, std::size_t unitCost);
/// Most copies a circular copy or a count-based along-vector copy accepts.
inline constexpr int kMaxCopyCount = 100'000;
/// Largest coordinate / distance magnitude (mm) accepted in options.
inline constexpr double kMaxCopyCoordinateMm = 1.0e9;
/// Most rows or columns of an array.
inline constexpr int kMaxArrayLines = 100'000;

struct CopyResult {
    bool ok{false};
    /// Empty on success; otherwise the reason, for a message box.
    std::string message;
    /// The new top-level copies (a group copy counts once), now selected, in creation order.
    std::vector<document::ObjectId> created;
    /// Number of copies of the selection made (an array of 3x2 makes 5; a group copy is one).
    std::size_t copies{0};
};

/// Menu enabling rules (cheap; nothing is computed beyond collecting the sources).
bool canArrayCopy(const document::Document& doc);
bool canCircularCopy(const document::Document& doc);
/// The last selected vector object (not a group): the default guide curve for Copy Along Vectors.
document::ObjectId defaultGuideObject(const document::Document& doc);
bool canCopyAlongVectors(const document::Document& doc);

// --- Array Copy ---------------------------------------------------------------------------------

enum class ArraySpacing {
    Gap,    ///< x/y are the clear gaps between copies: the step is the selection's size plus the gap.
    Offset  ///< x/y are the steps between copy origins (the same cell of neighbouring copies).
};

/// Block symmetry of alternate cells. Cells are indexed (row, column) from the original at
/// (0, 0); "alternate" means odd index. A mirrored cell is mirrored about its own centre.
enum class ArraySymmetry {
    None,
    MirrorColumns,  ///< Odd columns mirrored left-right.
    MirrorRows,     ///< Odd rows mirrored top-bottom.
    MirrorBoth,     ///< Odd columns left-right and odd rows top-bottom (cell odd/odd turns 180 degrees).
    Rotate180       ///< Cells with odd (row + column) turned 180 degrees.
};

struct ArrayOptions {
    /// Rows (along +Y) and columns (along +X), 1 .. `kMaxArrayLines`, at least 2 cells in total.
    int rows{1};
    int columns{2};
    ArraySpacing spacing{ArraySpacing::Offset};
    /// Gap or step between columns (x) and rows (y) in mm, per `spacing`.
    double xMm{10.0};
    double yMm{10.0};
    /// Odd rows are shifted along +X by this many mm (brick pattern).
    double rowDisplacementMm{0.0};
    /// Odd columns are shifted along +Y by this many mm.
    double columnDisplacementMm{0.0};
    ArraySymmetry symmetry{ArraySymmetry::None};
    /// Each copy of the selection becomes one new group (the original cell is left as it is).
    bool groupCopies{false};
};

/// Empty when `options` are usable, else the reason.
std::string validateArrayOptions(const ArrayOptions& options);

/// Cell (0, 0) is the original selection (part of the count); every other cell is a copy.
/// The grid sits in the selection's bounding box: the Gap step is box size + gap.
CopyResult applyArrayCopy(document::Document& doc, const ArrayOptions& options);

// --- Circular Copy ------------------------------------------------------------------------------

enum class CircularAngleMode {
    TotalAngle,  ///< `angleRad` is spread over all copies.
    StepAngle    ///< `angleRad` is the angle between neighbouring copies.
};

struct CircularOptions {
    geometry::Vec2 centreMm{};
    /// Number of copies including the original (>= 2).
    int count{4};
    CircularAngleMode mode{CircularAngleMode::TotalAngle};
    /// Radians, non-zero, magnitude at most 2 pi. Following the feature inventory a negative
    /// step goes counter-clockwise (so a positive one clockwise).
    /// Total angle: a full circle (>= 2 pi) spaces the copies by total / count (the last copy does
    /// not land on the original); a smaller one by total / (count - 1) (the last copy ends the arc).
    double angleRad{6.283185307179586};
    /// Copies turn with the circle; otherwise they are only moved (the selection's bounding-box
    /// centre travels on the circle, the orientation stays).
    bool rotateCopies{true};
};

std::string validateCircularOptions(const CircularOptions& options);
CopyResult applyCircularCopy(document::Document& doc, const CircularOptions& options);

// --- Copy Along Vectors -------------------------------------------------------------------------

enum class AlongSpacing {
    Distance,  ///< Copies every `distanceMm` of arc length.
    Count      ///< `count` copies spread over the vector.
};

enum class AlongCopyKind {
    Objects,  ///< Copies of the selected objects (everything selected but the guide).
    Circles   ///< New circles of `circleDiameterMm` (the selection other than the guide is ignored).
};

struct AlongVectorOptions {
    /// The guide curve: a vector object on the active sheet (its first non-empty contour is used).
    document::ObjectId guide{};
    AlongCopyKind kind{AlongCopyKind::Objects};
    double circleDiameterMm{5.0};
    AlongSpacing spacing{AlongSpacing::Distance};
    double distanceMm{10.0};
    /// Count >= 1. Open guide: copies at both ends (a single one sits at the start); closed
    /// guide: evenly around the loop starting at its start point.
    int count{5};
    /// Distance mode only: change the distance slightly so the copies divide the length evenly
    /// (open guide: the last copy lands on the end; closed: no gap at the seam).
    bool forceEvenSpacing{false};
    /// Rotate each copy so its x axis follows the guide's tangent (objects only).
    bool alignToCurve{false};
    /// Put the copies on a new layer.
    bool newLayer{false};
    /// Walk the guide from its end to its start.
    bool reverse{false};
};

/// Distance mode without even spacing places copies at 0, d, 2d, ... (open guide: up to and
/// including the end; closed: while before the end). The selection's bounding-box centre is put
/// on each point. The copies are new objects; the originals stay and are not counted.
std::string validateAlongVectorOptions(const AlongVectorOptions& options);
CopyResult applyCopyAlongVectors(document::Document& doc, const AlongVectorOptions& options);

}  // namespace carvetto::viewmodel
