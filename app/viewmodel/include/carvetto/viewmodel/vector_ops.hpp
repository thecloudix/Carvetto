#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/tolerance.hpp"

// Qt-free boolean and offset operations on the current selection (T-0207, ADR-0014, ADR-0006,
// ADR-0012). Each operation is ONE undo step (a macro of `AddObjectCommand`s and one
// `RemoveObjectsCommand`), is all-or-nothing (the result is computed completely before the
// document is touched; an error leaves the document, undo history and selection unchanged) and
// leaves the new objects selected. Selection state itself is not part of the undo history.
//
// Which objects take part ("operands"):
//   * Only objects on the active sheet whose layer, and every ancestor group's layer, is visible
//     and unlocked. Others in the selection are ignored and never modified or deleted.
//   * Groups are resolved to their vector leaves (recursively, document order). A selected group
//     is one operand made of all its eligible leaves united; a leaf reached twice (selected and
//     inside a selected group) counts once, in the earliest operand. Bitmaps are ignored.
//   * Operands are taken in selection order; "last" means the last-selected operand.
//   * Booleans use the closed contours only; an operand without a closed contour is dropped (and not
//     consumed). Open contours inside a consumed object are consumed with it.
//   * Within one object the closed contours form a region by the even-odd rule (nested contours
//     are holes/islands whatever their winding); operands are then combined with the rules below.
//
// Operations (feature inventory "Weld/Subtract/Overlap Vectors"):
//   Weld      union of all operands (needs 2+ operands).
//   Subtract  every operand but the last, each minus the last-selected one (2+ operands).
//   Intersect every operand but the last, each kept only where it overlaps the last (2+).
//             (The inventory calls it Overlap.)
//   Exclude   symmetric difference, folded in selection order (2+).
// All consumed operands (including the last for Subtract/Intersect) are deleted: their leaf
// objects are removed (an emptied group goes with them). The result is plain vector objects (no
// creation parameters) on the layer of the operand they came from (the first operand for Weld
// and Exclude), on the active sheet, one object per connected region: an outer contour
// (counter-clockwise) with the holes (clockwise) found inside it.
//
// Errors come back in `VectorOpResult::message` (suitable for a dialog): too few operands, no
// closed vector, nothing left after the operation, input too large (more than `kMaxSpans` spans
// or a coordinate beyond `kMaxCoordinateMm`), invalid parameters, no free object ids.

namespace carvetto::viewmodel {

enum class BooleanKind { Weld, Subtract, Intersect, Exclude };

/// Most spans (over all operands) an operation accepts.
inline constexpr std::size_t kMaxVectorOpSpans = 2'000'000;
/// Largest coordinate magnitude (mm) an operation accepts.
inline constexpr double kMaxVectorOpCoordinateMm = 1.0e9;
/// Largest offset distance (mm) the offset operation accepts.
inline constexpr double kMaxOffsetDistanceMm = 1.0e6;

struct VectorOpResult {
    bool ok{false};
    /// Empty on success; otherwise the reason, for a message box.
    std::string message;
    /// The new objects, now selected, in creation order.
    std::vector<document::ObjectId> created;
    /// The objects removed (leaves only; emptied groups not counted).
    std::size_t removed{0};
};

/// Menu enabling rules (cheap; no geometry is computed): the operation would not fail for lack
/// of operands. `canWeld` etc. need two operands with a closed vector; `canOffset` needs one
/// vector leaf (open or closed).
bool canBoolean(const document::Document& doc, BooleanKind kind);
bool canWeld(const document::Document& doc);
bool canSubtract(const document::Document& doc);
bool canIntersect(const document::Document& doc);
bool canExclude(const document::Document& doc);
bool canOffset(const document::Document& doc);

/// Runs a boolean operation on the document's selection.
VectorOpResult applyBoolean(document::Document& doc, BooleanKind kind,
                            const geometry::Tolerance& tol = geometry::Tolerance::defaults());

enum class OffsetDirection {
    Outwards,  ///< Closed: grow. Open: to the right of the direction of travel.
    Inwards    ///< Closed: shrink. Open: to the left of the direction of travel.
};

struct OffsetOptions {
    /// Offset distance in mm: finite, > 0 and at most `kMaxOffsetDistanceMm`.
    double distanceMm{1.0};
    OffsetDirection direction{OffsetDirection::Outwards};
    geometry::JoinType join{geometry::JoinType::Round};
    /// Mitre limit for Sharp joins, as a multiple of the distance (>= 1).
    double mitreLimit{2.0};
    /// Remove the source objects whose offset produced something.
    bool deleteOriginals{false};
};

/// Empty when `options` are usable, else the reason.
std::string validateOffsetOptions(const OffsetOptions& options);

/// Offsets every selected vector (leaf) separately: its closed contours together with
/// `geometry::offsetContours` (holes follow the core's orientation convention), its open contours
/// each with `geometry::offsetOpenContour` (chains). The result of one source is one new plain
/// vector object on the source's layer. A source whose offset is empty (e.g. an inward offset
/// past its inradius) contributes nothing and is kept even with `deleteOriginals`.
VectorOpResult applyOffset(document::Document& doc, const OffsetOptions& options,
                           const geometry::Tolerance& tol = geometry::Tolerance::defaults());

}  // namespace carvetto::viewmodel
