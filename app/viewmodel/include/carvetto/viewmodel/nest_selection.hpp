#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/geometry/nesting.hpp"

// Qt-free "Nest Selected Vectors" (T-0337, ADR-0130, ADR-0007): the selected vectors are grouped into
// rigid parts, packed by `geometry::nestParts` and copied to their nested positions as ONE undo step.
//
// Parts. Selected vector objects (top level, on the active sheet, layer visible and unlocked; groups
// are not nested) form one part when their closed outlines overlap or one lies inside another.
// A part's outline is its outermost closed contour with the contours inside it as holes; a part whose
// outlines cross, or that has islands inside holes, is nested by the convex hull of its outlines
// (no holes). Selected open vectors move with the part whose outline contains their first point;
// open vectors outside every part are left where they are and reported.
//
// Area. `NewSheets`: every sheet the nester needs becomes a new document sheet (sizes from `sheets`,
// the last repeating; empty means the active sheet's size), objects sit in sheet-local millimetres
// with the datum at the bottom left. `ActiveSheet`: the active sheet's material rectangle, one
// sheet. `Boundary`: the closed vectors on `boundaryLayer` (active sheet, not selected) are the
// region, inner ones are exclusions; one sheet.
//
// Result. Each placed copy is a new object (same layer, name and payload; parametric creation
// parameters are dropped except text) on the target sheet, selected afterwards. `removeOriginals`
// deletes the originals of every part that got at least one copy. All-or-nothing: when nothing is
// placed, or the document cannot allocate the ids, the document, undo history and selection are
// untouched.

namespace carvetto::viewmodel {

enum class NestArea { NewSheets, ActiveSheet, Boundary };

struct NestSelectionOptions {
    NestArea area{NestArea::NewSheets};
    /// `NewSheets`: sheet sizes (mm). Empty: the active sheet's size.
    std::vector<geometry::NestSheet> sheets;
    /// `Boundary`: layer whose closed vectors on the active sheet make the region.
    document::LayerId boundaryLayer{};
    double toolDiameterMm{6.0};
    double clearanceMm{2.0};
    double borderGapMm{10.0};
    double rotationStepRad{0.0};
    bool mirror{false};
    bool allowInside{false};
    geometry::NestCorner corner{geometry::NestCorner::BottomLeft};
    geometry::NestDirection direction{geometry::NestDirection::AlongX};
    /// Copies of each part, 1 .. `kMaxNestCopies`.
    int copies{1};
    bool removeOriginals{true};
    std::size_t maxSheets{64};
};

inline constexpr int kMaxNestCopies = 1000;
/// Most selected objects a nest accepts.
inline constexpr std::size_t kMaxNestObjects = 2000;

struct NestSelectionResult {
    bool ok{false};
    std::string message;  ///< Empty on success; otherwise the reason, for a message box.
    std::vector<document::ObjectId> created;
    std::vector<document::SheetId> sheets;  ///< Sheets created (`NewSheets`), in order.
    std::size_t parts{0};
    std::size_t placed{0};
    std::size_t unplaced{0};
    std::size_t skippedOpen{0};  ///< Selected open vectors outside every part.
};

/// Empty when `options` are usable, else the reason.
std::string validateNestOptions(const NestSelectionOptions& options);
/// Enabling rule: at least one eligible selected vector.
bool canNestSelection(const document::Document& doc);
NestSelectionResult nestSelection(document::Document& doc, const NestSelectionOptions& options);

}  // namespace carvetto::viewmodel
