#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/document/toolpath_record.hpp"
#include "carvetto/geometry/contour.hpp"

// The Vector Selector of a toolpath (T-0220, ADR-0031; feature-inventory "Vector Selector"): which
// vectors a toolpath machines, re-resolved against the document every time the toolpath is
// calculated or checked for staleness ("Associate with toolpath").
//
// A selector is a base selection (every vector, the vectors of some layers, or explicit objects
// such as the current selection) narrowed by filters: open vectors, closed vectors, and only
// circles (optionally of one diameter within a tolerance). Resolution rules:
//   * Only objects on the active sheet, whose layer and every ancestor group's layer is visible and
//     unlocked, contribute. Others are skipped and counted (never an error).
//   * Order is the core's: the base selection's order (`Document::selectedVectors`), each object's
//     contours in order. Stale ids (deleted items) name nothing.
//   * Filters apply per contour; an object stays in `objects` when one of its contours passes.
//
// What a record keeps: the native record stores only a `document::VectorSelection` (ADR-0021). A
// selector without filters is stored as its base. With filters, the record stores the resolved
// objects, so a reloaded file keeps the vectors of the last calculation (the filters themselves
// live in the session only).

namespace carvetto::viewmodel {

/// Which vectors a toolpath machines.
struct VectorSelector {
    /// Every vector, the vectors on layers, or explicit objects (groups stand for their vectors).
    document::VectorSelection base{document::VectorSelection::allVectors()};
    bool includeOpen{true};
    bool includeClosed{true};
    /// Keep only closed contours that are circles (arcs about one centre, one full turn).
    bool onlyCircles{false};
    /// With `onlyCircles`: keep circles of this diameter (mm) within `circleToleranceMm`; 0 = any.
    double circleDiameterMm{0.0};
    double circleToleranceMm{0.01};

    bool operator==(const VectorSelector&) const = default;

    /// True when some filter narrows the base.
    bool hasFilters() const {
        return !includeOpen || !includeClosed || onlyCircles;
    }

    static VectorSelector allVectors() { return VectorSelector{}; }
    static VectorSelector fromLayers(std::vector<document::LayerId> layers);
    static VectorSelector fromObjects(std::vector<document::ObjectId> objects);
    /// The document's current selection (selection order); ids that name nothing are dropped.
    static VectorSelector fromSelection(const document::Document& doc);
};

/// One contour taking part, with the object it belongs to. Valid while the document is unchanged.
struct ContourRef {
    document::ObjectId object;
    const geometry::Contour* contour{nullptr};
};

struct ResolvedVectors {
    /// Objects with at least one contour taking part, in order, each once.
    std::vector<document::ObjectId> objects;
    /// The contours taking part, in machining-input order.
    std::vector<ContourRef> contours;
    /// Vector objects left out, by reason (an object counts once, for its first reason).
    std::size_t skippedHidden{0};
    std::size_t skippedLocked{0};
    std::size_t skippedOtherSheet{0};
    /// Contours left out by the open/closed/circle filters.
    std::size_t filteredOut{0};
    /// Total spans of the contours taking part.
    std::size_t spanCount{0};

    /// Copies of the contours taking part.
    std::vector<geometry::Contour> copyContours() const;
    /// The object that contour `index` belongs to (null when out of range).
    document::ObjectId ownerOf(std::size_t contour) const;
};

/// Resolves `selector` against `doc` (see the rules above). Never fails.
ResolvedVectors resolveVectors(const document::Document& doc, const VectorSelector& selector);

/// The bitmap a photo V-carve reads (T-0250b): the first bitmap object among the selector's
/// explicit objects (a selection of every vector or of layers names none).
struct ResolvedBitmap {
    /// Null when there is none; then `problem` says why.
    document::ObjectId object;
    /// Valid while the document is unchanged; null when `object` is null.
    const document::BitmapObject* bitmap{nullptr};
    /// Why it cannot be read (no bitmap selected, hidden, locked, on another sheet, no embedded image).
    std::string problem;
};

/// Finds the bitmap of `selector` in `doc` under the same usability rules as vectors (active
/// sheet, visible and unlocked layers). Never fails.
ResolvedBitmap resolveBitmap(const document::Document& doc, const VectorSelector& selector);

/// Empty when the selector's ids are all usable in a record (allocated ids of the right kind,
/// `all` alone, no repeats), else the reason. Filter values must be finite and non-negative.
std::string validateSelector(const document::Document& doc, const VectorSelector& selector);

/// `selector` with repeated ids dropped (first kept), ready to be stored.
VectorSelector normalisedSelector(VectorSelector selector);

/// The selection a record stores for `selector` resolved to `resolved`: the base when the selector
/// has no filters, else the resolved objects.
document::VectorSelection storedSelection(const VectorSelector& selector, const ResolvedVectors& resolved);

/// True when `contour` is a closed circle (all spans arcs about one centre and radius, together
/// one full turn) and, when `diameterMm` > 0, of that diameter within `toleranceMm`.
bool isCircleContour(const geometry::Contour& contour, double diameterMm, double toleranceMm);

/// 64-bit FNV-1a style hash for change detection (not cryptographic). Doubles are hashed by value
/// (-0 and 0 alike); equal inputs always give equal hashes.
class Fingerprint {
   public:
    void add(std::uint64_t value);
    void add(double value);
    void add(std::string_view text);
    void add(const geometry::Contour& contour);
    std::uint64_t value() const { return state_; }

   private:
    std::uint64_t state_{14695981039346656037ULL};
};

/// Hash of what a photo V-carve reads: the bitmap's object, image key, size and placement (the
/// pixels are named by their content key), or its problem.
std::uint64_t fingerprint(const ResolvedBitmap& resolved);

/// Hash of the resolved vectors: object ids and the geometry of every contour taking part.
std::uint64_t fingerprint(const ResolvedVectors& resolved);

}  // namespace carvetto::viewmodel
