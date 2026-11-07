#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/tolerance.hpp"

/// SVG interchange (ADR-0026).
///
/// `importSvg` reads SVG 1.1 geometry with Carvetto's own bounded XML subset parser and returns
/// plain geometry in the same shape as the DXF importer (ADR-0015): layers, contours tagged with
/// a layer index, and warnings. Everything is converted to millimetres and radians, with Y
/// pointing up (the page's bottom-left corner is the origin). Fill and stroke are ignored.
///
/// `exportSvg` writes each contour as a `<path>` in
/// millimetres, one `<g>` per layer, with a viewBox covering the geometry.
namespace carvetto::io {

/// Why an SVG operation failed.
struct SvgError {
    /// Failure category.
    enum class Code {
        InvalidFormat,   ///< Not well-formed XML of the supported subset, or not an `<svg>` document.
        Unsupported,     ///< Well-formed but refused on purpose (DOCTYPE / entity declarations).
        ResourceLimit,   ///< A size, depth, element or span budget was exceeded.
        InvalidArgument  ///< Export input is unusable (bad layer index, non-finite geometry).
    };
    Code code{Code::InvalidFormat};
    /// Human-readable description.
    std::string message;
    /// 1-based line of the input; 0 when not applicable.
    std::size_t line{0};

    /// "line N: message" (or just the message when `line` is 0).
    std::string describe() const;
};

/// Value-or-error return type (a small stand-in for C++23 `std::expected`).
template <class T>
class SvgResult {
   public:
    SvgResult(T value) : v_(std::move(value)) {}         // NOLINT(google-explicit-constructor)
    SvgResult(SvgError error) : v_(std::move(error)) {}  // NOLINT(google-explicit-constructor)

    /// True when a value is held.
    bool ok() const { return v_.index() == 0; }
    explicit operator bool() const { return ok(); }

    /// The value; only valid when `ok()`.
    T& value() { return std::get<0>(v_); }
    /// The value; only valid when `ok()`.
    const T& value() const { return std::get<0>(v_); }
    /// The error; only valid when `!ok()`.
    const SvgError& error() const { return std::get<1>(v_); }

   private:
    std::variant<T, SvgError> v_;
};

/// A non-fatal note produced while importing. Identical messages are merged and counted.
struct SvgWarning {
    /// Line of the first occurrence; 0 when not applicable.
    std::size_t line{0};
    /// Number of occurrences merged into this warning.
    std::size_t count{1};
    /// Human-readable description.
    std::string message;

    /// "line N: message", with " (xK)" appended when `count` > 1.
    std::string describe() const;
};

/// An SVG layer (an Inkscape layer, or a top-level `<g>` with an id).
struct SvgLayer {
    std::string name{"0"};

    bool operator==(const SvgLayer&) const = default;
};

/// A contour in millimetres, tagged with the index of its layer in `SvgDrawing::layers`.
struct SvgContour {
    std::size_t layer{0};
    geometry::Contour contour;

    bool operator==(const SvgContour&) const = default;
};

/// Options for `importSvg`.
struct SvgImportOptions {
    /// `chordal` bounds the deviation of curves approximated by cubic Beziers (elliptical arcs
    /// and ellipses); `distance` decides when a closing `Z` needs a line (within it: no line, first point wins).
    geometry::Tolerance tolerance{};
    /// Resolution that maps CSS pixels (the unit of unitless lengths) to millimetres.
    double dpi{96.0};
    /// Maximum input size in bytes.
    std::size_t maxInputBytes{32U * 1024U * 1024U};
    /// Maximum element nesting depth.
    std::size_t maxDepth{128};
    /// Maximum number of elements in the document.
    std::size_t maxElements{1'000'000};
    /// Maximum number of attributes on one element.
    std::size_t maxAttributes{256};
    /// Maximum length of an attribute value in bytes (except `d` and `points`).
    std::size_t maxAttributeLength{64U * 1024U};
    /// Maximum length of path data (`d`) and point lists (`points`) in bytes.
    std::size_t maxPathLength{16U * 1024U * 1024U};
    /// Maximum number of spans in the result.
    std::size_t maxSpans{1'000'000};
    /// Maximum number of element visits while rendering (`<use>` expansion counts each visit).
    std::size_t maxVisits{2'000'000};
    /// Maximum nesting of `<use>` references.
    std::size_t maxUseDepth{16};
    /// Maximum number of layers.
    std::size_t maxLayers{100'000};
};

/// Result of `importSvg`.
struct SvgDrawing {
    /// Layers in order of first appearance.
    std::vector<SvgLayer> layers;
    /// Geometry in mm, Y up.
    std::vector<SvgContour> contours;
    /// Skipped elements and other non-fatal findings, in order of first occurrence.
    std::vector<SvgWarning> warnings;
    /// Page size in mm (0 when the document declares neither a size nor a viewBox).
    double widthMm{0.0};
    double heightMm{0.0};
};

/// Imports SVG bytes. Never throws on bad input; fails only for unreadable data, refused
/// constructs (DOCTYPE) or exceeded resource budgets. Unsupported elements produce warnings.
SvgResult<SvgDrawing> importSvg(std::span<const std::uint8_t> bytes, const SvgImportOptions& options = {});

/// Options for `exportSvg`.
struct SvgExportOptions {
    /// Empty border around the geometry in the page, in mm.
    double margin{0.0};
};

/// Exports layers and contours (mm, Y up) as SVG. Fails with `InvalidArgument` when a contour
/// names a missing layer or holds non-finite coordinates. The page is the bounding box of the
/// geometry plus `margin` (at least 1 mm square); the root element carries a
/// `data-origin-mm="x y"` attribute holding the model coordinates of the page's bottom-left
/// corner, which `importSvg` honours so that a round trip restores the original positions.
/// Every contour is a `<path>`; arcs sweeping more than 270 degrees (including full circles,
/// closed or open, any start angle and direction) are written as two arc commands, so direction,
/// start point and the closed flag survive a round trip (a full circle comes back as two arcs).
/// Layer names are written as valid XML: `& < > "` escaped, characters not allowed in XML 1.0
/// and invalid UTF-8 replaced by U+FFFD, an empty name written as "Layer".
/// When `warnings` is given it receives counted notes: contours without spans and zero-length
/// spans (both dropped), and layer names that were changed.
SvgResult<std::vector<std::uint8_t>> exportSvg(std::span<const SvgLayer> layers,
                                               std::span<const SvgContour> contours,
                                               const SvgExportOptions& options = {},
                                               std::vector<SvgWarning>* warnings = nullptr);

}  // namespace carvetto::io
