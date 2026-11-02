#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/vec2.hpp"

/// DXF interchange (ADR-0015).
///
/// `importDxf` reads ASCII DXF (R12 through 2018) and binary DXF (R12 and R13+ group-code
/// layouts) with Carvetto's own group-code reader and returns plain geometry: layers, contours
/// tagged with a layer index, text items for later rendering, and warnings. It is independent of
/// the document model so the CLI and the app can map the result as they need. Everything is
/// converted to millimetres and radians using `$INSUNITS` (falling back to `$MEASUREMENT`).
///
/// `exportDxf` writes DXF R2000 (AC1015) ASCII in millimetres: lines and arcs as LWPOLYLINE
/// (arcs via bulges), Bezier runs as clamped cubic SPLINEs, single full circles as CIRCLE.
/// Output is deterministic (fixed handles, shortest round-trip numbers).
namespace carvetto::io {

/// Why a DXF operation failed.
struct DxfError {
    /// Failure category.
    enum class Code {
        InvalidFormat,      ///< Not DXF, or a group code / value that cannot be read.
        Truncated,          ///< The data ends inside a pair or a section.
        UnsupportedFormat,  ///< Recognised but unsupported variant (e.g. unknown binary layout).
        ResourceLimit,      ///< A span, instance, text or nesting budget was exceeded.
        InvalidArgument,    ///< Export input is unusable (bad layer index, non-finite geometry).
    };
    Code code{Code::InvalidFormat};
    /// Human-readable description.
    std::string message;
    /// 1-based line of an ASCII file (byte offset for binary files); 0 when not applicable.
    std::size_t line{0};

    /// "line N: message" (or just the message when `line` is 0).
    std::string describe() const;
};

/// Value-or-error return type (a small stand-in for C++23 `std::expected`).
template <class T>
class DxfResult {
   public:
    DxfResult(T value) : v_(std::move(value)) {}         // NOLINT(google-explicit-constructor)
    DxfResult(DxfError error) : v_(std::move(error)) {}  // NOLINT(google-explicit-constructor)

    /// True when a value is held.
    bool ok() const { return v_.index() == 0; }
    explicit operator bool() const { return ok(); }

    /// The value; only valid when `ok()`.
    T& value() { return std::get<0>(v_); }
    /// The value; only valid when `ok()`.
    const T& value() const { return std::get<0>(v_); }
    /// The error; only valid when `!ok()`.
    const DxfError& error() const { return std::get<1>(v_); }

   private:
    std::variant<T, DxfError> v_;
};

/// A non-fatal note produced while importing. Identical messages are merged and counted.
struct DxfWarning {
    /// Line (ASCII) or byte offset (binary) of the first occurrence; 0 when not applicable.
    std::size_t line{0};
    /// Number of occurrences merged into this warning.
    std::size_t count{1};
    /// Human-readable description.
    std::string message;

    /// "line N: message", with " (xK)" appended when `count` > 1.
    std::string describe() const;
};

/// A DXF layer. `rgb` is always meaningful; `aci` is the AutoCAD Colour Index (1-255) or 0 when
/// it should be derived from `rgb` (export only).
struct DxfLayer {
    std::string name{"0"};
    /// Colour index 1-255 as stored in the file (absolute value of group 62); 0 = derive from rgb.
    int aci{7};
    /// 0xRRGGBB: the layer's true colour (group 420) or the colour of its `aci`.
    std::uint32_t rgb{0xFFFFFF};
    /// False when the layer is off (negative colour) or frozen.
    bool visible{true};

    bool operator==(const DxfLayer&) const = default;
};

/// A contour in millimetres, tagged with the index of its layer in `DxfDrawing::layers`.
struct DxfContour {
    std::size_t layer{0};
    geometry::Contour contour;

    bool operator==(const DxfContour&) const = default;
};

/// Which entity a text item came from.
enum class DxfTextKind { Text, MText, Attrib };
/// Horizontal justification (TEXT group 72; MTEXT attachment columns map to Left/Center/Right).
enum class DxfHAlign { Left, Center, Right, Aligned, Middle, Fit };
/// Vertical justification (TEXT group 73; MTEXT attachment rows map to Top/Middle/Bottom).
enum class DxfVAlign { Baseline, Bottom, Middle, Top };

/// A text entity kept for later rendering (positions in mm, angles in radians).
struct DxfText {
    std::size_t layer{0};
    DxfTextKind kind{DxfTextKind::Text};
    /// UTF-8 text; MTEXT formatting codes stripped, paragraphs separated by '\n'.
    std::string text;
    /// Text style name (group 7), "Standard" when absent.
    std::string style{"Standard"};
    /// Primary font file of the style from the STYLE table (e.g. "arial.ttf"); empty if unknown.
    std::string font;
    /// Character height in mm (MTEXT: nominal height).
    double height{0.0};
    /// Relative character width (TEXT group 41), adjusted for non-uniform block scaling.
    double widthFactor{1.0};
    /// First alignment / insertion point (group 10).
    geometry::Vec2 insertion;
    /// Second alignment point (TEXT group 11), present when the justification uses it.
    std::optional<geometry::Vec2> alignmentPoint;
    /// Baseline direction, radians from +X.
    double rotation{0.0};
    DxfHAlign hAlign{DxfHAlign::Left};
    DxfVAlign vAlign{DxfVAlign::Baseline};
    /// MTEXT reference rectangle width in mm (0 = no wrapping).
    double referenceWidth{0.0};
    /// True when the text reads mirrored (TEXT backwards flag, or a mirroring block insert).
    bool mirrored{false};

    bool operator==(const DxfText&) const = default;
};

/// Where the import's unit scale came from.
enum class DxfUnitSource { Insunits, Measurement, Default, Forced };

/// Options for `importDxf`.
struct DxfImportOptions {
    /// `chordal` bounds the deviation of curves approximated by cubic Beziers (ellipses, NURBS
    /// splines, arcs under non-uniform block scaling); `distance` decides coincident end points.
    geometry::Tolerance tolerance{};
    /// Millimetres per drawing unit when the file declares no units (default: millimetres).
    double unitlessScale{1.0};
    /// Overrides the file's units: millimetres per drawing unit.
    std::optional<double> forcedScale;
    /// Join consecutive open contours on the same layer whose ends coincide within
    /// `tolerance.distance` (default off: every entity stays its own contour; a later "Join
    /// vectors" tool owns joining).
    bool joinContiguous{false};
    /// Maximum number of spans in the result; exceeding it fails with `ResourceLimit`.
    std::size_t maxSpans{5'000'000};
    /// Maximum number of text items in the result; exceeding it fails with `ResourceLimit`.
    std::size_t maxTexts{1'000'000};
    /// Maximum number of block instances expanded (MINSERT cells count individually).
    std::size_t maxInstances{1'000'000};
    /// Maximum block nesting depth; deeper INSERTs are skipped with a warning.
    std::size_t maxBlockDepth{64};
};

/// Result of `importDxf`.
struct DxfDrawing {
    /// Layers in table order, followed by layers only referenced by entities.
    std::vector<DxfLayer> layers;
    /// Model-space geometry, blocks flattened, in mm.
    std::vector<DxfContour> contours;
    /// Text, attribute and multi-line text items, in mm.
    std::vector<DxfText> texts;
    /// Skipped entities and other non-fatal findings, in order of first occurrence.
    std::vector<DxfWarning> warnings;
    /// `$ACADVER` (e.g. "AC1015"); empty when the header does not say.
    std::string version;
    /// True when the input was binary DXF.
    bool binary{false};
    /// `$INSUNITS` as read (0 when absent).
    int insunits{0};
    /// Millimetres per drawing unit that was applied.
    double unitScale{1.0};
    DxfUnitSource unitSource{DxfUnitSource::Default};
};

/// Imports DXF bytes. Never throws on bad input; fails only for unreadable data or exceeded
/// resource budgets. Unsupported entities produce warnings.
DxfResult<DxfDrawing> importDxf(std::span<const std::uint8_t> bytes, const DxfImportOptions& options = {});

/// Exports layers and contours (mm) as DXF R2000 ASCII. Fails with `InvalidArgument` when a
/// contour names a missing layer or holds non-finite coordinates. Contours are expected to be
/// continuous. Arcs sweeping more than pi are written as several bulge segments, and contours
/// written as several entities carry Carvetto XDATA, so `importDxf` restores both exactly.
/// Zero-length lines and Beziers (all points coincident) cannot be represented and are dropped.
DxfResult<std::vector<std::uint8_t>> exportDxf(std::span<const DxfLayer> layers,
                                               std::span<const DxfContour> contours);

/// Millimetres per unit for a `$INSUNITS` code, or nullopt for 0 (unitless) and unknown codes.
std::optional<double> insunitsToMillimetres(int code);

/// 0xRRGGBB for an AutoCAD Colour Index 1-255 (7 maps to white; out-of-range gives white).
std::uint32_t aciToRgb(int aci);

/// The colour index 1-255 whose `aciToRgb` is nearest to `rgb`.
int nearestAci(std::uint32_t rgb);

/// Strips MTEXT inline formatting (\P paragraphs become '\n', stacks become "a/b", font/height/
/// colour codes and braces are removed, \U+XXXX escapes decoded). Input and output are UTF-8.
std::string stripMTextFormatting(std::string_view raw);

}  // namespace carvetto::io
