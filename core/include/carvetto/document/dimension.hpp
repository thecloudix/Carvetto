#pragma once

#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/document/object.hpp"
#include "carvetto/document/units.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/vec2.hpp"

/// Dimensions (T-0246): measuring, text, drawing geometry and undoable edits for
/// `DimensionObject`. All lengths mm, angles radians. A dimension stores only its defining points
/// (optionally anchored to part geometry) and options; the measured value is derived on demand, so
/// it follows every move, scale or node edit of the part with no bookkeeping and undo/redo of the
/// part edit restores it too.
namespace carvetto::document {

inline constexpr int kMaxDimensionDecimals = 8;
inline constexpr int kMaxDimensionAngleDecimals = 6;
inline constexpr std::size_t kMaxDimensionTextBytes = 1024;
/// Largest accepted coordinate or offset, and text height (mm); hostile input is refused beyond.
inline constexpr double kMaxDimensionMagnitudeMm = 1e9;
inline constexpr double kMaxDimensionTextHeightMm = 10000.0;
/// The layer name dimensions are placed on by default.
inline constexpr const char* kDimensionLayerName = "Dimensions";

/// Checks finiteness and ranges (decimals, text height > 0, text length, anchor indices). Empty
/// when fine, otherwise the problem.
std::string dimensionProblem(const DimensionObject& dim);

/// The point `anchor` names in `doc`, or nullopt when the object, contour or span is missing or
/// the span has no such point (a centre of a line or Bezier).
std::optional<geometry::Vec2> resolveAnchor(const Document& doc, const DimensionAnchor& anchor);

/// The defining points with anchors re-resolved (unresolvable anchors keep the stored position).
/// Only the first `dimensionPointCount(kind)` entries are meaningful.
std::array<geometry::Vec2, 3> resolveDimensionPoints(const Document& doc, const DimensionObject& dim);

/// Number of defining points the kind uses: 2 (linear, aligned, radial) or 3 (angular).
std::size_t dimensionPointCount(DimensionKind kind);

/// The measured value from `points` (mm; radians for angular, in (0, pi], or (pi, 2*pi) when reflex, see below; the radius or, with
/// `diameter`, the diameter for radial). Nullopt when degenerate (coincident points, a zero-length
/// arm, non-finite input).
std::optional<double> measureDimensionPoints(const DimensionObject& dim,
                                             const std::array<geometry::Vec2, 3>& points);

/// `measureDimensionPoints` on the resolved points.
std::optional<double> measureDimension(const Document& doc, const DimensionObject& dim);

/// Text of a measured value: lengths in `units` with `decimals` places and the unit omitted;
/// radial with the prefix "R" or the diameter sign; angles in degrees with `angleDecimals` places
/// and a degree sign (UTF-8).
std::string formatDimensionValue(const DimensionObject& dim, double value, Units units);

/// The text shown: the custom text when `useCustomText`, else the formatted measurement, empty
/// when the dimension is degenerate.
std::string dimensionText(const Document& doc, const DimensionObject& dim, Units units);

/// The `offsetMm` that puts the annotation through `placement` (the third/last click of the
/// dimension tool): linear/aligned, the signed distance from the points' midpoint along the
/// normal; angular, the distance from the vertex; radial, the leader length beyond the circle
/// (never negative). `points` are resolved positions. Nullopt when degenerate.
std::optional<double> offsetForPlacement(const DimensionObject& dim, const std::array<geometry::Vec2, 3>& points,
                                         geometry::Vec2 placement);

/// An angular dimension measures the reflex angle (2*pi minus the smaller angle) when its
/// `offsetMm` is negative (the radius is its magnitude); the dimension tool sets that when the arc
/// is placed outside the smaller angle. This needs no stored flag, so the `.carv` format is
/// unchanged.

/// Drawing geometry of a dimension: extension lines, dimension line or arc and arrowheads (closed
/// triangles), plus where the text goes.
struct DimensionGeometry {
    std::vector<geometry::Contour> contours;
    geometry::Vec2 textCentre{};  ///< Centre of the text box, mm.
    double textAngle{0.0};        ///< Baseline direction, radians, within (-pi/2, pi/2].
    bool valid{false};            ///< False when the dimension is degenerate (nothing drawn).
};

/// Arrowhead length as a multiple of the text height; width is a third of the length.
inline constexpr double kDimensionArrowFactor = 1.0;

/// Builds the geometry from the resolved points. A closed-form function of the dimension and the
/// part, so exploding to DXF and canvas drawing agree.
DimensionGeometry dimensionGeometry(const Document& doc, const DimensionObject& dim);

/// Control points of a dimension (T-0255), as the inventory describes: white handles change the
/// size or placement (points, text position, arc radius), black ones the legs and the text height.
enum class DimensionHandleKind {
    Point,       ///< Defining point `index` (white).
    Text,        ///< Text position: moves the annotation (white).
    ArcRadius,   ///< Angular: radius of the dimension arc (white).
    Leg,         ///< Linear/aligned: end of the extension legs, i.e. the dimension line position (black).
    TextHeight,  ///< Text height (black).
};
struct DimensionHandle {
    DimensionHandleKind kind{DimensionHandleKind::Point};
    std::size_t index{0};
    geometry::Vec2 pos{};
    bool white{true};
    bool operator==(const DimensionHandle&) const = default;
};

/// Handles at their current positions (empty for a degenerate dimension).
std::vector<DimensionHandle> dimensionHandles(const Document& doc, const DimensionObject& dim);

/// The dimension with `handle` dragged to `to`: a point moves (and loses its anchor), the text,
/// arc radius or leg handles change the offset, the text-height handle scales the text height by
/// its distance from the text centre. Nullopt when the result would be invalid or degenerate or
/// `to` is not finite / beyond 1e9 mm. Never touches the document.
std::optional<DimensionObject> moveDimensionHandle(const Document& doc, const DimensionObject& dim,
                                                   const DimensionHandle& handle, geometry::Vec2 to);

/// The nearest vertex or arc centre of a vector object on the active sheet within `toleranceMm`
/// of `p`, as an anchor with its position. Used by the dimension tool to attach picked points to
/// the part. Ties go to the lower object id.
struct AnchorHit {
    DimensionAnchor anchor;
    geometry::Vec2 pos;
};
std::optional<AnchorHit> findAnchorNear(const Document& doc, SheetId sheet, geometry::Vec2 p,
                                        double toleranceMm);

/// Adds a dimension on `layer` (the named layer is created by `addDimension` when `layer` is null
/// and none called "Dimensions" exists). One undo step. Returns the new object id, or nullopt when
/// the dimension has a problem (`dimensionProblem`), the sheet is unknown or no ids are left.
std::optional<ObjectId> addDimension(Document& doc, const DimensionObject& dim, SheetId sheet,
                                     LayerId layer = {});

/// Replaces the payload of a dimension. All or nothing: false when `id` is not a dimension, the
/// new value has a problem, or nothing changes. Undo restores the previous payload.
class SetDimensionCommand : public Command {
   public:
    SetDimensionCommand(ObjectId id, DimensionObject dim, std::string text = "Edit dimension")
        : id_(id), dim_(std::move(dim)), text_(std::move(text)) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return text_; }

   private:
    ObjectId id_;
    DimensionObject dim_;
    std::string text_;
    std::optional<DimensionObject> before_;
};

}  // namespace carvetto::document
