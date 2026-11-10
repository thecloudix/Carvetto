#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "carvetto/document/ids.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/transform2.hpp"
#include "carvetto/geometry/vec2.hpp"

namespace carvetto::document {

// ---------------------------------------------------------------------------------------------
// Creation parameters. Every parametric object keeps the values it was made from so the UI can
// re-open its form and rebuild the geometry. The variant is extended as new shape tools land.
// ---------------------------------------------------------------------------------------------

/// Geometry with no parametric origin (imported, drawn freehand, or edited at node level).
struct NoParams {
    constexpr bool operator==(const NoParams&) const = default;
};

/// Rectangle, optionally with rounded or chamfered corners.
struct RectangleParams {
    enum class CornerStyle { Square, Rounded, Chamfered };
    geometry::Vec2 centreMm{};
    double widthMm{0.0};
    double heightMm{0.0};
    CornerStyle corners{CornerStyle::Square};
    double cornerRadiusMm{0.0};

    bool operator==(const RectangleParams&) const = default;
};

/// Circle from centre and radius.
struct CircleParams {
    geometry::Vec2 centreMm{};
    double radiusMm{0.0};

    bool operator==(const CircleParams&) const = default;
};

/// Ellipse from centre, full axes and rotation (radians).
struct EllipseParams {
    geometry::Vec2 centreMm{};
    double widthMm{0.0};
    double heightMm{0.0};
    double angle{0.0};

    bool operator==(const EllipseParams&) const = default;
};

/// Regular polygon from centre, side count, circumscribed radius and rotation (radians).
struct PolygonParams {
    geometry::Vec2 centreMm{};
    int sides{3};
    double radiusMm{0.0};
    double angle{0.0};

    bool operator==(const PolygonParams&) const = default;
};

/// Star from centre, point count, outer/inner radii and rotation (radians).
struct StarParams {
    geometry::Vec2 centreMm{};
    int points{5};
    double outerRadiusMm{0.0};
    double innerRadiusMm{0.0};
    double angle{0.0};

    bool operator==(const StarParams&) const = default;
};

/// Horizontal alignment of text lines about the anchor (point text), the box edges or centre (text
/// in a box), or the start offset along the guide (text on a curve).
enum class TextAlign { Left, Centre, Right };

/// How the text is placed (T-0211, ADR-0033 amendment A1).
enum class TextMode {
    Point,  ///< Lines grow from a baseline anchor at the local origin; no wrapping.
    Box,    ///< Wrapped to `boxWidthMm`; the box spans x in [0, w], y in [-h, 0] of the local frame.
    Curve,  ///< One line set along `guide` (a snapshot of the guide contour in the local frame).
};

/// A text object: the string and everything needed to lay it out again. The object's contours (the
/// glyph outlines, a cached derived view, see `buildTextContours`) are stored next to it in the
/// `VectorObject`, so every geometry operation and toolpath works on text unchanged; this record
/// is what the text tool edits. Layout happens in a local frame and `placement` maps it into the
/// sheet, so moving, rotating, scaling or mirroring the object only composes into `placement`.
struct TextParams {
    std::string text;     ///< UTF-8; line feed separates lines/paragraphs.
    std::string fontKey;  ///< Key of an embedded font in `Document::fonts()`.
    double sizeMm{10.0};  ///< Em size in mm.
    TextAlign align{TextAlign::Left};
    double letterSpacingMm{0.0};
    double lineSpacing{1.0};
    bool kerning{true};
    TextMode mode{TextMode::Point};
    double boxWidthMm{0.0};   ///< Box mode.
    double boxHeightMm{0.0};  ///< Box mode (informational: overflow is reported, not clipped).
    std::optional<ObjectId> guideId;   ///< Curve mode: the object the guide was taken from (informational).
    geometry::Contour guide;           ///< Curve mode: guide contour in the local frame.
    double startOffsetMm{0.0};         ///< Curve mode: arc length of the alignment anchor on the guide.
    geometry::Transform2 placement{};  ///< Local frame -> sheet.

    bool operator==(const TextParams&) const = default;
};

/// Tagged set of creation parameters.
using CreationParams = std::variant<NoParams, RectangleParams, CircleParams, EllipseParams, PolygonParams,
                                    StarParams, TextParams>;

// ---------------------------------------------------------------------------------------------
// Object payloads.
// ---------------------------------------------------------------------------------------------

/// Vector artwork: one or more contours plus the parameters that produced them.
struct VectorObject {
    std::vector<geometry::Contour> contours;
    CreationParams params{NoParams{}};

    bool operator==(const VectorObject&) const = default;

    /// Union of the contours' bounds (empty when there are none).
    geometry::Box2 bounds() const;
    /// Applies `m` to every contour.
    void transform(const geometry::Transform2& m);
};

/// Placeholder for a raster image placed in the drawing. Pixel data lives in the `bitmap` module;
/// here only the placement and a reference to the source are kept.
struct BitmapObject {
    int widthPx{0};
    int heightPx{0};
    /// Placement rectangle in document millimetres.
    geometry::Box2 placementMm{};
    /// Opaque reference to the pixel data (e.g. a member name inside the `.carv` container).
    std::string sourceRef;

    bool operator==(const BitmapObject&) const = default;

    geometry::Box2 bounds() const { return placementMm; }
    /// Moves/scales the placement rectangle (the axis-aligned box of the transformed corners).
    void transform(const geometry::Transform2& m);
};

/// A group of other objects. Children stay in the document's object table with `parent` set to
/// the group; the group itself owns no geometry.
struct GroupObject {
    std::vector<ObjectId> children;

    bool operator==(const GroupObject&) const = default;
};

/// Kinds of dimension (T-0246).
enum class DimensionKind {
    Linear,   ///< Distance between two points projected on the X axis (or Y with `vertical`).
    Aligned,  ///< True distance between two points, dimension line parallel to them.
    Angular,  ///< Angle at a vertex between two arms (the smaller angle, 0..pi).
    Radial,   ///< Radius (or diameter) of a circle given by its centre and a point on it.
};

/// Where a dimension point is attached so it follows the part (associativity). It names a span
/// of a vector object's contour; the point is that span's start, end or arc centre. When the
/// object or span is gone, or the span has no such point, the stored position is used instead.
struct DimensionAnchor {
    enum class Which { Start, End, Centre };
    ObjectId object{};
    std::uint32_t contour{0};
    std::uint32_t span{0};
    Which which{Which::Start};

    bool operator==(const DimensionAnchor&) const = default;
};

/// One defining point: its last known position in the sheet (mm) and, optionally, the anchor
/// that re-resolves it when the part it measures changes.
struct DimensionPoint {
    geometry::Vec2 pos{};
    std::optional<DimensionAnchor> anchor;

    bool operator==(const DimensionPoint&) const = default;
};

/// A dimension (T-0246). The defining points are `points[0..1]` for linear/aligned, vertex and
/// the two arm points for angular, and centre plus a point on the circle for radial. `offsetMm`
/// places the annotation: linear/aligned, signed distance of the dimension line from the midpoint
/// of the two points (positive to the left of point 0 -> point 1; for linear, towards +Y, or -X
/// when `vertical`); angular, radius of the dimension arc; radial, length of the leader beyond
/// the circle. The measured value is never stored: it is computed from the (resolved) points, so
/// the text follows scaling and moving of the part.
struct DimensionObject {
    DimensionKind kind{DimensionKind::Aligned};
    bool vertical{false};  ///< Linear: measure along Y instead of X.
    bool diameter{false};  ///< Radial: show the diameter instead of the radius.
    std::array<DimensionPoint, 3> points{};
    double offsetMm{5.0};
    double textHeightMm{3.0};
    int decimals{2};       ///< Lengths and radii, 0..8.
    int angleDecimals{2};  ///< Angles, 0..6.
    bool useCustomText{false};
    std::string customText;  ///< Shown instead of the measurement when `useCustomText`.
    std::string fontKey;     ///< Informational; empty selects the application's default font.

    bool operator==(const DimensionObject&) const = default;

    /// Maps the stored positions by `m`; the offset follows (scaled by a conformal transform's
    /// scale, and re-signed for mirrors so the annotation stays on the same side of the part).
    void transform(const geometry::Transform2& m);
};

/// Tagged object payload.
using ObjectData = std::variant<VectorObject, BitmapObject, GroupObject, DimensionObject>;

/// Discriminator for `Object::kind()`.
enum class ObjectKind { Vector, Bitmap, Group, Dimension };

/// A drawing object: identity, where it lives (layer, sheet, parent group) and its payload.
struct Object {
    ObjectId id{};
    LayerId layer{};
    SheetId sheet{};
    std::optional<ObjectId> parent{};
    std::string name;
    ObjectData data{VectorObject{}};

    bool operator==(const Object&) const = default;

    ObjectKind kind() const { return static_cast<ObjectKind>(data.index()); }
    bool isVector() const { return std::holds_alternative<VectorObject>(data); }
    bool isBitmap() const { return std::holds_alternative<BitmapObject>(data); }
    bool isGroup() const { return std::holds_alternative<GroupObject>(data); }
    bool isDimension() const { return std::holds_alternative<DimensionObject>(data); }

    VectorObject* asVector() { return std::get_if<VectorObject>(&data); }
    const VectorObject* asVector() const { return std::get_if<VectorObject>(&data); }
    BitmapObject* asBitmap() { return std::get_if<BitmapObject>(&data); }
    const BitmapObject* asBitmap() const { return std::get_if<BitmapObject>(&data); }
    GroupObject* asGroup() { return std::get_if<GroupObject>(&data); }
    const GroupObject* asGroup() const { return std::get_if<GroupObject>(&data); }
    DimensionObject* asDimension() { return std::get_if<DimensionObject>(&data); }
    const DimensionObject* asDimension() const { return std::get_if<DimensionObject>(&data); }

    /// Convenience: a vector object with one contour and no parameters.
    static Object vector(geometry::Contour contour, LayerId layerId, SheetId sheetId,
                         CreationParams params = NoParams{});
    /// Convenience: a vector object with several contours.
    static Object vector(std::vector<geometry::Contour> contours, LayerId layerId, SheetId sheetId,
                         CreationParams params = NoParams{});
};

}  // namespace carvetto::document
