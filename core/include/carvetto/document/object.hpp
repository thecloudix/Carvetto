#pragma once

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

/// Tagged set of creation parameters.
using CreationParams =
    std::variant<NoParams, RectangleParams, CircleParams, EllipseParams, PolygonParams, StarParams>;

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

/// Tagged object payload.
using ObjectData = std::variant<VectorObject, BitmapObject, GroupObject>;

/// Discriminator for `Object::kind()`.
enum class ObjectKind { Vector, Bitmap, Group };

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

    VectorObject* asVector() { return std::get_if<VectorObject>(&data); }
    const VectorObject* asVector() const { return std::get_if<VectorObject>(&data); }
    BitmapObject* asBitmap() { return std::get_if<BitmapObject>(&data); }
    const BitmapObject* asBitmap() const { return std::get_if<BitmapObject>(&data); }
    GroupObject* asGroup() { return std::get_if<GroupObject>(&data); }
    const GroupObject* asGroup() const { return std::get_if<GroupObject>(&data); }

    /// Convenience: a vector object with one contour and no parameters.
    static Object vector(geometry::Contour contour, LayerId layer, SheetId sheet,
                         CreationParams params = NoParams{});
    /// Convenience: a vector object with several contours.
    static Object vector(std::vector<geometry::Contour> contours, LayerId layer, SheetId sheet,
                         CreationParams params = NoParams{});
};

}  // namespace carvetto::document
