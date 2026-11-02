#pragma once

#include "carvetto/document/units.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/vec2.hpp"

namespace carvetto::document {

/// Kind of job: a flat single-sided blank, a flat blank machined from both sides, or a cylinder
/// on a rotary axis.
enum class JobType { SingleSided, DoubleSided, Rotary };

/// Where Z = 0 sits: the top of the material, the machine bed under it, or (rotary) the axis of
/// the cylinder.
enum class ZZero { MaterialSurface, MachineBed, CylinderAxis };

/// XY datum position on the material: a corner or the centre.
enum class Datum { BottomLeft, BottomRight, TopLeft, TopRight, Centre };

/// How a double-sided job is flipped between sides.
enum class FlipDirection { Horizontal, Vertical };

/// Machine axis the rotary cylinder lies along.
enum class RotaryAxis { X, Y };

/// Three extents in one unit system (mm internally, display units at the edges).
struct JobSize {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    constexpr bool operator==(const JobSize&) const = default;
};

/// Position of a datum relative to the bottom-left corner of a `width` × `height` rectangle.
geometry::Vec2 datumAnchor(Datum datum, double width, double height);

/// Material rectangle in document coordinates when the datum point is placed at `datumOffset`.
geometry::Box2 materialBounds(Datum datum, double width, double height, geometry::Vec2 datumOffset);

/// Job setup: type, material size, display units, Z-zero and XY datum. Sizes are millimetres.
/// For a rotary job `sizeMm.x`/`.y` hold the unwrapped flat design area (length along the axis,
/// circumference around it) and `sizeMm.z` equals the diameter.
struct Job {
    JobType type{JobType::SingleSided};
    JobSize sizeMm{300.0, 300.0, 18.0};
    Units displayUnits{Units::Millimetres};
    ZZero zZero{ZZero::MaterialSurface};
    Datum datum{Datum::BottomLeft};
    /// Document coordinates of the datum point ("Use Offset"); (0,0) by default.
    geometry::Vec2 datumOffsetMm{};

    // Double-sided only.
    FlipDirection flipDirection{FlipDirection::Horizontal};
    bool zeroOffSameSide{false};

    // Rotary only.
    double diameterMm{0.0};
    RotaryAxis rotaryAxis{RotaryAxis::X};
    bool flipDesign{false};

    bool operator==(const Job&) const = default;

    /// Flat single-sided job from values in `units`.
    static Job singleSided(double width, double height, double thickness, Units units,
                           Datum datumPos = Datum::BottomLeft, ZZero zero = ZZero::MaterialSurface);
    /// Flat double-sided job from values in `units`.
    static Job doubleSided(double width, double height, double thickness, Units units,
                           Datum datumPos = Datum::BottomLeft, ZZero zero = ZZero::MaterialSurface,
                           FlipDirection flip = FlipDirection::Horizontal);
    /// Rotary job from a length and diameter in `units`; the flat design area is length × π·diameter.
    static Job rotary(double length, double diameter, Units units, RotaryAxis axis = RotaryAxis::X,
                      ZZero zero = ZZero::CylinderAxis);

    /// Size expressed in `displayUnits`.
    JobSize displaySize() const;
    /// Sets the size from values in `displayUnits`.
    void setDisplaySize(const JobSize& size);
    /// Rotary diameter in `displayUnits`.
    double displayDiameter() const { return toDisplay(diameterMm, displayUnits); }
    /// Datum offset in `displayUnits`.
    geometry::Vec2 displayDatumOffset() const;

    /// Position of the datum relative to the material's bottom-left corner, in mm.
    geometry::Vec2 datumAnchorMm() const { return datumAnchor(datum, sizeMm.x, sizeMm.y); }
    /// Material rectangle in document coordinates (the datum point sits at `datumOffsetMm`).
    geometry::Box2 materialBoundsMm() const;

    /// True when every extent is positive and type-specific fields are consistent.
    bool isValid() const;
};

}  // namespace carvetto::document
