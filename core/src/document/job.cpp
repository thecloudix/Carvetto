#include "carvetto/document/job.hpp"

namespace carvetto::document {

using geometry::Box2;
using geometry::Vec2;

Vec2 datumAnchor(Datum datum, double width, double height) {
    switch (datum) {
        case Datum::BottomLeft:
            return {0.0, 0.0};
        case Datum::BottomRight:
            return {width, 0.0};
        case Datum::TopLeft:
            return {0.0, height};
        case Datum::TopRight:
            return {width, height};
        case Datum::Centre:
            return {width * 0.5, height * 0.5};
    }
    return {0.0, 0.0};
}

Box2 materialBounds(Datum datum, double width, double height, Vec2 datumOffset) {
    const Vec2 bottomLeft = datumOffset - datumAnchor(datum, width, height);
    return Box2{bottomLeft, bottomLeft + Vec2{width, height}};
}

Job Job::singleSided(double width, double height, double thickness, Units units, Datum datumPos, ZZero zero) {
    Job job;
    job.type = JobType::SingleSided;
    job.displayUnits = units;
    job.sizeMm = {fromDisplay(width, units), fromDisplay(height, units), fromDisplay(thickness, units)};
    job.datum = datumPos;
    job.zZero = zero;
    return job;
}

Job Job::doubleSided(double width, double height, double thickness, Units units, Datum datumPos, ZZero zero,
                     FlipDirection flip) {
    Job job = singleSided(width, height, thickness, units, datumPos, zero);
    job.type = JobType::DoubleSided;
    job.flipDirection = flip;
    return job;
}

Job Job::rotary(double length, double diameter, Units units, RotaryAxis axis, ZZero zero, bool flip) {
    Job job;
    job.type = JobType::Rotary;
    job.displayUnits = units;
    job.rotaryAxis = axis;
    job.zZero = zero;
    job.flipDesign = flip;
    job.setRotaryMm(fromDisplay(length, units), fromDisplay(diameter, units));
    return job;
}

void Job::setRotaryMm(double lengthMmValue, double diameterMmValue) {
    diameterMm = diameterMmValue;
    const double circumference = geometry::kPi * diameterMmValue;
    if (rotaryAxis == RotaryAxis::X) {
        sizeMm = {lengthMmValue, circumference, diameterMmValue};
    } else {
        sizeMm = {circumference, lengthMmValue, diameterMmValue};
    }
}

JobSize Job::displaySize() const {
    return {toDisplay(sizeMm.x, displayUnits), toDisplay(sizeMm.y, displayUnits),
            toDisplay(sizeMm.z, displayUnits)};
}

void Job::setDisplaySize(const JobSize& size) {
    sizeMm = {fromDisplay(size.x, displayUnits), fromDisplay(size.y, displayUnits),
              fromDisplay(size.z, displayUnits)};
}

Vec2 Job::displayDatumOffset() const {
    return {toDisplay(datumOffsetMm.x, displayUnits), toDisplay(datumOffsetMm.y, displayUnits)};
}

Box2 Job::materialBoundsMm() const {
    return materialBounds(datum, sizeMm.x, sizeMm.y, datumOffsetMm);
}

bool Job::isValid() const {
    if (!(sizeMm.x > 0.0 && sizeMm.y > 0.0 && sizeMm.z > 0.0)) {
        return false;
    }
    if (type == JobType::Rotary) {
        return diameterMm > 0.0 && gapOutsideMm >= 0.0 && gapInsideMm >= 0.0 && modelThicknessMm() > 0.0;
    }
    return zZero != ZZero::CylinderAxis;
}

}  // namespace carvetto::document
