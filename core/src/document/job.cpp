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

Job Job::singleSided(double width, double height, double thickness, Units units, Datum datum, ZZero zZero) {
    Job job;
    job.type = JobType::SingleSided;
    job.displayUnits = units;
    job.sizeMm = {fromDisplay(width, units), fromDisplay(height, units), fromDisplay(thickness, units)};
    job.datum = datum;
    job.zZero = zZero;
    return job;
}

Job Job::doubleSided(double width, double height, double thickness, Units units, Datum datum, ZZero zZero,
                     FlipDirection flip) {
    Job job = singleSided(width, height, thickness, units, datum, zZero);
    job.type = JobType::DoubleSided;
    job.flipDirection = flip;
    return job;
}

Job Job::rotary(double length, double diameter, Units units, RotaryAxis axis, ZZero zZero) {
    Job job;
    job.type = JobType::Rotary;
    job.displayUnits = units;
    job.rotaryAxis = axis;
    job.zZero = zZero;
    job.diameterMm = fromDisplay(diameter, units);
    const double lengthMm = fromDisplay(length, units);
    const double circumferenceMm = geometry::kPi * job.diameterMm;
    if (axis == RotaryAxis::X) {
        job.sizeMm = {lengthMm, circumferenceMm, job.diameterMm};
    } else {
        job.sizeMm = {circumferenceMm, lengthMm, job.diameterMm};
    }
    return job;
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
        return diameterMm > 0.0;
    }
    return zZero != ZZero::CylinderAxis;
}

}  // namespace carvetto::document
