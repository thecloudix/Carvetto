#include "carvetto/document/job.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "carvetto/document/sheet.hpp"
#include "carvetto/document/units.hpp"

using namespace carvetto::document;
using carvetto::geometry::Box2;
using carvetto::geometry::Vec2;
using Catch::Matchers::WithinAbs;

TEST_CASE("units convert between mm and inches", "[document][units]") {
    CHECK_THAT(fromDisplay(1.0, Units::Inches), WithinAbs(25.4, 1e-12));
    CHECK_THAT(toDisplay(25.4, Units::Inches), WithinAbs(1.0, 1e-12));
    CHECK_THAT(fromDisplay(7.5, Units::Millimetres), WithinAbs(7.5, 1e-12));
    CHECK_THAT(toDisplay(7.5, Units::Millimetres), WithinAbs(7.5, 1e-12));
    CHECK(unitSuffix(Units::Inches) == "in");
    CHECK(unitSuffix(Units::Millimetres) == "mm");
}

TEST_CASE("AC1: a 10x10x0.5 in job is stored in mm and queried in inches", "[document][job]") {
    const Job job = Job::singleSided(10.0, 10.0, 0.5, Units::Inches, Datum::BottomLeft);

    CHECK(job.type == JobType::SingleSided);
    CHECK_THAT(job.sizeMm.x, WithinAbs(254.0, 1e-12));
    CHECK_THAT(job.sizeMm.y, WithinAbs(254.0, 1e-12));
    CHECK_THAT(job.sizeMm.z, WithinAbs(12.7, 1e-12));
    CHECK(job.datum == Datum::BottomLeft);
    CHECK(job.displayUnits == Units::Inches);

    const JobSize shown = job.displaySize();
    CHECK_THAT(shown.x, WithinAbs(10.0, 1e-12));
    CHECK_THAT(shown.y, WithinAbs(10.0, 1e-12));
    CHECK_THAT(shown.z, WithinAbs(0.5, 1e-12));

    // Datum bottom-left at (0,0): the material spans [0,254]^2.
    const Box2 bounds = job.materialBoundsMm();
    CHECK(bounds.min == Vec2{0.0, 0.0});
    CHECK_THAT(bounds.max.x, WithinAbs(254.0, 1e-12));
    CHECK_THAT(bounds.max.y, WithinAbs(254.0, 1e-12));
    CHECK(job.isValid());
}

TEST_CASE("setDisplaySize converts from the display unit", "[document][job]") {
    Job job = Job::singleSided(100.0, 50.0, 10.0, Units::Millimetres);
    job.displayUnits = Units::Inches;
    job.setDisplaySize({2.0, 1.0, 0.25});
    CHECK_THAT(job.sizeMm.x, WithinAbs(50.8, 1e-12));
    CHECK_THAT(job.sizeMm.y, WithinAbs(25.4, 1e-12));
    CHECK_THAT(job.sizeMm.z, WithinAbs(6.35, 1e-12));
}

TEST_CASE("datum anchors and offsets place the material rectangle", "[document][job]") {
    CHECK(datumAnchor(Datum::BottomLeft, 100.0, 50.0) == Vec2{0.0, 0.0});
    CHECK(datumAnchor(Datum::BottomRight, 100.0, 50.0) == Vec2{100.0, 0.0});
    CHECK(datumAnchor(Datum::TopLeft, 100.0, 50.0) == Vec2{0.0, 50.0});
    CHECK(datumAnchor(Datum::TopRight, 100.0, 50.0) == Vec2{100.0, 50.0});
    CHECK(datumAnchor(Datum::Centre, 100.0, 50.0) == Vec2{50.0, 25.0});

    Job job = Job::singleSided(100.0, 50.0, 10.0, Units::Millimetres, Datum::Centre);
    CHECK(job.materialBoundsMm() == Box2{Vec2{-50.0, -25.0}, Vec2{50.0, 25.0}});

    // "Use Offset": the datum point sits at the offset, so the material shifts with it.
    job.datumOffsetMm = {10.0, 5.0};
    CHECK(job.materialBoundsMm() == Box2{Vec2{-40.0, -20.0}, Vec2{60.0, 30.0}});
    job.displayUnits = Units::Inches;
    CHECK_THAT(job.displayDatumOffset().x, WithinAbs(10.0 / 25.4, 1e-12));

    job.datum = Datum::TopRight;
    job.datumOffsetMm = {};
    CHECK(job.materialBoundsMm() == Box2{Vec2{-100.0, -50.0}, Vec2{0.0, 0.0}});
}

TEST_CASE("double-sided job keeps flip direction", "[document][job]") {
    const Job job = Job::doubleSided(100.0, 50.0, 10.0, Units::Millimetres, Datum::BottomLeft,
                                     ZZero::MachineBed, FlipDirection::Vertical);
    CHECK(job.type == JobType::DoubleSided);
    CHECK(job.flipDirection == FlipDirection::Vertical);
    CHECK(job.zZero == ZZero::MachineBed);
    CHECK_FALSE(job.zeroOffSameSide);
    CHECK(job.isValid());
}

TEST_CASE("rotary job unwraps length x circumference and stores the diameter", "[document][job]") {
    const Job job = Job::rotary(12.0, 2.0, Units::Inches, RotaryAxis::X);
    CHECK(job.type == JobType::Rotary);
    CHECK(job.zZero == ZZero::CylinderAxis);
    CHECK_THAT(job.diameterMm, WithinAbs(50.8, 1e-12));
    CHECK_THAT(job.displayDiameter(), WithinAbs(2.0, 1e-12));
    CHECK_THAT(job.sizeMm.x, WithinAbs(304.8, 1e-12));
    CHECK_THAT(job.sizeMm.y, WithinAbs(carvetto::geometry::kPi * 50.8, 1e-9));
    CHECK_THAT(job.sizeMm.z, WithinAbs(50.8, 1e-12));
    CHECK(job.isValid());

    const Job alongY = Job::rotary(300.0, 50.0, Units::Millimetres, RotaryAxis::Y);
    CHECK_THAT(alongY.sizeMm.y, WithinAbs(300.0, 1e-12));
    CHECK_THAT(alongY.sizeMm.x, WithinAbs(carvetto::geometry::kPi * 50.0, 1e-9));
}

TEST_CASE("job validity rejects non-positive sizes and a cylinder-axis zero on flat jobs",
          "[document][job]") {
    Job job = Job::singleSided(100.0, 50.0, 0.0, Units::Millimetres);
    CHECK_FALSE(job.isValid());
    job.sizeMm.z = 10.0;
    CHECK(job.isValid());
    job.zZero = ZZero::CylinderAxis;
    CHECK_FALSE(job.isValid());

    Job rotary = Job::rotary(100.0, 0.0, Units::Millimetres);
    CHECK_FALSE(rotary.isValid());
}

TEST_CASE("sheet takes size, Z-zero and datum from the job", "[document][sheet]") {
    const Job job = Job::singleSided(10.0, 8.0, 0.75, Units::Inches, Datum::Centre, ZZero::MachineBed);
    const Sheet sheet = Sheet::fromJob(job, "Front");
    CHECK(sheet.name == "Front");
    CHECK(sheet.sizeMm == job.sizeMm);
    CHECK(sheet.zZero == ZZero::MachineBed);
    CHECK(sheet.datum == Datum::Centre);
    CHECK(sheet.visible);
    CHECK(sheet.materialBoundsMm() == job.materialBoundsMm());
}
