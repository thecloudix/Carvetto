#include "carvetto/viewmodel/new_job_form.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <limits>

using namespace carvetto::document;
using carvetto::viewmodel::convertFormUnits;
using carvetto::viewmodel::formFromJob;
using carvetto::viewmodel::NewJobField;
using carvetto::viewmodel::NewJobForm;
using carvetto::viewmodel::validateNewJob;
using Catch::Matchers::WithinAbs;

namespace {

bool hasError(const carvetto::viewmodel::NewJobResult& r, NewJobField f) {
    for (const auto& e : r.errors) {
        if (e.field == f) {
            return true;
        }
    }
    return false;
}

}  // namespace

TEST_CASE("AC3: valid input gives exactly Job::singleSided in both units", "[viewmodel][newjob]") {
    const Units units = GENERATE(Units::Millimetres, Units::Inches);
    const Datum datum = GENERATE(Datum::BottomLeft, Datum::TopRight, Datum::Centre);
    const ZZero zero = GENERATE(ZZero::MaterialSurface, ZZero::MachineBed);
    NewJobForm form;
    form.units = units;
    form.width = 24.5;
    form.height = 12.25;
    form.thickness = 0.75;
    form.datum = datum;
    form.zZero = zero;
    const auto result = validateNewJob(form);
    REQUIRE(result.ok());
    REQUIRE(result.errors.empty());
    CHECK(*result.job == Job::singleSided(24.5, 12.25, 0.75, units, datum, zero));
}

TEST_CASE("AC3: zero, negative and non-finite sizes are rejected", "[viewmodel][newjob]") {
    const double bad = GENERATE(0.0, -1.0, -0.0001, std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::infinity());
    const Units units = GENERATE(Units::Millimetres, Units::Inches);
    NewJobForm form;
    form.units = units;
    SECTION("width") {
        form.width = bad;
        const auto r = validateNewJob(form);
        CHECK_FALSE(r.ok());
        CHECK(hasError(r, NewJobField::Width));
    }
    SECTION("height") {
        form.height = bad;
        const auto r = validateNewJob(form);
        CHECK_FALSE(r.ok());
        CHECK(hasError(r, NewJobField::Height));
    }
    SECTION("thickness") {
        form.thickness = bad;
        const auto r = validateNewJob(form);
        CHECK_FALSE(r.ok());
        CHECK(hasError(r, NewJobField::Thickness));
    }
    SECTION("rotary length and diameter") {
        form.type = JobType::Rotary;
        form.zZero = ZZero::CylinderAxis;
        form.length = bad;
        form.diameter = bad;
        const auto r = validateNewJob(form);
        CHECK_FALSE(r.ok());
        CHECK(hasError(r, NewJobField::Length));
        CHECK(hasError(r, NewJobField::Diameter));
    }
}

TEST_CASE("all errors are reported together", "[viewmodel][newjob]") {
    NewJobForm form;
    form.width = 0.0;
    form.height = -2.0;
    form.thickness = 0.0;
    form.zZero = ZZero::CylinderAxis;
    const auto r = validateNewJob(form);
    CHECK(r.errors.size() == 4);
    CHECK_FALSE(r.errors.front().message.empty());
}

TEST_CASE("double-sided, rotary and offset forms map to the core factories", "[viewmodel][newjob]") {
    NewJobForm form;
    form.units = Units::Inches;
    form.type = JobType::DoubleSided;
    form.width = 10.0;
    form.height = 5.0;
    form.thickness = 1.0;
    form.flip = FlipDirection::Vertical;
    auto r = validateNewJob(form);
    REQUIRE(r.ok());
    CHECK(*r.job == Job::doubleSided(10.0, 5.0, 1.0, Units::Inches, Datum::BottomLeft, ZZero::MaterialSurface,
                                     FlipDirection::Vertical));

    form.type = JobType::SingleSided;
    form.useOffset = true;
    form.offsetX = 1.0;
    form.offsetY = -2.0;
    r = validateNewJob(form);
    REQUIRE(r.ok());
    CHECK_THAT(r.job->datumOffsetMm.x, WithinAbs(25.4, 1e-12));
    CHECK_THAT(r.job->datumOffsetMm.y, WithinAbs(-50.8, 1e-12));
    form.offsetX = std::numeric_limits<double>::quiet_NaN();
    CHECK(hasError(validateNewJob(form), NewJobField::OffsetX));

    NewJobForm rot;
    rot.type = JobType::Rotary;
    rot.units = Units::Millimetres;
    rot.length = 400.0;
    rot.diameter = 60.0;
    rot.rotaryAxis = RotaryAxis::Y;
    rot.zZero = ZZero::CylinderAxis;
    r = validateNewJob(rot);
    REQUIRE(r.ok());
    CHECK(*r.job == Job::rotary(400.0, 60.0, Units::Millimetres, RotaryAxis::Y, ZZero::CylinderAxis));
    rot.zZero = ZZero::MachineBed;
    CHECK(hasError(validateNewJob(rot), NewJobField::ZZero));
}

TEST_CASE("switching units converts every length", "[viewmodel][newjob]") {
    NewJobForm form;
    form.width = 254.0;
    form.height = 127.0;
    form.thickness = 12.7;
    form.offsetX = 25.4;
    const NewJobForm inches = convertFormUnits(form, Units::Inches);
    CHECK(inches.units == Units::Inches);
    CHECK_THAT(inches.width, WithinAbs(10.0, 1e-12));
    CHECK_THAT(inches.height, WithinAbs(5.0, 1e-12));
    CHECK_THAT(inches.thickness, WithinAbs(0.5, 1e-12));
    CHECK_THAT(inches.offsetX, WithinAbs(1.0, 1e-12));
    CHECK(convertFormUnits(form, Units::Millimetres) == form);
    // Both forms describe the same material.
    CHECK_THAT(validateNewJob(inches).job->sizeMm.x, WithinAbs(validateNewJob(form).job->sizeMm.x, 1e-9));
}

TEST_CASE("formFromJob round-trips through validation", "[viewmodel][newjob]") {
    const Job flat = Job::singleSided(10.0, 8.0, 0.75, Units::Inches, Datum::TopLeft, ZZero::MachineBed);
    const auto r = validateNewJob(formFromJob(flat));
    REQUIRE(r.ok());
    CHECK(r.job->type == flat.type);
    CHECK(r.job->datum == flat.datum);
    CHECK(r.job->zZero == flat.zZero);
    CHECK_THAT(r.job->sizeMm.x, WithinAbs(flat.sizeMm.x, 1e-9));
    CHECK_THAT(r.job->sizeMm.z, WithinAbs(flat.sizeMm.z, 1e-9));

    const Job rot = Job::rotary(300.0, 50.0, Units::Millimetres, RotaryAxis::Y);
    const auto rr = validateNewJob(formFromJob(rot));
    REQUIRE(rr.ok());
    CHECK_THAT(rr.job->diameterMm, WithinAbs(50.0, 1e-9));
    CHECK_THAT(rr.job->sizeMm.y, WithinAbs(300.0, 1e-9));
}
