// T-0319: rotary Job Setup, Material Setup and Edit Sheet (ADR-0098).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <optional>
#include <span>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/viewmodel/double_sided_ops.hpp"
#include "carvetto/viewmodel/layers_sheets.hpp"
#include "carvetto/viewmodel/new_job_form.hpp"
#include "carvetto/viewmodel/rotary_material.hpp"

using namespace carvetto::document;
using namespace carvetto::viewmodel;
using Catch::Approx;

namespace {

SheetMaterialEdit editLength(double mm) {
    SheetMaterialEdit e;
    e.lengthMm = mm;
    return e;
}

SheetMaterialEdit editWidth(double mm) {
    SheetMaterialEdit e;
    e.widthMm = mm;
    return e;
}

SheetMaterialEdit editThickness(double mm) {
    SheetMaterialEdit e;
    e.thicknessMm = mm;
    return e;
}

SheetMaterialEdit editZero(ZZero z, std::optional<double> lengthMm = std::nullopt) {
    SheetMaterialEdit e;
    e.zZero = z;
    e.lengthMm = lengthMm;
    return e;
}

Document rotaryDoc() {
    return Document(Job::rotary(400.0, 60.0, Units::Millimetres, RotaryAxis::X, ZZero::CylinderAxis));
}

}  // namespace

TEST_CASE("rotary job form carries flip design and round-trips through the job", "[viewmodel][rotary]") {
    NewJobForm form;
    form.type = JobType::Rotary;
    form.zZero = ZZero::CylinderAxis;
    form.length = 12.0;
    form.diameter = 2.0;
    form.units = Units::Inches;
    form.rotaryAxis = RotaryAxis::Y;
    form.flipDesign = true;
    const NewJobResult r = validateNewJob(form);
    REQUIRE(r.ok());
    CHECK(r.job->flipDesign);
    CHECK(*r.job == Job::rotary(12.0, 2.0, Units::Inches, RotaryAxis::Y, ZZero::CylinderAxis, true));
    CHECK(r.job->sizeMm.y == Approx(304.8));
    CHECK(r.job->sizeMm.x == Approx(carvetto::geometry::kPi * 50.8));
    const NewJobForm back = formFromJob(*r.job);
    CHECK(back.length == Approx(12.0));
    CHECK(back.diameter == Approx(2.0));
    CHECK(back.rotaryAxis == RotaryAxis::Y);
    CHECK(back.flipDesign);
    CHECK(back.zZero == ZZero::CylinderAxis);
}

TEST_CASE("Job Setup on a rotary job resizes every sheet and keeps the gaps", "[viewmodel][rotary]") {
    Document doc = rotaryDoc();
    REQUIRE(applyRotaryMaterial(doc, {Units::Millimetres, 60.0, 2.0, 3.0, ZZero::CylinderAxis}).ok);
    REQUIRE(addSheet(doc, "Second").ok);
    REQUIRE(doc.sheets().size() == 2);

    Job job = Job::rotary(500.0, 80.0, Units::Millimetres, RotaryAxis::Y, ZZero::MaterialSurface, true);
    const std::size_t steps = doc.undoStack().count();
    REQUIRE(applyJobSetup(doc, job).ok);
    CHECK(doc.undoStack().count() == steps + 1);
    CHECK(doc.job().gapOutsideMm == 2.0);
    CHECK(doc.job().gapInsideMm == 3.0);
    CHECK(doc.job().rotaryAxis == RotaryAxis::Y);
    for (const Sheet& s : doc.sheets()) {
        CHECK(s.sizeMm.y == Approx(500.0));
        CHECK(s.sizeMm.x == Approx(carvetto::geometry::kPi * 80.0));
        CHECK(s.sizeMm.z == Approx(80.0));
        CHECK(s.zZero == ZZero::MaterialSurface);
    }
    CHECK(sheetTopZ(doc, doc.activeSheet()) == 0.0);
    REQUIRE(doc.undo());
    CHECK(doc.job().rotaryAxis == RotaryAxis::X);
    CHECK(doc.sheets()[0].sizeMm.x == Approx(400.0));
    CHECK(doc.sheets()[1].sizeMm.z == Approx(60.0));

    // A cylinder too small for the gaps is refused and changes nothing.
    const std::size_t before = doc.undoStack().count();
    CHECK_FALSE(applyJobSetup(doc, Job::rotary(400.0, 8.0, Units::Millimetres)).ok);
    CHECK(doc.undoStack().count() == before);
    CHECK(doc.job().diameterMm == Approx(60.0));
}

TEST_CASE("Material Setup edits diameter, gaps and Z-zero as one undo step", "[viewmodel][rotary]") {
    Document doc = rotaryDoc();
    REQUIRE(addSheet(doc, "Second").ok);
    REQUIRE(setSheetMaterial(doc, doc.sheets()[1].id, editLength(250.0)).ok);
    CHECK(doc.sheets()[1].sizeMm.x == Approx(250.0));

    const auto form = rotaryMaterialFromJob(doc.job());
    REQUIRE(form.has_value());
    CHECK(form->diameter == Approx(60.0));
    RotaryMaterialForm edit = *form;
    edit.diameter = 100.0;
    edit.gapOutside = 4.0;
    edit.gapInside = 6.0;
    edit.zZero = ZZero::MaterialSurface;
    CHECK(rotaryModelThickness(edit) == Approx(40.0));
    const std::size_t steps = doc.undoStack().count();
    REQUIRE(applyRotaryMaterial(doc, edit).ok);
    CHECK(doc.undoStack().count() == steps + 1);
    const Job& job = doc.job();
    CHECK(job.diameterMm == Approx(100.0));
    CHECK(job.lengthMm() == Approx(400.0));
    CHECK(job.modelThicknessMm() == Approx(40.0));
    CHECK(job.rotaryModelTopZ() == Approx(-4.0));
    CHECK(job.rotaryModelBottomZ() == Approx(-44.0));
    CHECK(doc.sheets()[0].sizeMm.y == Approx(carvetto::geometry::kPi * 100.0));
    CHECK(doc.sheets()[0].sizeMm.z == Approx(100.0));
    CHECK(doc.sheets()[1].sizeMm.x == Approx(250.0));  // each sheet keeps its own length
    CHECK(doc.sheets()[1].zZero == ZZero::MaterialSurface);

    CHECK(applyRotaryMaterial(doc, edit).changed == false);  // no change, no step
    REQUIRE(doc.undo());
    CHECK(doc.job().diameterMm == Approx(60.0));
    CHECK(doc.job().gapOutsideMm == 0.0);
    CHECK(doc.sheets()[1].sizeMm.z == Approx(60.0));
    REQUIRE(doc.redo());
    CHECK(doc.job().gapInsideMm == Approx(6.0));
}

TEST_CASE("Material Setup validation", "[viewmodel][rotary]") {
    RotaryMaterialForm form{Units::Millimetres, 50.0, 0.0, 0.0, ZZero::CylinderAxis};
    CHECK(validateRotaryMaterial(form).empty());
    form.gapOutside = 12.5;
    form.gapInside = 12.5;  // sums to the radius
    CHECK_FALSE(validateRotaryMaterial(form).empty());
    form.gapInside = 12.0;
    CHECK(validateRotaryMaterial(form).empty());
    form.gapOutside = -1.0;
    CHECK_FALSE(validateRotaryMaterial(form).empty());
    form.gapOutside = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(validateRotaryMaterial(form).empty());
    form = {Units::Millimetres, 0.0, 0.0, 0.0, ZZero::CylinderAxis};
    CHECK_FALSE(validateRotaryMaterial(form).empty());
    form = {Units::Millimetres, std::numeric_limits<double>::infinity(), 0.0, 0.0, ZZero::CylinderAxis};
    CHECK_FALSE(validateRotaryMaterial(form).empty());
    form = {Units::Millimetres, 50.0, 0.0, 0.0, ZZero::MachineBed};
    CHECK_FALSE(validateRotaryMaterial(form).empty());

    Document flat(Job::singleSided(100.0, 100.0, 10.0, Units::Millimetres));
    CHECK_FALSE(rotaryMaterialFromJob(flat.job()).has_value());
    const std::size_t steps = flat.undoStack().count();
    CHECK_FALSE(applyRotaryMaterial(flat, {Units::Millimetres, 50.0, 0.0, 0.0, ZZero::CylinderAxis}).ok);
    CHECK(flat.undoStack().count() == steps);

    Document rot = rotaryDoc();
    CHECK_FALSE(applyRotaryMaterial(rot, {Units::Millimetres, 50.0, 30.0, 0.0, ZZero::CylinderAxis}).ok);
    CHECK(rot.job().diameterMm == Approx(60.0));
}

TEST_CASE("Edit Sheet for a rotary sheet", "[viewmodel][rotary][sheets]") {
    Document doc = rotaryDoc();
    const SheetId id = doc.sheets()[0].id;
    CHECK_FALSE(setSheetMaterial(doc, id, editWidth(100.0)).ok);
    CHECK_FALSE(setSheetMaterial(doc, id, editThickness(100.0)).ok);
    CHECK_FALSE(setSheetMaterial(doc, id, editZero(ZZero::MachineBed)).ok);
    CHECK_FALSE(setSheetMaterial(doc, id, editLength(-5.0)).ok);
    CHECK_FALSE(setSheetMaterial(doc, id, editLength(1e12)).ok);
    REQUIRE(setSheetMaterial(doc, id, editZero(ZZero::MaterialSurface, 150.0)).ok);
    CHECK(doc.sheets()[0].sizeMm.x == Approx(150.0));
    CHECK(doc.sheets()[0].sizeMm.y == Approx(carvetto::geometry::kPi * 60.0));
    CHECK(doc.sheets()[0].sizeMm.z == Approx(60.0));
    CHECK(sheetTopZ(doc, id) == 0.0);
    REQUIRE(setSheetMaterial(doc, id, editZero(ZZero::CylinderAxis)).ok);
    CHECK(sheetTopZ(doc, id) == Approx(30.0));

    Document flat(Job::singleSided(100.0, 100.0, 10.0, Units::Millimetres));
    CHECK_FALSE(setSheetMaterial(flat, flat.sheets()[0].id, editLength(5.0)).ok);
    CHECK_FALSE(setSheetMaterial(flat, flat.sheets()[0].id, editZero(ZZero::CylinderAxis)).ok);
}

TEST_CASE("a rotary job with sheets, gaps and flip design saves and reloads whole", "[viewmodel][rotary][carv]") {
    Document doc(Job::rotary(300.0, 70.0, Units::Millimetres, RotaryAxis::Y, ZZero::CylinderAxis, true));
    REQUIRE(applyRotaryMaterial(doc, {Units::Millimetres, 80.0, 5.0, 10.0, ZZero::MaterialSurface}).ok);
    REQUIRE(addSheet(doc, "Second").ok);
    REQUIRE(setSheetMaterial(doc, doc.sheets()[1].id, editLength(120.0)).ok);
    const auto saved = saveCarv(doc, CarvExtras{});
    REQUIRE(saved.ok());
    const auto loaded = loadCarv(std::span<const std::uint8_t>(saved.value()));
    REQUIRE(loaded.ok());
    const Document& back = loaded.value().document;
    CHECK(back.job() == doc.job());
    REQUIRE(back.sheets().size() == 2);
    CHECK(back.sheets()[1].sizeMm == doc.sheets()[1].sizeMm);
    CHECK(back.job().flipDesign);
    CHECK(back.job().modelThicknessMm() == Approx(25.0));
}
