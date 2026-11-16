#include "carvetto/viewmodel/rotary_material.hpp"

#include <cmath>
#include <memory>
#include <utility>

#include "carvetto/document/commands.hpp"
#include "carvetto/viewmodel/layers_sheets.hpp"

namespace carvetto::viewmodel {

namespace {

EditResult refusedWith(std::string message) {
    EditResult r;
    r.ok = false;
    r.message = std::move(message);
    return r;
}

bool finiteNonNegative(double v) {
    return std::isfinite(v) && v >= 0.0;
}

}  // namespace

std::optional<RotaryMaterialForm> rotaryMaterialFromJob(const document::Job& job) {
    if (job.type != document::JobType::Rotary) {
        return std::nullopt;
    }
    RotaryMaterialForm form;
    form.units = job.displayUnits;
    form.diameter = job.displayDiameter();
    form.gapOutside = document::toDisplay(job.gapOutsideMm, job.displayUnits);
    form.gapInside = document::toDisplay(job.gapInsideMm, job.displayUnits);
    form.zZero = job.zZero;
    return form;
}

std::vector<RotaryMaterialError> validateRotaryMaterial(const RotaryMaterialForm& form) {
    std::vector<RotaryMaterialError> errors;
    const double diameterMm = document::fromDisplay(form.diameter, form.units);
    const double outsideMm = document::fromDisplay(form.gapOutside, form.units);
    const double insideMm = document::fromDisplay(form.gapInside, form.units);
    bool sizesOk = true;
    if (!std::isfinite(diameterMm) || !(diameterMm > 0.0) || diameterMm > kMaxSheetSizeMm) {
        errors.push_back({RotaryMaterialField::Diameter, "The diameter must be a positive number."});
        sizesOk = false;
    }
    if (!finiteNonNegative(outsideMm)) {
        errors.push_back({RotaryMaterialField::GapOutside, "The gap outside the model must be zero or more."});
        sizesOk = false;
    }
    if (!finiteNonNegative(insideMm)) {
        errors.push_back({RotaryMaterialField::GapInside, "The gap inside the model must be zero or more."});
        sizesOk = false;
    }
    if (sizesOk && !(outsideMm + insideMm < 0.5 * diameterMm)) {
        errors.push_back({RotaryMaterialField::GapInside,
                          "The gaps leave no model thickness: they must add up to less than the radius."});
    }
    if (form.zZero == document::ZZero::MachineBed) {
        errors.push_back(
            {RotaryMaterialField::ZZero, "A rotary job zeroes on the cylinder surface or the cylinder axis."});
    }
    return errors;
}

double rotaryModelThickness(const RotaryMaterialForm& form) {
    return 0.5 * form.diameter - form.gapOutside - form.gapInside;
}

bool resizeRotarySheets(document::Document& doc, const document::Job& job, bool takeLength) {
    const std::vector<document::Sheet> sheets = doc.sheets();
    for (const document::Sheet& sheet : sheets) {
        document::Job shape = job;
        const double length =
            takeLength ? job.lengthMm() : (job.rotaryAxis == document::RotaryAxis::X ? sheet.sizeMm.x : sheet.sizeMm.y);
        shape.setRotaryMm(length, job.diameterMm);
        document::Sheet next = sheet;
        next.sizeMm = shape.sizeMm;
        next.zZero = job.zZero;
        if (next == sheet) {
            continue;
        }
        if (!doc.execute(std::make_unique<document::SetSheetPropertiesCommand>(std::move(next)))) {
            return false;
        }
    }
    return true;
}

EditResult applyRotaryMaterial(document::Document& doc, const RotaryMaterialForm& form) {
    if (doc.job().type != document::JobType::Rotary) {
        return refusedWith("Material Setup for a rotary job needs a rotary job");
    }
    const std::vector<RotaryMaterialError> errors = validateRotaryMaterial(form);
    if (!errors.empty()) {
        return refusedWith(errors.front().message);
    }
    document::Job job = doc.job();
    job.setRotaryMm(job.lengthMm(), document::fromDisplay(form.diameter, form.units));
    job.gapOutsideMm = document::fromDisplay(form.gapOutside, form.units);
    job.gapInsideMm = document::fromDisplay(form.gapInside, form.units);
    job.zZero = form.zZero;

    doc.undoStack().breakCoalescing();
    const std::size_t stepsBefore = doc.undoStack().count();
    doc.undoStack().beginMacro("Material setup");
    bool ok = job == doc.job() || doc.execute(std::make_unique<document::SetJobCommand>(job));
    ok = ok && resizeRotarySheets(doc, job, false);
    doc.undoStack().endMacro();
    if (!ok && doc.undoStack().count() > stepsBefore) {
        doc.undoStack().undoAndDiscard(doc);
    }
    doc.undoStack().breakCoalescing();
    if (!ok) {
        return refusedWith("the material cannot be changed");
    }
    EditResult r;
    r.changed = doc.undoStack().count() > stepsBefore;
    return r;
}

}  // namespace carvetto::viewmodel
