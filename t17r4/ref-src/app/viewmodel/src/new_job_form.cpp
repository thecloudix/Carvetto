#include "carvetto/viewmodel/new_job_form.hpp"

#include <cmath>

namespace carvetto::viewmodel {

using document::Job;
using document::JobType;
using document::Units;
using document::ZZero;

namespace {

void requirePositive(double value, NewJobField field, const char* name, std::vector<NewJobError>& errors) {
    if (!std::isfinite(value) || !(value > 0.0)) {
        errors.push_back({field, std::string(name) + " must be greater than zero."});
    }
}

double convert(double value, Units from, Units to) {
    return document::toDisplay(document::fromDisplay(value, from), to);
}

}  // namespace

NewJobResult validateNewJob(const NewJobForm& form) {
    NewJobResult result;
    auto& errors = result.errors;
    if (form.type == JobType::Rotary) {
        requirePositive(form.length, NewJobField::Length, "Length", errors);
        requirePositive(form.diameter, NewJobField::Diameter, "Diameter", errors);
        if (form.zZero == ZZero::MachineBed) {
            errors.push_back(
                {NewJobField::ZZero, "A rotary job zeroes on the material surface or the cylinder axis."});
        }
    } else {
        requirePositive(form.width, NewJobField::Width, "Width", errors);
        requirePositive(form.height, NewJobField::Height, "Height", errors);
        requirePositive(form.thickness, NewJobField::Thickness, "Thickness", errors);
        if (form.zZero == ZZero::CylinderAxis) {
            errors.push_back({NewJobField::ZZero, "Only a rotary job can zero on the cylinder axis."});
        }
    }
    if (form.useOffset && form.type != JobType::Rotary) {
        if (!std::isfinite(form.offsetX)) {
            errors.push_back({NewJobField::OffsetX, "Datum offset X must be a number."});
        }
        if (!std::isfinite(form.offsetY)) {
            errors.push_back({NewJobField::OffsetY, "Datum offset Y must be a number."});
        }
    }
    if (!errors.empty()) {
        return result;
    }

    Job job;
    switch (form.type) {
        case JobType::SingleSided:
            job =
                Job::singleSided(form.width, form.height, form.thickness, form.units, form.datum, form.zZero);
            break;
        case JobType::DoubleSided:
            job = Job::doubleSided(form.width, form.height, form.thickness, form.units, form.datum,
                                   form.zZero, form.flip);
            break;
        case JobType::Rotary:
            job = Job::rotary(form.length, form.diameter, form.units, form.rotaryAxis, form.zZero);
            break;
    }
    if (form.useOffset && form.type != JobType::Rotary) {
        job.datumOffsetMm = {document::fromDisplay(form.offsetX, form.units),
                             document::fromDisplay(form.offsetY, form.units)};
    }
    result.job = job;
    return result;
}

NewJobForm convertFormUnits(const NewJobForm& form, Units units) {
    NewJobForm out = form;
    out.units = units;
    if (units == form.units) {
        return out;
    }
    out.width = convert(form.width, form.units, units);
    out.height = convert(form.height, form.units, units);
    out.thickness = convert(form.thickness, form.units, units);
    out.length = convert(form.length, form.units, units);
    out.diameter = convert(form.diameter, form.units, units);
    out.offsetX = convert(form.offsetX, form.units, units);
    out.offsetY = convert(form.offsetY, form.units, units);
    return out;
}

NewJobForm formFromJob(const Job& job) {
    NewJobForm form;
    form.type = job.type;
    form.units = job.displayUnits;
    form.zZero = job.zZero;
    form.datum = job.datum;
    form.flip = job.flipDirection;
    form.rotaryAxis = job.rotaryAxis;
    const document::JobSize size = job.displaySize();
    if (job.type == JobType::Rotary) {
        form.length = job.rotaryAxis == document::RotaryAxis::X ? size.x : size.y;
        form.diameter = job.displayDiameter();
    } else {
        form.width = size.x;
        form.height = size.y;
        form.thickness = size.z;
    }
    const geometry::Vec2 offset = job.displayDatumOffset();
    form.useOffset = offset.x != 0.0 || offset.y != 0.0;
    form.offsetX = offset.x;
    form.offsetY = offset.y;
    return form;
}

}  // namespace carvetto::viewmodel
