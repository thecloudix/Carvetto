#pragma once

#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/job.hpp"
#include "carvetto/document/units.hpp"

namespace carvetto::viewmodel {

/// Values of the File ▸ New job form, in the form's own unit (`units`). Flat jobs use width /
/// height / thickness; rotary jobs use length / diameter.
struct NewJobForm {
    document::JobType type{document::JobType::SingleSided};
    document::Units units{document::Units::Millimetres};
    double width{300.0};
    double height{300.0};
    double thickness{18.0};
    double length{300.0};
    double diameter{50.0};
    document::RotaryAxis rotaryAxis{document::RotaryAxis::X};
    document::FlipDirection flip{document::FlipDirection::Horizontal};
    document::ZZero zZero{document::ZZero::MaterialSurface};
    document::Datum datum{document::Datum::BottomLeft};
    /// "Offset datum": place the datum point at (offsetX, offsetY) instead of (0, 0).
    bool useOffset{false};
    double offsetX{0.0};
    double offsetY{0.0};

    bool operator==(const NewJobForm&) const = default;
};

/// Form field a validation message refers to.
enum class NewJobField { Width, Height, Thickness, Length, Diameter, ZZero, OffsetX, OffsetY };

/// One validation failure.
struct NewJobError {
    NewJobField field;
    std::string message;
};

/// Outcome of validating the form: a job, or the list of problems.
struct NewJobResult {
    std::optional<document::Job> job;
    std::vector<NewJobError> errors;

    bool ok() const { return job.has_value(); }
};

/// Validates the form. Sizes must be finite and > 0; flat jobs cannot zero on the cylinder axis
/// and rotary jobs cannot zero on the machine bed; offsets must be finite. A valid flat form
/// yields exactly `Job::singleSided(...)` / `Job::doubleSided(...)` with the same arguments (plus
/// the datum offset in mm when `useOffset`); a rotary form yields `Job::rotary(...)`.
NewJobResult validateNewJob(const NewJobForm& form);

/// The same form expressed in `units` (every length converted; other fields unchanged).
NewJobForm convertFormUnits(const NewJobForm& form, document::Units units);

/// A form pre-filled from an existing job (for re-opening the dialog on the current job).
NewJobForm formFromJob(const document::Job& job);

}  // namespace carvetto::viewmodel
