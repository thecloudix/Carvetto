#pragma once

#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/job.hpp"
#include "carvetto/document/units.hpp"
#include "carvetto/viewmodel/properties_model.hpp"

// Qt-free Material Setup for rotary jobs (T-0319, ADR-0098): the cylinder diameter, the gaps
// between the model band and the cylinder surface / axis, and the Z-zero (cylinder surface or
// axis). Values are in the form's own unit; the document stores millimetres.

namespace carvetto::viewmodel {

struct RotaryMaterialForm {
    document::Units units{document::Units::Millimetres};
    double diameter{50.0};
    /// Radial gap between the cylinder surface and the top of the model ("Gap Outside Model").
    double gapOutside{0.0};
    /// Radial gap between the bottom of the model and the axis ("Gap Inside Model").
    double gapInside{0.0};
    document::ZZero zZero{document::ZZero::CylinderAxis};

    bool operator==(const RotaryMaterialForm&) const = default;
};

enum class RotaryMaterialField { Diameter, GapOutside, GapInside, ZZero };

struct RotaryMaterialError {
    RotaryMaterialField field;
    std::string message;
};

/// Form pre-filled from a rotary job (an empty optional for a flat job).
std::optional<RotaryMaterialForm> rotaryMaterialFromJob(const document::Job& job);

/// Problems with `form` for a job of `type`; empty when it can be applied. The diameter must be
/// finite and positive (at most `kMaxSheetSizeMm`), the gaps finite and >= 0 with
/// gapOutside + gapInside < radius, and the Z-zero the cylinder surface or axis.
std::vector<RotaryMaterialError> validateRotaryMaterial(const RotaryMaterialForm& form);

/// Radial thickness of the model band (read-only "Model Thickness") in the form's unit.
double rotaryModelThickness(const RotaryMaterialForm& form);

/// Applies `form` to a rotary job (one undo step): job diameter, gaps and Z-zero, and every
/// sheet's circumference, diameter and Z-zero (each sheet keeps its own length). Refused (document
/// untouched) for a flat job or an invalid form.
EditResult applyRotaryMaterial(document::Document& doc, const RotaryMaterialForm& form);

/// Resizes every sheet of the document to the rotary `job`: the job's diameter and circumference
/// and Z-zero, and (`takeLength`) its length, otherwise each sheet's own length. Runs inside the
/// caller's undo macro; false when a command failed.
bool resizeRotarySheets(document::Document& doc, const document::Job& job, bool takeLength);

}  // namespace carvetto::viewmodel
