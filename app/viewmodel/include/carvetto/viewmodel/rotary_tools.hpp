#pragma once

#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/geometry/vector_unwrapper.hpp"
#include "carvetto/io/mesh.hpp"
#include "carvetto/viewmodel/model_io.hpp"
#include "carvetto/viewmodel/vector_ops.hpp"

// Qt-free logic of the rotary tools of T-0321 (ADR-0114): the Vector Unwrapper form and the "Importing a 3D
// Model into a Rotary job" form. Both need a rotary job. Each is one undo step and all-or-nothing.

namespace carvetto::viewmodel {

struct UnwrapperSettings {
    geometry::UnwrapCentre centre{geometry::UnwrapCentre::Contour};
    geometry::Vec2 customCentre;
    bool refine{true};
    double tolerance{0.01};
};

/// Unwraps every selected vector of the active sheet (open and closed contours; curves are flattened to the
/// tolerance) round the chosen centre and adds the unrolled contours as new vector objects on the layer of their
/// source, selected. The originals stay. The unrolled contour has the circumference coordinate as X and the distance
/// from the centre as Y; it is not placed on the design area (the user moves it where the modelling tool needs it).
/// Refused for a job that is not rotary, an empty selection or bad settings.
VectorOpResult unwrapSelectedVectors(document::Document& doc, const UnwrapperSettings& settings);

enum class RotaryModelType { Full3D, Flat };

struct RotaryImportSettings {
    /// Units, orientation (top face, quarter turns), size and the zero plane / cell size of the flat import.
    MeshImportSettings mesh;
    RotaryModelType type{RotaryModelType::Full3D};
    /// Full 3D: scale the model uniformly so that its length fits the job length and its radius the job radius.
    bool fitToMaterial{false};
    /// Full 3D: Rotation Axis Movement. The axis normally passes through the middle of the model's cross-section;
    /// with this on it is moved by `axisShift` (mesh mm across the axis: Y,Z for a cylinder along X, X,Z along Y).
    bool moveAxis{false};
    geometry::Vec2 axisShift;
    /// Full 3D: the model's start along the axis, mm from the start of the design area; when `centreAlong` it is centred.
    bool centreAlong{true};
    double alongOffset{0.0};
};

/// Imports `original` into the rotary job as a relief component (see `meshToRotaryField` for Full 3D; Flat is the
/// ordinary top-down import onto the flat design area), on the top level of `sheet`; one undo step.
MeshImportOutcome importMeshIntoRotaryJob(document::Document& doc, document::SheetId sheet,
                                          const io::TriangleMesh& original, const RotaryImportSettings& settings,
                                          const std::string& name);

}  // namespace carvetto::viewmodel
