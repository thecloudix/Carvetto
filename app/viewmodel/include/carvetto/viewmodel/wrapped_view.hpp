#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/job.hpp"
#include "carvetto/document/toolpath_record.hpp"
#include "carvetto/geometry/cylinder_map.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/simulate/stock.hpp"
#include "carvetto/viewmodel/model_view.hpp"

// The Auto-Wrapping View of a rotary job (T-0320, ADR-0103): vectors, toolpaths, the relief model and
// the simulated stock are computed flat, and drawn wrapped on the job's cylinder with a software
// renderer (Qt-free, deterministic). A flat/wrapped toggle selects between this and the flat views.

namespace carvetto::viewmodel {

enum class RotaryViewMode { Flat, Wrapped };

/// The view toggle. Only rotary jobs have a wrapped view; for any other job `effective` is Flat.
struct RotaryViewState {
    RotaryViewMode mode{RotaryViewMode::Wrapped};

    static bool available(const document::Job& job) { return job.type == document::JobType::Rotary; }
    void toggle() { mode = mode == RotaryViewMode::Wrapped ? RotaryViewMode::Flat : RotaryViewMode::Wrapped; }
    RotaryViewMode effective(const document::Job& job) const {
        return available(job) ? mode : RotaryViewMode::Flat;
    }
};

/// The flat-to-cylinder map of a rotary job: along its axis, radius, Flip Design, axis Z
/// (`rotarySurfaceZ() - radius`). Not meaningful for other jobs (radius 0 is replaced by 1).
geometry::CylinderMap cylinderMapFor(const document::Job& job);

/// A grid of machine heights (Z, mm) over the flat design area to draw wrapped; NaN = no surface.
struct WrappedSurface {
    std::size_t nx{0};
    std::size_t ny{0};
    double cellSize{1.0};
    geometry::Vec2 origin{};
    std::vector<float> z;
    /// 0xFFRRGGBB per cell (row-major, same layout as z); 0 = the view's material. Empty = all material.
    std::vector<std::uint32_t> colours;
    bool ok() const { return nx >= 2 && ny >= 2 && z.size() == nx * ny; }
};

/// The relief model as machine heights: model height h sits at `job.rotaryModelBottomZ() + h` (the model band
/// starts at the inside gap), so the surface wraps at radius = gap inside + h.
WrappedSurface wrappedSurfaceFromModel(const ModelSurface& model, const document::Job& job);
/// The simulated stock (machine heights as they are). Cut cells are tinted; passes are not coloured.
WrappedSurface wrappedSurfaceFromSimulation(const simulate::Heightfield& field);

struct WrappedRenderOptions {
    std::size_t width{640};
    std::size_t height{480};
    ModelMaterial material{ModelMaterial::Wood};
    std::uint32_t background{0xFFE2E4E8u};
    /// Paths in flat machine coordinates (z = machine Z), drawn as one-pixel lines on the cylinder.
    std::vector<ModelPath> paths;
    /// Largest angle between drawn points of a path, radians.
    double maxStepRad{0.05};
};

/// Draws `surface` wrapped by `map`, then the paths, from `camera` (orthographic orbit; the cylinder axis lies
/// along X for a job along X). Deterministic; cost is O(cells + pixels covered + path points).
Raster renderWrapped(const WrappedSurface& surface, const geometry::CylinderMap& map, const OrbitCamera& camera,
                     const WrappedRenderOptions& options = {});

/// A plain cylinder surface (machine Z = the job's surface) over the job's design area, drawn when a rotary job has
/// no model yet so that vectors and toolpaths still show on the cylinder. Empty (not ok) for a job without size.
WrappedSurface wrappedBlankCylinder(const document::Job& job);

/// The visible vector contours on the active sheet as flat paths at the cylinder surface (z = `job` surface Z).
std::vector<ModelPath> wrappedVectorPaths(const document::Document& doc, std::uint32_t colour = 0xFF1A5FB4u);
/// The cutting moves of every record with moves, in machine coordinates (arcs split into chords of at most
/// `arcStepRad`), one path per record with colours cycling; rapids break a run.
std::vector<ModelPath> wrappedToolpathPaths(const std::vector<document::ToolpathRecord>& records,
                                            double arcStepRad = 0.1, std::size_t maxPoints = 200000);

/// The flat/wrapped picture of the model for a rotary job: the wrapped view when `state` says so and the job is
/// rotary (a plain cylinder when `model` is empty), else the existing flat `renderModel` (`options` are used for size, material and toolpaths there).
Raster renderRotaryModel(const ModelSurface& model, const document::Job& job, const RotaryViewState& state,
                         const OrbitCamera& camera, const ModelRenderOptions& options,
                         const std::vector<ModelPath>& wrappedPaths = {});

}  // namespace carvetto::viewmodel
