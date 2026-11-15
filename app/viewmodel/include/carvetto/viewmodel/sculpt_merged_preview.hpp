#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/relief/relief.hpp"
#include "carvetto/viewmodel/model_view.hpp"
#include "carvetto/viewmodel/relief_sculpt.hpp"

// Sculpt preview merged with the rest of the model (T-0316, ADR-0092). The whole visible model of
// the sheet is composed once at `begin`, with the working component split out; each `update` then
// recomputes only the cells the stroke may have touched, so a mouse move costs work proportional to
// the stroke's footprint, not to the model. The surface equals `composeModelSurface` of a document
// whose sculpted component holds the working copy (tested).

namespace carvetto::viewmodel {

class SculptMergedPreview {
   public:
    /// Composes everything but the working component and the first full surface. False with
    /// `problem()` set when the model cannot be composed (the caller then shows the component alone).
    /// The component is shown even when it or its level is hidden in the document.
    bool begin(const document::Document& doc, document::ComponentId target, const relief::HeightField& working,
               std::size_t maxDimension = kMaxSurfaceDimension);
    void end();
    bool active() const { return active_; }
    const std::string& problem() const { return problem_; }

    /// Brings the surface up to date with `working` after the changes described by `dirty`.
    void update(const relief::HeightField& working, const SculptDirty& dirty);
    const ModelSurface& surface() const { return surface_; }

    /// Cell evaluations done so far: one per surface cell per layer consulted (the working
    /// component, each component above it in its level, the level, each level above). The tests use
    /// it to bound the cost of a move without timing.
    std::uint64_t cellEvaluations() const { return evaluations_; }

   private:
    struct LevelRef {
        relief::CombineMode mode{relief::CombineMode::Add};
        relief::HeightField field;
    };
    void recompute(std::size_t i0, std::size_t i1, std::size_t j0, std::size_t j1);

    bool active_{false};
    std::string problem_;
    ModelSurface surface_;
    std::uint64_t evaluations_{0};

    relief::Grid grid_;
    std::size_t stride_{1};
    relief::Component target_;                 ///< Its field is the working copy.
    relief::Level level_;                      ///< The target's level (target and hidden items as given).
    relief::HeightField lowerModel_;           ///< Levels below the target's level, composed.
    relief::HeightField below_;                ///< Components below the target in its level.
    std::vector<relief::CombineMode> aboveModes_;
    std::vector<relief::HeightField> above_;   ///< Components above the target, composed.
    std::vector<LevelRef> upper_;              ///< Levels above, each composed with effects.
    relief::HeightField levelPre_;             ///< The target's level before effects.
    relief::HeightField levelFin_;             ///< After effects.
};

}  // namespace carvetto::viewmodel
