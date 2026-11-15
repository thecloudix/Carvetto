#pragma once

#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/relief/relief.hpp"
#include "carvetto/viewmodel/properties_model.hpp"

// Qt-free logic of the level effect actions (T-0302, ADR-0084): Clipping (Apply / Update / Remove)
// from the selected closed vectors and Mirror Mode, each one undo step through the commands of
// T-0295. Curves are flattened here: the document stores clip polygons in design space.

namespace carvetto::viewmodel {

const std::vector<relief::MirrorMode>& allMirrorModes();
const char* mirrorModeName(relief::MirrorMode mode);

/// The level the effect actions act on: `picked` when it exists, else the level of the single
/// selected component, else the top level of the active sheet. Null when the sheet has none.
document::LevelId effectsTargetLevel(const document::Document& doc, document::LevelId picked = {});

struct LevelEffectsSummary {
    std::string levelName;
    std::size_t clipPolygons{0};
    std::size_t clipVertices{0};
    relief::MirrorMode mirror{relief::MirrorMode::None};
    relief::Point mirrorCenter;
    /// Closed vectors in the selection that Apply / Update Clipping would use.
    std::size_t selectedClosedVectors{0};
    std::string text() const;  ///< "Clipped by 2 polygons; mirror: Left to right".
};
std::optional<LevelEffectsSummary> summarizeEffects(const document::Document& doc, document::LevelId level);

/// The centre of the model of the level's sheet (the union grid of its visible components): where
/// the mirror axes pass. Nullopt when the sheet shows nothing.
std::optional<relief::Point> modelCentre(const document::Document& doc, document::SheetId sheet);

/// The selected closed vectors flattened to polygons (even-odd: a polygon inside another is a hole).
relief::Clip clipFromSelection(const document::Document& doc);

/// Apply / Update Clipping: replaces the level's clip by the selected closed vectors.
EditResult applyLevelClip(document::Document& doc, document::LevelId level);
EditResult removeLevelClip(document::Document& doc, document::LevelId level);
/// Sets the mirror mode; the axes pass through the model centre (the existing centre is kept when
/// the sheet shows nothing).
EditResult setLevelMirror(document::Document& doc, document::LevelId level, relief::MirrorMode mode);
/// Makes the level's effects permanent (BakeReliefLevelEffectsCommand).
EditResult bakeLevelEffects(document::Document& doc, document::LevelId level);

}  // namespace carvetto::viewmodel
