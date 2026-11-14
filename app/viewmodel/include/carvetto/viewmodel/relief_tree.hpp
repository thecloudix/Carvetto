#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/relief/relief.hpp"
#include "carvetto/viewmodel/properties_model.hpp"

// Qt-free logic of the Component Tree panel and the Component Properties form (T-0299, ADR-0077).
// Stateless functions over a `Document&` passed per call. Every edit is ONE undo step through the
// commands of `document/relief_commands.hpp` (ADR-0061); a refusal returns a message and leaves
// document and history alone.

namespace carvetto::viewmodel {

/// One row of the tree. Levels are listed top of the stack first (the way a stack is drawn), each
/// followed by its components, also top first.
struct ReliefTreeRow {
    bool isLevel{false};
    document::LevelId level;          ///< The level (of the component for a component row).
    document::ComponentId component;  ///< Null for a level row.
    std::string name;
    relief::CombineMode mode{relief::CombineMode::Add};
    bool visible{true};
    bool selected{false};
    /// Index among the level's components (components) or among the sheet's levels (levels),
    /// bottom first, as the reorder commands take it.
    std::size_t index{0};
};

/// The tree of `sheet`.
std::vector<ReliefTreeRow> buildReliefTree(const document::Document& doc, document::SheetId sheet);

const char* combineModeName(relief::CombineMode mode);
/// Every mode in menu order.
const std::vector<relief::CombineMode>& allCombineModes();

// --- Edits ------------------------------------------------------------------------------------

/// Adds a level on top of the sheet's levels; an empty name picks "Level N". The new id is stored
/// in `created` when given.
EditResult addReliefLevel(document::Document& doc, document::SheetId sheet, const std::string& name = {},
                          document::LevelId* created = nullptr);
EditResult renameReliefComponent(document::Document& doc, document::ComponentId id, const std::string& name);
EditResult renameReliefLevel(document::Document& doc, document::LevelId id, const std::string& name);
EditResult setReliefComponentMode(document::Document& doc, document::ComponentId id, relief::CombineMode mode);
EditResult setReliefLevelMode(document::Document& doc, document::LevelId id, relief::CombineMode mode);
EditResult setReliefComponentVisible(document::Document& doc, document::ComponentId id, bool visible);
EditResult setReliefLevelVisible(document::Document& doc, document::LevelId id, bool visible);
/// Show (This / Only This / All But This / All) and Hide (This / All) of the component menu are
/// compositions of these two.
EditResult setAllReliefComponentsVisible(document::Document& doc, document::SheetId sheet, bool visible);
EditResult showOnlyReliefComponent(document::Document& doc, document::ComponentId id);
EditResult deleteReliefComponents(document::Document& doc, const std::vector<document::ComponentId>& ids);
EditResult deleteReliefLevel(document::Document& doc, document::LevelId id);
EditResult duplicateReliefComponent(document::Document& doc, document::ComponentId id,
                                    document::ComponentId* created = nullptr);
/// Moves a component one step up (towards the top of the stack) or down within its level.
EditResult moveReliefComponent(document::Document& doc, document::ComponentId id, bool up);
EditResult moveReliefLevel(document::Document& doc, document::LevelId id, bool up);
/// Drag and drop: puts the component into `level` at `index` (bottom first). Within its own level
/// this reorders; into another level it moves there and then places it.
EditResult dropReliefComponent(document::Document& doc, document::ComponentId id, document::LevelId level,
                               std::size_t index);
EditResult moveReliefComponentToLevel(document::Document& doc, document::ComponentId id, document::LevelId level);

// --- Selection (not undoable) -------------------------------------------------------------------

/// Selects exactly `ids` (unknown ids are dropped).
void selectReliefComponents(document::Document& doc, const std::vector<document::ComponentId>& ids);

// --- Component Properties form ------------------------------------------------------------------

/// Text of the Component Properties form. Lengths in millimetres, angles in degrees, as typed.
struct ReliefPropertiesForm {
    std::string name;
    relief::CombineMode mode{relief::CombineMode::Add};
    std::string shapeHeightPercent;
    std::string baseHeight;
    bool visible{true};
    bool fadeEnabled{false};
    std::string fadePercent;
    std::string fadeAnchorX, fadeAnchorY, fadeSecondX, fadeSecondY;
    bool tiltEnabled{false};
    std::string tiltAngleDeg;
    std::string tiltPivotX, tiltPivotY, tiltRaiseX, tiltRaiseY;
    /// Cell grid of the field, for the read-only size line ("120 x 80 cells, 0.5 mm").
    std::string fieldInfo;
};

/// The form for component `id`; false when it does not exist.
bool describeReliefComponent(const document::Document& doc, document::ComponentId id, ReliefPropertiesForm& out);

/// Applies the form to the component as one undo step (name, mode and properties together).
/// Bad numbers, a degenerate fade/tilt direction or a shape height outside 0..1000 % are refused
/// with the reason and nothing changes.
EditResult applyReliefProperties(document::Document& doc, document::ComponentId id, const ReliefPropertiesForm& form);

/// The level's Base Height, Combine Mode and name (levels expose only these).
EditResult applyReliefLevelProperties(document::Document& doc, document::LevelId id, const std::string& name,
                                      relief::CombineMode mode, const std::string& baseHeight);

}  // namespace carvetto::viewmodel
