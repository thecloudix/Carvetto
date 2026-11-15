#pragma once

#include <string>

#include "carvetto/document/document.hpp"
#include "carvetto/viewmodel/relief_modelling.hpp"

// Qt-free logic of the surface-building forms (T-0308, ADR-0087): Two Rail Sweep, Extrude and
// Weave, Turn and Spin, Textured Area. Same shape as the modify forms: `buildFields` lists the
// fields of a page, a `BuildForm` is a label -> text map, `previewBuild` makes the component
// without touching the document, and the result is applied with `ModellingSession::apply`
// (add the first time, update in place afterwards; one undo step each).

namespace carvetto::viewmodel {

enum class BuildTool { TwoRailSweep, ExtrudeWeave, TurnSpin, TexturedArea };
inline constexpr int kBuildToolCount = 4;
const char* buildToolName(BuildTool tool);
const std::vector<ModifyField>& buildFields(BuildTool tool);
/// One line of help shown under the form (what the tool takes from the selection).
const char* buildHelp(BuildTool tool);

struct BuildForm {
    BuildTool tool{BuildTool::TwoRailSweep};
    std::map<std::string, std::string> values;   ///< Missing labels read as the field default.
};
BuildForm defaultBuildForm(BuildTool tool);

/// Parses "a,b; a,b; ..." into points (at least `minPoints`); false with `error` when malformed.
bool parsePointList(const std::string& text, std::vector<relief::Point>& out, std::string& error,
                    std::size_t minPoints = 2);

/// Builds the component of the form from the current selection: Sweep uses the first two selected
/// vectors as rails; Extrude all selected vectors as rails (and, for the Component source, the first
/// selected component); Turn/Spin the Profile text, else the first selected open vector; Textured
/// Area the first selected component as the seed tile and the selected closed vectors as boundary.
ModellingPreview previewBuild(const document::Document& doc, const BuildForm& form);

}  // namespace carvetto::viewmodel
