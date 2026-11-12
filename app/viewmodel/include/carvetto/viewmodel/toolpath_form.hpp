#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/toolpath_record.hpp"
#include "carvetto/document/units.hpp"
#include "carvetto/tooling/tool_database.hpp"
#include "carvetto/viewmodel/toolpath_calc.hpp"
#include "carvetto/viewmodel/tools.hpp"

// Qt-free form contracts of the five toolpath operations (T-0220, ADR-0031, ADR-0014): Profile
// (T-0120), Pocket (T-0121), Drill (T-0150), Quick Engrave (T-0151) and V-Carve (T-0153). A form
// has the same contract as the tool fields of T-0204 and the transform forms of T-0205:
//   * `fields()` lists the numeric inputs that apply to the current choices (label, kind, group,
//     live text in display units), `choices()` the option inputs (drop-downs and check boxes);
//   * `setField()` / `setChoice()` validate one entry (display units for lengths, degrees for
//     angles) and reject a bad one with a message and NO side effect; an empty text resets the
//     field to its default;
//   * values map 1:1 onto the core params structs (`ToolpathDefinition::params`: mm, radians);
//     feeds, speeds, pass depth and stepover come from the tool database (the form shows the
//     tool's values as the live text of a field left at 0 = "use the tool's").
//
// Field ids (the same in every operation where they apply)
//   name        text (`setName`)
//   Profile     startDepth cutDepth passDepth lastPassThickness safeZ allowance lastPassAllowance
//               tabLength tabThickness tabCount tabDistance rampDistance rampAngle leadLength
//               leadAngle leadRadius leadSweep overcut startX startY mitreLimit chordal
//   Pocket      startDepth cutDepth lastPassThickness safeZ allowance rasterAngle rampDistance chordal
//   Drill       startDepth cutDepth safeZ retractGap dwellTime startX startY distance chordal
//   Engrave     depth passes safeZ stepover rasterAngle startX startY chordal
//   V-Carve     startDepth flatDepth safeZ clearanceRasterAngle rampDistance startX startY chordal
//   Chamfer     angle width cutDepth overcut safeZ startX startY chordal (angle, width and cutDepth
//               are linked: cutDepth = width / (2 tan angle); a V-bit's angle comes from the tool)
//   Thread Mill startDepth maxDepth threadLength safeZ threadedLength diameter pitch fitTolerance startX startY
//               distance chordal (pitch is hidden while a pitch preset is chosen)
//   Inlay       startDepth cutDepth stepDepth stepWidth allowance safeZ startX startY chordal (step fields only
//               for a stepped male; choices piece, maleStyle, femaleStyle, direction, order)
//   Fluting     fluteDepth rampLength|rampPercent (by the ramp basis; hidden for a ramp over the complete
//               length) safeZ startX startY chordal
//   Prism       depth (hidden with fullPrism) stepover safeZ startX startY chordal
// Choices carry the enum/bool inputs; see `choices()`.

namespace carvetto::viewmodel {

enum class ToolpathFieldKind { Length, Angle, Count, Seconds, Factor };

/// One numeric input of a toolpath form.
struct ToolpathField {
    std::string id;
    std::string label;
    /// UI section ("Depth", "Tabs", ...).
    std::string group;
    ToolpathFieldKind kind{ToolpathFieldKind::Length};
    bool allowNegative{false};
    bool allowZero{false};
    /// Current value in display units (lengths), degrees (angles), seconds or a plain number.
    /// For a `fromTool` field left at 0 it shows the tool database's value instead.
    std::string text;
    /// True when the value differs from the field's default (it was typed).
    bool typed{false};
    /// True when 0 means "use the tool database's value" and `text` shows that value.
    bool fromTool{false};
};

/// One option input: a drop-down, or a check box when it has two options "No"/"Yes".
struct ToolpathChoice {
    std::string id;
    std::string label;
    std::string group;
    std::vector<std::string> options;
    int current{0};
    bool isBool{false};
};

/// The tool a form is set to, as the form knows it (for display and the defaults shown).
struct ToolSummary {
    bool valid{false};
    std::string id;
    std::string name;
    tooling::ToolType type{tooling::ToolType::EndMill};
    double diameterMm{0.0};
    /// Included angle of the tool, radians (V-bits and the like; 0 otherwise).
    double includedAngleRad{0.0};
    /// Deepest cut a chamfer can make with the tool, mm (read-only "Max Cut Depth"; 0 when the tool
    /// is not a chamfer tool).
    double maxCutDepthMm{0.0};
    /// The cutting data the toolpath will use (the chosen machine/material entry, else the default).
    tooling::CuttingData data;
    /// No entry for the chosen machine and material: the default data stands in.
    bool fallback{false};
};

/// The tools of `db` an operation accepts (`toolAllowedFor`), in id order; `clearance` = the V-carve's
/// clearance tools. Pointers are valid while `db` is unchanged.
std::vector<const tooling::Tool*> toolsFor(const tooling::ToolDatabase& db, document::ToolpathOperation operation,
                                           bool clearance = false);

/// The machine/material pairs `tool` has cutting data for, as tool sources (in key order), after the
/// source with neither (the tool's default data).
std::vector<document::ToolSource> cuttingDataChoices(const tooling::Tool& tool);

class ToolpathForm {
   public:
    ToolpathForm(document::ToolpathOperation operation, document::Units units);
    /// A form holding `definition` (the tool is shown once `refreshTools` has seen the database).
    ToolpathForm(const ToolpathDefinition& definition, document::Units units);

    document::ToolpathOperation operation() const { return definition_.operation(); }
    document::Units units() const { return units_; }
    void setUnits(document::Units units) { units_ = units; }

    const std::string& name() const { return definition_.name(); }
    /// Rejects an empty name, an over-long name and control characters, changing nothing.
    FieldResult setName(const std::string& name);

    std::vector<ToolpathField> fields() const;
    std::vector<ToolpathChoice> choices() const;
    /// Validates and stores one entry; an empty text resets the field. A bad value changes nothing.
    FieldResult setField(const std::string& id, const std::string& text);
    /// Calculation edit boxes (T-0257): a numeric entry starting with `=` is evaluated by
    /// `calculate` (units of the form) and its text is kept, field id -> text; the field's value in
    /// the definition is the result. `fields()` shows the kept text for such a field.
    const std::map<std::string, std::string>& expressions() const { return expressions_; }
    /// The expression of field `id`, or empty.
    std::string expression(const std::string& id) const;
    /// Document variables the kept expressions name (sorted, unique).
    std::vector<std::string> usedVariables() const;
    /// Re-enters every expression (e.g. when loading a form for editing); the first failure is
    /// returned, the rest are still applied.
    FieldResult setExpressions(const std::map<std::string, std::string>& expressions);
    /// Sets one option (index into `ToolpathChoice::options`; a bool is 0/1).
    FieldResult setChoice(const std::string& id, int option);

    // --- Tools -------------------------------------------------------------------------------
    /// Picks the tool from `db`: it must exist, suit the operation (`toolAllowedFor`) and its
    /// machine/material must exist (both or neither). A bad pick changes nothing.
    FieldResult setTool(const tooling::ToolDatabase& db, const document::ToolSource& source);
    /// V-carve: the clearance end mills, in order (each checked like `setTool`; all or nothing).
    FieldResult setClearanceTools(const tooling::ToolDatabase& db, const std::vector<document::ToolSource>& sources);
    const document::ToolSource& tool() const { return definition_.tool; }
    const std::vector<document::ToolSource>& clearanceTools() const { return definition_.clearanceTools; }
    /// The chosen tool and the cutting data it brings (feed, plunge, spindle, pass depth, stepover).
    const ToolSummary& toolSummary() const { return toolSummary_; }
    /// Re-reads the tool from `db` (after the database changed or the form was loaded).
    void refreshTools(const tooling::ToolDatabase& db);

    // --- Vectors -----------------------------------------------------------------------------
    const VectorSelector& selector() const { return definition_.selector; }
    void setSelector(VectorSelector selector) { definition_.selector = std::move(selector); }

    // --- Result ------------------------------------------------------------------------------
    /// The definition the form holds (always the core's params; may still be incomplete, see
    /// `problems`).
    const ToolpathDefinition& definition() const { return definition_; }
    /// What must be fixed before a toolpath can be calculated (no tool, no depth, ...); the core
    /// operation reports further problems with the vectors at calculation time.
    std::vector<std::string> problems() const;
    /// Back to the operation's defaults (tool, clearance tools and selector are kept).
    void resetValues();

    // --- Table access (also used by the session's tests) ---------------------------------------
    /// Ids of all numeric fields and choices of the operation, visible or not.
    std::vector<std::string> allFieldIds() const;
    std::vector<std::string> allChoiceIds() const;

   private:
    FieldResult checkTool(const tooling::ToolDatabase& db, const document::ToolSource& source, bool clearance,
                          ToolSummary* summary) const;

    ToolpathDefinition definition_;
    document::Units units_;
    ToolSummary toolSummary_;
    std::map<std::string, std::string> expressions_;
};

}  // namespace carvetto::viewmodel
