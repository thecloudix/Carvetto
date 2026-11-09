#pragma once

#include <string>
#include <vector>

#include "carvetto/document/document.hpp"

// Qt-free model of the Properties panel (T-0210, ADR-0014). The panel is stateless glue: it asks
// `describeSelection()` for the fields of the current selection and passes every edit to
// `setProperty()`. Nothing keeps a `Document` reference between calls, so replacing the document
// (File > New / Open) cannot leave a dangling pointer behind.
//
// Every edit is ONE undo step, validated completely before the document is touched (a refusal
// changes nothing, not even the undo history), and goes through an existing command
// (`MoveObjectsToLayerCommand`, `TransformObjectsCommand` via `transform_ops`, `SetTextCommand`) or
// one of the two of `object_property_commands.hpp`. Objects on a locked layer are never edited.
//
// Fields (ids) by selection
//   any non-empty       "layer"   Choice: the layer names; mixed selection shows no choice.
//   one object          "name"
//   editable targets    "x", "y"  lower-left corner of the bounds of the selection (display units)
//   group, bitmap,      "width", "height"   size of the bounds (scales the selection from its
//   plain vector, text                      lower-left corner)
//   any but a circle    "rotation" degrees; for a single ellipse, polygon or star it is the angle
//                       parameter (the shape stays parametric), otherwise an absolute rotation of
//                       the selection about its centre (a plain vector unless a 90 degree multiple)
//   rectangle           "width", "height" (parameters), "corners" (Choice), "cornerRadius"
//   circle              "diameter"
//   ellipse             "width", "height", "rotation"
//   polygon             "sides", "radius", "rotation"
//   star                "points", "outerRadius", "innerRadius", "rotation"
//   text                "text", "size", "align" (Choice), "letterSpacing", "lineSpacing" (percent),
//                       "kerning" (Choice off/on)

namespace carvetto::viewmodel {

enum class PropertyKind { Text, Length, Angle, Count, Percent, Choice };

struct PropertyField {
    std::string id;
    std::string label;
    PropertyKind kind{PropertyKind::Text};
    /// Current value as the user would type it (lengths in display units without suffix, angles in
    /// degrees, percent without sign); empty for a mixed or absent value.
    std::string text;
    bool multiline{false};     ///< Text: may hold line breaks.
    std::vector<std::string> choices;  ///< Choice
    int choice{-1};                    ///< Choice: index of the current one, -1 when mixed
    bool allowNegative{false};
    bool allowZero{false};
    bool editable{true};
    /// Unit suffix to show next to the value ("mm", "in", "deg", "%"), empty for none.
    std::string suffix;
};

struct PropertyView {
    /// "No selection", "Rectangle", "Text", "3 objects", ...
    std::string title;
    std::vector<PropertyField> fields;
};

/// Fields of the current selection (empty `fields` for no selection).
PropertyView describeSelection(const document::Document& doc);

struct EditResult {
    bool ok{true};
    /// The reason when refused; empty on success.
    std::string message;
    /// The document changed (one undo step was added).
    bool changed{false};
};

/// Applies one edit. `text` is what the user typed (a choice: its index as decimal text). A field
/// that does not exist for the selection, a bad number, or a refusal of the command gives
/// `ok == false` with a message and leaves the document as it was.
EditResult setProperty(document::Document& doc, const std::string& fieldId, const std::string& text);

}  // namespace carvetto::viewmodel
