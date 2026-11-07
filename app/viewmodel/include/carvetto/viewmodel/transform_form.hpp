#pragma once

#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/units.hpp"
#include "carvetto/viewmodel/selection_tools.hpp"
#include "carvetto/viewmodel/tools.hpp"
#include "carvetto/viewmodel/transform_ops.hpp"

// Qt-free numeric forms of Move, Scale, Rotate and Mirror (T-0205, ADR-0014, ADR-0029). The form
// has the same contract as the tool fields of T-0204: `fields()` lists the numeric inputs (label,
// kind, live text), `setField()` validates one entry with the display units and rejects a bad one
// without changing anything, an empty entry clears it, and `apply()` runs the operation of
// `transform_ops.hpp` (one undo step, all-or-nothing).
//
// Fields
//   Move    "x", "y"                 relative offset (default 0), or the absolute position of the anchor
//                                    (default: where it is now).
//   Scale   "width", "height"        a length in display units (> 0), or a factor typed as `1.5 S`.
//                                    With Link XY a single entry drives both axes (the width wins when
//                                    both are typed). Nothing typed is refused.
//   Rotate  "angle"                  degrees, counter-clockwise, negative allowed; relative or absolute.
//           "pivotX", "pivotY"       only with `useCoordinates`; default: the pivot anchor of the bounds.
//   Mirror  (no numeric fields)      options only.

namespace carvetto::viewmodel {

enum class TransformFormKind { Move, Scale, Rotate, Mirror };

struct TransformFormOptions {
    // Move (absolute: position of `anchor`) and Scale (about `anchor`).
    bool absolute{false};
    HandleKind anchor{HandleKind::Centre};
    // Scale
    ScaleMode scaleMode{ScaleMode::Selection};
    bool linkXY{true};
    // Rotate
    bool useCoordinates{false};
    HandleKind pivotAnchor{HandleKind::Centre};
    // Mirror
    MirrorAxis axis{MirrorAxis::Horizontal};
    bool createCopy{false};
    bool useRotatedBounds{false};
    bool aboutMaterialCentre{false};
    bool aboutSelectedLine{false};
};

class TransformForm {
   public:
    TransformForm(TransformFormKind kind, document::Units units);

    TransformFormKind kind() const { return kind_; }
    document::Units units() const { return units_; }
    /// Display units for lengths (follows the document; typed texts are re-read by the caller).
    void setUnits(document::Units units) { units_ = units; }
    TransformFormOptions& options() { return options_; }
    const TransformFormOptions& options() const { return options_; }

    /// Numeric fields for the current options; `text` shows the typed value, else the live value
    /// computed from `doc`'s selection (empty when there is nothing to transform).
    std::vector<NumericField> fields(const document::Document& doc) const;
    /// Validates and stores one entry; an empty `text` clears it. A bad value changes nothing.
    FieldResult setField(const std::string& id, const std::string& text);
    /// Forgets every typed value.
    void clear();
    /// Runs the operation on `doc`'s selection with the typed values.
    TransformResult apply(document::Document& doc) const;

   private:
    struct Entry {
        double value{0.0};   ///< millimetres, degrees, or a factor
        bool factor{false};  ///< Scale: typed as `value S`
        std::string text;
    };
    const Entry* entry(const std::string& id) const;

    TransformFormKind kind_;
    document::Units units_;
    TransformFormOptions options_;
    std::vector<std::pair<std::string, Entry>> entries_;
};

}  // namespace carvetto::viewmodel
