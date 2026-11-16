#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/document/units.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/wrapped_layout.hpp"
#include "carvetto/viewmodel/tools.hpp"

// Qt-free model of the Wrapped Fluting Layout and Wrapped Spiral Layout dialogs (T-0322, ADR-0115; generators
// `geometry::wrappedFlutingLayout` / `wrappedSpiralLayout`). Same contract as the other numeric forms: `fields()`
// and `choices()` list the inputs, `setField()` / `setChoice()` validate one entry (display units for lengths)
// and refuse a bad one without changing anything, `preview()` computes the vectors for the job without touching
// the document, and `apply()` adds them as ONE vector object in ONE undo step, all-or-nothing, and selects it.
// The cylinder (diameter, axis, Flip Design) is the document's rotary job; any other job is refused.
//
// Fluting fields (defaults): "flutes" count [4]; "alongStart" [0] and "alongEnd" [job length]; "offset" of the
// first flute round the circumference [0]; "width" (outline shapes only) [4 mm]; "depth" [2 mm]. Choice "shape":
// centre lines (for the Fluting toolpath) | rectangles | rounded (cove) ends.
// Spiral fields: "strands" count [1]; "alongStart", "alongEnd", "offset"; then by choice "mode" one of "pitch"
// (along the axis per turn) [25 mm], "spacing" (between strands) or "twist" (total twist in degrees over the
// length: 360 = one turn). Choice "direction": right-hand | left-hand.
// `reset()` keeps the extent along the axis of the last `startFrom` (the job's length).

namespace carvetto::viewmodel {

enum class RotaryLayoutKind { Fluting, Spiral };

struct ChoiceField {
    std::string id;
    std::string label;
    std::vector<std::string> options;
    int selected{0};
};

struct RotaryLayoutPreview {
    bool ok{false};
    /// Why there is nothing to show; empty when ok.
    std::string message;
    std::vector<geometry::Contour> contours;
    /// Flutes whose outline leaves the flat design area at the edge of the turn, and the like.
    std::vector<std::string> warnings;
    /// Spiral: the pitch used (mm) and the turns made; Fluting: the spacing between flutes (pitch) round the circumference.
    double pitch{0.0};
    double turns{0.0};
};

struct RotaryLayoutResult {
    bool ok{false};
    std::string message;
    std::optional<document::ObjectId> created;
    std::size_t vectors{0};
    std::vector<std::string> warnings;
};

class RotaryLayoutForm {
   public:
    RotaryLayoutForm(RotaryLayoutKind kind, document::Units units);

    RotaryLayoutKind kind() const { return kind_; }
    document::Units units() const { return units_; }
    void setUnits(document::Units units) { units_ = units; }

    /// Takes the extent along the axis from the job (0 .. length); a non-rotary job changes nothing.
    void startFrom(const document::Document& doc);

    std::vector<NumericField> fields() const;
    std::vector<ChoiceField> choices() const;
    FieldResult setField(const std::string& id, const std::string& text);
    FieldResult setChoice(const std::string& id, int option);
    void reset();

    void setTargetLayer(std::optional<document::LayerId> layer) { layer_ = layer; }
    std::optional<document::LayerId> targetLayer() const { return layer_; }
    document::LayerId effectiveLayer(const document::Document& doc) const;

    /// The generator parameters for `doc`'s job (valid only for a rotary job).
    geometry::WrappedFlutingParams flutingParams(const document::Document& doc) const;
    geometry::WrappedSpiralParams spiralParams(const document::Document& doc) const;

    RotaryLayoutPreview preview(const document::Document& doc) const;
    RotaryLayoutResult apply(document::Document& doc) const;

   private:
    RotaryLayoutKind kind_;
    document::Units units_;
    int count_{4};  // flutes or strands
    double alongStart_{0.0};
    double alongEnd_{100.0};
    double offset_{0.0};
    double width_{4.0};
    double depth_{2.0};
    int shape_{0};
    // Spiral.
    int mode_{0};
    double pitch_{25.0};
    double spacing_{25.0};
    double twistDeg_{360.0};
    int direction_{0};
    std::optional<document::LayerId> layer_;
};

}  // namespace carvetto::viewmodel
