#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/document/units.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/vector_texture.hpp"
#include "carvetto/viewmodel/tools.hpp"

// Qt-free model of the Create Vector Texture dialog (T-0254, ADR-0014; generator T-0251). Same
// contract as the other numeric forms: `fields()` lists the inputs (label, kind, live text),
// `setField()` validates one entry with the display units and refuses a bad one without changing
// anything, `preview()` computes the lines for the current selection without touching the
// document, and `apply()` adds them as ONE undo step, all-or-nothing.
//
// Fields (defaults in brackets)
//   "angle"       degrees, -90..90, line direction            [0]
//   "spacing"     length > 0, the maximum line spacing        [5 mm]
//   "variation"   percent 0..100, how much spacing may shrink [0]
//   "wavelength"  length > 0                                  [10 mm]
//   "amplitude"   length >= 0 (0 gives straight lines)        [1 mm]
//   "noise"       percent 0..100, randomises phase/amplitude  [0]
//   "seed"        whole number 0..2^64-1                      [1]
// Lengths are typed in the document's display units unless they carry a `mm` / `in` suffix.
//
// The lines are clipped to the selected closed vectors (`selectedClosedRegion`: holes excluded,
// objects united) and added as ONE plain vector object on the target layer (default: the active
// layer) of the active sheet; the new object becomes the selection. Work is bounded: the
// generator's vertex limit, the region span limit, and a lighter limit for the live preview
// (`kTexturePreviewMaxVertices`: beyond it the preview is skipped, OK still creates the texture).

namespace carvetto::viewmodel {

/// Estimated sample vertices above which `preview()` is skipped (still creatable).
inline constexpr double kTexturePreviewMaxVertices = 200'000.0;

struct TexturePreview {
    bool ok{false};
    /// Why there is no preview (invalid entry, no region, too many vertices ...); empty when ok.
    std::string message;
    /// True when the preview was skipped only for its size (the texture can still be created).
    bool skipped{false};
    std::vector<geometry::Contour> contours;
    std::size_t vertexCount{0};
    /// Bounds of the clip region (mm), for drawing; empty when there is none.
    geometry::Box2 region;
};

struct TextureResult {
    bool ok{false};
    std::string message;
    /// The new object, now selected.
    std::optional<document::ObjectId> created;
};

class TextureForm {
   public:
    explicit TextureForm(document::Units units);

    document::Units units() const { return units_; }
    /// Display units for lengths; typed entries are kept as values (mm) and re-formatted.
    void setUnits(document::Units units) { units_ = units; }

    std::vector<NumericField> fields() const;
    /// Validates and stores one entry; a bad value changes nothing. Empty text is refused.
    FieldResult setField(const std::string& id, const std::string& text);
    /// Back to the defaults (the target layer too).
    void reset();

    /// Generator parameters for the current entries (radians, mm).
    geometry::VectorTextureParams params() const { return params_; }
    /// Every entry is range-checked by `setField`; the density limit needs a region, so it is
    /// reported by `preview()` / `apply()`.

    /// The layer the texture goes on; unset means the document's active layer.
    void setTargetLayer(std::optional<document::LayerId> layer) { layer_ = layer; }
    std::optional<document::LayerId> targetLayer() const { return layer_; }
    /// The layer apply() will use for `doc` (the target if it still exists, else the active layer).
    document::LayerId effectiveLayer(const document::Document& doc) const;

    TexturePreview preview(const document::Document& doc) const;
    TextureResult apply(document::Document& doc) const;

   private:
    document::Units units_;
    geometry::VectorTextureParams params_;
    std::optional<document::LayerId> layer_;
};

}  // namespace carvetto::viewmodel
