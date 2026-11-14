#pragma once

#include <string>
#include <vector>

#include "carvetto/document/bitmap_store.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/relief/bitmap_relief.hpp"
#include "carvetto/relief/relief.hpp"
#include "carvetto/relief/shape.hpp"
#include "carvetto/viewmodel/properties_model.hpp"

// Qt-free logic of the modelling forms (T-0300, ADR-0080): Create Shape, Bitmap to relief, Zero
// Plane and Scale Model Z. Every form is plain text as typed (millimetres, degrees); `preview*`
// builds the result without touching the document, and `ModellingSession::apply` commits it as
// ONE undo step. Nothing here changes the document except `ModellingSession`.

namespace carvetto::viewmodel {

/// A previewed result: either a component to add/replace or the reason there is none.
struct ModellingPreview {
    bool ok{false};
    std::string message;           ///< Why `ok` is false.
    relief::Component component;   ///< Create Shape, Bitmap, Zero Plane.
    std::string info;              ///< One line for the form ("120 x 80 cells, top 12.5 mm").
};

// --- Create Shape ---------------------------------------------------------------------------------

struct CreateShapeForm {
    std::string name{"Shape"};
    relief::ShapeKind kind{relief::ShapeKind::Angled};
    std::string angleDeg{"45"};
    bool limitEnabled{false};
    std::string limit{"1"};
    bool preserveInternalCorners{false};
    bool blendToInnerVectors{false};
    std::string smoothing{"0"};          ///< -1 concave .. +1 convex.
    std::string scalePercent{"100"};
    std::string baseHeight{"0"};
    relief::CombineMode mode{relief::CombineMode::Add};
    std::string cellSize{"1"};
};

/// Number of closed vector contours of the selected vectors that a shape would use.
std::size_t selectedShapeVectorCount(const document::Document& doc);
ModellingPreview previewCreateShape(const document::Document& doc, const CreateShapeForm& form);

// --- Bitmap to relief -------------------------------------------------------------------------------

struct BitmapReliefForm {
    std::string name{"Bitmap"};
    /// Image file; empty uses the selected bitmap object of the sheet.
    std::string filePath;
    std::string widthMm{"100"};
    std::string zHeightMm{"5"};
    std::string cornerX{"0"};
    std::string cornerY{"0"};
    bool invert{false};
    bool autoScale{false};
    relief::CombineMode mode{relief::CombineMode::Add};
};

/// A form filled from the selected bitmap object (its width and lower-left corner); the defaults
/// when no bitmap is selected.
BitmapReliefForm defaultBitmapForm(const document::Document& doc);
ModellingPreview previewBitmapRelief(const document::Document& doc, const BitmapReliefForm& form,
                                     const document::BitmapDecoder& decoder);

// --- Zero plane -------------------------------------------------------------------------------------

struct ZeroPlaneForm {
    std::string name{"Zero Plane"};
    std::string cellSize{"1"};
    std::string cornerX{"0"};
    std::string cornerY{"0"};
};

ModellingPreview previewZeroPlane(const document::Document& doc, const ZeroPlaneForm& form);

// --- Scale Model Z ----------------------------------------------------------------------------------

struct ScaleZForm {
    relief::ScaleZ::Kind kind{relief::ScaleZ::Kind::Percent};
    std::string value{"100"};
    bool bothSides{false};
};

/// Scaled copies of the selected components (the whole active sheet's components when none are
/// selected), in document order. `ok` is false with the reason when any one cannot be scaled.
struct ScaleZPreview {
    bool ok{false};
    std::string message;
    std::vector<document::ComponentId> ids;
    std::vector<relief::Component> components;
    double factor{1.0};   ///< Factor of the first component, for the form's read-out.
};
ScaleZPreview previewScaleZ(const document::Document& doc, const ScaleZForm& form);

/// Applies the preview as one undo step.
EditResult applyScaleZ(document::Document& doc, const ScaleZForm& form);

// --- Apply / Start New Component ----------------------------------------------------------------------

/// The common Apply / Start New Component behaviour of the creating forms. The first Apply adds the
/// component to the target level; Apply again (after the user changed a parameter) replaces that
/// same component in place, so a form can be tuned without piling up copies. "Start New Component"
/// forgets it: the next Apply adds another. Each Apply is one undo step.
class ModellingSession {
   public:
    /// `level` null: the top level of the active sheet, created ("Level 1") when the sheet has none.
    EditResult apply(document::Document& doc, relief::Component component, document::LevelId level = {});
    void startNew() { current_ = {}; }
    /// The component the last Apply made (null after `startNew`, or when it was deleted).
    document::ComponentId current(const document::Document& doc) const;
    document::ComponentId lastCreated() const { return current_; }

   private:
    document::ComponentId current_;
};

}  // namespace carvetto::viewmodel
