#pragma once

#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/viewmodel/model_view.hpp"
#include "carvetto/viewmodel/tools.hpp"

// Drawing in the 3D (Model) view (T-0345, ADR-0121): the basic shapes are drawn by clicking on the
// model's zero plane (design plane, model height 0). The pointer position is turned into a point of
// that plane by a ray-to-plane pick against the orthographic orbit camera, then handed to the ordinary
// `ToolController`, so the result is an ordinary document vector made by one `AddObjectCommand`
// (one undo step) that stays re-editable, exactly like a vector drawn on the 2D canvas.

namespace carvetto::viewmodel {

/// Result of picking the zero plane under a pixel of the 3D view.
struct PlanePick {
    bool ok{false};
    geometry::Vec2 mm{};        ///< design-plane point in millimetres
    double pxPerMm{0.0};        ///< on-screen scale of the plane at the pick (geometric mean of its axes)
    std::string refusal;        ///< why the pick was refused (empty when ok)
};

/// A camera looking along the plane (elevation below about 6 degrees) cannot place points on it.
inline constexpr double kMinPlaneSine = 0.1;
/// Picks further than this many model diagonals from the model's centre are refused.
inline constexpr double kMaxPickDiagonals = 4.0;

/// Where the view ray through pixel (px, py) of a render with `options` from `camera` meets the zero
/// plane (model height 0). Refused (not ok, with a reason) for an empty surface, the wrapped rotary
/// view (`wrapped`: the flat design plane is not what is on screen), a view that grazes the plane,
/// non-finite input, and picks far beyond the model. Deterministic.
PlanePick pickZeroPlane(const ModelSurface& surface, const OrbitCamera& camera, const ModelRenderOptions& options,
                        double px, double py, bool wrapped = false);

/// `surface` when it is usable, otherwise a flat zero-height surface over the job's material so
/// there is always a plane to draw on (empty surface when the job has no usable material).
ModelSurface drawingSurface(const document::Document& doc, const ModelSurface& surface);

/// The visible vectors of the active sheet as paths on the zero plane (z = 0), for drawing over the
/// flat model.
std::vector<ModelPath> planeVectorPaths(const document::Document& doc, std::uint32_t colour = 0xFF1A5FB4u);

/// The 3D view's pointer mode: a drawing shape, or (T-0346, ADR-0125) Select / NodeEdit, which edit the
/// document's vectors through the very same `ToolController` select (move / scale / rotate handles) and
/// node-edit tools the 2D canvas uses.
enum class Draw3DShape { Line, Polyline, Rectangle, Circle, Ellipse, Polygon, Select, NodeEdit };
const char* draw3DShapeName(Draw3DShape shape);

/// What the 3D view shows the controller for each pointer event.
struct Draw3DView {
    const ModelSurface* surface{nullptr};
    const OrbitCamera* camera{nullptr};
    const ModelRenderOptions* options{nullptr};
    bool wrapped{false};
};

struct Draw3DResult {
    ToolResult tool;       ///< what the shape tool did (`committed`: a vector was added)
    bool refused{false};   ///< the pick was refused; nothing happened
    std::string message;   ///< reason when refused or when the tool refused the input
};

class Draw3DController {
   public:
    explicit Draw3DController(document::Document* doc = nullptr);

    /// Document drawn into (not owned); cancels anything in progress.
    void setDocument(document::Document* doc);
    document::Document* document() const { return doc_; }
    /// Chooses the shape (cancels one in progress).
    void setShape(Draw3DShape shape);
    Draw3DShape shape() const { return shape_; }
    /// The shape tools' options and numeric entry (corner style, polygon sides, ...).
    ToolController& tools() { return tool_; }
    const ToolController& tools() const { return tool_; }

    Draw3DResult pointerMove(const Draw3DView& view, double px, double py, Modifiers mods = {});
    Draw3DResult pointerPress(const Draw3DView& view, double px, double py, Modifiers mods = {});
    Draw3DResult pointerRelease(const Draw3DView& view, double px, double py, Modifiers mods = {});
    Draw3DResult doubleClick(const Draw3DView& view, double px, double py, Modifiers mods = {});
    Draw3DResult key(ToolKey k);

    bool active() const { return tool_.active(); }
    /// True for the editing modes (Select, NodeEdit) as opposed to drawing a shape.
    bool editing() const { return shape_ == Draw3DShape::Select || shape_ == Draw3DShape::NodeEdit; }
    /// Draft outline in design millimetres (drape it on the plane with `projectModelPoint`).
    ToolPreview preview() const { return tool_.preview(); }

   private:
    // Picks and aims the tool's view so its pseudo-pixels (mm x scale) map back to the picked point.
    bool prepare(const Draw3DView& view, double px, double py, geometry::Vec2& pseudo, Draw3DResult& out);
    Draw3DResult afterPress(Draw3DResult r);

    document::Document* doc_{nullptr};
    ToolController tool_;
    Draw3DShape shape_{Draw3DShape::Rectangle};
    int linePoints_{0};
};

}  // namespace carvetto::viewmodel
