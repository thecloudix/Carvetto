#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/transform2.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/viewmodel/selection_tools.hpp"

// Interactive move / scale / rotate of the selection (T-0205, ADR-0029). The pointer positions are
// in millimetres and already snapped by the caller (`ToolController`). The gesture keeps the
// transform of the whole drag as one absolute map from the start state and pushes only the change
// since the last step, as `TransformObjectsCommand`s that coalesce on the undo stack: any number
// of steps is ONE undo step with the same final geometry as the single numeric transform.
//
// Modifiers (feature inventory, "Common on-canvas handles"):
//   Move    Alt: constrain to one axis (the dominant one).
//   Scale   corner handle: proportional; Alt: free (non-proportional); Shift: about the centre.
//           edge handle: one axis; Shift: proportional.
//   Rotate  Alt: steps of 15 degrees (of the total rotation since the press).
// A step that would make a scale factor <= 0 (past the anchor) or leave +-1e9 mm is skipped; the
// geometry stays at the last valid step.

namespace carvetto::viewmodel {

enum class GestureKind { None, Move, Scale, Rotate };

class TransformGesture {
   public:
    /// Rotation step for Alt, radians.
    static constexpr double kRotateStepRad = 15.0 * 3.14159265358979323846 / 180.0;

    /// Starts a gesture on the current targets of the selection. `handle` is the grabbed handle
    /// (Scale), the rotation corner is only used for display; `startMm` is the snapped press point
    /// and `pivotMm` the rotation pivot (Rotate). False (nothing started) when there is nothing to
    /// transform or the gesture is degenerate (rotating with the pointer on the pivot).
    bool begin(document::Document& doc, GestureKind kind, HandleKind handle, geometry::Vec2 startMm,
               geometry::Vec2 pivotMm);
    /// Moves the gesture to `pointMm`. True when the document changed (a step was pushed).
    bool update(document::Document& doc, geometry::Vec2 pointMm, bool shift, bool ctrl, bool alt);
    /// Ends the gesture; the changes stay as one undo step.
    void end(document::Document& doc);
    /// Reverts the gesture's changes (undoes its step). False when there was nothing to revert.
    bool cancel(document::Document& doc);

    bool active() const { return kind_ != GestureKind::None; }
    GestureKind kind() const { return kind_; }
    /// Objects the gesture moves (for excluding them from snapping).
    const std::vector<document::ObjectId>& targets() const { return targets_; }
    /// The net transform applied so far.
    const geometry::Transform2& applied() const { return applied_; }

   private:
    geometry::Transform2 absoluteFor(geometry::Vec2 pointMm, bool shift, bool alt, bool* valid);

    GestureKind kind_{GestureKind::None};
    HandleKind handle_{HandleKind::Centre};
    geometry::Vec2 start_{};
    geometry::Vec2 pivot_{};
    geometry::Box2 bounds_;
    std::vector<document::ObjectId> targets_;
    geometry::Transform2 applied_;
    bool pushed_{false};
    std::uint64_t generation_{0};
    // Rotate: unwrapped total angle.
    double totalAngle_{0.0};
    double lastAngle_{0.0};
};

}  // namespace carvetto::viewmodel
