#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "carvetto/relief/relief.hpp"

// Sculpting engine (T-0294, ADR-0070). A stroke is a deterministic list of dabs applied to a
// working copy of a height field; the session keeps the baseline for Undo Brush, per-stroke
// undo (bounded) and Keep / Discard. Units are mm. No I/O, no UI.

namespace carvetto::relief {

enum class BrushKind { Smooth, Smudge, Deposit, Remove, UndoBrush };

/// Normal: the brush result as is. Raise: a cell may only go up. Lower: a cell may only go down.
enum class SculptMode { Normal, Raise, Lower };

/// A stamp used instead of the round kernel: the field is normalised to [0,1] over its opaque
/// cells (transparent = 0) and scaled so its longer side equals the brush diameter.
struct CustomBrush {
    HeightField stamp;
    double angleRad = 0.0;
    bool dynamicRotation = false;  ///< Also rotate by the stroke direction at each dab.
};

struct BrushParams {
    BrushKind kind = BrushKind::Deposit;
    double diameter = 10.0;    ///< mm, > 0.
    /// Deposit / Remove: mm of height per diameter of travel at kernel 1. Smooth, Smudge and
    /// UndoBrush: a fraction, clamped to 1.
    double strength = 0.5;
    double smoothness = 0.5;   ///< 0 = hard edge, 1 = falloff from the centre; in [0,1].
    SculptMode mode = SculptMode::Normal;
    bool preserveTransparency = true;
    std::shared_ptr<const CustomBrush> custom;  ///< Optional.
};

struct Dab {
    Point pos;
    double weight = 1.0;  ///< Distance covered in diameters (a lone dab is 1).
    Point delta;          ///< Movement since the previous dab (Smudge, dynamic rotation).
};

/// Brush falloff at distance d from the centre (1 inside the plateau, 0 at the radius).
double brushKernel(double distance, double diameter, double smoothness);

/// Resamples a polyline by arc length into n = ceil(length/spacing) equal pieces, one dab at
/// each piece midpoint with weight piece/diameter. Independent of the input vertex density. A
/// zero-length path gives one dab of weight 1. BadParameter on bad spacing/diameter/points.
Status dabsAlong(const std::vector<Point>& path, double spacing, double diameter, std::vector<Dab>& out);

struct SculptOptions {
    std::size_t maxUndoBytes = std::size_t{64} << 20;  ///< Per-stroke undo history cap.
};

class SculptSession {
public:
    /// Starts a session on a copy of `baseline` (also the Undo Brush reference).
    static Status begin(const HeightField& baseline, SculptOptions options, SculptSession& out);

    bool active() const { return !working_.empty(); }
    const HeightField& working() const { return working_; }
    const HeightField& baseline() const { return baseline_; }

    /// Validates the brush, then applies the dabs in order and records one undo step.
    Status applyStroke(const BrushParams& brush, const std::vector<Dab>& dabs);

    bool canUndo() const { return !undo_.empty(); }
    /// Reverts the last applied stroke exactly. False when there is none.
    bool undoStroke();

    /// Keep: the working copy becomes the new baseline; undo history is cleared.
    void keep();
    /// Discard: the working copy returns to the baseline exactly; undo history is cleared.
    void discard();

    std::size_t undoBytes() const { return undoBytes_; }

private:
    struct Patch {
        std::size_t i0 = 0, j0 = 0, w = 0, h = 0;
        std::vector<float> before;
    };
    HeightField baseline_;
    HeightField working_;
    SculptOptions options_;
    std::vector<Patch> undo_;
    std::size_t undoBytes_ = 0;
};

}  // namespace carvetto::relief
