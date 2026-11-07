#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/transform2.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/viewmodel/selection_tools.hpp"

// Qt-free move, scale, rotate, mirror, align and space operations on the current selection
// (T-0205, ADR-0014, ADR-0027, ADR-0029). Every operation is ONE undo step (a macro of
// `TransformObjectsCommand`s, plus a `PasteObjectsCommand` for a mirrored copy), is all-or-nothing
// (it is validated completely before the document is touched; a refusal changes nothing, not even
// the undo history) and leaves the selection as it was (a mirrored copy selects the copies).
//
// Which objects are touched ("targets"):
//   * Vector and bitmap leaves on the active sheet whose layer, and every ancestor group's layer,
//     is visible and unlocked. Groups are resolved to their leaves, so a group transforms whole;
//     hidden, locked and other-sheet objects in the selection are never touched.
//   * The bounds used for anchors are the exact bounds of the target contours (not stored values).
//
// Creation parameters: kept for translations, and for 90-degree multiples only when rebuilding the
// shape from the turned parameters reproduces the turned contours span by span (start point
// included); every other transform (mirror, any scale, other angles) makes a plain vector. This is
// the rule of `TransformObjectsCommand` (ADR-0029).
//
// Directions: anchors use the `HandleKind` names of the selection box (Top is +Y, Left is -X).
// Angles are radians, counter-clockwise positive, in the core; the numeric form converts degrees.

namespace carvetto::viewmodel {

/// Largest coordinate magnitude (mm) an operation accepts for the input and for the result.
inline constexpr double kMaxTransformCoordinateMm = 1.0e9;
/// Smallest scale factor accepted (a factor of 0 or below is refused).
inline constexpr double kMinScaleFactor = 1.0e-6;
/// Largest scale factor accepted.
inline constexpr double kMaxScaleFactor = 1.0e6;

struct TransformResult {
    bool ok{false};
    /// Empty on success; otherwise the reason, for a message box.
    std::string message;
    /// Number of objects (leaves) changed; 0 with `ok` means the request was a no-op (nothing pushed).
    std::size_t objects{0};
    /// Objects created (mirrored copies), now selected.
    std::vector<document::ObjectId> created;
};

/// Target leaves of the selection, in selection order (groups expanded in child order, each leaf
/// once).
std::vector<document::ObjectId> transformTargets(const document::Document& doc);
/// Exact bounds of the targets; nullopt when there are none.
std::optional<geometry::Box2> transformBounds(const document::Document& doc);
/// Point of `box` named by a handle kind (Top is the maximum Y).
geometry::Vec2 anchorPoint(const geometry::Box2& box, HandleKind anchor);
/// Menu enabling: there is at least one target.
bool canTransform(const document::Document& doc);

/// Applies `m` to the targets as one undo step (generic building block of the operations below).
/// Refused when `m` is not finite or invertible, or the result leaves +-1e9 mm.
TransformResult applyTransform(document::Document& doc, const geometry::Transform2& m,
                               const std::string& text = "Transform");

// --- Move ---------------------------------------------------------------------------------------

struct MoveOptions {
    /// Relative: translate by (xMm, yMm). Absolute: put `anchor` of the selection bounds at (xMm, yMm).
    bool absolute{false};
    HandleKind anchor{HandleKind::Centre};
    double xMm{0.0};
    double yMm{0.0};
};
TransformResult moveSelection(document::Document& doc, const MoveOptions& options);

// --- Scale --------------------------------------------------------------------------------------

enum class ScaleMode {
    Selection,    ///< the selection scales as one block about the anchor of its bounds
    Individually  ///< every top-level object scales about the same anchor of its own bounds
};

struct ScaleOptions {
    ScaleMode mode{ScaleMode::Selection};
    HandleKind anchor{HandleKind::Centre};
    /// Factors in [kMinScaleFactor, kMaxScaleFactor]; zero, negative and NaN are refused.
    double scaleX{1.0};
    double scaleY{1.0};
};
TransformResult scaleSelection(document::Document& doc, const ScaleOptions& options);

/// Factors that bring `bounds` to the requested size. A missing value keeps that dimension, or
/// (with `linkXY`) follows the other one. nullopt when a requested dimension is not finite or <= 0,
/// or when the bounds are too thin to be scaled to a size (zero extent on a requested axis).
struct ScaleFactors {
    double x{1.0};
    double y{1.0};
};
std::optional<ScaleFactors> scaleFactorsForSize(const geometry::Box2& bounds, std::optional<double> widthMm,
                                                std::optional<double> heightMm, bool linkXY);

// --- Rotate -------------------------------------------------------------------------------------

struct RotateOptions {
    double angleRad{0.0};
    /// Relative: rotate by `angleRad`. Absolute: rotate until the orientation is `angleRad`; the
    /// current orientation is the angle parameter of a single selected ellipse, polygon or star,
    /// and 0 for everything else.
    bool absolute{false};
    /// Pivot: coordinates when set, else the named point of the selection bounds.
    std::optional<geometry::Vec2> pivotMm;
    HandleKind pivotAnchor{HandleKind::Centre};
};
TransformResult rotateSelection(document::Document& doc, const RotateOptions& options);
/// Current orientation (radians) used by absolute rotation.
double selectionOrientation(const document::Document& doc);

// --- Mirror -------------------------------------------------------------------------------------

enum class MirrorAxis {
    Horizontal,  ///< flips left and right (X reverses); the axis is a vertical line
    Vertical     ///< flips top and bottom (Y reverses); the axis is a horizontal line
};

struct MirrorOptions {
    MirrorAxis axis{MirrorAxis::Horizontal};
    /// Keep the originals and create mirrored copies (selected afterwards).
    bool createCopy{false};
    /// Use the minimum-area rotated bounding box of the selection: the axis passes through its
    /// centre along its sides (Horizontal = across its first side direction).
    bool useRotatedBounds{false};
    /// Mirror about the centre of the active sheet's material instead of the selection centre.
    bool aboutMaterialCentre{false};
    /// Mirror about the last-selected object, which must be a single straight line; it is not
    /// mirrored itself and `axis`, `useRotatedBounds` and `aboutMaterialCentre` are ignored.
    bool aboutSelectedLine{false};
};
TransformResult mirrorSelection(document::Document& doc, const MirrorOptions& options);

// --- Align and space ------------------------------------------------------------------------------

enum class AlignKind {
    Centre,            ///< centre onto the reference's centre (both axes)
    VerticalCentre,    ///< centre Y onto the reference's centre Y (moves vertically only)
    HorizontalCentre,  ///< centre X onto the reference's centre X (moves horizontally only)
    Left,              ///< left edge onto the reference's left edge
    Right,             ///< right edge onto the reference's right edge
    OutsideTop,        ///< bottom edge onto the reference's top edge (placed above it)
    OutsideBottom      ///< top edge onto the reference's bottom edge (placed below it)
};

/// Moves every selected top-level object but the last-selected one (the reference) by a
/// translation. Needs two operands with geometry.
TransformResult alignToLast(document::Document& doc, AlignKind kind);

enum class CentreAxes { Both, XOnly, YOnly };
/// Moves the selection as a block so the centre of its bounds is the centre of the active
/// sheet's material (on the chosen axes).
TransformResult centreInMaterial(document::Document& doc, CentreAxes axes = CentreAxes::Both);

enum class SpaceDirection { Horizontal, Vertical };
struct SpaceOptions {
    SpaceDirection direction{SpaceDirection::Horizontal};
    /// Order by selection order instead of by position (first and last in that order stay put).
    bool selectionOrder{false};
    /// The last-selected object is a container: the others are spread inside its bounds with equal
    /// gaps, also to its edges. The container stays.
    bool insideLast{false};
};
/// Even spacing (equal gaps between the bounding boxes). Without a container, the first and last
/// operand stay and the ones between move; needs three operands (two with `insideLast`: one
/// container and at least one other).
TransformResult spaceSelection(document::Document& doc, const SpaceOptions& options);

}  // namespace carvetto::viewmodel
