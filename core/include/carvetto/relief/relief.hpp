#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

// Relief core (T-0281, ADR-0060): height fields, non-destructive components, levels and
// composition into the visible model. Units are mm. No I/O, no UI, no generators.
//
// A transparent cell is NaN. A cell (i, j) has its centre at origin + (i, j) * cellSize.

namespace carvetto::relief {

/// Most cells a HeightField (or a composition grid) may hold: 2^25 floats = 128 MiB.
inline constexpr std::size_t kMaxCells = std::size_t{1} << 25;

inline constexpr float kTransparent = std::numeric_limits<float>::quiet_NaN();

enum class Status {
    Ok,
    ZeroSize,      ///< Width, height or cell size is zero, negative or not finite.
    TooLarge,      ///< More than kMaxCells cells.
    BadRegion,     ///< Crop or resample region outside the field / empty.
    BadParameter,  ///< Non-finite property (anchor, angle, ...) or degenerate fade/tilt direction.
};

const char* statusMessage(Status s);

struct Point {
    double x = 0.0;
    double y = 0.0;
};

/// Grid geometry shared by fields and composition targets.
struct Grid {
    std::size_t width = 0;
    std::size_t height = 0;
    double cellSize = 1.0;
    Point origin;
};

/// Validates a grid (non-zero, finite, within kMaxCells).
Status validate(const Grid& g);

class HeightField {
public:
    HeightField() = default;

    /// Creates a field filled with `fill` (default transparent). Status != Ok leaves `out` empty.
    static Status create(const Grid& grid, float fill, HeightField& out);
    /// Creates a field from row-major data (size must be width*height).
    static Status fromData(const Grid& grid, std::vector<float> data, HeightField& out);

    const Grid& grid() const { return grid_; }
    std::size_t width() const { return grid_.width; }
    std::size_t height() const { return grid_.height; }
    double cellSize() const { return grid_.cellSize; }
    Point origin() const { return grid_.origin; }
    bool empty() const { return data_.empty(); }
    const std::vector<float>& data() const { return data_; }
    std::vector<float>& mutableData() { return data_; }

    float at(std::size_t i, std::size_t j) const { return data_[j * grid_.width + i]; }
    void set(std::size_t i, std::size_t j, float v) { data_[j * grid_.width + i] = v; }

    /// Bilinear sample at a model point. Transparent when outside the cell-centre rectangle or
    /// when any of the (up to) four contributing cells with non-zero weight is transparent.
    /// Exactly a cell value when the point is on a cell centre.
    float sample(double x, double y) const;

    /// Minimum and maximum of the opaque cells; false when every cell is transparent.
    bool minMax(float& lo, float& hi) const;

    /// Sub-rectangle of cells [i0, i0+w) x [j0, j0+h); the origin moves with it.
    Status crop(std::size_t i0, std::size_t j0, std::size_t w, std::size_t h, HeightField& out) const;

    /// Bilinear resample onto `target` (any origin/size/cell size).
    Status resample(const Grid& target, HeightField& out) const;

private:
    Grid grid_;
    std::vector<float> data_;
};

enum class CombineMode { Add, Subtract, MergeHigh, MergeLow, Multiply };

/// Combines `below` (accumulated) with `above` (the new item). NaN is transparent; see ADR-0060:
/// above transparent -> below under every mode; below transparent -> above for Add/MergeHigh/
/// MergeLow, and stays transparent for Subtract and Multiply.
float combine(CombineMode mode, float below, float above);

/// Linear Z fade: factor 1 at `anchor` falling to 1 - percent/100 at `second` (clamped beyond).
struct Fade {
    bool enabled = false;
    Point anchor;
    Point second;
    double percent = 0.0;
};

/// Tilt about the line through `pivot` perpendicular to pivot->raise; adds
/// tan(angle) * (signed distance along pivot->raise) to the height.
struct Tilt {
    bool enabled = false;
    Point pivot;
    Point raise;
    double angleRad = 0.0;
};

struct Component {
    std::string name;
    HeightField field;
    CombineMode mode = CombineMode::Add;
    double shapeHeightPercent = 100.0;
    double baseHeight = 0.0;
    Fade fade;
    Tilt tilt;
    bool visible = true;
};

/// Mirror Mode of a level (T-0295, ADR-0079). Directions are in design space, Y up. For the
/// half modes the named side is the source and is copied onto the other side; for the quadrant
/// modes the named quadrant is the source and is copied onto the other three.
enum class MirrorMode {
    None,
    LeftToRight,
    RightToLeft,
    TopToBottom,
    BottomToTop,
    QuadTopRight,
    QuadTopLeft,
    QuadBottomLeft,
    QuadBottomRight,
};

/// Clipping of a level: only cells whose centre lies inside the closed polygons (even-odd rule,
/// so a polygon inside another is a hole) stay opaque. No polygons = not clipped.
struct Clip {
    std::vector<std::vector<Point>> polygons;
};

/// Caps on a clip (also enforced by commands and the file loader).
inline constexpr std::size_t kMaxClipPolygons = 256;
inline constexpr std::size_t kMaxClipVertices = 65536;

/// Non-destructive per-level effects, applied at compose time after the level's components are
/// combined and its base height added: mirror first, then clip (so the clip bounds the result).
struct LevelEffects {
    Clip clip;
    MirrorMode mirror = MirrorMode::None;
    Point mirrorCenter;  ///< The mirror axes pass through this point (design space).
    bool any() const { return !clip.polygons.empty() || mirror != MirrorMode::None; }
};

struct Level {
    LevelEffects effects;
    std::string name;
    std::vector<Component> components;  ///< Bottom first.
    CombineMode mode = CombineMode::Add;
    double baseHeight = 0.0;
    bool visible = true;
};

/// Checks the dynamic properties (finite numbers, non-degenerate fade/tilt direction when enabled).
/// Does not look at the field. BadParameter on failure (T-0282 reuses it to validate edits).
Status validateProperties(const Component& c);

/// The value of one component at a model point after shape height, fade, tilt and base height.
/// Transparent where the stored field is. Does not modify the component.
float componentValueAt(const Component& c, double x, double y);

/// Component sampled onto `grid` with its dynamic properties applied.
Status composeComponent(const Component& c, const Grid& grid, HeightField& out);

/// The component's properties made permanent, on its own grid (equal to composeComponent on
/// c.field.grid()). Level base height is not part of a component and is never included.
Status bake(const Component& c, HeightField& out);

/// Smallest grid covering every visible component, at the finest visible cell size; the origin
/// is the minimum cell-centre corner. Error when nothing is visible or the grid is too large.
Status unionGrid(const std::vector<Level>& levels, Grid& out);

/// Composes the visible model on `grid`: per level, components combine in order with their modes
/// (first against an all-transparent field), level base height is added to opaque cells, then
/// the level combines into the model with its mode. Hidden items are skipped.
Status compose(const std::vector<Level>& levels, const Grid& grid, HeightField& out);

}  // namespace carvetto::relief
