#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/relief/relief.hpp"
#include "carvetto/relief/sculpt.hpp"
#include "carvetto/viewmodel/properties_model.hpp"

// Qt-free logic of the Sculpt mode (T-0302, ADR-0084): the brush palette and its settings,
// brush presets, and `SculptController`, which drives one `relief::SculptSession` on a component
// of the document. Painting works on a working copy; only Keep touches the document (one undo
// step through ReplaceReliefComponentCommand). Positions are design-space mm.

namespace carvetto::viewmodel {

/// The five brushes of the palette, in palette order.
const std::vector<relief::BrushKind>& allBrushKinds();
const char* brushKindName(relief::BrushKind kind);
const char* sculptModeName(relief::SculptMode mode);

/// What the sliders and boxes of the Sculpt panel hold.
struct BrushSettings {
    relief::BrushKind kind{relief::BrushKind::Deposit};
    double diameter{10.0};     ///< mm.
    double strength{0.5};      ///< See `strengthRange`.
    double smoothness{0.5};    ///< 0 hard edge .. 1 falloff from the centre.
    relief::SculptMode mode{relief::SculptMode::Normal};
    bool preserveTransparency{true};
    /// Optional stamp from an image (T-0309); replaces the round kernel. Not saved in presets.
    std::shared_ptr<const relief::CustomBrush> custom;
    bool operator==(const BrushSettings&) const = default;
};

/// A picture handed over by the app: row-major 8-bit RGBA, `width * height * 4` bytes.
struct RgbaImage {
    std::size_t width{0};
    std::size_t height{0};
    std::vector<std::uint8_t> rgba;
};

inline constexpr std::size_t kMaxStampDimension = 256;

/// Builds a custom brush stamp from `image` (T-0309): the luminance is the height (bright = high,
/// or the reverse with `invert`), cells with alpha < 128 are transparent (the brush has no effect
/// there). Images larger than `kMaxStampDimension` are box-averaged down. Null with `problem` set
/// when the image is empty, malformed, too large (> 16384 per side) or has no opaque pixel.
std::shared_ptr<const relief::CustomBrush> customBrushFromImage(const RgbaImage& image, bool invert,
                                                                double angleRad, bool dynamicRotation,
                                                                std::string& problem);

struct Range {
    double lo{0.0};
    double hi{1.0};
};
Range diameterRange();
/// Deposit and Remove: mm of height per diameter of travel (0..2); the others: a fraction (0..1).
Range strengthRange(relief::BrushKind kind);

/// Clamps every value into its range (a kind change re-clamps the strength).
BrushSettings clampedBrush(BrushSettings s);
/// Empty when the settings are usable, otherwise the reason.
std::string brushProblem(const BrushSettings& s);
relief::BrushParams toBrushParams(const BrushSettings& s);

// --- Presets ---------------------------------------------------------------------------------------

struct BrushPreset {
    std::string name;
    BrushSettings brush;
    bool builtin{false};
};

/// Built-in presets followed by the user's, saved for the session. Names are unique; saving a name
/// again replaces the user's preset, a built-in name is refused.
class BrushPresets {
   public:
    BrushPresets();
    const std::vector<BrushPreset>& all() const { return presets_; }
    const BrushPreset* find(const std::string& name) const;
    /// Empty on success, otherwise the reason (empty or over-long name, built-in name).
    std::string save(const std::string& name, const BrushSettings& brush);
    /// False for a built-in or unknown name.
    bool remove(const std::string& name);

    /// The user presets as text for persistence across sessions (T-0309): a "carvetto-brush-presets 1"
    /// header and one tab-separated line per preset. Built-ins and custom stamps are not written.
    std::string serialiseUser() const;
    /// Adds the presets of `text` (as written by `serialiseUser`); lines that are malformed, invalid,
    /// built-in names or past the cap of 256 user presets are skipped. Returns how many were added
    /// or replaced. Never throws on hostile text.
    std::size_t loadUser(const std::string& text);

   private:
    std::vector<BrushPreset> presets_;
};

// --- Controller ------------------------------------------------------------------------------------

/// The component a sculpt session should start on: the single selected component, else the only
/// component of the active sheet. Null when there is none or the choice is ambiguous.
document::ComponentId defaultSculptTarget(const document::Document& doc);

/// A stroke is frozen into a chunk after this many brush diameters of path.
inline constexpr double kChunkDiameters = 30.0;

class SculptController {
   public:
    /// Starts on `id` (null: `defaultSculptTarget`). Ends a running session first (discarding it).
    EditResult begin(const document::Document& doc, document::ComponentId id = {});
    /// Ends the session; uncommitted strokes are dropped.
    void end();
    bool active() const { return session_.active(); }
    document::ComponentId target() const { return target_; }
    /// The field being painted; null when no session.
    const relief::HeightField* working() const { return session_.active() ? &session_.working() : nullptr; }
    /// True when the working copy differs from the last kept state.
    bool dirty() const;
    /// True when the component in `doc` still holds the last kept state (false after an outside
    /// edit such as an Undo of the Keep, or when the component is gone).
    bool documentMatches(const document::Document& doc) const;

    /// Pressing the mouse: applies a single dab at `at`. The stroke stays open for `extendStroke`.
    EditResult beginStroke(const BrushSettings& brush, relief::Point at);
    /// Moving with the button down: the whole path so far is re-applied, so the result does not
    /// depend on how often the mouse reported.
    EditResult extendStroke(relief::Point at);
    /// Releasing: the stroke becomes one undo step of the session.
    void endStroke();
    bool strokeOpen() const { return strokeOpen_; }
    /// Session steps the open stroke currently owns (1 until it is long enough to be chunked).
    std::size_t openStrokeSteps() const { return openSteps_; }
    /// A complete stroke along `path` (begin, extend, end).
    EditResult stroke(const BrushSettings& brush, const std::vector<relief::Point>& path);

    bool canUndoStroke() const { return !strokeSteps_.empty() && !strokeOpen_; }
    /// Reverts the last stroke ("Undo Stroke"); the Undo Brush of the palette paints back to the
    /// last kept state instead.
    bool undoStroke();
    /// Keep: writes the working copy into the component as one undo step; painting continues from it.
    EditResult keep(document::Document& doc);
    /// Discard: back to the last kept state.
    void discard();

   private:
    EditResult applyTail();

    relief::SculptSession session_;
    document::ComponentId target_;
    BrushSettings brush_;
    /// The part of the open stroke not yet frozen; re-applied whole on every move. A long stroke is
    /// cut into chunks of `kChunkDiameters` brush diameters so a move costs a bounded amount.
    std::vector<relief::Point> path_;
    double tailLength_{0.0};
    bool strokeOpen_{false};
    bool tailApplied_{false};   ///< A session step exists for `path_`.
    std::size_t openSteps_{0};  ///< Session steps of the open stroke.
    std::vector<std::size_t> strokeSteps_;  ///< Session steps of each finished stroke since the last Keep.
};

}  // namespace carvetto::viewmodel
