#pragma once

#include <atomic>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/document/units.hpp"
#include "carvetto/geometry/shapes.hpp"
#include "carvetto/io/bitmap_trace.hpp"
#include "carvetto/io/raster.hpp"

// Trace Bitmap view logic (T-0260, ADR-0038, ADR-0014). Qt-free and QImage-free: the app layer
// decodes a file into an `io::RasterImage` and everything here takes that.
//
//  * `TraceForm` is the dialog's model (values in the document's display units); `validateTraceForm`
//    gives the message to show and `toTraceParams` the core parameters.
//  * `previewImage` shrinks a big image so the live preview stays fast; `TraceRunner` runs one trace
//    at a time on a worker thread with a cancel flag (the core polls it, ADR-0038).
//  * `applyTrace` puts a result into the document as ONE undo step: one new layer per colour, each
//    holding one vector object (all that colour's contours; the even-odd rule makes nested contours
//    holes) on the active sheet. All-or-nothing; the new objects end up selected.

namespace carvetto::viewmodel {

/// What the size entry means.
enum class TraceSizeBasis {
    PerPixel,      ///< `sizeValue` is the length of one pixel (display unit per pixel)
    TargetWidth,   ///< `sizeValue` is the width of the whole traced image
    TargetHeight,  ///< `sizeValue` is the height of the whole traced image
};

/// Largest result edge accepted (mm): keeps coordinates well inside `kMaxVectorOpCoordinateMm`.
inline constexpr double kTraceMaxSizeMm = 100'000.0;
/// Smallest pixel pitch accepted (mm).
inline constexpr double kTraceMinPixelMm = 1.0e-4;
/// Longest edge (pixels) of the image used for the live preview.
inline constexpr int kTracePreviewEdge = 400;

struct TraceForm {
    io::TraceMode mode{io::TraceMode::BlackWhite};
    int threshold{128};        ///< 0..255
    bool invert{false};
    int colourCount{8};        ///< 2..16
    int noiseFilter{2};        ///< pixels, 0..1000 (0/1 off)
    double cornerFit{0.5};     ///< 0..1
    TraceSizeBasis sizeBasis{TraceSizeBasis::PerPixel};
    double sizeValue{0.1};     ///< in `units`
    document::Units units{document::Units::Millimetres};
    geometry::Vec2 originMm{0, 0};  ///< bottom-left corner of the traced image, mm
};

/// Empty when valid for an image of `imageWidth` x `imageHeight` pixels; otherwise the reason.
std::string validateTraceForm(const TraceForm& form, int imageWidth, int imageHeight);

/// The millimetres per pixel the form gives for that image (error when invalid).
geometry::ShapeResult<double> traceMmPerPixel(const TraceForm& form, int imageWidth, int imageHeight);

/// Core parameters for a trace of an image of that size.
geometry::ShapeResult<io::TraceParams> toTraceParams(const TraceForm& form, int imageWidth, int imageHeight);

/// `image` shrunk (box average) so its longest edge is at most `maxEdge`; `scale` is the factor
/// original/preview (>= 1; 1 when the image is already small, then a plain copy).
struct PreviewImage {
    io::RasterImage image;
    double scale{1.0};
};
PreviewImage previewImage(const io::RasterImage& image, int maxEdge = kTracePreviewEdge);

/// Trace parameters for the preview image of `scale` (mm per pixel grows by `scale`, the noise
/// filter shrinks by `scale` squared, so the preview covers the same area as the full trace).
io::TraceParams previewParams(const io::TraceParams& full, double scale);

/// A finished trace: layers, or the reason it failed / was cancelled.
struct TraceOutcome {
    bool ok{false};
    bool cancelled{false};
    std::string message;
    std::vector<io::TraceLayer> layers;
};

/// Runs one trace at a time on a worker thread. `start`, `cancel`, `finished`, `take` and the
/// destructor are for the owner's thread; `onFinished` runs on the worker thread (post to the UI
/// thread from there; it must not touch the runner). The destructor cancels and joins; the core
/// polls the flag, so that takes milliseconds.
class TraceRunner {
   public:
    TraceRunner() = default;
    ~TraceRunner();
    TraceRunner(const TraceRunner&) = delete;
    TraceRunner& operator=(const TraceRunner&) = delete;

    /// False (nothing started) while a trace is running or its result has not been taken. The
    /// image is shared, never copied, and must not change.
    bool start(std::shared_ptr<const io::RasterImage> image, const io::TraceParams& params,
               std::function<void()> onFinished = {});
    void cancel();
    bool busy() const { return state_ != nullptr; }
    bool finished() const;
    /// The result of the finished trace and idle again; blocks until done; nullopt when idle.
    std::optional<TraceOutcome> take();

   private:
    struct State {
        std::shared_ptr<const io::RasterImage> image;
        io::TraceParams params;
        std::atomic<bool> cancel{false};
        std::atomic<bool> done{false};
        TraceOutcome outcome;
        std::function<void()> onFinished;
    };
    std::shared_ptr<State> state_;
    std::thread worker_;
};

/// Synchronous trace (same as one `TraceRunner` run), polling `cancel` when given.
TraceOutcome runTrace(const io::RasterImage& image, const io::TraceParams& params,
                      const std::atomic<bool>* cancel = nullptr);

struct TraceApplyResult {
    bool ok{false};
    std::string message;                      ///< empty on success
    std::vector<document::LayerId> layers;    ///< the new layers, in result order
    std::vector<document::ObjectId> created;  ///< the new objects (also selected)
};

/// Adds `layers` to the document as one undo step. Refuses an empty result (nothing to trace) or
/// a document without enough free ids; the document is then untouched.
TraceApplyResult applyTrace(document::Document& doc, const std::vector<io::TraceLayer>& layers,
                            io::TraceMode mode = io::TraceMode::BlackWhite);

/// Layer name for a traced colour: "Trace" for black and white, else "Trace #RRGGBB".
std::string traceLayerName(const io::TraceLayer& layer, io::TraceMode mode);

}  // namespace carvetto::viewmodel
