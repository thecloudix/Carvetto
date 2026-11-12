#include "carvetto/viewmodel/bitmap_trace.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

#include "carvetto/document/commands.hpp"
#include "carvetto/geometry/cancel.hpp"

namespace carvetto::viewmodel {

namespace {

bool finite(double v) { return std::isfinite(v); }

}  // namespace

std::string validateTraceForm(const TraceForm& f, int w, int h) {
    if (w <= 0 || h <= 0) return "There is no image to trace.";
    if (f.threshold < 0 || f.threshold > 255) return "The threshold must be between 0 and 255.";
    if (f.mode == io::TraceMode::Colour && (f.colourCount < 2 || f.colourCount > 16)) {
        return "The number of colours must be between 2 and 16.";
    }
    if (f.noiseFilter < 0 || f.noiseFilter > 1000) return "The noise filter must be between 0 and 1000 pixels.";
    if (!finite(f.cornerFit) || f.cornerFit < 0.0 || f.cornerFit > 1.0) return "The corner fit must be between 0 and 1.";
    if (!finite(f.originMm.x) || !finite(f.originMm.y) || std::fabs(f.originMm.x) > 1.0e6 || std::fabs(f.originMm.y) > 1.0e6) {
        return "The position is out of range.";
    }
    if (!finite(f.sizeValue) || f.sizeValue <= 0.0) return "The size must be greater than zero.";
    const double mm = document::fromDisplay(f.sizeValue, f.units);
    double perPixel = mm;
    double widthMm = mm * w, heightMm = mm * h;
    if (f.sizeBasis == TraceSizeBasis::TargetWidth) {
        perPixel = mm / w;
        widthMm = mm;
        heightMm = perPixel * h;
    } else if (f.sizeBasis == TraceSizeBasis::TargetHeight) {
        perPixel = mm / h;
        heightMm = mm;
        widthMm = perPixel * w;
    }
    if (perPixel < kTraceMinPixelMm) return "The size is too small: a pixel would be under 0.0001 mm.";
    if (widthMm > kTraceMaxSizeMm || heightMm > kTraceMaxSizeMm) return "The size is too large: the result would exceed 100 m.";
    return {};
}

geometry::ShapeResult<double> traceMmPerPixel(const TraceForm& f, int w, int h) {
    const std::string problem = validateTraceForm(f, w, h);
    if (!problem.empty()) return problem;
    const double mm = document::fromDisplay(f.sizeValue, f.units);
    switch (f.sizeBasis) {
        case TraceSizeBasis::PerPixel: return mm;
        case TraceSizeBasis::TargetWidth: return mm / w;
        case TraceSizeBasis::TargetHeight: return mm / h;
    }
    return mm;
}

geometry::ShapeResult<io::TraceParams> toTraceParams(const TraceForm& f, int w, int h) {
    const auto mm = traceMmPerPixel(f, w, h);
    if (!mm.ok()) return mm.error();
    io::TraceParams p;
    p.mode = f.mode;
    p.threshold = static_cast<std::uint8_t>(f.threshold);
    p.invert = f.invert;
    p.colourCount = f.colourCount;
    p.noiseFilter = f.noiseFilter;
    p.cornerFit = f.cornerFit;
    p.mmPerPixel = mm.value();
    p.origin = f.originMm;
    return p;
}

PreviewImage previewImage(const io::RasterImage& im, int maxEdge) {
    PreviewImage out;
    if (!im.valid() || maxEdge < 1) return out;
    const int longest = std::max(im.width, im.height);
    if (longest <= maxEdge) {
        out.image = im;
        return out;
    }
    const double scale = static_cast<double>(longest) / maxEdge;
    const int w = std::max(1, static_cast<int>(std::lround(im.width / scale)));
    const int h = std::max(1, static_cast<int>(std::lround(im.height / scale)));
    out.image.width = w;
    out.image.height = h;
    out.image.rgb.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 3, 0);
    for (int y = 0; y < h; ++y) {
        const int y0 = static_cast<int>(static_cast<long long>(y) * im.height / h);
        const int y1 = std::max(y0 + 1, static_cast<int>(static_cast<long long>(y + 1) * im.height / h));
        for (int x = 0; x < w; ++x) {
            const int x0 = static_cast<int>(static_cast<long long>(x) * im.width / w);
            const int x1 = std::max(x0 + 1, static_cast<int>(static_cast<long long>(x + 1) * im.width / w));
            unsigned long long sum[3] = {0, 0, 0};
            for (int yy = y0; yy < y1; ++yy) {
                for (int xx = x0; xx < x1; ++xx) {
                    const std::size_t i = (static_cast<std::size_t>(yy) * static_cast<std::size_t>(im.width) + static_cast<std::size_t>(xx)) * 3;
                    for (int c = 0; c < 3; ++c) sum[c] += im.rgb[i + static_cast<std::size_t>(c)];
                }
            }
            const unsigned long long count = static_cast<unsigned long long>(y1 - y0) * static_cast<unsigned long long>(x1 - x0);
            const std::size_t o = (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 3;
            for (int c = 0; c < 3; ++c) out.image.rgb[o + static_cast<std::size_t>(c)] = static_cast<std::uint8_t>((sum[c] + count / 2) / count);
        }
    }
    out.scale = scale;
    return out;
}

io::TraceParams previewParams(const io::TraceParams& full, double scale) {
    io::TraceParams p = full;
    if (scale > 1.0) {
        p.mmPerPixel = full.mmPerPixel * scale;
        p.noiseFilter = static_cast<int>(std::lround(full.noiseFilter / (scale * scale)));
    }
    return p;
}

TraceOutcome runTrace(const io::RasterImage& image, const io::TraceParams& params, const std::atomic<bool>* cancel) {
    TraceOutcome out;
    geometry::CancelScope scope(cancel);
    auto r = io::traceBitmap(image, params);
    if (cancel != nullptr && cancel->load(std::memory_order_relaxed)) {
        out.cancelled = true;
        out.message = "Trace cancelled.";
        return out;
    }
    if (!r.ok()) {
        out.message = r.error();
        return out;
    }
    out.ok = true;
    out.layers = std::move(r.value());
    return out;
}

TraceRunner::~TraceRunner() {
    if (state_) state_->cancel.store(true);
    if (worker_.joinable()) worker_.join();
}

bool TraceRunner::start(std::shared_ptr<const io::RasterImage> image, const io::TraceParams& params,
                        std::function<void()> onFinished) {
    if (state_ || !image) return false;
    if (worker_.joinable()) worker_.join();
    auto st = std::make_shared<State>();
    st->image = std::move(image);
    st->params = params;
    st->onFinished = std::move(onFinished);
    state_ = st;
    worker_ = std::thread([st]() {
        try {
            st->outcome = runTrace(*st->image, st->params, &st->cancel);
        } catch (const std::exception& e) {
            st->outcome = TraceOutcome{};
            st->outcome.message = std::string("Trace failed: ") + e.what();
        } catch (...) {
            st->outcome = TraceOutcome{};
            st->outcome.message = "Trace failed.";
        }
        st->done.store(true);
        if (st->onFinished) st->onFinished();
    });
    return true;
}

void TraceRunner::cancel() {
    if (state_) state_->cancel.store(true);
}

bool TraceRunner::finished() const {
    return state_ != nullptr && state_->done.load();
}

std::optional<TraceOutcome> TraceRunner::take() {
    if (!state_) return std::nullopt;
    if (worker_.joinable()) worker_.join();
    TraceOutcome out = std::move(state_->outcome);
    state_.reset();
    return out;
}

std::string traceLayerName(const io::TraceLayer& layer, io::TraceMode mode) {
    if (mode == io::TraceMode::BlackWhite) return "Trace";
    char buf[24];
    std::snprintf(buf, sizeof buf, "Trace #%02X%02X%02X", layer.r, layer.g, layer.b);
    return buf;
}

TraceApplyResult applyTrace(document::Document& doc, const std::vector<io::TraceLayer>& layers, io::TraceMode mode) {
    TraceApplyResult result;
    std::vector<const io::TraceLayer*> usable;
    for (const io::TraceLayer& l : layers) {
        if (!l.contours.empty()) usable.push_back(&l);
    }
    if (usable.empty()) {
        result.message = "Nothing was traced: no shapes were found with these settings.";
        return result;
    }
    if (!doc.idGenerator().canAllocate(usable.size() * 2)) {
        result.message = "The document has no free ids left.";
        return result;
    }
    doc.undoStack().breakCoalescing();
    doc.undoStack().beginMacro("Trace bitmap");
    for (const io::TraceLayer* l : usable) {
        document::Layer layer;
        layer.name = traceLayerName(*l, mode);
        layer.colour = document::Colour{l->r, l->g, l->b};
        auto addLayer = std::make_unique<document::AddLayerCommand>(std::move(layer));
        const document::AddLayerCommand* rawLayer = addLayer.get();
        if (!doc.execute(std::move(addLayer))) continue;
        const document::LayerId layerId = rawLayer->id();
        result.layers.push_back(layerId);
        auto addObject = std::make_unique<document::AddObjectCommand>(
            document::Object::vector(l->contours, layerId, doc.activeSheet()));
        const document::AddObjectCommand* rawObject = addObject.get();
        if (doc.execute(std::move(addObject))) result.created.push_back(rawObject->id());
    }
    doc.undoStack().endMacro();
    doc.selection().assign(result.created);
    result.ok = !result.created.empty();
    if (!result.ok) result.message = "The traced shapes could not be added to the document.";
    return result;
}

}  // namespace carvetto::viewmodel
