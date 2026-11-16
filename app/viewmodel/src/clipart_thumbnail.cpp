#include "carvetto/viewmodel/clipart_thumbnail.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <utility>

#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/io/raster.hpp"

namespace carvetto::viewmodel {

namespace {

constexpr int kPad = 3;
/// A cache smaller than one largest thumbnail could never hold anything.
constexpr std::size_t kMinCacheBytes = 1024U * 1024U;
constexpr std::uint8_t kBackground = 245;
constexpr std::uint8_t kInk = 40;

ClipartRender fail(std::string why) {
    ClipartRender r;
    r.error = std::move(why);
    return r;
}

struct Canvas {
    int w{0};
    int h{0};
    std::vector<std::uint8_t> rgb;
    void plot(int x, int y) {
        if (x < 0 || y < 0 || x >= w || y >= h) {
            return;
        }
        const std::size_t at = (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 3U;
        rgb[at] = rgb[at + 1] = rgb[at + 2] = kInk;
    }
};

/// Liang-Barsky clip of the segment to [0,w-1] x [0,h-1]; false when nothing is inside.
bool clipSegment(double& x0, double& y0, double& x1, double& y1, double xmax, double ymax) {
    double t0 = 0.0;
    double t1 = 1.0;
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double p[4] = {-dx, dx, -dy, dy};
    const double q[4] = {x0, xmax - x0, y0, ymax - y0};
    for (int i = 0; i < 4; ++i) {
        if (p[i] == 0.0) {
            if (q[i] < 0.0) {
                return false;
            }
        } else {
            const double t = q[i] / p[i];
            if (p[i] < 0.0) {
                t0 = std::max(t0, t);
            } else {
                t1 = std::min(t1, t);
            }
        }
    }
    if (t0 > t1) {
        return false;
    }
    const double nx0 = x0 + t0 * dx;
    const double ny0 = y0 + t0 * dy;
    x1 = x0 + t1 * dx;
    y1 = y0 + t1 * dy;
    x0 = nx0;
    y0 = ny0;
    return true;
}

void drawSegment(Canvas& c, double x0, double y0, double x1, double y1) {
    if (!std::isfinite(x0) || !std::isfinite(y0) || !std::isfinite(x1) || !std::isfinite(y1)) {
        return;
    }
    if (!clipSegment(x0, y0, x1, y1, c.w - 1, c.h - 1)) {
        return;
    }
    const double len = std::max(std::fabs(x1 - x0), std::fabs(y1 - y0));
    const int n = std::max(1, static_cast<int>(std::ceil(len)));
    for (int i = 0; i <= n; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(n);
        c.plot(static_cast<int>(std::lround(x0 + (x1 - x0) * t)), static_cast<int>(std::lround(y0 + (y1 - y0) * t)));
    }
}

ClipartRender renderVector(std::span<const std::uint8_t> bytes, ClipartKind kind, int edge, const ClipartLimits& limits) {
    const ClipartGeometry g = parseClipartVectors(bytes, kind, limits);
    if (!g.ok) {
        return fail(g.error);
    }
    geometry::Box2 box;
    for (const geometry::Contour& c : g.contours) {
        box.expand(c.bounds());
    }
    if (box.isEmpty()) {
        return fail(g.textItems > 0 ? "the drawing holds only text, which thumbnails do not show"
                                    : "the drawing holds no vectors");
    }
    const double w = box.width();
    const double h = box.height();
    const double longest = std::max(w, h);
    if (!std::isfinite(longest)) {
        return fail("the drawing is too large to measure");
    }
    const int usable = edge - 2 * kPad - 1;
    const double scale = longest > 0.0 ? static_cast<double>(usable) / longest : 1.0;
    Canvas canvas;
    canvas.w = static_cast<int>(std::lround(w * scale)) + 1 + 2 * kPad;
    canvas.h = static_cast<int>(std::lround(h * scale)) + 1 + 2 * kPad;
    canvas.rgb.assign(static_cast<std::size_t>(canvas.w) * static_cast<std::size_t>(canvas.h) * 3U, kBackground);

    geometry::Tolerance tol;
    tol.chordal = std::max(longest / (static_cast<double>(edge) * 8.0), 1e-9);
    ClipartRender out;
    const auto px = [&](geometry::Vec2 p) { return kPad + (p.x - box.min.x) * scale; };
    const auto py = [&](geometry::Vec2 p) { return static_cast<double>(canvas.h - 1 - kPad) - (p.y - box.min.y) * scale; };
    for (const geometry::Contour& c : g.contours) {
        if (c.isEmpty()) {
            continue;
        }
        const geometry::Polyline poly = c.polygonise(tol);
        const std::size_t segs = poly.segmentCount();
        if (segs == 0 && poly.vertexCount() == 1) {
            canvas.plot(static_cast<int>(std::lround(px(poly.points[0]))), static_cast<int>(std::lround(py(poly.points[0]))));
        }
        for (std::size_t i = 0; i < segs; ++i) {
            if (out.segmentsDrawn >= kMaxThumbSegments) {
                out.truncated = true;
                break;
            }
            const geometry::Vec2 a = poly.segmentStart(i);
            const geometry::Vec2 b = poly.segmentEnd(i);
            drawSegment(canvas, px(a), py(a), px(b), py(b));
            ++out.segmentsDrawn;
        }
        if (out.truncated) {
            break;
        }
    }
    char buf[96];
    std::snprintf(buf, sizeof buf, "%.1f x %.1f mm, %zu shape%s", w, h, g.contours.size(), g.contours.size() == 1 ? "" : "s");
    out.info = buf;
    out.ok = true;
    out.thumb.width = canvas.w;
    out.thumb.height = canvas.h;
    out.thumb.rgb = std::move(canvas.rgb);
    return out;
}

ClipartRender renderPng(std::span<const std::uint8_t> bytes, int edge, const ClipartLimits& limits,
                        const document::BitmapDecoder& decoder) {
    if (!decoder) {
        return fail("no image decoder is available to read PNG files");
    }
    // Re-checked here so a caller that skipped `checkClipartBytes` still cannot reach the decoder
    // with a hostile header.
    const auto dims = pngDimensions(bytes);
    if (!dims || dims->first == 0 || dims->second == 0 || dims->first > static_cast<std::uint32_t>(limits.maxImageEdge) ||
        dims->second > static_cast<std::uint32_t>(limits.maxImageEdge) ||
        static_cast<std::uint64_t>(dims->first) * dims->second > limits.maxImagePixels) {
        return fail("the PNG header is damaged or its size is over the limit");
    }
    if (const std::string bomb = checkPngPixelData(bytes, limits); !bomb.empty()) {
        return fail(bomb);
    }
    const geometry::ShapeResult<io::RasterImage> decoded = decoder(bytes);
    if (!decoded.ok()) {
        return fail("the image cannot be read: " + decoded.error());
    }
    const io::RasterImage& src = decoded.value();
    if (!src.valid()) {
        return fail("the decoder returned an unusable image");
    }
    const int longest = std::max(src.width, src.height);
    const double scale = longest > edge ? static_cast<double>(edge) / static_cast<double>(longest) : 1.0;
    const int dw = std::max(1, static_cast<int>(std::lround(src.width * scale)));
    const int dh = std::max(1, static_cast<int>(std::lround(src.height * scale)));
    ClipartRender out;
    out.thumb.width = dw;
    out.thumb.height = dh;
    out.thumb.rgb.resize(static_cast<std::size_t>(dw) * static_cast<std::size_t>(dh) * 3U);
    const auto sw = static_cast<std::size_t>(src.width);
    const auto sh = static_cast<std::size_t>(src.height);
    for (int dy = 0; dy < dh; ++dy) {
        const std::size_t y0 = static_cast<std::size_t>(dy) * sh / static_cast<std::size_t>(dh);
        const std::size_t y1 = std::max(y0 + 1, static_cast<std::size_t>(dy + 1) * sh / static_cast<std::size_t>(dh));
        for (int dx = 0; dx < dw; ++dx) {
            const std::size_t x0 = static_cast<std::size_t>(dx) * sw / static_cast<std::size_t>(dw);
            const std::size_t x1 = std::max(x0 + 1, static_cast<std::size_t>(dx + 1) * sw / static_cast<std::size_t>(dw));
            std::size_t sum[3] = {0, 0, 0};
            for (std::size_t y = y0; y < y1; ++y) {
                for (std::size_t x = x0; x < x1; ++x) {
                    const std::size_t at = (y * sw + x) * 3U;
                    sum[0] += src.rgb[at];
                    sum[1] += src.rgb[at + 1];
                    sum[2] += src.rgb[at + 2];
                }
            }
            const std::size_t n = (y1 - y0) * (x1 - x0);
            const std::size_t o = (static_cast<std::size_t>(dy) * static_cast<std::size_t>(dw) + static_cast<std::size_t>(dx)) * 3U;
            for (std::size_t k = 0; k < 3; ++k) {
                out.thumb.rgb[o + k] = static_cast<std::uint8_t>((sum[k] + n / 2) / n);
            }
        }
    }
    out.info = std::to_string(src.width) + " x " + std::to_string(src.height) + " px";
    out.ok = true;
    return out;
}

}  // namespace

ClipartRender renderClipartThumbnail(std::span<const std::uint8_t> bytes, ClipartKind kind, int edge,
                                     const ClipartLimits& limits, const document::BitmapDecoder& decoder) {
    edge = std::clamp(edge, kMinThumbEdge, kMaxThumbEdge);
    try {
        return kind == ClipartKind::Png ? renderPng(bytes, edge, limits, decoder) : renderVector(bytes, kind, edge, limits);
    } catch (const std::exception& e) {
        return fail(std::string("the thumbnail could not be made: ") + e.what());
    }
}

ClipartRender renderClipartThumbnailFile(const std::string& path, int edge, const ClipartLimits& limits,
                                         const document::BitmapDecoder& decoder) {
    const ClipartFile f = readClipartFile(path, limits);
    if (!f.ok) {
        return fail(f.error);
    }
    return renderClipartThumbnail(f.bytes, f.kind, edge, limits, decoder);
}

std::size_t ClipartThumbnailCache::costOf(const ClipartCacheEntry& e) {
    return 64U + e.error.size() + e.info.size() + (e.thumb ? e.thumb->byteSize() : 0U);
}

const ClipartCacheEntry* ClipartThumbnailCache::get(const std::string& key) {
    const auto it = index_.find(key);
    if (it == index_.end()) {
        return nullptr;
    }
    lru_.splice(lru_.begin(), lru_, it->second);
    return &lru_.front().entry;
}

void ClipartThumbnailCache::put(const std::string& key, ClipartCacheEntry entry) {
    const auto it = index_.find(key);
    if (it != index_.end()) {
        bytes_ -= it->second->cost;
        lru_.erase(it->second);
        index_.erase(it);
    }
    const std::size_t cost = costOf(entry);
    if (cost > maxBytes_) {
        return;
    }
    lru_.push_front(Node{key, std::move(entry), cost});
    index_[key] = lru_.begin();
    bytes_ += cost;
    evict();
}

void ClipartThumbnailCache::evict() {
    while (!lru_.empty() && (bytes_ > maxBytes_ || lru_.size() > maxEntries_)) {
        const Node& last = lru_.back();
        bytes_ -= last.cost;
        index_.erase(last.key);
        lru_.pop_back();
        ++evictions_;
    }
}

void ClipartThumbnailCache::clear() {
    lru_.clear();
    index_.clear();
    bytes_ = 0;
}

ClipartThumbnailService::ClipartThumbnailService(Config config, document::BitmapDecoder decoder)
    : config_(std::move(config)),
      decoder_(std::move(decoder)),
      cache_(std::max<std::size_t>(config_.cacheBytes, kMinCacheBytes), std::max<std::size_t>(config_.cacheEntries, 1)),
      worker_([this] { run(); }) {}

ClipartThumbnailService::~ClipartThumbnailService() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
    }
    wake_.notify_all();
    worker_.join();
}

bool ClipartThumbnailService::request(const std::string& path) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_.count(path) != 0 || cache_.contains(path)) {
        return true;
    }
    if (queue_.size() >= config_.maxPending) {
        return false;
    }
    queue_.push_back(path);
    pending_[path] = true;
    wake_.notify_one();
    return true;
}

ClipartThumbState ClipartThumbnailService::state(const std::string& path) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_.count(path) != 0) {
        return ClipartThumbState::Pending;
    }
    const ClipartCacheEntry* e = cache_.get(path);
    if (e == nullptr) {
        return ClipartThumbState::Unknown;
    }
    return e->thumb ? ClipartThumbState::Ready : ClipartThumbState::Failed;
}

ClipartCacheEntry ClipartThumbnailService::lookup(const std::string& path) {
    std::lock_guard<std::mutex> lock(mutex_);
    const ClipartCacheEntry* e = cache_.get(path);
    return e != nullptr ? *e : ClipartCacheEntry{};
}

std::vector<std::string> ClipartThumbnailService::takeCompleted() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> out;
    out.swap(completed_);
    return out;
}

void ClipartThumbnailService::cancelPending() {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
    pending_.clear();  // the running request too: its result is stale now and will be discarded
    ++generation_;
    if (!busy_) {
        idle_.notify_all();
    }
}

void ClipartThumbnailService::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
    pending_.clear();
    cache_.clear();
    completed_.clear();
    ++generation_;
    if (!busy_) {
        idle_.notify_all();
    }
}

void ClipartThumbnailService::waitIdle() {
    std::unique_lock<std::mutex> lock(mutex_);
    idle_.wait(lock, [this] { return queue_.empty() && !busy_; });
}

std::size_t ClipartThumbnailService::processed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return processed_;
}

std::size_t ClipartThumbnailService::cacheBytes() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cache_.bytes();
}

std::size_t ClipartThumbnailService::cacheEvictions() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cache_.evictions();
}

void ClipartThumbnailService::run() {
    std::unique_lock<std::mutex> lock(mutex_);
    for (;;) {
        wake_.wait(lock, [this] { return stop_ || !queue_.empty(); });
        if (stop_) {
            return;
        }
        const std::string path = queue_.front();
        queue_.pop_front();
        busy_ = true;
        const std::size_t gen = generation_;
        lock.unlock();
        ClipartRender r = renderClipartThumbnailFile(path, config_.edge, config_.limits, decoder_);
        lock.lock();
        busy_ = false;
        if (gen == generation_) {
            pending_.erase(path);
            ClipartCacheEntry e;
            if (r.ok) {
                e.thumb = std::make_shared<const ClipartThumbnail>(std::move(r.thumb));
                e.info = std::move(r.info);
            } else {
                e.error = std::move(r.error);
            }
            cache_.put(path, std::move(e));
            // Bounded between takeCompleted calls: beyond the cache's own entry bound the oldest are
            // dropped (their thumbnails are evicted from the cache by then anyway).
            completed_.push_back(path);
            if (completed_.size() > std::max<std::size_t>(1, config_.cacheEntries)) {
                completed_.erase(completed_.begin());
            }
        } else if (std::find(queue_.begin(), queue_.end(), path) == queue_.end()) {
            pending_.erase(path);  // stale result of a cleared generation; a fresh request stays pending
        }
        ++processed_;
        if (queue_.empty()) {
            idle_.notify_all();
        }
    }
}

}  // namespace carvetto::viewmodel
