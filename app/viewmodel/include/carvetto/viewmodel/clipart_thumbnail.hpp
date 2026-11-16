#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <list>
#include <map>
#include <memory>
#include <span>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "carvetto/document/bitmap_store.hpp"
#include "carvetto/viewmodel/clipart_files.hpp"

// Clipart library, part 2 (T-0333, ADR-0109): thumbnails. A software renderer (no Qt) draws SVG/DXF
// outlines and box-filters decoded PNGs; a bounded LRU cache keeps the results; a single worker
// thread produces them off the UI thread. The PNG decoder is injected (the app passes QImage).

namespace carvetto::viewmodel {

/// An 8-bit RGB thumbnail, row 0 at the top.
struct ClipartThumbnail {
    int width{0};
    int height{0};
    std::vector<std::uint8_t> rgb;
    std::size_t byteSize() const { return rgb.size(); }
};

/// What `renderClipartThumbnail` produced.
struct ClipartRender {
    bool ok{false};
    std::string error;
    ClipartThumbnail thumb;
    /// One-line description for a tooltip ("120.0 x 80.0 mm, 14 shapes" / "640 x 480 px").
    std::string info;
    /// Line segments drawn (a work counter; vector files only).
    std::size_t segmentsDrawn{0};
    /// True when the segment budget ran out and the drawing is incomplete.
    bool truncated{false};
};

/// Smallest and largest thumbnail edge honoured.
inline constexpr int kMinThumbEdge = 16;
inline constexpr int kMaxThumbEdge = 512;
/// Most line segments one vector thumbnail draws.
inline constexpr std::size_t kMaxThumbSegments = 400'000;

/// Renders checked clipart bytes (see `checkClipartBytes`) so that the longer side is `edge` pixels
/// (clamped to 16..512). Vectors are drawn dark on light grey with Y up; a PNG is shrunk with a box
/// filter (never enlarged). `decoder` is only used for PNG; without one a PNG fails cleanly.
ClipartRender renderClipartThumbnail(std::span<const std::uint8_t> bytes, ClipartKind kind, int edge,
                                     const ClipartLimits& limits, const document::BitmapDecoder& decoder);

/// Reads `path` within the limits and renders it.
ClipartRender renderClipartThumbnailFile(const std::string& path, int edge, const ClipartLimits& limits,
                                         const document::BitmapDecoder& decoder);

/// A thumbnail or the reason there is none.
struct ClipartCacheEntry {
    std::shared_ptr<const ClipartThumbnail> thumb;  ///< Null for a failure.
    std::string error;                              ///< Set for a failure.
    std::string info;
};

/// LRU cache bounded by total bytes and by entry count. Not thread-safe by itself.
class ClipartThumbnailCache {
   public:
    ClipartThumbnailCache(std::size_t maxBytes, std::size_t maxEntries) : maxBytes_(maxBytes), maxEntries_(maxEntries) {}
    /// The entry for `key` (made most recently used) or null.
    const ClipartCacheEntry* get(const std::string& key);
    /// Stores `entry` (replacing any), then evicts least recently used entries until both bounds
    /// hold. An entry larger than `maxBytes` alone is not stored. A failure costs 64 bytes plus text.
    void put(const std::string& key, ClipartCacheEntry entry);
    void clear();
    bool contains(const std::string& key) const { return index_.count(key) != 0; }
    std::size_t bytes() const { return bytes_; }
    std::size_t size() const { return index_.size(); }
    std::size_t evictions() const { return evictions_; }

   private:
    struct Node {
        std::string key;
        ClipartCacheEntry entry;
        std::size_t cost{0};
    };
    static std::size_t costOf(const ClipartCacheEntry& e);
    void evict();
    std::size_t maxBytes_;
    std::size_t maxEntries_;
    std::size_t bytes_{0};
    std::size_t evictions_{0};
    std::list<Node> lru_;  ///< Front = most recently used.
    std::map<std::string, std::list<Node>::iterator> index_;
};

/// State of a path in the service.
enum class ClipartThumbState { Unknown, Pending, Ready, Failed };

/// Produces thumbnails on one worker thread and holds them in a bounded cache. All members are
/// thread-safe. The UI asks for the visible entries (`request`), draws what `lookup` returns and
/// repaints when `takeCompleted` lists paths.
class ClipartThumbnailService {
   public:
    struct Config {
        int edge{128};
        std::size_t cacheBytes{32U * 1024U * 1024U};
        std::size_t cacheEntries{4096};
        /// Requests waiting for the worker; further requests are refused until it catches up.
        std::size_t maxPending{512};
        ClipartLimits limits;
    };
    ClipartThumbnailService(Config config, document::BitmapDecoder decoder);
    ~ClipartThumbnailService();
    ClipartThumbnailService(const ClipartThumbnailService&) = delete;
    ClipartThumbnailService& operator=(const ClipartThumbnailService&) = delete;

    /// Queues `path` unless it is already pending, cached or failed. False when the queue is full.
    bool request(const std::string& path);
    ClipartThumbState state(const std::string& path) const;
    /// The cached result (thumbnail or error) of `path`, or an empty entry when none.
    ClipartCacheEntry lookup(const std::string& path);
    /// Paths finished since the last call, oldest first. At most `Config::cacheEntries`
    /// are kept between calls (the oldest are dropped beyond that), so a UI that never polls costs
    /// bounded memory.
    std::vector<std::string> takeCompleted();
    /// Drops queued requests that have not started.
    void cancelPending();
    /// Forgets every cached result (after a rescan, so edited files are read again).
    void clear();
    /// Blocks until the queue is empty and the worker is idle (tests, self-test).
    void waitIdle();
    /// Thumbnails rendered or failed by the worker so far (a work counter).
    std::size_t processed() const;
    std::size_t cacheBytes() const;
    std::size_t cacheEvictions() const;
    int edge() const { return config_.edge; }

   private:
    void run();
    Config config_;
    document::BitmapDecoder decoder_;
    mutable std::mutex mutex_;
    std::condition_variable wake_;
    std::condition_variable idle_;
    std::deque<std::string> queue_;
    std::map<std::string, bool> pending_;  ///< Queued or running.
    std::vector<std::string> completed_;
    mutable ClipartThumbnailCache cache_;
    std::size_t processed_{0};
    std::size_t generation_{0};
    bool busy_{false};
    bool stop_{false};
    std::thread worker_;
};

}  // namespace carvetto::viewmodel
