#pragma once

#include <cassert>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>

namespace carvetto::document {

/// Strongly typed 64-bit identifier. `Tag` keeps object, layer and sheet ids distinct at compile
/// time. The value 0 is the null id; real ids start at 1 and are never reused within a document.
template <class Tag>
struct Id {
    std::uint64_t value{0};

    constexpr Id() = default;
    constexpr explicit Id(std::uint64_t v) : value(v) {}

    constexpr bool isNull() const { return value == 0; }
    constexpr explicit operator bool() const { return value != 0; }
    constexpr auto operator<=>(const Id&) const = default;
};

struct ObjectTag {};
struct LayerTag {};
struct SheetTag {};

/// Identifier of a drawing object (vector, bitmap or group).
using ObjectId = Id<ObjectTag>;
/// Identifier of a layer.
using LayerId = Id<LayerTag>;
/// Identifier of a sheet.
using SheetId = Id<SheetTag>;

/// Monotonic id allocator shared by every id kind in a document. Ids are never handed out twice,
/// so an object removed by undo and re-created by redo keeps its identity while a pasted copy
/// always receives a fresh one.
///
/// Valid ids lie in `[1, kMaxId)`. The generator never wraps: once `peek() == kMaxId` it is
/// *exhausted* and hands out nothing more (ADR-0007, amendment A1). Callers that can fail
/// gracefully check `canAllocate(n)` first (every command does, and becomes a no-op when the
/// check fails); `allocate()` on an exhausted generator is a programmer error.
class IdGenerator {
   public:
    /// Ceiling on id values: every id in a document is strictly below 2^62. File I/O (T-0012) is to
    /// use it to reject ids at or above it on load. Leaves ample headroom below 2^64 so `value + 1`
    /// and small sums never overflow.
    static constexpr std::uint64_t kMaxId = std::uint64_t{1} << 62;

    /// Returns the next unused value and advances. Asserts (Debug) when exhausted; in Release it
    /// returns 0 (the null id) without advancing, so it never wraps or reuses a value.
    std::uint64_t allocate() {
        assert(next_ < kMaxId && "IdGenerator exhausted: no id below kMaxId is left");
        if (next_ >= kMaxId) {
            return 0;
        }
        return next_++;
    }
    /// Returns the next unused value and advances, or nullopt (without advancing) when exhausted.
    std::optional<std::uint64_t> tryAllocate() {
        if (next_ >= kMaxId) {
            return std::nullopt;
        }
        return next_++;
    }
    /// True when `count` more ids can be allocated before reaching `kMaxId`.
    bool canAllocate(std::uint64_t count = 1) const { return count <= remaining(); }
    /// Number of ids still available.
    std::uint64_t remaining() const { return next_ < kMaxId ? kMaxId - next_ : 0; }
    /// True when no further id can be allocated.
    bool exhausted() const { return next_ >= kMaxId; }
    /// Value the next `allocate()` would return (`kMaxId` once exhausted).
    std::uint64_t peek() const { return next_; }
    /// Ensures future ids are greater than `value` (used when loading a document). A `value` at or
    /// above `kMaxId - 1` leaves the generator exhausted (never wraps); such ids are out of range
    /// and reported by `Document::checkIntegrity()`.
    void reserveThrough(std::uint64_t value) {
        if (value >= kMaxId - 1) {
            next_ = kMaxId;
        } else if (value >= next_) {
            next_ = value + 1;
        }
    }

   private:
    std::uint64_t next_{1};
};

}  // namespace carvetto::document

namespace std {
/// Hashes an id by its value so ids can key unordered containers.
template <class Tag>
struct hash<carvetto::document::Id<Tag>> {
    std::size_t operator()(const carvetto::document::Id<Tag>& id) const noexcept {
        return std::hash<std::uint64_t>{}(id.value);
    }
};
}  // namespace std
