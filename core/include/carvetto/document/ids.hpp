#pragma once

#include <compare>
#include <cstddef>
#include <cstdint>
#include <functional>

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
class IdGenerator {
   public:
    /// Returns the next unused value and advances.
    std::uint64_t allocate() { return next_++; }
    /// Value the next `allocate()` would return.
    std::uint64_t peek() const { return next_; }
    /// Ensures future ids are greater than `value` (used when loading a document).
    void reserveThrough(std::uint64_t value) {
        if (value >= next_) {
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
