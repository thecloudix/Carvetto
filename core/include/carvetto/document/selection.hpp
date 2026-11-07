#pragma once

#include <cstddef>
#include <unordered_map>
#include <vector>

#include "carvetto/document/ids.hpp"

namespace carvetto::document {

/// The set of currently selected objects, kept in selection order so iteration is deterministic.
/// Selection state is not part of the undo history.
class Selection {
   public:
    /// Adds `id`; returns false when it was already selected.
    bool add(ObjectId id);
    /// Removes `id`; returns false when it was not selected.
    bool remove(ObjectId id);
    /// Adds if absent, removes if present. Returns true when the id is now selected.
    bool toggle(ObjectId id);
    /// Replaces the selection (duplicates collapse, first occurrence wins).
    void assign(const std::vector<ObjectId>& ids);
    void clear();

    bool contains(ObjectId id) const { return pos_.contains(id); }
    bool empty() const { return pos_.empty(); }
    std::size_t size() const { return pos_.size(); }
    /// Selected ids in the order they were selected. The reference stays valid until the next
    /// mutation. Not safe to call from several threads at once (it may compact in place).
    const std::vector<ObjectId>& ids() const {
        compact();
        return order_;
    }
    /// The single selected id, or null when zero or more than one is selected.
    ObjectId single() const { return pos_.size() == 1 ? ids().front() : ObjectId{}; }

    bool operator==(const Selection& o) const { return ids() == o.ids(); }

   private:
    // Insertion order is kept in `order_`; `pos_` maps each selected id to its slot. `remove`
    // leaves a null tombstone in the slot (O(1)) and the vector is compacted lazily, when `ids()`
    // is asked for or when tombstones outnumber live entries (amortised O(1) per removal).
    void compact() const;

    mutable std::vector<ObjectId> order_;
    mutable std::unordered_map<ObjectId, std::size_t> pos_;
    mutable std::size_t dead_{0};
};

}  // namespace carvetto::document
