#pragma once

#include <cstddef>
#include <unordered_set>
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

    bool contains(ObjectId id) const { return set_.contains(id); }
    bool empty() const { return order_.empty(); }
    std::size_t size() const { return order_.size(); }
    /// Selected ids in the order they were selected.
    const std::vector<ObjectId>& ids() const { return order_; }
    /// The single selected id, or null when zero or more than one is selected.
    ObjectId single() const { return order_.size() == 1 ? order_.front() : ObjectId{}; }

    bool operator==(const Selection& o) const { return order_ == o.order_; }

   private:
    std::vector<ObjectId> order_;
    std::unordered_set<ObjectId> set_;
};

}  // namespace carvetto::document
