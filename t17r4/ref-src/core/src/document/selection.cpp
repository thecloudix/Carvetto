#include "carvetto/document/selection.hpp"

#include <algorithm>

namespace carvetto::document {

bool Selection::add(ObjectId id) {
    if (id.isNull() || !set_.insert(id).second) {
        return false;
    }
    order_.push_back(id);
    return true;
}

bool Selection::remove(ObjectId id) {
    if (set_.erase(id) == 0) {
        return false;
    }
    order_.erase(std::find(order_.begin(), order_.end(), id));
    return true;
}

bool Selection::toggle(ObjectId id) {
    if (contains(id)) {
        remove(id);
        return false;
    }
    return add(id);
}

void Selection::assign(const std::vector<ObjectId>& ids) {
    clear();
    for (ObjectId id : ids) {
        add(id);
    }
}

void Selection::clear() {
    order_.clear();
    set_.clear();
}

}  // namespace carvetto::document
