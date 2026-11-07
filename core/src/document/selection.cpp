#include "carvetto/document/selection.hpp"

namespace carvetto::document {

bool Selection::add(ObjectId id) {
    if (id.isNull() || pos_.contains(id)) {
        return false;
    }
    pos_.emplace(id, order_.size());
    order_.push_back(id);
    return true;
}

bool Selection::remove(ObjectId id) {
    const auto it = pos_.find(id);
    if (it == pos_.end()) {
        return false;
    }
    order_[it->second] = ObjectId{};
    pos_.erase(it);
    ++dead_;
    while (!order_.empty() && order_.back().isNull()) {
        order_.pop_back();
        --dead_;
    }
    if (dead_ > 16 && dead_ > pos_.size()) {
        compact();
    }
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
    pos_.clear();
    dead_ = 0;
}

void Selection::compact() const {
    if (dead_ == 0) {
        return;
    }
    std::size_t out = 0;
    for (std::size_t i = 0; i < order_.size(); ++i) {
        if (!order_[i].isNull()) {
            order_[out] = order_[i];
            pos_[order_[out]] = out;
            ++out;
        }
    }
    order_.resize(out);
    dead_ = 0;
}

}  // namespace carvetto::document
