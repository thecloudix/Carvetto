#include "carvetto/toolpath/box_index.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace carvetto::toolpath {

namespace {

constexpr double kMaxCellsPerAxis = 2048.0;
constexpr std::size_t kMaxCellsPerBox = 64;

}  // namespace

BoxIndex::BoxIndex(std::vector<geometry::Box2> boxes) : boxes_(std::move(boxes)) {
    std::vector<double> sizes;
    sizes.reserve(boxes_.size());
    for (const geometry::Box2& b : boxes_) {
        if (!b.isEmpty() && std::isfinite(b.width()) && std::isfinite(b.height())) {
            all_.expand(b);
            sizes.push_back(std::max(b.width(), b.height()));
        }
    }
    if (sizes.empty()) {
        return;
    }
    const auto mid = sizes.begin() + static_cast<std::ptrdiff_t>(sizes.size() / 2);
    std::nth_element(sizes.begin(), mid, sizes.end());
    const double extent = std::max(all_.width(), all_.height());
    const double byCount = std::sqrt(all_.width() * all_.height() / (4.0 * static_cast<double>(sizes.size())));
    cell_ = std::max({*mid, byCount, extent / kMaxCellsPerAxis, 1e-9});
    nx_ = static_cast<std::size_t>(all_.width() / cell_) + 1;
    ny_ = static_cast<std::size_t>(all_.height() / cell_) + 1;
    start_.assign(nx_ * ny_ + 1, 0);
    for (int pass = 0; pass < 2; ++pass) {
        std::vector<std::size_t> fill;
        if (pass == 1) {
            for (std::size_t c = 1; c < start_.size(); ++c) {
                start_[c] += start_[c - 1];
            }
            items_.resize(start_.back());
            fill.assign(start_.begin(), start_.end() - 1);
        }
        for (std::size_t i = 0; i < boxes_.size(); ++i) {
            const geometry::Box2& b = boxes_[i];
            if (b.isEmpty()) {
                continue;
            }
            if (!std::isfinite(b.min.x) || !std::isfinite(b.min.y) || !std::isfinite(b.max.x) ||
                !std::isfinite(b.max.y)) {
                if (pass == 0) {
                    big_.push_back(i);
                }
                continue;
            }
            const std::size_t i0 = col(b.min.x), i1 = col(b.max.x);
            const std::size_t j0 = row(b.min.y), j1 = row(b.max.y);
            if ((i1 - i0 + 1) * (j1 - j0 + 1) > kMaxCellsPerBox) {
                if (pass == 0) {
                    big_.push_back(i);
                }
                continue;
            }
            for (std::size_t j = j0; j <= j1; ++j) {
                for (std::size_t k = i0; k <= i1; ++k) {
                    const std::size_t c = j * nx_ + k;
                    if (pass == 0) {
                        ++start_[c + 1];
                    } else {
                        items_[fill[c]++] = i;
                    }
                }
            }
        }
    }
}

std::size_t BoxIndex::col(double x) const {
    const double f = std::floor((x - all_.min.x) / cell_);
    return static_cast<std::size_t>(std::clamp(f, 0.0, static_cast<double>(nx_ - 1)));
}

std::size_t BoxIndex::row(double y) const {
    const double f = std::floor((y - all_.min.y) / cell_);
    return static_cast<std::size_t>(std::clamp(f, 0.0, static_cast<double>(ny_ - 1)));
}

std::vector<std::size_t> BoxIndex::query(const geometry::Box2& q) const {
    std::vector<std::size_t> out;
    for (std::size_t i : big_) {
        if (boxes_[i].intersects(q)) {
            out.push_back(i);
        }
    }
    if (nx_ == 0 || !q.intersects(all_)) {
        return out;
    }
    const std::size_t i0 = col(q.min.x), i1 = col(q.max.x);
    const std::size_t j0 = row(q.min.y), j1 = row(q.max.y);
    for (std::size_t j = j0; j <= j1; ++j) {
        for (std::size_t k = i0; k <= i1; ++k) {
            const std::size_t c = j * nx_ + k;
            for (std::size_t s = start_[c]; s < start_[c + 1]; ++s) {
                if (boxes_[items_[s]].intersects(q)) {
                    out.push_back(items_[s]);
                }
            }
        }
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

}  // namespace carvetto::toolpath
