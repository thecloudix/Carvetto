#pragma once

// Internal to core (intersect.cpp) and its tests: first-match lookup of points within a tolerance.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/vec2.hpp"

namespace carvetto::geometry::detail {

// Finds the first (lowest index) stored point within `tol.distance` of a query, as a linear scan
// over insertion order would, but through a hash grid of cell 2 * tol.distance (T-0178: merging n
// hits was O(n^2), tens of seconds for the ~1e5 hits of two near-coincident long Beziers). Points
// whose cell index would not fit (non-finite, absurd extent/tolerance ratio) go to a list scanned
// linearly, so the answer is always the linear scan's.
class HitIndex {
   public:
    static constexpr std::size_t npos = static_cast<std::size_t>(-1);

    explicit HitIndex(const Tolerance& tol) : tol_(tol), cell_(2.0 * tol.distance) {}

    void add(std::size_t index, Vec2 p) {
        std::int64_t cx = 0;
        std::int64_t cy = 0;
        if (cellOf(p, cx, cy)) {
            cells_[key(cx, cy)].push_back(index);
        } else {
            loose_.push_back(index);
        }
    }

    /// Lowest stored index i with tol.equalPoints(pointOf(i), p), or npos.
    template <class PointOf>
    std::size_t first(Vec2 p, PointOf&& pointOf) const {
        std::size_t best = npos;
        const auto test = [&](std::size_t i) {
            if (i < best && tol_.equalPoints(pointOf(i), p)) {
                best = i;
            }
        };
        std::int64_t cx = 0;
        std::int64_t cy = 0;
        if (cellOf(p, cx, cy)) {
            for (std::int64_t dx = -1; dx <= 1; ++dx) {
                for (std::int64_t dy = -1; dy <= 1; ++dy) {
                    const auto it = cells_.find(key(cx + dx, cy + dy));
                    if (it != cells_.end()) {
                        for (const std::size_t i : it->second) {
                            test(i);
                        }
                    }
                }
            }
        } else {
            for (const auto& cell : cells_) {
                for (const std::size_t i : cell.second) {
                    test(i);
                }
            }
        }
        for (const std::size_t i : loose_) {
            test(i);
        }
        return best;
    }

   private:
    bool cellOf(Vec2 p, std::int64_t& cx, std::int64_t& cy) const {
        constexpr double kLimit = 1e15;
        if (!(cell_ > 0.0)) {
            return false;
        }
        const double fx = std::floor(p.x / cell_);
        const double fy = std::floor(p.y / cell_);
        if (!(std::fabs(fx) < kLimit) || !(std::fabs(fy) < kLimit)) {
            return false;
        }
        cx = static_cast<std::int64_t>(fx);
        cy = static_cast<std::int64_t>(fy);
        return true;
    }
    static std::uint64_t key(std::int64_t cx, std::int64_t cy) {
        // A key collision only merges two cells' candidate lists; equalPoints decides.
        return static_cast<std::uint64_t>(cx) * 0x9E3779B97F4A7C15ULL ^
               (static_cast<std::uint64_t>(cy) + 0x7F4A7C15ULL) * 0xC2B2AE3D27D4EB4FULL;
    }

    Tolerance tol_;
    double cell_;
    std::unordered_map<std::uint64_t, std::vector<std::size_t>> cells_;
    std::vector<std::size_t> loose_;
};

}  // namespace carvetto::geometry::detail
