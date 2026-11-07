#pragma once

// Internal to core (intersect.cpp) and its tests: the bookkeeping of the touch-settling walks
// (T-0178). Both containers are ordered by a key that is unique (value, acceptance sequence), so
// iteration order never depends on how a standard library inserts equal keys.

#include <cstddef>
#include <limits>
#include <optional>
#include <set>
#include <utility>
#include <vector>

namespace carvetto::geometry::detail {

/// Gap between A(t) and B, with B's parameter of the nearest point.
struct GapSample {
    double t;
    double tB;
    double gap;
};

/// Every gap evaluation of a span pair's walks, ordered by t; among equal t the later one comes
/// first (what an insertion at the lower bound of a sorted vector does). A tree instead of a
/// vector: mid-insertions were O(n) each, quadratic over the ~1e5 samples of near-coincident long
/// Beziers.
class GapTrail {
   public:
    void record(const GapSample& g) { samples_.insert(Entry{g, next_++}); }

    /// The samples either side of t: the last one with sample.t < t and the first with
    /// sample.t >= t (the latest recorded among equals); empty when t is outside the recorded range.
    std::optional<std::pair<GapSample, GapSample>> around(double t) const {
        const auto right = samples_.lower_bound(Entry{GapSample{t, 0.0, 0.0}, std::numeric_limits<std::size_t>::max()});
        if (right == samples_.end() || right == samples_.begin()) {
            return std::nullopt;
        }
        return std::pair{std::prev(right)->g, right->g};
    }

    std::size_t size() const { return samples_.size(); }

   private:
    struct Entry {
        GapSample g;
        std::size_t seq;
    };
    struct Before {
        bool operator()(const Entry& l, const Entry& r) const {
            return l.g.t < r.g.t || (l.g.t == r.g.t && l.seq > r.seq);
        }
    };
    std::set<Entry, Before> samples_;
    std::size_t next_{0};
};

/// Minima accepted by earlier walks, in acceptance order, with an index by t for range queries.
class GapMinima {
   public:
    void push_back(const GapSample& g) {
        byT_.insert({g.t, list_.size()});
        list_.push_back(g);
    }

    /// The accepted minimum with l < t < h (both ends excluded) and gap <= maxGap that has the
    /// lowest gap (the earliest accepted among equal gaps), or null.
    const GapSample* lowestIn(double l, double h, double maxGap) const {
        const GapSample* found = nullptr;
        std::size_t foundAt = 0;
        for (auto it = byT_.upper_bound({l, std::numeric_limits<std::size_t>::max()}); it != byT_.end() && it->first < h;
             ++it) {
            const GapSample& k = list_[it->second];
            if (k.gap <= maxGap &&
                (found == nullptr || k.gap < found->gap || (k.gap == found->gap && it->second < foundAt))) {
                found = &k;
                foundAt = it->second;
            }
        }
        return found;
    }

    std::size_t size() const { return list_.size(); }

   private:
    std::vector<GapSample> list_;
    std::set<std::pair<double, std::size_t>> byT_;
};

}  // namespace carvetto::geometry::detail
