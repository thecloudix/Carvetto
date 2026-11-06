#pragma once

// Internal to core (offset.cpp) and its tests: a set of boxes that grows one box at a time and
// reports, for a query box, every held box that meets it (T-0172, offsetOpenContour step 7).
//
// Unlike a uniform grid (offset.cpp BoxGrid), its memory and query cost do not depend on how the
// boxes are spread: clustered next to one far-away box, nested (the boxes of a spiral's arcs),
// collinear, all huge, all tiny or mixed. Each box is held exactly once, in one of a few static
// trees (the logarithmic method: a box added starts a tree of its own, and trees of equal size are
// rebuilt as one, so there are at most log2(n) + 1 trees and each box is rebuilt at most log2(n)
// times: O(n log^2 n) in all, O(n) memory; the last few boxes added wait in a short list until
// there are kLeaf of them). A tree is a k-d tree of its boxes treated as points (min x, min y,
// max x, max y) in 4-D: median splits along the four coordinates in turn (skipping one along which
// the node's boxes do not differ), down to leaves of at most kLeaf boxes. A box meets a query box
// where its point lies in an orthant of that space, and every node keeps the bounds of its boxes,
// which is the node's bounding box in 4-D, so a query descends only into nodes whose bounds meet
// it; as for any k-d tree, a query visits O(n^(3/4)) nodes plus those of the boxes it reports,
// whatever the boxes: splitting on the corners rather than the centres also separates nested
// boxes (same centre, different sizes) and big boxes from small ones.
//
// A query reports each item whose box intersects the query box (Box2::intersects) exactly once,
// optionally only those with ids in a given range (every node also keeps the range of its ids, so
// a whole node outside it is skipped), in no particular order, so it is only used where the order
// of the reports cannot matter (minima and yes/no answers).

#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

#include "carvetto/geometry/box2.hpp"

namespace carvetto::geometry::detail {

class BoxTree {
   public:
    BoxTree() = default;

    /// Holds `boxes` (ids = positions), built as one tree.
    explicit BoxTree(const std::vector<Box2>& boxes) : boxes_(boxes) {
        std::vector<std::size_t> ids;
        ids.reserve(boxes_.size());
        for (std::size_t i = 0; i < boxes_.size(); ++i) {
            if (holdable(boxes_[i])) {
                ids.push_back(i);
            }
        }
        if (!ids.empty()) {
            trees_.push_back(build(std::move(ids)));
        }
    }

    /// Adds an item; its id is the number of items added before it.
    std::size_t add(const Box2& box) {
        boxes_.push_back(box);
        const std::size_t id = boxes_.size() - 1;
        if (!holdable(box)) {
            return id;  // never reported
        }
        pending_.push_back(id);
        if (pending_.size() < kLeaf) {
            return id;
        }
        std::vector<std::size_t> ids = std::move(pending_);
        pending_.clear();
        while (!trees_.empty() && trees_.back().ids.size() <= ids.size()) {
            const std::vector<std::size_t>& more = trees_.back().ids;
            ids.insert(ids.end(), more.begin(), more.end());
            trees_.pop_back();
        }
        trees_.push_back(build(std::move(ids)));
        return id;
    }

    const Box2& box(std::size_t i) const { return boxes_[i]; }

    /// Calls f(i) once for every item whose box intersects `query`.
    template <class F>
    void query(const Box2& query, F&& f) {
        queryUntil(query, [&](std::size_t i) {
            f(i);
            return false;
        });
    }

    /// Like query(), but stops as soon as f(i) returns true; returns whether it did.
    template <class F>
    bool queryUntil(const Box2& query, F&& f) {
        return queryUntil(query, 0, std::numeric_limits<std::size_t>::max(), f);
    }

    /// Like queryUntil(), for the items with ids in [lo, hi] only (nodes whose ids all lie
    /// outside are skipped whole).
    template <class F>
    bool queryUntil(const Box2& query, std::size_t lo, std::size_t hi, F&& f) {
        if (query.isEmpty() || lo > hi) {
            return false;
        }
        for (const std::size_t id : pending_) {
            ++work_;
            if (id >= lo && id <= hi && boxes_[id].intersects(query) && f(id)) {
                return true;
            }
        }
        std::array<std::size_t, kMaxStack> stack;  // not zeroed: written before read (and zeroing
                                                   // it per query cost ~15% of a hatch's offset)
        for (const Tree& t : trees_) {
            std::size_t top = 0;
            stack[top++] = 0;
            while (top > 0) {
                const Node& node = t.nodes[stack[--top]];
                ++work_;
                if (node.maxId < lo || node.minId > hi || !node.bounds.intersects(query)) {
                    continue;
                }
                if (node.left == kLeafNode) {
                    for (std::size_t k = node.begin; k < node.end; ++k) {
                        ++work_;
                        const std::size_t id = t.ids[k];
                        if (id >= lo && id <= hi && t.boxes[k].intersects(query) && f(id)) {
                            return true;
                        }
                    }
                } else {
                    stack[top++] = node.left + 1;
                    stack[top++] = node.left;
                }
            }
        }
        return false;
    }

    /// For tests: items held (each at most once), tree nodes, and the nodes and boxes looked at by
    /// all queries so far.
    std::size_t heldCount() const {
        std::size_t n = pending_.size();
        for (const Tree& t : trees_) {
            n += t.ids.size();
        }
        return n;
    }
    std::size_t nodeCount() const {
        std::size_t n = 0;
        for (const Tree& t : trees_) {
            n += t.nodes.size();
        }
        return n;
    }
    std::size_t treeCount() const { return trees_.size(); }
    std::size_t work() const { return work_; }

   private:
    static constexpr std::size_t kLeaf = 8;
    static constexpr std::size_t kLeafNode = 0;  // `left` of a leaf (the root is never a child)
    // Median splits: depth <= log2(n) + 1, and a depth-first walk keeps at most one pending
    // sibling per level.
    static constexpr std::size_t kMaxStack = 2 * 64 + 2;

    struct Node {
        Box2 bounds;
        std::size_t begin = 0;  // the node's boxes: positions [begin, end) of its tree
        std::size_t end = 0;
        std::size_t left = kLeafNode;  // children: left and left + 1
        std::size_t minId = 0;  // least and greatest id of the node's items
        std::size_t maxId = 0;
    };

    struct Tree {
        std::vector<Node> nodes;   // nodes[0] is the root
        std::vector<std::size_t> ids;  // item ids in leaf order
        std::vector<Box2> boxes;       // their boxes, in the same order
    };

    // Boxes that no query can meet are not held: empty ones, and those with a NaN coordinate
    // (every comparison in Box2::intersects is false for them).
    static bool holdable(const Box2& b) { return b.min.x <= b.max.x && b.min.y <= b.max.y; }

    static double coord(const Box2& b, int axis) {
        switch (axis) {
            case 0:
                return b.min.x;
            case 1:
                return b.min.y;
            case 2:
                return b.max.x;
            default:
                return b.max.y;
        }
    }

    Tree build(std::vector<std::size_t> ids) const {
        Tree t;
        t.ids = std::move(ids);
        t.nodes.reserve(2 * (t.ids.size() / kLeaf) + 1);
        t.nodes.emplace_back();
        split(t, 0, 0, t.ids.size(), 3);
        t.boxes.reserve(t.ids.size());
        for (const std::size_t id : t.ids) {
            t.boxes.push_back(boxes_[id]);
        }
        return t;
    }

    // Lays out node `ni` over ids [lo, hi) of `t`, recursively (depth <= log2(n) + 1).
    void split(Tree& t, std::size_t ni, std::size_t lo, std::size_t hi, int parentAxis) const {
        std::array<double, 4> least{};
        std::array<double, 4> most{};
        least.fill(std::numeric_limits<double>::infinity());
        most.fill(-std::numeric_limits<double>::infinity());
        Box2 bounds;
        std::size_t minId = std::numeric_limits<std::size_t>::max();
        std::size_t maxId = 0;
        for (std::size_t k = lo; k < hi; ++k) {
            const Box2& b = boxes_[t.ids[k]];
            bounds.expand(b);
            minId = std::min(minId, t.ids[k]);
            maxId = std::max(maxId, t.ids[k]);
            for (std::size_t a = 0; a < 4; ++a) {
                least[a] = std::min(least[a], coord(b, static_cast<int>(a)));
                most[a] = std::max(most[a], coord(b, static_cast<int>(a)));
            }
        }
        t.nodes[ni].bounds = bounds;
        t.nodes[ni].minId = minId;
        t.nodes[ni].maxId = maxId;
        t.nodes[ni].begin = lo;
        t.nodes[ni].end = hi;
        if (hi - lo <= kLeaf) {
            return;
        }
        // The next coordinate after the parent's along which the boxes differ (round robin, so
        // that no coordinate goes unsplit for long however small its spread); all boxes equal:
        // one leaf (a query meets all of them or none).
        int axis = -1;
        for (int k = 1; k <= 4 && axis < 0; ++k) {
            const auto a = static_cast<std::size_t>((parentAxis + k) % 4);
            if (most[a] > least[a]) {
                axis = static_cast<int>(a);
            }
        }
        if (axis < 0) {
            return;
        }
        const std::size_t mid = lo + (hi - lo) / 2;
        const auto first = t.ids.begin();
        std::nth_element(first + static_cast<std::ptrdiff_t>(lo), first + static_cast<std::ptrdiff_t>(mid),
                         first + static_cast<std::ptrdiff_t>(hi), [&](std::size_t i, std::size_t j) {
                             return coord(boxes_[i], axis) < coord(boxes_[j], axis);
                         });
        const std::size_t left = t.nodes.size();
        t.nodes.emplace_back();
        t.nodes.emplace_back();
        t.nodes[ni].left = left;
        split(t, left, lo, mid, axis);
        split(t, left + 1, mid, hi, axis);
    }

    std::vector<Box2> boxes_;
    std::vector<Tree> trees_;  // sizes strictly decreasing
    std::vector<std::size_t> pending_;  // the last items added, fewer than kLeaf, not in a tree yet
    std::size_t work_ = 0;
};

}  // namespace carvetto::geometry::detail
