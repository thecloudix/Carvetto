#pragma once

// Internal to core (offset.cpp) and its tests: a set of boxes that grows one box at a time and
// reports, for a query box, every held box that meets it (T-0172 round 3, offsetOpenContour
// step 7's cover queries).
//
// A uniform grid (as offset.cpp's BoxGrid) answers a small query in O(1) where the boxes are
// spread evenly, whatever their shapes: a hatch whose lines start and end anywhere along x costs
// a k-d tree (BoxTree) ~3.4 sqrt(n) per query, as boxes that overlap along x and differ only in y
// are its weak case. But a grid degrades where boxes crowd a cell (a hatch next to one far-away
// feature, all in one cell) or one box covers many cells (the nested arcs of a spiral). So this
// is both: a box goes into the grid when it covers at most kMaxCellsPerBox cells and each of them
// holds fewer than kMaxPerCell boxes when it is added; any other box goes into a BoxTree. A grid
// query thus looks at no more than kMaxPerCell boxes per cell, and the grid holds at most
// kMaxCellsPerBox entries per box and kMaxPerCell per cell, whatever the boxes (bounded memory;
// round 1's clustering grid took gigabytes on nested boxes); what the grid does not take costs
// what it costs in a BoxTree. (32 per cell: an offset's chains and strays at pitch 2d fill a cell
// with ~10, while a hatch next to a far feature puts thousands into one; 256 cells: the lines of
// a hatch cover ~sqrt(length / pitch) cells each, 50 at 10 mm over 0.004 mm.)
//
// A query reports each item whose box intersects the query box (Box2::intersects) exactly once,
// in no particular order, so it is only used where the order of the reports cannot matter
// (minima and yes/no answers); which part holds a box changes the cost of a query, never its
// answer.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "box_tree.hpp"
#include "carvetto/geometry/box2.hpp"

namespace carvetto::geometry::detail {

class BoxIndex {
   public:
    /// Most boxes a grid cell takes, and most cells a box in the grid covers.
    static constexpr std::size_t kMaxPerCell = 32;
    static constexpr std::size_t kMaxCellsPerBox = 256;

    /// An empty index whose grid has about as many cells as `count` boxes over `layout`: square
    /// cells, at most max(1024, count) per side (BoxGrid's 1024 made cells of a long, narrow
    /// layout ~16 hatch lines tall; this way there are still at most ~3 count + 1 cells). Boxes
    /// outside `layout` are still held and found.
    BoxIndex(const Box2& layout, std::size_t count) : extent_(layout) {
        if (extent_.isEmpty() || !std::isfinite(extent_.width()) || !std::isfinite(extent_.height())) {
            extent_ = Box2{Vec2{}, Vec2{}};
        }
        const double w = extent_.width();
        const double h = extent_.height();
        const double n = static_cast<double>(std::max<std::size_t>(count, 1));
        const double maxSide = std::max(kMaxCells, n);
        double cell = std::sqrt(std::max(w * h, 1e-300) / n);
        cell = std::max({cell, w / maxSide, h / maxSide, 1e-9});
        nx_ = static_cast<std::size_t>(std::clamp(std::ceil(w / cell), 1.0, maxSide));
        ny_ = static_cast<std::size_t>(std::clamp(std::ceil(h / cell), 1.0, maxSide));
        // Cells per mm: a box and a query find their cells by the same monotone map, so the
        // cells of boxes that meet overlap (any map will do along a side of one cell; at most 1e9
        // cells per mm, finite: a side of more than one cell is at least 1e-9 mm per cell).
        perX_ = nx_ > 1 ? static_cast<double>(nx_) / w : 1.0;
        perY_ = ny_ > 1 ? static_cast<double>(ny_) / h : 1.0;
        cells_.resize(nx_ * ny_);
    }

    /// Holds `boxes` (ids = positions), laid out over them.
    explicit BoxIndex(const std::vector<Box2>& boxes) : BoxIndex(extentOf(boxes), boxes.size()) {
        for (const Box2& b : boxes) {
            add(b);
        }
    }

    /// Adds an item; its id is the number of items added before it.
    std::size_t add(const Box2& box) {
        const std::size_t id = boxes_.size();
        boxes_.push_back(box);
        Range r{};
        if (fitsGrid(box, r)) {
            for (std::size_t y = r.y0; y <= r.y1; ++y) {
                for (std::size_t x = r.x0; x <= r.x1; ++x) {
                    cells_[y * nx_ + x].push_back(Entry{box, r.x0, r.y0, id});
                }
            }
            filled_.expand(box);
        } else {
            tree_.add(box);
            treeIds_.push_back(id);
            if (box.min.x <= box.max.x && box.min.y <= box.max.y) {
                inTree_.expand(box);
            }
        }
        return id;
    }

    const Box2& box(std::size_t i) const { return boxes_[i]; }

    /// Calls f(i) once for every item whose box intersects `q`.
    template <class F>
    void query(const Box2& q, F&& f) {
        queryGrid(q, f);
        queryTree(q, f);
    }

    /// Like query(), for the items in the grid only.
    template <class F>
    void queryGrid(const Box2& q, F&& f) {
        if (!q.intersects(filled_)) {
            return;
        }
        // A box filed in several cells is reported from the first cell (least x, then least y)
        // of those it shares with the query: the cell ranges of boxes that meet overlap.
        const Range r = cellRange(q);
        for (std::size_t y = r.y0; y <= r.y1; ++y) {
            for (std::size_t x = r.x0; x <= r.x1; ++x) {
                const std::vector<Entry>& cell = cells_[y * nx_ + x];
                work_ += 1 + cell.size();
                for (const Entry& e : cell) {
                    if (e.box.intersects(q) && x == std::max(r.x0, e.x0) && y == std::max(r.y0, e.y0)) {
                        f(e.id);
                    }
                }
            }
        }
    }

    /// Like query(), for the items in the tree only.
    template <class F>
    void queryTree(const Box2& q, F&& f) {
        if (q.intersects(inTree_)) {
            tree_.query(q, [&](std::size_t i) { f(treeIds_[i]); });
        }
    }

    /// Items in the tree.
    std::size_t treeCount() const { return treeIds_.size(); }

    /// For tests: items in the grid, entries filed in its cells, the most items in one cell, and
    /// the cells and boxes looked at by all grid queries so far plus the tree's work.
    std::size_t gridCount() const { return boxes_.size() - treeIds_.size(); }
    std::size_t entryCount() const {
        std::size_t n = 0;
        for (const std::vector<Entry>& c : cells_) {
            n += c.size();
        }
        return n;
    }
    std::size_t fullestCell() const {
        std::size_t n = 0;
        for (const std::vector<Entry>& c : cells_) {
            n = std::max(n, c.size());
        }
        return n;
    }
    std::size_t cellCount() const { return cells_.size(); }
    std::size_t work() const { return work_ + tree_.work(); }

   private:
    static constexpr double kMaxCells = 1024.0;

    struct Range {
        std::size_t x0, x1, y0, y1;
    };

    // A box in a cell, with the first cell it is filed in (each box is reported from one cell).
    struct Entry {
        Box2 box;
        std::size_t x0;
        std::size_t y0;
        std::size_t id;
    };

    static Box2 extentOf(const std::vector<Box2>& boxes) {
        Box2 e;
        for (const Box2& b : boxes) {
            e.expand(b);
        }
        return e;
    }

    static std::size_t index(double v, double origin, double perMm, std::size_t count) {
        const double k = std::floor((v - origin) * perMm);
        return static_cast<std::size_t>(std::clamp(k, 0.0, static_cast<double>(count - 1)));
    }

    Range cellRange(const Box2& b) const {
        return {index(b.min.x, extent_.min.x, perX_, nx_), index(b.max.x, extent_.min.x, perX_, nx_),
                index(b.min.y, extent_.min.y, perY_, ny_), index(b.max.y, extent_.min.y, perY_, ny_)};
    }

    // Whether `b` goes into the grid: finite and not empty (a NaN or infinite coordinate goes to
    // the tree, which skips boxes no query meets), covering at most kMaxCellsPerBox cells, none
    // of them full; `r` is set to its cells.
    bool fitsGrid(const Box2& b, Range& r) const {
        if (!(b.min.x <= b.max.x && b.min.y <= b.max.y) || !std::isfinite(b.min.x) ||
            !std::isfinite(b.min.y) || !std::isfinite(b.max.x) || !std::isfinite(b.max.y)) {
            return false;
        }
        r = cellRange(b);
        const std::size_t cols = r.x1 - r.x0 + 1;
        if (cols > kMaxCellsPerBox || r.y1 - r.y0 + 1 > kMaxCellsPerBox / cols) {
            return false;
        }
        for (std::size_t y = r.y0; y <= r.y1; ++y) {
            for (std::size_t x = r.x0; x <= r.x1; ++x) {
                if (cells_[y * nx_ + x].size() >= kMaxPerCell) {
                    return false;
                }
            }
        }
        return true;
    }

    Box2 extent_;  // the cell layout
    Box2 filled_;  // the boxes in the grid
    Box2 inTree_;  // the boxes in the tree that a query can meet
    std::size_t nx_{0};
    std::size_t ny_{0};
    double perX_{1.0};
    double perY_{1.0};
    std::vector<std::vector<Entry>> cells_;
    std::vector<Box2> boxes_;
    BoxTree tree_;  // the items not in the grid
    std::vector<std::size_t> treeIds_;  // their ids, by tree id
    std::size_t work_{0};
};

}  // namespace carvetto::geometry::detail
