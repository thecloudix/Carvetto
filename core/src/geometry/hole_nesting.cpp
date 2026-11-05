#include "hole_nesting.hpp"

#include <clipper2/clipper.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace carvetto::geometry::detail {

RingSide clipperPointInRing(GridPoint p, const GridRing& ring) {
    Clipper2Lib::Path64 path;
    path.reserve(ring.size());
    for (const GridPoint& q : ring) {
        path.emplace_back(q.x, q.y);
    }
    switch (Clipper2Lib::PointInPolygon(Clipper2Lib::Point64(p.x, p.y), path)) {
        case Clipper2Lib::PointInPolygonResult::IsInside:
            return RingSide::Inside;
        case Clipper2Lib::PointInPolygonResult::IsOn:
            return RingSide::On;
        case Clipper2Lib::PointInPolygonResult::IsOutside:
            break;
    }
    return RingSide::Outside;
}

namespace {

// Clipper2's CrossProduct(pt1, pt2, pt3), the same expression.
template <class P>
double crossProduct(const P& a, const P& b, std::int64_t px, std::int64_t py) {
    return static_cast<double>(b.x - a.x) * static_cast<double>(py - b.y) -
           static_cast<double>(b.y - a.y) * static_cast<double>(px - b.x);
}

}  // namespace

// Clipper2's PointInPolygon walks the ring from the first vertex not on the line y = py
// ("first"), around to it again. Runs of vertices strictly on the side it is on are skipped; a
// vertex on the line is checked for lying on the point or on a horizontal edge at the line (IsOn);
// a vertex on the other side closes an edge across the line, which toggles the parity when it
// passes left of the point (IsOn when the point is on it), and switches the side. The edge back
// into "first" is closed after the walk if the side differs from the start.
//
// Every vertex the walk does not skip ends an edge whose y extent holds py (its predecessor is on
// the current side or on the line, it is on the line or on the other side), and a vertex ending
// such an edge that is on the current side is skipped by the walk as well. So visiting only the
// end vertices of the edges that reach py, in the walk's cyclic order, gives the same result
// with the same arithmetic.
template <class Ring>
RingSide RingLocator<Ring>::locate(std::int64_t px, std::int64_t py) {
    const Ring& r = *ring_;
    const std::size_t n = r.size();
    if (n < 3) {
        return RingSide::Outside;
    }
    ++tests_;
    if (leaves_ == 0 && tests_ >= 2 && n >= kRingIndexMinVertices) {
        buildIndex();
    }
    std::size_t first = 0;
    while (first < n && r[first].y == py) {
        ++first;
    }
    if (first == n) {
        return RingSide::Outside;  // not a proper polygon
    }
    const bool startingAbove = r[first].y < py;
    bool isAbove = startingAbove;
    int val = 0;
    // Returns true when the point is on the ring.
    auto visit = [&](std::size_t k) {
        const auto& c = r[k];
        const auto& prev = r[k == 0 ? n - 1 : k - 1];
        if (c.y == py) {
            return c.x == px || (c.y == prev.y && ((px < prev.x) != (px < c.x)));
        }
        if ((c.y < py) == isAbove) {
            return false;  // still on the same side
        }
        if (px < c.x && px < prev.x) {
            // the edge crosses right of the point
        } else if (px > prev.x && px > c.x) {
            val = 1 - val;
        } else {
            const double d = crossProduct(prev, c, px, py);
            if (d == 0) {
                return true;
            }
            if ((d < 0) == isAbove) {
                val = 1 - val;
            }
        }
        isAbove = !isAbove;
        return false;
    };
    if (leaves_ == 0) {
        for (std::size_t k = first + 1; k < n; ++k) {
            if (visit(k)) {
                return RingSide::On;
            }
        }
        for (std::size_t k = 0; k < first; ++k) {
            if (visit(k)) {
                return RingSide::On;
            }
        }
    } else {
        events_.clear();
        stabbed(py, events_);
        const std::size_t shift = n - first - 1;  // first + 1 -> 0, first -> n - 1
        for (std::size_t& k : events_) {
            k = (k + shift) % n;
        }
        std::sort(events_.begin(), events_.end());
        for (const std::size_t key : events_) {
            if (key == n - 1) {
                break;  // the edge into "first" is closed below
            }
            if (visit((key + first + 1) % n)) {
                return RingSide::On;
            }
        }
    }
    if (isAbove != startingAbove) {
        const auto& c = r[first];
        const auto& prev = r[first == 0 ? n - 1 : first - 1];
        const double d = crossProduct(prev, c, px, py);
        if (d == 0) {
            return RingSide::On;
        }
        if ((d < 0) == isAbove) {
            val = 1 - val;
        }
    }
    return val == 0 ? RingSide::Outside : RingSide::Inside;
}

template <class Ring>
void RingLocator<Ring>::buildIndex() {
    const Ring& r = *ring_;
    const std::size_t n = r.size();
    std::vector<std::int64_t> lo(n);
    std::vector<std::int64_t> hi(n);
    for (std::size_t k = 0; k < n; ++k) {
        const std::int64_t a = r[k == 0 ? n - 1 : k - 1].y;
        const std::int64_t b = r[k].y;
        lo[k] = std::min(a, b);
        hi[k] = std::max(a, b);
    }
    edge_.resize(n);
    for (std::size_t k = 0; k < n; ++k) {
        edge_[k] = k;
    }
    std::sort(edge_.begin(), edge_.end(),
              [&lo](std::size_t a, std::size_t b) { return lo[a] < lo[b] || (lo[a] == lo[b] && a < b); });
    lo_.resize(n);
    leaves_ = 1;
    while (leaves_ < n) {
        leaves_ *= 2;
    }
    maxHi_.assign(2 * leaves_, std::numeric_limits<std::int64_t>::min());
    for (std::size_t i = 0; i < n; ++i) {
        lo_[i] = lo[edge_[i]];
        maxHi_[leaves_ + i] = hi[edge_[i]];
    }
    for (std::size_t i = leaves_ - 1; i >= 1; --i) {
        maxHi_[i] = std::max(maxHi_[2 * i], maxHi_[2 * i + 1]);
    }
}

template <class Ring>
void RingLocator<Ring>::stabbed(std::int64_t py, std::vector<std::size_t>& out) const {
    // Edges with lo <= py form a prefix of the sorted order; of those, the ones with hi >= py.
    const auto count = static_cast<std::size_t>(std::upper_bound(lo_.begin(), lo_.end(), py) - lo_.begin());
    if (count == 0) {
        return;
    }
    struct Node {
        std::size_t node;
        std::size_t begin;
        std::size_t width;
    };
    std::vector<Node> stack{{1, 0, leaves_}};
    while (!stack.empty()) {
        const Node nd = stack.back();
        stack.pop_back();
        if (nd.begin >= count || maxHi_[nd.node] < py) {
            continue;
        }
        if (nd.width == 1) {
            out.push_back(edge_[nd.begin]);
            continue;
        }
        const std::size_t half = nd.width / 2;
        stack.push_back({2 * nd.node + 1, nd.begin + half, half});
        stack.push_back({2 * nd.node, nd.begin, half});
    }
}

namespace {

struct Box {
    std::int64_t x0{0};
    std::int64_t y0{0};
    std::int64_t x1{0};
    std::int64_t y1{0};
    bool holds(const Box& b) const { return x0 <= b.x0 && x1 >= b.x1 && y0 <= b.y0 && y1 >= b.y1; }
};

template <class Ring>
Box boxOf(const Ring& r) {
    Box b{std::numeric_limits<std::int64_t>::max(), std::numeric_limits<std::int64_t>::max(),
          std::numeric_limits<std::int64_t>::min(), std::numeric_limits<std::int64_t>::min()};
    for (const auto& p : r) {
        b.x0 = std::min(b.x0, static_cast<std::int64_t>(p.x));
        b.y0 = std::min(b.y0, static_cast<std::int64_t>(p.y));
        b.x1 = std::max(b.x1, static_cast<std::int64_t>(p.x));
        b.y1 = std::max(b.y1, static_cast<std::int64_t>(p.y));
    }
    return b;
}

double areaOf(const Box& b) { return static_cast<double>(b.x1 - b.x0) * static_cast<double>(b.y1 - b.y0); }

// Bounding-volume hierarchy over the boxes of the rings wound like the lowest. A node's box holds
// every box below it, so a node whose box does not hold the hole's box has no candidate below it.
class BoxTree {
public:
    BoxTree(const std::vector<Box>& boxes, std::vector<std::size_t> items) : boxes_(boxes), items_(std::move(items)) {
        if (!items_.empty()) {
            build(0, items_.size());
        }
    }

    // Calls `test(i)` for the items whose box holds `q`, smallest box area first (ties by item
    // index), until it returns true; returns whether it did.
    template <class Test>
    bool anyHolding(const Box& q, Test&& test) {
        if (nodes_.empty()) {
            return false;
        }
        heap_.clear();
        pushNode(0, q);
        while (!heap_.empty()) {
            std::pop_heap(heap_.begin(), heap_.end());
            const Entry e = heap_.back();
            heap_.pop_back();
            if (e.isItem) {
                if (test(e.id)) {
                    return true;
                }
                continue;
            }
            const Node& nd = nodes_[e.id];
            if (nd.leaf) {
                for (std::size_t i = nd.a; i < nd.b; ++i) {
                    const std::size_t item = items_[i];
                    if (boxes_[item].holds(q)) {
                        push({areaOf(boxes_[item]), item, true});
                    }
                }
            } else {
                pushNode(nd.a, q);
                pushNode(nd.b, q);
            }
        }
        return false;
    }

private:
    struct Node {
        Box box;
        double minArea{0.0};
        std::size_t a{0};  // leaf: item range [a, b); inner: children a, b
        std::size_t b{0};
        bool leaf{false};
    };
    struct Entry {
        double key;
        std::size_t id;
        bool isItem;
        // The heap keeps the largest on top: inverted for smallest key, then nodes first, then id.
        bool operator<(const Entry& o) const {
            if (key != o.key) {
                return key > o.key;
            }
            if (isItem != o.isItem) {
                return isItem;
            }
            return id > o.id;
        }
    };
    static constexpr std::size_t kLeafSize = 4;

    void push(const Entry& e) {
        heap_.push_back(e);
        std::push_heap(heap_.begin(), heap_.end());
    }

    void pushNode(std::size_t id, const Box& q) {
        if (nodes_[id].box.holds(q)) {
            push({nodes_[id].minArea, id, false});
        }
    }

    std::size_t build(std::size_t begin, std::size_t end) {
        const std::size_t id = nodes_.size();
        nodes_.push_back({});
        Box box = boxes_[items_[begin]];
        double minArea = areaOf(box);
        // Extent of the box centres (doubled, to stay in integers).
        std::int64_t cx0 = std::numeric_limits<std::int64_t>::max();
        std::int64_t cy0 = cx0;
        std::int64_t cx1 = std::numeric_limits<std::int64_t>::min();
        std::int64_t cy1 = cx1;
        for (std::size_t i = begin; i < end; ++i) {
            const Box& b = boxes_[items_[i]];
            box.x0 = std::min(box.x0, b.x0);
            box.y0 = std::min(box.y0, b.y0);
            box.x1 = std::max(box.x1, b.x1);
            box.y1 = std::max(box.y1, b.y1);
            minArea = std::min(minArea, areaOf(b));
            cx0 = std::min(cx0, b.x0 / 2 + b.x1 / 2);
            cx1 = std::max(cx1, b.x0 / 2 + b.x1 / 2);
            cy0 = std::min(cy0, b.y0 / 2 + b.y1 / 2);
            cy1 = std::max(cy1, b.y0 / 2 + b.y1 / 2);
        }
        nodes_[id].box = box;
        nodes_[id].minArea = minArea;
        if (end - begin <= kLeafSize) {
            nodes_[id].leaf = true;
            nodes_[id].a = begin;
            nodes_[id].b = end;
            return id;
        }
        const bool alongX = static_cast<double>(cx1) - static_cast<double>(cx0) >=
                            static_cast<double>(cy1) - static_cast<double>(cy0);
        const std::size_t mid = begin + (end - begin) / 2;
        const std::vector<Box>& boxes = boxes_;
        std::nth_element(items_.begin() + static_cast<std::ptrdiff_t>(begin),
                         items_.begin() + static_cast<std::ptrdiff_t>(mid),
                         items_.begin() + static_cast<std::ptrdiff_t>(end),
                         [&boxes, alongX](std::size_t a, std::size_t b) {
                             const Box& ba = boxes[a];
                             const Box& bb = boxes[b];
                             const std::int64_t ca = alongX ? ba.x0 / 2 + ba.x1 / 2 : ba.y0 / 2 + ba.y1 / 2;
                             const std::int64_t cb = alongX ? bb.x0 / 2 + bb.x1 / 2 : bb.y0 / 2 + bb.y1 / 2;
                             return ca < cb || (ca == cb && a < b);
                         });
        const std::size_t left = build(begin, mid);
        const std::size_t right = build(mid, end);
        nodes_[id].a = left;
        nodes_[id].b = right;
        return id;
    }

    const std::vector<Box>& boxes_;
    std::vector<std::size_t> items_;
    std::vector<Node> nodes_;
    std::vector<Entry> heap_;  // a max-heap under Entry::operator<
};

}  // namespace

template <class Ring>
bool everyHoleNested(const std::vector<Ring>& rings, const std::vector<int>& kind) {
    std::vector<Box> boxes(rings.size());
    std::vector<std::size_t> with;
    for (std::size_t i = 0; i < rings.size(); ++i) {
        if (kind[i] != 0 && !rings[i].empty()) {
            boxes[i] = boxOf(rings[i]);
        }
        if (kind[i] == 1 && !rings[i].empty()) {
            with.push_back(i);
        }
    }
    // Locators that may index their ring live as long as the scan; small rings are walked.
    std::vector<std::size_t> slot(rings.size(), std::numeric_limits<std::size_t>::max());
    std::deque<RingLocator<Ring>> locators;
    std::int64_t px = 0;
    std::int64_t py = 0;
    auto holds = [&](std::size_t w) {
        if (rings[w].size() < kRingIndexMinVertices) {
            return RingLocator<Ring>(rings[w]).locate(px, py) == RingSide::Inside;
        }
        if (slot[w] == std::numeric_limits<std::size_t>::max()) {
            slot[w] = locators.size();
            locators.emplace_back(rings[w]);
        }
        return locators[slot[w]].locate(px, py) == RingSide::Inside;
    };
    // Shapes usually come with their holes right after them: the last ring wound like the lowest
    // before a hole is tried first, and the box hierarchy is only built when that fails. Which
    // candidate is found first does not change the decision.
    std::optional<BoxTree> tree;
    std::size_t lastWith = std::numeric_limits<std::size_t>::max();
    for (std::size_t h = 0; h < rings.size(); ++h) {
        if (kind[h] == 1 && !rings[h].empty()) {
            lastWith = h;
        }
        if (kind[h] != 2) {
            continue;
        }
        if (rings[h].empty()) {
            return false;
        }
        px = rings[h].front().x;
        py = rings[h].front().y;
        if (lastWith != std::numeric_limits<std::size_t>::max() && boxes[lastWith].holds(boxes[h]) && holds(lastWith)) {
            continue;
        }
        if (!tree) {
            tree.emplace(boxes, with);
        }
        if (!tree->anyHolding(boxes[h], holds)) {
            return false;
        }
    }
    return true;
}

template class RingLocator<GridRing>;
template class RingLocator<Clipper2Lib::Path64>;
template bool everyHoleNested<GridRing>(const std::vector<GridRing>&, const std::vector<int>&);
template bool everyHoleNested<Clipper2Lib::Path64>(const Clipper2Lib::Paths64&, const std::vector<int>&);

}  // namespace carvetto::geometry::detail
