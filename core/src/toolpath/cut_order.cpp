#include "carvetto/toolpath/cut_order.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/toolpath/box_index.hpp"

namespace carvetto::toolpath {

namespace {

enum class Strategy { Selection, LeftToRight, BottomToTop, Grid, Nearest };

struct Node {
    std::vector<std::size_t> children;
};

// Sorts sibling items by a strategy; Nearest is handled during the walk.
void sortSiblings(std::vector<std::size_t>& ids, const std::vector<OrderItem>& items,
                  const std::vector<geometry::Box2>& boxes, Strategy s) {
    auto bySel = [&](std::size_t a, std::size_t b) {
        return items[a].selectionIndex != items[b].selectionIndex ? items[a].selectionIndex < items[b].selectionIndex
                                                                  : a < b;
    };
    switch (s) {
        case Strategy::Selection:
            std::sort(ids.begin(), ids.end(), bySel);
            break;
        case Strategy::LeftToRight:
            std::sort(ids.begin(), ids.end(), [&](std::size_t a, std::size_t b) {
                if (boxes[a].min.x != boxes[b].min.x) return boxes[a].min.x < boxes[b].min.x;
                if (boxes[a].min.y != boxes[b].min.y) return boxes[a].min.y < boxes[b].min.y;
                return bySel(a, b);
            });
            break;
        case Strategy::BottomToTop:
            std::sort(ids.begin(), ids.end(), [&](std::size_t a, std::size_t b) {
                if (boxes[a].min.y != boxes[b].min.y) return boxes[a].min.y < boxes[b].min.y;
                if (boxes[a].min.x != boxes[b].min.x) return boxes[a].min.x < boxes[b].min.x;
                return bySel(a, b);
            });
            break;
        case Strategy::Grid: {
            std::sort(ids.begin(), ids.end(), [&](std::size_t a, std::size_t b) {
                if (boxes[a].min.y != boxes[b].min.y) return boxes[a].min.y < boxes[b].min.y;
                if (boxes[a].min.x != boxes[b].min.x) return boxes[a].min.x < boxes[b].min.x;
                return bySel(a, b);
            });
            std::vector<std::size_t> out;
            std::size_t i = 0;
            bool leftToRight = true;
            while (i < ids.size()) {
                const geometry::Box2& first = boxes[ids[i]];
                std::size_t j = i + 1;
                while (j < ids.size() && boxes[ids[j]].min.y <= first.max.y) {
                    ++j;
                }
                std::vector<std::size_t> row(ids.begin() + static_cast<std::ptrdiff_t>(i),
                                             ids.begin() + static_cast<std::ptrdiff_t>(j));
                std::sort(row.begin(), row.end(), [&](std::size_t a, std::size_t b) {
                    if (boxes[a].min.x != boxes[b].min.x) {
                        return leftToRight ? boxes[a].min.x < boxes[b].min.x : boxes[a].min.x > boxes[b].min.x;
                    }
                    return bySel(a, b);
                });
                out.insert(out.end(), row.begin(), row.end());
                leftToRight = !leftToRight;
                i = j;
            }
            ids = out;
            break;
        }
        case Strategy::Nearest:
            std::sort(ids.begin(), ids.end(), bySel);
            break;
    }
}

void walk(std::size_t node, const std::vector<Node>& tree, const std::vector<OrderItem>& items,
          const std::vector<geometry::Box2>& boxes, Strategy s, geometry::Vec2& pos, std::vector<std::size_t>& out) {
    std::vector<std::size_t> kids = tree[node].children;
    sortSiblings(kids, items, boxes, s);
    if (s == Strategy::Nearest) {
        // Greedy nearest neighbour: the unused sibling whose start is nearest to the tool, ties to
        // the first in selection order. The starts are indexed (T-0123): a square of half-width h
        // round the tool is searched, growing until the best distance found is <= h (every start
        // outside it is farther), so the choice is the same as a scan over all siblings.
        std::vector<geometry::Box2> starts(kids.size());
        geometry::Box2 all;
        for (std::size_t k = 0; k < kids.size(); ++k) {
            starts[k] = geometry::Box2{items[kids[k]].start, items[kids[k]].start};
            all.expand(items[kids[k]].start);
        }
        const BoxIndex index(std::move(starts));
        const double count = static_cast<double>(std::max<std::size_t>(kids.size(), 1));
        const double step = std::max(std::max(all.width(), all.height()) / std::sqrt(count), 1e-9);
        std::vector<bool> used(kids.size(), false);
        for (std::size_t n = 0; n < kids.size(); ++n) {
            std::size_t best = kids.size();
            double bestD = std::numeric_limits<double>::infinity();
            for (double h = step;; h *= 2.0) {
                const geometry::Box2 square{pos - geometry::Vec2{h, h}, pos + geometry::Vec2{h, h}};
                for (const std::size_t k : index.query(square)) {
                    if (used[k]) continue;
                    const double d = geometry::distance(pos, items[kids[k]].start);
                    if (d < bestD || (d == bestD && k < best)) {
                        bestD = d;
                        best = k;
                    }
                }
                if (bestD <= h || square.contains(all)) {
                    break;  // nothing outside the square can be nearer, or nothing is outside it
                }
            }
            used[best] = true;
            walk(kids[best] + 1, tree, items, boxes, s, pos, out);
        }
    } else {
        for (std::size_t k : kids) {
            walk(k + 1, tree, items, boxes, s, pos, out);
        }
    }
    if (node > 0) {
        out.push_back(node - 1);
        pos = items[node - 1].end;
    }
}

}  // namespace

double travelLength(const std::vector<OrderItem>& items, const std::vector<std::size_t>& order, geometry::Vec2 from) {
    double total = 0.0;
    geometry::Vec2 pos = from;
    for (std::size_t i : order) {
        total += geometry::distance(pos, items[i].start);
        pos = items[i].end;
    }
    return total;
}

std::vector<std::size_t> orderCuts(const std::vector<OrderItem>& items, const OrderOptions& options,
                                   geometry::Vec2 from, const geometry::Tolerance& tol) {
    const std::size_t n = items.size();
    std::vector<geometry::Box2> boxes(n);
    std::vector<double> areas(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        boxes[i] = items[i].path.bounds();
        areas[i] = items[i].path.closed ? geometry::area(items[i].path) : 0.0;
    }
    // Containment tree: the parent of an item is the smallest-area closed item, larger than it,
    // that contains a point of it. Node 0 is the root, node i+1 is item i.
    std::vector<Node> tree(n + 1);
    std::vector<geometry::Box2> closedBoxes(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (items[i].path.closed) {
            closedBoxes[i] = boxes[i];
        }
    }
    const BoxIndex index(std::move(closedBoxes));
    for (std::size_t i = 0; i < n; ++i) {
        if (items[i].path.isEmpty()) {
            tree[0].children.push_back(i);
            continue;
        }
        // Probe point: midpoint of the first span (avoids a junction shared with a neighbour).
        const geometry::Vec2 probe = geometry::pointAt(items[i].path.spans.front(), 0.5);
        std::size_t parent = n;
        double parentArea = std::numeric_limits<double>::infinity();
        for (std::size_t j : index.containing(probe)) {
            if (j == i || !items[j].path.closed || areas[j] <= areas[i] || areas[j] >= parentArea) {
                continue;
            }
            if (geometry::locatePoint(items[j].path, probe, geometry::FillRule::EvenOdd, tol) ==
                geometry::PointLocation::Inside) {
                parent = j;
                parentArea = areas[j];
            }
        }
        tree[parent == n ? 0 : parent + 1].children.push_back(i);
    }

    std::vector<Strategy> candidates;
    if (options.selectionOrder) candidates.push_back(Strategy::Selection);
    if (options.leftToRight) candidates.push_back(Strategy::LeftToRight);
    if (options.bottomToTop) candidates.push_back(Strategy::BottomToTop);
    if (options.grid) candidates.push_back(Strategy::Grid);
    if (candidates.empty()) candidates.push_back(Strategy::Nearest);

    std::vector<std::size_t> best;
    double bestCost = std::numeric_limits<double>::infinity();
    for (Strategy s : candidates) {
        std::vector<std::size_t> order;
        geometry::Vec2 pos = from;
        walk(0, tree, items, boxes, s, pos, order);
        const double cost = travelLength(items, order, from);
        if (cost < bestCost) {
            bestCost = cost;
            best = order;
        }
    }
    return best;
}

}  // namespace carvetto::toolpath
