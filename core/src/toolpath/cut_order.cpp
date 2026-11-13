#include "carvetto/toolpath/cut_order.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
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

using PointKey = std::array<std::uint64_t, 2>;

// Bit pattern of a point for exact-equality grouping (+0.0 folds -0.0 into 0.0).
PointKey pointKey(geometry::Vec2 p) {
    return {std::bit_cast<std::uint64_t>(p.x + 0.0), std::bit_cast<std::uint64_t>(p.y + 0.0)};
}

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
          const std::vector<geometry::Box2>& boxes, Strategy s, geometry::Vec2& pos, std::vector<std::size_t>& out, OrderStats* stats) {
    std::vector<std::size_t> kids = tree[node].children;
    sortSiblings(kids, items, boxes, s);
    if (s == Strategy::Nearest) {
        // Greedy nearest neighbour: the unused sibling whose start is nearest to the tool, ties to
        // the first in selection order. The starts are indexed (T-0123): a square of half-width h
        // round the tool is searched, growing until the best distance found is <= h (every start
        // outside it is farther), so the choice is the same as a scan over all siblings.
        // Starts that are exactly equal (coincident vectors, T-0267) form one group: its unused
        // members are taken in ascending order, and the index holds one point per group, so a
        // pile of identical starts costs one distance per step, not one per member.
        std::vector<std::size_t> byStart(kids.size());
        std::iota(byStart.begin(), byStart.end(), std::size_t{0});
        auto startKey = [&](std::size_t k) { return pointKey(items[kids[k]].start); };
        std::sort(byStart.begin(), byStart.end(), [&](std::size_t a, std::size_t b) {
            const auto ka = startKey(a), kb = startKey(b);
            return ka != kb ? ka < kb : a < b;
        });
        std::vector<std::vector<std::size_t>> groups;  // members (positions in kids), ascending
        for (std::size_t r = 0; r < byStart.size(); ++r) {
            if (r == 0 || startKey(byStart[r]) != startKey(byStart[r - 1])) {
                groups.emplace_back();
            }
            groups.back().push_back(byStart[r]);
        }
        std::vector<geometry::Box2> starts(groups.size());
        geometry::Box2 all;
        for (std::size_t g = 0; g < groups.size(); ++g) {
            const geometry::Vec2 st = items[kids[groups[g].front()]].start;
            starts[g] = geometry::Box2{st, st};
            all.expand(st);
        }
        const BoxIndex index(std::move(starts));
        const double count = static_cast<double>(std::max<std::size_t>(kids.size(), 1));
        const double step = std::max(std::max(all.width(), all.height()) / std::sqrt(count), 1e-9);
        std::vector<std::size_t> cursor(groups.size(), 0);
        for (std::size_t n = 0; n < kids.size(); ++n) {
            std::size_t best = kids.size();
            std::size_t bestGroup = groups.size();
            double bestD = std::numeric_limits<double>::infinity();
            for (double h = step;; h *= 2.0) {
                const geometry::Box2 square{pos - geometry::Vec2{h, h}, pos + geometry::Vec2{h, h}};
                for (const std::size_t g : index.query(square)) {
                    if (cursor[g] >= groups[g].size()) continue;
                    const std::size_t k = groups[g][cursor[g]];
                    const double d = geometry::distance(pos, items[kids[k]].start);
                    if (stats != nullptr) ++stats->distanceEvals;
                    if (d < bestD || (d == bestD && k < best)) {
                        bestD = d;
                        best = k;
                        bestGroup = g;
                    }
                }
                if (bestD <= h || square.contains(all)) {
                    break;  // nothing outside the square can be nearer, or nothing is outside it
                }
            }
            if (bestGroup == groups.size()) {
                // Only with non-finite coordinates: take the first unused sibling.
                for (std::size_t g = 0; g < groups.size(); ++g) {
                    if (cursor[g] < groups[g].size() && (bestGroup == groups.size() ||
                                                         groups[g][cursor[g]] < groups[bestGroup][cursor[bestGroup]])) {
                        bestGroup = g;
                    }
                }
                best = groups[bestGroup][cursor[bestGroup]];
            }
            ++cursor[bestGroup];
            walk(kids[best] + 1, tree, items, boxes, s, pos, out, stats);
        }
    } else {
        for (std::size_t k : kids) {
            walk(k + 1, tree, items, boxes, s, pos, out, stats);
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
                                   geometry::Vec2 from, const geometry::Tolerance& tol, OrderStats* stats) {
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
    // Closed items with the same bounds and area are grouped (T-0267): such items are all skipped
    // or all considered by the area tests below, so a pile of coincident loops is one index entry.
    std::vector<std::size_t> closedIds;
    for (std::size_t i = 0; i < n; ++i) {
        if (items[i].path.closed) closedIds.push_back(i);
    }
    auto shapeKey = [&](std::size_t i) {
        return std::array<std::uint64_t, 5>{std::bit_cast<std::uint64_t>(boxes[i].min.x + 0.0),
                                            std::bit_cast<std::uint64_t>(boxes[i].min.y + 0.0),
                                            std::bit_cast<std::uint64_t>(boxes[i].max.x + 0.0),
                                            std::bit_cast<std::uint64_t>(boxes[i].max.y + 0.0),
                                            std::bit_cast<std::uint64_t>(areas[i] + 0.0)};
    };
    std::sort(closedIds.begin(), closedIds.end(), [&](std::size_t a, std::size_t b) {
        const auto ka = shapeKey(a), kb = shapeKey(b);
        return ka != kb ? ka < kb : a < b;
    });
    std::vector<std::vector<std::size_t>> shapeGroups;  // members ascending
    std::vector<geometry::Box2> closedBoxes;
    for (std::size_t r = 0; r < closedIds.size(); ++r) {
        if (r == 0 || shapeKey(closedIds[r]) != shapeKey(closedIds[r - 1])) {
            shapeGroups.emplace_back();
            closedBoxes.push_back(boxes[closedIds[r]]);
        }
        shapeGroups.back().push_back(closedIds[r]);
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
        for (std::size_t g : index.containing(probe)) {
            const std::vector<std::size_t>& members = shapeGroups[g];
            if (areas[members.front()] <= areas[i]) continue;  // the whole group is not larger
            std::size_t tested = n;  // the last member located, and its answer
            bool testedInside = false;
            for (std::size_t j : members) {
                // Smallest area wins, equal areas go to the lower index (as a scan over all items).
                if (j == i || areas[j] > parentArea ||
                    (areas[j] == parentArea && (parent == n || j > parent))) {
                    continue;
                }
                bool inside;
                if (tested != n && items[tested].path == items[j].path) {
                    inside = testedInside;
                } else {
                    if (stats != nullptr) ++stats->containmentTests;
                    inside = geometry::locatePoint(items[j].path, probe, geometry::FillRule::EvenOdd, tol) ==
                             geometry::PointLocation::Inside;
                    tested = j;
                    testedInside = inside;
                }
                if (inside) {
                    parent = j;
                    parentArea = areas[j];
                    break;  // later members have the same area: never nearer than this one
                }
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
        walk(0, tree, items, boxes, s, pos, order, stats);
        const double cost = travelLength(items, order, from);
        if (cost < bestCost) {
            bestCost = cost;
            best = order;
        }
    }
    return best;
}

}  // namespace carvetto::toolpath
