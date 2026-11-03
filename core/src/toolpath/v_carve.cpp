#include "carvetto/toolpath/v_carve.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/geometry/intersect.hpp"
#include "carvetto/geometry/medial_axis.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/toolpath/box_index.hpp"
#include "carvetto/toolpath/move.hpp"

namespace carvetto::toolpath {

namespace {

using geometry::Box2;
using geometry::Contour;
using geometry::MedialAxis;
using geometry::Vec2;

constexpr double kMinChordal = 1e-4;      // mm
constexpr double kMaxMoves = 20000000.0;  // moves per V-carve (as Pocket, Drill, Engrave)

/// A number for a message: whole numbers in full, others to 6 significant digits.
std::string num(double v) {
    if (std::isfinite(v) && std::fabs(v) < 1e15 && v == std::trunc(v)) {
        return std::to_string(static_cast<long long>(v));
    }
    std::array<char, 64> buf{};
    const auto res = std::to_chars(buf.data(), buf.data() + buf.size(), v, std::chars_format::general, 6);
    return std::string(buf.data(), res.ptr);
}

std::string vecName(std::size_t i) {
    return "vector " + std::to_string(i);
}

std::string paramProblem(const VCarvingParams& p) {
    const double values[] = {p.startDepth,       p.flatDepth,       p.safeZ,
                             p.clearanceRasterAngle, p.rampDistance, p.startPosition.x,
                             p.startPosition.y,  p.tolerance.distance, p.tolerance.chordal,
                             p.tolerance.angle};
    for (const double v : values) {
        if (!std::isfinite(v)) {
            return "a parameter is not a finite number";
        }
    }
    if (p.startDepth < 0.0) {
        return "start depth must not be negative";
    }
    if (p.flatDepth < 0.0) {
        return "flat depth must not be negative (0 = no flat depth)";
    }
    if (!(p.safeZ > 0.0)) {
        return "safe Z must be above the material surface";
    }
    if (p.rampPlunge && !(p.rampDistance > 0.0)) {
        return "ramp distance must be greater than zero";
    }
    const geometry::Tolerance& t = p.tolerance;
    if (t.chordal < kMinChordal || t.chordal > 1.0 || !(t.distance > 0.0) || t.distance > t.chordal) {
        return "invalid tolerance: chordal must be between 0.0001 and 1 mm, distance > 0 and <= chordal";
    }
    return {};
}

const char* toolTypeText(tooling::ToolType type) {
    switch (type) {
        case tooling::ToolType::EndMill: return "an end mill";
        case tooling::ToolType::BallNose: return "a ball nose";
        case tooling::ToolType::VBit: return "a V-bit";
        case tooling::ToolType::Engraving: return "an engraving tool";
        case tooling::ToolType::TaperedBallNose: return "a tapered ball nose";
        case tooling::ToolType::RadiusedEndMill: return "a radiused end mill";
        case tooling::ToolType::FormTool: return "a form tool";
        case tooling::ToolType::DiamondDrag: return "a diamond drag tool";
        case tooling::ToolType::Drill: return "a drill";
    }
    return "an unknown tool type";
}

std::string toolProblem(const tooling::Tool& tool, const ToolSnapshot& s) {
    const std::string label = "tool '" + tool.name + "'";
    if (!vCarvingToolAllowed(tool.type)) {
        return label + " is " + toolTypeText(tool.type) + "; V-carving needs a V-bit or a ball nose";
    }
    if (const std::optional<std::string> bad = tooling::validate(tool)) {
        return label + ": " + *bad;
    }
    if (!(vCarvingFullWidthDepth(tool) > 0.0) || !std::isfinite(vCarvingFullWidthDepth(tool))) {
        return label + " has no usable cutting depth";
    }
    if (!std::isfinite(s.plungeRate) || !(s.plungeRate > 0.0)) {
        return label + ": the plunge rate must be greater than zero";
    }
    if (!std::isfinite(s.feedRate) || !(s.feedRate > 0.0)) {
        return label + ": the feed rate must be greater than zero";
    }
    return {};
}

/// Half the width the carving tool cuts at `depth` below the start plane.
double footprint(const tooling::Tool& tool, double depth) {
    return 0.5 * tooling::cuttingWidthAtDepth(tool, depth);
}

// ----------------------------------------------------------------------------- regions

/// A boundary contour of the region once crossings are resolved.
struct Boundary {
    Contour contour;
    /// Smallest input index it comes from (names it in diagnostics and selection order).
    std::size_t source{0};
    /// Polygonised by the even-odd union of crossing input (`kResolveFraction` of chordal).
    bool resolved{false};
    Box2 box;
    double area{0.0};
};

/// Crossing input is polygonised at this fraction of the chordal tolerance; the carve margin of
/// such regions grows by the same amount (the polygon may lie that far outside a curved wall).
constexpr double kResolveFraction = 0.125;

std::size_t findRoot(std::vector<std::size_t>& parent, std::size_t i) {
    while (parent[i] != i) {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }
    return i;
}

/// The usable closed vectors as non-crossing boundaries of the same even-odd region: vectors
/// that cross, touch or overlap each other or themselves (`geometry::intersect`,
/// `hasSelfIntersections`) are resolved together with `geometry::unionContours` under the
/// even-odd rule, as Pocket resolves its region; the rest stay as they are (exact curves).
/// Even-odd membership is the parity over all contours, so resolving each crossing cluster on
/// its own gives the same region.
std::vector<Boundary> resolveCrossings(const std::vector<Contour>& contours, const std::vector<std::size_t>& usable,
                                       const geometry::Tolerance& tol, std::size_t& clusters) {
    const std::size_t n = usable.size();
    std::vector<Box2> boxes(n);
    for (std::size_t k = 0; k < n; ++k) {
        boxes[k] = contours[usable[k]].bounds();
    }
    const BoxIndex index(boxes);
    std::vector<std::size_t> parent(n);
    std::iota(parent.begin(), parent.end(), std::size_t{0});
    std::vector<bool> crossing(n, false);
    for (std::size_t k = 0; k < n; ++k) {
        if (geometry::hasSelfIntersections(contours[usable[k]], tol)) {
            crossing[k] = true;
        }
        for (const std::size_t j : index.query(boxes[k])) {
            if (j <= k) {
                continue;
            }
            if (!geometry::intersect(contours[usable[k]], contours[usable[j]], tol).isEmpty()) {
                crossing[k] = true;
                crossing[j] = true;
                const std::size_t ra = findRoot(parent, k);
                const std::size_t rb = findRoot(parent, j);
                if (ra != rb) {
                    parent[std::max(ra, rb)] = std::min(ra, rb);
                }
            }
        }
    }
    std::vector<std::vector<std::size_t>> members(n);
    for (std::size_t k = 0; k < n; ++k) {
        members[findRoot(parent, k)].push_back(k);
    }
    geometry::Tolerance fine = tol;
    fine.chordal = tol.chordal * kResolveFraction;
    fine.distance = std::min(tol.distance, fine.chordal);
    std::vector<Boundary> out;
    clusters = 0;
    for (std::size_t k = 0; k < n; ++k) {
        if (members[k].empty()) {
            continue;  // not a cluster root
        }
        bool cross = false;
        for (const std::size_t m : members[k]) {
            cross = cross || crossing[m];
        }
        if (!cross) {
            out.push_back(Boundary{contours[usable[k]], usable[k], false, boxes[k], 0.0});
            continue;
        }
        ++clusters;
        std::vector<Contour> cluster;
        for (const std::size_t m : members[k]) {
            cluster.push_back(contours[usable[m]]);
        }
        for (Contour& c : geometry::unionContours(cluster, fine, geometry::FillRule::EvenOdd)) {
            const Box2 box = c.bounds();
            out.push_back(Boundary{std::move(c), usable[k], true, box, 0.0});
        }
    }
    for (Boundary& b : out) {
        b.area = std::fabs(geometry::area(b.contour));
    }
    return out;
}

/// One connected part of the even-odd region: an outer boundary (at even nesting depth) and the
/// boundaries directly inside it (its holes). Parts cannot interact, so each gets its own medial
/// axis; a frame round a block of text is one part and every letter another.
struct Group {
    std::vector<std::size_t> boundaries;  // [0] the outer, then its holes
    std::vector<std::size_t> sources;     // input vectors, ascending, unique
    Box2 box;
    bool resolved{false};
};

std::vector<Group> nestRegions(const std::vector<Boundary>& bs, const geometry::Tolerance& tol) {
    const std::size_t n = bs.size();
    std::vector<Box2> boxes(n);
    for (std::size_t i = 0; i < n; ++i) {
        boxes[i] = bs[i].box;
    }
    const BoxIndex index(boxes);
    constexpr std::size_t kNone = std::numeric_limits<std::size_t>::max();
    std::vector<std::size_t> parent(n, kNone);
    for (std::size_t i = 0; i < n; ++i) {
        if (bs[i].contour.isEmpty()) {
            continue;
        }
        // The boundaries do not cross or touch, so any point of one decides its nesting.
        const Vec2 probe = geometry::pointAt(bs[i].contour.spans.front(), 0.5);
        double best = std::numeric_limits<double>::infinity();
        for (const std::size_t j : index.query(boxes[i])) {
            if (j == i || !(bs[j].area > bs[i].area) || !(bs[j].area < best) || !boxes[j].contains(boxes[i])) {
                continue;
            }
            if (geometry::locatePoint(bs[j].contour, probe, geometry::FillRule::EvenOdd, tol) ==
                geometry::PointLocation::Inside) {
                parent[i] = j;
                best = bs[j].area;
            }
        }
    }
    std::vector<int> depth(n, -1);
    for (std::size_t i = 0; i < n; ++i) {
        std::vector<std::size_t> chain;
        std::size_t c = i;
        while (c != kNone && depth[c] < 0) {
            chain.push_back(c);
            c = parent[c];
        }
        int d = c == kNone ? -1 : depth[c];
        for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
            depth[*it] = ++d;
        }
    }
    std::vector<Group> groups;
    std::vector<std::size_t> groupOf(n, kNone);
    for (std::size_t i = 0; i < n; ++i) {
        if (depth[i] % 2 == 0) {
            groupOf[i] = groups.size();
            Group g;
            g.boundaries.push_back(i);
            g.box = boxes[i];
            groups.push_back(std::move(g));
        }
    }
    for (std::size_t i = 0; i < n; ++i) {
        if (depth[i] % 2 == 1) {
            Group& g = groups[groupOf[parent[i]]];
            g.boundaries.push_back(i);
        }
    }
    for (Group& g : groups) {
        for (const std::size_t i : g.boundaries) {
            g.sources.push_back(bs[i].source);
            g.resolved = g.resolved || bs[i].resolved;
        }
        std::sort(g.sources.begin(), g.sources.end());
        g.sources.erase(std::unique(g.sources.begin(), g.sources.end()), g.sources.end());
    }
    std::stable_sort(groups.begin(), groups.end(),
                     [](const Group& x, const Group& y) { return x.sources.front() < y.sources.front(); });
    return groups;
}

// ----------------------------------------------------------------------------- skeleton walks

struct SkelPoint {
    Vec2 p;
    double r{0.0};
};
using Trail = std::vector<SkelPoint>;

Vec2 unitOr(Vec2 v, Vec2 fallback) {
    const double l = v.length();
    return l > 0.0 ? v / l : fallback;
}

/// The skeleton as continuous walks: each branch is used once; a walk starts at an odd-degree
/// node when there is one (an end of the skeleton: corner tips) and at every node takes the
/// unused branch that turns least, so a connected skeleton with k ends is cut in about k/2
/// walks. Isolated nodes (a circle's centre) are walks of one point.
std::vector<Trail> skeletonTrails(const MedialAxis& ax) {
    std::vector<Trail> trails;
    const std::size_t nb = ax.branches.size();
    std::vector<bool> used(nb, false);
    std::vector<std::size_t> starts;
    for (std::size_t n = 0; n < ax.nodes.size(); ++n) {
        if (ax.nodes[n].branches.size() % 2 == 1) {
            starts.push_back(n);
        }
    }
    for (std::size_t n = 0; n < ax.nodes.size(); ++n) {
        if (ax.nodes[n].branches.size() % 2 == 0) {
            starts.push_back(n);
        }
    }
    auto leaving = [&](std::size_t b, bool forward) {
        const auto& pts = ax.branches[b].points;
        if (pts.size() < 2) {
            return Vec2{0.0, 0.0};
        }
        return forward ? pts[1].point - pts[0].point : pts[pts.size() - 2].point - pts.back().point;
    };
    for (const std::size_t s : starts) {
        const geometry::MedialNode& node = ax.nodes[s];
        if (node.branches.empty()) {
            trails.push_back(Trail{SkelPoint{node.point, node.radius}});
            continue;
        }
        for (;;) {
            std::size_t cur = s;
            Trail trail;
            Vec2 dir{0.0, 0.0};
            bool any = false;
            for (;;) {
                std::size_t best = nb;
                bool bestForward = true;
                double bestScore = -std::numeric_limits<double>::infinity();
                for (const std::size_t b : ax.nodes[cur].branches) {
                    if (used[b]) {
                        continue;
                    }
                    const geometry::MedialBranch& br = ax.branches[b];
                    // A loop (start == end) is walked forwards.
                    const bool forward = br.startNode == cur;
                    const Vec2 out = unitOr(leaving(b, forward), Vec2{0.0, 0.0});
                    const double score = trail.empty() ? 0.0 : dir.dot(out);
                    if (best == nb || score > bestScore) {
                        best = b;
                        bestForward = forward;
                        bestScore = score;
                    }
                }
                if (best == nb) {
                    break;
                }
                used[best] = true;
                any = true;
                const geometry::MedialBranch& br = ax.branches[best];
                const std::size_t count = br.points.size();
                for (std::size_t k = 0; k < count; ++k) {
                    const geometry::MedialPoint& m = br.points[bestForward ? k : count - 1 - k];
                    if (k == 0 && !trail.empty()) {
                        continue;  // the node the walk stands on
                    }
                    trail.push_back(SkelPoint{m.point, m.radius});
                }
                if (trail.size() >= 2) {
                    dir = unitOr(trail.back().p - trail[trail.size() - 2].p, dir);
                }
                cur = bestForward ? br.endNode : br.startNode;
            }
            if (!any) {
                break;
            }
            trails.push_back(std::move(trail));
        }
    }
    return trails;
}

/// Drops walk points that lie within `eps` (position plus radius) of the segment joining their
/// neighbours, so long straight bisectors become single moves.
Trail simplify(const Trail& in, double eps) {
    if (in.size() <= 2) {
        return in;
    }
    constexpr std::size_t kWindow = 512;
    Trail out;
    out.push_back(in.front());
    std::size_t a = 0;
    while (a + 1 < in.size()) {
        std::size_t b = a + 1;
        // Extend b while every point between a and the candidate stays within eps.
        while (b + 1 < in.size() && b + 1 - a <= kWindow) {
            const std::size_t c = b + 1;
            const Vec2 pa = in[a].p;
            const Vec2 ab = in[c].p - pa;
            const double l2 = ab.lengthSquared();
            bool fits = true;
            for (std::size_t k = a + 1; k < c && fits; ++k) {
                double t = l2 > 0.0 ? (in[k].p - pa).dot(ab) / l2 : 0.0;
                t = std::clamp(t, 0.0, 1.0);
                const Vec2 q = pa + ab * t;
                const double rq = in[a].r + (in[c].r - in[a].r) * t;
                fits = geometry::distance(in[k].p, q) + std::fabs(in[k].r - rq) <= eps;
            }
            if (!fits) {
                break;
            }
            b = c;
        }
        out.push_back(in[b]);
        a = b;
    }
    return out;
}

struct CarvePoint {
    Vec2 p;
    double depth{0.0};  // below the start plane
};

struct DepthRule {
    const tooling::Tool* tool{nullptr};
    double margin{0.0};    // subtracted from the inscribed radius
    double flatDepth{0.0}; // the depth limit (flat depth or the tool's full-width depth)
    double limitRadius{0.0};  // the footprint radius at the limit
    double depth(double r) const { return std::min(flatDepth, vCarvingDepthForRadius(*tool, std::max(0.0, r - margin))); }
};

/// Slope dd/ds of the ball nose's depth as a function of its footprint radius s (its inverse
/// footprint d(s) = R − sqrt(R² − s²) is convex); infinite at s >= R.
double ballSlope(double radius, double s) {
    const double q = (radius - s) * (radius + s);
    return q > 0.0 ? s / std::sqrt(q) : std::numeric_limits<double>::infinity();
}

/// Ball nose piece a → b on which the allowed footprint s = r − margin is linear and within
/// [0, footprint at the limit]: the depth d(s) is convex, so linear Z is too deep in the middle.
/// At a depth c of the chord the tool's footprint exceeds the allowed one by at most
/// |Δs|·(1 − K/k), K the chord's slope and k the curve's slope at the deeper end (the curve
/// lies above its tangent there), so pieces are halved until that is at most margin/2.
void appendBallPiece(const DepthRule& rule, const SkelPoint& a, double da, const SkelPoint& b, double db,
                     std::vector<CarvePoint>& out, int level) {
    const double radius = 0.5 * rule.tool->diameter;
    const double s0 = std::max(0.0, a.r - rule.margin);
    const double s1 = std::max(0.0, b.r - rule.margin);
    const double ds = std::fabs(s1 - s0);
    if (level < 40 && ds > 0.0) {
        const double chord = std::fabs(db - da) / ds;
        const double deep = ballSlope(radius, std::max(s0, s1));
        const double excess = std::isfinite(deep) && deep > 0.0 ? ds * std::max(0.0, 1.0 - chord / deep) : ds;
        if (excess > 0.5 * rule.margin) {
            const SkelPoint m{(a.p + b.p) * 0.5, 0.5 * (a.r + b.r)};
            const double dm = rule.depth(m.r);
            appendBallPiece(rule, a, da, m, dm, out, level + 1);
            appendBallPiece(rule, m, dm, b, db, out, level + 1);
            return;
        }
    }
    out.push_back(CarvePoint{b.p, db});
}

/// Appends the carve points of the segment a → b (a excluded), Z linear between them. The
/// segment is first cut where the allowed footprint r − margin reaches 0 or the footprint at the
/// limit: there the depth rule min(limit, depth(max(0, r − margin))) has a kink, and linear Z
/// across it would cut too shallow (at the limit) or too deep (at 0). Between kinks a V-bit's
/// depth is linear in the radius, so its footprint follows the interpolated radius less the
/// margin exactly; a ball nose's pieces are refined by `appendBallPiece`.
void appendSegment(const DepthRule& rule, const SkelPoint& a, double da, const SkelPoint& b, double db,
                   std::vector<CarvePoint>& out) {
    const double kinks[] = {rule.margin, rule.limitRadius + rule.margin};
    std::vector<double> cuts;
    for (const double k : kinks) {
        if ((a.r - k) * (b.r - k) < 0.0) {
            cuts.push_back((k - a.r) / (b.r - a.r));
        }
    }
    std::sort(cuts.begin(), cuts.end());
    SkelPoint from = a;
    double dFrom = da;
    const bool vBit = rule.tool->type == tooling::ToolType::VBit;
    for (const double t : cuts) {
        const SkelPoint m{a.p + (b.p - a.p) * t, a.r + (b.r - a.r) * t};
        const double dm = rule.depth(m.r);
        if (vBit) {
            out.push_back(CarvePoint{m.p, dm});
        } else {
            appendBallPiece(rule, from, dFrom, m, dm, out, 0);
        }
        from = m;
        dFrom = dm;
    }
    if (vBit) {
        out.push_back(CarvePoint{b.p, db});
    } else {
        appendBallPiece(rule, from, dFrom, b, db, out, 0);
    }
}

std::vector<CarvePoint> carvePoints(const Trail& trail, const DepthRule& rule) {
    std::vector<CarvePoint> out;
    if (trail.empty()) {
        return out;
    }
    double prev = rule.depth(trail.front().r);
    out.push_back(CarvePoint{trail.front().p, prev});
    for (std::size_t k = 1; k < trail.size(); ++k) {
        const double d = rule.depth(trail[k].r);
        appendSegment(rule, trail[k - 1], prev, trail[k], d, out);
        prev = d;
    }
    return out;
}

/// One cut of a group: a skeleton walk (open) or a flat-area wall (closed ring at the limit).
struct Cut {
    std::vector<CarvePoint> points;
    bool closed{false};
};

struct GroupResult {
    std::vector<Contour> local;  // the group's boundaries (outer first)
    std::vector<Cut> cuts;
    double maxRadius{0.0};
    double margin{0.0};
    bool flat{false};  // has an area deeper than the depth limit
};

/// Pocket diagnostics of many regions: the first of each kind (issue and tool) is kept, later
/// ones only counted, so 5,000 letters do not repeat one warning 5,000 times.
struct DiagnosticMerger {
    std::vector<ToolpathDiagnostic> kept;
    std::vector<std::size_t> extra;
    void add(ToolpathDiagnostic d) {
        for (std::size_t i = 0; i < kept.size(); ++i) {
            if (kept[i].issue == d.issue && kept[i].tool == d.tool) {
                ++extra[i];
                return;
            }
        }
        kept.push_back(std::move(d));
        extra.push_back(0);
    }
    void flushInto(std::vector<ToolpathDiagnostic>& out) {
        for (std::size_t i = 0; i < kept.size(); ++i) {
            if (extra[i] > 0) {
                kept[i].message += " (and in " + std::to_string(extra[i]) + " more region(s))";
            }
            out.push_back(std::move(kept[i]));
        }
        kept.clear();
        extra.clear();
    }
};

void emitCut(const Cut& cut, double startDepth, double safeZ, std::vector<Move>& out) {
    const CarvePoint& first = cut.points.front();
    const auto z = [startDepth](double depth) { return -(startDepth + depth); };
    out.push_back(Move::rapid({first.p.x, first.p.y, safeZ}));
    out.push_back(Move::plunge({first.p.x, first.p.y, z(first.depth)}));
    Vec3 at{first.p.x, first.p.y, z(first.depth)};
    auto feedTo = [&](const CarvePoint& c) {
        const Vec3 to{c.p.x, c.p.y, z(c.depth)};
        if (std::fabs(to.x - at.x) > 1e-12 || std::fabs(to.y - at.y) > 1e-12 || std::fabs(to.z - at.z) > 1e-12) {
            out.push_back(Move::feed(to));
            at = to;
        }
    };
    for (std::size_t k = 1; k < cut.points.size(); ++k) {
        feedTo(cut.points[k]);
    }
    if (cut.closed) {
        feedTo(first);
    }
    out.push_back(Move::retract({at.x, at.y, safeZ}));
}

std::size_t cutMoves(const Cut& c) {
    return c.points.size() + 4;
}

/// Moves `moves` down by `dz` (Pocket computed them from the start plane as its surface).
void shiftZ(std::vector<Move>& moves, double dz) {
    for (Move& m : moves) {
        m.to.z -= dz;
    }
}

bool refusal(ToolpathIssue i) {
    return i == ToolpathIssue::InvalidTool || i == ToolpathIssue::InvalidParameters ||
           i == ToolpathIssue::WorkLimitExceeded || i == ToolpathIssue::NoTool;
}

}  // namespace

std::vector<Toolpath> VCarvingResult::toolpaths() const {
    std::vector<Toolpath> out;
    for (const Toolpath& c : clearance) {
        if (!c.isEmpty()) {
            out.push_back(c);
        }
    }
    if (!toolpath.isEmpty()) {
        out.push_back(toolpath);
    }
    return out;
}

bool vCarvingToolAllowed(tooling::ToolType type) {
    return type == tooling::ToolType::VBit || type == tooling::ToolType::BallNose;
}

bool vCarvingClearanceToolAllowed(tooling::ToolType type) {
    return type == tooling::ToolType::EndMill;
}

double vCarvingFullWidthDepth(const tooling::Tool& tool) {
    const double radius = 0.5 * tool.diameter;
    if (!(radius > 0.0) || !std::isfinite(radius)) {
        return 0.0;
    }
    if (tool.type == tooling::ToolType::VBit) {
        const double t = std::tan(0.5 * tool.includedAngle);
        return t > 0.0 && std::isfinite(t) ? radius / t : 0.0;
    }
    if (tool.type == tooling::ToolType::BallNose) {
        return radius;
    }
    return 0.0;
}

double vCarvingDepthForRadius(const tooling::Tool& tool, double r) {
    const double full = vCarvingFullWidthDepth(tool);
    const double radius = 0.5 * tool.diameter;
    if (!(full > 0.0) || !(r > 0.0)) {
        return 0.0;
    }
    if (r >= radius) {
        return full;
    }
    double d = 0.0;
    if (tool.type == tooling::ToolType::VBit) {
        d = r / std::tan(0.5 * tool.includedAngle);
    } else {
        d = radius - std::sqrt((radius - r) * (radius + r));
    }
    return std::clamp(d, 0.0, full);
}

VCarvingResult vCarvingToolpath(const std::vector<Contour>& contours, const tooling::Tool& tool,
                                const std::optional<tooling::CuttingData>& cuttingData,
                                const std::vector<PocketTool>& clearanceTools, const VCarvingParams& params) {
    VCarvingResult result;
    auto say = [&result](ToolpathIssue issue, std::optional<std::size_t> contour, std::string message,
                         std::optional<std::size_t> toolIndex = std::nullopt) {
        result.diagnostics.push_back(ToolpathDiagnostic{issue, contour, std::move(message), toolIndex});
    };
    const bool multi = !clearanceTools.empty();
    result.toolpath.name = multi ? params.name + " [" + tool.name + "]" : params.name;
    if (const std::string bad = paramProblem(params); !bad.empty()) {
        say(ToolpathIssue::InvalidParameters, std::nullopt, bad);
        return result;
    }
    const tooling::CuttingData& data = effectiveCuttingData(tool, cuttingData);
    result.toolpath.tool = toolSnapshot(tool, data);
    if (const std::string bad = toolProblem(tool, result.toolpath.tool); !bad.empty()) {
        say(ToolpathIssue::InvalidTool, std::nullopt, bad);
        return result;
    }
    for (std::size_t i = 0; i < clearanceTools.size(); ++i) {
        const tooling::Tool& ct = clearanceTools[i].tool;
        if (!vCarvingClearanceToolAllowed(ct.type)) {
            say(ToolpathIssue::InvalidTool, std::nullopt,
                "clearance tool " + std::to_string(i + 1) + " ('" + ct.name + "') is " + toolTypeText(ct.type) +
                    "; clearance tools must be end mills",
                i);
            return result;
        }
    }

    const geometry::Tolerance& tol = params.tolerance;
    const double full = vCarvingFullWidthDepth(tool);
    const double limit = params.flatDepth > 0.0 ? std::min(params.flatDepth, full) : full;
    if (params.flatDepth > full) {
        say(ToolpathIssue::CarveDepthLimited, std::nullopt,
            "the flat depth " + num(params.flatDepth) + " mm is deeper than tool '" + tool.name +
                "' reaches at its full width; the flat areas are cut at " + num(full) + " mm");
    }
    const double limitRadius = footprint(tool, limit);

    // Usable vectors.
    std::vector<std::size_t> usable;
    std::size_t openCount = 0;
    std::optional<std::size_t> firstOpen;
    for (std::size_t i = 0; i < contours.size(); ++i) {
        const Contour& c = contours[i];
        if (c.isEmpty()) {
            say(ToolpathIssue::EmptyContour, i, vecName(i) + " has no spans; skipped");
            continue;
        }
        if (!c.closed) {
            ++openCount;
            if (!firstOpen) {
                firstOpen = i;
            }
            continue;
        }
        if (!c.isContinuous(tol)) {
            say(ToolpathIssue::DiscontinuousContour, i, vecName(i) + " has gaps between spans; skipped");
            continue;
        }
        usable.push_back(i);
    }
    if (openCount > 0) {
        say(ToolpathIssue::OpenContourIgnored, firstOpen,
            std::to_string(openCount) + " open vector(s) ignored: V-carving needs closed vectors");
    }
    std::size_t crossingClusters = 0;
    const std::vector<Boundary> boundaries = resolveCrossings(contours, usable, tol, crossingClusters);
    const std::vector<Group> groups = nestRegions(boundaries, tol);
    result.regions = groups.size();

    // Medial axis and skeleton cuts per region.
    std::vector<GroupResult> carved(groups.size());
    std::size_t ignored = 0;
    std::optional<std::size_t> firstIgnored;
    bool anyCapped = false;
    for (std::size_t g = 0; g < groups.size(); ++g) {
        GroupResult& gr = carved[g];
        for (const std::size_t b : groups[g].boundaries) {
            gr.local.push_back(boundaries[b].contour);
        }
        gr.margin = (0.5 + (groups[g].resolved ? kResolveFraction : 0.0)) * tol.chordal;
        const MedialAxis ax = geometry::medialAxis(gr.local, tol);
        if (ax.ignoredContours > 0) {
            ignored += ax.ignoredContours;
            if (!firstIgnored) {
                firstIgnored = groups[g].sources.front();
            }
        }
        if (ax.accuracyCapped) {
            anyCapped = true;
            std::string names;
            for (const std::size_t i : groups[g].sources) {
                names += names.empty() ? "" : ", ";
                names += std::to_string(i);
            }
            say(ToolpathIssue::MedialAxisCapped, groups[g].sources.front(),
                "the medial axis of vector(s) " + names +
                    " could not be computed within the tolerance (too detailed for its budget, or too far from "
                    "the origin); nothing is carved: use a coarser chordal tolerance or simplify the vectors");
            continue;
        }
        if (anyCapped) {
            continue;  // nothing will be produced; only report the other capped regions
        }
        DepthRule rule;
        rule.tool = &tool;
        rule.margin = gr.margin;
        rule.flatDepth = limit;
        rule.limitRadius = limitRadius;
        for (const Trail& walk : skeletonTrails(ax)) {
            for (const SkelPoint& sp : walk) {
                gr.maxRadius = std::max(gr.maxRadius, sp.r);
            }
            Cut cut;
            cut.points = carvePoints(simplify(walk, tol.chordal / 16.0), rule);
            double deepest = 0.0;
            for (const CarvePoint& c : cut.points) {
                deepest = std::max(deepest, c.depth);
            }
            if (!(deepest >= tol.distance)) {
                continue;  // touches nothing
            }
            gr.cuts.push_back(std::move(cut));
        }
        gr.flat = gr.maxRadius - gr.margin > limitRadius;
    }
    if (anyCapped) {
        return result;
    }
    if (crossingClusters > 0) {
        say(ToolpathIssue::CrossingVectorsResolved, std::nullopt,
            std::to_string(crossingClusters) +
                " group(s) of crossing or touching vectors were resolved into their even-odd region (polygonised "
                "within " + num(tol.chordal * kResolveFraction) + " mm)");
    }
    if (ignored > 0) {
        say(ToolpathIssue::NoRegion, firstIgnored,
            std::to_string(ignored) +
                " closed vector(s) enclose no area, or are cancelled by a coincident vector (even-odd); ignored");
    }
    std::size_t flatGroups = 0;
    std::optional<std::size_t> firstFlat;
    for (std::size_t g = 0; g < groups.size(); ++g) {
        if (carved[g].flat) {
            ++flatGroups;
            if (!firstFlat) {
                firstFlat = groups[g].sources.front();
            }
        }
    }
    if (flatGroups > 0 && !(params.flatDepth > 0.0)) {
        say(ToolpathIssue::CarveDepthLimited, firstFlat,
            std::to_string(flatGroups) + " region(s) are wider than tool '" + tool.name + "' (" + num(tool.diameter) +
                " mm) can carve: carved to its full depth of " + num(full) + " mm, the flat bottom cleared");
    }

    // Order: regions by the chosen strategy (each region's cuts nearest first, below).
    std::vector<OrderItem> regionItems;
    std::vector<std::size_t> regionOf;
    for (std::size_t g = 0; g < groups.size(); ++g) {
        if (carved[g].cuts.empty()) {
            say(ToolpathIssue::NoRegion, groups[g].sources.front(),
                vecName(groups[g].sources.front()) +
                    ": nothing to carve (too narrow for the tolerance: no part is deeper than the distance tolerance)");
            continue;
        }
        OrderItem item;
        item.path.append(geometry::LineSpan{groups[g].box.min, groups[g].box.max});
        item.path.closed = false;
        item.selectionIndex = groups[g].sources.front();
        item.start = groups[g].box.centre();
        item.end = item.start;
        regionItems.push_back(std::move(item));
        regionOf.push_back(g);
    }
    const std::vector<std::size_t> order = orderCuts(regionItems, params.order, params.startPosition, tol);

    // Flat areas, region by region in machining order: Pocket per region keeps its work caps and
    // cost per region (one call over 5,000 letters would hit Pocket's vertex cap).
    PocketParams pocket;
    pocket.name = params.name;
    pocket.startDepth = 0.0;  // from the start plane; moved down afterwards
    pocket.cutDepth = limit;
    pocket.maintainExactStep = true;
    pocket.lastPassThickness = 0.0;
    pocket.safeZ = params.safeZ + params.startDepth;
    pocket.strategy = params.clearanceStrategy;
    pocket.direction = params.clearanceDirection;
    pocket.rasterAngle = params.clearanceRasterAngle;
    pocket.profilePass = ProfilePass::Last;
    pocket.allowance = multi ? limitRadius : 0.0;
    pocket.rampPlunge = params.rampPlunge;
    pocket.rampDistance = params.rampDistance;
    pocket.tolerance = tol;
    PocketParams walls = pocket;
    walls.allowance = 0.0;
    std::vector<PocketTool> pocketTools;
    if (multi) {
        pocketTools = clearanceTools;
    } else {
        pocketTools.push_back(PocketTool{tool, cuttingData});
    }
    for (std::size_t i = 0; i < clearanceTools.size(); ++i) {
        Toolpath tp;
        tp.name = params.name + " [" + clearanceTools[i].tool.name + "]";
        tp.tool = toolSnapshot(clearanceTools[i].tool,
                               effectiveCuttingData(clearanceTools[i].tool, clearanceTools[i].cuttingData));
        result.clearance.push_back(std::move(tp));
    }
    DiagnosticMerger merged;
    std::vector<std::vector<Move>> selfClear(groups.size());
    double estimate = 0.0;
    for (const std::size_t k : order) {
        const std::size_t g = regionOf[k];
        GroupResult& gr = carved[g];
        if (!gr.flat) {
            continue;
        }
        auto sourceOf = [&](std::optional<std::size_t> local) -> std::optional<std::size_t> {
            if (local && *local < groups[g].boundaries.size()) {
                return boundaries[groups[g].boundaries[*local]].source;
            }
            return local;
        };
        PocketResult pr = pocketToolpath(gr.local, pocketTools, pocket);
        bool refused = false;
        for (ToolpathDiagnostic& d : pr.diagnostics) {
            d.contour = sourceOf(d.contour);
            if (!multi) {
                d.tool.reset();
            }
            d.message = "flat area clearance: " + d.message;
            refused = refused || refusal(d.issue);
            merged.add(std::move(d));
        }
        if (refused) {
            merged.flushInto(result.diagnostics);
            result.clearance.clear();
            return result;
        }
        if (multi) {
            for (std::size_t t = 0; t < clearanceTools.size() && t < pr.toolpaths.size(); ++t) {
                std::vector<Move>& moves = pr.toolpaths[t].moves;
                shiftZ(moves, params.startDepth);
                estimate += static_cast<double>(moves.size());
                result.clearance[t].moves.insert(result.clearance[t].moves.end(), moves.begin(), moves.end());
            }
            // The carving tool cuts the walls of the flat areas at the limit.
            std::vector<ToolpathDiagnostic> wd;
            const std::vector<Contour> rings = pocketCentreRegion(gr.local, 2.0 * limitRadius, walls, &wd);
            for (ToolpathDiagnostic& d : wd) {
                d.contour = sourceOf(d.contour);
                d.message = "flat area walls: " + d.message;
                merged.add(std::move(d));
            }
            for (const Contour& ring : rings) {
                Cut cut;
                cut.closed = true;
                for (const geometry::Span& sp : ring.spans) {
                    cut.points.push_back(CarvePoint{geometry::startPoint(sp), limit});
                }
                if (!cut.points.empty()) {
                    gr.cuts.push_back(std::move(cut));
                }
            }
        } else if (!pr.toolpaths.empty()) {
            selfClear[g] = std::move(pr.toolpaths.front().moves);
            shiftZ(selfClear[g], params.startDepth);
            estimate += static_cast<double>(selfClear[g].size());
        }
        if (!(estimate <= kMaxMoves)) {
            break;
        }
    }
    merged.flushInto(result.diagnostics);

    // Work cap before the carve moves are made.
    for (const GroupResult& gr : carved) {
        for (const Cut& c : gr.cuts) {
            estimate += static_cast<double>(cutMoves(c));
        }
    }
    if (!(estimate <= kMaxMoves)) {
        result.clearance.clear();
        say(ToolpathIssue::WorkLimitExceeded, std::nullopt,
            "the V-carve would have more than " + num(kMaxMoves) + " moves (too many or too detailed vectors)");
        return result;
    }

    // Moves: per region, its own flat-area clearing (without clearance tools), then its cuts
    // nearest first from where the tool is.
    Vec2 pos = params.startPosition;
    for (const std::size_t k : order) {
        const std::size_t g = regionOf[k];
        const GroupResult& gr = carved[g];
        result.toolpath.moves.insert(result.toolpath.moves.end(), selfClear[g].begin(), selfClear[g].end());
        if (!selfClear[g].empty()) {
            const Vec3 at = selfClear[g].back().to;
            pos = Vec2{at.x, at.y};
        }
        std::vector<OrderItem> items(gr.cuts.size());
        for (std::size_t c = 0; c < gr.cuts.size(); ++c) {
            items[c].selectionIndex = c;
            items[c].start = gr.cuts[c].points.front().p;
            items[c].end = gr.cuts[c].closed ? items[c].start : gr.cuts[c].points.back().p;
        }
        for (const std::size_t c : orderCuts(items, OrderOptions{}, pos, tol)) {
            emitCut(gr.cuts[c], params.startDepth, params.safeZ, result.toolpath.moves);
            pos = items[c].end;
        }
    }
    if (result.toolpath.isEmpty()) {
        result.clearance.clear();
        if (result.diagnostics.empty() || groups.empty()) {
            say(ToolpathIssue::NoRegion, std::nullopt, "no closed region to carve");
        }
    }
    return result;
}

}  // namespace carvetto::toolpath
