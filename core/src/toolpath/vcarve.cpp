#include "carvetto/toolpath/vcarve.hpp"

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

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/fill_rule.hpp"
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

std::string paramProblem(const VCarveParams& p) {
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
    if (!vcarveToolAllowed(tool.type)) {
        return label + " is " + toolTypeText(tool.type) + "; V-carving needs a V-bit or a ball nose";
    }
    if (const std::optional<std::string> bad = tooling::validate(tool)) {
        return label + ": " + *bad;
    }
    if (!(vcarveFullWidthDepth(tool) > 0.0) || !std::isfinite(vcarveFullWidthDepth(tool))) {
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

/// Closed vectors whose bounding boxes overlap, transitively: carved together (a letter with
/// its counters; overlapping or touching shapes). Boxes that do not overlap cannot interact
/// under the even-odd rule, so each group's medial axis is that of its part of the region.
struct Group {
    std::vector<std::size_t> contours;  // input indices, ascending
    Box2 box;
};

std::size_t findRoot(std::vector<std::size_t>& parent, std::size_t i) {
    while (parent[i] != i) {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }
    return i;
}

std::vector<Group> groupContours(const std::vector<Contour>& contours, const std::vector<std::size_t>& usable) {
    std::vector<Box2> boxes;
    boxes.reserve(usable.size());
    for (const std::size_t i : usable) {
        boxes.push_back(contours[i].bounds());
    }
    const BoxIndex index(boxes);
    std::vector<std::size_t> parent(usable.size());
    std::iota(parent.begin(), parent.end(), std::size_t{0});
    for (std::size_t k = 0; k < usable.size(); ++k) {
        for (const std::size_t j : index.query(boxes[k])) {
            if (j <= k) {
                continue;
            }
            const std::size_t a = findRoot(parent, k);
            const std::size_t b = findRoot(parent, j);
            if (a != b) {
                parent[std::max(a, b)] = std::min(a, b);
            }
        }
    }
    std::vector<Group> groups;
    std::vector<std::size_t> slot(usable.size(), std::numeric_limits<std::size_t>::max());
    for (std::size_t k = 0; k < usable.size(); ++k) {
        const std::size_t root = findRoot(parent, k);
        if (slot[root] == std::numeric_limits<std::size_t>::max()) {
            slot[root] = groups.size();
            groups.emplace_back();
        }
        Group& g = groups[slot[root]];
        g.contours.push_back(usable[k]);
        g.box.expand(boxes[k]);
    }
    return groups;  // ordered by their first vector (roots are the smallest index)
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
    double depth(double r) const { return std::min(flatDepth, vcarveDepthForRadius(*tool, std::max(0.0, r - margin))); }
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

/// Appends the carve points of the segment a → b (a excluded), Z linear between them. For a
/// V-bit the footprint is linear in depth and the depth linear in the radius (clamping to the
/// limit only makes it shallower), so the footprint along the segment stays at the interpolated
/// radius less the margin. For a ball nose the segment is first cut where the allowed footprint
/// reaches 0 or the limit's footprint (there the depth rule changes form), then each piece in
/// between is refined by `appendBallPiece`.
void appendSegment(const DepthRule& rule, const SkelPoint& a, double da, const SkelPoint& b, double db,
                   std::vector<CarvePoint>& out) {
    if (rule.tool->type == tooling::ToolType::VBit) {
        out.push_back(CarvePoint{b.p, db});
        return;
    }
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
    for (const double t : cuts) {
        const SkelPoint m{a.p + (b.p - a.p) * t, a.r + (b.r - a.r) * t};
        const double dm = rule.depth(m.r);
        appendBallPiece(rule, from, dFrom, m, dm, out, 0);
        from = m;
        dFrom = dm;
    }
    appendBallPiece(rule, from, dFrom, b, db, out, 0);
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
    std::vector<Cut> cuts;
    double maxRadius{0.0};
    bool capped{false};
    std::size_t ignored{0};
    bool flat{false};  // has an area deeper than the depth limit
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

std::vector<Toolpath> VCarveResult::toolpaths() const {
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

bool vcarveToolAllowed(tooling::ToolType type) {
    return type == tooling::ToolType::VBit || type == tooling::ToolType::BallNose;
}

bool vcarveClearanceToolAllowed(tooling::ToolType type) {
    return type == tooling::ToolType::EndMill;
}

double vcarveFullWidthDepth(const tooling::Tool& tool) {
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

double vcarveDepthForRadius(const tooling::Tool& tool, double r) {
    const double full = vcarveFullWidthDepth(tool);
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

VCarveResult vcarveToolpath(const std::vector<Contour>& contours, const tooling::Tool& tool,
                            const std::optional<tooling::CuttingData>& cuttingData,
                            const std::vector<PocketTool>& clearanceTools, const VCarveParams& params) {
    VCarveResult result;
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
        if (!vcarveClearanceToolAllowed(ct.type)) {
            say(ToolpathIssue::InvalidTool, std::nullopt,
                "clearance tool " + std::to_string(i + 1) + " ('" + ct.name + "') is " + toolTypeText(ct.type) +
                    "; clearance tools must be end mills",
                i);
            return result;
        }
    }

    const geometry::Tolerance& tol = params.tolerance;
    const double full = vcarveFullWidthDepth(tool);
    const double limit = params.flatDepth > 0.0 ? std::min(params.flatDepth, full) : full;
    if (params.flatDepth > full) {
        say(ToolpathIssue::CarveDepthLimited, std::nullopt,
            "the flat depth " + num(params.flatDepth) + " mm is deeper than tool '" + tool.name +
                "' reaches at its full width; the flat areas are cut at " + num(full) + " mm");
    }
    const double limitRadius = footprint(tool, limit);
    DepthRule rule;
    rule.tool = &tool;
    rule.margin = 0.5 * tol.chordal;
    rule.flatDepth = limit;
    rule.limitRadius = limitRadius;

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
    const std::vector<Group> groups = groupContours(contours, usable);
    result.regions = groups.size();

    // Medial axis and cuts per group.
    std::vector<GroupResult> carved(groups.size());
    std::size_t ignored = 0;
    std::optional<std::size_t> firstIgnored;
    bool anyCapped = false;
    for (std::size_t g = 0; g < groups.size(); ++g) {
        std::vector<Contour> local;
        local.reserve(groups[g].contours.size());
        for (const std::size_t i : groups[g].contours) {
            local.push_back(contours[i]);
        }
        const MedialAxis ax = geometry::medialAxis(local, tol);
        GroupResult& gr = carved[g];
        gr.capped = ax.accuracyCapped;
        gr.ignored = ax.ignoredContours;
        if (ax.ignoredContours > 0) {
            ignored += ax.ignoredContours;
            if (!firstIgnored) {
                firstIgnored = groups[g].contours.front();
            }
        }
        if (gr.capped) {
            anyCapped = true;
            std::string names;
            for (const std::size_t i : groups[g].contours) {
                names += names.empty() ? "" : ", ";
                names += std::to_string(i);
            }
            say(ToolpathIssue::MedialAxisCapped, groups[g].contours.front(),
                "the medial axis of vector(s) " + names +
                    " could not be computed within the tolerance (too detailed for its budget, or too far from "
                    "the origin); nothing is carved: use a coarser chordal tolerance or simplify the vectors");
            continue;
        }
        if (anyCapped) {
            continue;  // nothing will be produced; only report the other capped groups
        }
        for (const Trail& walk : skeletonTrails(ax)) {
            for (const SkelPoint& s : walk) {
                gr.maxRadius = std::max(gr.maxRadius, s.r);
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
        gr.flat = gr.maxRadius - rule.margin > limitRadius;
    }
    if (anyCapped) {
        return result;
    }
    if (ignored > 0) {
        say(ToolpathIssue::NoRegion, firstIgnored,
            std::to_string(ignored) +
                " closed vector(s) enclose no area, or are cancelled by a coincident vector (even-odd); ignored");
    }

    // Flat areas.
    std::vector<std::size_t> flatContours;  // input indices of the groups with a flat area
    std::size_t flatGroups = 0;
    for (std::size_t g = 0; g < groups.size(); ++g) {
        if (carved[g].flat) {
            ++flatGroups;
            flatContours.insert(flatContours.end(), groups[g].contours.begin(), groups[g].contours.end());
        }
    }
    std::sort(flatContours.begin(), flatContours.end());
    if (flatGroups > 0 && !(params.flatDepth > 0.0)) {
        say(ToolpathIssue::CarveDepthLimited, flatContours.front(),
            std::to_string(flatGroups) + " region(s) are wider than tool '" + tool.name + "' (" + num(tool.diameter) +
                " mm) can carve: carved to its full depth of " + num(full) + " mm, the flat bottom cleared");
    }
    std::vector<Move> selfClear;
    if (flatGroups > 0) {
        std::vector<Contour> flatInput;
        flatInput.reserve(flatContours.size());
        for (const std::size_t i : flatContours) {
            flatInput.push_back(contours[i]);
        }
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
        std::vector<PocketTool> tools;
        if (multi) {
            tools = clearanceTools;
        } else {
            tools.push_back(PocketTool{tool, cuttingData});
        }
        PocketResult pr = pocketToolpath(flatInput, tools, pocket);
        bool refused = false;
        for (ToolpathDiagnostic& d : pr.diagnostics) {
            if (d.contour && *d.contour < flatContours.size()) {
                d.contour = flatContours[*d.contour];
            }
            if (!multi) {
                d.tool.reset();
            }
            d.message = "flat area clearance: " + d.message;
            refused = refused || refusal(d.issue);
            result.diagnostics.push_back(std::move(d));
        }
        if (refused) {
            return result;
        }
        if (multi) {
            for (std::size_t i = 0; i < clearanceTools.size(); ++i) {
                Toolpath tp;
                if (i < pr.toolpaths.size()) {
                    tp = std::move(pr.toolpaths[i]);
                }
                tp.name = params.name + " [" + clearanceTools[i].tool.name + "]";
                tp.tool = toolSnapshot(clearanceTools[i].tool,
                                       effectiveCuttingData(clearanceTools[i].tool, clearanceTools[i].cuttingData));
                shiftZ(tp.moves, params.startDepth);
                result.clearance.push_back(std::move(tp));
            }
            // The carving tool cuts the walls of the flat areas at the limit.
            PocketParams walls = pocket;
            walls.allowance = 0.0;
            for (std::size_t g = 0; g < groups.size(); ++g) {
                if (!carved[g].flat) {
                    continue;
                }
                std::vector<Contour> local;
                for (const std::size_t i : groups[g].contours) {
                    local.push_back(contours[i]);
                }
                std::vector<ToolpathDiagnostic> wd;
                const std::vector<Contour> rings = pocketCentreRegion(local, 2.0 * limitRadius, walls, &wd);
                for (ToolpathDiagnostic& d : wd) {
                    if (d.contour && *d.contour < groups[g].contours.size()) {
                        d.contour = groups[g].contours[*d.contour];
                    }
                    d.message = "flat area walls: " + d.message;
                    result.diagnostics.push_back(std::move(d));
                }
                for (const Contour& ring : rings) {
                    Cut cut;
                    cut.closed = true;
                    for (const geometry::Span& s : ring.spans) {
                        cut.points.push_back(CarvePoint{geometry::startPoint(s), limit});
                    }
                    if (!cut.points.empty()) {
                        carved[g].cuts.push_back(std::move(cut));
                    }
                }
            }
        } else if (!pr.toolpaths.empty()) {
            selfClear = std::move(pr.toolpaths.front().moves);
            shiftZ(selfClear, params.startDepth);
        }
    } else {
        for (std::size_t i = 0; i < clearanceTools.size(); ++i) {
            Toolpath tp;
            tp.name = params.name + " [" + clearanceTools[i].tool.name + "]";
            tp.tool = toolSnapshot(clearanceTools[i].tool,
                                   effectiveCuttingData(clearanceTools[i].tool, clearanceTools[i].cuttingData));
            result.clearance.push_back(std::move(tp));
        }
    }

    // Work cap before any carve moves are made.
    double estimate = static_cast<double>(selfClear.size());
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

    // Order: regions by the chosen strategy, then the cuts of each region nearest first.
    std::vector<OrderItem> regionItems;
    std::vector<std::size_t> regionOf;
    for (std::size_t g = 0; g < groups.size(); ++g) {
        if (carved[g].cuts.empty()) {
            say(ToolpathIssue::NoRegion, groups[g].contours.front(),
                vecName(groups[g].contours.front()) +
                    ": nothing to carve (too narrow for the tolerance: no part is deeper than the distance tolerance)");
            continue;
        }
        OrderItem item;
        item.path.append(geometry::LineSpan{groups[g].box.min, groups[g].box.max});
        item.path.closed = false;
        item.selectionIndex = groups[g].contours.front();
        item.start = groups[g].box.centre();
        item.end = item.start;
        regionItems.push_back(std::move(item));
        regionOf.push_back(g);
    }
    std::vector<Move> carve;
    Vec2 pos = params.startPosition;
    for (const std::size_t k : orderCuts(regionItems, params.order, params.startPosition, tol)) {
        const GroupResult& gr = carved[regionOf[k]];
        std::vector<OrderItem> items(gr.cuts.size());
        for (std::size_t c = 0; c < gr.cuts.size(); ++c) {
            items[c].selectionIndex = c;
            items[c].start = gr.cuts[c].points.front().p;
            items[c].end = gr.cuts[c].closed ? items[c].start : gr.cuts[c].points.back().p;
        }
        for (const std::size_t c : orderCuts(items, OrderOptions{}, pos, tol)) {
            emitCut(gr.cuts[c], params.startDepth, params.safeZ, carve);
            pos = items[c].end;
        }
    }
    result.toolpath.moves = std::move(selfClear);
    result.toolpath.moves.insert(result.toolpath.moves.end(), carve.begin(), carve.end());
    if (result.toolpath.isEmpty()) {
        result.clearance.clear();
        if (result.diagnostics.empty() || groups.empty()) {
            say(ToolpathIssue::NoRegion, std::nullopt, "no closed region to carve");
        }
    }
    return result;
}

}  // namespace carvetto::toolpath
