#include "carvetto/toolpath/profile.hpp"

#include <algorithm>
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
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/toolpath/clearance.hpp"
#include "carvetto/toolpath/path_walk.hpp"

namespace carvetto::toolpath {

using geometry::Contour;
using geometry::Orientation;
using geometry::Span;
using geometry::Tolerance;
using geometry::Vec2;

bool ProfileResult::has(ProfileIssue issue) const {
    return std::any_of(diagnostics.begin(), diagnostics.end(),
                       [&](const ProfileDiagnostic& d) { return d.issue == issue; });
}

double profileToolRadius(const tooling::Tool& tool, const PassPlan& passes) {
    return 0.5 * tooling::cuttingWidthAtDepth(tool, passes.startDepth + passes.cutDepth);
}

namespace {

// One cut: a path machined through its levels, with what planning decided about it.
struct Cut {
    Contour path;
    std::vector<std::size_t> sources;  // input vectors it was offset from
    bool contourOnLeft{true};          // the vector lies on the left of the direction of travel
    bool fromClosed{true};
    bool exact{true};      // exact offset of its source (trusted against it); false = merged region
    double distance{0.0};  // |offset| from the vectors
    ArcLengthPath walk;
    std::vector<double> tabs;
};

struct Input {
    std::vector<Contour> contours;  // closed ones oriented by nesting parity
    std::vector<bool> valid;
    std::vector<int> depth;
    std::vector<double> area;
};

class DisjointSets {
   public:
    explicit DisjointSets(std::size_t n) : parent_(n) { std::iota(parent_.begin(), parent_.end(), std::size_t{0}); }
    std::size_t find(std::size_t x) {
        while (parent_[x] != x) {
            parent_[x] = parent_[parent_[x]];
            x = parent_[x];
        }
        return x;
    }
    void unite(std::size_t a, std::size_t b) {
        a = find(a);
        b = find(b);
        if (a != b) {
            parent_[std::max(a, b)] = std::min(a, b);
        }
    }

   private:
    std::vector<std::size_t> parent_;
};

Vec2 probePoint(const Contour& c) {
    return geometry::pointAt(c.spans.front(), 0.5);
}

bool hasBezier(const std::vector<Contour>& cs) {
    for (const Contour& c : cs) {
        for (const Span& s : c.spans) {
            if (std::holds_alternative<geometry::BezierSpan>(s)) {
                return true;
            }
        }
    }
    return false;
}

std::vector<int> nestingDepths(const std::vector<Contour>& cs, const std::vector<bool>& use, const Tolerance& tol) {
    const std::size_t n = cs.size();
    std::vector<int> depth(n, 0);
    std::vector<geometry::Box2> boxes(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (use[i]) {
            boxes[i] = cs[i].bounds();
        }
    }
    for (std::size_t i = 0; i < n; ++i) {
        if (!use[i]) {
            continue;
        }
        const Vec2 p = probePoint(cs[i]);
        for (std::size_t j = 0; j < n; ++j) {
            if (j == i || !use[j] || !boxes[j].contains(p)) {
                continue;
            }
            if (geometry::locatePoint(cs[j], p, geometry::FillRule::EvenOdd, tol) == geometry::PointLocation::Inside) {
                ++depth[i];
            }
        }
    }
    return depth;
}

Vec2 anchorOf(const geometry::Box2& b, BoxAnchor a) {
    const double xs[3] = {b.min.x, 0.5 * (b.min.x + b.max.x), b.max.x};
    const double ys[3] = {b.min.y, 0.5 * (b.min.y + b.max.y), b.max.y};
    const int k = static_cast<int>(a);
    return {xs[k % 3], ys[k / 3]};
}

std::string vecName(std::size_t i) {
    return "vector " + std::to_string(i);
}

class ProfileBuilder {
   public:
    ProfileBuilder(const std::vector<Contour>& contours, const tooling::Tool& tool, const ProfileParams& params)
        : raw_(contours), tool_(tool), p_(params), tol_(params.tolerance) {}

    ProfileResult run();

   private:
    void diag(ProfileIssue issue, std::optional<std::size_t> contour, std::string message) {
        result_.diagnostics.push_back({issue, contour, std::move(message)});
    }
    bool validate();
    void prepareInputs();
    std::vector<Cut> buildCuts(double extra, bool finishing);
    void orient(Cut& c, bool finishing) const;
    std::vector<Cut> trim(std::vector<Cut> cuts);
    void chooseStarts(std::vector<Cut>& cuts);
    std::vector<std::size_t> order(const std::vector<Cut>& cuts) const;
    void placeTabsOn(std::vector<Cut>& primary);
    void projectTabs(const std::vector<Cut>& from, std::vector<Cut>& to) const;
    void emit(const Cut& cut, const std::vector<double>& levels, bool finishing);
    void emitLeg(const Cut& cut, const PathLeg& leg, bool lastLevel, std::vector<bool>& cornerDone);
    template <class Make>
    Contour fitLead(Make make, std::size_t contour, const char* what);

    const std::vector<Contour>& raw_;
    const tooling::Tool& tool_;
    ProfileParams p_;
    Tolerance tol_;
    Tolerance offTol_;
    tooling::CuttingData cd_;
    PassPlan passes_;
    double radius_{0.0};
    bool vbit_{false};
    bool sharpExternal_{false};
    bool sharpInternal_{false};
    Input in_;
    ClearanceField allField_;
    ProfileResult result_;
    std::vector<bool> tooLargeReported_;
};

bool ProfileBuilder::validate() {
    cd_ = p_.cuttingData.value_or(tool_.defaultCuttingData);
    passes_ = p_.passes;
    if (!(passes_.passDepth > 0.0)) {
        passes_.passDepth = cd_.passDepth;
    }
    radius_ = profileToolRadius(tool_, passes_);
    const double vals[] = {p_.allowance,       p_.safeZ,           passes_.startDepth, passes_.cutDepth,
                           passes_.passDepth,  p_.lastPassAllowance, p_.tabs.length,   p_.tabs.thickness,
                           p_.tabs.distance,   p_.ramp.distance,   p_.ramp.angle,      p_.leads.length,
                           p_.leads.angle,     p_.leads.radius,    p_.leads.sweep,     p_.leads.overcut,
                           p_.corners.mitreLimit, radius_};
    for (double v : vals) {
        if (!std::isfinite(v)) {
            diag(ProfileIssue::InvalidParameters, std::nullopt, "a parameter is not a finite number");
            return false;
        }
    }
    if (!(radius_ > 0.0)) {
        diag(ProfileIssue::InvalidParameters, std::nullopt, "the tool has no cutting width at the cut depth");
        return false;
    }
    if (passes_.startDepth < 0.0 || p_.safeZ <= -passes_.startDepth) {
        diag(ProfileIssue::InvalidParameters, std::nullopt, "safe Z must be above the start of the cut");
        return false;
    }
    if (p_.separateLastPass && p_.lastPassAllowance < 0.0) {
        diag(ProfileIssue::InvalidParameters, std::nullopt, "last pass allowance must not be negative");
        return false;
    }
    if (passes_.levels().empty()) {
        diag(ProfileIssue::NoPasses, std::nullopt, "the cut depth yields no passes");
        return false;
    }
    vbit_ = tool_.type == tooling::ToolType::VBit;
    if ((p_.corners.sharpExternal || p_.corners.sharpInternal) && !vbit_) {
        diag(ProfileIssue::CornerNeedsVBit, std::nullopt, "sharp corner options need a V-bit; ignored");
    }
    sharpExternal_ = vbit_ && p_.corners.sharpExternal;
    sharpInternal_ = vbit_ && p_.corners.sharpInternal;
    return true;
}

void ProfileBuilder::prepareInputs() {
    const std::size_t n = raw_.size();
    in_.contours = raw_;
    in_.valid.assign(n, false);
    in_.area.assign(n, 0.0);
    tooLargeReported_.assign(n, false);
    std::vector<bool> closedValid(n, false);
    for (std::size_t i = 0; i < n; ++i) {
        const Contour& c = raw_[i];
        if (c.isEmpty()) {
            diag(ProfileIssue::EmptyContour, i, vecName(i) + " has no spans; skipped");
            continue;
        }
        if (!c.isContinuous(tol_)) {
            diag(ProfileIssue::DiscontinuousContour, i, vecName(i) + " has gaps between spans; skipped");
            continue;
        }
        in_.valid[i] = true;
        closedValid[i] = c.closed;
        if (c.closed) {
            in_.area[i] = geometry::area(c);
        }
    }
    in_.depth = nestingDepths(in_.contours, closedValid, tol_);
    for (std::size_t i = 0; i < n; ++i) {
        if (closedValid[i]) {
            geometry::setOrientation(in_.contours[i], in_.depth[i] % 2 == 0 ? Orientation::CCW : Orientation::CW, tol_);
        }
    }
    offTol_ = tol_;
    if (hasBezier(raw_)) {
        // Bezier spans are polygonised before offsetting; keep the chords within half the
        // distance tolerance of the curve so the offset stays within tolerance of r.
        offTol_.chordal = std::min(tol_.chordal, 0.5 * tol_.distance);
    }
    std::vector<Contour> valid;
    for (std::size_t i = 0; i < n; ++i) {
        if (in_.valid[i]) {
            valid.push_back(in_.contours[i]);
        }
    }
    allField_ = ClearanceField(std::move(valid));
}

// Signed outward distance for a pass with `extra` added (last-pass allowance for roughing).
std::vector<Cut> ProfileBuilder::buildCuts(double extra, bool finishing) {
    std::vector<Cut> cuts;
    const std::size_t n = in_.contours.size();
    double s = 0.0;
    switch (p_.side) {
        case ProfileSide::Outside:
            s = radius_ + p_.allowance + extra;
            break;
        case ProfileSide::Inside:
            s = -(radius_ + p_.allowance + extra);
            break;
        case ProfileSide::On:
            s = p_.allowance + extra;
            break;
    }
    const double d = std::fabs(s);
    const geometry::OffsetSide side = s >= 0.0 ? geometry::OffsetSide::Right : geometry::OffsetSide::Left;
    const geometry::JoinType join = sharpExternal_ ? geometry::JoinType::Sharp : geometry::JoinType::Round;
    const bool guard = p_.side != ProfileSide::On && d > 0.0;

    std::vector<std::vector<Contour>> chains(n);
    std::vector<bool> selfBad(n, false);
    for (std::size_t i = 0; i < n; ++i) {
        if (!in_.valid[i]) {
            continue;
        }
        const Contour& c = in_.contours[i];
        if (d == 0.0) {
            chains[i] = {ArcLengthPath(c, offTol_).contour()};
        } else {
            chains[i] = geometry::offsetOpenContour(c, d, side, join, offTol_, p_.corners.mitreLimit);
        }
        for (Contour& ch : chains[i]) {
            if (!ch.closed && !ch.isEmpty() && geometry::distance(ch.startPoint(), ch.endPoint()) <= tol_.distance) {
                ch.closed = true;
            }
            if (c.closed && !ch.closed) {
                selfBad[i] = true;
            }
        }
        if (chains[i].empty() && !tooLargeReported_[i]) {
            tooLargeReported_[i] = true;
            diag(ProfileIssue::ToolTooLarge, i,
                 vecName(i) + ": no room for the tool (offset by " + std::to_string(d) + " mm vanishes)");
        }
    }

    // Closed vectors whose offsets come too close to another closed vector are offset together.
    DisjointSets sets(n);
    std::vector<bool> merge(n, false);
    if (guard) {
        for (std::size_t i = 0; i < n; ++i) {
            if (!in_.valid[i] || !in_.contours[i].closed) {
                continue;
            }
            if (selfBad[i]) {
                merge[i] = true;
            }
            for (const Contour& ch : chains[i]) {
                const ArcLengthPath walk(ch, offTol_);
                const geometry::Box2 cb = ch.bounds().inflated(d);
                for (std::size_t j = 0; j < n; ++j) {
                    if (j == i || !in_.valid[j] || !in_.contours[j].closed || sets.find(i) == sets.find(j)) {
                        continue;
                    }
                    if (!cb.intersects(in_.contours[j].bounds())) {
                        continue;
                    }
                    const ClearanceField fj({in_.contours[j]});
                    if (!isClear(walk, fj, d - tol_.distance, 0.5 * tol_.distance)) {
                        sets.unite(i, j);
                        merge[i] = merge[j] = true;
                    }
                }
            }
        }
    }

    for (std::size_t i = 0; i < n; ++i) {
        if (!in_.valid[i] || merge[i]) {
            continue;
        }
        for (Contour& ch : chains[i]) {
            Cut cut;
            cut.path = std::move(ch);
            cut.sources = {i};
            cut.contourOnLeft = s >= 0.0;
            cut.fromClosed = in_.contours[i].closed;
            cut.distance = d;
            cuts.push_back(std::move(cut));
        }
    }

    // Region offsets of merged groups (polygonal). The extra chordal margin keeps the result at
    // least d from the true curves, which bulge up to `chordal` beyond their polygonisation.
    for (std::size_t root = 0; root < n; ++root) {
        std::vector<std::size_t> group;
        for (std::size_t i = 0; i < n; ++i) {
            if (merge[i] && sets.find(i) == root) {
                group.push_back(i);
            }
        }
        if (group.empty()) {
            continue;
        }
        // The group's region under the even-odd rule: every edge of every member is a region
        // boundary, also where members cross. Outers CCW, holes CW (region on the left).
        std::vector<Contour> members;
        std::size_t outer = group.front();
        for (std::size_t i : group) {
            members.push_back(in_.contours[i]);
            if (in_.area[i] > in_.area[outer]) {
                outer = i;
            }
        }
        const std::vector<Contour> region = geometry::unionContours(members, offTol_, geometry::FillRule::EvenOdd);
        // Nesting parity of the group as a whole: how many other closed vectors contain it. When
        // odd, the group sits in a hole and the global region is the complement of its own.
        int base = 0;
        {
            const Vec2 probe = probePoint(in_.contours[outer]);
            for (std::size_t j = 0; j < n; ++j) {
                if (!in_.valid[j] || !in_.contours[j].closed || std::find(group.begin(), group.end(), j) != group.end()) {
                    continue;
                }
                if (geometry::locatePoint(in_.contours[j], probe, geometry::FillRule::EvenOdd, tol_) ==
                    geometry::PointLocation::Inside) {
                    base ^= 1;
                }
            }
        }
        double dd = (s >= 0.0 ? 1.0 : -1.0) * (d + offTol_.chordal);
        if (base == 1) {
            dd = -dd;
        }
        std::vector<Contour> res = geometry::offsetContours(region, dd, join, offTol_, p_.corners.mitreLimit);
        for (Contour& r : res) {
            if (base == 1) {
                r.reverse();  // keep the global region on the left
            }
            Cut cut;
            cut.path = std::move(r);
            cut.sources = group;
            cut.contourOnLeft = s >= 0.0;
            cut.fromClosed = true;
            cut.exact = false;
            cut.distance = d;
            cuts.push_back(std::move(cut));
        }
        for (std::size_t i : group) {
            diag(ProfileIssue::MergedWithNeighbour, i,
                 vecName(i) + (group.size() > 1 ? ": cut merged with nearby vectors (offset as one region, polygonal)"
                                                : ": offset as a region (polygonal); its exact offset was not closed"));
        }
        if (res.empty()) {
            for (std::size_t i : group) {
                if (!tooLargeReported_[i]) {
                    tooLargeReported_[i] = true;
                    diag(ProfileIssue::ToolTooLarge, i, vecName(i) + ": no room for the tool");
                }
            }
        }
    }
    for (Cut& c : cuts) {
        orient(c, finishing);
    }
    if (guard) {
        cuts = trim(std::move(cuts));
    }
    return cuts;
}

void ProfileBuilder::orient(Cut& c, bool finishing) const {
    const bool climb = p_.direction == CutDirection::Climb;
    // Climb keeps the vector on the right of travel.
    bool reverse = c.contourOnLeft == climb;
    if (p_.side == ProfileSide::On && !c.fromClosed) {
        reverse = !climb;  // open vectors cut "on" keep their direction when climbing
    }
    if (finishing && p_.lastPassReverse) {
        reverse = !reverse;
    }
    if (reverse) {
        c.path.reverse();
        c.contourOnLeft = !c.contourOnLeft;
    }
}

std::vector<Cut> ProfileBuilder::trim(std::vector<Cut> cuts) {
    std::vector<Cut> out;
    for (Cut& cut : cuts) {
        std::vector<Contour> others;
        for (std::size_t j = 0; j < in_.contours.size(); ++j) {
            // An exact offset is at least its distance from its own vector by construction; a
            // merged region offset is checked against every vector.
            const bool own = std::find(cut.sources.begin(), cut.sources.end(), j) != cut.sources.end();
            if (in_.valid[j] && (!own || !cut.exact)) {
                others.push_back(in_.contours[j]);
            }
        }
        const ClearanceField field(std::move(others));
        const ArcLengthPath walk(cut.path, offTol_);
        const double len = walk.length();
        const auto bad = findViolations(walk, 0.0, len, field, cut.distance - tol_.distance, 0.5 * tol_.distance);
        if (bad.empty()) {
            out.push_back(std::move(cut));
            continue;
        }
        for (std::size_t s : cut.sources) {
            diag(ProfileIssue::TrimmedNearVector, s,
                 vecName(s) + ": part of the cut was left out because it passes within the tool radius of "
                              "another vector");
        }
        // Keep the complement of the violating stretches.
        std::vector<std::pair<double, double>> keep;
        if (walk.closed()) {
            for (std::size_t k = 0; k < bad.size(); ++k) {
                const double a = bad[k].second;
                const double b = k + 1 < bad.size() ? bad[k + 1].first : bad.front().first + len;
                keep.push_back({a, b});
            }
        } else {
            double a = 0.0;
            for (const auto& v : bad) {
                keep.push_back({a, v.first});
                a = v.second;
            }
            keep.push_back({a, len});
        }
        for (const auto& [a, b] : keep) {
            if (b - a <= tol_.distance) {
                continue;
            }
            Cut piece;
            piece.path.spans = walk.extract(a, b);
            piece.path.closed = false;
            piece.sources = cut.sources;
            piece.contourOnLeft = cut.contourOnLeft;
            piece.fromClosed = cut.fromClosed;
            piece.exact = cut.exact;
            piece.distance = cut.distance;
            if (!piece.path.isEmpty()) {
                out.push_back(std::move(piece));
            }
        }
    }
    return out;
}

void ProfileBuilder::chooseStarts(std::vector<Cut>& cuts) {
    // Slivers no longer than the tolerance carry no cut.
    cuts.erase(std::remove_if(cuts.begin(), cuts.end(),
                              [&](const Cut& c) { return c.path.isEmpty() || c.path.length() <= tol_.distance; }),
               cuts.end());
    for (Cut& c : cuts) {
        const ArcLengthPath walk(c.path, offTol_);
        if (walk.closed() && p_.startAt != StartAt::Optimise) {
            Vec2 target = walk.pointAt(0.0);
            if (p_.startAt == StartAt::VectorStart) {
                target = raw_[c.sources.front()].startPoint();
            } else {
                target = anchorOf(c.path.bounds(), p_.startAnchor);
            }
            const double u = walk.project(target);
            c.path.spans = walk.extract(u, u + walk.length());
        }
        c.walk = ArcLengthPath(c.path, offTol_);
    }
}

std::vector<std::size_t> ProfileBuilder::order(const std::vector<Cut>& cuts) const {
    std::vector<OrderItem> items;
    items.reserve(cuts.size());
    for (const Cut& c : cuts) {
        OrderItem it;
        it.path = c.walk.contour();
        it.selectionIndex = *std::min_element(c.sources.begin(), c.sources.end());
        it.start = c.walk.pointAt(0.0);
        it.end = c.walk.closed() ? it.start : c.walk.pointAt(c.walk.length());
        items.push_back(std::move(it));
    }
    return orderCuts(items, p_.order, p_.startPosition, tol_);
}

void ProfileBuilder::placeTabsOn(std::vector<Cut>& primary) {
    if (!p_.tabs.enabled) {
        return;
    }
    const double raised = p_.tabs.length + 2.0 * radius_;
    for (Cut& c : primary) {
        const TabPlacement tp = placeTabs(c.walk, raised, p_.tabs);
        c.tabs = tp.centres;
        if (tp.notAvoided > 0 || tp.dropped > 0) {
            diag(ProfileIssue::TabsNotPlaced, c.sources.front(),
                 vecName(c.sources.front()) + ": " + std::to_string(tp.dropped) + " tab(s) dropped, " +
                     std::to_string(tp.notAvoided) + " left on a corner or curve");
        }
    }
}

void ProfileBuilder::projectTabs(const std::vector<Cut>& from, std::vector<Cut>& to) const {
    const double reach = p_.lastPassAllowance + 2.0 * tol_.distance + 1e-9;
    for (Cut& c : to) {
        for (const Cut& f : from) {
            for (double u : f.tabs) {
                const Vec2 p = f.walk.pointAt(u);
                const double v = c.walk.project(p);
                if (geometry::distance(c.walk.pointAt(v), p) <= reach) {
                    c.tabs.push_back(v);
                }
            }
        }
        std::sort(c.tabs.begin(), c.tabs.end());
        c.tabs.erase(std::unique(c.tabs.begin(), c.tabs.end()), c.tabs.end());
    }
}

// The lead `make(scale)` at full size, or halved up to three times until it stays at least the
// tool radius from every vector; empty (with a diagnostic) when even the smallest gouges.
template <class Make>
Contour ProfileBuilder::fitLead(Make make, std::size_t contour, const char* what) {
    for (double scale : {1.0, 0.5, 0.25, 0.125}) {
        Contour cand = make(scale);
        if (cand.isEmpty()) {
            return cand;
        }
        if (p_.side == ProfileSide::On ||
            isClear(ArcLengthPath(cand, offTol_), allField_, radius_ - tol_.distance, 0.5 * tol_.distance)) {
            if (scale < 1.0) {
                diag(ProfileIssue::LeadShortened, contour,
                     vecName(contour) + ": " + what + " shortened to keep clear of the vectors");
            }
            return cand;
        }
    }
    diag(ProfileIssue::LeadDropped, contour, vecName(contour) + ": " + what + " left out (it would gouge a vector)");
    return {};
}

void ProfileBuilder::emitLeg(const Cut& cut, const PathLeg& leg, bool lastLevel, std::vector<bool>& cornerDone) {
    const double bottom = passes_.bottomZ();
    ZProfile z = leg.z;
    if (!cut.tabs.empty()) {
        const double zTab = bottom + p_.tabs.thickness;
        z = ZProfile::max(z, tabCeiling(cut.walk, leg.u0, leg.u1, cut.tabs, p_.tabs.length + 2.0 * radius_, zTab,
                                        bottom, p_.tabs.threeD));
    }
    std::vector<LegInsert> inserts;
    if (lastLevel && sharpInternal_) {
        const double len = cut.walk.length();
        const double travel = std::fabs(leg.u1 - leg.u0);
        const double dir = leg.u1 >= leg.u0 ? 1.0 : -1.0;
        const auto kinks = cut.walk.kinks(1e-3);
        for (std::size_t k = 0; k < kinks.size(); ++k) {
            const bool inside = cut.contourOnLeft ? kinks[k].turn < 0.0 : kinks[k].turn > 0.0;
            if (!inside || cornerDone[k]) {
                continue;
            }
            std::vector<double> lambdas;
            if (cut.walk.closed()) {
                for (double m = std::floor((std::min(leg.u0, leg.u1) - kinks[k].u) / len) - 1.0;
                     kinks[k].u + m * len <= std::max(leg.u0, leg.u1) + 1e-9; m += 1.0) {
                    lambdas.push_back((kinks[k].u + m * len - leg.u0) * dir);
                }
            } else {
                lambdas.push_back((kinks[k].u - leg.u0) * dir);
            }
            for (double lam : lambdas) {
                if (lam < -1e-9 || lam > travel + 1e-9 || std::fabs(z.after(lam) - bottom) > 1e-9) {
                    continue;
                }
                // Target: the vector's corner (the span junction nearest the path's corner).
                const Vec2 q = cut.walk.pointAt(kinks[k].u);
                Vec2 v = q;
                double best = std::numeric_limits<double>::infinity();
                for (std::size_t s : cut.sources) {
                    for (const Span& sp : in_.contours[s].spans) {
                        const Vec2 c = geometry::startPoint(sp);
                        if (geometry::distance(c, q) < best) {
                            best = geometry::distance(c, q);
                            v = c;
                        }
                    }
                }
                inserts.push_back({lam, {Move::feed(Vec3{v, 0.0 - passes_.startDepth}), Move::feed(Vec3{q, bottom})}});
                cornerDone[k] = true;
                break;
            }
        }
    }
    appendLeg(cut.walk, leg.u0, leg.u1, z, result_.toolpath, inserts);
}

void ProfileBuilder::emit(const Cut& cut, const std::vector<double>& levels, bool finishing) {
    Toolpath& out = result_.toolpath;
    const ArcLengthPath& walk = cut.walk;
    if (walk.isEmpty() || levels.empty()) {
        return;
    }
    const double len = walk.length();
    const double z0 = 0.0 - passes_.startDepth;  // top of the cut (never -0)
    const double zn = levels.back();
    RampOptions ramp = finishing ? RampOptions{} : p_.ramp;
    if (ramp.type == RampType::Spiral && !walk.closed()) {
        diag(ProfileIssue::SpiralNeedsClosed, cut.sources.front(),
             vecName(cut.sources.front()) + ": spiral ramp needs a closed path; smooth ramp used");
        ramp.type = RampType::Smooth;
    }
    const bool rampFirst = ramp.type != RampType::None && levels.front() < z0;
    const std::size_t src = cut.sources.front();

    // Lead-in, checked against every vector.
    Contour in;
    if (p_.leads.type != LeadType::None) {
        const Vec2 at = walk.pointAt(0.0);
        const Vec2 t = walk.tangentAt(0.0);
        const bool wasteLeft = !cut.contourOnLeft;
        in = fitLead([&](double scale) { return leadIn(at, t, wasteLeft, p_.leads, scale); }, src, "lead-in");
    }
    const Vec2 entry = in.isEmpty() ? walk.pointAt(0.0) : in.startPoint();
    out.append(Move::rapid(Vec3{entry, p_.safeZ}));
    double zFrom = levels.front();
    if (!in.isEmpty()) {
        const ArcLengthPath lw(in, offTol_);
        if (rampFirst && ramp.onLeadIn) {
            out.append(Move::plunge(Vec3{entry, z0}));
            appendLeg(lw, 0.0, lw.length(), ZProfile::linear(z0, levels.front(), lw.length()), out);
        } else if (rampFirst) {
            out.append(Move::plunge(Vec3{entry, z0}));
            appendLeg(lw, 0.0, lw.length(), ZProfile::flat(z0), out);
            zFrom = z0;
        } else {
            out.append(Move::plunge(Vec3{entry, levels.front()}));
            appendLeg(lw, 0.0, lw.length(), ZProfile::flat(levels.front()), out);
        }
    } else if (rampFirst) {
        out.append(Move::plunge(Vec3{entry, z0}));
        zFrom = z0;
    } else {
        out.append(Move::plunge(Vec3{entry, levels.front()}));
    }

    std::vector<bool> cornerDone(walk.kinks(1e-3).size(), false);
    double u = 0.0;
    for (std::size_t k = 0; k < levels.size(); ++k) {
        const bool last = k + 1 == levels.size();
        const PassLegs plan = planPass(walk, u, zFrom, levels[k], ramp);
        for (const PathLeg& leg : plan.legs) {
            emitLeg(cut, leg, last, cornerDone);
        }
        u = plan.endU;
        zFrom = levels[k];
    }
    if (walk.closed() && ramp.type == RampType::Spiral && levels.front() < z0) {
        emitLeg(cut, PathLeg{u, u + len, ZProfile::flat(zn)}, true, cornerDone);
    }
    if (walk.closed() && p_.leads.overcut > 0.0) {
        emitLeg(cut, PathLeg{u, u + p_.leads.overcut, ZProfile::flat(zn)}, true, cornerDone);
        u += p_.leads.overcut;
    }

    // Lead-out from where the cut ends, in the direction of travel there.
    if (p_.leads.type != LeadType::None && p_.leads.leadOut) {
        Vec2 at = walk.pointAt(u);
        Vec2 t = walk.tangentAt(u);
        bool wasteLeft = !cut.contourOnLeft;
        if (!walk.closed()) {
            const bool forward = u >= 0.5 * len;  // ended at the far end: travelled forwards
            at = walk.pointAt(forward ? len : 0.0);
            t = forward ? walk.tangentAt(len) : -walk.tangentAt(0.0);
            wasteLeft = forward ? !cut.contourOnLeft : cut.contourOnLeft;
        }
        const Contour lo =
            fitLead([&](double scale) { return leadOut(at, t, wasteLeft, p_.leads, scale); }, src, "lead-out");
        if (!lo.isEmpty()) {
            const ArcLengthPath lw(lo, offTol_);
            appendLeg(lw, 0.0, lw.length(), ZProfile::flat(zn), out);
        }
    }
    out.append(Move::retract(Vec3{out.moves.back().to.xy(), p_.safeZ}));
}

ProfileResult ProfileBuilder::run() {
    result_.toolpath.name = p_.name;
    if (!validate()) {
        return std::move(result_);
    }
    ToolSnapshot& snap = result_.toolpath.tool;
    snap.name = tool_.name;
    snap.toolNumber = cd_.toolNumber;
    snap.diameter = tool_.diameter;
    snap.feedRate = cd_.feedRate;
    snap.plungeRate = cd_.plungeRate;
    snap.spindleSpeed = cd_.spindleSpeed;

    prepareInputs();
    const std::vector<double> levels = passes_.levels();
    const bool separate = p_.separateLastPass;
    std::vector<Cut> rough = buildCuts(separate ? p_.lastPassAllowance : 0.0, false);
    std::vector<Cut> finish;
    if (separate) {
        finish = buildCuts(0.0, true);
    }
    chooseStarts(rough);
    chooseStarts(finish);
    auto applyOptimisedStarts = [&](std::vector<Cut>& cuts, const std::vector<std::size_t>& ord) {
        if (p_.startAt != StartAt::Optimise) {
            return;
        }
        Vec2 prev = p_.startPosition;
        for (std::size_t i : ord) {
            Cut& c = cuts[i];
            if (c.walk.closed()) {
                const double u = c.walk.project(prev);
                c.path.spans = c.walk.extract(u, u + c.walk.length());
                c.walk = ArcLengthPath(c.path, offTol_);
            }
            prev = c.walk.pointAt(0.0);
        }
    };
    const std::vector<std::size_t> roughOrder = order(rough);
    applyOptimisedStarts(rough, roughOrder);
    const std::vector<std::size_t> finishOrder = order(finish);
    applyOptimisedStarts(finish, finishOrder);
    if (separate) {
        placeTabsOn(finish);
        projectTabs(finish, rough);
    } else {
        placeTabsOn(rough);
    }
    for (std::size_t i : roughOrder) {
        emit(rough[i], levels, false);
    }
    for (std::size_t i : finishOrder) {
        emit(finish[i], {levels.back()}, true);
    }
    // Roughing and finishing report the same vector-level issue once.
    std::vector<ProfileDiagnostic> unique;
    for (ProfileDiagnostic& d : result_.diagnostics) {
        const bool seen = std::any_of(unique.begin(), unique.end(), [&](const ProfileDiagnostic& u) {
            return u.issue == d.issue && u.contour == d.contour && u.message == d.message;
        });
        if (!seen) {
            unique.push_back(std::move(d));
        }
    }
    result_.diagnostics = std::move(unique);
    if (result_.toolpath.isEmpty() && result_.diagnostics.empty()) {
        diag(ProfileIssue::ToolTooLarge, std::nullopt, "no vector produced a cut");
    }
    return std::move(result_);
}

}  // namespace

ProfileResult profileToolpath(const std::vector<Contour>& contours, const tooling::Tool& tool,
                              const ProfileParams& params) {
    ProfileBuilder builder(contours, tool, params);
    return builder.run();
}

}  // namespace carvetto::toolpath
