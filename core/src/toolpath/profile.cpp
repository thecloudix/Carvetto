#include "carvetto/toolpath/profile.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
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
    std::vector<TabSpan> tabs;  // raised stretches (arc length on `walk`)
    Contour leadIn;  // planned lead-in (empty: none or none fits)
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

// A closed cut may close only within the tolerance; snap a line end so the loop closes exactly.
void closeExactly(Contour& c) {
    if (!c.closed || c.spans.empty()) {
        return;
    }
    const Vec2 first = geometry::startPoint(c.spans.front());
    if (auto* last = std::get_if<geometry::LineSpan>(&c.spans.back())) {
        last->end = first;
    } else if (auto* head = std::get_if<geometry::LineSpan>(&c.spans.front())) {
        head->start = geometry::endPoint(c.spans.back());
    }
}

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
    void placeTabsOn(std::vector<Cut>& primary, const std::vector<Cut>& otherPass);
    void matchRoughingTabs(std::vector<Cut>& finish, std::vector<Cut>& rough);
    void dropSweptTabs(std::vector<Cut>& primary, std::vector<Cut>& secondary);
    bool mapTab(const Cut& from, const TabSpan& t, const Cut& onto, TabSpan& out, bool& near) const;
    bool spanFits(const Cut& c, const TabSpan& t, const std::vector<double>& keepClear) const;
    std::vector<double> otherStarts(const Cut& c, const std::vector<Cut>& otherPass) const;
    void reportTabs();
    void collectTabMaterial(const std::vector<Cut>& cuts);
    double leadFloor(const Contour& lead) const;
    std::vector<Vec2> tabMaterial_;  // sample points of every tab's material (all cuts)
    std::map<std::size_t, std::size_t> tabsRequested_;  // per vector, over its primary cuts
    // For each primary (finishing, or only) cut and tab index: the matching spans on the other
    // pass's cuts (cut index, span), so a tab and its roughing cover are kept or dropped together.
    std::vector<std::vector<std::vector<std::pair<std::size_t, TabSpan>>>> tabLinks_;
    std::map<std::size_t, std::size_t> tabsDropped_;
    std::map<std::size_t, std::size_t> tabsOnCorner_;
    void emit(const Cut& cut, const std::vector<double>& levels, bool finishing);
    void emitLeg(const Cut& cut, const PathLeg& leg, bool lastLevel, std::vector<bool>& cornerDone);
    template <class Make>
    bool leadFits(Make make, double scale, Contour& out, bool& gentler) const;
    template <class Make>
    Contour fitLead(Make make, std::size_t contour, const char* what);
    void planLeadIns(std::vector<Cut>& cuts);
    std::vector<Move> cornerMoves(const Cut& cut, Vec2 q, double turn, double bottom) const;
    double coneClearance(Vec2 p, double tipZ) const;
    /// Margin and resolution of the safety checks (trim, leads): 5 % of the distance tolerance.
    double guardTol() const { return 0.05 * tol_.distance; }
    std::vector<LeadOptions> leadVariants() const;

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
    double vbitTan_{1.0};  // tan(half included angle) of a V-bit
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
    if (p_.tabs.enabled && p_.tabs.thickness > passes_.cutDepth) {
        diag(ProfileIssue::TabThicknessClamped, std::nullopt,
             "tab thickness " + std::to_string(p_.tabs.thickness) + " mm is more than the cut depth; clamped to " +
                 std::to_string(passes_.cutDepth) + " mm");
        p_.tabs.thickness = passes_.cutDepth;
    }
    vbit_ = tool_.type == tooling::ToolType::VBit;
    if ((p_.corners.sharpExternal || p_.corners.sharpInternal) && !vbit_) {
        diag(ProfileIssue::CornerNeedsVBit, std::nullopt, "sharp corner options need a V-bit; ignored");
    }
    sharpExternal_ = vbit_ && p_.corners.sharpExternal;
    sharpInternal_ = vbit_ && p_.corners.sharpInternal;
    if (sharpInternal_) {
        vbitTan_ = std::tan(0.5 * tool_.includedAngle);
        if (!(vbitTan_ > 0.0) || !std::isfinite(vbitTan_)) {
            diag(ProfileIssue::InvalidParameters, std::nullopt, "the V-bit has no valid included angle");
            return false;
        }
    }
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
        // Strict: trimmed cuts keep within 1.5 x guardTol() of the full distance (merged regions are
        // offset by d + chordal, so this rarely removes anything there).
        const auto bad = findViolations(walk, 0.0, len, field, cut.distance - guardTol(), guardTol());
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
        closeExactly(c.path);
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

// Positions on `c` (arc length) of the start points of the other pass's cuts that lie near it:
// where the other pass plunges and its lead arrives, so `c` must not have a tab there either.
std::vector<double> ProfileBuilder::otherStarts(const Cut& c, const std::vector<Cut>& otherPass) const {
    std::vector<double> out;
    // Generous reach: at a corner the other pass's start lies allowance / sin(half angle) away,
    // and an extra keep-clear point only shifts a tab.
    const double reach = radius_ + p_.lastPassAllowance + 2.0 * tol_.distance;
    for (const Cut& o : otherPass) {
        const Vec2 p = o.walk.pointAt(0.0);
        const double v = c.walk.project(p);
        if (geometry::distance(c.walk.pointAt(v), p) <= reach) {
            out.push_back(v);
        }
    }
    return out;
}

void ProfileBuilder::placeTabsOn(std::vector<Cut>& primary, const std::vector<Cut>& otherPass) {
    if (!p_.tabs.enabled) {
        return;
    }
    const double raised = p_.tabs.length + 2.0 * radius_;
    for (Cut& c : primary) {
        // At least a tool radius cut to full depth between neighbouring tabs, so they stay apart.
        const TabPlacement tp = placeTabs(c.walk, raised, p_.tabs, otherStarts(c, otherPass), radius_);
        c.tabs.clear();
        for (double u : tp.centres) {
            c.tabs.push_back({u - 0.5 * raised, u + 0.5 * raised});
        }
        tabsRequested_[c.sources.front()] += tp.centres.size() + tp.dropped;
        tabsDropped_[c.sources.front()] += tp.dropped;
        tabsOnCorner_[c.sources.front()] += tp.notAvoided;
    }
}

// True when `t` (a raised stretch on `c`) can be cut on `c`: its footprint (3D ramps included)
// clears the start of a closed cut and the ends of an open one, every point in `keepClear`, and
// the cut's other tabs, each by kMinTabGap.
bool ProfileBuilder::spanFits(const Cut& c, const TabSpan& t, const std::vector<double>& keepClear) const {
    const double ramp = p_.tabs.threeD ? p_.tabs.thickness : 0.0;
    const double a = t.a - ramp;
    const double b = t.b + ramp;
    const double len = c.walk.length();
    if (a < kMinTabGap || b > len - kMinTabGap) {
        return false;
    }
    for (double f : keepClear) {
        if (f > a - kMinTabGap && f < b + kMinTabGap) {
            return false;
        }
    }
    for (const TabSpan& o : c.tabs) {
        if (a < o.b + ramp + kMinTabGap && o.a - ramp < b + kMinTabGap) {
            return false;
        }
    }
    return true;
}

// Maps the raised stretch `t` of cut `from` onto cut `onto` (a cut of the other pass): its two
// ends and its middle are projected, and the stretch between them (following the direction that
// passes through the middle) is widened by the distance tolerance, so the mapped stretch covers
// the tab on `onto` round corners and curves too (projection keeps the angle on arcs). `near` is
// false when `onto` does not pass the tab at all (nothing to map); a partial or ambiguous match
// returns false with `near` true.
bool ProfileBuilder::mapTab(const Cut& from, const TabSpan& t, const Cut& onto, TabSpan& out, bool& near) const {
    const double reach = radius_ + p_.lastPassAllowance + 2.0 * tol_.distance;
    const Vec2 pts[3] = {from.walk.pointAt(t.a), from.walk.pointAt(0.5 * (t.a + t.b)), from.walk.pointAt(t.b)};
    double v[3];
    int hits = 0;
    for (int i = 0; i < 3; ++i) {
        v[i] = onto.walk.project(pts[i]);
        hits += geometry::distance(onto.walk.pointAt(v[i]), pts[i]) <= reach ? 1 : 0;
    }
    near = hits > 0;
    if (hits < 3) {
        return false;
    }
    const double len = onto.walk.length();
    double a = 0.0;
    double b = 0.0;
    if (onto.walk.closed()) {
        auto fwd = [&](double x, double y) {
            const double d = std::fmod(y - x, len);
            return d < 0.0 ? d + len : d;
        };
        const double forward = fwd(v[0], v[1]) + fwd(v[1], v[2]);
        const double backward = fwd(v[1], v[0]) + fwd(v[2], v[1]);
        if (forward <= backward) {
            a = v[0];
            b = v[0] + forward;
        } else {  // `onto` runs the other way (last pass reversed)
            a = v[2];
            b = v[2] + backward;
        }
        if (b - a >= 0.5 * len) {
            return false;
        }
    } else {
        a = std::min(v[0], v[2]);
        b = std::max(v[0], v[2]);
        if (v[1] < a || v[1] > b) {
            return false;
        }
    }
    // Consistency: the stretch must be about as long as the tab (the cuts lie the last-pass
    // allowance apart; corners change lengths by less than that plus a tool radius per end), and
    // every point along the tab must project into it — otherwise the projections landed on
    // different parts of `onto` (a nearby lobe) and the match is ambiguous.
    {
        const double slack = 2.0 * (p_.lastPassAllowance + radius_) + tol_.distance;
        if (std::fabs((b - a) - (t.b - t.a)) > slack) {
            return false;
        }
        const int n = std::max(2, static_cast<int>(std::ceil((t.b - t.a) / 0.25)));
        for (int i = 0; i <= n; ++i) {
            const Vec2 q = from.walk.pointAt(t.a + (t.b - t.a) * i / n);
            const double w = onto.walk.project(q);
            if (geometry::distance(onto.walk.pointAt(w), q) > reach) {
                return false;
            }
            double rel = w - a;
            if (onto.walk.closed()) {
                rel = std::fmod(rel, len);
                if (rel < 0.0) rel += len;
            }
            if (rel < -tol_.distance || rel > (b - a) + tol_.distance) {
                return false;
            }
        }
    }
    // The ends are projected along the normals, so the roughing tool at a mapped end stands the
    // last-pass allowance further from the vector than the finishing tool at the tab's end and
    // cuts none of the tab; widen by the tolerance only (a longer roughing tab would have a part
    // the finishing pass then cuts at full depth).
    out = {a - tol_.distance, b + tol_.distance};
    // Near a corner the normal projection is not enough: the tool standing at a mapped end can
    // still reach the tab's core (the finishing stretch less a tool radius at each end) across the
    // corner. Extend the ends, 0.05 mm at a time and by at most a tool diameter, until it cannot.
    std::vector<Vec2> core;
    {
        const double ca = t.a + radius_;
        const double cb = t.b - radius_;
        const int n = std::max(1, static_cast<int>(std::ceil((cb - ca) / 0.05)));
        for (int i = 0; i <= n && cb >= ca; ++i) {  // exact ends included
            core.push_back(from.walk.pointAt(ca + (cb - ca) * i / n));
        }
    }
    auto reaches = [&](double u) {
        const Vec2 q = onto.walk.pointAt(u);
        return std::any_of(core.begin(), core.end(),
                           [&](Vec2 c) { return geometry::distance(q, c) < radius_ + tol_.distance; });
    };
    double extA = 0.0;
    double extB = 0.0;
    while (extA < 2.0 * radius_ && reaches(out.a - extA)) {
        extA += 0.05;
    }
    while (extB < 2.0 * radius_ && reaches(out.b + extB)) {
        extB += 0.05;
    }
    // Both ends by the same amount, so the mapped stretch stays centred on the tab.
    const double ext = std::max(extA, extB);
    out.a -= ext;
    out.b += ext;
    if (!onto.walk.closed() && (out.a < 0.0 || out.b > len)) {
        return false;
    }
    return true;
}

// Separate last pass: the finishing cuts' tabs (placed first) are carried onto the roughing cuts.
// Every finishing tab gets a covering raised stretch on each roughing cut that passes it; when
// one cannot be placed there (it would cover a roughing start, a finishing start or another tab),
// the finishing tab is left out too, so the two passes always agree.
void ProfileBuilder::matchRoughingTabs(std::vector<Cut>& finish, std::vector<Cut>& rough) {
    for (Cut& r : rough) {
        r.tabs.clear();
    }
    std::vector<std::vector<double>> roughKeepClear(rough.size());
    for (std::size_t k = 0; k < rough.size(); ++k) {
        roughKeepClear[k] = otherStarts(rough[k], finish);
    }
    tabLinks_.assign(finish.size(), {});
    for (std::size_t fi = 0; fi < finish.size(); ++fi) {
        Cut& f = finish[fi];
        std::vector<TabSpan> kept;
        for (const TabSpan& t : f.tabs) {
            std::vector<std::pair<std::size_t, TabSpan>> mapped;
            bool ok = true;
            for (std::size_t k = 0; k < rough.size() && ok; ++k) {
                TabSpan m;
                bool near = false;
                if (mapTab(f, t, rough[k], m, near)) {
                    ok = spanFits(rough[k], m, roughKeepClear[k]);
                    mapped.push_back({k, m});
                } else if (near) {
                    ok = false;  // passes the tab only in part: cannot cover it
                }
            }
            if (!ok) {
                ++tabsDropped_[f.sources.front()];
                continue;
            }
            for (const auto& [k, m] : mapped) {
                rough[k].tabs.push_back(m);
            }
            kept.push_back(t);
            tabLinks_[fi].push_back(mapped);
        }
        f.tabs = std::move(kept);
    }
    for (Cut& r : rough) {
        std::sort(r.tabs.begin(), r.tabs.end(), [](const TabSpan& x, const TabSpan& y) { return x.a < y.a; });
    }
}

// A tab only holds if no other part of any cut sweeps over it at full depth: in a slot narrower
// than the tool (or past a nearby corner) the cut along the opposite wall removes the tab's
// material. Each primary tab, with its matched stretches on the other pass, is dropped (and
// reported) when a point of the middle half of its core (the raised stretch less a tool radius
// and 0.05 mm at each end) lies within a tool radius of any cut, its own included, outside the
// raised stretches of the tabs still kept. Sweeps nearer the core ends (a corner next to the
// tab) only shorten the tab.
// Repeated until nothing changes, since a dropped tab exposes more of its cut.
void ProfileBuilder::dropSweptTabs(std::vector<Cut>& primary, std::vector<Cut>& secondary) {
    constexpr double h = 0.05;  // sampling step of the cut paths
    const bool linked = !tabLinks_.empty();
    struct Ref {
        bool second;
        std::size_t cut;
        TabSpan span;
    };
    // Groups: a primary tab and its linked secondary spans.
    std::vector<std::vector<Ref>> groups;
    std::vector<std::size_t> groupSource;
    for (std::size_t c = 0; c < primary.size(); ++c) {
        for (std::size_t t = 0; t < primary[c].tabs.size(); ++t) {
            std::vector<Ref> g{{false, c, primary[c].tabs[t]}};
            if (linked && c < tabLinks_.size() && t < tabLinks_[c].size()) {
                for (const auto& [k, m] : tabLinks_[c][t]) {
                    g.push_back({true, k, m});
                }
            }
            groups.push_back(std::move(g));
            groupSource.push_back(primary[c].sources.front());
        }
    }
    if (groups.empty()) {
        return;
    }
    std::vector<bool> alive(groups.size(), true);
    auto cutOf = [&](const Ref& r) -> const Cut& { return r.second ? secondary[r.cut] : primary[r.cut]; };
    struct Sample {
        bool second;
        std::size_t cut;
        double u;
        Vec2 p;
    };
    // Unprotected samples of every cut: points every h outside the raised stretches still alive,
    // plus the exact ends of those stretches (where the tool goes back down).
    auto unprotected = [&]() {
        std::vector<Sample> pts;
        auto add = [&](const std::vector<Cut>& cuts, bool second) {
            for (std::size_t c = 0; c < cuts.size(); ++c) {
                std::vector<TabSpan> keep;
                for (std::size_t g = 0; g < groups.size(); ++g) {
                    if (!alive[g]) continue;
                    for (const Ref& r : groups[g]) {
                        if (r.second == second && r.cut == c) keep.push_back(r.span);
                    }
                }
                const ArcLengthPath& w = cuts[c].walk;
                for (const TabSpan& t : keep) {
                    pts.push_back({second, c, t.a, w.pointAt(t.a)});
                    pts.push_back({second, c, t.b, w.pointAt(t.b)});
                }
                const int n = std::max(1, static_cast<int>(std::ceil(w.length() / h)));
                for (int i = 0; i <= n; ++i) {
                    const double u = w.length() * i / n;
                    const bool covered = std::any_of(keep.begin(), keep.end(),
                                                     [&](const TabSpan& t) { return u > t.a && u < t.b; });
                    if (!covered) pts.push_back({second, c, u, w.pointAt(u)});
                }
            }
        };
        add(primary, false);
        add(secondary, true);
        return pts;
    };
    bool changed = true;
    while (changed) {
        changed = false;
        const std::vector<Sample> open = unprotected();
        // Bucket the samples on a grid of tool-radius cells; a query looks at the 3 x 3 cells round it.
        const double cell = std::max(radius_, 1e-3);
        std::map<std::pair<long, long>, std::vector<Vec2>> grid;
        auto key = [&](Vec2 p) {
            return std::pair<long, long>{static_cast<long>(std::floor(p.x / cell)),
                                         static_cast<long>(std::floor(p.y / cell))};
        };
        for (const Sample& q : open) {
            grid[key(q.p)].push_back(q.p);
        }
        auto sweptAt = [&](Vec2 p) {
            const auto [cx, cy] = key(p);
            for (long dx = -1; dx <= 1; ++dx) {
                for (long dy = -1; dy <= 1; ++dy) {
                    const auto it = grid.find({cx + dx, cy + dy});
                    if (it == grid.end()) continue;
                    for (const Vec2& q : it->second) {
                        if (geometry::distance(p, q) < radius_ - tol_.distance) return true;
                    }
                }
            }
            return false;
        };
        for (std::size_t g = 0; g < groups.size(); ++g) {
            if (!alive[g]) continue;
            bool swept = false;
            for (const Ref& r : groups[g]) {
                // The middle half of the core: sweeps nearer its ends (a corner next to the tab)
                // only shorten the tab; a sweep over its middle removes it.
                const double c0 = r.span.a + radius_ + h;
                const double c1 = r.span.b - radius_ - h;
                const double a = c0 + 0.25 * (c1 - c0);
                const double b = c1 - 0.25 * (c1 - c0);
                const ArcLengthPath& w = cutOf(r).walk;
                const int n = std::max(1, static_cast<int>(std::ceil((b - a) / 0.25)));
                for (int i = 0; i <= n && b >= a && !swept; ++i) {  // exact core ends included
                    const Vec2 p = w.pointAt(a + (b - a) * i / n);
                    swept = sweptAt(p);
                }
                if (swept) break;
            }
            if (swept) {
                alive[g] = false;
                ++tabsDropped_[groupSource[g]];
                changed = true;
                break;  // recompute the unprotected samples
            }
        }
    }
    for (Cut& c : primary) c.tabs.clear();
    for (Cut& c : secondary) c.tabs.clear();
    for (std::size_t g = 0; g < groups.size(); ++g) {
        if (!alive[g]) continue;
        for (const Ref& r : groups[g]) {
            (r.second ? secondary[r.cut] : primary[r.cut]).tabs.push_back(r.span);
        }
    }
    auto byStart = [](const TabSpan& x, const TabSpan& y) { return x.a < y.a; };
    for (Cut& c : primary) std::sort(c.tabs.begin(), c.tabs.end(), byStart);
    for (Cut& c : secondary) std::sort(c.tabs.begin(), c.tabs.end(), byStart);
}

// Points (every 0.1 mm) along the stretch of each tab that leaves material: its footprint minus a
// tool radius at each end (the raised stretch includes the tool diameter).
void ProfileBuilder::collectTabMaterial(const std::vector<Cut>& cuts) {
    const double ramp = p_.tabs.threeD ? p_.tabs.thickness : 0.0;
    for (const Cut& c : cuts) {
        for (const TabSpan& t : c.tabs) {
            const double a = t.a - ramp + radius_;
            const double b = t.b + ramp - radius_;
            if (b <= a) {
                continue;
            }
            const int n = std::max(1, static_cast<int>(std::ceil((b - a) / 0.1)));
            for (int i = 0; i <= n; ++i) {
                tabMaterial_.push_back(c.walk.pointAt(a + (b - a) * i / n));
            }
        }
    }
}

// Lowest height a lead may run at: a lead whose tool comes within reach of tab material (closer
// than a tool diameter, plus the 0.1 mm sampling) runs no lower than the tab top, so it never
// notches a tab; the plunge to the level then happens at the cut start, which tabs keep clear.
double ProfileBuilder::leadFloor(const Contour& lead) const {
    if (lead.isEmpty() || tabMaterial_.empty()) {
        return kNoCeiling;
    }
    const double reach = 2.0 * radius_ + 0.1;
    const ArcLengthPath lw(lead, offTol_);
    const int n = std::max(1, static_cast<int>(std::ceil(lw.length() / 0.1)));
    const geometry::Box2 box = lead.bounds().inflated(reach);
    for (const Vec2& m : tabMaterial_) {
        if (!box.contains(m)) {
            continue;
        }
        for (int i = 0; i <= n; ++i) {
            if (geometry::distance(lw.pointAt(lw.length() * i / n), m) < reach) {
                return passes_.bottomZ() + p_.tabs.thickness;
            }
        }
    }
    return kNoCeiling;
}

// One diagnostic per vector with the totals over all its cuts (so kept + left out adds up).
void ProfileBuilder::reportTabs() {
    for (const auto& [src, n] : tabsDropped_) {
        if (n > 0) {
            diag(ProfileIssue::TabsNotPlaced, src,
                 vecName(src) + ": " + std::to_string(n) + " of " + std::to_string(tabsRequested_[src]) +
                     " tab(s) left out (no room: they would overlap, run past an open end, cover where the tool "
                     "plunges, leave no stretch cut to full depth, or could not be matched on the roughing cut)");
        }
    }
    for (const auto& [src, n] : tabsOnCorner_) {
        if (n > 0) {
            diag(ProfileIssue::TabsNotPlaced, src,
                 vecName(src) + ": " + std::to_string(n) +
                     " tab(s) left on a corner or curve (no straight stretch long enough)");
        }
    }
}

// Lead options to try, in order: as asked, then a gentler variant that clears an adjacent wall
// more often (straight: perpendicular; circular: half the sweep).
std::vector<LeadOptions> ProfileBuilder::leadVariants() const {
    std::vector<LeadOptions> v{p_.leads};
    LeadOptions alt = p_.leads;
    if (alt.type == LeadType::Straight && alt.angle != geometry::kPi / 2.0) {
        alt.angle = geometry::kPi / 2.0;
        v.push_back(alt);
    } else if (alt.type == LeadType::Circular) {
        alt.sweep *= 0.5;
        v.push_back(alt);
    }
    return v;
}

// True (and the lead in `out`) when some variant of `make(options, scale)` stays at least the tool
// radius from every vector.
template <class Make>
bool ProfileBuilder::leadFits(Make make, double scale, Contour& out, bool& gentler) const {
    const std::vector<LeadOptions> variants = leadVariants();
    for (std::size_t k = 0; k < variants.size(); ++k) {
        Contour cand = make(variants[k], scale);
        if (cand.isEmpty()) {
            continue;
        }
        if (p_.side == ProfileSide::On ||
            isClear(ArcLengthPath(cand, offTol_), allField_, radius_ + std::max(0.0, p_.allowance) - guardTol(),
                    guardTol())) {
            out = std::move(cand);
            gentler = k > 0;
            return true;
        }
    }
    return false;
}

// The lead `make(options, scale)` at full size, or halved up to three times until it stays at
// least the tool radius from every vector; empty (with a diagnostic) when even the smallest gouges.
template <class Make>
Contour ProfileBuilder::fitLead(Make make, std::size_t contour, const char* what) {
    for (double scale : {1.0, 0.5, 0.25, 0.125}) {
        Contour lead;
        bool gentler = false;
        if (leadFits(make, scale, lead, gentler)) {
            if (scale < 1.0 || gentler) {
                diag(ProfileIssue::LeadShortened, contour,
                     vecName(contour) + ": " + what + (gentler ? " made gentler" : "") +
                         (scale < 1.0 ? " shortened" : "") + " to keep clear of the vectors");
            }
            return lead;
        }
    }
    diag(ProfileIssue::LeadDropped, contour, vecName(contour) + ": " + what + " left out (it would gouge a vector)");
    return {};
}

// Lead-ins are planned before tabs: when no lead fits at the chosen start of a closed cut (for
// example an inside corner, where the approach would cross the adjacent wall), the start is moved
// to the middle of a span (the start span first, then the others from the longest). At each start
// the lead is tried at full size and then halved up to three times.
void ProfileBuilder::planLeadIns(std::vector<Cut>& cuts) {
    if (p_.leads.type == LeadType::None) {
        return;
    }
    for (Cut& c : cuts) {
        const std::size_t src = c.sources.front();
        const bool wasteLeft = !c.contourOnLeft;
        std::vector<double> starts{0.0};
        if (c.walk.closed()) {
            std::vector<std::size_t> spans(c.walk.spanCount());
            std::iota(spans.begin(), spans.end(), std::size_t{0});
            std::stable_sort(spans.begin() + 1, spans.end(), [&](std::size_t x, std::size_t y) {
                return c.walk.spanLength(x) > c.walk.spanLength(y);
            });
            for (std::size_t i : spans) {
                starts.push_back(c.walk.spanStart(i) + 0.5 * c.walk.spanLength(i));
            }
        }
        bool found = false;
        for (double u : starts) {
            for (double scale : {1.0, 0.5, 0.25, 0.125}) {
                const Vec2 at = c.walk.pointAt(u);
                const Vec2 t = c.walk.tangentAt(u);
                Contour lead;
                bool gentler = false;
                if (!leadFits([&](const LeadOptions& o, double sc) { return leadIn(at, t, wasteLeft, o, sc); }, scale,
                              lead, gentler)) {
                    continue;
                }
                if (u != 0.0) {
                    c.path.spans = c.walk.extract(u, u + c.walk.length());
                    closeExactly(c.path);
                    c.walk = ArcLengthPath(c.path, offTol_);
                    diag(ProfileIssue::StartMoved, src,
                         vecName(src) + ": start moved along the cut so that the lead-in fits");
                }
                if (scale < 1.0 || gentler) {
                    diag(ProfileIssue::LeadShortened, src,
                         vecName(src) + ": lead-in" + (gentler ? " made gentler" : "") +
                             (scale < 1.0 ? " shortened" : "") + " to keep clear of the vectors");
                }
                c.leadIn = std::move(lead);
                found = true;
                break;
            }
            if (found) {
                break;
            }
        }
        if (!found) {
            diag(ProfileIssue::LeadDropped, src, vecName(src) + ": lead-in left out (it would gouge a vector)");
        }
    }
}

// How far the V-bit's cone stays inside the walls with the tip at `p`, `tipZ`: distance from p to
// the nearest vector, minus the allowance, minus the radius the cone cuts at the material surface
// (Z = 0; capped at the tool radius). Negative = it would cut past a wall.
double ProfileBuilder::coneClearance(Vec2 p, double tipZ) const {
    const double cone = std::min(std::max(0.0, -tipZ) * vbitTan_, 0.5 * tool_.diameter);
    return allField_.distance(p) - std::max(0.0, p_.allowance) - cone;
}

// The sharp-internal-corner excursion for a V-bit at the path's inside corner `q` (tip at
// `bottom`), returning to q; empty when there is no genuine corner of the vector there.
//
// The corner V must be a junction of a source vector whose direction turns by more than 5° (so
// polyline vertices of polygonised curves and merged-region offsets do not count), on an exact
// offset (merged polygonal regions get none). The tip runs from q, at the "knee" depth where the
// cone itself is r wide, towards V and rises linearly to the surface where the distance to the
// walls equals the allowance (V itself when the allowance is 0). That straight climb is exact
// between two straight walls; it is checked against the real walls every 0.005 mm, and when it
// would cut past one (a curved wall next to the corner) the climb follows the walls instead: at
// samples 0.05 mm apart the tip is held at (distance - allowance - 0.05) / tan(half angle) below
// the surface, which keeps the cone inside the walls between samples too (distance changes by at
// most the sample spacing).
std::vector<Move> ProfileBuilder::cornerMoves(const Cut& cut, Vec2 q, double turn, double bottom) const {
    constexpr double kCornerTurn = 0.0872664625997165;  // 5 degrees
    // The path itself must turn sharply here: polyline vertices of polygonised curves turn by
    // fractions of a degree and are not corners.
    if (!cut.exact || p_.side == ProfileSide::On || std::fabs(turn) <= kCornerTurn) {
        return {};
    }
    Vec2 v = q;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t s : cut.sources) {
        const Contour& c = in_.contours[s];
        const std::size_t n = c.spans.size();
        for (std::size_t j = 0; j < n; ++j) {
            if (j == 0 && !c.closed) {
                continue;
            }
            const Span& prev = c.spans[(j + n - 1) % n];
            const Span& next = c.spans[j];
            const Vec2 a = geometry::tangentAt(prev, 1.0);
            const Vec2 b = geometry::tangentAt(next, 0.0);
            if (std::fabs(std::atan2(a.cross(b), a.dot(b))) <= kCornerTurn) {
                continue;
            }
            const Vec2 corner = geometry::startPoint(next);
            const double d = geometry::distance(corner, q);
            if (d < best) {
                best = d;
                v = corner;
            }
        }
    }
    const double r = radius_;
    const double a = std::max(0.0, p_.allowance);
    // Between straight walls turning by `turn`, Q lies (r + a) / cos(turn / 2) from the corner.
    // Accept the nearest genuine corner only if it is about that far (twice, for curved walls).
    const double expected = (r + a) / std::cos(0.5 * std::min(std::fabs(turn), geometry::kPi - kCornerTurn));
    if (!(best <= 2.0 * expected) || best <= tol_.distance) {
        return {};
    }
    const double knee = std::max(bottom, -r / vbitTan_);
    // Where the climb ends: the point towards V whose distance to the walls equals the allowance.
    Vec2 target = v;
    if (a > 0.0) {
        double lo = 0.0;
        double hi = 1.0;
        for (int it = 0; it < 60; ++it) {
            const double mid = 0.5 * (lo + hi);
            (allField_.distance(geometry::lerp(q, v, mid)) > a ? lo : hi) = mid;
        }
        target = geometry::lerp(q, v, lo);
    }
    const double len = geometry::distance(q, target);
    bool straight = true;
    const int checks = std::max(2, static_cast<int>(std::ceil(len / 0.005)));
    for (int i = 0; i <= checks && straight; ++i) {
        const double s = static_cast<double>(i) / checks;
        straight = coneClearance(geometry::lerp(q, target, s), knee * (1.0 - s)) >= -1e-9;
    }
    std::vector<Move> up;
    if (straight) {
        if (knee > bottom) {
            up.push_back(Move::feed(Vec3{q, knee}));
        }
        up.push_back(Move::feed(Vec3{target, 0.0}));
    } else {
        constexpr double h = 0.05;
        auto heldZ = [&](Vec2 p) {
            const double room = allField_.distance(p) - a - h;
            return std::max(knee, -std::max(0.0, room) / vbitTan_);
        };
        up.push_back(Move::feed(Vec3{q, heldZ(q)}));
        const int n = std::max(1, static_cast<int>(std::ceil(len / h)));
        for (int i = 1; i <= n; ++i) {
            const Vec2 p = geometry::lerp(q, target, static_cast<double>(i) / n);
            up.push_back(Move::feed(Vec3{p, heldZ(p)}));
        }
    }
    // Back down the same way, then to the bottom at q.
    // Vertical descents are plunges; sloped ones (along the bisector) are feeds.
    std::vector<Move> moves = up;
    auto down = [&](Vec3 to) {
        const Vec3 from = moves.back().to;
        if (from == to) {
            return;  // no zero-length moves
        }
        moves.push_back(from.xy() == to.xy() ? Move::plunge(to) : Move::feed(to));
    };
    for (std::size_t i = up.size() - 1; i-- > 0;) {
        down(up[i].to);
    }
    down(Vec3{q, bottom});  // straight climb without a knee: one sloped feed back to q
    return moves;
}

void ProfileBuilder::emitLeg(const Cut& cut, const PathLeg& leg, bool lastLevel, std::vector<bool>& cornerDone) {
    const double bottom = passes_.bottomZ();
    ZProfile z = leg.z;
    if (!cut.tabs.empty()) {
        const double zTab = bottom + p_.tabs.thickness;
        z = ZProfile::max(z, tabCeiling(cut.walk, leg.u0, leg.u1, cut.tabs, zTab, bottom, p_.tabs.threeD));
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
                std::vector<Move> moves = cornerMoves(cut, cut.walk.pointAt(kinks[k].u), kinks[k].turn, bottom);
                if (moves.empty()) {
                    cornerDone[k] = true;
                    break;
                }
                inserts.push_back({lam, std::move(moves)});
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

    const Contour& in = cut.leadIn;  // planned and checked by planLeadIns
    const Vec2 entry = in.isEmpty() ? walk.pointAt(0.0) : in.startPoint();
    out.append(Move::rapid(Vec3{entry, p_.safeZ}));
    double zFrom = levels.front();
    if (!in.isEmpty()) {
        const ArcLengthPath lw(in, offTol_);
        const ZProfile floor = ZProfile::flat(leadFloor(in));  // keeps the lead out of tabs
        if (rampFirst && ramp.onLeadIn) {
            const ZProfile z = ZProfile::max(ZProfile::linear(z0, levels.front(), lw.length()), floor);
            out.append(Move::plunge(Vec3{entry, z.after(0.0)}));
            appendLeg(lw, 0.0, lw.length(), z, out);
        } else if (rampFirst) {
            const ZProfile z = ZProfile::max(ZProfile::flat(z0), floor);
            out.append(Move::plunge(Vec3{entry, z.after(0.0)}));
            appendLeg(lw, 0.0, lw.length(), z, out);
            zFrom = z0;
        } else {
            const ZProfile z = ZProfile::max(ZProfile::flat(levels.front()), floor);
            out.append(Move::plunge(Vec3{entry, z.after(0.0)}));
            appendLeg(lw, 0.0, lw.length(), z, out);
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
            fitLead([&](const LeadOptions& o, double scale) { return leadOut(at, t, wasteLeft, o, scale); }, src,
                    "lead-out");
        if (!lo.isEmpty()) {
            // Leave at the height the cut ended at: inside a tab (e.g. after an overcut) that is
            // the tab top, so the lead-out never cuts into the tab.
            const ArcLengthPath lw(lo, offTol_);
            const double z = std::max(out.moves.back().to.z, leadFloor(lo));
            if (z > out.moves.back().to.z) {
                out.append(Move::feed(Vec3{out.moves.back().to.xy(), z}));  // up out of the way of a tab
            }
            appendLeg(lw, 0.0, lw.length(), ZProfile::flat(z), out);
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
                closeExactly(c.path);
                c.walk = ArcLengthPath(c.path, offTol_);
            }
            prev = c.walk.pointAt(0.0);
        }
    };
    const std::vector<std::size_t> roughOrder = order(rough);
    applyOptimisedStarts(rough, roughOrder);
    // Finishing cuts follow the roughing order: each takes the position of the roughing cut
    // nearest to it (they lie the last-pass allowance apart).
    std::vector<std::size_t> finishOrder(finish.size());
    {
        std::vector<std::size_t> rank(rough.size());
        for (std::size_t k = 0; k < roughOrder.size(); ++k) {
            rank[roughOrder[k]] = k;
        }
        // Counterpart = the roughing cut with the smallest median distance over 17 points along
        // the finishing cut (robust where trimmed or merged pieces lie close together), if that
        // median is within the last-pass allowance + a tool radius. A finishing cut without one
        // (e.g. a small loop that vanished at the roughing distance) takes the rank of the
        // roughing cut nearest its start point.
        std::vector<double> key(finish.size(), static_cast<double>(rough.size()));
        const double match = p_.lastPassAllowance + radius_ + tol_.distance;
        for (std::size_t f = 0; f < finish.size(); ++f) {
            double best = std::numeric_limits<double>::infinity();
            double nearestStart = std::numeric_limits<double>::infinity();
            double startKey = key[f];
            const Vec2 start = finish[f].walk.pointAt(0.0);
            for (std::size_t r = 0; r < rough.size(); ++r) {
                std::vector<double> ds;
                for (int k = 0; k <= 16; ++k) {
                    const Vec2 p = finish[f].walk.pointAt(finish[f].walk.length() * k / 16.0);
                    ds.push_back(geometry::distanceTo(rough[r].walk.contour(), p));
                }
                std::nth_element(ds.begin(), ds.begin() + 8, ds.end());
                if (ds[8] < best) {
                    best = ds[8];
                    key[f] = static_cast<double>(rank[r]);
                }
                const double ds0 = geometry::distanceTo(rough[r].walk.contour(), start);
                if (ds0 < nearestStart) {
                    nearestStart = ds0;
                    startKey = static_cast<double>(rank[r]);
                }
            }
            if (!(best <= match)) {
                key[f] = startKey;
            }
        }
        std::iota(finishOrder.begin(), finishOrder.end(), std::size_t{0});
        std::stable_sort(finishOrder.begin(), finishOrder.end(),
                         [&](std::size_t x, std::size_t y) { return key[x] < key[y]; });
    }
    applyOptimisedStarts(finish, finishOrder);
    planLeadIns(rough);
    planLeadIns(finish);
    if (separate) {
        placeTabsOn(finish, rough);
        matchRoughingTabs(finish, rough);
        dropSweptTabs(finish, rough);
    } else {
        placeTabsOn(rough, {});
        dropSweptTabs(rough, finish);
    }
    reportTabs();
    collectTabMaterial(rough);
    collectTabMaterial(finish);
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
