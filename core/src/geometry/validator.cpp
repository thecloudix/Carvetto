#include "carvetto/geometry/validator.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>

#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/geometry/intersect.hpp"

namespace carvetto::geometry::validator {

namespace {

using Objects = std::vector<const ObjectContours*>;

constexpr char kUnknown = 2;

bool okNumber(double v) { return std::isfinite(v) && std::fabs(v) <= kMaxCoordinateMm; }
bool okPoint(Vec2 p) { return okNumber(p.x) && okNumber(p.y); }

bool okSpan(const Span& s) {
    if (const auto* l = std::get_if<LineSpan>(&s)) {
        return okPoint(l->start) && okPoint(l->end);
    }
    if (const auto* a = std::get_if<ArcSpan>(&s)) {
        return okPoint(a->centre) && okNumber(a->radius) && okNumber(a->startAngle) && okNumber(a->endAngle);
    }
    const auto& b = std::get<BezierSpan>(s);
    return okPoint(b.p0) && okPoint(b.p1) && okPoint(b.p2) && okPoint(b.p3);
}

bool okContour(const Contour& c) {
    return std::all_of(c.spans.begin(), c.spans.end(), [](const Span& s) { return okSpan(s); });
}

/// Per-run state: which contours are usable and the memoised crossing checks.
struct Context {
    const Objects& objs;
    const Options& opt;
    std::vector<std::vector<char>> valid;
    std::vector<std::vector<char>> selfState;  ///< 0 clean, 1 crossing, 2 not evaluated, 3 could not be checked.
    std::size_t totalSpans{0};

    Context(const Objects& o, const Options& options) : objs(o), opt(options) {
        valid.resize(o.size());
        selfState.resize(o.size());
        for (std::size_t i = 0; i < o.size(); ++i) {
            const std::size_t n = o[i] != nullptr ? o[i]->size() : 0;
            valid[i].assign(n, 1);
            selfState[i].assign(n, kUnknown);
            for (std::size_t j = 0; j < n; ++j) {
                totalSpans += (*o[i])[j].spans.size();
            }
        }
        if (totalSpans > kMaxSpans) {
            return;  // the caller bails out before looking at any contour
        }
        for (std::size_t i = 0; i < o.size(); ++i) {
            for (std::size_t j = 0; j < valid[i].size(); ++j) {
                valid[i][j] = okContour((*o[i])[j]) ? 1 : 0;
            }
        }
    }

    const Contour& contour(std::size_t o, std::size_t c) const { return (*objs[o])[c]; }
    std::size_t contourCount(std::size_t o) const { return valid[o].size(); }
    bool usable(std::size_t o, std::size_t c) const { return valid[o][c] != 0; }

    char selfIntersectState(std::size_t o, std::size_t c) {
        char& st = selfState[o][c];
        if (st != kUnknown) {
            return st;
        }
        const Contour& k = contour(o, c);
        if (k.spans.size() > kMaxSelfIntersectSpans) {
            return st = 3;
        }
        // Zero-length spans would read as the contour touching itself; judge it without them.
        const bool dust = std::any_of(k.spans.begin(), k.spans.end(),
                                      [&](const Span& s) { return isDegenerate(s, opt.tol); });
        Contour cleaned;
        if (dust) {
            cleaned.closed = k.closed;
            for (const Span& s : k.spans) {
                if (!isDegenerate(s, opt.tol)) {
                    cleaned.spans.push_back(s);
                }
            }
        }
        const ContourIntersections r = selfIntersections(dust ? cleaned : k, opt.tol);
        if (!r.complete()) {
            return st = 3;
        }
        return st = r.isEmpty() ? 0 : 1;
    }
};

void push(Report& rep, Issue issue) {
    ++rep.counts[static_cast<std::size_t>(issue.kind)];
    rep.issues.push_back(issue);
}

void detectInvalid(Context& cx, Report& rep) {
    for (std::size_t o = 0; o < cx.objs.size(); ++o) {
        for (std::size_t c = 0; c < cx.contourCount(o); ++c) {
            if (!cx.usable(o, c)) {
                push(rep, {IssueKind::InvalidGeometry, o, c, 0, 0, 0, false});
            }
        }
    }
}

void detectZeroLength(Context& cx, Report& rep) {
    for (std::size_t o = 0; o < cx.objs.size(); ++o) {
        for (std::size_t c = 0; c < cx.contourCount(o); ++c) {
            if (!cx.usable(o, c)) {
                continue;
            }
            const Contour& k = cx.contour(o, c);
            for (std::size_t s = 0; s < k.spans.size(); ++s) {
                if (isDegenerate(k.spans[s], cx.opt.tol)) {
                    push(rep, {IssueKind::ZeroLengthSpan, o, c, s, 0, 0, true});
                }
            }
        }
    }
}

void detectTiny(Context& cx, Report& rep) {
    for (std::size_t o = 0; o < cx.objs.size(); ++o) {
        for (std::size_t c = 0; c < cx.contourCount(o); ++c) {
            if (!cx.usable(o, c)) {
                continue;
            }
            const Contour& k = cx.contour(o, c);
            bool tiny = k.spans.empty();
            if (!tiny) {
                const Box2 b = k.bounds();
                tiny = b.width() <= cx.opt.tinySizeMm && b.height() <= cx.opt.tinySizeMm;
            }
            if (tiny) {
                push(rep, {IssueKind::TinyContour, o, c, 0, 0, 0, true});
            }
        }
    }
}

void detectOpen(Context& cx, Report& rep) {
    for (std::size_t o = 0; o < cx.objs.size(); ++o) {
        for (std::size_t c = 0; c < cx.contourCount(o); ++c) {
            if (!cx.usable(o, c)) {
                continue;
            }
            const Contour& k = cx.contour(o, c);
            if (k.closed || k.spans.empty()) {
                continue;
            }
            const bool fixable = distance(k.endPoint(), k.startPoint()) <= cx.opt.closeGapMm;
            push(rep, {IssueKind::OpenContour, o, c, 0, 0, 0, fixable});
        }
    }
}

bool near(const Tolerance& tol, Vec2 a, Vec2 b) { return tol.equalPoints(a, b); }

bool isFullCircleSpan(const ArcSpan& a) { return a.isFullCircle(); }

bool spansMatch(const Span& a, const Span& b, const Tolerance& tol) {
    if (a.index() != b.index()) {
        return false;
    }
    if (const auto* la = std::get_if<LineSpan>(&a)) {
        const auto& lb = std::get<LineSpan>(b);
        return near(tol, la->start, lb.start) && near(tol, la->end, lb.end);
    }
    if (const auto* aa = std::get_if<ArcSpan>(&a)) {
        const auto& ab = std::get<ArcSpan>(b);
        if (!near(tol, aa->centre, ab.centre) || !tol.equalDistance(aa->radius, ab.radius)) {
            return false;
        }
        if (isFullCircleSpan(*aa) || isFullCircleSpan(ab)) {
            return isFullCircleSpan(*aa) && isFullCircleSpan(ab);
        }
        return aa->direction == ab.direction && near(tol, aa->startPoint(), ab.startPoint()) &&
               near(tol, aa->endPoint(), ab.endPoint());
    }
    const auto& ba = std::get<BezierSpan>(a);
    const auto& bb = std::get<BezierSpan>(b);
    return near(tol, ba.p0, bb.p0) && near(tol, ba.p1, bb.p1) && near(tol, ba.p2, bb.p2) &&
           near(tol, ba.p3, bb.p3);
}

/// True when `b` traces the same spans as `a` in the same order, starting at span `shift` of `b`.
bool sameRun(const Contour& a, const Contour& b, std::size_t shift, const Tolerance& tol, std::size_t& work) {
    const std::size_t n = a.spans.size();
    for (std::size_t i = 0; i < n; ++i) {
        ++work;
        if (!spansMatch(a.spans[i], b.spans[(i + shift) % n], tol)) {
            return false;
        }
    }
    return true;
}

bool sameContour(const Contour& a, const Contour& b, const Tolerance& tol, std::size_t& work) {
    if (a.closed != b.closed || a.spans.size() != b.spans.size() || a.spans.empty()) {
        return false;
    }
    const std::size_t n = a.spans.size();
    if (n == 1 && a.closed) {
        const auto* ca = std::get_if<ArcSpan>(&a.spans[0]);
        const auto* cb = std::get_if<ArcSpan>(&b.spans[0]);
        if (ca != nullptr && cb != nullptr && isFullCircleSpan(*ca) && isFullCircleSpan(*cb)) {
            return near(tol, ca->centre, cb->centre) && tol.equalDistance(ca->radius, cb->radius);
        }
    }
    const std::size_t shifts = a.closed ? n : 1;
    const Contour rev = b.reversed();
    for (std::size_t s = 0; s < shifts; ++s) {
        // Cheap reject before the full run: the start points must agree.
        if (near(tol, startPoint(a.spans[0]), startPoint(b.spans[s]))) {
            if (sameRun(a, b, s, tol, work)) {
                return true;
            }
        }
        if (near(tol, startPoint(a.spans[0]), startPoint(rev.spans[s]))) {
            if (sameRun(a, rev, s, tol, work)) {
                return true;
            }
        }
        ++work;
    }
    return false;
}

void detectDuplicates(Context& cx, Report& rep) {
    struct Item {
        std::size_t o, c;
        Box2 box;
    };
    std::vector<Item> items;
    for (std::size_t o = 0; o < cx.objs.size(); ++o) {
        for (std::size_t c = 0; c < cx.contourCount(o); ++c) {
            if (cx.usable(o, c) && !cx.contour(o, c).spans.empty()) {
                items.push_back({o, c, cx.contour(o, c).bounds()});
            }
        }
    }
    // `items` is in (object, contour) order: an item's rank is its position.
    std::vector<std::size_t> order(items.size());
    std::iota(order.begin(), order.end(), std::size_t{0});
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        if (items[a].box.min.x != items[b].box.min.x) {
            return items[a].box.min.x < items[b].box.min.x;
        }
        return a < b;
    });
    constexpr std::size_t kNone = static_cast<std::size_t>(-1);
    std::vector<std::size_t> dupOf(items.size(), kNone);
    const double slack = 2.0 * cx.opt.tol.distance;
    std::size_t work = 0;
    bool budget = true;
    for (std::size_t ai = 0; ai < order.size() && budget; ++ai) {
        const Item& ia = items[order[ai]];
        for (std::size_t bi = ai + 1; bi < order.size(); ++bi) {
            const Item& ib = items[order[bi]];
            if (ib.box.min.x - ia.box.min.x > slack) {
                break;
            }
            if (++work > kMaxDuplicateComparisons) {
                budget = false;
                break;
            }
            if (std::fabs(ib.box.min.y - ia.box.min.y) > slack || std::fabs(ib.box.max.x - ia.box.max.x) > slack ||
                std::fabs(ib.box.max.y - ia.box.max.y) > slack) {
                continue;
            }
            const std::size_t lo = std::min(order[ai], order[bi]);
            const std::size_t hi = std::max(order[ai], order[bi]);
            if (dupOf[hi] != kNone && dupOf[hi] <= lo) {
                continue;
            }
            if (sameContour(cx.contour(items[lo].o, items[lo].c), cx.contour(items[hi].o, items[hi].c), cx.opt.tol,
                            work)) {
                dupOf[hi] = lo;
            }
            if (work > kMaxDuplicateComparisons) {
                budget = false;
                break;
            }
        }
    }
    if (!budget) {
        rep.truncated = true;
    }
    for (std::size_t i = 0; i < items.size(); ++i) {
        if (dupOf[i] != kNone) {
            push(rep, {IssueKind::DuplicateContour, items[i].o, items[i].c, 0, items[dupOf[i]].o,
                       items[dupOf[i]].c, true});
        }
    }
}

void detectSelfIntersections(Context& cx, Report& rep, const std::vector<bool>& skip) {
    for (std::size_t o = 0; o < cx.objs.size(); ++o) {
        if (o < skip.size() && skip[o]) {
            continue;
        }
        for (std::size_t c = 0; c < cx.contourCount(o); ++c) {
            if (!cx.usable(o, c) || cx.contour(o, c).spans.empty()) {
                continue;
            }
            const char st = cx.selfIntersectState(o, c);
            if (st == 1) {
                push(rep, {IssueKind::SelfIntersecting, o, c, 0, 0, 0, false});
            } else if (st == 3) {
                ++rep.skipped;
            }
        }
    }
}

void detectDirection(Context& cx, Report& rep) {
    const Tolerance& tol = cx.opt.tol;
    for (std::size_t o = 0; o < cx.objs.size(); ++o) {
        const std::size_t n = cx.contourCount(o);
        if (n < 1) {
            continue;
        }
        if (n > kMaxNestingContours) {
            ++rep.skipped;
            continue;
        }
        // Closed, simple contours take part; anything else has no reliable inside.
        std::vector<char> simple(n, 0);
        std::vector<Box2> boxes(n);
        for (std::size_t c = 0; c < n; ++c) {
            const Contour& k = cx.contour(o, c);
            if (cx.usable(o, c) && k.closed && !k.spans.empty() && cx.selfIntersectState(o, c) == 0) {
                simple[c] = 1;
                boxes[c] = k.bounds();
            }
        }
        for (std::size_t c = 0; c < n; ++c) {
            if (!simple[c]) {
                continue;
            }
            const Contour& k = cx.contour(o, c);
            const Orientation have = orientation(k, tol);
            if (have == Orientation::Degenerate) {
                continue;
            }
            const Vec2 p = pointAt(k.spans[0], 0.5);
            std::size_t depth = 0;
            for (std::size_t d = 0; d < n; ++d) {
                if (d == c || !simple[d] || !boxes[d].contains(p)) {
                    continue;
                }
                if (locatePoint(cx.contour(o, d), p, FillRule::EvenOdd, tol) == PointLocation::Inside) {
                    ++depth;
                }
            }
            const Orientation want = depth % 2 == 0 ? Orientation::CCW : Orientation::CW;
            if (have != want) {
                push(rep, {IssueKind::WrongDirection, o, c, 0, 0, 0, true});
            }
        }
    }
}

Report runAll(const Objects& objs, const Options& opt, const std::vector<bool>& skipIntersections) {
    Report rep;
    Context cx(objs, opt);
    if (cx.totalSpans > kMaxSpans) {
        rep.truncated = true;
        rep.skipped = objs.size();
        return rep;
    }
    detectInvalid(cx, rep);
    detectZeroLength(cx, rep);
    detectTiny(cx, rep);
    detectOpen(cx, rep);
    detectDuplicates(cx, rep);
    detectSelfIntersections(cx, rep, skipIntersections);
    detectDirection(cx, rep);
    return rep;
}

Objects viewOf(const std::vector<ObjectContours>& objects) {
    Objects v;
    v.reserve(objects.size());
    for (const ObjectContours& o : objects) {
        v.push_back(&o);
    }
    return v;
}

/// Runs one detector on the current state of `objects`.
template <typename Detect>
Report detectOne(const std::vector<ObjectContours>& objects, const Options& opt, Detect detect) {
    const Objects v = viewOf(objects);
    Context cx(v, opt);
    Report rep;
    if (cx.totalSpans > kMaxSpans) {
        rep.truncated = true;
        return rep;
    }
    detect(cx, rep);
    return rep;
}

/// Erases the contours named by `issues` (descending so indices stay valid).
std::size_t eraseContours(std::vector<ObjectContours>& objects, const std::vector<Issue>& issues) {
    std::vector<std::pair<std::size_t, std::size_t>> at;
    for (const Issue& i : issues) {
        at.emplace_back(i.object, i.contour);
    }
    std::sort(at.begin(), at.end(), std::greater<>());
    at.erase(std::unique(at.begin(), at.end()), at.end());
    for (const auto& [o, c] : at) {
        objects[o].erase(objects[o].begin() + static_cast<std::ptrdiff_t>(c));
    }
    return at.size();
}

}  // namespace

std::size_t Report::fixableCount() const {
    return static_cast<std::size_t>(
        std::count_if(issues.begin(), issues.end(), [](const Issue& i) { return i.fixable; }));
}

Report validate(const std::vector<const ObjectContours*>& objects, const Options& options,
                const std::vector<bool>& skipIntersections) {
    return runAll(objects, options, skipIntersections);
}

FixSummary fix(std::vector<ObjectContours>& objects, const Options& options, const FixFlags& flags) {
    FixSummary sum;
    if (flags.zeroLength) {
        const Report r = detectOne(objects, options, detectZeroLength);
        std::vector<Issue> sorted = r.issues;
        std::sort(sorted.begin(), sorted.end(), [](const Issue& a, const Issue& b) {
            return std::tie(a.object, a.contour, a.span) > std::tie(b.object, b.contour, b.span);
        });
        for (const Issue& i : sorted) {
            auto& spans = objects[i.object][i.contour].spans;
            spans.erase(spans.begin() + static_cast<std::ptrdiff_t>(i.span));
            ++sum.spansRemoved;
        }
        for (ObjectContours& o : objects) {
            o.erase(std::remove_if(o.begin(), o.end(), [](const Contour& c) { return c.spans.empty(); }), o.end());
        }
    }
    if (flags.tiny) {
        const Report r = detectOne(objects, options, detectTiny);
        sum.tinyRemoved = eraseContours(objects, r.issues);
    }
    if (flags.open) {
        const Report r = detectOne(objects, options, detectOpen);
        for (const Issue& i : r.issues) {
            if (!i.fixable) {
                continue;
            }
            Contour& c = objects[i.object][i.contour];
            const Vec2 from = c.endPoint();
            const Vec2 to = c.startPoint();
            if (!options.tol.equalPoints(from, to)) {
                c.spans.push_back(LineSpan{from, to});
            }
            c.closed = true;
            ++sum.contoursClosed;
        }
    }
    if (flags.duplicates) {
        const Report r = detectOne(objects, options, detectDuplicates);
        sum.duplicatesRemoved = eraseContours(objects, r.issues);
    }
    if (flags.direction) {
        const Report r = detectOne(objects, options, detectDirection);
        for (const Issue& i : r.issues) {
            objects[i.object][i.contour].reverse();
            ++sum.contoursReversed;
        }
    }
    return sum;
}

}  // namespace carvetto::geometry::validator
