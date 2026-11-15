#include "carvetto/toolpath/moulding.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/simulate/tool_shape.hpp"
#include "carvetto/toolpath/ramp_entry.hpp"

namespace carvetto::toolpath {

namespace {

using geometry::Polyline;
using geometry::Vec2;

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kEps = 1e-9;
constexpr int kTernaryIterations = 100;
constexpr double kMinStepFraction = 0.05;  // smallest plan step of a varied stepover, x stepover
constexpr std::size_t kMaxPasses = 2'000'000;
constexpr std::size_t kMaxLevels = 100'000;
constexpr double kMitreLimit = 4.0;

bool fin(double v) {
    return std::isfinite(v);
}

/// Maximum of H(t) - h(|t - u|) with the tool at offset u (see the header); `work` counts evaluations.
double dropHeightImpl(const std::vector<Vec2>& pts, const simulate::ToolShape& tool, double u, std::uint64_t& work) {
    const double R = tool.radius();
    const auto h = [&](double r) { return tool.heightAt(std::min(std::fabs(r), R)); };
    double best = -kInf;
    const std::size_t n = pts.size();
    // First segment whose end lies at or beyond u - R.
    std::size_t i = static_cast<std::size_t>(
        std::lower_bound(pts.begin(), pts.end(), u - R, [](const Vec2& a, double x) { return a.x < x; }) - pts.begin());
    i = i > 0 ? i - 1 : 0;
    for (; i + 1 < n; ++i) {
        const Vec2 a = pts[i];
        const Vec2 b = pts[i + 1];
        if (a.x > u + R) {
            break;
        }
        if (b.x < u - R) {
            continue;
        }
        if (!(b.x - a.x > 0.0)) {  // vertical step: both heights at one offset
            const double r = std::fabs(a.x - u);
            if (r <= R) {
                best = std::max({best, a.y - h(r), b.y - h(r)});
            }
            work += 2;
            continue;
        }
        const double slope = (b.y - a.y) / (b.x - a.x);
        const auto g = [&](double t) { return a.y + (t - a.x) * slope - h(t - u); };
        const double lo = std::max(a.x, u - R);
        const double hi = std::min(b.x, u + R);
        // H is linear and -h(|t - u|) concave on each side of u (h is convex), so g is concave there.
        const double pieces[2][2] = {{lo, std::min(hi, u)}, {std::max(lo, u), hi}};
        for (const auto& pc : pieces) {
            double l = pc[0];
            double r = pc[1];
            if (l > r) {
                continue;
            }
            best = std::max({best, g(l), g(r)});
            for (int it = 0; it < kTernaryIterations; ++it) {
                const double m1 = l + (r - l) / 3.0;
                const double m2 = r - (r - l) / 3.0;
                if (g(m1) < g(m2)) {
                    l = m1;
                } else {
                    r = m2;
                }
            }
            best = std::max(best, g(0.5 * (l + r)));
            work += 2 * kTernaryIterations + 3;
        }
    }
    return best;
}

/// The section after position, mirroring and boundary extension: ascending x = offset to the left.
struct Section {
    std::vector<Vec2> pts;
    double u0{0.0};
    double u1{0.0};
};

struct ToolPlan {
    simulate::ToolShape shape;
    double stepover{0.0};
    double passDepth{0.0};
    ToolSnapshot snapshot;
};

struct Pass {
    double u{0.0};
    double tip{0.0};  // machine Z the tip cuts at (rough: includes the allowance; clamped to the floor)
    bool flat{false};
};

struct FlatRun {
    double a;
    double b;
};

std::vector<FlatRun> flatRuns(const Section& s, double minWidth, double flatAngle) {
    std::vector<FlatRun> runs;
    const double tanA = std::tan(flatAngle);
    bool open = false;
    FlatRun cur{0.0, 0.0};
    const auto close = [&]() {
        if (open && cur.b - cur.a >= minWidth - kEps) {
            runs.push_back(cur);
        }
        open = false;
    };
    for (std::size_t i = 0; i + 1 < s.pts.size(); ++i) {
        const double dx = s.pts[i + 1].x - s.pts[i].x;
        const double dy = std::fabs(s.pts[i + 1].y - s.pts[i].y);
        if (dx > 1e-12 && dy <= dx * tanA) {
            if (!open) {
                cur = {s.pts[i].x, s.pts[i + 1].x};
                open = true;
            } else {
                cur.b = s.pts[i + 1].x;
            }
        } else {
            close();
        }
    }
    close();
    return runs;
}

/// Steepest |dy/dx| of the section over (ua, ub); infinity across a vertical step.
double maxSlope(const Section& s, double ua, double ub) {
    double m = 0.0;
    for (std::size_t i = 0; i + 1 < s.pts.size(); ++i) {
        const Vec2 a = s.pts[i];
        const Vec2 b = s.pts[i + 1];
        if (b.x < ua || a.x > ub) {
            continue;
        }
        const double dx = b.x - a.x;
        if (!(dx > 1e-12)) {
            if (a.x > ua && a.x < ub) {
                return kInf;
            }
            continue;
        }
        m = std::max(m, std::fabs(b.y - a.y) / dx);
    }
    return m;
}

/// Offsets of the passes across the section, ascending.
std::vector<double> passOffsets(const Section& s, double stepover, bool vary) {
    std::vector<double> us;
    us.push_back(s.u0);
    if (!(s.u1 - s.u0 > kEps)) {
        return us;
    }
    double u = s.u0;
    while (u < s.u1 - kEps && us.size() < kMaxPasses) {
        double step = stepover;
        if (vary) {
            const double m = maxSlope(s, u, u + stepover);
            step = std::isinf(m) ? stepover * kMinStepFraction : stepover / std::sqrt(1.0 + m * m);
            step = std::max(step, stepover * kMinStepFraction);
        }
        u = std::min(u + step, s.u1);
        us.push_back(u);
    }
    return us;
}

/// Chains of the rail offset to the left by `u` (negative: right), as polylines.
std::vector<Polyline> offsetChains(const Polyline& rail, double u, geometry::JoinType join,
                                   const geometry::Tolerance& tol) {
    std::vector<Polyline> out;
    if (std::fabs(u) < 1e-12) {
        out.push_back(rail);
        return out;
    }
    const geometry::Contour c = geometry::Contour::fromPolyline(rail);
    const auto chains = geometry::offsetOpenContour(
        c, std::fabs(u), u > 0.0 ? geometry::OffsetSide::Left : geometry::OffsetSide::Right, join, tol, kMitreLimit);
    for (const geometry::Contour& ch : chains) {
        Polyline pl = ch.polygonise(tol);
        pl.removeDuplicates(1e-9);
        if (pl.points.size() >= 2) {
            out.push_back(std::move(pl));
        }
    }
    return out;
}

class Emitter {
public:
    Emitter(Toolpath& tp, const MouldingParams& p, std::uint64_t moveLimit, const geometry::Tolerance& tol)
        : tp_(tp), p_(p), moveLimit_(moveLimit), tol_(tol) {}

    bool overLimit() const { return tp_.moves.size() > moveLimit_; }

    /// Cuts `path` at constant `z`; with `zFrom` (> z) the tool enters by a ramp starting at `zFrom`.
    void chain(const Polyline& path, double z, std::optional<double> zFrom) {
        const Vec2 start = path.points.front();
        tp_.append(Move::rapid({start, p_.safeZ}));
        if (zFrom && *zFrom > z + kEps) {
            tp_.append(Move::plunge({start, *zFrom}));
            const double len = (*zFrom - z) / std::tan(p_.rampAngle);
            appendPathRamp(path, *zFrom, z, len, 1e-6, tol_, tp_);
        } else {
            tp_.append(Move::plunge({start, z}));
        }
        Vec2 pos = start;
        const auto feedTo = [&](Vec2 q) {
            if (geometry::distance(q, pos) > 0.0) {
                tp_.append(Move::feed({q, z}));
                pos = q;
            }
        };
        for (std::size_t i = 1; i < path.points.size(); ++i) {
            feedTo(path.points[i]);
        }
        if (path.closed) {
            feedTo(start);
        }
        tp_.append(Move::retract({pos, p_.safeZ}));
    }

private:
    Toolpath& tp_;
    const MouldingParams& p_;
    std::uint64_t moveLimit_;
    geometry::Tolerance tol_;
};

}  // namespace

double mouldingDropHeight(const std::vector<geometry::Vec2>& profile, const simulate::ToolShape& tool, double u) {
    std::uint64_t work = 0;
    return dropHeightImpl(profile, tool, u, work);
}

MouldingResult mouldingToolpath(const Rough3dTool& tool, const MouldingParams& p, const ToolpathRunControl& control) {
    MouldingResult res;
    const bool rough = p.mode == MouldingMode::Rough;
    const bool useClear = rough && p.clearTool.has_value();
    const std::size_t nTools = useClear ? 2 : 1;
    res.toolpaths.resize(nTools);
    Toolpath& main = res.toolpaths[nTools - 1];
    main.name = p.name;
    if (useClear) {
        res.toolpaths[0].name = p.name + " [Clear]";
    }
    const auto diag = [&](ToolpathIssue issue, const std::string& msg, std::optional<std::size_t> contour = {},
                          std::optional<std::size_t> toolIdx = {}) {
        res.diagnostics.push_back({issue, contour, msg, toolIdx});
    };
    const auto refuse = [&](ToolpathIssue issue, const std::string& msg) {
        for (Toolpath& t : res.toolpaths) {
            t.moves.clear();
        }
        diag(issue, msg);
        return res;
    };
    const auto cancelled = [&]() {
        return control.cancel != nullptr && control.cancel->load(std::memory_order_relaxed);
    };

    // Parameters.
    if (!fin(p.zOffset) || !fin(p.stockTop) || !fin(p.stockBottom) || !fin(p.safeZ) || !fin(p.allowance) ||
        !fin(p.boundaryOffset) || !fin(p.flatAngle) || !fin(p.rampAngle) || !fin(p.restTolerance) ||
        !fin(p.tolerance)) {
        return refuse(ToolpathIssue::InvalidParameters, "a parameter is not a finite number");
    }
    if (!(p.stockBottom < p.stockTop) || !(p.safeZ > p.stockTop) || p.allowance < 0.0 || p.boundaryOffset < 0.0 ||
        p.restTolerance < 0.0 || !(p.tolerance > 0.0) || !(p.flatAngle >= 0.0 && p.flatAngle < 1.5) ||
        !(p.rampAngle > 0.0 && p.rampAngle < 1.5)) {
        return refuse(ToolpathIssue::InvalidParameters,
                      "stock bottom < top < safe Z, allowance >= 0, boundary offset >= 0, tolerance > 0, flat and "
                      "ramp angles in range are required");
    }
    if (p.rails.empty()) {
        return refuse(ToolpathIssue::InvalidParameters, "no drive rail");
    }
    for (const Vec2& q : p.profile.points) {
        if (!fin(q.x) || !fin(q.y)) {
            return refuse(ToolpathIssue::InvalidParameters, "the profile has a non-finite point");
        }
    }
    if (p.profile.closed || p.profile.points.size() < 2) {
        return refuse(ToolpathIssue::InvalidParameters, "the profile must be an open vector of at least two points");
    }

    // Tools.
    const auto prepare = [&](const Rough3dTool& rt, ToolPlan& tp, std::size_t idx) {
        const auto sh = simulate::ToolShape::fromTool(rt.tool, rt.fluteLength);
        if (!sh) {
            diag(ToolpathIssue::InvalidTool, "tool '" + rt.tool.name + "' has no usable shape", {}, idx);
            return false;
        }
        const tooling::CuttingData& cd = effectiveCuttingData(rt.tool, rt.cuttingData);
        if (!(cd.stepover > 0.0) || !fin(cd.stepover) || !(cd.passDepth > 0.0) || !fin(cd.passDepth)) {
            diag(ToolpathIssue::InvalidTool, "tool '" + rt.tool.name + "' needs a stepover and pass depth > 0", {}, idx);
            return false;
        }
        tp.shape = *sh;
        tp.stepover = cd.stepover;
        tp.passDepth = cd.passDepth;
        tp.snapshot = toolSnapshot(rt.tool, cd);
        if (tp.stepover > sh->diameter()) {
            tp.stepover = sh->diameter();
            diag(ToolpathIssue::StepoverLimited, "the stepover was limited to the tool diameter", {}, idx);
        }
        return true;
    };
    ToolPlan mainTool;
    ToolPlan clearPlan;
    if (!prepare(tool, mainTool, nTools - 1)) {
        return refuse(ToolpathIssue::InvalidTool, "the tool cannot be used");
    }
    if (useClear && !prepare(*p.clearTool, clearPlan, 0)) {
        return refuse(ToolpathIssue::InvalidTool, "the clearance tool cannot be used");
    }
    main.tool = mainTool.snapshot;
    if (useClear) {
        res.toolpaths[0].tool = clearPlan.snapshot;
    }

    // Section: monotone in x (as drawn or reversed), positioned, mirrored, extended.
    std::vector<Vec2> drawn = p.profile.points;
    bool asc = true;
    bool desc = true;
    for (std::size_t i = 1; i < drawn.size(); ++i) {
        asc = asc && drawn[i].x >= drawn[i - 1].x;
        desc = desc && drawn[i].x <= drawn[i - 1].x;
    }
    double xmin = drawn.front().x;
    double xmax = drawn.front().x;
    for (const Vec2& q : drawn) {
        xmin = std::min(xmin, q.x);
        xmax = std::max(xmax, q.x);
    }
    if ((!asc && !desc) || !(xmax - xmin > kEps)) {
        return refuse(ToolpathIssue::InvalidParameters,
                      "the profile must be single valued across the rail (x never turns back) and have a width");
    }
    double anchor = 0.0;
    switch (p.position) {
        case MouldingPosition::AsDrawn: anchor = 0.0; break;
        case MouldingPosition::Start: anchor = drawn.front().x; break;
        case MouldingPosition::Centre: anchor = 0.5 * (xmin + xmax); break;
        case MouldingPosition::End: anchor = drawn.back().x; break;
    }
    Section sec;
    sec.pts.reserve(drawn.size() + 2);
    for (const Vec2& q : drawn) {
        sec.pts.push_back({p.reverseProfile ? anchor - q.x : q.x - anchor, q.y});
    }
    if (p.reverseProfile == asc) {
        // Ascending-after-mirroring needs the point order reversed exactly when the sign of the walk flips.
        std::reverse(sec.pts.begin(), sec.pts.end());
    }
    sec.u0 = sec.pts.front().x;
    sec.u1 = sec.pts.back().x;
    const double boundary = p.autoBoundaryOffset ? mainTool.shape.radius() : p.boundaryOffset;
    if (boundary > 0.0) {
        sec.pts.insert(sec.pts.begin(), Vec2{sec.u0 - boundary, sec.pts.front().y});
        sec.pts.push_back({sec.u1 + boundary, sec.pts.back().y});
        sec.u0 -= boundary;
        sec.u1 += boundary;
    }

    geometry::Tolerance tol = geometry::Tolerance::defaults();
    tol.chordal = p.tolerance;
    tol.distance = std::min(tol.distance, 0.5 * p.tolerance);
    const geometry::JoinType join = p.sharpCorners ? geometry::JoinType::Sharp : geometry::JoinType::Round;
    const std::uint64_t workLimit = control.workLimit > 0 ? control.workLimit : kMouldingDefaultWorkLimit;
    const std::uint64_t moveLimit = control.moveLimit > 0 ? control.moveLimit : kMouldingDefaultMoveLimit;
    std::uint64_t work = 0;

    // Rails.
    std::vector<Polyline> rails;
    for (std::size_t i = 0; i < p.rails.size(); ++i) {
        Polyline r = p.rails[i];
        bool ok = true;
        for (const Vec2& q : r.points) {
            ok = ok && fin(q.x) && fin(q.y);
        }
        r.removeDuplicates(1e-9);
        if (!ok || r.points.size() < 2) {
            diag(ToolpathIssue::EmptyContour, "rail " + std::to_string(i) + " has no length and was skipped", i);
            continue;
        }
        if (p.reverseRails) {
            r.reverse();
        }
        rails.push_back(std::move(r));
    }

    // Passes of one tool: offsets, tip heights, flat flags. Ordered as they are cut.
    enum class Fail { None, Work, Cancel };
    Fail fail = Fail::None;
    const auto makePasses = [&](const ToolPlan& tp, bool vary, bool addAllowance) {
        std::vector<Pass> out;
        const std::vector<double> us = passOffsets(sec, tp.stepover, vary);
        const std::vector<FlatRun> runs = flatRuns(sec, tp.stepover, p.flatAngle);
        for (const double u : us) {
            if (cancelled()) {
                fail = Fail::Cancel;
                return out;
            }
            double d = dropHeightImpl(sec.pts, tp.shape, u, work);
            if (work > workLimit || us.size() >= kMaxPasses) {
                fail = Fail::Work;
                return out;
            }
            d += p.zOffset + (addAllowance ? p.allowance : 0.0);
            Pass ps;
            ps.u = u;
            ps.tip = std::max(d, p.stockBottom);
            for (const FlatRun& fr : runs) {
                ps.flat = ps.flat || (u > fr.a + kEps && u < fr.b - kEps);
            }
            out.push_back(ps);
        }
        // Climb: the next pass lies on the right of the travel direction (towards -u).
        const bool descending = (p.direction == CutDirection::Climb) != p.reverseSteps;
        if (descending) {
            std::reverse(out.begin(), out.end());
        }
        return out;
    };
    const auto failure = [&]() {
        return refuse(fail == Fail::Cancel ? ToolpathIssue::Cancelled : ToolpathIssue::WorkLimitExceeded,
                      fail == Fail::Cancel ? "cancelled" : "the moulding needs more work than allowed");
    };

    const auto cutRails = [&](Toolpath& tp, const ToolPlan& plan, const std::vector<Pass>& passes,
                              const std::vector<double>* restTop /* per pass, rough */, bool honourFlat) {
        Emitter em(tp, p, moveLimit, tol);
        const double eps = restTop != nullptr && useClear ? p.restTolerance : kEps;
        for (const Polyline& rail : rails) {
            std::vector<std::vector<Polyline>> cache(passes.size());
            std::vector<bool> have(passes.size(), false);
            const auto chainsOf = [&](std::size_t k) -> const std::vector<Polyline>& {
                if (!have[k]) {
                    cache[k] = offsetChains(rail, passes[k].u, join, tol);
                    have[k] = true;
                }
                return cache[k];
            };
            const auto runPass = [&](std::size_t k, std::size_t parity, double z, std::optional<double> zFrom) {
                for (Polyline c : chainsOf(k)) {
                    if (p.travel == RasterTravel::ZigZag && (parity & 1U) != 0U) {
                        c.reverse();
                    }
                    em.chain(c, z, zFrom);
                }
                if (cancelled()) {
                    fail = Fail::Cancel;
                }
                if (em.overLimit()) {
                    fail = Fail::Work;
                }
            };
            if (!rough) {
                std::size_t parity = 0;
                for (std::size_t k = 0; k < passes.size() && fail == Fail::None; ++k) {
                    if ((honourFlat && p.skipFlatRegions && passes[k].flat) || !(passes[k].tip < p.stockTop - kEps)) {
                        continue;
                    }
                    runPass(k, parity++, passes[k].tip, std::nullopt);
                }
                continue;
            }
            double minTip = p.stockTop;
            for (std::size_t k = 0; k < passes.size(); ++k) {
                if (honourFlat && !p.machineFlatRegions && passes[k].flat) {
                    continue;
                }
                minTip = std::min(minTip, passes[k].tip);
            }
            const double levelsD = std::ceil((p.stockTop - minTip) / plan.passDepth);
            if (levelsD > static_cast<double>(kMaxLevels)) {
                fail = Fail::Work;
                return;
            }
            const std::size_t levels = static_cast<std::size_t>(std::max(levelsD, 0.0));
            for (std::size_t n = 1; n <= levels && fail == Fail::None; ++n) {
                const double top = p.stockTop - static_cast<double>(n - 1) * plan.passDepth;
                const double lvl = p.stockTop - static_cast<double>(n) * plan.passDepth;
                std::size_t parity = 0;
                for (std::size_t k = 0; k < passes.size() && fail == Fail::None; ++k) {
                    if (honourFlat && !p.machineFlatRegions && passes[k].flat) {
                        continue;
                    }
                    const double above = std::min(top, restTop != nullptr ? (*restTop)[k] : top);
                    if (!(passes[k].tip < above - eps)) {
                        continue;
                    }
                    const double z = std::max(lvl, passes[k].tip);
                    runPass(k, parity++, z, p.rampPlunge ? std::optional<double>(above) : std::nullopt);
                }
                work += passes.size();
                if (work > workLimit) {
                    fail = Fail::Work;
                }
            }
        }
    };

    if (rails.empty()) {
        return refuse(ToolpathIssue::InvalidParameters, "no usable drive rail");
    }

    if (!rough) {
        const std::vector<Pass> passes = makePasses(mainTool, p.varyStepover, false);
        if (fail != Fail::None) {
            return failure();
        }
        cutRails(main, mainTool, passes, nullptr, true);
        if (fail != Fail::None) {
            return failure();
        }
    } else {
        std::vector<double> restTop;
        const std::vector<Pass> mainPasses = makePasses(mainTool, false, true);
        if (fail != Fail::None) {
            return failure();
        }
        if (useClear) {
            const std::vector<Pass> clearPasses = makePasses(clearPlan, false, true);
            if (fail != Fail::None) {
                return failure();
            }
            cutRails(res.toolpaths[0], clearPlan, clearPasses, nullptr, false);
            if (fail != Fail::None) {
                return failure();
            }
            // Stock left by the clearance tool at each main pass offset: its passes cut down to their tip.
            std::vector<Pass> cut;
            for (const Pass& c : clearPasses) {
                if (c.tip < p.stockTop - kEps) {
                    cut.push_back(c);
                }
            }
            std::sort(cut.begin(), cut.end(), [](const Pass& a, const Pass& b) { return a.u < b.u; });
            const double reach = clearPlan.shape.radius();
            for (const Pass& m : mainPasses) {
                double left = p.stockTop;
                auto it = std::lower_bound(cut.begin(), cut.end(), m.u - reach,
                                           [](const Pass& a, double x) { return a.u < x; });
                for (; it != cut.end() && it->u <= m.u + reach; ++it) {
                    const double r = std::min(std::fabs(m.u - it->u), reach);
                    left = std::min(left, it->tip + clearPlan.shape.heightAt(r));
                }
                restTop.push_back(left);
            }
        }
        cutRails(main, mainTool, mainPasses, useClear ? &restTop : nullptr, true);
        if (fail != Fail::None) {
            return failure();
        }
    }

    bool any = false;
    for (const Toolpath& t : res.toolpaths) {
        any = any || !t.moves.empty();
    }
    if (!any) {
        diag(ToolpathIssue::NoPasses, "nothing to cut: the section lies at or above the stock top, or all passes were skipped");
    }
    return res;
}

}  // namespace carvetto::toolpath
