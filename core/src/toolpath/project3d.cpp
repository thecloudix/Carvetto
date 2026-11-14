#include "carvetto/toolpath/project3d.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <new>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/simulate/tool_shape.hpp"
#include "carvetto/toolpath/path_walk.hpp"
#include "carvetto/toolpath/rough3d.hpp"

namespace carvetto::toolpath {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kPi = 3.14159265358979323846;
constexpr int kMaxDepth = 0;
constexpr double kEps = 1e-9;

/// Bilinear sampler of a row-major node grid.
struct Surf {
    const float* v{nullptr};
    std::size_t w{0};
    std::size_t h{0};
    double cs{1.0};
    double ox{0.0};
    double oy{0.0};

    /// False outside the node rectangle or when a node with weight is not finite.
    bool at(double x, double y, double& z) const {
        const double fx = (x - ox) / cs;
        const double fy = (y - oy) / cs;
        if (!(fx >= 0.0 && fy >= 0.0 && fx <= static_cast<double>(w - 1) && fy <= static_cast<double>(h - 1))) {
            return false;
        }
        const std::size_t i0 = std::min(static_cast<std::size_t>(fx), w - 1);
        const std::size_t j0 = std::min(static_cast<std::size_t>(fy), h - 1);
        const std::size_t i1 = std::min(i0 + 1, w - 1);
        const std::size_t j1 = std::min(j0 + 1, h - 1);
        const double tx = i1 == i0 ? 0.0 : fx - static_cast<double>(i0);
        const double ty = j1 == j0 ? 0.0 : fy - static_cast<double>(j0);
        double sum = 0.0;
        const auto add = [&](std::size_t i, std::size_t j, double wt) {
            if (wt == 0.0) {
                return true;
            }
            const float f = v[j * w + i];
            if (!std::isfinite(f)) {
                return false;
            }
            sum += wt * static_cast<double>(f);
            return true;
        };
        if (!add(i0, j0, (1.0 - tx) * (1.0 - ty)) || !add(i1, j0, tx * (1.0 - ty)) || !add(i0, j1, (1.0 - tx) * ty) ||
            !add(i1, j1, tx * ty)) {
            return false;
        }
        z = sum;
        return true;
    }
};

struct Mapper {
    Surf surf;
    double offset{0.0};  // added to the sampled value (zOffset for Project; the drop surface has it in)
    double allowance{0.0};
    double outside{0.0};
    double floorZ{0.0};

    double surfaceAt(Vec2 xy) const {
        double s = 0.0;
        if (surf.at(xy.x, xy.y, s)) {
            return s + offset + allowance;
        }
        return outside;
    }
    Vec3 map(const Vec3& p) const {
        if (p.z > 0.0) {
            return p;
        }
        return {p.x, p.y, std::max(surfaceAt({p.x, p.y}) + p.z, floorZ)};
    }
};

void diag(Project3dResult& r, ToolpathIssue issue, std::string msg) {
    ToolpathDiagnostic d;
    d.issue = issue;
    d.message = std::move(msg);
    r.diagnostics.push_back(std::move(d));
}

bool finite3(const Vec3& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

struct RunPt {
    Vec3 p;
    std::optional<double> fo;
};

struct Breakpoint {
    double s{0.0};
    bool vertex{false};
    std::size_t k{0};  // vertex index
};

/// Raises one run of Feed moves (vertices with strictly increasing XY arc length) over its tabs.
/// Returns the replacement moves (excluding the start vertex) or false when nothing changed.
bool tabRun(const std::vector<RunPt>& run, const Mapper& mp, double zMinIn, const TabOptions& tabs, double toolDiameter,
            std::vector<Move>& outMoves, bool& notPlaced) {
    const std::size_t n = run.size();
    if (n < 2) {
        return false;
    }
    std::vector<double> s(n, 0.0);
    geometry::Polyline pl;
    pl.points.push_back({run[0].p.x, run[0].p.y});
    for (std::size_t k = 1; k < n; ++k) {
        s[k] = s[k - 1] + std::hypot(run[k].p.x - run[k - 1].p.x, run[k].p.y - run[k - 1].p.y);
        pl.points.push_back({run[k].p.x, run[k].p.y});
    }
    const double total = s[n - 1];
    if (!(total > 1e-6)) {
        return false;
    }
    const ArcLengthPath path(geometry::Contour::fromPolyline(pl), geometry::Tolerance::defaults());
    const double raised = tabs.length + toolDiameter;
    const TabPlacement placed = placeTabs(path, raised, tabs);
    if (placed.dropped > 0 || placed.notAvoided > 0) {
        notPlaced = true;
    }
    if (placed.centres.empty()) {
        return false;
    }
    struct Span {
        double a, b;
    };
    std::vector<Span> spans;
    for (const double c : placed.centres) {
        spans.push_back({std::max(0.0, c - 0.5 * raised), std::min(total, c + 0.5 * raised)});
    }
    const double th = tabs.thickness;
    // Breakpoints: vertices plus the places where the ceiling changes.
    std::vector<Breakpoint> bps;
    for (std::size_t k = 0; k < n; ++k) {
        bps.push_back({s[k], true, k});
    }
    std::vector<double> knots;
    for (const Span& sp : spans) {
        knots.push_back(sp.a);
        knots.push_back(sp.b);
        if (tabs.threeD) {
            knots.push_back(sp.a - th);
            knots.push_back(sp.b + th);
        }
    }
    std::sort(knots.begin(), knots.end());
    for (const double kn : knots) {
        if (!(kn > kEps && kn < total - kEps)) {
            continue;
        }
        const auto it = std::lower_bound(s.begin(), s.end(), kn - kEps);
        if (it != s.end() && *it <= kn + kEps) {
            continue;  // at a vertex
        }
        if (!bps.empty() && !bps.back().vertex && std::fabs(bps.back().s - kn) <= kEps) {
            continue;
        }
        bps.push_back({kn, false, 0});
    }
    std::sort(bps.begin(), bps.end(), [](const Breakpoint& x, const Breakpoint& y) {
        return x.s != y.s ? x.s < y.s : (x.vertex && !y.vertex);
    });
    // Position and unraised height at a breakpoint.
    const auto locate = [&](const Breakpoint& b, Vec3& pt, std::optional<double>& fo) {
        if (b.vertex) {
            pt = run[b.k].p;
            fo = run[b.k].fo;
            return;
        }
        const std::size_t k1 = static_cast<std::size_t>(std::upper_bound(s.begin(), s.end(), b.s) - s.begin());
        const std::size_t k0 = k1 - 1;
        const double t = (b.s - s[k0]) / (s[k1] - s[k0]);
        pt = {run[k0].p.x + t * (run[k1].p.x - run[k0].p.x), run[k0].p.y + t * (run[k1].p.y - run[k0].p.y),
              run[k0].p.z + t * (run[k1].p.z - run[k0].p.z)};
        fo = run[k1].fo;
    };
    // Ceiling just left (side < 0) or right (side > 0) of s.
    const auto ceilingAt = [&](double sv, int side, double bottom) {
        double c = -kInf;
        for (const Span& sp : spans) {
            if (tabs.threeD) {
                double d = 0.0;
                if (sv < sp.a) {
                    d = sp.a - sv;
                } else if (sv > sp.b) {
                    d = sv - sp.b;
                }
                if (d < th) {
                    c = std::max(c, bottom + th - d);
                }
            } else {
                const bool atA = std::fabs(sv - sp.a) <= kEps;
                const bool atB = std::fabs(sv - sp.b) <= kEps;
                bool in = false;
                if (atA) {
                    in = side > 0;
                } else if (atB) {
                    in = side < 0;
                } else {
                    in = sv > sp.a && sv < sp.b;
                }
                if (in) {
                    c = std::max(c, bottom + th);
                }
            }
        }
        return c;
    };
    bool changed = false;
    outMoves.clear();
    for (const Breakpoint& b : bps) {
        if (b.vertex && b.k == 0) {
            continue;
        }
        Vec3 pt;
        std::optional<double> fo;
        locate(b, pt, fo);
        const double bottom = std::max(mp.surfaceAt({pt.x, pt.y}) + zMinIn, mp.floorZ);
        const double zl = std::max(pt.z, ceilingAt(b.s, -1, bottom));
        const double zr = std::max(pt.z, ceilingAt(b.s, +1, bottom));
        const bool last = b.s >= total - kEps;
        outMoves.push_back(Move::feed({pt.x, pt.y, zl}, fo));
        if (!last && zr != zl) {
            outMoves.push_back(Move::feed({pt.x, pt.y, zr}, fo));
        }
        if (zl != pt.z || zr != pt.z) {
            changed = true;
        }
    }
    return changed;
}

}  // namespace

Project3dResult projectToolpath(const Toolpath& in, const relief::HeightField& model, const tooling::Tool& tool,
                                const Project3dParams& p, const ToolpathRunControl& control) {
    Project3dResult res;
    auto refuse = [&](ToolpathIssue issue, const std::string& msg) {
        diag(res, issue, msg);
        res.path = Toolpath{};
        return res;
    };
    auto fin = [](double v) { return std::isfinite(v); };
    if (model.empty() || relief::validate(model.grid()) != relief::Status::Ok) {
        return refuse(ToolpathIssue::InvalidParameters, "the model has no valid height field");
    }
    if (!fin(p.zOffset) || !fin(p.allowance) || !fin(p.outsideSurface) || !fin(p.stockBottom) ||
        !fin(p.tolerance) || !fin(p.maxSegment)) {
        return refuse(ToolpathIssue::InvalidParameters, "a parameter is not finite");
    }
    if (p.allowance < 0.0 || !(p.tolerance > 0.0) || p.maxSegment < 0.0) {
        return refuse(ToolpathIssue::InvalidParameters, "allowance, tolerance or maximum segment out of range");
    }
    if (p.tabs.enabled) {
        if (!fin(p.tabs.length) || !fin(p.tabs.thickness) || !fin(p.tabs.distance) || p.tabs.length < 0.0 ||
            !(p.tabs.thickness > 0.0) ||
            (p.tabs.spacing == TabSpacing::ConstantNumber ? p.tabs.count < 1 : !(p.tabs.distance > 0.0))) {
            return refuse(ToolpathIssue::InvalidParameters, "tab length, thickness, count or distance out of range");
        }
    }
    for (const Move& m : in.moves) {
        if (!finite3(m.to) || (isArc(m.kind) && !(fin(m.centre.x) && fin(m.centre.y)))) {
            return refuse(ToolpathIssue::InvalidParameters, "a move has a non-finite coordinate");
        }
    }
    const double cs = model.cellSize();
    const double step = p.maxSegment > 0.0 ? p.maxSegment : cs;
    const std::uint64_t workLimit = control.workLimit != 0 ? control.workLimit : kProject3dDefaultWorkLimit;
    const std::uint64_t moveLimit = control.moveLimit != 0 ? control.moveLimit : kProject3dDefaultMoveLimit;
    const auto cancelled = [&]() { return control.cancel != nullptr && control.cancel->load(std::memory_order_relaxed); };

    Mapper mp;
    mp.allowance = p.allowance;
    mp.outside = p.outsideSurface;
    mp.floorZ = p.stockBottom;
    std::vector<float> drop;
    if (p.mode == ProjectTipMode::DropTool) {
        const auto shape = simulate::ToolShape::fromTool(tool);
        if (!shape) {
            return refuse(ToolpathIssue::InvalidTool, "the tool has no usable shape");
        }
        std::uint64_t work = 0;
        const DropStatus st = dropToolSurface(model, *shape, p.zOffset, drop, workLimit, work, control.cancel);
        if (st == DropStatus::Cancelled) {
            return refuse(ToolpathIssue::Cancelled, "cancelled");
        }
        if (st != DropStatus::Ok) {
            return refuse(ToolpathIssue::WorkLimitExceeded, "the drop-tool surface needs more work than allowed");
        }
        mp.surf = {drop.data(), model.width(), model.height(), cs, model.origin().x, model.origin().y};
        mp.offset = 0.0;
    } else {
        mp.surf = {model.data().data(), model.width(), model.height(), cs, model.origin().x, model.origin().y};
        mp.offset = p.zOffset;
    }

    Toolpath& out = res.path;
    out.name = p.name.empty() ? in.name + " (3D)" : p.name;
    out.tool = in.tool;
    std::uint64_t work = 0;
    bool over = false;
    const auto emit = [&](MoveKind k, const Vec3& to, const std::optional<double>& fo) {
        if (out.moves.size() >= moveLimit) {
            over = true;
            return;
        }
        Move m;
        m.kind = isArc(k) ? MoveKind::Feed : k;
        m.to = to;
        m.feedOverride = fo;
        out.moves.push_back(m);
    };
    double zMinIn = 0.0;
    Vec3 inPos{};
    Vec3 outPos{};
    bool haveInPos = false;
    for (const Move& m : in.moves) {
        if (cancelled()) {
            return refuse(ToolpathIssue::Cancelled, "cancelled");
        }
        if (over || work > workLimit) {
            break;
        }
        if (!haveInPos) {
            inPos = m.to;
            outPos = (m.kind == MoveKind::Rapid || m.kind == MoveKind::Dwell) ? m.to : mp.map(m.to);
            haveInPos = true;
        }
        if (m.kind == MoveKind::Rapid) {
            Move r = m;
            out.moves.push_back(r);
            inPos = m.to;
            outPos = m.to;
            continue;
        }
        if (m.kind == MoveKind::Dwell) {
            Move d = m;
            d.to = outPos;
            out.moves.push_back(d);
            continue;
        }
        zMinIn = std::min(zMinIn, m.to.z);
        // Parametrise the input move: P(t), t in [0, 1].
        const Vec3 a = inPos;
        const Vec3 b = m.to;
        const bool arc = isArc(m.kind);
        double sweep = 0.0;
        double ang0 = 0.0;
        double rad = 0.0;
        double xyLen = std::hypot(b.x - a.x, b.y - a.y);
        if (arc) {
            rad = arcRadius(m, {a.x, a.y});
            sweep = arcSweep(m, {a.x, a.y});
            ang0 = std::atan2(a.y - m.centre.y, a.x - m.centre.x);
            xyLen = std::fabs(sweep) * rad;
        }
        const auto P = [&](double t) {
            if (t >= 1.0) {
                return b;
            }
            if (arc) {
                const double ang = ang0 + t * sweep;
                return Vec3{m.centre.x + rad * std::cos(ang), m.centre.y + rad * std::sin(ang), a.z + t * (b.z - a.z)};
            }
            return Vec3{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.z + t * (b.z - a.z)};
        };
        double nPieces = std::ceil(xyLen / step);
        if (arc && rad > 0.0) {
            const double cosv = std::clamp(1.0 - p.tolerance / rad, -1.0, 1.0);
            const double dth = std::max(2.0 * std::acos(cosv), 1e-3);
            nPieces = std::max(nPieces, std::ceil(std::fabs(sweep) / dth));
        }
        nPieces = std::max(nPieces, 1.0);
        if (!(nPieces <= static_cast<double>(moveLimit))) {
            return refuse(ToolpathIssue::WorkLimitExceeded, "a move would need more pieces than allowed");
        }
        const std::size_t n = static_cast<std::size_t>(nPieces);
        const double minLen = cs / 16.0;
        // Recursive refinement of [t0, t1]: emits the interior points.
        std::function<void(double, const Vec3&, double, const Vec3&, int)> refine =
            [&](double t0, const Vec3& r0, double t1, const Vec3& r1, int depth) {
                if (depth >= kMaxDepth || (t1 - t0) * xyLen <= minLen) {
                    return;
                }
                const double tm = 0.5 * (t0 + t1);
                const Vec3 qm = mp.map(P(tm));
                ++work;
                if (std::fabs(qm.z - 0.5 * (r0.z + r1.z)) > p.tolerance) {
                    refine(t0, r0, tm, qm, depth + 1);
                    emit(m.kind, qm, m.feedOverride);
                    refine(tm, qm, t1, r1, depth + 1);
                }
            };
        Vec3 q0 = outPos;
        for (std::size_t i = 1; i <= n && !over; ++i) {
            const double t0 = static_cast<double>(i - 1) / static_cast<double>(n);
            const double t1 = static_cast<double>(i) / static_cast<double>(n);
            const Vec3 q1 = mp.map(P(t1));
            ++work;
            refine(t0, q0, t1, q1, 0);
            emit(m.kind, q1, m.feedOverride);
            q0 = q1;
        }
        outPos = q0;
        inPos = b;
    }
    if (over) {
        return refuse(ToolpathIssue::WorkLimitExceeded, "the toolpath would exceed the move limit");
    }
    if (work > workLimit) {
        return refuse(ToolpathIssue::WorkLimitExceeded, "the projection needs more work than allowed");
    }

    if (p.tabs.enabled) {
        if (!(zMinIn < 0.0)) {
            diag(res, ToolpathIssue::TabsNotPlaced, "the toolpath does not cut below the surface: no tabs placed");
        } else {
            const auto isRunMove = [&](std::size_t i) {
                if (out.moves[i].kind != MoveKind::Feed) {
                    return false;
                }
                const Vec3 from = out.startOf(i);
                return out.moves[i].to.x != from.x || out.moves[i].to.y != from.y;
            };
            std::vector<Move> rebuilt;
            rebuilt.reserve(out.moves.size());
            bool notPlaced = false;
            std::size_t i = 0;
            while (i < out.moves.size()) {
                if (!isRunMove(i)) {
                    rebuilt.push_back(out.moves[i]);
                    ++i;
                    continue;
                }
                std::size_t j = i;
                std::vector<RunPt> run;
                run.push_back({out.startOf(i), std::nullopt});
                while (j < out.moves.size() && isRunMove(j)) {
                    run.push_back({out.moves[j].to, out.moves[j].feedOverride});
                    ++j;
                }
                std::vector<Move> repl;
                if (tabRun(run, mp, zMinIn, p.tabs, in.tool.diameter, repl, notPlaced)) {
                    rebuilt.insert(rebuilt.end(), repl.begin(), repl.end());
                } else {
                    rebuilt.insert(rebuilt.end(), out.moves.begin() + static_cast<std::ptrdiff_t>(i),
                                   out.moves.begin() + static_cast<std::ptrdiff_t>(j));
                }
                i = j;
            }
            out.moves = std::move(rebuilt);
            if (notPlaced) {
                diag(res, ToolpathIssue::TabsNotPlaced, "some tabs could not be placed");
            }
        }
    }
    return res;
}

}  // namespace carvetto::toolpath
