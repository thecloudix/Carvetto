#include "carvetto/geometry/nesting.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <tuple>

#include "carvetto/geometry/cancel.hpp"

namespace carvetto::geometry {

namespace {

constexpr double kEps = 1e-9;
constexpr double kMinGapMm = 0.001;
constexpr int kSlideRounds = 2;
constexpr int kSlideBisections = 14;

struct Ring {
    std::vector<Vec2> pts;
    Box2 box;
};

/// One orientation of a part in the packing frame (reflected for the start corner).
struct Orientation {
    std::vector<Ring> rings;  // [0] outer, then holes (only with allowInside)
    Box2 box;                 // min at the origin
    double w{0.0};
    double h{0.0};
    Transform2 base;  // input -> oriented, bounding-box minimum at the origin (actual frame)
    double angle{0.0};
    bool mirrored{false};
};

struct PartData {
    bool valid{false};
    double area{0.0};
    std::vector<Orientation> orientations;
};

struct Placed {
    const Orientation* o{nullptr};
    Vec2 pos;
    Box2 box;  // absolute bounding box in the packing frame
};

struct Region {
    double x0{0}, y0{0}, x1{0}, y1{0};
    bool hasBoundary{false};
    std::vector<Ring> rings;  // reflected boundary polygons
};

struct SheetState {
    Region region;
    std::vector<Placed> placed;
};

struct Ctx {
    std::uint64_t ops{0};
    std::uint64_t maxOps{0};
    bool exhausted{false};
    bool cancelled{false};
    bool stopped() const { return exhausted || cancelled; }
    void charge(std::uint64_t n) {
        ops += n;
        if (ops > maxOps) exhausted = true;
    }
};

double cross3(Vec2 a, Vec2 b, Vec2 c) { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); }

bool properCross(Vec2 p1, Vec2 p2, Vec2 q1, Vec2 q2) {
    const double d1 = cross3(q1, q2, p1);
    const double d2 = cross3(q1, q2, p2);
    const double d3 = cross3(p1, p2, q1);
    const double d4 = cross3(p1, p2, q2);
    return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
}

double pointSegDist2(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 ab = b - a;
    const double len2 = ab.x * ab.x + ab.y * ab.y;
    double t = 0.0;
    if (len2 > 0.0) t = std::clamp(((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / len2, 0.0, 1.0);
    const Vec2 c = a + ab * t;
    const double dx = p.x - c.x;
    const double dy = p.y - c.y;
    return dx * dx + dy * dy;
}

bool segmentsWithin(Vec2 a0, Vec2 a1, Vec2 b0, Vec2 b1, double gap2) {
    if (properCross(a0, a1, b0, b1)) return true;
    return pointSegDist2(a0, b0, b1) < gap2 || pointSegDist2(a1, b0, b1) < gap2 ||
           pointSegDist2(b0, a0, a1) < gap2 || pointSegDist2(b1, a0, a1) < gap2;
}

bool boxesNear(const Box2& a, Vec2 offA, const Box2& b, Vec2 offB, double gap) {
    return a.min.x + offA.x < b.max.x + offB.x + gap && b.min.x + offB.x < a.max.x + offA.x + gap &&
           a.min.y + offA.y < b.max.y + offB.y + gap && b.min.y + offB.y < a.max.y + offA.y + gap;
}

bool inRing(const Ring& r, Vec2 p) {
    bool in = false;
    const std::size_t n = r.pts.size();
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const Vec2 a = r.pts[i];
        const Vec2 b = r.pts[j];
        if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) in = !in;
    }
    return in;
}

/// Inside the outer ring and outside every hole ring (rings placed at `off`).
bool inSolid(const std::vector<Ring>& rings, Vec2 off, Vec2 p, Ctx& ctx) {
    const Vec2 q = p - off;
    for (const Ring& r : rings) ctx.charge(r.pts.size());
    if (!inRing(rings[0], q)) return false;
    for (std::size_t i = 1; i < rings.size(); ++i) {
        if (inRing(rings[i], q)) return false;
    }
    return true;
}

/// True when any edge of `a` (at offA) comes within `gap` of any edge of `b` (at offB).
bool edgesNear(const std::vector<Ring>& a, Vec2 offA, const std::vector<Ring>& b, Vec2 offB, double gap, Ctx& ctx) {
    // Candidates sit exactly `gap` from their neighbours; the slack keeps rounding from rejecting them.
    const double gapTest = gap - 1e-7;
    const double gap2 = gapTest * gapTest;
    for (const Ring& ra : a) {
        for (const Ring& rb : b) {
            ctx.charge(1);
            if (!boxesNear(ra.box, offA, rb.box, offB, gap)) continue;
            const std::size_t na = ra.pts.size();
            const std::size_t nb = rb.pts.size();
            for (std::size_t i = 0; i < na; ++i) {
                const Vec2 a0 = ra.pts[i] + offA;
                const Vec2 a1 = ra.pts[(i + 1) % na] + offA;
                const double ax0 = std::min(a0.x, a1.x) - gap;
                const double ax1 = std::max(a0.x, a1.x) + gap;
                const double ay0 = std::min(a0.y, a1.y) - gap;
                const double ay1 = std::max(a0.y, a1.y) + gap;
                ctx.charge(1);
                if (ax1 < rb.box.min.x + offB.x || ax0 > rb.box.max.x + offB.x || ay1 < rb.box.min.y + offB.y ||
                    ay0 > rb.box.max.y + offB.y) {
                    continue;
                }
                ctx.charge(nb);
                if (ctx.exhausted) return true;
                for (std::size_t j = 0; j < nb; ++j) {
                    const Vec2 b0 = rb.pts[j] + offB;
                    const Vec2 b1 = rb.pts[(j + 1) % nb] + offB;
                    if (std::max(b0.x, b1.x) < ax0 || std::min(b0.x, b1.x) > ax1 || std::max(b0.y, b1.y) < ay0 ||
                        std::min(b0.y, b1.y) > ay1) {
                        continue;
                    }
                    if (segmentsWithin(a0, a1, b0, b1, gap2)) return true;
                }
            }
        }
    }
    return false;
}

bool collide(const Orientation& a, Vec2 offA, const Orientation& b, Vec2 offB, double gap, Ctx& ctx) {
    ctx.charge(1);
    if (!boxesNear(a.box, offA, b.box, offB, gap)) return false;
    if (edgesNear(a.rings, offA, b.rings, offB, gap, ctx)) return true;
    if (inSolid(b.rings, offB, a.rings[0].pts[0] + offA, ctx)) return true;
    if (inSolid(a.rings, offA, b.rings[0].pts[0] + offB, ctx)) return true;
    return false;
}

Ring makeRing(std::vector<Vec2> pts) {
    Ring r;
    for (const Vec2& p : pts) r.box.expand(p);
    r.pts = std::move(pts);
    return r;
}

bool finitePolygon(const Polyline& p) {
    if (p.points.size() < 3) return false;
    for (const Vec2& v : p.points) {
        if (!std::isfinite(v.x) || !std::isfinite(v.y)) return false;
    }
    return true;
}

bool samePoints(std::vector<Vec2> a, std::vector<Vec2> b) {
    if (a.size() != b.size()) return false;
    const auto less = [](Vec2 p, Vec2 q) { return p.x != q.x ? p.x < q.x : p.y < q.y; };
    std::sort(a.begin(), a.end(), less);
    std::sort(b.begin(), b.end(), less);
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (std::fabs(a[i].x - b[i].x) > 1e-6 || std::fabs(a[i].y - b[i].y) > 1e-6) return false;
    }
    return true;
}

class Nester {
   public:
    Nester(const std::vector<NestPart>& parts, const NestingOptions& o) : parts_(parts), opt_(o) {
        sx_ = (o.corner == NestCorner::BottomRight || o.corner == NestCorner::TopRight) ? -1.0 : 1.0;
        sy_ = (o.corner == NestCorner::TopLeft || o.corner == NestCorner::TopRight) ? -1.0 : 1.0;
        const double chord = std::isfinite(o.chordalMm) && o.chordalMm > 0.0 ? o.chordalMm : 0.0;
        const double g = std::isfinite(o.gapMm) && o.gapMm > 0.0 ? o.gapMm : 0.0;
        gap_ = std::max(g, kMinGapMm) + chord;
        const double m = std::isfinite(o.marginMm) && o.marginMm > 0.0 ? o.marginMm : 0.0;
        margin_ = m;
        edgeMargin_ = std::max(m, kMinGapMm) + chord;
        ctx_.maxOps = o.maxOperations;
        for (const Polyline& b : o.boundary) {
            if (finitePolygon(b)) boundary_.push_back(b);
        }
        for (const Polyline& b : boundary_) bbox_.expand(b.bounds());
    }

    NestingResult run() {
        NestingResult res;
        data_.reserve(parts_.size());
        for (const NestPart& p : parts_) data_.push_back(prepare(p.shape));

        struct Inst {
            std::size_t part, copy;
        };
        std::vector<Inst> order;
        for (std::size_t i = 0; i < parts_.size(); ++i) {
            for (std::size_t c = 0; c < parts_[i].copies; ++c) order.push_back({i, c});
        }
        std::stable_sort(order.begin(), order.end(), [&](const Inst& a, const Inst& b) {
            if (data_[a.part].area != data_[b.part].area) return data_[a.part].area > data_[b.part].area;
            if (a.part != b.part) return a.part < b.part;
            return a.copy < b.copy;
        });

        for (const Inst& in : order) {
            const PartData& pd = data_[in.part];
            if (!pd.valid) {
                res.unplaced.push_back({in.part, in.copy, NestStopReason::Invalid});
                continue;
            }
            if (ctx_.stopped()) {
                res.unplaced.push_back(
                    {in.part, in.copy, ctx_.cancelled ? NestStopReason::Cancelled : NestStopReason::Budget});
                continue;
            }
            bool done = false;
            for (std::size_t s = 0; s < sheets_.size() && !done && !ctx_.stopped(); ++s) {
                done = tryPlace(in.part, in.copy, s, res);
            }
            if (done) continue;
            if (ctx_.stopped()) {
                res.unplaced.push_back(
                    {in.part, in.copy, ctx_.cancelled ? NestStopReason::Cancelled : NestStopReason::Budget});
                continue;
            }
            if (sheets_.size() >= opt_.maxSheets) {
                res.unplaced.push_back({in.part, in.copy, NestStopReason::NoSheet});
                continue;
            }
            const NestSheet size = sheetSize(sheets_.size());
            const auto key = std::make_tuple(in.part, boundary_.empty() ? size.lengthMm : 0.0,
                                             boundary_.empty() ? size.widthMm : 0.0);
            if (std::find(unfit_.begin(), unfit_.end(), key) != unfit_.end()) {
                res.unplaced.push_back({in.part, in.copy, NestStopReason::TooLarge});
                continue;
            }
            sheets_.push_back(newSheet(sheets_.size()));
            if (tryPlace(in.part, in.copy, sheets_.size() - 1, res)) continue;
            sheets_.pop_back();
            if (ctx_.stopped()) {
                res.unplaced.push_back(
                    {in.part, in.copy, ctx_.cancelled ? NestStopReason::Cancelled : NestStopReason::Budget});
            } else {
                unfit_.push_back(key);
                res.unplaced.push_back({in.part, in.copy, NestStopReason::TooLarge});
            }
        }
        std::sort(res.unplaced.begin(), res.unplaced.end(), [](const NestUnplaced& a, const NestUnplaced& b) {
            return a.part != b.part ? a.part < b.part : a.copy < b.copy;
        });
        res.sheetCount = sheets_.size();
        res.operations = ctx_.ops;
        res.budgetExhausted = ctx_.exhausted;
        res.cancelled = ctx_.cancelled;
        return res;
    }

   private:
    NestSheet sheetSize(std::size_t i) const {
        if (opt_.sheets.empty()) return NestSheet{};
        return opt_.sheets[std::min(i, opt_.sheets.size() - 1)];
    }

    SheetState newSheet(std::size_t index) const {
        SheetState s;
        Region& r = s.region;
        if (!boundary_.empty()) {
            r.hasBoundary = true;
            r.x0 = bbox_.min.x;
            r.y0 = bbox_.min.y;
            r.x1 = bbox_.max.x;
            r.y1 = bbox_.max.y;
            for (const Polyline& b : boundary_) {
                std::vector<Vec2> pts;
                pts.reserve(b.points.size());
                for (const Vec2& p : b.points) {
                    pts.push_back({sx_ < 0 ? r.x0 + r.x1 - p.x : p.x, sy_ < 0 ? r.y0 + r.y1 - p.y : p.y});
                }
                r.rings.push_back(makeRing(std::move(pts)));
            }
        } else {
            const NestSheet sz = sheetSize(index);
            r.x1 = sz.lengthMm;
            r.y1 = sz.widthMm;
        }
        return s;
    }

    PartData prepare(const NestShape& shape) const {
        PartData pd;
        if (!finitePolygon(shape.outer)) return pd;
        Polyline outer = shape.outer;
        outer.closed = true;
        pd.area = std::fabs(outer.signedArea());
        if (!(pd.area > kEps)) return pd;
        pd.valid = true;

        std::vector<Polyline> holes;
        if (opt_.allowInside) {
            for (const Polyline& h : shape.holes) {
                if (finitePolygon(h)) holes.push_back(h);
            }
        }
        std::size_t nAngles = 1;
        double step = 0.0;
        if (std::isfinite(opt_.rotationStepRad) && opt_.rotationStepRad > 0.0) {
            step = std::max(opt_.rotationStepRad, kPi / 180.0);
            nAngles = std::clamp<std::size_t>(static_cast<std::size_t>(std::floor(kTwoPi / step + 1e-9)), 1, 360);
        }
        std::vector<std::vector<Vec2>> seen;
        for (int mir = 0; mir < (opt_.mirror ? 2 : 1); ++mir) {
            for (std::size_t k = 0; k < nAngles; ++k) {
                const double angle = static_cast<double>(k) * step;
                const Transform2 m = Transform2::rotation(angle) * (mir ? Transform2::mirrorX(0.0) : Transform2::identity());
                const Polyline o = outer.transformed(m);
                const Box2 b = o.bounds();
                const Transform2 base = Transform2::translation(-b.min) * m;
                std::vector<Vec2> norm;
                norm.reserve(o.points.size());
                for (const Vec2& p : o.points) norm.push_back(p - b.min);
                bool dup = false;
                for (const auto& s : seen) {
                    if (samePoints(s, norm)) {
                        dup = true;
                        break;
                    }
                }
                if (dup) continue;
                seen.push_back(norm);

                Orientation ori;
                ori.base = base;
                ori.angle = angle;
                ori.mirrored = mir != 0;
                ori.w = b.width();
                ori.h = b.height();
                ori.box = Box2(Vec2{0, 0}, Vec2{ori.w, ori.h});
                const auto reflect = [&](std::vector<Vec2> pts) {
                    for (Vec2& p : pts) {
                        if (sx_ < 0) p.x = ori.w - p.x;
                        if (sy_ < 0) p.y = ori.h - p.y;
                    }
                    return pts;
                };
                ori.rings.push_back(makeRing(reflect(std::move(norm))));
                for (const Polyline& h : holes) {
                    std::vector<Vec2> pts;
                    for (const Vec2& p : h.points) pts.push_back(base.apply(p));
                    ori.rings.push_back(makeRing(reflect(std::move(pts))));
                }
                pd.orientations.push_back(std::move(ori));
            }
        }
        return pd;
    }

    bool valid(const Orientation& o, Vec2 pos, const SheetState& sh) {
        const Region& r = sh.region;
        ctx_.charge(1);
        if (pos.x < r.x0 + margin_ - kEps || pos.y < r.y0 + margin_ - kEps || pos.x + o.w > r.x1 - margin_ + kEps ||
            pos.y + o.h > r.y1 - margin_ + kEps) {
            return false;
        }
        if (ctx_.stopped()) return false;
        if (r.hasBoundary) {
            if (edgesNear(o.rings, pos, r.rings, Vec2{}, edgeMargin_, ctx_)) return false;
            if (ctx_.stopped()) return false;
            // Whole outline inside the even-odd region; no exclusion swallowed.
            for (const Vec2& p : o.rings[0].pts) {
                int in = 0;
                for (const Ring& b : r.rings) in += inRing(b, p + pos) ? 1 : 0;
                if (in % 2 == 0) return false;
            }
            ctx_.charge(o.rings[0].pts.size() * r.rings.size());
            for (const Ring& b : r.rings) {
                if (inSolid(o.rings, pos, b.pts[0], ctx_)) return false;
            }
        }
        for (const Placed& p : sh.placed) {
            if (collide(o, pos, *p.o, p.pos, gap_, ctx_)) return false;
            if (ctx_.stopped()) return false;
        }
        return true;
    }

    static void addUnique(std::vector<double>& v, double x) { v.push_back(x); }

    static void finish(std::vector<double>& v) {
        std::sort(v.begin(), v.end());
        v.erase(std::unique(v.begin(), v.end(), [](double a, double b) { return std::fabs(a - b) < 1e-7; }), v.end());
    }

    std::optional<std::pair<std::size_t, Vec2>> best(const PartData& pd, const SheetState& sh) {
        const Region& r = sh.region;
        if (!(r.x1 - r.x0 > 0.0 && r.y1 - r.y0 > 0.0)) return std::nullopt;  // also rejects NaN sizes
        std::vector<double> xs;
        std::vector<double> ys;
        addUnique(xs, r.x0 + margin_);
        addUnique(ys, r.y0 + margin_);
        for (const Placed& p : sh.placed) {
            addUnique(xs, p.box.max.x + gap_);
            addUnique(xs, p.box.min.x);
            addUnique(ys, p.box.max.y + gap_);
            addUnique(ys, p.box.min.y);
            for (std::size_t i = 1; i < p.o->rings.size(); ++i) {
                const Box2& hb = p.o->rings[i].box;
                addUnique(xs, p.pos.x + hb.min.x + gap_);
                addUnique(ys, p.pos.y + hb.min.y + gap_);
            }
        }
        for (const Ring& b : r.rings) {
            for (const Vec2& v : b.pts) {
                addUnique(xs, v.x + edgeMargin_);
                addUnique(ys, v.y + edgeMargin_);
            }
        }
        finish(xs);
        finish(ys);

        const bool alongX = opt_.direction == NestDirection::AlongX;
        std::optional<std::pair<std::size_t, Vec2>> bestPos;
        double bestA = 0.0;
        double bestB = 0.0;
        for (std::size_t oi = 0; oi < pd.orientations.size() && !ctx_.stopped(); ++oi) {
            const Orientation& o = pd.orientations[oi];
            if (o.w > r.x1 - r.x0 - 2.0 * margin_ + kEps || o.h > r.y1 - r.y0 - 2.0 * margin_ + kEps) continue;
            const double maxX = r.x1 - margin_ - o.w + kEps;
            const double maxY = r.y1 - margin_ - o.h + kEps;
            std::optional<Vec2> found;
            const std::vector<double>& outerV = alongX ? ys : xs;
            const std::vector<double>& innerV = alongX ? xs : ys;
            const double outerMax = alongX ? maxY : maxX;
            const double innerMax = alongX ? maxX : maxY;
            for (double ov : outerV) {
                if (ov > outerMax) break;
                for (double iv : innerV) {
                    if (iv > innerMax) break;
                    const Vec2 pos = alongX ? Vec2{iv, ov} : Vec2{ov, iv};
                    ctx_.charge(1);
                    if (cancelRequested()) ctx_.cancelled = true;
                    if (ctx_.stopped()) break;
                    if (valid(o, pos, sh)) {
                        found = pos;
                        break;
                    }
                }
                if (found || ctx_.stopped()) break;
            }
            if (!found) continue;
            Vec2 pos = *found;
            for (int round = 0; round < kSlideRounds; ++round) {
                const Vec2 before = pos;
                if (alongX) {
                    pos.y = slide(o, pos, sh, true);
                    pos.x = slide(o, pos, sh, false);
                } else {
                    pos.x = slide(o, pos, sh, false);
                    pos.y = slide(o, pos, sh, true);
                }
                if (pos.x == before.x && pos.y == before.y) break;
            }
            const double ka = alongX ? pos.y + o.h : pos.x + o.w;
            const double kb = alongX ? pos.x : pos.y;
            if (!bestPos || ka < bestA - kEps || (std::fabs(ka - bestA) <= kEps && kb < bestB - kEps)) {
                bestPos = std::make_pair(oi, pos);
                bestA = ka;
                bestB = kb;
            }
        }
        return bestPos;
    }

    /// Moves toward the sheet origin along one axis as far as a valid position is found (bisection);
    /// returns the new coordinate on that axis.
    double slide(const Orientation& o, Vec2 pos, const SheetState& sh, bool yAxis) {
        const double low = (yAxis ? sh.region.y0 : sh.region.x0) + margin_;
        double cur = yAxis ? pos.y : pos.x;
        const auto at = [&](double v) { return yAxis ? Vec2{pos.x, v} : Vec2{v, pos.y}; };
        if (cur - low <= kEps) return cur;
        if (valid(o, at(low), sh)) return low;
        double lo = low;  // invalid
        double hi = cur;  // valid
        for (int i = 0; i < kSlideBisections && !ctx_.stopped(); ++i) {
            const double mid = 0.5 * (lo + hi);
            if (valid(o, at(mid), sh)) {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        return hi;
    }

    bool tryPlace(std::size_t part, std::size_t copy, std::size_t sheetIdx, NestingResult& res) {
        SheetState& sh = sheets_[sheetIdx];
        const PartData& pd = data_[part];
        const auto b = best(pd, sh);
        if (!b) return false;
        const Orientation& o = pd.orientations[b->first];
        const Vec2 pos = b->second;
        Placed pl;
        pl.o = &o;
        pl.pos = pos;
        pl.box = Box2(pos, Vec2{pos.x + o.w, pos.y + o.h});
        sh.placed.push_back(pl);
        const Region& r = sh.region;
        const Vec2 act{sx_ < 0 ? r.x0 + r.x1 - pos.x - o.w : pos.x, sy_ < 0 ? r.y0 + r.y1 - pos.y - o.h : pos.y};
        NestPlacementResult out;
        out.part = part;
        out.copy = copy;
        out.sheet = sheetIdx;
        out.transform = Transform2::translation(act) * o.base;
        out.angleRad = o.angle;
        out.mirrored = o.mirrored;
        res.placements.push_back(out);
        return true;
    }

    const std::vector<NestPart>& parts_;
    const NestingOptions& opt_;
    double sx_{1}, sy_{1};
    double gap_{0}, margin_{0}, edgeMargin_{0};
    std::vector<Polyline> boundary_;
    Box2 bbox_;
    Ctx ctx_;
    std::vector<PartData> data_;
    std::vector<SheetState> sheets_;
    std::vector<std::tuple<std::size_t, double, double>> unfit_;
};

}  // namespace

NestingResult nestParts(const std::vector<NestPart>& parts, const NestingOptions& options) {
    Nester n(parts, options);
    return n.run();
}

}  // namespace carvetto::geometry
