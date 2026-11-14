#include "carvetto/relief/sculpt.hpp"

#include <algorithm>
#include <cmath>

namespace carvetto::relief {

namespace {

bool finite(double v) { return std::isfinite(v); }

struct Region {
    long i0 = 0, j0 = 0, i1 = -1, j1 = -1;  // inclusive
    bool empty() const { return i1 < i0 || j1 < j0; }
};

Region cellRegion(const HeightField& f, Point c, double radius, long margin) {
    const double cs = f.cellSize();
    const long w = static_cast<long>(f.width());
    const long h = static_cast<long>(f.height());
    Region r;
    r.i0 = std::max(0L, static_cast<long>(std::floor((c.x - radius - f.origin().x) / cs)) - margin);
    r.j0 = std::max(0L, static_cast<long>(std::floor((c.y - radius - f.origin().y) / cs)) - margin);
    r.i1 = std::min(w - 1, static_cast<long>(std::ceil((c.x + radius - f.origin().x) / cs)) + margin);
    r.j1 = std::min(h - 1, static_cast<long>(std::ceil((c.y + radius - f.origin().y) / cs)) + margin);
    return r;
}

/// Copy of a region of a field, for reads of the pre-dab values.
struct Snapshot {
    long i0 = 0, j0 = 0, w = 0, h = 0;
    std::vector<float> v;

    void take(const HeightField& f, const Region& r) {
        i0 = r.i0;
        j0 = r.j0;
        w = r.empty() ? 0 : r.i1 - r.i0 + 1;
        h = r.empty() ? 0 : r.j1 - r.j0 + 1;
        v.resize(static_cast<std::size_t>(w * h));
        for (long j = 0; j < h; ++j)
            for (long i = 0; i < w; ++i)
                v[static_cast<std::size_t>(j * w + i)] =
                    f.at(static_cast<std::size_t>(i0 + i), static_cast<std::size_t>(j0 + j));
    }
    bool has(long i, long j) const { return i >= i0 && j >= j0 && i < i0 + w && j < j0 + h; }
    float at(long i, long j) const { return v[static_cast<std::size_t>((j - j0) * w + (i - i0))]; }

    /// Bilinear at fractional cell coordinates; weights of absent/transparent cells are dropped
    /// and the rest renormalised. Transparent when nothing contributes.
    float sampleOpaque(double fx, double fy) const {
        const double bx = std::floor(fx);
        const double by = std::floor(fy);
        const long ix = static_cast<long>(bx);
        const long iy = static_cast<long>(by);
        const double tx = fx - bx;
        const double ty = fy - by;
        double sum = 0.0, wsum = 0.0;
        for (int dj = 0; dj < 2; ++dj)
            for (int di = 0; di < 2; ++di) {
                const double wt = (di ? tx : 1.0 - tx) * (dj ? ty : 1.0 - ty);
                if (wt <= 0.0 || !has(ix + di, iy + dj)) continue;
                const float c = at(ix + di, iy + dj);
                if (std::isnan(c)) continue;
                sum += wt * static_cast<double>(c);
                wsum += wt;
            }
        if (wsum <= 0.0) return kTransparent;
        return static_cast<float>(sum / wsum);
    }
};

double customProfile(const CustomBrush& cb, double diameter, double dx, double dy, double angle) {
    const HeightField& s = cb.stamp;
    float lo = 0.0f, hi = 0.0f;
    if (!s.minMax(lo, hi)) return 0.0;
    const double ext = static_cast<double>(std::max(s.width(), s.height())) * s.cellSize();
    const double scale = diameter / ext;
    const double ca = std::cos(angle);
    const double sa = std::sin(angle);
    // Inverse rotation of the dab-local offset, then back to stamp mm.
    const double u = (ca * dx + sa * dy) / scale;
    const double v = (-sa * dx + ca * dy) / scale;
    const double cx = s.origin().x + 0.5 * static_cast<double>(s.width() - 1) * s.cellSize();
    const double cy = s.origin().y + 0.5 * static_cast<double>(s.height() - 1) * s.cellSize();
    const float val = s.sample(cx + u, cy + v);
    if (std::isnan(val)) return 0.0;
    if (hi <= lo) return 1.0;
    return (static_cast<double>(val) - static_cast<double>(lo)) / (static_cast<double>(hi) - static_cast<double>(lo));
}

double customRadius(const CustomBrush& cb, double diameter) {
    const HeightField& s = cb.stamp;
    const double ext = static_cast<double>(std::max(s.width(), s.height())) * s.cellSize();
    const double scale = diameter / ext;
    return 0.5 * std::hypot(static_cast<double>(s.width()) * s.cellSize(), static_cast<double>(s.height()) * s.cellSize()) *
           scale;
}

/// 1 - (1 - a)^w : blend factor for a dab of weight w, compounding exactly across dabs.
double blendFactor(double a, double w) {
    a = std::clamp(a, 0.0, 1.0);
    if (w <= 0.0 || a <= 0.0) return 0.0;
    if (a >= 1.0) return 1.0;
    return 1.0 - std::pow(1.0 - a, w);
}

float applyMode(SculptMode mode, float oldV, float newV) {
    if (std::isnan(oldV) || std::isnan(newV)) return newV;
    if (mode == SculptMode::Raise) return std::max(oldV, newV);
    if (mode == SculptMode::Lower) return std::min(oldV, newV);
    return newV;
}

void applyDab(HeightField& f, const HeightField& baseline, const BrushParams& b, const Dab& d) {
    const double cs = f.cellSize();
    const double radius = b.custom ? customRadius(*b.custom, b.diameter) : 0.5 * b.diameter;
    const double dlen = std::hypot(d.delta.x, d.delta.y);
    const long margin = (b.kind == BrushKind::Smudge) ? static_cast<long>(std::ceil(dlen / cs)) + 2 : 2;
    const Region r = cellRegion(f, d.pos, radius, 0);
    if (r.empty()) return;
    Snapshot snap;
    snap.take(f, Region{std::max(0L, r.i0 - margin), std::max(0L, r.j0 - margin),
                        std::min(static_cast<long>(f.width()) - 1, r.i1 + margin),
                        std::min(static_cast<long>(f.height()) - 1, r.j1 + margin)});
    const double angle = b.custom
                             ? b.custom->angleRad + (b.custom->dynamicRotation && dlen > 0.0 ? std::atan2(d.delta.y, d.delta.x) : 0.0)
                             : 0.0;
    for (long j = r.j0; j <= r.j1; ++j) {
        for (long i = r.i0; i <= r.i1; ++i) {
            const double x = f.origin().x + static_cast<double>(i) * cs;
            const double y = f.origin().y + static_cast<double>(j) * cs;
            const double dx = x - d.pos.x;
            const double dy = y - d.pos.y;
            const double k = b.custom ? customProfile(*b.custom, b.diameter, dx, dy, angle)
                                      : brushKernel(std::hypot(dx, dy), b.diameter, b.smoothness);
            if (k <= 0.0) continue;
            const float oldV = snap.at(i, j);
            const bool oldT = std::isnan(oldV);
            if (oldT && b.preserveTransparency) continue;
            float nv = oldV;
            switch (b.kind) {
                case BrushKind::Deposit:
                case BrushKind::Remove: {
                    const double amount = b.strength * k * d.weight * (b.kind == BrushKind::Deposit ? 1.0 : -1.0);
                    nv = static_cast<float>((oldT ? 0.0 : static_cast<double>(oldV)) + amount);
                    break;
                }
                case BrushKind::Smooth: {
                    const double a = blendFactor(std::min(1.0, b.strength) * k, d.weight);
                    if (a <= 0.0) continue;
                    double sum = 0.0, wsum = 0.0;
                    for (long dj = -1; dj <= 1; ++dj)
                        for (long di = -1; di <= 1; ++di) {
                            if (!snap.has(i + di, j + dj)) continue;
                            const float c = snap.at(i + di, j + dj);
                            if (std::isnan(c)) continue;
                            const double wt = (di == 0 ? 2.0 : 1.0) * (dj == 0 ? 2.0 : 1.0);
                            sum += wt * static_cast<double>(c);
                            wsum += wt;
                        }
                    if (wsum <= 0.0) continue;
                    const double blur = sum / wsum;
                    nv = oldT ? static_cast<float>(blur)
                              : static_cast<float>(static_cast<double>(oldV) + (blur - static_cast<double>(oldV)) * a);
                    break;
                }
                case BrushKind::Smudge: {
                    if (dlen <= 0.0) continue;
                    const double a = std::min(1.0, b.strength) * k;
                    const float src = snap.sampleOpaque((x - a * d.delta.x - f.origin().x) / cs,
                                                        (y - a * d.delta.y - f.origin().y) / cs);
                    if (std::isnan(src)) continue;
                    nv = src;
                    break;
                }
                case BrushKind::UndoBrush: {
                    const double a = blendFactor(std::min(1.0, b.strength) * k, d.weight);
                    if (a <= 0.0) continue;
                    const float orig = baseline.at(static_cast<std::size_t>(i), static_cast<std::size_t>(j));
                    if (a >= 1.0) {
                        nv = orig;
                    } else if (std::isnan(orig) || oldT) {
                        nv = a >= 0.5 ? orig : oldV;
                    } else {
                        nv = static_cast<float>(static_cast<double>(oldV) +
                                                (static_cast<double>(orig) - static_cast<double>(oldV)) * a);
                    }
                    break;
                }
            }
            nv = applyMode(b.mode, oldV, nv);
            f.set(static_cast<std::size_t>(i), static_cast<std::size_t>(j), nv);
        }
    }
}

}  // namespace

double brushKernel(double distance, double diameter, double smoothness) {
    const double r = 0.5 * diameter;
    if (r <= 0.0) return 0.0;
    const double t = distance / r;
    if (t >= 1.0) return 0.0;
    const double inner = 1.0 - std::clamp(smoothness, 0.0, 1.0);
    if (t <= inner) return 1.0;
    const double u = (t - inner) / (1.0 - inner);
    return 1.0 - u * u * (3.0 - 2.0 * u);
}

Status dabsAlong(const std::vector<Point>& path, double spacing, double diameter, std::vector<Dab>& out) {
    out.clear();
    if (path.empty() || !finite(spacing) || spacing <= 0.0 || !finite(diameter) || diameter <= 0.0)
        return Status::BadParameter;
    for (const Point& p : path)
        if (!finite(p.x) || !finite(p.y)) return Status::BadParameter;
    std::vector<double> cum(path.size(), 0.0);
    for (std::size_t i = 1; i < path.size(); ++i)
        cum[i] = cum[i - 1] + std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    const double total = cum.back();
    if (total <= 0.0) {
        Dab d;
        d.pos = path.front();
        out.push_back(d);
        return Status::Ok;
    }
    const double nd = std::ceil(total / spacing);
    if (nd > 1.0e7) return Status::TooLarge;
    const std::size_t n = static_cast<std::size_t>(nd);
    const double piece = total / static_cast<double>(n);
    std::size_t seg = 1;
    auto at = [&](double s) {
        while (seg + 1 < path.size() && cum[seg] < s) ++seg;
        const double len = cum[seg] - cum[seg - 1];
        const double t = len > 0.0 ? std::clamp((s - cum[seg - 1]) / len, 0.0, 1.0) : 0.0;
        return Point{path[seg - 1].x + (path[seg].x - path[seg - 1].x) * t,
                     path[seg - 1].y + (path[seg].y - path[seg - 1].y) * t};
    };
    out.reserve(n);
    Point prev = at(0.0);
    for (std::size_t k = 0; k < n; ++k) {
        const Point mid = at((static_cast<double>(k) + 0.5) * piece);
        const Point next = at(static_cast<double>(k + 1) * piece);
        Dab d;
        d.pos = mid;
        d.weight = piece / diameter;
        d.delta = Point{next.x - prev.x, next.y - prev.y};
        prev = next;
        out.push_back(d);
    }
    return Status::Ok;
}

Status SculptSession::begin(const HeightField& baseline, SculptOptions options, SculptSession& out) {
    if (baseline.empty()) return Status::ZeroSize;
    out = SculptSession{};
    out.baseline_ = baseline;
    out.working_ = baseline;
    out.options_ = options;
    return Status::Ok;
}

Status SculptSession::applyStroke(const BrushParams& b, const std::vector<Dab>& dabs) {
    if (!active()) return Status::ZeroSize;
    if (!finite(b.diameter) || b.diameter <= 0.0 || !finite(b.strength) || b.strength < 0.0 || !finite(b.smoothness) ||
        b.smoothness < 0.0 || b.smoothness > 1.0)
        return Status::BadParameter;
    if (b.custom && (b.custom->stamp.empty() || !finite(b.custom->angleRad))) return Status::BadParameter;
    for (const Dab& d : dabs)
        if (!finite(d.pos.x) || !finite(d.pos.y) || !finite(d.weight) || d.weight < 0.0 || !finite(d.delta.x) ||
            !finite(d.delta.y))
            return Status::BadParameter;
    // Touched cell box (union of the dab boxes) for the undo patch.
    const double radius = b.custom ? customRadius(*b.custom, b.diameter) : 0.5 * b.diameter;
    Region box;
    for (const Dab& d : dabs) {
        const Region r = cellRegion(working_, d.pos, radius, 0);
        if (r.empty()) continue;
        if (box.empty()) box = r;
        else {
            box.i0 = std::min(box.i0, r.i0);
            box.j0 = std::min(box.j0, r.j0);
            box.i1 = std::max(box.i1, r.i1);
            box.j1 = std::max(box.j1, r.j1);
        }
    }
    if (box.empty()) return Status::Ok;
    Patch p;
    p.i0 = static_cast<std::size_t>(box.i0);
    p.j0 = static_cast<std::size_t>(box.j0);
    p.w = static_cast<std::size_t>(box.i1 - box.i0 + 1);
    p.h = static_cast<std::size_t>(box.j1 - box.j0 + 1);
    const std::size_t bytes = p.w * p.h * sizeof(float);
    if (bytes <= options_.maxUndoBytes) {
        p.before.reserve(p.w * p.h);
        for (std::size_t j = 0; j < p.h; ++j)
            for (std::size_t i = 0; i < p.w; ++i) p.before.push_back(working_.at(p.i0 + i, p.j0 + j));
    }
    for (const Dab& d : dabs) applyDab(working_, baseline_, b, d);
    // A patch over the cap evicts itself and everything before it (history is never inexact).
    undo_.push_back(std::move(p));
    undoBytes_ += bytes;
    while (undoBytes_ > options_.maxUndoBytes && !undo_.empty()) {
        undoBytes_ -= undo_.front().w * undo_.front().h * sizeof(float);
        undo_.erase(undo_.begin());
    }
    return Status::Ok;
}

bool SculptSession::undoStroke() {
    if (undo_.empty()) return false;
    const Patch& p = undo_.back();
    for (std::size_t j = 0; j < p.h; ++j)
        for (std::size_t i = 0; i < p.w; ++i) working_.set(p.i0 + i, p.j0 + j, p.before[j * p.w + i]);
    undoBytes_ -= p.w * p.h * sizeof(float);
    undo_.pop_back();
    return true;
}

void SculptSession::keep() {
    baseline_ = working_;
    undo_.clear();
    undoBytes_ = 0;
}

void SculptSession::discard() {
    working_ = baseline_;
    undo_.clear();
    undoBytes_ = 0;
}

}  // namespace carvetto::relief
