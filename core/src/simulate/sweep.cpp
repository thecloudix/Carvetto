#include "sweep.hpp"

#include <cmath>
#include <limits>

#include "det_math.hpp"

namespace carvetto::simulate {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
/// Segments shorter than this (XY, mm) are stamps of the tool at one place (Z may still change).
constexpr double kMinSegment = 1e-9;
constexpr int kMaxArcDepth = 60;

/// First candidate cell index for centres >= x (one cell of slack), clamped to [0, n].
std::size_t lowIndex(double x, double origin, double cs, std::size_t n) {
    const double v = std::ceil((x - origin) / cs - 0.5) - 1.0;
    if (!(v > 0.0)) {
        return 0;
    }
    if (v >= static_cast<double>(n)) {
        return n;
    }
    return static_cast<std::size_t>(v);
}

/// One past the last candidate cell index for centres <= x (one cell of slack), clamped to [0, n].
std::size_t highIndex(double x, double origin, double cs, std::size_t n) {
    const double v = std::floor((x - origin) / cs - 0.5) + 2.0;
    if (!(v > 0.0)) {
        return 0;
    }
    if (v >= static_cast<double>(n)) {
        return n;
    }
    return static_cast<std::size_t>(v);
}

/// Radius within which a tool whose tip is `gap` below the stock top can touch it (with a hair of
/// slack for rounding); the whole tool radius when that is smaller.
double reachOf(const ToolShape& tool, double gap) {
    const double r = tool.reachAtGap(gap) + 1e-9;
    return r < tool.radius() ? r : tool.radius();
}

double clampTo(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/// Lowest cutting-surface height over a segment for one cell, exact. The tool tip moves along the
/// segment (parameter s in [0, L], tip Z = az + g * s); the cell is at distance q from the segment's
/// line and u along it. With w = s - u the surface height is
///     az + g * u + [ g * w + h(sqrt(q^2 + w^2)) ]
/// and the bracket is convex in w for every profile (h is convex and non-decreasing), so its
/// minimum over the feasible w interval is found per profile piece in closed form and clamped
/// (ADR-0032 §2). Returns +infinity when the cell is out of reach.
double segmentMin(const ToolShape& tool, double q2, double u, double len, double az, double g) {
    double best = kInf;
    const double wlo = -u;
    const double whi = len - u;
    const std::size_t n = tool.pieceCount();
    for (std::size_t k = 0; k < n; ++k) {
        const ProfilePiece& p = tool.piece(k);
        const double bSq = p.r1 * p.r1 - q2;
        if (bSq < 0.0) {
            continue;
        }
        const double b = std::sqrt(bSq);
        const double aSq = p.r0 * p.r0 - q2;
        const double a = aSq > 0.0 ? std::sqrt(aSq) : 0.0;
        for (int side = 0; side < 2; ++side) {
            double ilo;
            double ihi;
            if (a <= 0.0) {
                if (side == 1) {
                    break;  // the two sides are one interval [-b, b]
                }
                ilo = -b;
                ihi = b;
            } else if (side == 0) {
                ilo = a;
                ihi = b;
            } else {
                ilo = -b;
                ihi = -a;
            }
            const double lo = ilo > wlo ? ilo : wlo;
            const double hi = ihi < whi ? ihi : whi;
            if (lo > hi) {
                continue;
            }
            double w;
            switch (p.kind) {
                case ProfilePiece::Kind::Flat:
                    w = g > 0.0 ? lo : (g < 0.0 ? hi : lo);
                    break;
                case ProfilePiece::Kind::Cone: {
                    if (g >= p.slope) {
                        w = lo;
                    } else if (g <= -p.slope) {
                        w = hi;
                    } else {
                        w = clampTo(-g * std::sqrt(q2) / std::sqrt(p.slope * p.slope - g * g), lo, hi);
                    }
                    break;
                }
                case ProfilePiece::Kind::Sphere: {
                    const double rem = p.sphereRadius * p.sphereRadius - q2;
                    w = clampTo(-g * std::sqrt((rem > 0.0 ? rem : 0.0) / (1.0 + g * g)), lo, hi);
                    break;
                }
                default:
                    w = lo;
                    break;
            }
            const double s = clampTo(u + w, 0.0, len);
            double h;
            switch (p.kind) {
                case ProfilePiece::Kind::Flat:
                    h = p.base;
                    break;
                case ProfilePiece::Kind::Cone:
                    h = p.base + p.slope * (std::sqrt(q2 + w * w) - p.r0);
                    break;
                case ProfilePiece::Kind::Sphere: {
                    const double rem = p.sphereRadius * p.sphereRadius - (q2 + w * w);
                    h = p.base + p.sphereRadius - std::sqrt(rem > 0.0 ? rem : 0.0);
                    break;
                }
                default:
                    h = 0.0;
                    break;
            }
            const double v = az + g * s + h;
            if (v < best) {
                best = v;
            }
        }
    }
    return best;
}

}  // namespace

template <SweepMode M>
void Sweeper::applyCell(std::size_t idx, double value, std::size_t& count) {
    if constexpr (M == SweepMode::Cut) {
        const double clamped = value < f_.bottomZ_ ? f_.bottomZ_ : value;
        const float nv = static_cast<float>(clamped);
        if (nv < f_.z_[idx]) {
            f_.z_[idx] = nv;
            if (f_.cut_[idx] == 0) {
                f_.cut_[idx] = 1;
                ++f_.cutCount_;
            }
            if (!f_.ids_.empty()) {
                f_.ids_[idx] = passId_;
            }
            ++count;
        }
    } else {
        if (value < static_cast<double>(f_.z_[idx]) - probeTol_) {
            ++count;
        }
    }
}

template <SweepMode M>
std::size_t Sweeper::segmentImpl(const ToolShape& tool, double ax, double ay, double az, double bx, double by,
                                 double bz) {
    std::size_t count = 0;
    const double zlow = az < bz ? az : bz;
    if (!(zlow < f_.topZ_)) {
        return 0;  // the stock surface is its highest point: nothing above it to remove
    }
    // Only the part of the tool low enough to reach the stock surface matters (cells farther away
    // keep their height): shrinks the footprint of a V-bit or ball at shallow depth.
    const double radius = reachOf(tool, f_.topZ_ - zlow);
    const double r2max = radius * radius;
    const double cs = f_.cellSize_;
    const double ox = f_.origin_.x;
    const double oy = f_.origin_.y;
    const std::size_t nx = f_.nx_;
    const std::size_t ny = f_.ny_;
    const double dxs = bx - ax;
    const double dys = by - ay;
    const double len = std::sqrt(dxs * dxs + dys * dys);
    const bool stamp = !(len > kMinSegment);
    const double ymin = (ay < by ? ay : by) - radius;
    const double ymax = (ay < by ? by : ay) + radius;
    const std::size_t jlo = lowIndex(ymin, oy, cs, ny);
    const std::size_t jhi = highIndex(ymax, oy, cs, ny);
    double ex = 0.0;
    double ey = 0.0;
    double g = 0.0;
    if (!stamp) {
        ex = dxs / len;
        ey = dys / len;
        g = (bz - az) / len;
    }
    for (std::size_t j = jlo; j < jhi; ++j) {
        const double cy = oy + (static_cast<double>(j) + 0.5) * cs;
        double xmin;
        double xmax;
        if (stamp || dys == 0.0) {
            if (std::fabs(cy - ay) > radius) {
                continue;
            }
            xmin = (ax < bx ? ax : bx) - radius;
            xmax = (ax < bx ? bx : ax) + radius;
            if (stamp) {
                xmin = ax - radius;
                xmax = ax + radius;
            }
        } else {
            const double t1 = (cy - radius - ay) / dys;
            const double t2 = (cy + radius - ay) / dys;
            double tlo = t1 < t2 ? t1 : t2;
            double thi = t1 < t2 ? t2 : t1;
            tlo = tlo < 0.0 ? 0.0 : tlo;
            thi = thi > 1.0 ? 1.0 : thi;
            if (tlo > thi) {
                continue;
            }
            const double x1 = ax + tlo * dxs;
            const double x2 = ax + thi * dxs;
            xmin = (x1 < x2 ? x1 : x2) - radius;
            xmax = (x1 < x2 ? x2 : x1) + radius;
        }
        const std::size_t ilo = lowIndex(xmin, ox, cs, nx);
        const std::size_t ihi = highIndex(xmax, ox, cs, nx);
        for (std::size_t i = ilo; i < ihi; ++i) {
            const std::size_t idx = j * nx + i;
            work_ += 1.0;
            if (zlow >= static_cast<double>(f_.z_[idx])) {
                continue;
            }
            const double cx = ox + (static_cast<double>(i) + 0.5) * cs;
            const double dx = cx - ax;
            const double dy = cy - ay;
            double v;
            if (stamp) {
                const double r2 = dx * dx + dy * dy;
                if (r2 > r2max) {
                    continue;
                }
                v = zlow + tool.heightAt(std::sqrt(r2));
            } else {
                const double u = dx * ex + dy * ey;
                const double qs = ex * dy - ey * dx;
                const double q2 = qs * qs;
                if (q2 > r2max) {
                    continue;
                }
                v = segmentMin(tool, q2, u, len, az, g);
                if (!(v < kInf)) {
                    continue;
                }
            }
            applyCell<M>(idx, v, count);
        }
    }
    return count;
}

template <SweepMode M>
std::size_t Sweeper::planarLeaf(const ToolShape& tool, const ArcGeom& g) {
    std::size_t count = 0;
    const double z = g.za;
    if (!(z < f_.topZ_)) {
        return 0;
    }
    const double radius = reachOf(tool, f_.topZ_ - z);
    const double r2max = radius * radius;
    const double cs = f_.cellSize_;
    const double ox = f_.origin_.x;
    const double oy = f_.origin_.y;
    const std::size_t nx = f_.nx_;
    const std::size_t ny = f_.ny_;
    const double pax = g.cx + g.ax;
    const double pay = g.cy + g.ay;
    const double pbx = g.cx + g.bx;
    const double pby = g.cy + g.by;
    const double chx = g.bx - g.ax;
    const double chy = g.by - g.ay;
    const double margin = (chx * chx + chy * chy) / (4.0 * g.rho) + radius;
    const std::size_t ilo = lowIndex((pax < pbx ? pax : pbx) - margin, ox, cs, nx);
    const std::size_t ihi = highIndex((pax < pbx ? pbx : pax) + margin, ox, cs, nx);
    const std::size_t jlo = lowIndex((pay < pby ? pay : pby) - margin, oy, cs, ny);
    const std::size_t jhi = highIndex((pay < pby ? pby : pay) + margin, oy, cs, ny);
    const double d = static_cast<double>(g.dir);
    for (std::size_t j = jlo; j < jhi; ++j) {
        const double py = oy + (static_cast<double>(j) + 0.5) * cs - g.cy;
        for (std::size_t i = ilo; i < ihi; ++i) {
            const std::size_t idx = j * nx + i;
            work_ += 1.0;
            if (z >= static_cast<double>(f_.z_[idx])) {
                continue;
            }
            const double px = ox + (static_cast<double>(i) + 0.5) * cs - g.cx;
            // Inside the angular wedge of the (at most quarter) arc: distance to the circle;
            // otherwise distance to the nearer end point.
            const double c1 = d * (g.ax * py - g.ay * px);
            const double c2 = d * (px * g.by - py * g.bx);
            double dist;
            if (c1 >= 0.0 && c2 >= 0.0) {
                dist = std::fabs(std::sqrt(px * px + py * py) - g.rho);
                if (dist > radius) {
                    continue;
                }
            } else {
                const double da = (px - g.ax) * (px - g.ax) + (py - g.ay) * (py - g.ay);
                const double db = (px - g.bx) * (px - g.bx) + (py - g.by) * (py - g.by);
                const double m = da < db ? da : db;
                if (m > r2max) {
                    continue;
                }
                dist = std::sqrt(m);
            }
            applyCell<M>(idx, z + tool.heightAt(dist), count);
        }
    }
    return count;
}

template <SweepMode M>
std::size_t Sweeper::arcRecurse(const ToolShape& tool, const ArcGeom& g, bool helix, int depth) {
    const double zlow = g.za < g.zb ? g.za : g.zb;
    if (!(zlow < f_.topZ_)) {
        return 0;
    }
    const double radius = reachOf(tool, f_.topZ_ - zlow);
    const double cs = f_.cellSize_;
    const double chx = g.bx - g.ax;
    const double chy = g.by - g.ay;
    const double chord2 = chx * chx + chy * chy;
    const double sag = chord2 / (4.0 * g.rho);  // >= the true sagitta of an arc of at most a quarter turn
    const double margin = sag + radius;
    const double pax = g.cx + g.ax;
    const double pay = g.cy + g.ay;
    const double pbx = g.cx + g.bx;
    const double pby = g.cy + g.by;
    const std::size_t ilo = lowIndex((pax < pbx ? pax : pbx) - margin, f_.origin_.x, cs, f_.nx_);
    const std::size_t ihi = highIndex((pax < pbx ? pbx : pax) + margin, f_.origin_.x, cs, f_.nx_);
    const std::size_t jlo = lowIndex((pay < pby ? pay : pby) - margin, f_.origin_.y, cs, f_.ny_);
    const std::size_t jhi = highIndex((pay < pby ? pby : pay) + margin, f_.origin_.y, cs, f_.ny_);
    if (ilo >= ihi || jlo >= jhi) {
        return 0;
    }
    const double chordMax = (2.0 * radius > 4.0 * cs ? 2.0 * radius : 4.0 * cs);
    const bool leaf = depth >= kMaxArcDepth || (helix ? sag <= cs / 64.0 : chord2 <= chordMax * chordMax);
    if (leaf) {
        if (helix) {
            return segmentImpl<M>(tool, pax, pay, g.za, pbx, pby, g.zb);
        }
        return planarLeaf<M>(tool, g);
    }
    const double sx = g.ax + g.bx;
    const double sy = g.ay + g.by;
    const double sl = std::sqrt(sx * sx + sy * sy);
    ArcGeom first = g;
    ArcGeom second = g;
    const double mx = sx * g.rho / sl;
    const double my = sy * g.rho / sl;
    const double zm = (g.za + g.zb) * 0.5;
    first.full = false;
    first.bx = mx;
    first.by = my;
    first.zb = zm;
    second.full = false;
    second.ax = mx;
    second.ay = my;
    second.za = zm;
    return arcRecurse<M>(tool, first, helix, depth + 1) + arcRecurse<M>(tool, second, helix, depth + 1);
}

template <SweepMode M>
std::size_t Sweeper::arcImpl(const ToolShape& tool, const ArcGeom& g) {
    const bool helix = g.za != g.zb;
    const double d = static_cast<double>(g.dir);
    double theta = detail::kTwoPi;
    if (helix && !g.full) {
        const double cr = g.ax * g.by - g.ay * g.bx;
        const double dt = g.ax * g.bx + g.ay * g.by;
        theta = detail::atan2Det(d * cr, dt);
        if (theta <= 0.0) {
            theta += detail::kTwoPi;
        }
    }
    std::size_t count = 0;
    double cax = g.ax;
    double cay = g.ay;
    double angle = 0.0;
    double zcur = g.za;
    for (int k = 0; k < 4; ++k) {
        // Quarter turn from the current radius vector in the direction of travel.
        const double nx = g.dir > 0 ? -cay : cay;
        const double ny = g.dir > 0 ? cax : -cax;
        bool last = k == 3;
        if (!g.full && !last) {
            last = d * (cax * g.by - cay * g.bx) >= 0.0 && d * (g.bx * ny - g.by * nx) >= 0.0;
        }
        ArcGeom piece = g;
        piece.full = false;
        piece.ax = cax;
        piece.ay = cay;
        piece.za = zcur;
        if (last) {
            piece.bx = g.full ? nx : g.bx;
            piece.by = g.full ? ny : g.by;
            piece.zb = g.zb;
        } else {
            piece.bx = nx;
            piece.by = ny;
            angle += detail::kHalfPi;
            piece.zb = g.za + (g.zb - g.za) * (angle / theta);
        }
        count += arcRecurse<M>(tool, piece, helix, 0);
        if (last) {
            break;
        }
        cax = nx;
        cay = ny;
        zcur = piece.zb;
    }
    return count;
}

std::size_t Sweeper::segment(const ToolShape& tool, double ax, double ay, double az, double bx, double by, double bz,
                             SweepMode mode) {
    return mode == SweepMode::Cut ? segmentImpl<SweepMode::Cut>(tool, ax, ay, az, bx, by, bz)
                                  : segmentImpl<SweepMode::Probe>(tool, ax, ay, az, bx, by, bz);
}

std::size_t Sweeper::arc(const ToolShape& tool, const ArcGeom& g, SweepMode mode) {
    return mode == SweepMode::Cut ? arcImpl<SweepMode::Cut>(tool, g) : arcImpl<SweepMode::Probe>(tool, g);
}

}  // namespace carvetto::simulate
