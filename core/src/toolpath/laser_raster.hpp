#pragma once

// Shared machinery of the laser image toolpaths (T-0324, ADR-0104): exact nearest-pixel runs along a
// straight line over a bitmap, and the strokes (beam-on pieces with their overscan) built from them.
// Private to core/src/toolpath.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/toolpath/image_engrave.hpp"
#include "carvetto/toolpath/laser.hpp"
#include "carvetto/toolpath/operation.hpp"
#include "carvetto/toolpath/toolpath.hpp"
#include "carvetto/toolpath/move.hpp"

namespace carvetto::toolpath::laser_detail {

using geometry::Box2;
using geometry::Vec2;

/// A stretch of one straight line over which the bitmap's nearest pixel does not change.
struct Run {
    double t0{0.0};  ///< Parameter along the line, 0 at its start, 1 at its end.
    double t1{0.0};
    int value{0};  ///< The pixel value (0..255).
};

/// Index of the pixel under `p` (row 0 at the top of the placement), edges clamped.
inline std::uint8_t pixelAt(const GrayImage& image, const Box2& placement, Vec2 p) {
    const double sx = static_cast<double>(image.width) / placement.width();
    const double sy = static_cast<double>(image.height) / placement.height();
    const double fx = std::floor((p.x - placement.min.x) * sx);
    const double fy = std::floor((placement.max.y - p.y) * sy);
    const int x = static_cast<int>(std::clamp(fx, 0.0, static_cast<double>(image.width - 1)));
    const int y = static_cast<int>(std::clamp(fy, 0.0, static_cast<double>(image.height - 1)));
    return image.pixels[static_cast<std::size_t>(y) * static_cast<std::size_t>(image.width) +
                        static_cast<std::size_t>(x)];
}

/// The runs of equal pixel value along a -> b (both inside the placement): the breakpoints are the exact
/// crossings of the pixel column and row edges, each stretch between two is valued at its middle, and
/// neighbouring stretches of one value merge. Returns the number of edge crossings found (the work done).
inline std::size_t pixelRuns(const GrayImage& image, const Box2& placement, Vec2 a, Vec2 b, std::vector<Run>& out) {
    out.clear();
    const double sx = static_cast<double>(image.width) / placement.width();
    const double sy = static_cast<double>(image.height) / placement.height();
    std::vector<double> ts;
    ts.push_back(0.0);
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    if (dx != 0.0) {
        const double lo = (std::min(a.x, b.x) - placement.min.x) * sx;
        const double hi = (std::max(a.x, b.x) - placement.min.x) * sx;
        for (auto i = static_cast<long long>(std::floor(lo)) + 1; static_cast<double>(i) < hi; ++i) {
            ts.push_back((placement.min.x + static_cast<double>(i) / sx - a.x) / dx);
        }
    }
    if (dy != 0.0) {
        const double lo = (placement.max.y - std::max(a.y, b.y)) * sy;
        const double hi = (placement.max.y - std::min(a.y, b.y)) * sy;
        for (auto j = static_cast<long long>(std::floor(lo)) + 1; static_cast<double>(j) < hi; ++j) {
            ts.push_back((placement.max.y - static_cast<double>(j) / sy - a.y) / dy);
        }
    }
    ts.push_back(1.0);
    const std::size_t crossings = ts.size() - 2;
    std::sort(ts.begin(), ts.end());
    for (std::size_t k = 0; k + 1 < ts.size(); ++k) {
        const double t0 = std::clamp(ts[k], 0.0, 1.0);
        const double t1 = std::clamp(ts[k + 1], 0.0, 1.0);
        if (!(t1 > t0)) {
            continue;
        }
        const double tm = 0.5 * (t0 + t1);
        const int v = pixelAt(image, placement, {a.x + dx * tm, a.y + dy * tm});
        if (!out.empty() && out.back().value == v) {
            out.back().t1 = t1;
        } else {
            out.push_back({t0, t1, v});
        }
    }
    return crossings;
}

/// Clips the segment a -> b to `box` (Liang-Barsky); false when nothing is left.
inline bool clipSegment(const Box2& box, Vec2& a, Vec2& b) {
    double t0 = 0.0;
    double t1 = 1.0;
    const double d[2] = {b.x - a.x, b.y - a.y};
    const double p0[2] = {a.x, a.y};
    const double lo[2] = {box.min.x, box.min.y};
    const double hi[2] = {box.max.x, box.max.y};
    for (int axis = 0; axis < 2; ++axis) {
        if (d[axis] == 0.0) {
            if (p0[axis] < lo[axis] || p0[axis] > hi[axis]) {
                return false;
            }
            continue;
        }
        double ta = (lo[axis] - p0[axis]) / d[axis];
        double tb = (hi[axis] - p0[axis]) / d[axis];
        if (ta > tb) {
            std::swap(ta, tb);
        }
        t0 = std::max(t0, ta);
        t1 = std::min(t1, tb);
    }
    if (!(t1 - t0 > 1e-12)) {
        return false;
    }
    const Vec2 na{a.x + d[0] * t0, a.y + d[1] * t0};
    const Vec2 nb{a.x + d[0] * t1, a.y + d[1] * t1};
    a = na;
    b = nb;
    return true;
}

/// One piece of a stroke: a straight feed to `to` with the beam at `power` (0 = off).
struct Piece {
    Vec2 to;
    double power{0.0};
    int seg{0};  ///< The straight segment of the path it lies on: pieces merge only within one.
};

/// A stretch of beam travel starting at `start`, made of pieces; the first and last carry the beam.
struct Stroke {
    Vec2 start;
    std::vector<Piece> pieces;
    /// Unit direction of the line it lies on (the overscan runs along it).
    Vec2 dir;
};

/// Beam-off gaps at least this long (mm) split a line into separate strokes (a rapid between them); shorter ones
/// are crossed with a beam-off feed.
inline constexpr double kSplitGap = 5.0;

/// One straight stretch of beam travel with one pixel value (-1 = off the bitmap: the beam stays off).
struct Span {
    Vec2 p0;
    Vec2 p1;
    int value{0};
    int seg{0};  ///< Index of the straight segment of the path it lies on (0 for a single line).
};

/// Appends the spans of the line a -> b given its runs.
inline void spansOfLine(Vec2 a, Vec2 b, const std::vector<Run>& runs, std::vector<Span>& out, int seg = 0) {
    const auto at = [&](double t) { return Vec2{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t}; };
    for (const Run& r : runs) {
        out.push_back({at(r.t0), at(r.t1), r.value, seg});
    }
}

/// Builds the strokes of a chain of spans (end to start): `powerOf(value)` is the beam power of a span (0 = off, a
/// value of -1 is always off). Neighbouring pieces of one power merge; a beam-off gap of `kSplitGap` or more splits
/// the chain into separate strokes. `dir` is the direction the strokes' overscan runs in.
template <class PowerOf>
void buildStrokes(const std::vector<Span>& spans, Vec2 dir, const PowerOf& powerOf, std::vector<Stroke>& out) {
    bool open = false;
    Stroke cur;
    std::vector<Piece> gap;  // beam-off pieces after the last lit one
    const auto add = [](std::vector<Piece>& pieces, Vec2 to, double power, int seg) {
        if (!pieces.empty() && pieces.back().power == power && pieces.back().seg == seg) {
            pieces.back().to = to;
        } else {
            pieces.push_back({to, power, seg});
        }
    };
    const auto close = [&]() {
        if (open) {
            out.push_back(std::move(cur));
        }
        cur = Stroke{};
        gap.clear();
        open = false;
    };
    double gapLength = 0.0;
    for (const Span& s : spans) {
        const double p = s.value < 0 ? 0.0 : powerOf(s.value);
        if (p > 0.0) {
            if (!open) {
                cur = Stroke{};
                cur.start = s.p0;
                cur.dir = dir;
                open = true;
            } else {
                for (const Piece& g : gap) {
                    add(cur.pieces, g.to, 0.0, g.seg);
                }
            }
            gap.clear();
            gapLength = 0.0;
            add(cur.pieces, s.p1, p, s.seg);
        } else if (open) {
            gapLength += std::hypot(s.p1.x - s.p0.x, s.p1.y - s.p0.y);
            if (gapLength >= kSplitGap) {
                close();
                gapLength = 0.0;
            } else {
                add(gap, s.p1, 0.0, s.seg);
            }
        }
    }
    close();
}

/// Appends the moves of `strokes` at Z 0: rapid to the start (to the overscan point first, then a beam-off feed to
/// the start), the pieces with their power, then the beam-off overscan run. Returns the end position.
inline Vec2 emitStrokes(const std::vector<Stroke>& strokes, double overscan, double z, std::vector<Move>& moves,
                        Vec2 last) {
    for (const Stroke& s : strokes) {
        if (overscan > 0.0) {
            moves.push_back(Move::rapid({s.start.x - s.dir.x * overscan, s.start.y - s.dir.y * overscan, z}));
            moves.push_back(Move::feed({s.start.x, s.start.y, z}));
        } else {
            moves.push_back(Move::rapid({s.start.x, s.start.y, z}));
        }
        last = s.start;
        for (const Piece& p : s.pieces) {
            Move m = Move::feed({p.to.x, p.to.y, z});
            m.jetPower = p.power;
            moves.push_back(m);
            last = p.to;
        }
        if (overscan > 0.0) {
            last = {last.x + s.dir.x * overscan, last.y + s.dir.y * overscan};
            moves.push_back(Move::feed({last.x, last.y, z}));
        }
    }
    return last;
}

/// Floyd-Steinberg error diffusion of a darkness image (0..255): every pixel becomes 0 or 255. Scanned row by
/// row from the top, left to right; a pixel is dark (255) when its value plus the carried error is at least 128; its
/// error goes 7/16 right, 3/16 down-left, 5/16 down, 1/16 down-right (the remainder to the last, so none is lost;
/// shares that fall off the image are dropped). Integer arithmetic: the result is exact and platform independent.
inline GrayImage ditherDarkness(const GrayImage& in) {
    GrayImage out = in;
    if (!in.valid()) {
        return out;
    }
    const auto w = static_cast<std::size_t>(in.width);
    const auto h = static_cast<std::size_t>(in.height);
    // Values carry the error in 1/256 units (255 * 256 fits an int).
    std::vector<long> cur(w + 2, 0);
    std::vector<long> nxt(w + 2, 0);
    for (std::size_t y = 0; y < h; ++y) {
        std::fill(nxt.begin(), nxt.end(), 0L);
        for (std::size_t x = 0; x < w; ++x) {
            const long v = static_cast<long>(in.pixels[y * w + x]) * 256 + cur[x + 1];
            const bool dark = v >= 128 * 256;
            const long err = v - (dark ? 255L * 256 : 0L);
            out.pixels[y * w + x] = dark ? 255 : 0;
            const long a = err * 7 / 16;
            const long b = err * 3 / 16;
            const long c = err * 5 / 16;
            const long d = err - a - b - c;
            cur[x + 2] += a;
            nxt[x] += b;
            nxt[x + 1] += c;
            nxt[x + 2] += d;
        }
        std::swap(cur, nxt);
        // The slots at the image edges (indices 0 and w + 1) are dropped shares; they are never read.
    }
    return out;
}

/// Sets the toolpath's tool snapshot for a laser: speed from `moveSpeed` or the cutting data's feed rate, no spindle,
/// marked as a laser's (as Laser Cut / Fill does, ADR-0099). Returns the problem text, empty when fine.
inline std::string prepareLaserTool(const tooling::Tool& tool, const std::optional<tooling::CuttingData>& cuttingData,
                                    double moveSpeed, Toolpath& out) {
    tooling::CuttingData data = effectiveCuttingData(tool, cuttingData);
    const double speed = moveSpeed > 0.0 ? moveSpeed : data.feedRate;
    if (!std::isfinite(speed) || !(speed > 0.0)) {
        return "tool '" + tool.name + "': the move speed (or the cutting data's feed rate) must be greater than zero";
    }
    data.feedRate = speed;
    data.plungeRate = speed;
    out.tool = toolSnapshot(tool, data);
    out.tool.spindleSpeed = 0.0;
    out.tool.laser = true;
    return {};
}

}  // namespace carvetto::toolpath::laser_detail
