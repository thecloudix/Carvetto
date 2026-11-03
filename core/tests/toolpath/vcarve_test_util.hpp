#pragma once

// Shared fixtures for the V-carve tests (T-0153): tools and the generative case generator used
// by the cone/wall sweep (AC1).

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "../geometry/medial_axis_glyphs.hpp"
#include "../geometry/test_rng.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/vcarve.hpp"

namespace carvetto::test::vcarve {

using geometry::Box2;
using geometry::Contour;
using geometry::Vec2;

inline tooling::Tool vBit(double angleDeg, double diameter, const std::string& name = "V-bit", int number = 1) {
    tooling::Tool t;
    t.id = name;
    t.name = name;
    t.type = tooling::ToolType::VBit;
    t.diameter = diameter;
    t.includedAngle = angleDeg * geometry::kPi / 180.0;
    t.defaultCuttingData.toolNumber = number;
    t.defaultCuttingData.feedRate = 1500.0;
    t.defaultCuttingData.plungeRate = 500.0;
    t.defaultCuttingData.spindleSpeed = 18000.0;
    t.defaultCuttingData.passDepth = 1.0;
    t.defaultCuttingData.stepover = 0.25;
    return t;
}

inline tooling::Tool ballNose(double diameter, const std::string& name = "Ball", int number = 2) {
    tooling::Tool t = vBit(0.0, diameter, name, number);
    t.type = tooling::ToolType::BallNose;
    t.includedAngle = 0.0;
    return t;
}

inline tooling::Tool endMill(double diameter, const std::string& name = "End mill", int number = 3) {
    tooling::Tool t;
    t.id = name;
    t.name = name;
    t.type = tooling::ToolType::EndMill;
    t.diameter = diameter;
    t.defaultCuttingData.toolNumber = number;
    t.defaultCuttingData.feedRate = 1200.0;
    t.defaultCuttingData.plungeRate = 400.0;
    t.defaultCuttingData.spindleSpeed = 20000.0;
    t.defaultCuttingData.passDepth = 1.0;
    t.defaultCuttingData.stepover = 0.4 * diameter;
    return t;
}

inline Contour polygon(std::vector<Vec2> pts) {
    return Contour::fromPolyline(geometry::Polyline{std::move(pts), true});
}

/// Star-shaped random polygon round `c` (n vertices, radii in [rMin, rMax]).
inline Contour starPolygon(SplitMix& rng, Vec2 c, int n, double rMin, double rMax) {
    std::vector<Vec2> pts;
    for (int i = 0; i < n; ++i) {
        const double a = geometry::kTwoPi * (static_cast<double>(i) + rng.uniform(-0.3, 0.3)) / n;
        const double r = rng.uniform(rMin, rMax);
        pts.push_back({c.x + r * std::cos(a), c.y + r * std::sin(a)});
    }
    return polygon(std::move(pts));
}

/// Rectangle with rounded corners (lines and quarter arcs), counter-clockwise.
inline Contour roundedRect(Box2 b, double radius) {
    using geometry::ArcDirection;
    using geometry::ArcSpan;
    using geometry::LineSpan;
    const double r = radius;
    Contour c;
    c.closed = true;
    const double pi = geometry::kPi;
    c.append(LineSpan{{b.min.x + r, b.min.y}, {b.max.x - r, b.min.y}});
    c.append(ArcSpan{{b.max.x - r, b.min.y + r}, r, -pi / 2, 0.0, ArcDirection::CCW});
    c.append(LineSpan{{b.max.x, b.min.y + r}, {b.max.x, b.max.y - r}});
    c.append(ArcSpan{{b.max.x - r, b.max.y - r}, r, 0.0, pi / 2, ArcDirection::CCW});
    c.append(LineSpan{{b.max.x - r, b.max.y}, {b.min.x + r, b.max.y}});
    c.append(ArcSpan{{b.min.x + r, b.max.y - r}, r, pi / 2, pi, ArcDirection::CCW});
    c.append(LineSpan{{b.min.x, b.max.y - r}, {b.min.x, b.min.y + r}});
    c.append(ArcSpan{{b.min.x + r, b.min.y + r}, r, pi, 1.5 * pi, ArcDirection::CCW});
    return c;
}

/// Closed blob of `n` cubic Beziers round `c` (smooth, tangent-continuous).
inline Contour bezierBlob(SplitMix& rng, Vec2 c, int n, double rMin, double rMax) {
    std::vector<Vec2> pts;
    std::vector<Vec2> tangents;
    for (int i = 0; i < n; ++i) {
        const double a = geometry::kTwoPi * i / n;
        const double r = rng.uniform(rMin, rMax);
        pts.push_back({c.x + r * std::cos(a), c.y + r * std::sin(a)});
        const double k = r * geometry::kTwoPi / n / 3.0;
        tangents.push_back({-std::sin(a) * k, std::cos(a) * k});
    }
    Contour out;
    out.closed = true;
    for (int i = 0; i < n; ++i) {
        const auto j = static_cast<std::size_t>((i + 1) % n);
        const auto k = static_cast<std::size_t>(i);
        out.append(geometry::BezierSpan{pts[k], pts[k] + tangents[k], pts[j] - tangents[j], pts[j]});
    }
    return out;
}

/// One generated V-carve job.
struct SweepCase {
    std::string what;
    std::vector<Contour> region;
    tooling::Tool tool;
    std::vector<toolpath::PocketTool> clearance;
    toolpath::VCarveParams params;
};

/// Case `k` of the generative sweep: the region kind, tool, flat depth, clearance tools, start
/// depth and tolerance all drawn from `k`.
inline SweepCase sweepCase(std::uint64_t k) {
    SplitMix rng{0x5EEDC0DEULL ^ (k * 0x9E3779B97F4A7C15ULL)};
    SweepCase sc;
    const auto kind = static_cast<int>(k % 8);
    const Vec2 o{rng.uniform(-50.0, 50.0), rng.uniform(-50.0, 50.0)};
    switch (kind) {
        case 0: {
            sc.what = "star polygon";
            sc.region.push_back(starPolygon(rng, o, 3 + static_cast<int>(rng.next() % 10), 2.0, 8.0));
            break;
        }
        case 1: {
            sc.what = "star polygon with a hole";
            sc.region.push_back(starPolygon(rng, o, 5 + static_cast<int>(rng.next() % 8), 6.0, 10.0));
            if (rng.next() % 2 == 0) {
                sc.region.push_back(Contour::circle(o, rng.uniform(0.5, 3.0)));
            } else {
                Contour hole = starPolygon(rng, o, 3 + static_cast<int>(rng.next() % 5), 1.0, 3.5);
                hole.reverse();
                sc.region.push_back(hole);
            }
            break;
        }
        case 2: {
            sc.what = "symmetric polygon";
            const int n = 3 + static_cast<int>(rng.next() % 6);
            if (n == 3) {
                const double w = rng.uniform(4.0, 20.0);
                const double h = rng.uniform(1.0, 10.0);
                sc.region.push_back(polygon({o, o + Vec2{w, 0.0}, o + Vec2{0.5 * w, h}}));
            } else {
                const double r = rng.uniform(2.0, 8.0);
                std::vector<Vec2> pts;
                for (int i = 0; i < n; ++i) {
                    const double a = geometry::kTwoPi * i / n;
                    pts.push_back({o.x + r * std::cos(a), o.y + r * std::sin(a)});
                }
                sc.region.push_back(polygon(std::move(pts)));
            }
            break;
        }
        case 3: {
            sc.what = "rounded rectangle";
            const double w = rng.uniform(2.0, 20.0);
            const double h = rng.uniform(1.0, 12.0);
            const double r = rng.uniform(0.1, 0.49) * std::min(w, h);
            sc.region.push_back(roundedRect(Box2{o, o + Vec2{w, h}}, r));
            break;
        }
        case 4: {
            sc.what = "glyph";
            medial::appendGlyph(sc.region, static_cast<int>(rng.next() % 5), o);
            break;
        }
        case 5: {
            sc.what = "Bezier blob";
            sc.region.push_back(bezierBlob(rng, o, 3 + static_cast<int>(rng.next() % 6), 2.0, 7.0));
            break;
        }
        case 6: {
            sc.what = "thin strip";
            const double len = rng.uniform(3.0, 30.0);
            const double width = rng.uniform(0.03, 1.0);
            sc.region.push_back(polygon({o, o + Vec2{len, 0.0}, o + Vec2{len, width}, o + Vec2{0.0, width}}));
            break;
        }
        default: {
            sc.what = "ring and neighbours";
            const double r = rng.uniform(3.0, 8.0);
            sc.region.push_back(Contour::circle(o, r));
            sc.region.push_back(Contour::circle(o, r * rng.uniform(0.3, 0.8), geometry::ArcDirection::CW));
            sc.region.push_back(starPolygon(rng, o + Vec2{r + 6.0, 0.0}, 4 + static_cast<int>(rng.next() % 4), 1.5, 4.0));
            break;
        }
    }
    if (rng.next() % 4 == 0) {
        sc.tool = ballNose(rng.uniform(1.0, 10.0));
    } else {
        static const double angles[] = {30.0, 60.0, 90.0, 120.0, 150.0};
        sc.tool = vBit(angles[rng.next() % 5], rng.uniform(3.0, 20.0));
    }
    toolpath::VCarveParams& p = sc.params;
    static const double chordals[] = {0.01, 0.02, 0.05};
    p.tolerance.chordal = chordals[rng.next() % 3];
    if (rng.next() % 3 == 0) {
        p.startDepth = rng.uniform(0.0, 3.0);
    }
    if (rng.next() % 2 == 0) {
        p.flatDepth = rng.uniform(0.2, 2.5);
        if (rng.next() % 2 == 0) {
            sc.clearance.push_back({endMill(rng.uniform(0.8, 4.0), "Clear 1", 4), std::nullopt});
            if (rng.next() % 3 == 0) {
                sc.clearance.push_back({endMill(rng.uniform(0.2, 0.8), "Clear 2", 5), std::nullopt});
            }
        }
        p.clearanceStrategy = rng.next() % 2 == 0 ? toolpath::PocketStrategy::Offset : toolpath::PocketStrategy::Raster;
        p.clearanceRasterAngle = rng.uniform(0.0, geometry::kPi / 2);
        if (sc.clearance.empty()) {
            // The carving tool clears the flat areas itself: its stepover must fit the width it
            // cuts at the flat depth.
            const double depth = std::min(p.flatDepth, toolpath::vcarveFullWidthDepth(sc.tool));
            sc.tool.defaultCuttingData.stepover =
                std::min(0.25, 0.8 * tooling::cuttingWidthAtDepth(sc.tool, depth));
        }
    }
    sc.what += sc.tool.type == tooling::ToolType::BallNose ? ", ball nose" : ", V-bit";
    return sc;
}

}  // namespace carvetto::test::vcarve
