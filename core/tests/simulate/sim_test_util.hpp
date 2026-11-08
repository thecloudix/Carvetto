#pragma once

// Shared fixtures for the simulator tests (T-0222).

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/simulate/simulator.hpp"
#include "carvetto/toolpath/toolpath.hpp"
#include "../geometry/test_rng.hpp"
#include "oracle.hpp"

namespace carvetto::test::sim {

using simulate::Heightfield;
using simulate::Simulator;
using simulate::Stock;
using simulate::ToolShape;
using toolpath::Toolpath;
using geometry::Vec2;

constexpr double kDeg = kPi / 180.0;

/// Portable PRNG (SplitMix64) with a [0, 1) draw.
struct Rng {
    carvetto::test::SplitMix m;
    explicit Rng(std::uint64_t seed) : m{seed} {}
    double uniform() { return m.uniform(0.0, 1.0); }
    double uniform(double lo, double hi) { return m.uniform(lo, hi); }
    std::uint64_t next() { return m.next(); }
};

struct NamedTool {
    std::string name;
    ToolShape shape;
    Profile profile;
};

/// The tool shapes of the tests, each with the independent profile of the same cutter.
inline std::vector<NamedTool> allTools() {
    std::vector<NamedTool> t;
    t.push_back({"flat 3", *ToolShape::flatEndMill(3.0), flatProfile(3.0)});
    t.push_back({"ball 3", *ToolShape::ballNose(3.0), ballProfile(3.0)});
    t.push_back({"V 90", *ToolShape::vBit(6.0, 90 * kDeg), vProfile(6.0, 90 * kDeg)});
    t.push_back({"V 60", *ToolShape::vBit(6.0, 60 * kDeg), vProfile(6.0, 60 * kDeg)});
    t.push_back({"V 20", *ToolShape::vBit(4.0, 20 * kDeg), vProfile(4.0, 20 * kDeg)});
    t.push_back({"engraving 60 tip 1", *ToolShape::vBit(6.0, 60 * kDeg, 1.0), vProfile(6.0, 60 * kDeg, 1.0)});
    t.push_back({"tapered 30 ball 2", *ToolShape::taperedBallNose(6.0, 30 * kDeg, 2.0),
                 taperedProfile(6.0, 30 * kDeg, 2.0)});
    t.push_back({"tapered 60 ball 4", *ToolShape::taperedBallNose(8.0, 60 * kDeg, 4.0),
                 taperedProfile(8.0, 60 * kDeg, 4.0)});
    return t;
}

inline Stock stockOf(double w, double h, double cell, double thickness = 10.0) {
    Stock s;
    s.width = w;
    s.height = h;
    s.thickness = thickness;
    s.cellSize = cell;
    return s;
}

/// Cuts `moves` (preceded by a rapid to `start`) with `tool` into a fresh stock.
inline Simulator run(const Stock& stock, const ToolShape& tool, const Vec3& start, const std::vector<Move>& moves,
                     std::uint32_t id = 1) {
    Simulator sim(stock);
    Toolpath tp;
    tp.tool.feedRate = 1000.0;
    tp.tool.plungeRate = 300.0;
    tp.append(Move::rapid(start));
    for (const Move& m : moves) {
        tp.append(m);
    }
    sim.addToolpath(tp, tool, id);
    sim.advanceAll();
    return sim;
}

inline double distToSegment(double px, double py, double ax, double ay, double bx, double by) {
    const double dx = bx - ax;
    const double dy = by - ay;
    const double l2 = dx * dx + dy * dy;
    double t = l2 > 0 ? ((px - ax) * dx + (py - ay) * dy) / l2 : 0.0;
    t = std::max(0.0, std::min(1.0, t));
    return std::hypot(px - (ax + t * dx), py - (ay + t * dy));
}

}  // namespace carvetto::test::sim
