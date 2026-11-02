#pragma once

// Shared toolpath fixture for the shipped-post golden tests (T-0110). Every post in posts/ is run
// against exactly these toolpaths and compared byte for byte with
// core/tests/golden/post/<post>_fixture.nc.
//
// The fixture covers, in order:
//   1. "Profile"  (tool 1, 6 mm end mill): a 50 mm square at Z-3.
//   2. "Pocket"   (tool 1, same tool -> NEW_SEGMENT): a full CCW circle (r 5), a CW circle written
//                 as two half arcs (r 8) and a CCW quarter arc, at Z-2.
//   3. "Drill"    (tool 2, 3 mm drill -> TOOLCHANGE + SPINDLE_ON): a peck-drilled hole (three
//                 pecks with partial retracts and a 0.5 s dwell at the bottom) and a single-plunge
//                 hole with a 0.25 s dwell.

#include <string>
#include <vector>

#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/toolpath.hpp"

namespace carvetto::test {

/// Safe Z of the fixture (mm).
inline constexpr double kFixtureSafeZ = 5.0;

inline toolpath::ToolSnapshot fixtureEndMill() {
    toolpath::ToolSnapshot t;
    t.name = "End Mill (6 mm)";
    t.toolNumber = 1;
    t.diameter = 6.0;
    t.feedRate = 1000.0;
    t.plungeRate = 300.0;
    t.spindleSpeed = 18000.0;
    return t;
}

inline toolpath::ToolSnapshot fixtureDrill() {
    toolpath::ToolSnapshot t;
    t.name = "Drill (3 mm)";
    t.toolNumber = 2;
    t.diameter = 3.0;
    t.feedRate = 600.0;
    t.plungeRate = 200.0;
    t.spindleSpeed = 10000.0;
    return t;
}

/// The three fixture toolpaths described at the top of this file.
inline std::vector<toolpath::Toolpath> fixtureToolpaths() {
    using toolpath::Move;
    const double s = kFixtureSafeZ;

    toolpath::Toolpath profile;
    profile.name = "Profile";
    profile.tool = fixtureEndMill();
    profile.moves = {Move::rapid({10, 10, s}),  Move::plunge({10, 10, -3}), Move::feed({60, 10, -3}),
                     Move::feed({60, 60, -3}),  Move::feed({10, 60, -3}),   Move::feed({10, 10, -3}),
                     Move::retract({10, 10, s})};

    toolpath::Toolpath pocket;
    pocket.name = "Pocket";
    pocket.tool = fixtureEndMill();
    pocket.moves = {
        Move::rapid({85, 30, s}),
        Move::plunge({85, 30, -2}),
        Move::arcCCW({85, 30, -2}, {90, 30}),  // full circle, r 5
        Move::feed({82, 30, -2}),
        Move::arcCW({98, 30, -2}, {90, 30}),   // upper half, r 8
        Move::arcCW({82, 30, -2}, {90, 30}),   // lower half, r 8
        Move::arcCCW({90, 22, -2}, {90, 30}),  // quarter, 180 deg -> 270 deg
        Move::retract({90, 22, s}),
    };

    toolpath::Toolpath drill;
    drill.name = "Drill";
    drill.tool = fixtureDrill();
    drill.moves = {
        // Peck-drilled hole: three pecks, partial retracts to clear chips, dwell at the bottom.
        Move::rapid({30, 30, s}),
        Move::plunge({30, 30, -2}),
        Move::retract({30, 30, 1}),
        Move::plunge({30, 30, -4}),
        Move::retract({30, 30, 1}),
        Move::plunge({30, 30, -6}),
        Move::dwell({30, 30, -6}, 0.5),
        Move::retract({30, 30, s}),
        // Single-plunge hole.
        Move::rapid({40, 30, s}),
        Move::plunge({40, 30, -3}),
        Move::dwell({40, 30, -3}, 0.25),
        Move::retract({40, 30, s}),
    };
    return {profile, pocket, drill};
}

inline post::PostContext fixtureContext() {
    post::PostContext ctx;
    ctx.fileName = "fixture";
    ctx.product = "Carvetto";
    ctx.home = {0, 0, 20};
    ctx.safeZ = kFixtureSafeZ;
    return ctx;
}

}  // namespace carvetto::test
