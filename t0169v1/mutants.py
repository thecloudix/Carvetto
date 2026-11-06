#!/usr/bin/env python3
# usage: mutants.py <orig pocket.cpp> <name> <out pocket.cpp>: writes one T-0169 mutant (see MUT below)
import sys
src = open(sys.argv[1]).read(); name = sys.argv[2]
OLD = """    /// Points of a way closer than this are merged"""
ADD = """    bool allowsOld(const RegionIndex& region, Vec2 a, Vec2 b) const { return region.segmentInside(a, b, strict_, lenient_).lenient; }
    bool allowsOld(const std::vector<Ring>& rings, Vec2 a, Vec2 b) const { return segmentInside(a, b, rings, lenient_); }
"""
src = src.replace(OLD, ADD + OLD, 1)
def sub(a, b, count=1):
    global src
    assert a in src, (name, a)
    src = src.replace(a, b, count)
MUT = {
 'all_old': lambda: (sub("return v.strict || (v.lenient && clearOfWalls(a, b));", "return v.lenient;"),
                     sub("        if (segmentInside(a, b, rings, strict_)) {\n            return true;\n        }\n        return segmentInside(a, b, rings, lenient_) && clearOfWalls(a, b);", "        return segmentInside(a, b, rings, lenient_);"),
                     sub("double mergeDistance() const { return strict_.accept; }", "double mergeDistance() const { return lenient_.accept; }")),
 'sag0': lambda: sub("const double wallSag = curvedWalls ? tol.chordal : 0.0;", "const double wallSag = curvedWalls ? 0.0 * tol.chordal : 0.0;"),
 'sag_half': lambda: sub("const double wallSag = curvedWalls ? tol.chordal : 0.0;", "const double wallSag = curvedWalls ? 0.5 * tol.chordal : 0.0;"),
 'strict_only': lambda: sub("return walls_ != nullptr && !walls_->grid().anyCloser(a, b, need_);", "(void)a; (void)b; return false;"),
 'walls_always': lambda: sub("return walls_ != nullptr && !walls_->grid().anyCloser(a, b, need_);", "(void)a; (void)b; return true;"),
 'merge_old': lambda: sub("double mergeDistance() const { return strict_.accept; }", "double mergeDistance() const { return lenient_.accept; }"),
 'raster_old': lambda: sub("!ctx.moves.allows(region, *cursor, s)", "!ctx.moves.allowsOld(region, *cursor, s)"),
 'bridge_old': lambda: sub("!moves.allows(rings, c.a, c.b)", "!moves.allowsOld(rings, c.a, c.b)"),
 'pull_old': lambda: sub("moves.allows(region, clean[i], clean[j + 1])", "moves.allowsOld(region, clean[i], clean[j + 1])"),
 'linkdepth_old': lambda: sub("                           moves_.allows(region, pos_, s)) {", "                           moves_.allowsOld(region, pos_, s)) {"),
 'travel_old': lambda: sub("        if (moves_.allows(region, pos_, s)) {", "        if (moves_.allowsOld(region, pos_, s)) {"),
 'slack10': lambda: sub("constexpr double kMoveSlack = 4e-6;", "constexpr double kMoveSlack = 4e-5;"),
 'grid_nowiden': lambda: sub("const std::size_t i0 = col(std::min(xa, xb) - d), i1 = col(std::max(xa, xb) + d);", "const std::size_t i0 = col(std::min(xa, xb)), i1 = col(std::max(xa, xb));"),
 'grid_rowband': lambda: sub("                                     : box_.min.y + static_cast<double>(j) * cell_ - d;", "                                     : box_.min.y + static_cast<double>(j) * cell_;"),
}
MUT[name]()
open(sys.argv[3], 'w').write(src)
