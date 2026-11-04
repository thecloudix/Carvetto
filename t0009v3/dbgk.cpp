#include "v9lib.cpp"
#include "xchk.hpp"
int main(int, char** argv) {
    std::uint64_t seed = std::strtoull(argv[1], nullptr, 0); Vec2 q{std::atof(argv[2]), std::atof(argv[3])};
    C9 cc = make9(seed); Case& c = cc.c; Contour ck = polyOf(c.in, c.tol);
    auto out = offsetOpenContour(ck, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
    xk::Checker X(ck, out, c.d, c.side, c.join, c.tol, c.mitre);
    auto cs = X.corners(); int i = 0;
    for (auto& cn : cs) { Vec2 v = q - cn.V; Vec2 b = (cn.n1 + cn.n2); double bl = b.length(); if (bl > 0) b = b / bl;
        double rel = std::remainder(std::atan2(v.y, v.x) - std::atan2(cn.n1.y, cn.n1.x), xk::PI2);
        std::printf("corner %d V(%.6g,%.6g) excuse %d arcs %d sw %.4f | |v| %.4g rel %.4f v.n1-d %.4g v.n2-d %.4g v.b-d %.4g\n", i++, cn.V.x, cn.V.y, cn.excuse, cn.arcs, cn.sw, v.length(), rel, v.dot(cn.n1) - c.d, v.dot(cn.n2) - c.d, v.dot(b) - c.d); }
}
