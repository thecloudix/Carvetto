#!/usr/bin/env python3
# tfs_instr.py SRC_PROFILE_CPP OUT_PROFILE_CPP: replace tieFreeStart (5e7ce1d) by an instrumented copy.
# Same decisions as the branch; TFS_OFF=1 -> plain projection (= main); TFS_LOG -> one stderr line per call with
# which rule fired and the margins of every threshold the rule compares (knife-edge detection).
import sys, re
src = open(sys.argv[1]).read()
a = src.index('double tieFreeStart(const ArcLengthPath& walk')
b = src.index('// A cross-section of a tab', a)
new = r'''double tieFreeStart(const ArcLengthPath& walk, Vec2 target, double eps, double cutDistance) {
    static const bool logOn = std::getenv("TFS_LOG") != nullptr;
    static const bool off = std::getenv("TFS_OFF") != nullptr;
    const Contour& c = walk.contour();
    const std::size_t n = c.spans.size();
    if (n == 0) {
        return 0.0;
    }
    std::vector<double> dist(n);
    std::vector<double> param(n);
    double nearest = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < n; ++i) {
        const geometry::ClosestPoint cp = geometry::closestPoint(c.spans[i], target);
        dist[i] = cp.distance;
        param[i] = cp.t;
        nearest = std::min(nearest, dist[i]);
    }
    const double up = walk.project(target);
    int rule = 0; double u = up;
    // margins (distance of each compared quantity from its threshold; small = knife-edge)
    double mArcC = 1e300, mArcR = 1e300, mArcS = 1e300, mCut = 1e300, mNear = 1e300, mGap = 1e300;
    int nearCountL = -1, runsL = -1, zeroDirArc = 0;
    for (std::size_t i = 0; i < n; ++i) {
        if (const auto* a = std::get_if<geometry::ArcSpan>(&c.spans[i])) {
            const double dc = geometry::distance(a->centre, target);
            const double rr = a->radius - (nearest + eps);
            const double ss = geometry::distance(a->startPoint(), target) - (nearest + eps);
            if (rr <= 0 && ss <= 0) mArcC = std::min(mArcC, std::fabs(dc - eps));
            if (dc <= eps && ss <= 0) mArcR = std::min(mArcR, std::fabs(rr));
            if (dc <= eps && rr <= 0) mArcS = std::min(mArcS, std::fabs(ss));
            if (rule == 0 && dc <= eps && rr <= 0 && ss <= 0) { rule = 1; u = walk.spanStart(i); }
        }
    }
    mCut = std::fabs(std::fabs(nearest - cutDistance) - eps);
    if (rule == 0 && walk.closed() && n >= 2 && std::fabs(nearest - cutDistance) <= eps) {
        auto isNear = [&](std::size_t i) { return dist[i] <= nearest + eps; };
        for (std::size_t i = 0; i < n; ++i) {
            mNear = std::min(mNear, std::fabs(dist[i] - (nearest + eps)));
            if (isNear(i)) if (const auto* a = std::get_if<geometry::ArcSpan>(&c.spans[i])) if (geometry::distance(a->centre, target) < 1e-6 * std::max(1.0, a->radius)) zeroDirArc = 1;
        }
        std::size_t nearCount = 0, runs = 0, first = n; bool longRun = false;
        for (std::size_t i = 0; i < n; ++i) {
            if (isNear(i)) ++nearCount;
            if (isNear(i) && !isNear((i + n - 1) % n)) { ++runs; first = i; longRun = isNear((i + 1) % n); }
        }
        nearCountL = (int)nearCount; runsL = (int)runs;
        bool found = runs == 1 && longRun;
        int r = 2;
        if (nearCount == n) {
            r = 3;
            const double len = walk.length();
            auto foot = [&](std::size_t i) { return walk.spanStart(i) + param[i] * walk.spanLength(i); };
            std::size_t widest = 0; double gap1 = -1.0, gap2 = -1.0;
            for (std::size_t i = 0; i < n; ++i) {
                const double g = i + 1 < n ? foot(i + 1) - foot(i) : foot(0) + len - foot(i);
                if (g > gap1) { gap2 = gap1; gap1 = g; widest = i; } else if (g > gap2) { gap2 = g; }
            }
            first = (widest + 1) % n;
            found = gap1 - gap2 > eps;
            mGap = std::fabs(gap1 - gap2 - eps);
        }
        if (found) {
            const double junction = walk.spanStart((first + 1) % n);
            u = walk.pointAt(up) == walk.pointAt(junction) ? up : junction;
            rule = r;
        }
    }
    if (logOn) {
        const Vec2 pr = walk.pointAt(u), pp = walk.pointAt(up);
        double du = std::fabs(u - up); du = std::min(du, walk.length() - du);
        std::fprintf(stderr, "TFS2 rule %d n %zu closed %d nearCount %d runs %d du %.17g dpt %.17g nearest %.17g cutd %.17g tgt %.17g %.17g ruleXY %.17g %.17g projXY %.17g %.17g mArcC %.6g mArcR %.6g mArcS %.6g mCut %.6g mNear %.6g mGap %.6g zda %d eps %.17g\n",
                     rule, n, (int)walk.closed(), nearCountL, runsL, du, geometry::distance(pr, pp), nearest, cutDistance, target.x, target.y, pr.x, pr.y, pp.x, pp.y,
                     mArcC, mArcR, mArcS, mCut, mNear, mGap, zeroDirArc, eps);
    }
    return off ? up : u;
}

'''
out = src[:a] + new + src[b:]
if '#include <cstdlib>' not in out: out = out.replace('#include <cstdio>\n', '#include <cstdio>\n#include <cstdlib>\n', 1)
open(sys.argv[2], 'w').write(out)
