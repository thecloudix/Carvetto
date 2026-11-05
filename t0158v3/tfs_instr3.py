#!/usr/bin/env python3
# tfs_instr3.py SRC_PROFILE_CPP OUT_PROFILE_CPP: replace tieFreeStart (5649b2b, round 3: slack, sharpest-junction pick with
# kTipTurnTie) by an instrumented copy. Same decisions as the branch; TFS_OFF=1 -> plain projection (= main);
# TFS_LOG -> one TFS2 line per call (round-2 format, st.cpp-compatible) + trailing "mTurn <m> pick <k> runLen <n> turns <t...>".
# mCut = min distance of nearest from either gate bound (cutd+eps, cutd-slack-eps); mTurn = min |turn - (sharpest - kTipTurnTie)|
# over the run's junctions (knife-edge of the tie threshold); jmin = run junction nearest the target, jPickMinusMin = how much
# farther the picked junction is than that one (pick != jmin: the start is not the junction nearest the target).
import sys
src = open(sys.argv[1]).read()
a = src.index('double tieFreeStart(const ArcLengthPath& walk')
b = src.index('// A cross-section of a tab', a)
new = r'''double tieFreeStart(const ArcLengthPath& walk, Vec2 target, double eps, double cutDistance, double slack) {
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
    double mArcC = 1e300, mArcR = 1e300, mArcS = 1e300, mCut = 1e300, mNear = 1e300, mGap = 1e300, mTurn = 1e300;
    int nearCountL = -1, runsL = -1, zeroDirArc = 0, pickL = -1, runLenL = -1, jminL = -1; double jPickMinusMin = 0;
    std::string turnStr;
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
    mCut = std::min(std::fabs(nearest - (cutDistance + eps)), std::fabs(nearest - (cutDistance - slack - eps)));
    if (rule == 0 && walk.closed() && n >= 2 && nearest <= cutDistance + eps && nearest >= cutDistance - slack - eps) {
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
            std::size_t runLength = 0;
            while (runLength < n && isNear((first + runLength) % n)) ++runLength;
            if (nearCount == n) runLength = n;
            std::vector<double> turns; double sharpest = 0.0;
            for (std::size_t k = 0; k + 1 < runLength; ++k) {
                const Vec2 ta = geometry::tangentAt(c.spans[(first + k) % n], 1.0);
                const Vec2 tb = geometry::tangentAt(c.spans[(first + k + 1) % n], 0.0);
                turns.push_back(std::fabs(std::atan2(ta.cross(tb), ta.dot(tb))));
                sharpest = std::max(sharpest, turns.back());
            }
            std::size_t pick = 0;
            while (pick + 1 < turns.size() && turns[pick] < sharpest - kTipTurnTie) ++pick;
            for (double t : turns) { if (t != sharpest) mTurn = std::min(mTurn, std::fabs(t - (sharpest - kTipTurnTie)));
                char tb2[32]; std::snprintf(tb2, sizeof tb2, " %.6g", t); if (turnStr.size() < 400) turnStr += tb2; }
            pickL = (int)pick; runLenL = (int)runLength;
            { double best = 1e300; for (std::size_t k = 0; k + 1 < runLength; ++k) { const double dj = geometry::distance(walk.pointAt(walk.spanStart((first + k + 1) % n)), target);
                if (dj < best - 1e-12) { best = dj; jminL = (int)k; } }
              jPickMinusMin = geometry::distance(walk.pointAt(walk.spanStart((first + pick + 1) % n)), target) - best; }
            const double junction = walk.spanStart((first + pick + 1) % n);
            u = walk.pointAt(up) == walk.pointAt(junction) ? up : junction;
            rule = r;
        }
    }
    if (logOn) {
        const Vec2 pr = walk.pointAt(u), pp = walk.pointAt(up);
        double du = std::fabs(u - up); du = std::min(du, walk.length() - du);
        std::fprintf(stderr, "TFS2 rule %d n %zu closed %d nearCount %d runs %d du %.17g dpt %.17g nearest %.17g cutd %.17g tgt %.17g %.17g ruleXY %.17g %.17g projXY %.17g %.17g mArcC %.6g mArcR %.6g mArcS %.6g mCut %.6g mNear %.6g mGap %.6g zda %d eps %.17g mTurn %.6g pick %d runLen %d slack %.6g jmin %d jPickMinusMin %.6g turns%s\n",
                     rule, n, (int)walk.closed(), nearCountL, runsL, du, geometry::distance(pr, pp), nearest, cutDistance, target.x, target.y, pr.x, pr.y, pp.x, pp.y,
                     mArcC, mArcR, mArcS, mCut, mNear, mGap, zeroDirArc, eps, mTurn, pickL, runLenL, slack, jminL, jPickMinusMin, turnStr.c_str());
    }
    return off ? up : u;
}

'''
out = src[:a] + new + src[b:]
for inc in ('#include <cstdlib>', '#include <string>', '#include <cstdio>'):
    if inc not in out: out = out.replace('#include <cmath>\n', '#include <cmath>\n' + inc + '\n', 1)
open(sys.argv[2], 'w').write(out)
