#!/usr/bin/env python3
# tfs_instr4.py SRC OUT: instrument the round-4 tieFreeStart (999a90e) by textual inserts (decisions untouched):
# TFS_OFF=1 -> plain projection (= main); TFS_LOG -> one TFS2 line per call (st.cpp format) + " K4 kind k pick p runLen r back b
# mOnC x mEqui y mLop z mVtx w" where kind 1 = round-join first point on circle, 2 = equidistant-junction pick, 3 = lopsided only
# (no equidistant junction), 0 = none; mOnC = min over the tested spans' starts of | |d(start,tgt)-cutd| / onCircle - 1 |
# (incl. the back-up candidates and the span before the run); mEqui = min over run junctions of | |ga-gb| / (4(na+nb)) - 1 |;
# mLop = |lopsided of pick - next best candidate| (mm; tiny = tie in "most even feet"); mVtx = distance of the chosen point to
# the projection (mm). Rule-1 / gate margins as in round 3.
import sys
s = open(sys.argv[1]).read()
def rep(a, b, cnt=1):
    global s
    assert s.count(a) == cnt, (a, s.count(a)); s = s.replace(a, b)
rep('double tieFreeStart(const ArcLengthPath& walk, Vec2 target, double eps, double cutDistance, double slack,\n                    bool roundJoins) {',
    '''struct TfsLog { int rule = 0, kind = 0, pick = -1, runLen = -1, back = 0, nearCount = -1, runs = -1; double mOnC = 1e300, mEqui = 1e300, mLop = 1e300, mGap = 1e300, nearest = 0, mNear = 1e300; };
static TfsLog g_tl;
double tieFreeStartImpl(const ArcLengthPath& walk, Vec2 target, double eps, double cutDistance, double slack,
                    bool roundJoins);
double tieFreeStart(const ArcLengthPath& walk, Vec2 target, double eps, double cutDistance, double slack,
                    bool roundJoins) {
    static const bool logOn = std::getenv("TFS_LOG") != nullptr;
    static const bool off = std::getenv("TFS_OFF") != nullptr;
    g_tl = TfsLog{};
    const double u = tieFreeStartImpl(walk, target, eps, cutDistance, slack, roundJoins);
    const double up = walk.project(target);
    if (logOn) {
        const Vec2 pr = walk.pointAt(u), pp = walk.pointAt(up);
        double du = std::fabs(u - up); du = std::min(du, walk.length() - du);
        const double mCut = std::min(std::fabs(g_tl.nearest - (cutDistance + eps)), std::fabs(g_tl.nearest - (cutDistance - slack - eps)));
        std::fprintf(stderr, "TFS2 rule %d n %zu closed %d nearCount %d runs %d du %.17g dpt %.17g nearest %.17g cutd %.17g tgt %.17g %.17g ruleXY %.17g %.17g projXY %.17g %.17g mArcC 1e300 mArcR 1e300 mArcS 1e300 mCut %.6g mNear %.6g mGap %.6g zda 0 eps %.17g mTurn 1e300 pick %d runLen %d slack %.6g jmin -1 jPickMinusMin 0 K4 kind %d back %d mOnC %.6g mEqui %.6g mLop %.6g rj %d\\n",
                     g_tl.rule, walk.contour().spans.size(), (int)walk.closed(), g_tl.nearCount, g_tl.runs, du, geometry::distance(pr, pp), g_tl.nearest, cutDistance, target.x, target.y, pr.x, pr.y, pp.x, pp.y,
                     mCut, g_tl.mNear, g_tl.mGap, eps, g_tl.pick, g_tl.runLen, slack, g_tl.kind, g_tl.back, g_tl.mOnC, g_tl.mEqui, g_tl.mLop, (int)roundJoins);
    }
    return off ? up : u;
}
double tieFreeStartImpl(const ArcLengthPath& walk, Vec2 target, double eps, double cutDistance, double slack,
                    bool roundJoins) {''')
rep('''        nearest = std::min(nearest, dist[i]);
    }
''', '''        nearest = std::min(nearest, dist[i]);
    }
    g_tl.nearest = nearest;
''')
rep('''                return walk.spanStart(i);
            }
        }
    }''', '''                g_tl.rule = 1;
                return walk.spanStart(i);
            }
        }
    }''')
rep('''        bool found = runs == 1 && longRun;''', '''        bool found = runs == 1 && longRun;
        g_tl.nearCount = (int)nearCount; g_tl.runs = (int)runs;
        for (std::size_t i = 0; i < n; ++i) g_tl.mNear = std::min(g_tl.mNear, std::fabs(dist[i] - (nearest + eps)));''')
rep('''            found = gap1 - gap2 > eps;''', '''            found = gap1 - gap2 > eps;
            g_tl.mGap = std::fabs(gap1 - gap2 - eps);''')
rep('''            if (nearCount == n) {
                runLength = n;
            }''', '''            if (nearCount == n) {
                runLength = n;
            }
            g_tl.rule = nearCount == n ? 3 : 2; g_tl.runLen = (int)runLength;''')
rep('''                for (std::size_t k = 0; k <= junctions; ++k) {
                    std::size_t i = spanOf(k);''', '''                auto onM = [&](std::size_t i) { const double dv = std::fabs(geometry::distance(geometry::startPoint(c.spans[i]), target) - cutDistance);
                    g_tl.mOnC = std::min(g_tl.mOnC, std::fabs(dv / onCircle - 1.0)); };
                onM((first + n - 1) % n);
                for (std::size_t k = 0; k <= junctions; ++k) onM(spanOf(k));
                for (std::size_t k = 0; k <= junctions; ++k) {
                    std::size_t i = spanOf(k);''')
rep('''                            i = (i + n - 1) % n;
                        }''', '''                            i = (i + n - 1) % n;
                            g_tl.back = (int)back + 1; onM((i + n - 1) % n);
                        }
                        g_tl.kind = 1;''')
rep('''                equi[k] = std::fabs(ga - gb) <= 4.0 * (na + nb) ? 1 : 0;''', '''                equi[k] = std::fabs(ga - gb) <= 4.0 * (na + nb) ? 1 : 0;
                g_tl.mEqui = std::min(g_tl.mEqui, std::fabs(std::fabs(ga - gb) / (4.0 * (na + nb)) - 1.0));''')
rep('''            if (pick < junctions) {
                while''', '''            if (pick < junctions) {
                for (std::size_t k = 0; k < junctions; ++k) if (k != pick && (equi[k] != 0 || !anyEqui)) g_tl.mLop = std::min(g_tl.mLop, std::fabs(lopsided[k] - lopsided[pick]));
                g_tl.kind = anyEqui ? 2 : 3; g_tl.pick = (int)pick;
                while''')
for inc in ('#include <cstdlib>',):
    if inc not in s: s = s.replace('#include <cmath>\n', '#include <cmath>\n' + inc + '\n', 1)
open(sys.argv[2], 'w').write(s)
