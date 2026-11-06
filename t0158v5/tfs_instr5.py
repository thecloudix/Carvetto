#!/usr/bin/env python3
# tfs_instr5.py SRC/core/src/toolpath/profile.cpp: instrument the round-5 tieFreeStart (decisions untouched) by textual inserts.
# startOn logs one TFS2 line per call to stderr when TFS_LOG is set (st.cpp format); TFS_OFF=1 returns walk.project = main.
# K4 fields reused: kind = deciding path (0 projection, 1 arc-centre rule, 2 corner (own edges), 3 fallback round-join first
# point, 4 fallback equidistant junction), back = number of corner places with a nearest span (count; -1 not reached),
# mOnC = min |dist-d|/onCircle - 1 over on-circle tests (ratio margin), mEqui = min |maxratio - 1| of span labelling bands,
# mLop = min |along.edgeDir| of the travel-direction test, rj = met (-2 not reached).
import sys, re
p = sys.argv[1]; s = open(p).read()
def rep(old, new, count=1):
    global s
    assert s.count(old) >= count, (old, s.count(old))
    s = s.replace(old, new, count)
rep('#include <cstdio>\n', '#include <cstdio>\n#include <cstdlib>\n')
rep('double tieFreeStart(const ArcLengthPath& walk,', 'static thread_local int g_kind, g_count, g_met; static thread_local double g_mOnC, g_mBand, g_mSign;\ndouble tieFreeStart(const ArcLengthPath& walk,')
rep('    const Contour& c = walk.contour();\n    const std::size_t n = c.spans.size();\n    if (n == 0) {',
    '    g_kind = 0; g_count = -1; g_met = -2; g_mOnC = 1e300; g_mBand = 1e300; g_mSign = 1e300;\n    const Contour& c = walk.contour();\n    const std::size_t n = c.spans.size();\n    if (n == 0) {')
rep('                return walk.spanStart(i);\n', '                g_kind = 1; return walk.spanStart(i);\n')
# labelling band margins (line)
rep('''                        if (std::fabs(g - cutDistance) > noise) {
                            return false;
                        }''', '''                        { const double rr = std::fabs(g - cutDistance) / noise; if (rr < 50.0) g_mBand = std::min(g_mBand, std::fabs(rr - 1.0)); }
                        if (std::fabs(g - cutDistance) > noise) {
                            return false;
                        }''')
rep('''                        if (std::min(std::fabs(rho - (a->radius + cutDistance)),
                                     std::fabs(rho - std::fabs(a->radius - cutDistance))) > noise) {''', '''                        { const double rr = std::min(std::fabs(rho - (a->radius + cutDistance)), std::fabs(rho - std::fabs(a->radius - cutDistance))) / noise; if (rr < 50.0) g_mBand = std::min(g_mBand, std::fabs(rr - 1.0)); }
                        if (std::min(std::fabs(rho - (a->radius + cutDistance)),
                                     std::fabs(rho - std::fabs(a->radius - cutDistance))) > noise) {''')
rep('''                    const int m = along.dot(edgeDir) > 0.0 ? 1 : 2;''', '''                    g_mSign = std::min(g_mSign, std::fabs(along.dot(edgeDir)));
                    const int m = along.dot(edgeDir) > 0.0 ? 1 : 2;''')
rep('''            if (met > 0) {''', '''            g_met = met;
            if (met > 0) {''')
rep('''                        if (std::fabs(geometry::distance(geometry::startPoint(c.spans[k]), target) - cutDistance) <=
                            onCircle) {''', '''                        g_mOnC = std::min(g_mOnC, std::fabs(std::fabs(geometry::distance(geometry::startPoint(c.spans[k]), target) - cutDistance) / onCircle - 1.0));
                        if (std::fabs(geometry::distance(geometry::startPoint(c.spans[k]), target) - cutDistance) <=
                            onCircle) {''')
rep('''                if (count == 1) {
                    return startAt(walk.spanStart(pick));''', '''                g_count = static_cast<int>(count);
                if (count == 1) {
                    g_kind = 2; return startAt(walk.spanStart(pick));''')
rep('''                        return startAt(walk.spanStart(i));
                    }''', '''                        g_kind = 3; return startAt(walk.spanStart(i));
                    }''')
rep('''                return startAt(walk.spanStart(spanOf(pick + 1)));''', '''                g_kind = 4; return startAt(walk.spanStart(spanOf(pick + 1)));''')
rep('''double ProfileBuilder::startOn(const Cut& c, const ArcLengthPath& walk, Vec2 target) const {
    return tieFreeStart(walk, target, tol_.distance, c.offsetBy, c.offsetSlack, !sharpExternal_,
                        [&] { return tipEdges(c, target); });''', '''double ProfileBuilder::startOn(const Cut& c, const ArcLengthPath& walk, Vec2 target) const {
    const double uR = tieFreeStart(walk, target, tol_.distance, c.offsetBy, c.offsetSlack, !sharpExternal_,
                        [&] { return tipEdges(c, target); });
    const double uP = walk.project(target);
    const Vec2 pr = walk.pointAt(uR), pp = walk.pointAt(uP);
    const int rule = pr == pp ? 0 : 1;
    if (std::getenv("TFS_LOG")) {
        std::fprintf(stderr, "TFS2 rule %d n %zu closed %d nearCount 0 runs 0 du %.17g dpt %.17g nearest 0 cutd %.17g tgt %.17g %.17g ruleXY %.17g %.17g projXY %.17g %.17g mArcC 1e300 mArcR 1e300 mArcS 1e300 mCut 1e300 mNear 1e300 mGap 1e300 zda 0 eps %.17g K4 kind %d back %d mOnC %.6g mEqui %.6g mLop %.6g rj %d\\n",
                     rule, walk.contour().spans.size(), walk.closed() ? 1 : 0, uR - uP, geometry::distance(pr, pp), c.offsetBy, target.x, target.y, pr.x, pr.y, pp.x, pp.y, tol_.distance, g_kind, g_count, g_mOnC, g_mBand, g_mSign, g_met);
    }
    return std::getenv("TFS_OFF") ? uP : uR;''')
open(p, 'w').write(s)
print('instrumented', p)
