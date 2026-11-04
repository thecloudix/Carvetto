T-0009 round-3 verifier harness (branch task/T-0009-subtol-spans @ 8b4e69e, which already contains origin/main 0fa0699).
Build: mk.sh br <src.cpp> <out>  (branch Release lib build-verify3-rel + origin/main namespaced cvmain build-verify3-mainns).
 xchk.hpp   round-2 exact checker + round-3 owner decision (c): runs (consecutive spans <= tol round a > 135 deg turn
            at a span <= tol, box diagonal <= 2 tol); their own loci are not required, but (1) the collapsed corner
            (end of A, tangents of A and B) is required with tolerance-sized slack (valid >= d - 2 tol, covered within
            3 tol) and (2) every valid uncovered point of the exempt loci that lies in the requested-side wedge at the
            collapsed vertex and is > 3 tol from the output is a "(c)-SIDE" miss. Kites excused at every corner except
            between two sub-tol spans (a), sub-tol near-reversals and corners of runs that reverse as a whole (c),
            plus each collapsed corner. Exempt otherwise only: region beyond d thinner than tol.
 x2.cpp     driver: st (v9 injector), g2, allj, cmp (AC3 vs main), re (recheck a list vs main: is the branch's worst
            miss point covered by main?), cl ("closed loops stay closed (as on main)"), libonly (crash isolation).
 parse.py / classify.py   log summaries (AC2 classes; branch-only vs main-also).  fuzz.sh  the AC2 run.
 ptq.cpp ptchk.cpp tipcap.cpp gap.cpp mcover.cpp dbgrun.cpp   brute-force point checks, chain gaps, main-only true
            offset (independent of loci), run dump.   repro/  repro.sh + crash_repro.cpp (std::length_error, g2 0x32000123).
 logs/      v9_*/g2_*/allj_* fuzz, re_*.out rechecks, cl_* closed-loop check, ac3/cmp_*, speed_*.
