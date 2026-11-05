T-0009 round-4 verifier harness (branch task/T-0009-subtol-r4 @ eab77ce vs origin/main e2e5345), from the round-3 harness t0009v3.
Build: mk.sh br <src.cpp> <out>  (branch Release lib build-verify4-rel + origin/main namespaced cvmain build-verify4-mainns,
       source build-verify4-mainsrc, all in /home/chris/wt-T0009r4; drivers in build-verify4-drv).
Unchanged from round 3: the checker's reference behaviour (xchk.hpp with no env vars), generators v9/g2, fuzz.sh seeds.
Round-4 additions:
 xchk.hpp  Checker::ref (main's checker): every non-thin miss stretch is measured against main's output: REG (strict:
           main within tol, stretch > tol), REG3 (main within 3 tol); (c)-side misses main covers split into thin
           (strip < tol) and not. Env variants (off by default = round-3 reference): XK_NETTURN=1 net-turn runs (box <= 2 tol,
           |turn| > 180 deg) are runs; XK_ARCSEED=1 sub-tol arcs sweeping > 135 deg seed runs; XK_ANCHORB=1 a collapsed
           corner's Sharp/Square region is also anchored along B's offset line (kite at B's start + region between the
           two offset lines).
 x2.cpp    re prints an RV4 line (REG, REG3, side, excess branch vs main, closed chains, dv/rv); adv/advone (adv.inc:
           adversarial cluster generator: 2/4 tol thresholds, net-turn runs, chain ends, neighbouring clusters, arc and
           Bezier clusters, closed with a cluster at the start, d from 1.5 tol to 100); thick (strip width at a point).
 re4sum.py RV4 summaries.  lprobe.cpp (locus sampling), loopchk.cpp (main loops vs branch, interior vs corner regions),
           subnear.cpp (point vs sub-tol spans), opengap.py (near-closed open chains), tools/near.py (spans near a point),
           fuzz_fresh.sh (fresh seeds 0x41..0x44), repro/repro4.sh.
 logs/     v9_/g2_/allj_ (round-3 seeds), fresh/, re_all.*.out, cl_*.log, adv/, ac3/, exw_*, reg_*, speed_*.
