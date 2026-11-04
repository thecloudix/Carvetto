T-0009 round-2 verifier harness (branch task/T-0009-subtol-spans @ ebe23f9). Build: mk.sh br|mg <src> <out>
(br = branch Release lib + base main namespaced cvmain; mg = scratch merge of origin/main 057c873 + main namespaced).
 x2.cpp       driver: st (v9 injector, AC2), g2 (new round-2 generator: corner runs, all-tiny chains, B1 tiny
              large-sweep arcs, collapsing arcs, noisy sampled lines, closed polygons with clusters, tiny arcs+Beziers),
              allj (each input x 3 joins x 2 sides), cmp (AC3 merged vs main, fam 0/2/3/4/5 and 7 = near-tol v9),
              mainone/v9one/g2one/shrink.  Both checkers run on every offset: vchk (round 1) and xk.
 xchk.hpp     independent checker: exact segment distance (analytic), rigorous Round max distance (Lipschitz
              branch and bound + per-span interval bounds), coverage of span loci and convex-corner arcs with stretch
              length and thickness (thin < tol = documented narrow-loop exemption), Sharp/Square kite excuse only at
              corners that keep the join (owner decision (a)); experimental "excess" region check (informational).
 selftest_xk.cpp  dSS/dPt vs brute force.   raychk.cpp / arcloc.cpp  grid-free brute-force confirmation of misses.
 speed.cpp    B2 probes (2000/4000/8000 spans, worst join/side, CPU) and T-0006 AC1 inputs, branch vs main.
 repro_cov.cpp + case_*.inc  repros (see header).   logs/  run summaries and flagged lines.
