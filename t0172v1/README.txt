T-0172 round-1 verifier (branch task/T-0172-stray-followups @ d1d19af vs origin/main 1c43377 (core = b637c16) and pre-T-0128 8cd3568;
worktree /home/chris/wt-T0172). VERDICT FAIL (step-7 ClusterGrid blow-up on nested arc boxes: spiral inputs, see below).
Builds (in the worktree): build-verify1 (Debug -Werror, ctest), build-verify1-rel (branch Release), build-verify1-{main,pre}src (git archive)
-> build-verify1-{main,pre}ns (Release, -Dcarvetto=cvmain), build-verify1-brns (branch namespaced: A/A timing control), build-verify1-drv
(drivers), build-verify1-mut (Debug test binaries per mutant), build-verify1-var<X> (Release lib variants), build-verify1-ulp (Debug tests
relinked with ulpwrap.c).
 mk.sh <src> <out>      driver vs branch Release lib (BLIB) + REF=main|pre|brns namespaced reference (main_wrap.cpp, pw.hpp).
 mkwb.sh                SRC=<x.cpp> OUT=<bin>: white-box driver that #includes the branch offset.cpp (anon-namespace access).
 wb.cpp                 `wb sx N seed` selfCrossingCuts vs the replaced pairwise test (carrierIntersections + inUnit), yes/no and greedy
                        cuts, adversarial near-tangent/collinear/T/grazing probes at 0..3.3e5 (WALK=1: random-walk strands);
                        `wb cg N seed` ClusterGrid query/queryUntil sets vs brute force on 9 adversarial box families (CGV=1 progress).
 direct.cpp             hand-built near-tangent pairs: pairwise says "meet", selfCrossingCuts misses (gap > box growth).
 cgone.cpp              ClusterGrid build stats (levels/cells/entries) vs BoxGrid for one box family (kinds 0-16).
 sxbf.cpp               selfCrossingCuts on a collinear back-and-forth strand of nested lengths (grid blow-up).
 cutfb.cpp              putBackStrays' cut-into-consecutive-parts fallback driven directly with self-crossing strays.
 split.cpp/bigloop.cpp  (t0128v2) long strays; split2.cpp: walls with sub-tol segments, branch vs main coverage and chain counts.
 perf2.cpp              t0128v2 perf2 (alternating order, min of reps) + cases rhatch/vhatch/rrect/nrect/rcomb/scrib/spiral/sslot;
                        CMP=1 prints bit identity vs the reference. perf.sh -> logs/perf/{main,pre,brns}.log.
 perf16.cpp/perf16.sh   t0016v1 zig-zag timings incl. varNOBS (branch without the step-2 boundary-split change) -> logs/perf16.log.
 cmp6.cpp + fuzz.sh     output identity vs origin/main (fresh seeds 0x9c1Fkkkk; K0/K1/K2 ranges) -> logs/fz, agg.py.
 sweeps.sh              chan + vdrv5 cusp sweeps -> logs/sw.   ulp.sh: perturbed [t0128]/[t0172]/[offset] -> logs/ulp.
 mkv3.sh + v3/          t0128v3 vdrv5/sx3.inc vs branch (WRAP=1 ulpwrap) -> logs/gap_*.txt (closed-loop junction gaps vs radius).
 mkmut.py               mutants of the branch offset.cpp (writes mutants/offset_<X>.cpp; mutants/*.diff kept); mkmutbin.sh (Debug
                        test binaries), runmut.sh -> logs/runmut.log; mkrelvar.sh (Release variants: CQ, NOBS, WCHK window probe,
                        TIMEPB putBackStrays timer; apply mutants/<X>.diff for the last three).
Results (2026-11-06, shared 16-thread machine, load 4-10):
 Debug -Werror build rc 0 (0 warnings); ctest 1053/1053 rc 0 (logs/ctest-dbg.log). changelog check --base origin/main ok; merge-tree clean.
 (a) split 514/2000: 1 chain 513/1999 spans; bigloop 600 closed: 1 chain (main 602); split 32000: 1 chain, 0.39 s. cutfb: parts consecutive,
   non-crossing, no tiny ends, exact junctions. selfCrossingCuts NOT exactly equivalent to the pairwise test: near-tangent pairs with gap
   between the box growth (4e-9 len + 1e-12 scale) and carrierIntersections' tangency tolerance (1e-9 abs circle-circle, 1.25e-10 r line-arc)
   are missed (direct.cpp; wb sx: 1 in 300 000 random probes, 0 in 200 000 walks). New only ever misses, never adds.
 (b) lhatch d 0.002 far 1e5 N 1000/2000/4000: branch 0.28/1.02/3.13 s, main 3.2/13.2/39.9, pre 0.32-0.38/0.95/3.0 (0.98-1.19x pre).
   lhatch far 50: 1.72-1.84x pre (0.26x main; same as plain hatch d 0.002 2.7x pre = T-0128's accepted per-stray cost).
   BLOCKING: ClusterGrid explodes on nested boxes (concentric arcs): perf2 sslot 400 0.5 0 0 1 0 0 (spiral slot exactly 2d wide, no far
   feature): 12.6 s vs main 0.47 s, 2.5 GB; sslot 200: 1.25 s vs 0.08. perf2 spiral 400 0.5 0 0 1 0 600: 5.9 s vs 0.041 s (143x, 2 GB);
   far 50: 10.6 s vs 0.049 (216x, 3.6 GB); spiral 700 far 1e3: bad_alloc at 6 GB; spiral 4000 0.1 far 1e5: 13 GB in 52 s (killed). Output
   identical to main (CMP); time is in putBackStrays (TIMEPB: 6.2 s for 1 stray + 409 chain spans). cgone: kind 16 (spiral semicircle
   boxes) 4000 -> 107 M entries; kind 3 (collinear nested) 80 -> 36 M, 120 -> > 8 GB; kind 14 clustered log-uniform sizes 20000: 3.6 s,
   54 M entries vs BoxGrid 0.06 s, 2.6 M. sxbf 400: selfCrossingCuts 9.9 s, 2.8 GB (pairwise ~ms).
   ClusterGrid query exactness (wb cg, capped sizes): 360 000 queries, 0 wrong; WCHK probe (shared-window sets vs brute force): 0 wrong on
   lhatch/fhatch/chatch/rhatch/vhatch/hatch/comb/longz/split and 6 cmp6 chunks.
 Output identity: cmp6 273 600 offsets (fam 0,2-5 x 7200 inputs, 21-23,25 x 2400): differ 0, CPU 0.989-1.007. chan (L 3.6 x 3 d, L 20)
   and cusp (10 sweeps) identical to main. CMP identical on rcomb/scrib/rhatch/spiral/sslot/lhatch/fhatch/chatch.
 Elsewhere (vs main, 11 reps, A/A brns 0.95-1.0): hatch 3000 Round 1.04, comb 1.03-1.08, rects 1.02-1.03, ring 1.00-1.02, hatch 4000 d 0.05
   1.11. Zig-zags (perf16): 0.96-1.04 Round (zz4000 1.12, A/A 1.06); varNOBS 1.13-1.23 -> step-2 change recovers it; BS mutant (reversed
   candidate order) survives all tests and cmp6 identity holds.
 (c) mutants (Debug, [t0172] [t0128] [offset]): killed M3 W2 S S4 P O O2 E2 EJ XF WR; survive SL (loop strand > 4 spans -> longest span), XI
   (no box growth), XC (no dropTinyEnds in cut runs), CQ (child grids visited only in the first column: wrong queries!), CG/WK/WA/BS
   (equivalent or perf-only). E2 not redundant (dropped-copies test: 20 uncovered samples).
 (e) ulpwrap K1-4: [t0128]~[perf] 200/200, [t0172]~[perf] 200/200, [offset]~[perf] 24/24 pass. Closed-loop gaps 9.05e-12 (0x7c2340188 K2 s3)
   and 9.64e-11 (0x7c434015c K4 s3) at arc-arc junctions next to radius 2.4e4 / 6.76e4: within 1e-12 x max(1,|x|,|y|,r) (logs/gap_*.txt).
