T-0128 round-1 verifier harness (branch task/T-0128-narrow-loops @ ddb4a8e vs origin/main 681327a; worktree /home/chris/wt-T0128).
Builds (inside the worktree): build-verify1 (Debug -Werror, ctest), build-verify1-rel (Release core), build-verify1-mainsrc
(git archive origin/main) -> build-verify1-mainns (Release, -Dcarvetto=cvmain), build-verify1-drv (drivers), build-verify1-mut
(mutants of offset.cpp linked into the Debug test binary by mut.sh; Release mutant libs X-rel for cmp6X / perf128H).
 builds.sh      Release branch lib + namespaced main.     mk.sh <src> <out>: driver vs branch lib (BLIB=...) + main.
 chan.cpp       AC1 channel sweep: U / reversed U / arc-ended U / closed rectangle, 2d + w wide, L 3.6, w in {0, 1e-16..1e-5},
                d 0.1317/1/3.175, every join, 3 shifts (0, 1.2e4, 1e5) x 5 rotations; centre-line coverage (<= tol), xk + vchk,
                duplicated length (dupLen), branch vs main.  chan [L] [verbose] [wmode 0 w>=0 | 1 both signs | 2 w<0] [d only]
 cmp6.cpp       AC3 (copy of t0016v1/cmp6.cpp): branch vs main, xk + vchk, fam 0/2/3/4/5/21/22/23/25.  fuzz.sh -> logs/f*.log.
 vdrv5.cpp+inc  copy of t0017v3/h (cusp.inc: cusp which d nG gmax log neg), linked to current main.
 perf128.cpp    AC4 CPU time interleaved (min of reps): zz (T-0016 card), zz2, comb (N slots 2d+w), hatch (boustrophedon at pitch
                2d+w), rects (N closed rectangles 3.6 x 2d+w in one open path -> thin loops), ring (serpentine half-ring channels).
 chk128.cpp     quality of the perf128 inputs (xk, vchk, dupLen) branch vs main.   ring17.cpp: the 3 T-0017 chain-split cases,
                medial-ring chains on branch and main.
 mut.sh         mutants A (no loop strand) B (no dropped-opposite hand-back) C (no removeCuspLoops) D (no strand distance check)
                E (no exact-range clamp) F (no cusp stand-off) G (strays emitted even where covered); H (coverGap without the linear
                scan over strays put back; Release, timing only).
Key repro (AC4, quadratic): ./perf128 hatch 3000 1 0 0 1 0  -> branch 2.94 s, main 0.04 s; perf128H (H mutant) 0.066 s.
Results (2026-11-05, shared machine, load ~30):
 Debug -Werror build rc 0; ctest 998/998 (rc 0). Merge with origin/task/T-0142-boolean-cost (098a7db) clean; its carvetto_tests
 [offset],[geometry],[t0128],[t0017] pass. changelog check ok. merge-tree vs origin/main clean.
 chan (logs/chan_*.log): 22 140 offsets L 3.6 + 7 380 L 20 (d 3.175): lost 0 (main 18 960 / 6 795), worse 0, sx 0, dup 0.
 cusp (logs/cusp_*.log): which 1 sx 0 at every d (main 21 @1.03, 15 @3.175, 9 @4.2), gaps 0; which 2 @1.03 sx 1 (main 22); which 0/3 = main.
 cmp6 (logs/fz, logs/f0a.log; agg.py): fam0 3600 + 18 000, fam2/3/4/5 9000 each, fam21/22/23/25 3600 each: worse 0.
   CPU branch/main summed: fam2 1.125, fam3 1.122, fam4 1.055, fam5 1.154, fam0 1.000, zig-zag families 0.975-0.986.
 perf128: T-0016 card zz 1000 0.02 1 d 7: Round 0.94x, Sharp 1.01x, Square 1.01x; zz2 4000: 0.95/1.01/1.03x.
   QUADRATIC: hatch (pitch exactly 2d) N 100/300/1000/3000 Round: 5.2x/10.7x/27x/78x main (2.94 s vs 0.037 s at 3000);
   Square w 1e-7 N 3000: 7.62 s vs 0.071 s; comb 3000 8.97 s vs 0.085 s; rects 3000 25.3 s vs 0.27 s; ring 3000 16.2 s vs 0.15 s.
   Cause: coverGap scans every stray already put back linearly (H mutant: hatch 3000 0.066 s).
 mutants vs tests: A, B, C caught by the T-0128 tests; D, E, F, G pass every [offset] test (F reproducible as WORSE by
   ./cmp6F 5 1 0x28500192, Round side 1).
 edge.cpp: thin rectangle loops (2d+e) x (2d+w), w 1e-6/1e-4: extent e in (tol, 2 tol] still dropped; kept from e >= 2.01e-3.
