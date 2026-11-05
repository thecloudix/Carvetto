T-0016 round-1 verifier harness (branch task/T-0016-sharp-cost @ 698f86f, worktree /home/chris/wt-T0016).
Builds (inside the worktree): build-verify1 (Debug -Werror), build-verify1-rel (Release lib), build-verify1-v3 (Release
-march=x86-64-v3), build-verify1-mainns (origin/main 6e537a1 namespaced cvmain), build-verify1-mainplain (origin/main, plain:
A/A timing control), build-verify1-wb (white-box copy of the branch, see below), build-verify1-m142 (merge with T-0142, core only).
 build-all.sh   the three first builds.   mk.sh <src> <out>  driver vs branch lib + main;  mk2.sh: same with BLIB=<lib build dir>.
 perf.cpp       CPU time branch vs main interleaved (min of reps): zz / zc (closed) / saw / hatch / arc (filleted) / bez zig-zags,
                any pitch, amplitude, d, side, mitre; MAINFIRST / ALT swap call order.  perf.sh: the AC1 matrix -> logs/perf.log.
 cmp6.cpp       AC2: branch vs main, every join x side: bit identity, else xk (t0009v3/xchk.hpp) + vchk metrics for both and a
                two-way sampled exact distance between the outputs. fam 0 = T-0006 makeCase; 2..5 = later adversarial families;
                20/21 dense zig-zag/saw/hatch/irregular (21: jitter+rotation+translation); 22 partial jitter 1e-9..1e-5;
                23 S-hatch (opposite copies, gap perturbed by 0/ulps/1e-12/1e-9/1e-6); 24 irregular pitch + tilt; 25 taller/deeper
                teeth; 26 all vertices jittered 1e-12..1e-9 (tilted copies crossing within stretches). DUMPONE=1 dumps input/outputs.
                fuzz.sh -> logs/f*.log.
 wb.cpp         white-box oracle, linked against build-verify1-wb (source copy build-verify1-wbsrc with an env-guarded block
                in offsetOpenContour): T16WB=1 recomputes the full pruning of every candidate, merges both kept sets with
                mergeOnCarriers and checks two-way coverage of the merged pieces (1e-9 x scale, same direction) plus state
                consistency; T16FB counts pruneLineCopies' fallback (chosen copy pruned) and overtake paths. Toggles:
                T16NOPRUNE (skip pruneLineCopies), T16NOSNAP (no snapJunction shortcut), T16INLINE (main's inline pruning loop),
                T16WBDUMP (merged pieces). logs/wb*.log, wbc*.log (with counters).
 zc.cpp         closed zig-zag offset inwards (pre-existing behaviour check).
Repros: ./cmp6 23 1 0x1690008b (DUMPONE=1): Sharp/Square side 1 differ from main by 3.9e-4 mm (sub-tol), merged pieces have
 the same owners (logs/f23_8b_pieces.txt), T16NOPRUNE makes it identical. ./cmp6 25 1 0x16d0016b: 8.5e-7 mm, same class.
 Round slowdown: ./perfAA vs ./perf (zz/arc 1000 0.02 1 7 0 2 3 1): main plain == main namespaced; branch +13-15 %.
