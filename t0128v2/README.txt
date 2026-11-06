T-0128 round-2 verifier harness (branch task/T-0128-narrow-loops @ 1ff53e5 vs origin/main 34db4d0; worktree /home/chris/wt-T0128).
Builds (inside the worktree): build-verify2 (Debug -Werror, ctest 1020/1020 rc 0), build-verify2-rel (Release), build-verify2-mainsrc
(git archive origin/main) -> build-verify2-mainns (Release, -Dcarvetto=cvmain), build-verify2-drv (drivers), build-verify2-mut (mutants).
 mk.sh <src> <out>  driver vs branch Release lib (BLIB=...) + namespaced main.
 perf2.cpp   AC4 CPU time with the call order ALTERNATING per rep (min of reps): perf128 cases + chatch/lhatch (compact hatch at pitch
             2d + far U channel: strays clustered in few grid cells), fhatch (+ background serpentine), long/longz (one channel N mm long;
             longz walls of unit segments).  perf2 <case> <N> <d> <w> <join> <reps> [side] [far]
 ftime.cpp   AC4 fuzz-family time: per offset min of 5 reps, alternating order, thread CPU clock (logs/ft).
 cmp6.cpp    AC3 exact checker (t0016v1 cmp6) with alternating call order; fuzz.sh -> logs/fz (fresh seeds 0x7a2Fxxxx); agg.py.
 rep.cpp     dump one cmp6 case + per-pair overlap list + vchk overlapIn with chains dropped / main reversed (checker order artefact).
 edge10.cpp  owner minimum at its edge: isolated thin loop (A), isolated 2d U channel (B), 2d slot off a pocket (C, touches at one end),
             2d channel between pockets (D), every join, at 0 and 1.2e4.
 split.cpp / bigloop.cpp  strays of > 512 spans: one chain per span (longz / walls of unit segments).
 wfz.cpp + build-verify2-mut/offset_W.cpp (W: round-1 strand distance check d + 5e-7 max(1,d) or corner region, as a probe printing WVIOL
             for every stray put back; W3 = the tests' allowance max(tol^2/8d, min(5e-7 max(1,d), 0.1 tol^2/d))). fam 90 = adversarial
             loop strands (thin rects, slanted-end U, arc channels, near-tangent huge-radius arc vs line, needle teeth; 0/1.2e4/1e5/3.3e5,
             random rotation, mitre 1-4). wfz.sh/wfz3.sh, wrun*.sh -> logs/w, logs/w3.
 chan.cpp, vdrv5.cpp (cusp), chk128.cpp, ring17.cpp: copies of t0128v1. sweeps.sh -> logs/chan_*, logs/cusp_*.
 build-verify2-mut: mut.sh (Debug test binaries), runmut.sh ([t0128] then [offset]), mutrel.sh (Release test binaries), mkmut.sh (Release libs).
   Mutants: A no loop strand, B no dropped-opposite hand-back, C no removeCuspLoops, E no exact-range clamp, E2 exactOver ignored in
   mergeOnCarriers, F no stand-off, G cover ignored, M minimum 1 tol, M2 isolated=false, M3 isolated=true, H linear scan over added strays
   (quadratic; timing), L Lipschitz skip doubled, P shorter loop strand, S 512 -> 4, O merge order ignored, W2 no widening.
Results (2026-11-05, shared machine, load 14-20):
 Debug -Werror build rc 0, 0 warnings; ctest 1020/1020 rc 0 (logs/ctest-dbg.log). changelog check ok; merge-tree vs origin/main clean.
 chan (both signs of w): L 3.6 at d 0.1317 / 1 / 3.175 (14 580 offsets each) and L 20 at 3.175 (7 380): lost w>=0 branch 0 (main 14298 /
   14082 / 10350 / 6795), worse 0, sx 0, dup 0, ov = main.  cusp (Round, 732 each): which 1 sx 0 at d 0.05/0.13/1.03/3.175/4.2 (main 0/0/21/15/9),
   gaps 0; which 2 @1.03 sx 1 (main 22), @3.175 0 (15); which 0/3 = main.
 cmp6 (logs/fz, alternating order): fam0/2/3/4/5 900 inputs each, fam21/22/23/25 450 each (43 200 offsets): worse 2 = fam5 0x7a250302 Sharp
   both sides, vchk overlapIn 1 vs 0 -- checker artefact: the overlap (ch0 sp0/sp1, 0.0153 mm) is identical in main, and vchk counts it for
   main too once main's chains are reversed (rep.cpp); the branch only adds a 0.139 mm stray in the 2d slot middle.  fam0 0x71280859 identical.
 ftime (min of 5, alternating): fam0 1.004, fam2 1.006, fam3 0.994, fam4 1.002, fam5 1.011, fam21-25 0.977-0.992.
 perf2: B1 inputs hatch 3000 Round 0.046 s (main 0.037; round 1 2.94 s), Square 2d+1e-7 1.06x, comb 1.25x / 1.05x, rects 1.01x, ring 1.02x;
   T-0016 card zz 0.89/0.99/1.00x, zz2 0.93/1.04/0.97x. hatch d 0.002 2.6x (linear), d 0.05 1.2x. Clustered strays + far U (lhatch):
   d 0.05 far 1e3..1e5 1.7-2.0x main (N 1000->4000 linear-ish); d 0.002 far 1e5 11.9x main, quadratic (main quadratic too, 0.32 s at 1000).
   long 10 m channel 0.4 ms. longz (walls of unit segments): > 512 spans -> one chain per span (1999 chains for 2000 mm; split.cpp, bigloop.cpp).
 Timing test: base ratio 3.6-4.2 (Debug 250/1000, Release 1000/4000); mutant H (linear scan) Debug 9.0-9.3, Release 10.5-11.5 -> fails both.
 W (lenient) 47 291 + W3 (strict, tests' allowance) 35 744 loop strands put back (fam 90 + fam 2-5): 0 samples outside d + allowance and
   outside the corner regions -> removal of the strand distance check (mutant D) accepted.
 Mutants vs tests: caught A B C E F G M M2 L (and H by the timing test); survive [t0128] + [offset]: E2, M3 (isolated always), W2 (no widening),
   P (shorter strand), S (512 -> 4: per-span split), O (merge order).
 edge10: isolated strand / 2d U stray returned from len > 10 tol (e = 0.00999 dropped, 0.01 kept, Round/Sharp/Square, 0 and 1.2e4); a 2d slot off
   a pocket: Round stub 1.5e-3 kept (touches the Round chain), Sharp/Square stubs < 10 tol dropped (mitre point 1 mm away: isolated).
