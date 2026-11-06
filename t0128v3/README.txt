T-0128 round-3 verifier (PR #61 MSVC fix cae3fa7 vs e87f549 / origin/main e9bf85e; worktree /home/chris/wt-T0128).
Builds (in the worktree): build-verify3 (Debug -Werror, ctest), build-verify3-rel (branch Release lib), build-verify3-{main,old}src (git archive
origin/main / e87f549) -> build-verify3-{main,old}ns (Release, -Dcarvetto=cvmain), build-verify3-brns (cae3fa7 namespaced), build-verify3-drv
(drivers), build-verify3-mut (F-rel.a mutant F; I-rel.a = cae3fa7 + EJNEW/EJKEEP/EJGAP stderr probes in exactJunction(s)),
build-verify3-ulp (Debug test binaries relinked with the libm wrapper; variants old/oldlib/oldtest/F/Fold, round-2 mutants m*).
 mk3.sh <src> <out>    driver vs BLIB (default branch Release) + reference cvmain MREF=main|old|br; LIBS="$(cat wrapflags.txt)" adds the wrapper.
 ulpwrap.c             -Wl,--wrap for sin cos atan2 hypot acos asin tan (+ sincos, atan): result moved by deterministic -K..K ulps per
                       argument (ULP_K, ULP_SEED; ULP_COUNT=1 prints call counts). Stand-in for MSVC's libm.
 sx3.inc (in vdrv5.cpp) sxCount = the tests' countSelfCrossings (cae3fa7) + the count without the noise filter (sxOld) + largest filtered
                       ratio dist*sine/unit (maxNoise) + smallest real ratio; reqChains = requireToolpathChains rules. Modes: sxcusp
                       (vdrv5 cusp which 0..3 + the test's g grid, 6 + 6 random shifts), sxone, sxchan (chan.cpp sweep, both signs of w),
                       sxfz (fuzz fam 2-5, branch vs ref with the tests' rules), sxseed (one seed, crossings listed), sxarc (arc-channel needles).
 mkulp.sh / mkmut3.sh  Debug variant binaries (object swap + relink); mkmut3 runs [t0128] then [offset] per mutant (logs/mut).
 ulpsweep.sh           perturbed [t0128] (4 variants x K1-4 x 50 seeds), [offset] (br/old x K1-4 x 6), sx cusp/chan (logs/ulp).
 ulpfz.sh              perturbed fuzz: I vs e87f549, branch vs main; F vs branch on glibc (logs/ulpfz).  cmpfz.sh: cmp6 vs old / main (logs/fz).
 arcloop.sh + probe/arcloop.cpp  closed all-arc loops (T-0017 generator at 6 places up to 3.3e5 mm), I vs e87f549, glibc + perturbed.
 probe/ovl.cpp (noise filter vs near-coincident adjacent arcs), probe/c5.cpp (0xc500076c lens ratio in main's output).
Results (2026-11-05):
 Debug -Werror build rc 0, 0 warnings; ctest 1021/1021 rc 0 (logs/ctest-dbg.log). changelog check --base origin/main ok; merge-tree vs origin/main e9bf85e clean.
 Perturbed [t0128]~[perf] (K 1-4 x 50 seeds): cae3fa7 200/200 pass; e87f549 28/200 (the PR #61 MSVC signature: Round cusps g = 0 shift 0 side 0,
   sx 1 == 0; + regression-seed noise crossings); oldlib (e87f549 exactJunction + new counter) 162/200 (cusp test, junction gap 1e-11..1e-10 at
   g = 1e-11..1e-10, e.g. 3.16e-11); oldtest (new lib + old counter) 29/200.
 Perturbed [offset]~[perf] (K 1-4 x 6): failing test sets of branch and origin/main identical in all 24 runs (T-0017 3-arc repro input REQUIRE;
   T-0006 AC3 absolute 1e-12 gap, trial 79 / 27, same maxGap on main) -> pre-existing; e87f549 fails 23/24.
 Perturbed sx sweeps: cusp (which 0-3 x 3 d x 12 shifts, 4656 offsets each) 890 sweeps vs main / e87f549: branch bad 0, gaps 0, max filtered
   ratio 23.2 of 100; main 5079 real crossings (min ratio 1330), 269 gaps; e87f549 249 gaps. chan 120 sweeps: branch bad 0 / lost 0.
 Fuzz perturbed (fam 2-5, 38 400 offsets each): I vs e87f549 2 differ (both e87f549 bad, branch clean), EJNEW 2, no gap from the new path;
   branch vs main worse 0 (2 branch-bad = main-identical closed all-arc loop gaps 9e-12 / 9.6e-11: fam4 0x7c2340188 K2 seed 3, 0x7c434015c K4 seed 3).
 cmp6 alternating: vs e87f549 162 000 offsets, 0 differ; vs origin/main 37 800 offsets, 2205 differ, worse 0.
 exactJunction new path (I probe): glibc cusp sweeps 0 triggers; perturbed 145 triggers, residual 1e-12..3.2e-10 closed to ~1e-32, no EJKEEP, no EJGAP.
 All-arc closed loops (arcloop, 81 000 offsets per run, 13 runs per lib): no gap, no back-step under perturbation; glibc 54 back-steps at (0,0)
   identical in branch, e87f549 and main (pre-existing).
 Mutants (cae3fa7 tests): A B C E G L M M2 caught by [t0128]; F survives [t0128] and [offset]. F vs branch fuzz (72 000 offsets): 36 differ,
   F never worse (removes noise crossings); F on 0x28500192 side 1 = closing junction moved 2.2e-8 mm along two externally touching circles,
   crossing ratio 0.73 units. Arc-channel needles (sxarc, 22 950 offsets): F identical to branch in coverage.
 Noise bound: 0xc500076c lens in main's output = 6275 units (ADR says ~1e8); near-coincident adjacent arcs: intersect() (tol 1e-9) reports
   nothing until the circles are 1e-8 apart (ratio >= 8.5e3), so the filter hides nothing the old count caught there (probe/ovl.cpp).
