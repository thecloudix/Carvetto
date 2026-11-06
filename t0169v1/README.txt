T-0169 verifier round 1 (task/T-0169-centre-dip 274b43f; base = merge base e9bf85e; origin/main 8cd3568).
Libraries (not committed): Release /home/chris/wt-T0169/build-verify1-rel (branch) and build-verify1-mainsrc/build-verify1-mrel
(git archive of e9bf85e); Debug -Werror /home/chris/wt-T0169/build-verify1; mutant tree build-verify1-mutsrc (Release).
Binaries in /home/chris/wt-T0169/build-verify1-bin. Build a driver: ./mk.sh <src.cpp> <out> <br|main> [flags].
  oc2.hpp      own exact clearance oracle: lines closed form; arcs (pieces <= 90 deg in their tangent triangle) and Beziers
               (control hull) by certified branch and bound; relative resolution at far coordinates. octest.cpp checks it
               against dense sampling (never above the sampled minimum by more than 3e-10).
  pk.cpp       Pocket adversarial families 0-9 (0 junction generator re-implemented, 1 letters with filleted reflex corners,
               2 bitten squares, 3 dumbbells/thin necks, 4 islands at gaps ~ tool D, 5 wavy Beziers, 6 the T-0153 "A",
               7 gears, 8 saw teeth with arc teeth, 9 line-only) + env PK_FAR/PK_SCALE/PK_DFR/PK_CH/PK_STRAT/PK_REST/PK_ALLOW;
               prints hash, retract/plunge counts, cut length, CPU and the worst clearance - allowance by oc2 (-DPK_XO also
               runs the branch's pocket_curve_oracle.hpp for cross-checking; PK_DUMP dumps geometry and moves).
  sweep.sh     branch vs main over families x configs; cmp.py (changed / violations / deltas), cmp2.py (changed cases
               without a main violation).
  wb.cpp       white box (pocket.cpp included): free segmentInside vs RegionIndex::segmentInside (both verdicts) vs exact
               max distance outside the region, on centre-region-like rings (fine round joins at reflex vertices, holes,
               1e-6 grid) with segment ends on/near edges; EdgeGrid::anyCloser vs brute force (wb2 = exact brute force;
               wb's brute force used the pre-existing distSegments, whose false 0 on near-collinear disjoint segments gave
               one false alarm: repro/gridmiss.cpp).
  mutants.py / runmut.sh / mutlib.sh / mutsweep.sh   T-0169 mutants of pocket.cpp: test runs ([pocket],[t0169]) and pk runs.
  vcx.cpp      V-Carve flat-area moves of a file (repro2/nest_frame.txt) vs the walls with oc2.
  vsweep.sh    vc3 (t0160v2) hash sweeps branch vs main (default, VC_FM=2) and the cone oracle on the branch.
  pp.cpp / perfi.sh / perfsum.py   CPU probes interleaved main/branch (T-0131 pf cases, big curved pockets, super-linear
               families, V-Carve AC2 via t0160v2 perf.cpp); bigsweep.sh: cost outlier hunt at radius ~1.5 m.
  repro/       lens.cpp (pre-existing: island with sagitta < chordal/2 dropped, tool cuts through it), tinyisland.cpp,
               sc001_1099_dump.txt, gridmiss.cpp + grid_miss_rings.txt.
  results/     sweeps, mutants, wb, vc, perf; NOTES.txt.
