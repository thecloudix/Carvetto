T-0158 round-4 verifier harness (branch task/T-0158-tip-start 999a90e; origin/main c2aa6f4; round 3 = 520ca28, 5649b2b merged with
main). Extends t0158v3 (st/gen/gen2/gen3/probe3/edge/wsf flood). GCC 15.2 only (GCC 13 not installed); cross-compiler stand-in =
logfma: core + Clipper2 built with -march=x86-64-v3 -ffp-contract=fast (instead of -ffp-contract=off).
Libraries (mkall.sh -> /home/chris/wt-T0158/build-verify4-rel): br (worktree), main (origin/main export), r3 (520ca28 export),
log/logv3/logfma (999a90e export + tfs_instr4.py; TFS_OFF=1 = main, verified hash-identical to br/main), logd (log + TFS_DETAIL
per-junction dump and mLopH). mk.sh builds a harness program per variant.
- tfs_instr4.py: instruments the round-4 tieFreeStart by textual inserts (decisions untouched); TFS2 line + " K4 kind back mOnC
  mEqui mLop rj" (kind 1 round-join first point on the circle, 2 equidistant-junction pick, 3 "most even feet" only).
- st.cpp (T line + K4 columns; st_logd adds mLopH), sweep.sh / rndsweep.sh, q4.sh (gen3/gen12 pert, generated fma, stored scn
  VARS=1 x {log,logoff,logfma,logfmaoff}, stored pert, random), q4b.sh (r3 hashes), q4c.sh (logd sweeps), margins.py (calls near
  each round-4 tolerance), xcomp.py / changed.py / rndrate.py / pmove.py (from round 3).
- probe4.cpp: col (neighbour vertex B at L before/after the tip, turning tB incl. exactly collinear), thr (bisects the tB at
  which the start flips between B's corner and A's mitre; start move under ulp/1e-9/translation at it), mrg (merged region end
  mill, region vertex x before the tip: 5 um bound, two-vertex corners), chord (exact offset, join arc length around tol); OFF=x
  translates (large coordinates). rect4.cpp: rectangle with one redundant collinear vertex L before the start corner.
- chk4.cpp: classifies the start of single-vector scenes (mitre / chamferFirst / chamferLast / foot / run4) from the moves
  (one pass, no tabs/leads/ramps) and |S - mitre|; one4.cpp: one run with stderr (TFS_DETAIL) and S/UP/DN/OX/OY transforms;
  mv4.cpp move dump.
- floodcmp4.sh (br / main / r3 whole-scene flood, wsf.cpp) + floodagg.py A B; mutants.sh (round-4 mutants M0-M7).
- repro4/: rect_collinear_* (20 x 10 rectangle, collinear vertex 0.05 mm before the start corner: Conventional starts 3.05 mm
  from the mitre), collinear_* (tip 1.2 rad, collinear vertex 20 um before: 2.07 mm off), nearcollinear_thr_* (neighbour turn at
  the equidistance tolerance: start flips 2.06-2.07 mm under +-1 ulp / 1e-9 / translation, and plain vs FMA), bigc_* (3 m
  coordinates: a 2 um edge turning 1e-3 rad takes the start, 2.06 mm), mrg_twovertex_* (merged region, region vertices 3 um
  apart: two near runs -> projection, 0.49 mm under 1e-9), fpoly_057_split (fine polyline: start 1 um off under 1e-12 scaling,
  a collinear split point of the walk taken as a corner).
- results/: NOTES.txt and summaries.
