T-0153 verifier round 3 (commit df19a98). Build: ./build.sh <worktree> <worktree>/build-verify3-rel <src.cpp> <out>
(links libcarvetto_core.a + Clipper2 from a Release build; bin/ is not committed).
Oracle fix vs round 2: acc.hpp inside() (ray parity) is a majority over three tiny y shifts. Round 2's insideExact
miscounts when the ray passes exactly through span end points (full circles starting at angle 0 end at y=-2e-15),
which gave false "centre outside region" alarms on y=0 skeletons (nearint_g*.txt, hole_near_cluster_min.txt).
  vc3.cpp + gen3.hpp + acc.hpp  vc2 oracle with: span-box index (acc.hpp: exact distance + inside), coverage bound
                      1.0 x chordal (VC_BOUND), chordal set {0.001..0.1} (VC_CH fixes it, e.g. 0.0003), families
                      16..23 (gen3.hpp: tiny spans + long span, single-point touch, nested within chordal, chains,
                      all-touching page, dense flat corners, self-touching unions, nesting probes on boundaries),
                      floor check (FLOOR lines: floor points left more than half the flat depth uncut), VC_F15=1
                      (family 15 against the exact outline, coincident edges removed), VC_EM/VC_CSO/VC_SO
                      (end mill + carving-tool stepover in VC_FIX mode).
  sweep.sh            round-3 sweep (xargs -P).   repros.sh  every round-1/2 repro through vc3.
  speck.cpp           specks / degenerate contours next to a job.   minch.cpp  chordal 0.00029/0.0003/0.001.
  grow.cpp            cost of iterative cluster growth (pairs with near holes: quadratic).
  steep.cpp           steep descending feed moves (ball nose at full radius).
  dbgin.cpp           the oracle inside() artefact.
  ma/                 T-0152 drivers (ac1, fuzz, scale, far, cap) + gab.cpp (Gabriel search on dense flat bands),
                      built against main and the branch.
  repro3/             arm_W*.txt (narrow arms: rest-tool ridges), sq10.txt, speckhole_*.txt.
  results/            sweep, repros, perf, ma, gcheck summaries; NOTES.txt.
