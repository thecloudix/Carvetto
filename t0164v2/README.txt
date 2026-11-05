T-0164 verifier round 2 (branch task/T-0164-vcarve-followups, commit cce938c; main reference origin/main 6e537a1).
Libraries: Release -march=x86-64-v3 builds in /home/chris/wt-T0164/build-verify2-rel and /home/chris/wt-T0164v2-main/build-verify2-rel;
sabotaged copies (sabotage164.patch from t0164v1, SAB=5/6/SABEND) and an instrumented copy (cluster-loop counters on stderr)
in /home/chris/scratch-t0164v2/src-{sab,sabmain,instr}; binaries in /home/chris/scratch-t0164v2/bin (not committed).
buildall.sh builds everything, builddrivers.sh [rel|sab|instr] the drivers. Carried over from t0164v1 (paths updated):
vc3/net/fuzz6/grow/perf drivers, hashsweep/advhash/regr (400-case + adv/adv2 identity), fuzzrun, sabend, perfrun, golden, cmp*.py.
New:
  grow2.cpp     cluster attack patterns (pairs, pairsrev: absorbed vector below the root, pairsmix: shuffled pairs + extra holes,
                star: one resolution absorbs N, rand, dense: shuffled cascades hole->island->..., nest: 30-200 deep cascade,
                chain/revchain/shufchain/meet/bridge (no absorption at these gaps; identity only), gear/gearh/dpoly: big single
                parts where own rings are measured at every point (cost of B2)). Prints CPU s + FNV hash of moves+diagnostics.
  ufsim.cpp     pure-logic differential test: main's restart loop vs the branch's worklist loop copied verbatim with a
                deterministic oracle for union/intersect (order-dependent touches, own members re-touched, union-find parent
                overwrites); compares final members, crossing flags and the member list each root was resolved with.
  chunk.cpp     SkeletonCheck::closer (16-segment run boxes) vs brute force; adversarial points that miss a run box by an ulp,
                touch it exactly, sit at ~limit from a segment; degenerate rings (repeated points, zero-width runs, 1-2 points).
  b3.cpp        StepoverLimited at stepover = width, ulps around it, within/beyond the distance tolerance (4 tools x 3 tolerances).
  sab5.sh       SAB=5 (per SABI) / SAB=6 / SAB=0 control on every geo/drop_* (7 new: multi, slot, nearwall, tiny, L, annulus_isl,
                ring_hole) x chordal x 3 tools, patched branch vs patched main.
  identrun.sh, densefuzz.sh, growrun2.sh, growrun3.sh, fuzzsum.py   runners / summaries.
  results/      NOTES.txt and raw outputs.
