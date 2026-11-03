T-0153 verifier round 2 (commit 619ec07). Build: ./build.sh <worktree> <worktree>/build-verify2-rel <src.cpp> <out>
(links libcarvetto_core.a + Clipper2 from a Release build). Ported from t0153v1 with the API renames
(v_carve.hpp, VCarvingParams, VCarvingResult, vCarvingToolpath): vc.cpp rep.cpp gen.cpp perf.cpp max.cpp.
New:
  vc2.cpp + gen2.hpp  exact cone/wall oracle over 16 generative families incl. crossing, touching, coincident,
                      self-crossing, nearly tangent (gap 0..chordal), frames with text, holes in holes, crossing
                      slivers, crossing cluster + near-tangent neighbour; membership by exact ray crossing
                      (oracle.hpp insideExact), witness search to separate coincident-edge false alarms.
                      env VC_FAM, VC_FILE, VC_FIX, VC_FAR, VC_SCALE, VC_PTS, VCMUT (oracle sanity), VCDUMP, VCDBG.
  sweep.sh            the round-2 sweep (5 jobs).   nest.cpp  region partition + machining order checks.
  perf.cpp            + VPERF_FRAME (letters as holes of one plate), VPERF_EM=<D> (end-mill clearance).
Repros (repro2/):
  hole_ignored_min.txt   REPNOCOV=1 rep 90 12 0.01 < repro2/hole_ignored_min.txt  -> diag 17 "ignored", 57 moves inside
                         the hole circle (r 2.995, 0.005 from the wall), plunge to Z-4.99 at (5,5). Same: nearint_c60,
                         nearint_c7, slivercircles_c18, nearsq_c14, nearsq_c57 (medialAxis drops a single-span hole whose
                         probe is within chordal + distance of another contour; V-carve carves on).
  hole_near_cluster_min.txt  REPNOCOV=1 rep 90 6 0.05 < ... -> plunge to Z-0.95 at the centre of the hole circle (r 1,
                         0.003 from circle A, which crosses circle B): the chordal/8 polygon of the crossing cluster
                         cuts the exact hole. Also hole_near_cluster.txt (sweep case).
  sharededge.txt         coincident edges: coverage numbers of vc2 are not meaningful for family 15 (distance includes
                         the cancelled edge); cone false alarms there are filtered by the witness.
  sliver_c82.txt         ball nose: a crescent part skipped as "nothing to carve" (depth < distance tolerance) though
                         its footprint shortfall is ~10 x chordal (VC_FIX="1 11.606 0 0.01 0 0").
  nest_*.txt             layouts written by nest.
