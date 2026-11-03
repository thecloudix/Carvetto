T-0153 verifier round 1 (commit 652cf0d). Build: ./build.sh <worktree> <worktree>/build-verify1-rel <src.cpp> <out>
(links libcarvetto_core.a + Clipper2 from a Release build; max.cpp/scale against main's medial axis: compile
 git show origin/main:core/src/geometry/medial_axis.cpp with -I<worktree>/core/src/geometry and link it first).
Repros: rep 90 6 0.02 < repro/sq2.txt      -> V-bit 2.49 mm deep at (7.5,7.5), outside the even-odd region
        REPQ=1 REPGRID=0.1 rep 90 4 0.02 < repro/tri.txt   -> lateral shortfall 0.18 mm (9 x chordal) near the apex
        REPQ=1 rep 155.3 8.618 0.02 0.339 0 0.5 < repro/star5.txt -> 0.22 mm (11 x chordal)
        max 0.02 < repro/c19.txt  (and max_main) -> 61 medial nodes outside the region on both
