T-0142 round-1 verifier harness (branch task/T-0142-boolean-cost @ 35b8ca9, base origin/main 6de8648).
Build: builds.sh (Debug -Werror, Release -march=x86-64-v3 branch, Release -march=x86-64-v3 main from git archive).
mk.sh src out {br|main}: links a driver against the branch or main Release build (bin/ is git-ignored).
Drivers:
  bat.cpp    (a) booleanOp batching: booleanOpBatched(minDepth 0) vs one call (kNever) vs default; 6 layouts incl.
             touching boxes, vertex-touching diamonds, 1-grid-unit gaps, crossing subject + open path across the row;
             all 4 ops x 2 fill rules; independent winding oracle from the input, XOR area, open-piece coverage.
  hb.cpp     (a) byte hashes of booleanOp/unionContours/offsetContours, G<32 and G>=32 groups, main vs branch.
  pb.cpp     (a) boolean scaling n->2n interleaved CPU (row/jit/holed/nest/grid x union/diff/inter/xor).
  strip.cpp  (e) OpenPathClipper vs booleanOp on >=1024-edge clips: comb, spiral band, crossing stars, aligned square
             grid, serpentine, edges ending +-1..4 grid units around the margin; angles 0/30/45/90/random; Pocket-like
             line batches, random zigzags, lines along clip edges; DIAG=1 classifies every unmatched end
             (ILL = on a clip edge within 1e-3 rad of the subject, REAL = well-conditioned end > 2 units off). SC=k one scene.
  c90.cpp    (e) clip90 with OpenPathClipper: comb inset vs raster clip scaling separately.
  pc.cpp     (e) Pocket raster on the comb: time + move hash per (teeth, angle), main vs branch.
  nearestfirst_revert_mutant.patch  (e) T-0131 nearest-first revert used to check the new guard test.
  lo.cpp     (b) outset oracle independent of Carvetto geometry: LOST / FAR / LEFT(over ring) vs region rel*winding>0;
             fam 0 lone+nested, 1 crossing, 2 tied lowest contours, 3 reversed-lid lone rings, 4 mixed, 5 card repro,
             6 undetected crossing (hole box inside a U-shaped outer's box, crossing into the notch).
  hn.cpp     (b) outsets of properly nested input, byte hashes main vs branch.
  po.cpp     outset cost of many properly nested holes (row/grid): quadratic nesting scan in resolveUnnestedHoles.
  dfl.cpp    (d) distance floor: Profile/Pocket/pocketCentreRegion/Engrave Fill+Outline/V-Carve (clearance, flats)
             across distances incl. nextafter(1e-6,0); NOPROF=1 skips the slow Profile cases on main.
  /home/chris/harness/t0131v1/hs.cpp, t0141v1/pcw.cpp (KIND=8/9 NTOOLS=3 safety oracle), t0141v1/perf.cpp, bez.cpp reused.
results/notes.txt: numbers and verdict.
