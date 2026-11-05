T-0164 verifier round 1 (branch task/T-0164-vcarve-followups, commit ff45327; main reference 6de8648).
Libraries: Release -march=x86-64-v3 builds in /home/chris/wt-T0164/build-verify1-rel and /home/chris/wt-T0164-main/build-verify1-rel;
binaries in /home/chris/scratch-t0164v1/bin (not committed). ./builddrivers.sh [rel|sab] builds the drivers (build.sh as in t0153v6).
Copied from t0153v6/v3 and extended:
  vc3.cpp        HASH line now prints mh (moves-only hash), dl (diag issue list), so/width (carver stepover vs 2*footprint(limit)), ncl;
                 VC_CSOW=<f> sets the carver's stepover to f x width in random clearance-tool cases (no RNG use).
  fuzz6.cpp      + BADEND_R=1 (moved branch end with an exact radius: only end==node broken), BADSTART=1 (branch start moved).
  grow.cpp       + hash of moves and diagnostics (byte identity across versions).
  sabotage164.patch  t0153 sabotage (SAB=2/3/4) + SAB=5 silently drops contour SABI (default 1) in medialAxis buildPieces without
                 counting it as ignored, SAB=6 drops every contour but 0, SABEND=<d> moves every branch's last point by (d,d)
                 (SABEND2=1 also moves the first point by -d in x). Apply to a copy of the source (never commit).
New:
  geo/drop_*.txt dropped-hole attack inputs (hole on the centre line of a 100x10 strip, off centre, big hole, near a square's corner,
                 on the diagonal, near an edge).
  cmphash.py     family hash sweep branch vs main: moves must match; diag differences only StepoverLimited (29) with ncl>0, so>width.
  csowcheck.py   (d): StepoverLimited present <=> moves change when the carver's stepover is cut to 0.5 x width.
  cmpadv.py      adv/adv2 advhash outputs branch vs main.
  growrun.sh, perfrun.sh, regr.sh, fuzzrun.sh, sabend.sh, golden.sh  background runners.
  results/       NOTES.txt and raw outputs.
