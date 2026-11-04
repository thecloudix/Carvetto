T-0153 verifier round 4 (commit 209499a). Build any driver: ./build.sh <worktree> <worktree>/build-verify4-rel <src.cpp> <out>
(links libcarvetto_core.a + Clipper2 from a Release build; binaries are not committed).
Reused from round 3: vc3.cpp (oracle; VC_FILE/VC_FIX/VC_DIST/VC_EM/VC_SO/VC_F15/VC_F23...), acc.hpp, gen3.hpp, dma.cpp, nsq.cpp,
perf.cpp, ma/ (T-0152 drivers built against the branch br_* and origin/main main_*).
New in round 4:
  genadv.py -> adv/   288 adversarial hole files around B1: D-holes (gap 0.0005..0.09, sweep 0.3..6.25 rad, 24/170 segments,
                      cw/ccw, rotated start), D on a polygonised wall, lunes (thin tips), L-holes and frame holes hugging a square,
                      sandwiches (outer/hole/island within g), annuli (+island, ccw, polygonised), touching/crossing D-holes,
                      half-disc pairs, thin bands, alternating nested circles, D-hole at nesting depth 3.
  runadv.sh           each adv file x chordal {0.1,0.05,0.01} x distance {default,1e-5} x {V90 D12, ball D6}, + chordal 1.0
                      (distance 1e-5) and V30 D6: one vc3 VC_FIX case each (results/adv.txt, 4320 runs).
  genadv2.py -> adv2/ near-full D-holes + island at the hole's deepest point (natural attempt on the net).
  sabotage.patch      fault injection (never committed to the branch): SAB=2 flips contour SABI's side in medialAxis
                      classifyContours, SAB=3 flips every contour but 0, SAB=4 disables V-carve nesting (every boundary its own part).
                      Apply to a copy of the branch, build core, link vc3 -> bin_vc3_sab. Shows the round-4 safety net misses a
                      wrong-side hole whose skeleton's deepest points lie inside an island (repro4/annulus_isl_g0.01.txt,
                      repro4/framehole_isl_g0.05.txt: -3.3 / -5.3 / -1.99 mm cone violations, not refused).
  brs.cpp             medialAxis branches of a file: deepest point per branch, radius, inside, distance (what the net looks at).
  wrongside.cpp       random search (non-crossing inputs) for a contour read on the wrong side without being ignored.
  minch4.cpp          B2: chordal 0.00029 / 0.0003 / 0.000300001 / 0.001 with flat depth (usable stepover) and an end mill.
  glyphs.cpp          per-glyph and page medial axis of ma/scale.cpp's text (page = crossing input).
  sweep4.sh           families 0..23 (25 cases), fam 18 x2 (incl. chordal 0.1), mixes, far/scale, chordal 0.0003 x 6 families.
  repro4/             fam18_s4818_c2.txt / f18.txt (fam 18 chordal 0.1 cone -0.0139 mm) and f18_nosliver.txt (same with the
                      0.63 um sliver removed: clean) - the documented sub-distance class; the two net-bypass inputs.
  results/            adv.txt, sweep/, ma/, perf.txt, minch4, wrongside_*, fam16_s316*, NOTES.txt.
