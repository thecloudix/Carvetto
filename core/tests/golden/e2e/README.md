# End-to-end acceptance goldens (T-0354)

G-code written by the shipped Grbl post for the three scenarios of `core/tests/e2e/acceptance_test.cpp`
(PLAN.md section 5, "MVP acceptance"). The test builds each job from scratch through the library
(no files but the fixtures below), compares the program byte for byte, re-reads it and matches every
motion line with the toolpath's moves, and simulates the toolpaths on a heightfield.

| File | Scenario | Inputs |
|---|---|---|
| `sign_grbl.nc` | 2D: DXF, text on a curve, pocket, V-carve with a clearance tool, profile with tabs | `core/tests/fixtures/e2e/bracket.dxf`, the OFL test font, starter tools `end-mill-6mm`, `end-mill-1mm`, `v-bit-60-1-4in`, `end-mill-3mm`, `posts/grbl_mm.pp` |
| `relief_grbl.nc` | 3D: STL (generated in the test) rastered top down, Z-level rough + raster finish | starter tools `end-mill-6mm`, `ball-nose-3mm`, `posts/grbl_mm.pp` |
| `rotary_grbl.nc` | Rotary: a vase-like solid of revolution on a 20 mm cylinder along X, rough + finish, wrapped to A | same tools; `posts/grbl_mm.pp` with `ROTARY_WRAP_Y = "A"` and an `[A]` word added to the move blocks by the test (no rotary post ships yet) |

Regenerate only with `python3 tools/regen_golden.py` (it runs the hidden `[.regen]` test), then
hand-review the diff against the checklist in `posts/README.md` (units and absolute mode first, rapids
only at safe Z, spindle on before cutting, tool change with the spindle off, nothing below the stock
bottom) before committing. A golden changing without an intended change in a toolpath generator, the
post or the starter tools is a regression.
