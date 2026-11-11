# ADR-0042: Prism carving toolpath (geometry and conventions)

Date: 2026-11-10 · Status: accepted, implemented (T-0240).

## Context
The inventory lists only the field names: Prism Depth, Set Depth for Full Prism, Cut Direction. The
card fixes two results: the peak depth equals the stated depth for a symmetric shape, and the full
prism option reaches the tool-limited depth. The meaning of "prism" is not described further.

## Decision
1. **Shape.** A prism is the inside of a closed vector carved into a faceted pyramid-like recess:
   the vector stays at the surface and the facets fall towards the middle at the tool's flank angle,
   so the lowest point (the "peak depth") is at the prism depth. Interpreted this way it is a V-carve
   whose depth is stated rather than found; the product owner confirmed this reading (2026-11-10). A raised prism (surface at the
   centre) would only change the sign of the facets; the field set would stay.
2. **Cuts.** The tool runs on inward offsets of the vector (round joins, via `profileToolpath`, side
   Inside, allowance 0): the cut at inset s is made at the depth where the tool's cutting width at the
   surface is 2 s (`vCarvingDepthForRadius`), so its edge sits on the vector. Insets run s = stepover,
   2 stepover, ..., to the last, outermost first. Every cut is one Profile run (order, direction,
   trimming near other vectors, diagnostics); regions are the even-odd region (holes stay raised).
3. **Extent.** The largest inset is the width cut at the prism depth, or, if the shape runs out
   before that (offset empty, found by bisection to about 5e-6 mm), the shape's own maximum inset
   less 1e-3 mm (a cut nearer the peak collapses to nothing in Profile). `reachedDepth` reports the lowest cut. A symmetric shape deeper than its inradius
   therefore ends at its peak, and a shallower prism at exactly the stated depth.
4. **Full prism.** The depth requested is the tool's maximum cut depth (`vCarvingFullWidthDepth`: a
   V-bit's cone height, a ball nose's radius); the prism ends at that depth or where the facets meet.
   A stated depth beyond the maximum is limited (`CarveDepthLimited`).
5. **Tools.** V-bit and ball nose. Open vectors are ignored with `OpenContourIgnored`.
6. **Limits.** At most 2000 cuts (`WorkLimitExceeded` beyond; stepover 0.001 to 100 mm). Refusals of
   a Profile run (parameters, work budget, cancel) end the run with nothing partial.
7. **Record and form.** `prism` record in job.json only (like chamfer); form fields depth (hidden with
   full prism), full prism flag, stepover, safe Z, start position, chordal tolerance, direction, order.
