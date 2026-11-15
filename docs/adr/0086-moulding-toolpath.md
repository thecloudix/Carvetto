# ADR-0086: Moulding toolpath

Status: accepted (T-0307; completes the part of T-0298 that ADR-0073 split off)

Behaviour source: `docs/reference/feature-inventory.md` section 5 "Moulding Toolpath" (Toolpath Position with Z from the
cross-section height, drive rails with Reverse/Clear, open profile with Reverse Profile, finishing Stepover / Vary
Stepover / Skip Flat Regions / Create Sharp Corners, roughing Use Larger Area Clearance Tool / Machine Flat Regions /
Ramp Plunge Moves / Machining Allowance, Boundary Offset / automatic boundary offset). The reference gives the option
names only; the readings below are product readings for owner review.

## Model
- **Section.** The profile is an open polyline in a plane across the rail: X = distance to the **left** of the rail's
  direction of travel (negative = right), Y = height; machine Z = Y + `zOffset`. X must be single valued
  (non-decreasing or non-increasing; equal X is a vertical step) and have a positive extent; otherwise
  `InvalidParameters`. The moulding surface is the extrusion of the section along each rail: its height at a point is the
  section height at the point's signed distance from the rail. Each rail is its own run (a frame is one closed rail).
- **Passes.** A pass follows the rail offset by u (`offsetOpenContour`, polygonised at `tolerance`; an open or closed
  rail; u = 0 is the rail itself; a result in several chains gives one run per chain) at **constant Z**: the exact drop
  height of the tool on the section, `max_t H(t) - h(|t - u|)` with the tool's exact profile (`simulate::ToolShape`).
  On each side of u H is linear and `-h` concave (h is convex), so the maximum of each piece is found by a ternary
  search (100 iterations, to rounding); end points of pieces and vertical steps are evaluated directly. Hence no pass
  position, and no point between two passes, cuts below the section (at the section's own resolution). The model is
  an extrusion: at a rail corner the Z of a pass is still the Z of its distance u (on the inside of a turn the true
  surface is only approximated, as for any swept moulding).
- **Toolpath Position.** (reading) The position says which part of the drawn section lies on the rail (offset 0):
  `AsDrawn` (the drawing's X = 0), `Start` (the first drawn point), `Centre` (middle of the X extent), `End` (the last
  drawn point). The anchor is subtracted from X. The Z comes from the section height, not from the position.
- **Reverse Profile.** (reading) mirrors the section about the anchor (left <-> right of the rail). **Reverse rails**
  reverses each rail's direction, which also swaps left and right and the start of every pass.
- **Boundary offset.** (reading) extends the machined band by that distance beyond both ends of the section, the
  section continuing flat at its end heights (so the tool leaves a level shoulder instead of cutting down outside the
  moulding). The automatic offset is the tool radius, so the tool edge reaches the end of the section.
- **Finishing.** Passes at u from the first to the last offset of the (extended) section in steps of the tool's
  stepover (limited to the tool diameter, `StepoverLimited`), the last pass exactly at the end. **Vary Stepover**
  (reading): the plan step is `stepover / sqrt(1 + s^2)` with s the steepest slope over the step (so the step measured
  along the profile stays at the stepover; across a vertical step the minimum step 0.05 x stepover is used). Order:
  Climb = from the left (high u) to the right so the uncut material is on the right; Conventional the reverse;
  `reverseSteps` flips; zig-zag alternates the direction along the rail, one-way keeps it. Each pass is
  rapid at safe Z, plunge, feed, retract (no linking). Passes whose tip would be at or above the stock top are not cut;
  deeper than `stockBottom` is capped silently. The allowance applies to roughing only.
- **Flat regions.** (reading) a run of consecutive section segments each no steeper than `flatAngle` (default 1 degree)
  and at least one stepover wide. **Skip Flat Regions** drops finishing passes strictly inside such a run (the passes at
  its edges stay); in roughing `machineFlatRegions = false` does the same. A narrower flat is not a flat region.
- **Sharp corners.** (reading) the corners of the rails' offset curves: mitred (`JoinType::Sharp`, mitre limit 4)
  instead of round on the outside of a turn.
- **Roughing.** Z levels from `stockTop` in steps of the tool's pass depth; at a level every pass whose target
  (drop height + allowance, floored) lies below the level above is cut at `max(level, target)`; so the stock left is
  above the section by at most the allowance plus what the tool cannot reach. **Ramp Plunge** enters each cut along the
  pass (`appendPathRamp`, angle `rampAngle`), after a vertical plunge only down to the level above.
- **Larger area clearance tool.** (reading) with `clearTool` a first toolpath "<name> [Clear]" roughs with the larger
  tool, including its flat regions (clearing flats is its point), and the main tool then cuts only where its target is
  more than `restTolerance` below the stock the clearance tool left at that offset
  (`min over clear passes of tip + h_clear(|u - u_c|)`, capped at the stock top). Result order: Clear, main.
- **Bounds.** Work (tool-drop evaluations and level x pass visits) limit 4e8 and 20 M moves per run
  (`ToolpathRunControl`), cancel polled per pass; refused runs return no moves and `WorkLimitExceeded` / `Cancelled`.
  An empty result gives `NoPasses`. No new `ToolpathIssue` values (the app switches over them).

## Not done
Linking between passes (every pass retracts to safe Z), two rails driving one section (the section scaled between
them), a section that overhangs (undercut), and an automatic rail from a boundary are left for later; a pass at a
constant Z does not follow a rail that itself rises (rails are planar). Drive-rail "Clear" is a UI action.

## Mutation notes
Twenty-four mutants were run by hand, one build each; every one fails at least one `moulding*` test: right-side piece
of the drop search skipped, no boundary extension, climb order inverted, centre anchor at the minimum, no mirroring for
reverse profile, vary stepover without the square root, flat edges inclusive, sharp corners ignored, rest stock ignored,
no allowance, ramp ignored, machine-flat inverted, no floor cap, no zig-zag, rail side swapped, ternary search of 3
iterations, flat width ignored, rails not reversed, `zOffset` ignored, skip-flat ignored in finishing, stepover not
limited, clearance tool honouring the flat rule, rest tolerance zero, passes at or above the stock top cut. Four
survived the first set of tests (stepover limit, clearance tool and flats, rest tolerance, narrow flat) and led to the
tests "a stepover wider than the tool", "the clearance tool clears flat regions", "within the rest tolerance" and the
1.5 mm flat in the skip-flat test. Three first builds were refused by `-Werror` (unused values), so their results
were void; they were rebuilt as warning-clean mutants and killed. The committed source is the unmutated file.
