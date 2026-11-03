# ADR-0022: Drilling toolpath (hole centre, tools, pecks, record schema)

Date: 2026-11-02 · Status: proposed (T-0150)

## Context
T-0150 adds the drilling toolpath (feature-inventory §5 "Drilling Toolpath"): drill the centres
of the selected closed vectors with a drill or an end mill, from a start depth to a cut depth,
optionally pecking (pass depth from the tool's cutting data; retract above the cutting start by a
retract gap, or above the previous pass), optionally dwelling at the bottom of each pass, holes
ordered like other cuts, written through the existing posts as plain moves, stored as a native
toolpath record (ADR-0021) and available as `carvetto-cli toolpath drill`. The card leaves open
what the "centre" of a closed vector is, which tools are allowed, and the details below.

## Decision

### 1. The centre of a vector is its area centroid
`drillCentre(contour, tol)` returns the centroid of the region the closed contour encloses, from
Green's theorem: A = ½∮(x dy − y dx), Mx = ⅓∮x(x dy − y dx), My = ⅓∮y(x dy − y dx). Lines are
exact, arcs use the closed form of these integrals (trigonometric polynomials of degree 2), and
cubic Beziers a 5-point Gauss–Legendre rule, which is exact for their degree-8 integrands; a gap
between the last span and the start is closed by a straight line, as in `geometry::signedArea`.
The moments are taken about the bounding-box centre so that coordinates far from the origin do
not cancel, and an offset from it at rounding level (≤ 10⁻¹²× the box size) is dropped, so
symmetric shapes (circles of any construction, rectangles, slots, regular polygons) get their
exact centre and print as clean coordinates.

Why the centroid:
- It is the circle's centre for every way a circle reaches Carvetto: one full arc, arcs, four
  Beziers from SVG/DXF splines, regular polygons from DXF polylines, so it is exact for the shapes
  people drill.
- It turns and moves with the shape: a hole marked by a triangle, a hexagon nut outline or any
  irregular outline drawn at an angle is drilled at the same point of the shape. The
  bounding-box centre is not rotation-invariant (a rotated triangle's box centre moves within it).
- It is cheap (one pass over the spans) and unique. The centre of the largest inscribed circle
  ("pole of inaccessibility") is always inside, but it is costly, only defined within a
  tolerance, and not unique (any point on the middle line of a rectangle or slot), so holes would
  jump between equally good points.

Consequences of the choice, each reported with the shared `ToolpathDiagnostic`:
- A centroid that is not inside the vector (a C or U shape, a ring drawn as one self-overlapping
  loop) is **not drilled** (`CentreOutsideVector`, naming the vector): drilling outside what was
  drawn could ruin a part, and such vectors are not hole marks.
- A vector without area (|area| ≤ tol.distance²: collinear points, a degenerate loop) has no
  centre (`NoRegion`); vectors without spans (`EmptyContour`) or with gaps
  (`DiscontinuousContour`) are skipped like Profile does; open vectors are ignored and counted in
  one `OpenContourIgnored` diagnostic (like Pocket), whose `contour` is the first one.
- "Vectors smaller than the tool": a hole is drilled whatever the vector's size — a small circle
  marking a hole position is the common way to draw holes for a bigger drill — and reported when
  the tool's cutting radius (half `cuttingWidthAtDepth` at the hole bottom, as `passTool`
  computes) exceeds the distance from the centre to the vector by more than tol.distance
  (`HoleWiderThanVector`). This is exact for any shape (a narrow slot too), unlike a
  bounding-box test.
- Holes within tol.distance of an earlier hole (concentric circles of a counterbore drawing, a
  vector imported twice) are drilled once (`DuplicateHole`, naming both vectors).
New `ToolpathIssue` values are appended at the end of the enum (`CentreOutsideVector`,
`HoleWiderThanVector`, `DuplicateHole`), so existing codes keep their values.

### 2. Tools: drills and end mills only
`drillToolAllowed` accepts `ToolType::Drill` and `ToolType::EndMill`, as the card and the
inventory name them. Everything else is refused with `InvalidTool` ("tool 'X' is a V-bit;
drilling needs a drill or an end mill"). A V-bit can spot or chamfer a hole, but that is what the
chamfer toolpath is for, and with a V-bit the hole size and "pass depth" mean something else;
ball noses, form tools, engraving and diamond-drag tools are no drilling tools. The list can be
widened later without a format change. The tool must also pass `tooling::validate` (a drill needs
a point angle), have a cutting width > 0 at the hole bottom, a plunge rate > 0 (every cut is a
plunge) and a feed rate > 0 (a computed record must hold one, ADR-0021 §3, so the toolpath could
not be saved otherwise), and, when pecking, a pass depth > 0.

### 3. Moves (no canned cycles)
For each hole, in order: `Rapid` to safe Z above the centre; for each pass level a `Plunge` to it
(at the plunge rate, no feed override), a `Dwell` of `dwellTime` seconds when that is > 0, and
between passes a `Retract`; finally a `Retract` to safe Z. Levels come from `PassPlan` with
`maintainExactStep` = true: without pecking one pass to the bottom; with it passes of the cutting
data's pass depth, the last one taking the remainder (the usual peck-drilling behaviour).
- **AboveStart**: between passes the tool retracts to the cutting start (−startDepth) + retract
  gap, clearing the chips.
- **AbovePreviousPass**: it retracts to the retract gap above the depth the pass just reached
  (chip breaking; the tool stays in the hole). "The previous pass" is read from the next pass's
  point of view, i.e. the pass just finished.
- Both are capped at safe Z. The way back down is a plunge at the plunge rate: no rapid ever runs
  below safe Z, so the descent never relies on knowing how deep the hole already is.
- `retractGap` must be > 0 when pecking (0 would make AbovePreviousPass a zero-length retract);
  `dwellTime` ≥ 0 (0 = no dwell; a dwell move is only emitted when > 0); cut depth > 0, start
  depth ≥ 0, safe Z above the cutting start, every value finite, tolerance as Pocket.
- The shipped posts write `Retract` as `G0 Z…`: with AbovePreviousPass that is an upward Z-only
  rapid inside the drilled hole (posts/README.md checklist item 3 allows peck retracts), never an
  XY rapid and never a downward rapid below safe Z.
- Work caps as Pocket's: more than 10,000 passes per hole or 20,000,000 moves is refused
  (`WorkLimitExceeded`) before any moves are made.

### 4. Order
The holes go through the shared `orderCuts` with `OrderOptions` (selection, left-to-right,
bottom-to-top, grid; none = nearest neighbour from `startPosition`). Each item's path is the
vector made open, so no hole "contains" another (containment ordering is for nested profile
cuts), and its box is the vector's box, which the sorting strategies use like Profile does; the
item starts and ends at the hole centre.

### 5. API
`drillToolpath(contours, const tooling::Tool&, const std::optional<tooling::CuttingData>&,
const DrillParams&) → DrillResult{toolpath, holes (centre + input index, in drilling order),
diagnostics}`. The cutting data is a separate argument (empty = the tool's default data,
`effectiveCuttingData`) rather than a member of the parameters as in `ProfileParams`, so
`DrillParams` holds only what a record stores and needs no normalising.

### 6. Record and params.json schema
`ToolpathParams` gains `DrillParams` as its third alternative and `ToolpathOperation::Drill`
(`"operation": "drill"`). job.json params (all optional on load, unknown nested keys dropped,
numbers must be finite, ADR-0021 §2) and carvetto-cli's params.json use the same snake_case keys:
`name`, `start_depth`, `cut_depth`, `safe_z`, `peck`, `peck_retract` (`"above_start"` |
`"above_previous_pass"`), `retract_gap` (mm), `dwell_time` (seconds), `order` (the Profile
object), `start_position` ([x, y]) and `tolerance` (job.json: `angle_rad`; params.json: degrees).
params.json adds `tool` and `layers` like the other operations; unknown keys are errors there.
`nonFiniteParam` names drill keys the same way. The format stays **1.1**: the addition is
optional, and ADR-0021 foresaw new operations as new variant alternatives. A Carvetto build from
before T-0150 refuses a file with a drill record ("unsupported toolpath operation 'drill' (written
by a newer Carvetto?)"), as for any unknown operation. There are no interim (T-0122) drill
members to migrate.

## Alternatives
- **Bounding-box centre:** simplest and what alignment tools call "centre", but not
  rotation-invariant and wrong for asymmetric outlines (a D-shaped hole, a teardrop).
- **Inscribed-circle centre:** always inside, but costly, tolerance-dependent and not unique.
- **Skip holes wider than their vector:** would refuse the common "small marker circle" drawing.
- **Drill a centroid outside its vector with a warning:** cuts where nothing was drawn.
- **Canned cycles (G81/G83):** not every post or controller supports them, their retract-plane
  semantics differ per controller, and the card does not require them; plain moves work through
  every post and keep the safety checks in Carvetto.
- **Rapid back down to just above the previous depth (as G83 does):** faster, but a rapid below
  safe Z into a hole whose depth the controller must trust; the card asks for every rapid at
  safe Z.
