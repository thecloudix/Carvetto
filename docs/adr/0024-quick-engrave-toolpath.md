# ADR-0024: Quick Engrave toolpath (depth and passes, Fill through Pocket, record schema)

Date: 2026-11-03 · Status: proposed (T-0151), amended 2026-11-03 (T-0154: diamond-drag tools run with the spindle off)

## Context
T-0151 adds the quick engraving toolpath (feature-inventory §5 "Quick Engraving Toolpath"): mark
the selected vectors at a depth, either following them (Outline) or filling the closed regions
with a pattern and stepover (Fill), in a number of passes, with V-bits, engraving tools, end
mills, ball noses or diamond-drag tools; stored as a native toolpath record (ADR-0021) and
available as `carvetto-cli toolpath engrave`. AC2 requires Fill to reuse Pocket's raster/offset
machinery (T-0121/T-0123) and to be gouge-free by the pocket oracle. Nose cone / pressure and
"output direct to machine" are out of scope. The card leaves open the depth and pass semantics,
how Fill treats V-bits and open vectors, and the details below.

## Decision

### 1. Depth and passes
The inventory form has one depth ("depth to engrave / mark") and a number of passes, and no
start depth, so `EngraveParams` has `depth` (> 0, below the material surface Z = 0) and
`passes` (1–1000). The passes are equal: `engraveLevels(depth, passes)` is `PassPlan` with
start depth 0, cut depth = depth, pass depth = depth / passes and `maintainExactStep` = true,
which yields exactly `passes` levels (−depth·k/passes) with the last one exactly −depth. Each
pass must be at least `kEngraveMinPassDepth` = 0.000001 mm deep (depth / passes); below that
PassPlan's then absolute 1e-9 mm rounding guard merged levels (relative since T-0145; the limit
stays, as no machine resolves thinner passes), so such parameters are refused with
`InvalidParameters` ("each pass must be at least 0.000001 mm deep …") and `engraveLevels`
returns no levels (T-0155). Fill
hands the same plan to Pocket, so both strategies cut the same levels. Safe Z must be above the
surface (> 0). The cap of 1000 passes keeps Pocket's level cap (10,000) out of reach; a total of
more than 20,000,000 moves is refused (`WorkLimitExceeded`) as in Pocket and Drill.

### 2. Outline
- The tool centre runs on the vector itself (engraving marks the line drawn; there is no side).
  Lines stay lines and arcs stay arcs; Beziers are flattened within `tolerance.chordal` (the
  shared `ArcLengthPath`/`appendLeg`, which also writes the vertical step-downs).
- A **closed** vector is cut round once per level, stepping straight down at its start point
  to the next level; the tool never leaves the groove between passes.
- An **open** vector is cut **back and forth**: forward at the first level, then the tool steps
  down at the end it reached and cuts back, and so on. Lifting to safe Z and returning to the
  start after every pass would add air moves without changing the groove (a centre-line groove
  has no climb/conventional side).
- Each vector: rapid at safe Z above its start, plunge at the plunge rate, cut at the feed
  rate, retract to safe Z. Vectors are ordered by the shared `orderCuts` with no ordering
  option set (nearest first from `startPosition`; a closed vector inside another is cut first,
  the helper's fixed rule). The inventory form has no ordering options, so none are offered.
- Vectors without spans or without length (`EmptyContour`) and with gaps
  (`DiscontinuousContour`) are skipped and named, as Profile does.

### 3. Fill is Pocket
Fill calls `pocketToolpath` with one `PocketTool` (the engraving tool with its cutting data,
stepover and pass depth replaced as below) and `PocketParams` mapped from the engraving:
start depth 0, cut depth = depth, pass depth = depth / passes with `maintainExactStep`, the
chosen pattern (`PocketStrategy::Offset` or `Raster`, with `rasterAngle`), climb, the boundary
pass **last** (clean edges of the marking; for Raster it also removes the wall scallops), no
allowance, no ramp, the same tolerance and safe Z. Pocket's moves become the engraving's moves
unchanged and its diagnostics are passed on, their `tool` index cleared (there is one tool) and
their wording made the engraving's: "tool 1 (name)" becomes "tool 'name'" and the pocket the fill
area ("too large for the fill area", "filling needs closed vectors", "at the engrave depth"). Only
Pocket's fixed phrases are replaced, never text inside the tool name (T-0155). No
fill code is duplicated; the tests compare Fill with a direct `pocketToolpath` call move for
move.

Patterns: the inventory lists "three fill patterns" shown only as images (their names are
`[NOT FOUND]`, feature-inventory "Gaps"). Fill offers Pocket's two, Offset and Raster, as
`fill_pattern`; another pattern can be added later as a new value without a format change.

### 4. Fill with V-bits: the width at the engrave depth
Pocket already converts the tool with `passTool` at the bottom of the cut: its radius is half
`tooling::cuttingWidthAtDepth(tool, depth)`, so for a V-bit the cone's radius at the engrave
depth (for an engraving tool its flat tip plus the cone, for a ball nose the chord of the ball).
The centre region is the region inset by that radius plus Pocket's chordal safety margin, so the
cone's top edge never crosses a vector at the final depth, and the shallower passes, narrower,
stay inside too. This is what "gouge-free by the pocket oracle" checks with the radius at depth.

The stepover is a toolpath parameter, `stepover` in mm (the inventory shows Stepover on the
form), with 0 meaning the cutting data's stepover. It must be > 0 and at most the width at the
engrave depth, otherwise the toolpath is refused with `InvalidTool` naming both widths ("the
stepover 2.54 mm is wider than the 1 mm the tool cuts at the engrave depth of 0.5 mm"). The
parameter matters for V-bits: their cutting-data stepovers (1.27–2.54 mm in the starter
database, meant for V-carve clearance) are wider than a shallow engraving cuts.

### 5. Open vectors in Fill are ignored
Fill fills regions; an open vector encloses none. Open vectors are ignored and counted in one
`OpenContourIgnored` diagnostic naming the first (as Pocket and Drill do), and only open vectors
give `NoRegion`. Engraving them as an outline instead would mix two strategies in one toolpath
with no way to tell from the parameters; a second Outline toolpath on the same selection does
that explicitly.

### 6. Tools
`engraveToolAllowed` accepts `VBit`, `Engraving`, `EndMill`, `BallNose` and (since T-0154)
`DiamondDrag`. Tapered ball noses, radiused end mills, form tools and drills are refused with
`InvalidTool` ("tool 'X' is a drill; quick engraving needs a V-bit, an engraving tool, an end
mill, a ball nose or a diamond drag tool"); the list can be widened without a format change. The
tool must pass `tooling::validate`, have a cutting width > 0 at the depth and feed and plunge
rates > 0.

**Diamond-drag tools run with the spindle off (owner decision 2026-11-03, T-0154).** A drag tool
must not turn, and every other toolpath starts the spindle with its tool's speed (the posts'
T-0134 rule, ADR-0020, refuses a speed ≤ 0 wherever `[S]` is written or `SPINDLE_ON` runs). Of
the options put to the owner (keep the rule with a nominal speed; "no spindle" toolpaths; those
plus `M5` and an operator stop before the drag toolpath; refuse drag tools for now) the owner
first chose to refuse them (T-0151), then had T-0154 build the third: a drag tool follows the
vectors at the given depth exactly like the other tools (Outline and Fill alike; its cutting
width is its diameter), and `engraveToolpath` marks the toolpath's tool snapshot `spindleOff`
with spindle speed 0, whatever the cutting data's speed, and records its tool type
`diamond_drag`. The post then writes no spindle start
for it and writes the post's `SPINDLE_OFF_PAUSE` (`M5`, safe Z, a message, `M0`, safe Z) before
it, and `SPINDLE_UNLOCK_PAUSE` after the drag section, before a spinning toolpath or the program
end (ADR-0020).
Quick Engrave is the only operation that sets the flag; the record format allows it on engrave
records only (§7). Profile and Pocket refuse diamond-drag tools (owner decision 2026-11-03), so
Fill hands Pocket the drag tool as an end mill of the same diameter (identical moves); the
toolpath keeps the drag tool's snapshot.

**Fill with a drag tool is allowed.** A drag tool marks at a constant depth along any path and
has no cutting direction, so Pocket's offset rings or raster lines at the engrave depth (plunging
at the plunge rate, no ramps: Fill turns ramps off) fill a region with lines one drag-tool width
apart, like a hatch; the stepover (§4) is at most the drag tool's diameter. Refusing Fill would
leave drag users without filled marking for no safety gain: the spindle rule is the same for
both strategies.

### 7. API, record and params.json
`engraveToolpath(contours, const tooling::Tool&, const std::optional<tooling::CuttingData>&,
const EngraveParams&) → EngraveResult{toolpath, diagnostics}`; the cutting data is a separate
argument as in Drill (ADR-0022 §5), so `EngraveParams` holds only record content. No new
`ToolpathIssue` values are needed. `ToolpathParams` gains `EngraveParams` as its fourth
alternative and `ToolpathOperation::Engrave` (`"operation": "engrave"`). job.json params (all
optional on load, unknown keys dropped, numbers finite, ADR-0021 §2) and params.json use the
same snake_case keys: `name`, `depth`, `passes` (an integer), `safe_z`, `strategy`
(`"outline"` | `"fill"`), `fill_pattern` (`"offset"` | `"raster"`), `stepover`,
`raster_angle_rad` (params.json: `raster_angle` in degrees), `start_position` ([x, y]) and
`tolerance`; params.json adds `tool` and `layers`, and unknown keys are errors there.
`nonFiniteParam` names engrave keys the same way. The record's `tool` object gains an optional boolean `spindle_off`
and `tool_type` (T-0154), both written only for a spindle-off snapshot (`"tool_type":
"diamond_drag"`); `spindle_off` requires an engrave record with that tool type, and the loader,
`saveCarv` and the toolpath commands refuse anything else (ADR-0021 §2). The format stays **1.1** (a new optional
variant alternative, as ADR-0021 foresaw); a build from before T-0151 refuses a file with an
engrave record ("unsupported toolpath operation 'engrave'").

## Alternatives
- **A start depth like Pocket's:** not on the inventory form; engraving marks from the surface.
- **Retract and restart open vectors at every pass:** more air moves, same groove.
- **Separate fill code for engraving (e.g. a hatch generator):** duplicates Pocket's safety
  guard and linking, which AC2 rules out.
- **Stepover as a percentage of the width at depth:** depth-independent, but the cutting data
  and every other toolpath use mm; the CLI and record stay in mm.
- **Outline open vectors in Fill:** see §5.
