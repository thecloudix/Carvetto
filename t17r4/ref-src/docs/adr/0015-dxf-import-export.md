# ADR-0015: DXF import and export

Date: 2026-10-25 · Status: accepted (director, 2026-10-25; T-0100)

## Context
T-0100 adds DXF interchange in `core/io` (`carvetto/io/dxf.hpp`). DXF is publicly documented
(the Autodesk DXF reference: group codes, value types, entity and table layouts). The card fixes
the scope: an own parser, a result made of geometry rather than a `Document`, R2000 ASCII export,
and robustness against malformed input. This ADR records the choices the card left open.

## Decision

### Reading
1. **Own pair reader.** `PairReader` streams group-code/value pairs and never copies the input.
   Values are typed by group-code range, as in the reference's value-type table. ASCII: the code
   line is trimmed and parsed as an integer. A value of a numeric code must parse completely and be
   finite (`std::from_chars`; a leading `+` is accepted, and so is an integral double in an integer
   field); anything else is an `InvalidFormat` error that gives the line. Codes outside the documented
   ranges are read as strings. A UTF-8 BOM and CRLF line ends are accepted. 999 comments are skipped.
2. **Binary DXF is supported** (R12 and R13+). The input is binary when it starts with the 22-byte
   `AutoCAD Binary DXF\r\n\x1a\0` sentinel. The first group code tells the layouts apart: 2-byte codes
   (R13+), or 1-byte codes with 255 escaping a 2-byte code (R12). Values use the type table: strings
   are NUL-terminated, doubles are 8-byte IEEE, 60–79/170–179/270–289/370–389/400–409/1060–1070 are
   16-bit, 90–99/420–459/1071 are 32-bit, 160–169 are 64-bit, 290–299 are 1 byte, and binary chunks
   (310–319, 1004) carry a length byte. An unknown code is an error, because its size is unknown.
   280–289 are 16-bit integers, as the current Autodesk DXF reference lists them. The test suite
   encodes its binary inputs from the ASCII fixtures; the director's review also checked binary
   files written by ezdxf.
3. **Structure.** HEADER (`$ACADVER`, `$INSUNITS`, `$MEASUREMENT`, `$DWGCODEPAGE`), TABLES (LAYER,
   STYLE), BLOCKS and ENTITIES are read. Other sections are skipped. A file may lack any section
   (R12 files with only ENTITIES are fine). Each entity is decoded into a small struct in its own
   coordinates. Model space is then emitted, with INSERTs expanded recursively.
4. **Errors and warnings.** These fail the import: unreadable pairs; data that ends inside a pair or
   a section (`Truncated`); input with no section at all; and exceeded budgets (`ResourceLimit`).
   A file that ends cleanly after its last section but has no `EOF` is accepted with a warning.
   Everything else is a warning: unsupported or invalid entities, tilted entities, widths, xrefs,
   undefined blocks, cycles and depth. Identical warnings are merged with a count, and at most 256
   distinct messages are kept.

### Geometry
5. **Entities** (all results are in mm):
   - **LINE:** a line span. Zero-length lines are skipped with a warning.
   - **ARC / CIRCLE:** exact arc spans. A circle is one full-circle span (ADR-0005) and its contour
     is closed. An arc whose start and end angles are equal is a full circle.
   - **ELLIPSE:** one exact arc when it is circular, otherwise cubic Beziers within
     `tolerance.chordal` (see 6).
   - **LWPOLYLINE, and POLYLINE with VERTEX/SEQEND:** each bulge `b` becomes an exact arc with sweep
     `4·atan b`. Its centre lies on the chord's bisector at signed distance `c(1−b²)/(4b)`. The
     closed flag adds the closing segment, which uses the last vertex's bulge. Constant and
     per-vertex widths are ignored with a warning. Spline-frame vertices (flag 16) are skipped. A 3D
     polyline is projected onto XY with a warning. Meshes and polyface meshes are skipped with a
     warning. A polyline whose vertices all coincide is skipped with a warning. When one segment is
     skipped (for example an out-of-range bulge arc), the whole polyline is dropped, not left with
     a gap.
   - **SPLINE:** see 6. A spline that has only fit points (no control points) is **interpolated**:
     a C2 cubic through every fit point with chord-length parameters, the 12/13 end tangents as unit
     directions when present, and natural end conditions otherwise. This comes with a warning,
     because the originating program may have parameterised the curve differently. A spline that
     produces no span (all control points coincide) is skipped with a warning.
   - **INSERT/BLOCK:** see 7.
   - **TEXT / ATTRIB / MTEXT:** `DxfText` items, see 9.
   - **ATTDEF:** ignored, because it is a template.
   - **Everything else** (HATCH, DIMENSION, POINT, SOLID, 3DFACE, …) is skipped with a warning.
6. **Curve approximation.**
   - **Ellipses** are affine images of the unit circle. A piece of angle θ deviates by at most
     σ_max·(2/27)·sin⁶(θ/4)/cos²(θ/4) when the standard 4/3·tan(θ/4) cubic is used, where σ_max is
     the larger semi-axis. The piece count is chosen from that bound, with a 10 % margin. Arcs under
     a non-uniform block scale go through the same path instead of ADR-0005's fixed 90° Beziers, so
     they also meet `tolerance.chordal`.
   - **NURBS:** each non-empty knot span is turned into a (rational) Bezier piece by blossoming,
     which is the de Boor recursion with one argument per level. This works for clamped,
     unclamped and periodic knot vectors.
     - *Non-rational, degree ≤ 3:* converts exactly. Degree 1 gives lines, degree 2 is
       degree-elevated, and degree 3 is copied.
     - *Rational, or degree 4–11:* each piece is approximated by a cubic Hermite (same ends and end
       derivatives). The piece is halved until 23 interior samples lie within `tolerance.chordal`
       of the true curve at the same parameter (an upper bound on the distance at those samples),
       with at most 24 halvings per knot span.
     - *Rejected with a warning:* degree above 11, knot counts that do not match, decreasing knots,
       and non-positive weights.
     - Splines are converted in world coordinates: the control points are transformed (NURBS are
       affine invariant), so the tolerance holds after block scaling. Control points beyond the
       coordinate limit are rejected before approximation.
7. **Blocks.**
   - **Insert transform:** `T(P)·R(rotation)·T(array offset)·S(sx, sy)·T(−base)`, applied after the
     parent's transform. Negative scales mirror, which flips arcs to CW.
   - **MINSERT:** arrays expand cell by cell. The column and row spacing are in the insert's
     rotated, unscaled frame.
   - **Layer "0" entities** inside a block take the layer of the inserting entity, as layer-0 blocks
     do in CAD programs.
   - **Cycles:** a block that is already on the expansion stack is skipped with a warning, so the
     instance that closes a cycle is dropped and the rest is kept.
   - **Depth:** nesting deeper than `maxBlockDepth` (64) is skipped with a warning.
   - **Instance budget:** each MINSERT cell counts against `maxInstances` (1,000,000), which stops
     exponential nesting even when blocks are empty.
   - **Other skips:** an external reference or an INSERT with zero scale is skipped with a warning.
8. **OCS.** The extrusion direction matters for 2D entities (ARC, CIRCLE, LWPOLYLINE, POLYLINE,
   TEXT, INSERT). (0,0,1) is the identity. (0,0,−1) is a mirror in X, because the arbitrary-axis
   algorithm gives OCS X = −world X. Any tilted direction is skipped with a warning. ELLIPSE uses the
   sign of its normal for the direction of its parameter, and SPLINE and LINE are world coordinates.
   Z is dropped.
9. **Text.**
   - **Fields:** string, style, font file (from the STYLE table), height, width factor, insertion
     and alignment points, rotation, horizontal/vertical justification, MTEXT reference width, and a
     mirrored flag.
   - **Under block transforms:** points map exactly. The rotation follows the mapped baseline.
     Height scales by |det|/|L·dir| and the width factor by |L·dir|²/|det|. Mirroring toggles
     `mirrored`.
   - **Strings:**
     - Files from R2007 on (AC1021+), and files without a version, are UTF-8; invalid bytes become
       U+FFFD.
     - Older files are read as Windows-1252, with a warning when `$DWGCODEPAGE` names another code
       page.
     - `\U+XXXX` escapes are decoded.
     - TEXT codes `%%d`, `%%p`, `%%c`, `%%%` and `%%nnn` are decoded, and `%%u`, `%%o` and `%%k`
       are removed.
     - MTEXT formatting is stripped: `\P` becomes a newline, `\S` stacks become `a/b`, the font,
       height and colour groups and the braces are removed.
     - Embedded-object data (group 101 onward) is ignored.
10. **Units.** The first rule that applies sets the scale:
    1. `forcedScale` from the options;
    2. a known nonzero `$INSUNITS` (codes 1–16 and 21);
    3. `$MEASUREMENT` (0 = inches, 1 = mm);
    4. `unitlessScale` (default mm).

    An unknown `$INSUNITS` gives a warning and the later rules apply. The scale is the root
    transform, so every coordinate, radius, text height and tolerance is in mm.
11. **Layers.** Layers come in table order, then layers that entities reference but the table does
    not define. Names are matched case-insensitively. `aci` is |62|, and `rgb` comes from 420 when
    present, otherwise from the ACI palette. A layer is visible when its colour is ≥ 0 and it is
    not frozen. Paper-space entities (67 = 1) are skipped with a warning.
    - *ACI palette:* `aciToRgb` uses the named colours 1–9 and the grey ramp 250–255. The hue rows
      10–249 are generated: 24 hues, 15° apart, with five brightness levels each, in full- and
      half-saturated variants. Rows 10–249 can therefore differ by a few units from tables that
      other programs ship. That is harmless, because export writes 420 whenever the RGB differs
      from the index colour.
12. **Joining is opt-in.** By default every entity is its own contour, and joining belongs to a
    later "Join vectors" tool. With `joinContiguous`, consecutive open contours on the same layer
    are joined when one's end meets the next one's start, or the next one's end (that contour is
    then reversed), within `tolerance.distance`. A chain is closed when it returns to its start.
13. **Budgets.** All of these are options:
    - `maxSpans` (5,000,000) is checked before large allocations: the piece count is computed
      before ellipse pieces are generated.
    - `maxTexts` (1,000,000).
    - `maxInstances` and `maxBlockDepth` (see 7).
    - Coordinates beyond 10¹² mm, and non-finite results, are skipped with a warning. Where a
      large coordinate would also make span generation expensive, the check runs **before** any
      spans are made: the centre and axis vectors of arcs (including bulge arcs) and ellipses,
      spline control points and fit points. So an ELLIPSE with a 10³⁰⁰ axis is a warning, not a
      `ResourceLimit` failure of the whole import. A LINE whose ends overflow to infinity (for
      example inch coordinates near the double limit) is reported as out of range, not as
      zero-length.
    - ARC start/end angles and ELLIPSE start/end parameters beyond 10⁵ rad in magnitude are
      skipped with a warning: from 10¹⁶ a double cannot even tell values 2π apart, and silently
      producing a full curve or degenerate pieces is worse than skipping. Accepted ELLIPSE start
      parameters are reduced modulo 2π before the pieces are generated.

    No recursion depends on input size except block nesting, which is bounded by `maxBlockDepth`.

### Writing
14. **R2000 ASCII (AC1015).** The file has these parts:
    - HEADER: `$ACADVER`, `$DWGCODEPAGE` = ANSI_1252, `$HANDSEED`, `$INSUNITS` = 4 and
      `$MEASUREMENT` = 1.
    - An empty CLASSES section.
    - TABLES: VPORT, LTYPE (ByBlock, ByLayer, Continuous), LAYER, STYLE (Standard), VIEW, UCS,
      APPID (ACAD, CARVETTO), DIMSTYLE and BLOCK_RECORD.
    - BLOCKS: \*Model_Space and \*Paper_Space.
    - ENTITIES.
    - OBJECTS: the root dictionary and ACAD_GROUP.

    Handles are assigned sequentially from 0x20, in output order. Numbers are written with
    `std::to_chars` in the shortest form that reads back exactly (with ".0" appended to integral
    values, and without "-0"). The same input therefore gives byte-identical output.
    - Exports load in ezdxf 1.4.5 with an audit of 0 errors and 0 fixes (director review). They
      have not yet been checked in other CAD programs.
15. **Entity mapping.** A contour is split into runs of line/arc spans and runs of Bezier spans:
    - **Line/arc runs** become **LWPOLYLINE**. Arcs are written as the bulge `tan(sweep/4)` on
      their start vertex. An arc sweeping more than π is split into ⌈|sweep|/π⌉ equal segments, so
      every |bulge| ≤ 1. A reader rebuilds a centre from the chord and the bulge, and for a single
      near-full segment the error grows as 1/(2π − sweep): about 7e-9 mm at a gap of 1e-4 rad and
      3e-4 mm at 2e-9 rad, for ±1000 mm coordinates. With bulges ≤ 1 it stays at rounding level. A
      full circle inside a longer contour is two half-circle segments. A contour that is closed and
      made only of lines and arcs becomes one closed LWPOLYLINE without a repeated vertex.
    - **Bezier runs** become one clamped degree-3 **SPLINE**: its control points are the chained
      Bezier controls, and its interior knots have multiplicity 3, so each Bezier is reproduced
      exactly.
    - **A contour that is a single, exactly full circle** (|sweep| = 2π, ADR-0005) becomes a
      **CIRCLE**. A near-full arc keeps its gap and goes to an LWPOLYLINE.
    - **Zero-length spans** cannot be represented and are dropped: a line whose ends coincide
      exactly, or a Bezier whose four points coincide. A contour made only of such spans is not
      written. `exportDxf` returns only bytes, so this is documented here and in the header rather
      than reported as a warning.
    - **Layer names** are made valid (`<>/\":;?*|=` and control characters become `_`, an empty
      name becomes `Layer`) and made unique case-insensitively with a `_2` suffix. Non-ASCII
      characters are written as `\U+XXXX`.
    - **Colours:** 62 is the layer's `aci` (or the nearest ACI), negated when the layer is hidden.
      420 is written when the RGB differs from the index colour.
16. **Carvetto XDATA for exact round trips.** Every exported entity carries
    `1001 CARVETTO` followed by 1071 values: tag version 1, contour index, piece index, piece
    count and closed flag.
    - A CIRCLE adds a direction value (1071) and the start angle (1040).
    - An LWPOLYLINE with split arcs adds the number S of split arcs (1071). It then gives, per
      arc, the first segment index, the segment count and the direction (three 1071 values), and
      finally, per arc, centre x, centre y, radius, start angle and end angle (five 1040 values).
    - The importer rejoins consecutive top-level pieces with matching tags on the same layer whose
      ends meet. It restores the closed flag and the circle's start angle and direction.
    - It replaces each run of split-arc segments with the original `ArcSpan`, bit for bit, mapped
      by the import transform. The run must have one arc span per segment, and the restored arc must
      trace the same path as those bulge segments: every segment turns the same way, the segment
      sweeps add up to the arc's sweep, and the segment points at parameters 0, ¼, ½, ¾ and 1 lie
      within max(1e-9 mm, 1e-14 × (|centre| + radius)) of the corresponding points of the arc.
      Otherwise the pieces are kept, so tampered or corrupted XDATA (direction flipped, another
      circle through the same end points, centre moved by 1e-6, radius scaled by 1 + 1e-7) can
      never change the geometry the bulges describe beyond that tolerance.
    - Files from other programs are unaffected, and other readers ignore the XDATA and see valid
      separate entities.
    - This is the only place where the importer joins entities without the option.
    - Round trip: lines, Beziers and split arcs come back bit for bit. Other arcs (|bulge| ≤ 1)
      are rebuilt from the bulge: their points come back within rounding level (verifier: worst
      3.3e-11 mm), but the centre and radius *parameters* of a very short arc are ill-conditioned
      (a chord much shorter than the radius fixes the circle poorly) and can differ by more: up to
      1.4e-5 mm for sweeps of 1e-6 to 1e-1 rad. The path is the same; the parameters are not.
      Span counts are preserved, full circles inside contours included.

### Director decisions (2026-10-25)
17. The export API is `exportDxf(layers, contours)`, not `exportDxf(doc)`; the T-0100 card is
    worded accordingly.
18. AC2 "a second export is byte-identical" means exporting the **same input** twice.
    `export(import(export(x)))` must preserve the geometry within 1e-9 mm and every layer name.
    Byte identity is **not** required there, because re-imported arcs are rebuilt from bulges.

## Consequences
- T-0122 (CLI) and the app map `DxfDrawing` onto the document: layers to layers, contours to
  vector objects, and texts to text objects once T-0130 renders them.
- Export takes layers and contours, not a `Document`, so callers decide what "visible vectors"
  means (feature inventory §2: with nothing selected, all visible vectors are exported).
- Hatches, dimensions, 3D entities and DWG remain out of scope; they produce warnings.
