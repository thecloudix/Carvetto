# ADR-0048: Double-sided UI, toolpath sheet link and per-side output (T-0261)

Status: accepted. Builds on ADR-0039 (T-0243).

## Toolpath records link to their sheet
`ToolpathRecord` gains `SheetId sheet` (null = no side). A toolpath calculation in a double-sided job
stamps the active sheet; single-sided jobs keep it null. Selections (`all`, layers) are not per sheet
(layers are shared), so the sheet cannot be derived from them once objects exist on both sides; the link
must be stored. Recalculating a linked toolpath is refused unless its sheet is active (the vectors it
machines are only resolved on the active sheet).

## Persistence: format 1.7
`job.json` toolpath records get an optional `sheet` (entity id), written ONLY for records whose sheet
exists. The writer's minor is the highest one any used feature needs: 1.7 only when such a record exists
(`kCarvFormatMinorToolpathSheet`), so every document without linked records keeps its previous minor and
bytes. A link to a deleted sheet is written unlinked; the loader refuses a link to a sheet the file does
not have. Older readers keep the unknown `sheet` key in their extras (the loader only checks the major).

## Output (Save Toolpaths)
`SaveSource`/`SaveJob` carry `sides` (sheet, side, `sheetTopZ`, thickness) for double-sided jobs. Records
linked to a side are posted with `document::shiftMovesZ(moves, topZ)`, the post's safe Z/home and
material Z range follow, and a file never mixes sides: one file per side in "single file" mode, groups
split by side in "per tool" mode, and names get `_front` / `_back`. Records without a side are posted as
before. The Z rule itself stays in `materialTopZ` (ADR-0039 "Z zero"). Note that a single-sided job with
machine-bed zero still posts unshifted moves (unchanged); only side-linked records are shifted.

## Copy / Move to Other Side
Objects are mirrored about the centre line of the partner sheet's material (`flipPoint`), the mapping the
multi-sided view uses, in one undo step. Toolpaths: the vectors a toolpath machines are copied mirrored,
the new record sits on the partner sheet over the copies with its moves mirrored (`flipMoves`: X/Y of
end points and arc centres; ArcCW and ArcCCW swap) and is named "<name> (back|front)". "Move" deletes the
original record; the original vectors stay on their side.

## Switching back
`makeSingleSided` sets the job type to single and unpairs the sheets; the back sheet is kept as an
ordinary sheet with its drawing (nothing is deleted).
