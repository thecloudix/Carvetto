# ADR-0043: Toolpath tiling (T-0242)

Status: accepted

## Model
`core/toolpath/tiling.hpp` splits toolpaths larger than the machine work area into tiles.
- Strategies (inventory "Toolpath Tiling Manager"): Individual tiles (width x height grid),
  Feed-through in X (strips `width` wide, full Y extent), Feed-through in Y.
- Tile rectangles start at the job extent's minimum corner and advance by `size - overlap`; the
  last tile is trimmed to the extent. "Machine smallest tile first" orders tiles by ascending
  area (stable).
- Cut moves are clipped to the tile rectangle: lines (Z interpolated) and arcs (split at the
  edges at exact circle/line intersections, helix Z interpolated, centre kept). Dwells stay in the
  tile containing them. Rapids of the source are dropped; pieces are relinked with
  retract to safe Z, rapid, plunge (safe Z = highest Z of the toolpaths unless given).
- Pieces shorter than 1e-6 mm are dropped (an arc that short would read as a full circle).

## Overlap semantics (interpretation)
The inventory does not say what the overlap does. Neighbouring tile rectangles overlap by
`overlap` mm and each tile cuts everything inside its rectangle, so a cut inside an overlap is in
both tiles (re-cutting an already cut path is harmless and guarantees full coverage). Moves
running exactly along a shared edge are likewise in both tiles. Tiles are always at most the
requested size, so they fit a work area of that size.

## Coordinates
By default (`originalPosition` false) a tile's minimum corner becomes X0 Y0 (the operator
re-positions the stock per tile); "Draw toolpaths in original position" keeps job coordinates.

## Save path
`SaveOptions::tiling` (Output Tiled Toolpaths) makes `prepareSave` replace the records by tile
records, tile by tile, named "<toolpath> (tile N)", so the existing file modes work unchanged
(one file per toolpath gives one file per tile and toolpath). A tile larger than the optional
work area fails the job. Keep-out zones are refused together with tile coordinates (zones are in
job coordinates); they still work with the original position.

## Not done
The dialog's Tile Toolpaths checkbox and the Tiling Manager panel are a UI follow-up card.
