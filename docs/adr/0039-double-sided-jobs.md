# ADR-0039: Double-sided jobs (T-0243)

Status: accepted

## Model
- `Job` already carries `type = DoubleSided`, `flipDirection` and `zeroOffSameSide` (T-0235).
- A `Sheet` gets `side` (`Front`/`Back`) and `partner` (the sheet of the other face). Both faces are
  ordinary sheets with their own objects, size and toolpath links; a pair names each other.
- `document/double_sided.hpp` holds the pure maths: `flipPoint`/`flipBox` mirror about the material
  centre line (horizontal flip negates X, vertical negates Y), and `materialTopZ` gives the machine Z
  of the stock top per side.

## Z zero (interpretation)
Without "Zero off same side" each side zeroes on its own convention (surface: cut face = 0; bed: bed
face = 0, cut face = thickness). With it, both sides share the physical face the front side used:
after the flip that face is the bed (surface zero) or the cut face (bed zero). The inventory does not
define this precisely; the rule is isolated in `materialTopZ` so it can be changed in one place.
Toolpaths are computed with the cut face at Z = 0; `shiftMovesZ(moves, sheetTopZ)` yields the output.

## Persistence and format version
The sheet record gains optional `side` (`"back"`, written only for back sheets) and `partner` (id,
written only when paired). Loading checks that pairs are mutual and front/back. The format minor
is **1.5** (`kCarvFormatMinorDoubleSided`), written ONLY when the document has a back or paired
sheet; the writer takes the highest minor any used feature requires (so keep-out zones plus paired
sheets is 1.5, dimensions plus paired sheets is 1.5, and documents with neither keep their previous
minors, byte-identical). The bump makes an older build refuse the file as too new instead of loading
a paired sheet as two independent sheets and later stranding it. `kCarvFormatMinor` is 5 for the
loader's too-new check.
