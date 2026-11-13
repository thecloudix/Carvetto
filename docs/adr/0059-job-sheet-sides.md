# ADR-0059: Job sheet: per-side sections for double-sided jobs

Status: accepted (T-0278)

## Context
ADR-0057 left the per-sheet/per-side split of the job sheet to the double-sided work (T-0243,
T-0261). Toolpath records now name their sheet, and each side posts with its own Z zero.

## Decisions
- `JobSheet::sides` holds one `JobSheetSide` per sheet (sheet order, at most `kJobSheetMaxSides` = 8;
  the rest are counted in `sidesOmitted`). It is filled only when the job type is double-sided, so
  single-sided and rotary sheets render exactly as before (checked byte for byte against the
  previous renderer's output).
- The whole-job block (summary, tools, list, layout) stays first; the "Sides" section follows it.
  Each side has its own setup (size, Z zero, XY datum), its own tool table, toolpath list and
  layout thumbnail, with the existing per-block caps applied per side, so the sheet stays bounded
  (sides x the single-block worst case).
- The zero reference of a side is `document::materialTopZ(zZero, zeroOffSameSide, thickness,
  side)`: the machine Z of the top face of the stock while that side is cut, the same value Save
  Toolpaths shifts the moves by. The job-level "zero off same side" flag is printed too.
- A toolpath belongs to the sheet named by `ToolpathRecord::sheet`; a record without one (made
  before the job became double-sided) is listed on the first sheet so that the sides add up to the
  job (Save Toolpaths posts such a record unshifted, with no side).
  A record naming a sheet that no longer exists appears only in the whole-job block.
- A side's time is the sum of its included toolpaths' times from the job summary, so the sides add up
  to the job total. PDF output carries the same HTML (without SVG).
- A direct print dialog stays out (PrintSupport not added), as in ADR-0057.
