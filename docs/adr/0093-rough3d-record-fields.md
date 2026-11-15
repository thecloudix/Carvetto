# ADR-0093: 3D rough record persists the T-0304 fields

Status: accepted (T-0315; amends ADR-0085 and ADR-0082)

## Decisions
- The `rough_3d` params record in `job.json` gains `clearing` (`raster`/`offset`), `profile` (`none`/`first`/`last`),
  `order` (`level_by_level`/`depth_first`) and `avoid_machined` (bool). They are in the strict key list and always
  written, so save -> load -> save is byte-identical and every field round-trips (T-0304 had left them out, so a
  saved job silently reverted to the defaults).
- A key absent from the file keeps the old default (`raster`, `none`, `level_by_level`, false): files written before
  this change load with exactly the behaviour they had.
- No minor-version bump. 3D records were already outside the minor ladder (ADR-0085), and a bump would not help an
  older reader, which refuses an unknown key in a strict record either way.
