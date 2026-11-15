# ADR-0095: per-component and per-level appearance in the Model view

Status: accepted (T-0305; follows ADR-0077 "Appearance is a view setting", which left this to a later card)

## Decisions
- **Data.** `relief::Appearance { kind, material, rgb }` (`relief/relief.hpp`) is a member of `relief::Component`
  and `relief::Level` and of the document's `ReliefLevel`. Kinds: Same as Parent, Solid Colour (`rgb` 0xRRGGBB),
  Material (Wood, Metal, Stone, Plastic) and Colour from Children. `relief::compose` ignores it; `Document::reliefModel`
  copies it. Commands `SetReliefComponentAppearanceCommand` / `SetReliefLevelAppearanceCommand` (no-op for an unknown
  id, an unchanged value or an invalid appearance; edits of one target coalesce like the other property commands);
  `checkIntegrity` reports a bad level appearance (`BadReliefAppearance`) and a bad component one as
  `BadReliefComponent`; `reliefAppearanceProblem` is the single validator (commands, loader, saver, integrity).
- **The view material stays.** The root of the chain is the Model view's material choice of ADR-0077
  (`ModelRenderOptions::material`). Parent chain: component -> level -> view.
- **Format 1.12** (`kCarvFormatMinorAppearance`). Level and component records get an optional `appearance` object,
  written only when the kind is not Same as Parent: `{"kind": "solid"|"material"|"children", "rgb": N}` for solid,
  `"material": "wood"|"metal"|"stone"|"plastic"` for material, nothing else for children. The loader is strict (one
  spelling per appearance, so save -> load -> save is byte-identical): `same_as_parent` is not stored, `rgb` is an
  integer 0..16777215 and present exactly for solid, `material` exactly for material, a component may not be
  `children`. The minor is 12 only when some level or component has an appearance, so every other file keeps its
  bytes and 1.9-1.11 files load unchanged. An older 1.x reader refuses a 1.12 file at the version check rather than
  silently dropping the colours (same policy as ADR-0079); unknown keys inside an appearance record are skipped.
- **Rendering.** `ModelSurface::colours` (per surface cell, 0xFFRRGGBB, 0 = the view's material) is filled by
  `composeModelSurface` through `composeModelColours`, which replays the composition per cell with the same
  `relief::combine` rules and tracks which operand supplies the surface: Add the upper operand, Merge High/Low the
  winner, Subtract and Multiply the one below. It runs only when some appearance on the sheet is not Same as Parent
  (otherwise `colours` is empty and nothing changes) and is skipped when cells x visible components exceeds
  `kMaxColourWork` (2^27; the view then falls back to the view material). The renderer colours each surface quad with
  the colour of its lower-left cell (flat per quad, lighting as before); `ModelRenderOptions::useAppearance=false`
  shows the view material alone.

## Product readings (for owner review)
1. **Colour from Children** is offered for levels only (a component has no children; the command, loader and form
   refuse it). Reading: the level lets its components show their own colours, and a component that is Same as Parent
   takes the **mean colour of the level's visible components that have an own colour**; no such component = the
   view material. The inventory only names the option, not its semantics.
2. **Same as Parent** of a component takes the level's colour (Solid/Material; for Colour from Children the mean above);
   a component's own appearance always wins over its level's.
3. **Which colour a cell gets** is the operand that supplies the surface height (see above), not a blend. A cell where
   a lower level shows through a Subtract keeps the lower colour.
4. **Level effects.** A clip makes cells transparent (no colour); a mirrored copy keeps the colour of the cell it was
   copied from, i.e. the colour map is not mirrored. The map is computed on the surface grid, so at a coarse stride a
   colour edge can move by up to one surface cell.
5. **Bake and re-created components** (bake of level effects, Create Component From Toolpath Preview, imports) produce
   components with Same as Parent; baking drops the colours of the components it merges. Duplicate keeps the appearance.
6. **Materials are flat base colours** shaded by the light (the existing Wood/Metal/Stone/Plastic colours); no textures.
7. **Colour input** in the form is six hex digits (`#RRGGBB`); there is no colour picker dialog yet.

## Tests and mutants
`relief_appearance_test` (commands, integrity, .carv 1.12 round trip, byte stability, minor only when used, hostile
records), `relief_tree_test` / `model_view_test` (viewmodel actions and form, colour map per combine mode and level,
Colour from Children, hidden child, view-material fallback, pixels), and the Qt self-test (form, menu entries, red pixel).
