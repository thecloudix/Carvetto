# ADR-0098: Rotary job: Job Setup, Material Setup, sheets and the model band

Date: 2026-11-16 · Status: proposed (T-0319)

## Context
T-0319 asks to audit what exists for rotary jobs and complete them: Job Setup rotary page, Material
Setup rotary (diameter, Gap Outside / Gap Inside Model), Edit Sheet for rotary, `.carv` round trip, and a
self-test. The behaviour described in `docs/reference/feature-inventory.md` (Job Setup, Material Setup,
Sheets) is the only source.

## Audit: what existed before this card
- `core/document/job.hpp`: `JobType::Rotary`, `RotaryAxis {X, Y}`, `ZZero::CylinderAxis`, `Job::diameterMm`,
  `rotaryAxis`, `flipDesign`, and `Job::rotary(length, diameter, units, axis, zero)` which derives the flat
  design area (length x pi x diameter, `sizeMm.z` = diameter). `Job::isValid` checked diameter > 0.
- `carv_io`: job records already stored `diameter_mm`, `rotary_axis`, `flip_design` (all required keys).
- Viewmodel `NewJobForm`/`validateNewJob`/`formFromJob`: type, length, diameter, axis, Z-zero validation, but
  **no `flipDesign`** (the field was never reachable from the UI and a Job Setup edit would have reset it).
- `NewJobDialog`: Rotary type, length, diameter, axis and a "Cylinder axis" Z-zero choice, but no Flip Design,
  and switching the type left an invalid Z-zero selected.
- `applyJobSetup` only replaced the job: **sheets did not follow** (a rotary job's sheet kept the old size).
- `setSheetMaterial` edited width/height/thickness freely: a rotary sheet could be made inconsistent with the job.
- `materialTopZ` treated `CylinderAxis` as the surface (0), so a cylinder-axis sheet had its top at 0, not the radius.
- No Material Setup of any kind (no gaps, no model band), no rotary self-test.
- Post `ROTARY_WRAP_X/Y` keys are parsed; wrapping itself is T-0320 and stays out of scope.

## Decisions
1. **Core model.** `Job` gains `gapOutsideMm` and `gapInsideMm` (radial, >= 0) and read-only helpers:
   `lengthMm()`, `circumferenceMm()`, `radiusMm()`, `modelThicknessMm()` (= radius - gaps),
   `rotarySurfaceZ()` (0 for a surface zero, the radius for an axis zero), `rotaryModelTopZ()` (surface minus
   outside gap), `rotaryModelBottomZ()`, and `setRotaryMm(length, diameter)` which re-derives the flat design
   area for the job's axis. `Job::rotary` takes a trailing `flip` argument. `isValid` additionally needs
   non-negative gaps leaving a positive model thickness. Units stay mm/radians in core.
2. **Readings I had to guess (for owner review).**
   - "Gap Outside / Gap Inside Model": the model occupies a radial band; Gap Outside is the distance from the
     cylinder surface down to the top of the model, Gap Inside from the bottom of the model to the axis, so
     the Model Thickness shown read-only is radius - outside - inside. (Flat jobs: gap above/below, thickness.)
   - Z-zero "cylinder surface" is `ZZero::MaterialSurface` for a rotary job; "centre" is `ZZero::CylinderAxis`.
     Selecting Rotary in the form switches the Z-zero to the recommended cylinder axis; selecting a flat type
     switches it back to the material surface.
   - "Flip Design" is stored and round-tripped; what it does to toolpaths/wrapping is T-0320.
   - Z of the model band: with an axis zero the surface is at +radius; with a surface zero at 0, the band
     below it (negative Z). `materialTopZ(CylinderAxis, ..., diameter, ...)` now returns the radius.
   - Job Setup is global to the cylinder: applying it gives **every sheet** the job's length, diameter,
     circumference and Z-zero (one undo step). Material Setup changes the diameter, gaps and Z-zero of the job
     and all sheets but each sheet **keeps its own length**.
   - Job Setup never touches the gaps (they belong to Material Setup); it refuses a diameter too small for them.
   - Edit Sheet for a rotary sheet edits the **length** (along the cylinder), the Z-zero (surface or axis) and
     the datum. Circumference and diameter are read-only there (the diameter is one value for the cylinder, set
     in Material Setup / Job Setup); width/height/thickness edits and the machine-bed zero are refused. For a
     flat job a length edit and the cylinder-axis zero are refused.
3. **`.carv` format 1.14.** `job.gap_outside_mm` and `job.gap_inside_mm` are optional keys, written only when
   either is non-zero; the minor becomes 1.14 only then (`kCarvFormatMinorRotaryGaps`), so existing files and
   rotary jobs without gaps are byte-identical. The loader requires them finite and >= 0, only on a rotary job,
   and leaving a positive model thickness; the 11 existing job keys stay required. Another card
   also claimed 1.13 (T-0323, merged first), so this card took 1.14.
4. **Viewmodel/app.** `rotary_material.{hpp,cpp}` (Qt-free): `RotaryMaterialForm`, `validateRotaryMaterial`,
   `applyRotaryMaterial`, `resizeRotarySheets`. `NewJobForm::flipDesign`. App: Flip Design in the Job Setup
   dialog, a Material Setup (rotary) dialog and Setup menu action (enabled for rotary jobs only), rotary labels
   in the Sheets panel, and `rotarySelfTest` (Job Setup, Material Setup, Edit Sheet, save and reload).

## Not covered here
Wrapped output, the Auto-Wrapping View, simulation of rotary jobs (T-0320), the Wrapping level effect (T-0321),
and a flat-job Material Setup page (gap above/below, Set Model Height), which no card has yet.
