# ADR-0091: Moulding, Project Toolpath onto Model, 3D Tabs and Create Component From Toolpath Preview in the app

Status: accepted (T-0311; follows ADR-0085 "Remaining", ADR-0086 and ADR-0073)

## Decisions
- **Two new operations.** `ToolpathOperation::Moulding` and `Project3d` extend the `ToolpathParams` variant with the
  core's `MouldingParams` / `Project3dParams` (appended: the enum order is the variant order). They are stored in
  `job.json` as `moulding` / `project_3d` (strict key lists, finite numbers, enums by name). Old readers refuse the new
  names like any unknown operation. Geometry is not part of a record: the moulding's `profile`, `rails` and
  `clearTool` are never stored.
- **Moulding reads the selection.** The record's selection holds the profile and the rails. The profile is the object
  named by `MouldingParams::profileObject` (an app-level id the core ignores; absent in the file = 0), else the first
  open vector in selection order; every other selected vector is a rail (open or closed). Fewer than two vectors, a
  profile that is not selected or that is closed are refused with a message. Staleness is the ordinary vector
  fingerprint (geometry and order of the resolved vectors). The form's stock fields default from the job like 3D
  Rough; the section's Z offset stays 0 (the section's heights are the drawing's own; the user types the offset).
- **Project Toolpath onto Model and 3D Tabs are one operation.** `Project3d` carries `Project3dParams` including the
  tabs; the panel offers two entries: "Project onto Model" (tabs off) and "3D Tabs" (tabs on, ramped), both
  `ToolpathSession::newProjectForm(source, tabs)`, which presets the source from the toolpath selected in the list, the
  tool from the source's tool when it suits, the name (`<source> (3D)` / `(3D tabs)`) and the placement from the model
  (`zOffset`, `outsideSurface` = stock top, `stockBottom`). Tabs can be switched on or off in the form either way.
- **The source toolpath is an input.** `Project3dParams::sourceToolpath` (app-level id, absent = 0) names the record;
  `ToolpathJob::source` holds its shared immutable moves and tool snapshot (`attachSource`, so `run()` stays
  document-free). `inputsFingerprint` of a projection = the model fingerprint plus a hash of the source's moves
  (`sourceFingerprint`), so recalculating, editing or deleting the source, or editing the model, marks the projection
  "needs recalculation"; a missing source refuses with a message. A projection cannot be its own source.
  `ToolpathRecord::safeZ()` of a projection is the highest Z of its moves (the retract heights of its source).
- **Overlay.** `modelPathsFromToolpaths` also draws projections (machine Z minus the record's `zOffset`); a moulding is
  not drawn on the model (its Z is the section's, not the model's).
- **Create Component From Toolpath Preview** is `viewmodel::createComponentFromToolpathPreview` (Qt-free): simulate
  all enabled, computed toolpaths with the existing simulator (`makeSimulationSetup`; cell size from the quality,
  capped by `maxCells`; cancel polled per 4096 moves), `simulate::componentFromPreview`, then add the component to the
  top level of the active sheet (a level is added when there is none) and select it, in one undo step. File menu entry
  "Create Component From Toolpath Preview" (the inventory's Model menu has no counterpart in this window yet).
  Readings for owner review: (1) the model height is the simulated Z minus the stock bottom by default (the carved
  block stands on height 0 and the uncut top is the thickness), `zOffset` overrides it; (2) the reference says only
  "Shift or Ctrl changes the result": Shift keeps the cut cells only (the rest transparent), Ctrl combines by Subtract
  instead of Add; (3) the simulation runs on the UI thread behind a wait cursor (a worker is a follow-up).

## Not done (left in T-0311's follow-ups)
- **Rest-tool lists and prior toolpaths for 3D Rough / 3D Finish, and the Moulding larger-area clearance tool.** They
  need several records per calculation (one per tool) and the session's group/role machinery, which infers groups of a
  loaded file from the tool *type* (clearance vs carving) for V-carve; a 3D rest tool is an ordinary end mill, so the
  inference cannot tell a rest group from separate toolpaths. Doing it right means storing the group in the record
  (a .carv minor-version bump) and is its own card. The core already supports them (`Rough3dTool` lists,
  `Finish3dParams::priors`, `MouldingParams::clearTool`); the records keep `restTolerance` and never store `priors`.
- A moulding's profile from a closed vector or a section that overhangs are the core's limits (ADR-0086).

Update (T-0314, ADR-0094): rest-tool lists, prior toolpaths and the Moulding clearance tool are done; the group is stored in the record.
