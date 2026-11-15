# ADR-0085: 3D Rough and 3D Finish in the app (toolpath records, model input, Model view overlay)

Status: accepted (T-0303, first part: rough and finish)

## Decisions
- **Two new operations.** `ToolpathOperation::Rough3d` and `Finish3d` extend the `ToolpathParams` variant with
  the core's `Rough3dParams` / `Finish3dParams`. They are stored in `job.json` as `rough_3d` / `finish_3d`
  (strict key lists, finite numbers only, enums by name). `vectors` (the boundary polylines) and `priors`
  are not part of a record: the vectors come from the record's selection and are polygonised when the job
  runs; priors are not offered yet. Old readers refuse the new operation names (the loader already rejects
  unknown operations); no format version bump was needed beyond the existing minor-version tolerance.
- **The model is an input of the job.** `ToolpathJob::model` holds the visible relief model of the active
  sheet composed at the finest cell size (`unionGrid` + `compose`), copied in `prepare` so `run()` stays
  document-free and worker-ready. `inputsFingerprint` of a 3D toolpath is a hash of the visible levels
  (modes, base heights, effects, component properties and every height value) plus the vectors for the
  Vectors boundary, so editing the model (and undoing the edit) moves the toolpath between "needs
  recalculation" and "current" like a moved vector does.
- **Placement defaults.** A new 3D form takes the stock from the job (`Z zero` at the material surface:
  top 0, bottom -thickness; machine bed: thickness and 0), the safe Z 5 mm above the stock top and a
  model Z offset that puts the highest point of the model at the stock top. All are ordinary form fields.
- **One tool per toolpath.** Rest machining (later tools in the list) is not wired: the session's
  group/role machinery is the V-carve's, and extending it is its own change (see Remaining).
- **Model view overlay.** `renderModel` takes `ModelRenderOptions::paths` (model-space polylines) and draws
  them as depth-tested one-pixel lines (bias 0.4 % of the model radius so paths lying on the surface
  show). `modelPathsFromToolpaths` turns the cutting moves (Feed, Plunge, arcs) of 3D records into paths
  (machine Z minus the record's Z offset), thinning each toolpath to at most 200 000 points.
- **Simulation and posting** need nothing new: records are ordinary moves, the simulator takes the tool
  shape from the tool database and Save Toolpaths posts them like any other record.
- **Moulding extension point.** A new operation is one enum value in `ToolpathOperation`, one params table in
  `toolpath_form.cpp`, one `usesModel`-style predicate and one branch in `ToolpathJob::run`; nothing else in
  the session depends on the operation list.

## Remaining (not in this change)
Project Toolpath onto Model and 3D Tabs entries (core: `project3d.hpp`), Create Component From Toolpath
Preview (core: `simulate/preview_component.hpp`), rest-tool lists and prior toolpaths for 3D rough/finish,
the Moulding toolpath (T-0307), vector-boundary offsets beyond what the core offers.

Update (T-0314, ADR-0094): rest-tool lists and prior toolpaths for 3D rough/finish are done.
