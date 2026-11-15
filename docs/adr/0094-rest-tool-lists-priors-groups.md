# ADR-0094: rest-tool lists, prior toolpaths and the Moulding clearance tool; toolpath groups stored in the record

Status: accepted (T-0314; completes "Not done" of ADR-0091 and "Remaining" of ADR-0085)

## Context
The cores already take several tools (`rough3dToolpath` / `finish3dToolpath` tool lists, `Finish3dParams::priors`,
`MouldingParams::clearTool`) and return one toolpath per tool. The session ties the toolpaths of one calculation
together as a group (V-carve with clearance tools) but, for a file that was loaded, infers the group from tool
types (a clearance end mill followed by a carving V-bit). A 3D rest tool is an ordinary end mill, so a rest group
cannot be told from separate toolpaths that way.

## Decisions
- **The tool list is "earlier tools, then the operation's own tool".** `ToolpathDefinition::clearanceTools` are the
  earlier tools (a V-carve's clearance end mills; a 3D rough/finish's rest tools in order; a moulding's one
  larger-area clearance tool), `tool` is the last and finest one: the core's tool order, which also is its result
  order. `hasClearanceTools` now also holds for 3D Rough, 3D Finish and Moulding (`hasRestTools` names those three);
  a moulding accepts at most one earlier tool (the form refuses more, the job refuses a definition with more).
  `rest_tolerance` of the three operations is a form field now ("Rest tolerance", 0..100 mm; it was only stored).
- **The group is stored in the record.** `ToolpathRecord::group` (`ToolGroup`: `id`, `role`, `tools`) holds the group
  id shared by the records of one calculation, the whole tool list (every member, so a member whose neighbours had
  nothing to cut still knows the list) and the index of its own tool in it. It is saved as an optional `group`
  object of the job.json toolpath record (strict key list; `id` nonzero, 1..64 tools, `role` below the tool count;
  the loader refuses anything else, as do `AddToolpathCommand`/`SetToolpathCommand` and `saveCarv` via
  `groupProblem`). Only rest-tool groups (3D Rough, 3D Finish, Moulding with earlier tools) store it: V-carve groups
  keep being inferred from tool types on load (their records and files are unchanged; the inference code only runs
  for the V-carve operations now, and skips records that carry a group).
- **Session.** `adopt` rebuilds the baseline of a stored-group record from the record: `tool` = last list entry,
  `clearanceTools` = the others, role -1 for the last tool, else its index. The definition of any member is
  therefore the whole calculation, recalculating one member recalculates the group (one undo step, same ids), and
  rename/duplicate/delete treat the group as one (duplicate gives the copy a new group id). A group missing its
  own-tool record (the last tool had nothing to cut, common for rest tools) stays editable and recalculable: the
  definition comes from any member (`Baseline::stored`); a V-carve still reports its lost carving toolpath. The
  session's group counter is raised above every stored id on load. A moulding group is named as the core names it
  ("name [Clear]" and "name").
- **Prior toolpaths of a 3D finish** are `Finish3dParams::priorToolpaths`, app-level record ids the core ignores
  (like `Project3dParams::sourceToolpath`, ADR-0091), saved as `prior_toolpaths` (array of ids, at most 256, absent
  = none). `attachPriors` copies each record's shared moves with its tool resolved from the record's `ToolSource`
  into `ToolpathJob::priors`; the run builds the core's `Finish3dPrior` list from them. Refused with a message: a
  missing or uncalculated prior, a prior whose tool is gone, a repeated id, a prior that is one of the toolpaths the
  job replaces. `inputsFingerprint` of a finish adds the moves hash of every prior (`sourceFingerprint`), so changing
  a prior marks the finish "needs recalculation". The form edits the list with `setPriorToolpaths`; the dialog has a
  comma separated id line (a picker from the list of toolpaths is a follow-up).
- **Simulation** needed nothing: `makeSimulationSetup` already takes each enabled record in job order with its own
  tool shape, which is what rest machining needs (the rest toolpath was planned against the stock the earlier
  tools left); a test checks the order and the shapes.
- **Format 1.11** (`kCarvFormatMinorToolGroups`): written only when a record has a group or a finish lists priors,
  otherwise the version is as before. A 1.10 reader keeps `group` as an unknown record key and the priors as an
  unknown params key (ADR-0013 forward compatibility), and writes them back; the loader strictness above applies to
  1.11 content. Files made before this card load with no groups (what they were) and no priors.

## Not done
- Picking priors and rest tools from the toolpath list instead of typing ids; "use these records as priors" from a
  rest group in one gesture (the ids of its members are typed).
- Rest-tool shank/flute diagnostics (`Rough3dTool::fluteLength` is passed empty).
- V-carve groups are not stored (not needed: their tool types tell the group; a later card can migrate them).
