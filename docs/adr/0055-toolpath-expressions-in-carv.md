# ADR-0055: Toolpath calculation expressions in `.carv`; background variable recalculation (T-0271)

Date: 2026-11-13 · Status: proposed (extends ADR-0003 and ADR-0013; follows T-0257)

## Context
T-0257 kept the `=...` texts of a toolpath's calculation boxes in session memory only, so the link
between a toolpath and the document variables it names (recalculation, delete protection) was lost on
save/load, and recalculating after a variable edit blocked the UI thread.

## Decision
1. `ToolpathRecord::expressions` (`field id -> text`, ordered) is part of the record and of its
   equality, so it follows undo/redo and duplication. `ToolpathDefinition::expressions` carries it
   from the form to the calculation to the record. The session no longer keeps a separate map.
2. `.carv` format minor 1.8: an optional `expressions` object (string values) per toolpath record in
   `job.json`, written only for a record that has one; the minor is 1.8 only then (highest-required
   scheme, other documents are byte-identical). A 1.7 or older reader keeps the unknown key verbatim
   (record unknown keys are preserved) and so loses nothing. Caps: 64 entries, 4096 bytes each; a
   non-string, duplicate or oversized entry is a load error.
3. A feet postfix (`'`) is 304.8 in millimetre mode and 12 in inch mode (`ExpressionContext::feet`).
4. Chamfer angle, width and cut depth are linked: a calculation in width or cut depth replaces the
   other's, and a kept cut-depth calculation is re-evaluated when the angle changes (own box or tool).
5. After a variable edit the dependent toolpaths are recalculated in the background through the
   existing calculation runner (`VariablesModel::setDeferRecalculation`,
   `ToolpathSession::prepareRecalculateBound`, `ToolpathsPanel::recalculateBound`). The variable edit
   and the recalculation are therefore two undo steps (undo twice) instead of one: holding an undo
   macro open across an asynchronous run would let other edits fall into it. Further variable edits
   are refused while the recalculation runs. The synchronous one-step path stays for headless use.
6. "Insert New Document Variable" is a shared right-click entry (`attachVariableMenu`) on the
   numeric boxes of the app, not only the toolpath forms.

## Consequences
`{Variable}` substitution in text and data-merge fields (card item 6) is not part of this change: it
awaits an owner decision.
