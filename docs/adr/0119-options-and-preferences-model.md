# ADR-0119: Options dialog and the preferences model

Date: 2026-11-17 · Status: proposed (T-0348) · Builds on ADR-0014 (Qt-free viewmodel)

## Context
Preferences were scattered: window geometry and recent files in `MainWindow`, clipart folders in `ClipartPanel`,
brush presets in the sculpt panel, gadget shortcuts in `GadgetsMenu`, the active machine in the Toolpaths dock, and
display colours / simulation quality as hard-coded constants. The card asks for one settings model and one Options
dialog; the feature inventory does not say which of these are "preferences". These are my readings, for owner review.

## Decision
1. **`viewmodel::Preferences`** (Qt-free) is the single value holding every *user preference*, with defaults;
   `validate` refuses bad values; `serialize` / `deserialize` map it to a versioned flat key list ("version" first,
   keys `group/name`); `PreferencesModel` is the editable copy (`apply` refuses invalid values and changes nothing,
   `restoreDefaults`, `load` / `save` through a `PreferencesStore`). Only the app knows QSettings
   (`QSettingsPreferencesStore`, group `preferences`).
2. **Loading never fails.** A missing key keeps its default; an unparsable or out-of-range stored value falls back to
   its own default (the other values survive) and is reported. A newer `version` than the build knows is reported and
   known keys are still read. `migrate` is the hook for older versions; versions 0 and 1 share their keys so it is
   empty today.
3. **Preference vs. application state.** Preferences are what the Options dialog edits. Data the app maintains for the
   user stays in its own stores and is *not* a preference: window geometry/dock layout, recent files, the active
   machine, sculpt brush presets, the clipart library (folders, favourites) and gadget shortcut overrides (edited in
   their own dialogs). They remain QSettings keys outside the `preferences/` group; this is the audit boundary.
4. **Audit list** (every value of the model, its consumer today):
   - `general/units` default units of File > New (applied), `general/language` (placeholder; consumed by T-0350).
   - `editing/undoDepth` (applied to every document through `UndoStack::setLimit`), `editing/autosaveEnabled` /
     `autosaveMinutes` (stored; consumed by T-0357).
   - `grid/show`, `grid/spacingMm`, `grid/snapEnabled`, `grid/snapTolerancePx` (stored; the canvas has no snap or grid
     UI yet, the snapping maths already takes these values).
   - `colours/canvasBackground`, `colours/materialFill`, `display/antialiasing` (canvas), `display/showRulers`
     (rulers): applied live.
   - `folders/open`, `save`, `export`: start folder of the main window's Open / Save As / Import / Export dialogs.
   - `tools/databasePath`: overrides the per-user tool setup file location (applied when persisting).
   - `simulation/quality`: the Preview panel's starting quality, followed when the preference changes.
5. **Validation ranges**: undo steps 10..10000, autosave 1..120 min, grid 0.01..1000 mm, snap distance 2..50 px, the two
   colours must differ, language `system` or `en`, folders must be valid paths and, in the dialog, exist. Existence is
   not checked when loading, so an unplugged drive does not silently reset a stored folder.
6. **Dialog**: Setup > Options, one page per category, one editor per field (object name `pref:<key>`), numbers are
   plain text fields so a bad entry can be refused with a message rather than silently clamped; OK stays open while
   anything is invalid; Restore Defaults resets the dialog (Cancel still discards it).
7. `UndoStack::setLimit` (core) was added so a changed depth takes effect on open documents.

## Alternatives
Putting window layout and library data into the model too: rejected, they are not user-edited settings and have their
own lifecycles. Clamping spin boxes instead of refusing: rejected, the card asks for refusal.
