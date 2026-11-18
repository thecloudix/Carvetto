# ADR-0129: Localization sweep conventions

Status: accepted (T-0351; extends ADR-0123)

- Every user-visible string in `app/src` goes through `QCoreApplication::translate("Class", "text")`; the
  context is the class (or file) that shows it. `tools/tr_literals_baseline.txt` is empty and stays empty.
- Strings kept in tables or passed through helpers (`const char*` labels, `addEdit`, `button`, `makeDock`)
  are marked with `QT_TRANSLATE_NOOP("Class", "text")` at the definition and translated with
  `QCoreApplication::translate("Class", text)` where the widget is built, so lupdate extracts them.
- Widgets that are found by a name derived from their label (Modelling and Sculpt panels) keep the English
  source text as the object name; only the visible text is translated. Stored values (combo choices,
  presets) are kept in item data, never read back from the translated text.
- Text produced by the Qt-free view model is English std::string (often with numbers spliced in). It is
  shown through `vmText()` (`app/src/vm_text.hpp`): a run-time lookup in the `ViewModel` context (panels with
  their own helper use their own context), so the pseudo-locale and hand-added catalog entries apply. These
  sentences are not extracted by lupdate; moving them behind a view-model message catalog is future work.
- Catalogs are written without source locations (`-locations none`) so code changes do not churn the .ts
  files. `tools/check_catalog.py` (ctest `lint.catalog`) fails when a translate()/QT_TRANSLATE_NOOP literal is
  missing from a catalog: run the `update_translations` target after changing strings.
- `l10n: ` in `--self-test` sweeps the main window (menus, toolbars, docks, panels), ten stand-alone dialogs
  and the toolpath form of every operation under the pseudo-locale and fails on any visible wording that
  is not wrapped in brackets. Names the user typed (layers, sheets, tools, machines, materials) are exempt.
