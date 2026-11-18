# ADR-0123: Localization infrastructure

Status: accepted (T-0350)

- User-visible strings in non-`Q_OBJECT` classes use `QCoreApplication::translate("Context", "text")`
  (lupdate extracts it); the context is the class name.
- Catalogs are `app/translations/carvetto_<lang>.ts` (shipped: `de`, a sample in our own wording).
  `qt_add_translations` provides the `update_translations` (lupdate) and `release_translations` (lrelease)
  targets; `.qm` files are installed to `share/carvetto/translations` and also found next to the binary
  or in the build tree. LinguistTools are optional at build time: without them the app runs in English.
- The `general/language` preference accepts `system`, `en`, `de`, `pseudo`. `system` maps the OS locale to a
  shipped language, else English. A missing catalog or string falls back to English (nothing installed).
- `pseudo` is a code-built translator (accented letters in brackets; mnemonics, `%n` placeholders and HTML
  tags preserved) so untranslated literals are visible. Language is applied at startup and when the
  preference changes; already-open windows keep their language until restart (no live retranslation).
- `tools/check_tr_literals.py` (ctest `lint.tr_literals`) is a ratchet: a file may not gain user-visible
  literals outside `tr()`/`translate()`; existing ones are grandfathered in `tools/tr_literals_baseline.txt`,
  which the sweep T-0351 empties.
