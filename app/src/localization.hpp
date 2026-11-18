#pragma once

// Localization plumbing (T-0350): loads Qt translations for the language preference, falls back to
// English when none is available, and offers a pseudo-locale that visibly transforms every string
// passed through tr()/QCoreApplication::translate (so untranslated literals stand out).

#include <QLocale>
#include <QString>
#include <QStringList>

namespace carvetto::app::l10n {

/// Language codes the Options dialog offers; "system" follows the OS locale.
/// "pseudo" is a test locale built in code, "en" is the source language (no catalog needed).
QStringList supportedLanguages();

/// The concrete language ("en", "de" or "pseudo") a preference value stands for: "system" maps the
/// given system locale to a shipped language, falling back to "en"; unknown codes give "en".
QString resolveLanguage(const QString& preference, const QLocale& system);

/// Pseudo-localisation of one source string: letters become accented look-alikes, the text is wrapped
/// in brackets; mnemonics (&x), placeholders (%1, %n) and HTML tags are left intact.
QString pseudoLocalize(const QString& source);

/// Directories searched for carvetto_<lang>.qm, best first (install share dir, next to the binary,
/// the build tree).
QStringList translationSearchDirs();

/// Replaces the installed translator with the one for `preference`. Returns the language that is now
/// active ("en" when nothing was installed, including when a catalog is missing: English fallback).
QString installLanguage(const QString& preference);

/// The language installLanguage() last made active ("en" before any call).
QString activeLanguage();

}  // namespace carvetto::app::l10n
