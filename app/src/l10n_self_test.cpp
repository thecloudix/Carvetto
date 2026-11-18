#include "l10n_self_test.hpp"

#include <QAction>
#include <QCoreApplication>
#include <QMenuBar>
#include <QPushButton>

#include "localization.hpp"
#include "main_window.hpp"
#include "options_dialog.hpp"

namespace carvetto::app {

namespace {

#define L10N_REQUIRE(cond, text)                     \
    do {                                             \
        if (!(cond)) {                               \
            l10n::installLanguage(QStringLiteral("en")); \
            return std::string("l10n: ") + (text);   \
        }                                            \
    } while (false)

QStringList menuTitles(const MainWindow& w) {
    QStringList titles;
    for (const QAction* a : w.menuBar()->actions()) {
        titles << a->text();
    }
    return titles;
}

}  // namespace

std::string l10nSelfTest(MainWindow& window) {
    using l10n::installLanguage;
    // Source language: untouched English.
    L10N_REQUIRE(installLanguage(QStringLiteral("en")) == QLatin1String("en"), "English did not stay English");
    const QStringList english = menuTitles(window);
    L10N_REQUIRE(english.contains(QStringLiteral("&File")) && english.contains(QStringLiteral("&Help")),
                 "the menu bar has no &File / &Help in English");

    // The pseudo-locale transforms menus and dialogs built afterwards.
    L10N_REQUIRE(installLanguage(QStringLiteral("pseudo")) == QLatin1String("pseudo"), "the pseudo-locale did not activate");
    {
        MainWindow pseudo;
        pseudo.setQuiet(true);
        const QStringList titles = menuTitles(pseudo);
        L10N_REQUIRE(titles.contains(l10n::pseudoLocalize(QStringLiteral("&File"))) &&
                         !titles.contains(QStringLiteral("&File")),
                     "the menu bar was not transformed by the pseudo-locale");
        OptionsDialog dlg(window.preferences());
        L10N_REQUIRE(dlg.windowTitle() == l10n::pseudoLocalize(QStringLiteral("Options")),
                     "the Options dialog title was not transformed");
        L10N_REQUIRE(dlg.restoreButton()->text() == l10n::pseudoLocalize(QStringLiteral("Restore Defaults")),
                     "the Options buttons were not transformed");
    }

    // Choosing the language in Options applies it (the pseudo code is accepted and stored).
    installLanguage(QStringLiteral("en"));
    {
        viewmodel::Preferences p = window.preferences();
        p.language = "pseudo";
        L10N_REQUIRE(window.setPreferences(p).empty(), "the pseudo language preference was refused");
        L10N_REQUIRE(l10n::activeLanguage() == QLatin1String("pseudo"), "changing the preference did not switch language");
        p.language = "en";
        L10N_REQUIRE(window.setPreferences(p).empty() && l10n::activeLanguage() == QLatin1String("en"),
                     "switching back to English failed");
        p.language = "system";
        window.setPreferences(p);
        installLanguage(QStringLiteral("en"));
    }

    // Missing translations fall back to English: a language with no catalog installs nothing, and
    // strings without a translation keep their source text.
    L10N_REQUIRE(l10n::resolveLanguage(QStringLiteral("fr"), QLocale()) == QLatin1String("en"), "unknown language not mapped to English");
    L10N_REQUIRE(QCoreApplication::translate("OptionsDialog", "No such string in any catalog") ==
                     QLatin1String("No such string in any catalog"),
                 "an untranslated string changed");

#ifdef CARVETTO_HAVE_SAMPLE_QM
    // The sample German catalog (own wording) translates the menus; a string missing from it stays English.
    L10N_REQUIRE(installLanguage(QStringLiteral("de")) == QLatin1String("de"), "the German catalog was not found");
    L10N_REQUIRE(QCoreApplication::translate("MainWindow", "&File") == QLatin1String("&Datei"), "the German sample did not apply");
    L10N_REQUIRE(QCoreApplication::translate("MainWindow", "No such string in any catalog") ==
                     QLatin1String("No such string in any catalog"),
                 "a string missing from the German catalog did not fall back to English");
    installLanguage(QStringLiteral("en"));
#endif
    return {};
}

}  // namespace carvetto::app
