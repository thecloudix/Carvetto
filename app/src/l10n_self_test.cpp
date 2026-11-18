#include "l10n_self_test.hpp"

#include <QAbstractButton>
#include <QAbstractSpinBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QTableWidget>
#include <QTreeWidget>
#include <QAction>
#include <QCoreApplication>
#include <QDockWidget>
#include <QGroupBox>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QPushButton>
#include <QRegularExpression>
#include <QTabBar>
#include <QToolBar>

#include "array_copy_menu.hpp"
#include "localization.hpp"
#include "main_window.hpp"
#include "new_job_dialog.hpp"
#include "node_edit_menu.hpp"
#include "options_dialog.hpp"
#include "plate_production_dialog.hpp"
#include "rotary_material_dialog.hpp"
#include "rotary_tools_dialogs.hpp"
#include "tool_setup_dialog.hpp"
#include "toolpath_form_dialog.hpp"
#include "toolpaths_panel.hpp"
#include "vector_ops_menu.hpp"

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

// True when a visible text is real wording (two letters in a row) that did not go through the
// pseudo-locale (which wraps every translated string in brackets).
QStringList g_dataNames;  // names the user typed (layers, sheets): data, not wording

bool untranslated(const QString& text) {
    static const QRegularExpression word(QStringLiteral("[A-Za-z]{2,}"));
    QString t = text;
    if (g_dataNames.contains(t.trimmed())) {
        return false;
    }
    t.remove(QLatin1Char('&'));
    t.remove(QRegularExpression(QStringLiteral("<[^>]*>")));
    t = t.trimmed();
    // Unit symbols are not wording.
    static const QRegularExpression unit(QStringLiteral("\\b(mm|in|deg|px|rpm|min|s2|s)\\b"));
    t.remove(unit);
    t = t.trimmed();
    if (t.isEmpty() || t.startsWith(QLatin1Char('[')) || !t.contains(word)) {
        return false;
    }
    // Names of things, not wording: the program name and shortcut-only labels.
    static const QStringList kept = {QStringLiteral("Carvetto"), QStringLiteral("Ctrl"), QStringLiteral("Shift")};
    return !kept.contains(t) && !g_dataNames.contains(t);
}

void sweepMenu(const QMenu* menu, const QString& path, QStringList& bad) {
    for (const QAction* a : menu->actions()) {
        if (a->isSeparator()) {
            continue;
        }
        const QString here = path + QLatin1Char('>') + a->text();
        if (untranslated(a->text())) {
            bad << here;
        }
        if (untranslated(a->toolTip()) && a->toolTip() != a->text()) {
            bad << here + QStringLiteral(" (tooltip)");
        }
        if (a->menu() != nullptr) {
            sweepMenu(a->menu(), here, bad);
        }
    }
}

/// Every visible text of a widget tree (labels, buttons, boxes, tabs, list and combo items, table and tree
/// headers, tooltips, suffixes) that is not wrapped by the pseudo-locale.
void sweepWidget(const QWidget& root, const QString& what, QStringList& bad) {
    const auto note = [&](const QString& kind, const QString& text) { bad << what + QLatin1Char(' ') + kind + QLatin1Char(' ') + text; };
    if (untranslated(root.windowTitle())) {
        note(QStringLiteral("title"), root.windowTitle());
    }
    for (const QWidget* w : root.findChildren<QWidget*>()) {
        if (untranslated(w->toolTip()) && !w->toolTip().isEmpty()) {
            note(QStringLiteral("tooltip"), w->toolTip());
        }
        if (const auto* b = qobject_cast<const QAbstractButton*>(w)) {
            if (untranslated(b->text())) {
                note(QStringLiteral("button"), b->text());
            }
        } else if (const auto* g = qobject_cast<const QGroupBox*>(w)) {
            if (untranslated(g->title())) {
                note(QStringLiteral("group"), g->title());
            }
        } else if (const auto* l = qobject_cast<const QLabel*>(w)) {
            if (untranslated(l->text())) {
                note(QStringLiteral("label"), l->text());
            }
        } else if (const auto* tabs = qobject_cast<const QTabBar*>(w)) {
            for (int i = 0; i < tabs->count(); ++i) {
                if (untranslated(tabs->tabText(i))) {
                    note(QStringLiteral("tab"), tabs->tabText(i));
                }
            }
        } else if (const auto* c = qobject_cast<const QComboBox*>(w)) {
            for (int i = 0; i < c->count(); ++i) {
                if (untranslated(c->itemText(i))) {
                    note(QStringLiteral("combo item"), c->itemText(i));
                }
            }
        } else if (const auto* sp = qobject_cast<const QAbstractSpinBox*>(w)) {
            const QString suffix = qobject_cast<const QDoubleSpinBox*>(sp) != nullptr
                                       ? qobject_cast<const QDoubleSpinBox*>(sp)->suffix()
                                       : (qobject_cast<const QSpinBox*>(sp) != nullptr ? qobject_cast<const QSpinBox*>(sp)->suffix() : QString());
            if (untranslated(suffix)) {
                note(QStringLiteral("suffix"), suffix);
            }
        } else if (const auto* tree = qobject_cast<const QTreeWidget*>(w)) {
            for (int i = 0; i < tree->columnCount(); ++i) {
                const QString h = tree->headerItem()->text(i);
                if (untranslated(h)) {
                    note(QStringLiteral("header"), h);
                }
            }
        } else if (const auto* table = qobject_cast<const QTableWidget*>(w)) {
            for (int i = 0; i < table->columnCount(); ++i) {
                const QTableWidgetItem* h = table->horizontalHeaderItem(i);
                if (h != nullptr && untranslated(h->text())) {
                    note(QStringLiteral("header"), h->text());
                }
            }
        }
    }
}

/// Every visible text of the main window and its panels under the pseudo-locale that is not wrapped.
QStringList sweepUntranslated(MainWindow& w) {
    QStringList bad;
    g_dataNames.clear();
    for (const auto& layer : w.session().document().layers()) {
        g_dataNames << QString::fromStdString(layer.name);
    }
    for (const auto& sheet : w.session().document().sheets()) {
        g_dataNames << QString::fromStdString(sheet.name);
    }
    for (const QAction* a : w.menuBar()->actions()) {
        if (untranslated(a->text())) {
            bad << a->text();
        }
        if (a->menu() != nullptr) {
            sweepMenu(a->menu(), a->text(), bad);
        }
    }
    for (const QToolBar* bar : w.findChildren<QToolBar*>()) {
        for (const QAction* a : bar->actions()) {
            if (!a->isSeparator() && (untranslated(a->text()) || untranslated(a->toolTip()))) {
                bad << QStringLiteral("toolbar ") + a->text();
            }
        }
    }
    for (const QDockWidget* dock : w.findChildren<QDockWidget*>()) {
        if (untranslated(dock->windowTitle())) {
            bad << QStringLiteral("dock ") + dock->windowTitle();
        }
    }
    sweepWidget(w, QStringLiteral("main window:"), bad);
    return bad;
}

/// The dialogs that can be built without a file or a running job, each swept under the pseudo-locale.
QStringList sweepDialogs(MainWindow& w) {
    QStringList bad;
    // Tool, machine and material names are the user's data (the starter database ships English names).
    const tooling::ToolDatabase& db = w.toolpaths()->toolDatabase();
    for (const auto& entry : db.tools()) {
        g_dataNames << QString::fromStdString(entry.second.name);
    }
    for (const auto& entry : db.machines()) {
        g_dataNames << QString::fromStdString(entry.second.name);
    }
    for (const auto& entry : db.materials()) {
        g_dataNames << QString::fromStdString(entry.second.name);
    }
    const document::Units mm = document::Units::Millimetres;
    {
        ArrayCopyDialog d(mm, viewmodel::ArrayOptions{});
        sweepWidget(d, QStringLiteral("Array Copy:"), bad);
    }
    {
        CircularCopyDialog d(mm, viewmodel::CircularOptions{});
        sweepWidget(d, QStringLiteral("Circular Copy:"), bad);
    }
    {
        OffsetDialog d(mm, viewmodel::OffsetOptions{});
        sweepWidget(d, QStringLiteral("Offset:"), bad);
    }
    {
        NodePropertiesDialog d(mm, std::nullopt);
        sweepWidget(d, QStringLiteral("Node Properties:"), bad);
    }
    {
        NewJobDialog d(viewmodel::NewJobForm{});
        sweepWidget(d, QStringLiteral("New Job:"), bad);
    }
    {
        RotaryMaterialDialog d(viewmodel::RotaryMaterialForm{});
        sweepWidget(d, QStringLiteral("Rotary Material:"), bad);
    }
    {
        UnwrapperDialog d;
        sweepWidget(d, QStringLiteral("Vector Unwrapper:"), bad);
    }
    {
        RotaryImportDialog d;
        sweepWidget(d, QStringLiteral("Rotary Import:"), bad);
    }
    {
        PlateProductionDialog d(w.session().document());
        sweepWidget(d, QStringLiteral("Plate Production:"), bad);
    }
    {
        ToolSetupDialog d(w.toolpaths()->toolDatabase(), w.toolpaths()->activeMachineId(), mm, {});
        d.setQuiet(true);
        sweepWidget(d, QStringLiteral("Tools, Machines and Materials:"), bad);
    }
    // The toolpath form of every operation.
    for (int op = 0; op <= static_cast<int>(document::ToolpathOperation::Rounding); ++op) {
        const auto operation = static_cast<document::ToolpathOperation>(op);
        const QString name = QString::fromUtf8(operationLabel(operation));
        w.toolpaths()->addAction(operation)->trigger();
        if (ToolpathFormDialog* form = w.toolpaths()->formDialog()) {
            sweepWidget(*form, QStringLiteral("toolpath form ") + name + QLatin1Char(':'), bad);
            form->reject();
            QCoreApplication::processEvents();
        }
    }
    return bad;
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
        // Scripted sweep: nothing visible in the menus, toolbars and panels escapes the pseudo-locale.
        QStringList bad = sweepUntranslated(pseudo);
        bad.removeDuplicates();
        L10N_REQUIRE(bad.isEmpty(), "untranslated visible text: " + bad.join(QStringLiteral("; ")).toStdString());
        QStringList badDialogs = sweepDialogs(pseudo);
        badDialogs.removeDuplicates();
        L10N_REQUIRE(badDialogs.isEmpty(),
                     "untranslated visible text in dialogs: " + badDialogs.join(QStringLiteral("; ")).toStdString());
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
