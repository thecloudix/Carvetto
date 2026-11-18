// Self-test of the Options dialog (T-0348): every preference has an editor, invalid values are refused,
// Restore Defaults works, preferences apply to the live window and round-trip through QSettings.

#include "options_self_test.hpp"

#include <QAction>
#include <QComboBox>
#include <QDir>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QStackedWidget>
#include <QTemporaryDir>
#include <set>

#include "canvas_widget.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/viewmodel/preferences.hpp"
#include "main_window.hpp"
#include "options_dialog.hpp"
#include "preview_panel.hpp"
#include "ruler_widget.hpp"

namespace carvetto::app {

namespace {

#define OPT_REQUIRE(cond, text)                    \
    do {                                           \
        if (!(cond)) {                             \
            return std::string("options: ") + (text); \
        }                                          \
    } while (false)

}  // namespace

std::string optionsSelfTest(MainWindow& window) {
    window.setQuiet(true);
    using viewmodel::Preferences;
    OPT_REQUIRE(window.optionsAction() != nullptr && window.optionsAction()->isEnabled(), "Setup > Options is missing");
    OPT_REQUIRE(window.preferences() == Preferences{}, "the window does not start with the default preferences");

    // 1. One editor per preference, one page per category, nothing else.
    {
        OptionsDialog dlg(window.preferences());
        std::set<std::string> keys;
        for (const viewmodel::PrefField& f : viewmodel::preferenceFields()) {
            OPT_REQUIRE(dlg.field(f.key) != nullptr, std::string("no editor for ") + f.key);
            keys.insert(f.key);
        }
        std::size_t editors = 0;
        for (QWidget* w : dlg.findChildren<QWidget*>()) {
            editors += w->objectName().startsWith(QStringLiteral("pref:")) ? 1U : 0U;
        }
        OPT_REQUIRE(editors == keys.size(), "the dialog has editors that are not preferences");
        OPT_REQUIRE(dlg.categories()->count() == dlg.pages()->count() && dlg.categories()->count() == 8,
                    "the category list and the pages disagree");
        OPT_REQUIRE(dlg.edited() == window.preferences(), "the dialog does not show the current preferences");
    }

    // 2. Invalid values are refused with a message; the dialog stays open.
    {
        OptionsDialog dlg(window.preferences());
        dlg.setFieldText("editing/undoDepth", QStringLiteral("many"));
        OPT_REQUIRE(!dlg.tryAccept() && !dlg.errorText().isEmpty(), "text for the undo depth was accepted");
        dlg.setFieldText("editing/undoDepth", QStringLiteral("3"));
        OPT_REQUIRE(!dlg.tryAccept(), "an undo depth of 3 was accepted");
        dlg.setFieldText("editing/undoDepth", QStringLiteral("100"));
        dlg.setFieldText("grid/spacingMm", QStringLiteral("0"));
        OPT_REQUIRE(!dlg.tryAccept(), "a grid spacing of 0 was accepted");
        dlg.setFieldText("grid/spacingMm", QStringLiteral("5"));
        dlg.setFieldText("folders/open", QDir::tempPath() + QStringLiteral("/carvetto-missing-folder-348"));
        OPT_REQUIRE(!dlg.tryAccept(), "a folder that does not exist was accepted");
        dlg.setFieldText("folders/open", QString());
        OPT_REQUIRE(dlg.tryAccept() && dlg.errorText().isEmpty(), "valid values were refused");
        OPT_REQUIRE(dlg.edited().undoDepth == 100 && dlg.edited().gridSpacingMm == 5.0, "edited values were lost");
        // Restore Defaults puts everything back.
        dlg.restoreButton()->click();
        OPT_REQUIRE(dlg.edited() == Preferences{}, "Restore Defaults did not reset the dialog");
    }

    // 3. Applying to the live window; a refused value changes nothing.
    Preferences p;
    p.undoDepth = 20;
    p.canvasBackground = 0x203040;
    p.materialFill = 0xFAFAF0;
    p.showRulers = false;
    p.antialiasing = false;
    p.simulationQuality = simulate::Quality::Fine;
    p.defaultUnits = document::Units::Inches;
    OPT_REQUIRE(window.setPreferences(p).empty(), "valid preferences were refused");
    OPT_REQUIRE(window.preferences() == p, "the window did not adopt the preferences");
    OPT_REQUIRE(window.currentDocument().undoStack().limit() == 20, "the undo depth was not applied");
    OPT_REQUIRE(materialFill().rgb() == QColor::fromRgb(0xFAFAF0).rgb() &&
                    canvasBackground().rgb() == QColor::fromRgb(0x203040).rgb(),
                "the colours were not applied");
    OPT_REQUIRE(!displayOptions().antialiasing, "antialiasing was not applied");
    OPT_REQUIRE(window.preview()->qualityCombo()->currentData().toInt() == static_cast<int>(simulate::Quality::Fine),
                "the simulation quality was not applied");
    OPT_REQUIRE(window.rulersHidden(), "the rulers were not hidden");
    Preferences bad = p;
    bad.undoDepth = 1;
    OPT_REQUIRE(!window.setPreferences(bad).empty() && window.preferences() == p, "an invalid value was adopted");

    // 4. A save / load round trip through QSettings (an INI file; the app's own settings stay untouched).
    QTemporaryDir dir;
    OPT_REQUIRE(dir.isValid(), "no temporary directory");
    const QString ini = dir.filePath(QStringLiteral("prefs.ini"));
    {
        QSettingsPreferencesStore store(ini);
        OPT_REQUIRE(store.save(viewmodel::serialize(p)), "saving the preferences failed");
    }
    {
        QSettingsPreferencesStore store(ini);
        viewmodel::PreferencesModel reloaded;
        const auto issues = reloaded.load(store);
        OPT_REQUIRE(issues.empty() && reloaded.current() == p, "the preferences did not survive a restart");
        // A damaged value falls back alone.
        viewmodel::PrefKeyValues kv = store.load();
        for (auto& e : kv) {
            if (e.first == "grid/spacingMm") {
                e.second = "garbage";
            }
        }
        store.save(kv);
        viewmodel::PreferencesModel damaged;
        OPT_REQUIRE(!damaged.load(store).empty() && damaged.current().gridSpacingMm == Preferences{}.gridSpacingMm &&
                        damaged.current().undoDepth == 20,
                    "a damaged stored value was not defaulted on its own");
    }

    // 5. Restore the defaults on the window so later self-tests see the normal look.
    OPT_REQUIRE(window.setPreferences(Preferences{}).empty() && window.preferences() == Preferences{},
                "restoring the defaults failed");
    OPT_REQUIRE(materialFill().rgb() == QColor::fromRgb(0xFFFFFF).rgb() && !window.rulersHidden(),
                "the default look did not come back");
    return {};
}

}  // namespace carvetto::app
