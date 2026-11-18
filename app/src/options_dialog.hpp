#pragma once

#include <QDialog>
#include <QString>
#include <map>
#include <string>

#include "carvetto/viewmodel/preferences.hpp"

class QLabel;
class QListWidget;
class QPushButton;
class QSettings;
class QStackedWidget;
class QWidget;

namespace carvetto::app {

/// Stores preferences in QSettings under the group "preferences" (the app's only QSettings use for them).
/// A default-constructed store uses the application's own settings; the path form (INI file) is for tests.
class QSettingsPreferencesStore : public viewmodel::PreferencesStore {
   public:
    QSettingsPreferencesStore() = default;
    explicit QSettingsPreferencesStore(QString iniPath) : iniPath_(std::move(iniPath)) {}
    viewmodel::PrefKeyValues load() override;
    bool save(const viewmodel::PrefKeyValues& values) override;

   private:
    QString iniPath_;
};

/// Setup > Options (T-0348): one page per `viewmodel::PrefCategory`, one editor per preference
/// (object name "pref:<key>"), Restore Defaults, and OK refused with a message while any value is invalid.
/// All validation is `viewmodel::validate`; the dialog only reads and writes widgets.
class OptionsDialog : public QDialog {
   public:
    OptionsDialog(const viewmodel::Preferences& initial, QWidget* parent = nullptr);

    /// What the widgets currently say. Unparsable numbers are left out of `parseIssues()` and keep
    /// their last good value in the result.
    viewmodel::Preferences edited() const;
    /// Problems found by reading the widgets (text that is not a number).
    std::vector<viewmodel::PrefIssue> parseIssues() const;
    void setPreferences(const viewmodel::Preferences& p);

    /// OK: accepts when the edited values are valid (paths checked), otherwise shows the first problem in
    /// the error label and stays open. Returns whether it accepted.
    bool tryAccept();
    /// The Restore Defaults button.
    void restoreDefaults();
    QString errorText() const;

    // --- Scripted access --------------------------------------------------------------------
    QWidget* field(const std::string& key) const;
    /// Types `text` into a text-entry preference, or picks a combo entry by its text.
    bool setFieldText(const std::string& key, const QString& text);
    QListWidget* categories() const { return categories_; }
    QStackedWidget* pages() const { return pages_; }
    QPushButton* restoreButton() const { return restore_; }

   private:
    void build();
    void showError(const QString& text);

    viewmodel::Preferences base_;
    std::map<std::string, QWidget*> fields_;
    QListWidget* categories_{nullptr};
    QStackedWidget* pages_{nullptr};
    QPushButton* restore_{nullptr};
    QLabel* error_{nullptr};
};

}  // namespace carvetto::app
