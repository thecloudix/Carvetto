#include "options_dialog.hpp"

#include <QCheckBox>
#include <QCoreApplication>
#include <QColorDialog>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QSettings>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <cstdint>
#include <memory>

namespace carvetto::app {

using viewmodel::PrefCategory;
using viewmodel::Preferences;
using viewmodel::PrefIssue;

namespace {

QString qs(const std::string& s) {
    return QString::fromUtf8(s.data(), static_cast<qsizetype>(s.size()));
}
std::string stdString(const QString& s) {
    return s.toStdString();
}

constexpr const char* kRgbProperty = "rgb";

void setColourButton(QPushButton* b, std::uint32_t rgb) {
    b->setProperty(kRgbProperty, static_cast<unsigned>(rgb));
    b->setText(qs(viewmodel::colourToString(rgb)));
    b->setStyleSheet(QStringLiteral("background-color: %1; color: %2;")
                         .arg(qs(viewmodel::colourToString(rgb)),
                              (((rgb >> 16) & 0xFFu) * 299 + ((rgb >> 8) & 0xFFu) * 587 + (rgb & 0xFFu) * 114) > 128000
                                  ? QStringLiteral("#000000")
                                  : QCoreApplication::translate("OptionsDialog", "#ffffff")));
}

std::uint32_t colourOf(const QWidget* w) {
    return static_cast<std::uint32_t>(w->property(kRgbProperty).toUInt());
}

}  // namespace

viewmodel::PrefKeyValues QSettingsPreferencesStore::load() {
    std::unique_ptr<QSettings> settings =
        iniPath_.isEmpty() ? std::make_unique<QSettings>() : std::make_unique<QSettings>(iniPath_, QSettings::IniFormat);
    viewmodel::PrefKeyValues kv;
    settings->beginGroup(QStringLiteral("preferences"));
    for (const QString& key : settings->allKeys()) {
        kv.emplace_back(stdString(key), stdString(settings->value(key).toString()));
    }
    settings->endGroup();
    return kv;
}

bool QSettingsPreferencesStore::save(const viewmodel::PrefKeyValues& values) {
    std::unique_ptr<QSettings> settings =
        iniPath_.isEmpty() ? std::make_unique<QSettings>() : std::make_unique<QSettings>(iniPath_, QSettings::IniFormat);
    settings->beginGroup(QStringLiteral("preferences"));
    settings->remove(QString());
    for (const auto& kv : values) {
        settings->setValue(qs(kv.first), qs(kv.second));
    }
    settings->endGroup();
    settings->sync();
    return settings->status() == QSettings::NoError;
}

OptionsDialog::OptionsDialog(const Preferences& initial, QWidget* parent) : QDialog(parent), base_(initial) {
    setWindowTitle(QCoreApplication::translate("OptionsDialog", "Options"));
    setObjectName(QStringLiteral("optionsDialog"));
    build();
    setPreferences(initial);
}

void OptionsDialog::build() {
    categories_ = new QListWidget(this);
    categories_->setObjectName(QStringLiteral("optionsCategories"));
    categories_->setFixedWidth(150);
    pages_ = new QStackedWidget(this);

    std::map<PrefCategory, QFormLayout*> forms;
    const PrefCategory order[] = {PrefCategory::General, PrefCategory::Editing, PrefCategory::GridSnap,
                                  PrefCategory::Colours, PrefCategory::Display, PrefCategory::Folders,
                                  PrefCategory::Tools,   PrefCategory::Simulation};
    for (PrefCategory c : order) {
        auto* page = new QWidget(pages_);
        auto* form = new QFormLayout(page);
        forms[c] = form;
        pages_->addWidget(page);
        categories_->addItem(QString::fromUtf8(viewmodel::categoryTitle(c)));
    }
    connect(categories_, &QListWidget::currentRowChanged, pages_, &QStackedWidget::setCurrentIndex);

    for (const viewmodel::PrefField& f : viewmodel::preferenceFields()) {
        const std::string key = f.key;
        QWidget* editor = nullptr;
        QWidget* shown = nullptr;  // what goes in the form (editor, or editor plus Browse)
        QFormLayout* form = forms[f.category];
        if (key == "general/units") {
            auto* c = new QComboBox(this);
            c->addItem(QCoreApplication::translate("OptionsDialog", "Millimetres"), QStringLiteral("mm"));
            c->addItem(QCoreApplication::translate("OptionsDialog", "Inches"), QStringLiteral("in"));
            editor = c;
        } else if (key == "general/language") {
            auto* c = new QComboBox(this);
            c->addItem(QCoreApplication::translate("OptionsDialog", "System default"), QStringLiteral("system"));
            c->addItem(QCoreApplication::translate("OptionsDialog", "English"), QStringLiteral("en"));
            c->addItem(QCoreApplication::translate("OptionsDialog", "Deutsch"), QStringLiteral("de"));  // endonym: shown in its own language
            c->addItem(QCoreApplication::translate("OptionsDialog", "Pseudo-locale (test)"), QStringLiteral("pseudo"));
            editor = c;
        } else if (key == "simulation/quality") {
            auto* c = new QComboBox(this);
            c->addItem(QCoreApplication::translate("OptionsDialog", "Coarse"), QStringLiteral("coarse"));
            c->addItem(QCoreApplication::translate("OptionsDialog", "Medium"), QStringLiteral("medium"));
            c->addItem(QCoreApplication::translate("OptionsDialog", "Fine"), QStringLiteral("fine"));
            editor = c;
        } else if (key.rfind("colours/", 0) == 0) {
            auto* b = new QPushButton(this);
            connect(b, &QPushButton::clicked, this, [this, b]() {
                const QColor picked = QColorDialog::getColor(QColor::fromRgb(colourOf(b)), this);
                if (picked.isValid()) {
                    setColourButton(b, picked.rgb() & 0xFFFFFFu);
                }
            });
            editor = b;
        } else if (key == "editing/autosaveEnabled" || key == "grid/show" || key == "grid/snapEnabled" ||
                   key == "display/antialiasing" || key == "display/showRulers") {
            editor = new QCheckBox(this);
        } else {
            auto* e = new QLineEdit(this);
            editor = e;
            const bool isFolder = key.rfind("folders/", 0) == 0;
            if (isFolder || key == "tools/databasePath") {
                auto* row = new QWidget(this);
                auto* h = new QHBoxLayout(row);
                h->setContentsMargins(0, 0, 0, 0);
                h->addWidget(e, 1);
                auto* browse = new QPushButton(QCoreApplication::translate("OptionsDialog", "Browse..."), row);
                h->addWidget(browse);
                connect(browse, &QPushButton::clicked, this, [this, e, isFolder]() {
                    const QString picked =
                        isFolder ? QFileDialog::getExistingDirectory(this, QCoreApplication::translate("OptionsDialog", "Choose folder"), e->text())
                                 : QFileDialog::getSaveFileName(this, QCoreApplication::translate("OptionsDialog", "Tool database file"), e->text(),
                                                                QCoreApplication::translate("OptionsDialog", "Tool database (*.json);;All files (*)"));
                    if (!picked.isEmpty()) {
                        e->setText(picked);
                    }
                });
                shown = row;
            }
        }
        editor->setObjectName(QStringLiteral("pref:") + qs(key));
        fields_[key] = editor;
        form->addRow(QString::fromUtf8(f.label) + QStringLiteral(":"), shown != nullptr ? shown : editor);
    }

    error_ = new QLabel(this);
    error_->setObjectName(QStringLiteral("optionsError"));
    error_->setStyleSheet(QStringLiteral("color: #b00020;"));
    error_->setWordWrap(true);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    restore_ = buttons->addButton(QCoreApplication::translate("OptionsDialog", "Restore Defaults"), QDialogButtonBox::ResetRole);
    restore_->setObjectName(QStringLiteral("optionsRestore"));
    connect(restore_, &QPushButton::clicked, this, [this]() { restoreDefaults(); });
    connect(buttons, &QDialogButtonBox::accepted, this, [this]() { tryAccept(); });
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    auto* top = new QHBoxLayout();
    top->addWidget(categories_);
    top->addWidget(pages_, 1);
    auto* outer = new QVBoxLayout(this);
    outer->addLayout(top, 1);
    outer->addWidget(error_);
    outer->addWidget(buttons);
    categories_->setCurrentRow(0);
}

QWidget* OptionsDialog::field(const std::string& key) const {
    const auto it = fields_.find(key);
    return it == fields_.end() ? nullptr : it->second;
}

void OptionsDialog::setPreferences(const Preferences& p) {
    base_ = p;
    auto setCombo = [&](const char* key, const QString& wanted) {
        auto* c = qobject_cast<QComboBox*>(field(key));
        const int i = c->findData(wanted);
        c->setCurrentIndex(i < 0 ? 0 : i);
    };
    auto setCheck = [&](const char* key, bool on) { qobject_cast<QCheckBox*>(field(key))->setChecked(on); };
    auto setLine = [&](const char* key, const std::string& text) { qobject_cast<QLineEdit*>(field(key))->setText(qs(text)); };
    const auto kv = viewmodel::serialize(p);
    auto value = [&](const char* key) {
        for (const auto& e : kv) {
            if (e.first == key) {
                return e.second;
            }
        }
        return std::string();
    };
    setCombo("general/units", qs(value("general/units")));
    setCombo("general/language", qs(p.language));
    setCombo("simulation/quality", qs(value("simulation/quality")));
    setLine("editing/undoDepth", value("editing/undoDepth"));
    setCheck("editing/autosaveEnabled", p.autosaveEnabled);
    setLine("editing/autosaveMinutes", value("editing/autosaveMinutes"));
    setCheck("grid/show", p.showGrid);
    setLine("grid/spacingMm", value("grid/spacingMm"));
    setCheck("grid/snapEnabled", p.snapEnabled);
    setLine("grid/snapTolerancePx", value("grid/snapTolerancePx"));
    setColourButton(qobject_cast<QPushButton*>(field("colours/canvasBackground")), p.canvasBackground);
    setColourButton(qobject_cast<QPushButton*>(field("colours/materialFill")), p.materialFill);
    setCheck("display/antialiasing", p.antialiasing);
    setCheck("display/showRulers", p.showRulers);
    setLine("folders/open", p.openFolder);
    setLine("folders/save", p.saveFolder);
    setLine("folders/export", p.exportFolder);
    setLine("tools/databasePath", p.toolDatabasePath);
    error_->clear();
}

Preferences OptionsDialog::edited() const {
    Preferences p = base_;
    auto combo = [&](const char* key) { return stdString(qobject_cast<QComboBox*>(field(key))->currentData().toString()); };
    auto check = [&](const char* key) { return qobject_cast<QCheckBox*>(field(key))->isChecked(); };
    auto text = [&](const char* key) { return qobject_cast<QLineEdit*>(field(key))->text(); };
    p.defaultUnits = combo("general/units") == "in" ? document::Units::Inches : document::Units::Millimetres;
    p.language = combo("general/language");
    const std::string q = combo("simulation/quality");
    p.simulationQuality = q == "coarse"   ? simulate::Quality::Coarse
                          : q == "fine"   ? simulate::Quality::Fine
                                          : simulate::Quality::Medium;
    bool ok = false;
    int i = text("editing/undoDepth").trimmed().toInt(&ok);
    if (ok) {
        p.undoDepth = i;
    }
    p.autosaveEnabled = check("editing/autosaveEnabled");
    i = text("editing/autosaveMinutes").trimmed().toInt(&ok);
    if (ok) {
        p.autosaveMinutes = i;
    }
    p.showGrid = check("grid/show");
    double d = text("grid/spacingMm").trimmed().toDouble(&ok);
    if (ok) {
        p.gridSpacingMm = d;
    }
    p.snapEnabled = check("grid/snapEnabled");
    d = text("grid/snapTolerancePx").trimmed().toDouble(&ok);
    if (ok) {
        p.snapTolerancePx = d;
    }
    p.canvasBackground = colourOf(field("colours/canvasBackground"));
    p.materialFill = colourOf(field("colours/materialFill"));
    p.antialiasing = check("display/antialiasing");
    p.showRulers = check("display/showRulers");
    p.openFolder = stdString(text("folders/open").trimmed());
    p.saveFolder = stdString(text("folders/save").trimmed());
    p.exportFolder = stdString(text("folders/export").trimmed());
    p.toolDatabasePath = stdString(text("tools/databasePath").trimmed());
    return p;
}

std::vector<PrefIssue> OptionsDialog::parseIssues() const {
    std::vector<PrefIssue> out;
    bool ok = false;
    auto text = [&](const char* key) { return qobject_cast<QLineEdit*>(field(key))->text().trimmed(); };
    text("editing/undoDepth").toInt(&ok);
    if (!ok) {
        out.push_back({"editing/undoDepth", QCoreApplication::translate("OptionsDialog", "Undo steps must be a whole number.").toStdString()});
    }
    text("editing/autosaveMinutes").toInt(&ok);
    if (!ok) {
        out.push_back({"editing/autosaveMinutes", QCoreApplication::translate("OptionsDialog", "The autosave interval must be a whole number of minutes.").toStdString()});
    }
    text("grid/spacingMm").toDouble(&ok);
    if (!ok) {
        out.push_back({"grid/spacingMm", QCoreApplication::translate("OptionsDialog", "Grid spacing must be a number.").toStdString()});
    }
    text("grid/snapTolerancePx").toDouble(&ok);
    if (!ok) {
        out.push_back({"grid/snapTolerancePx", QCoreApplication::translate("OptionsDialog", "Snap distance must be a number.").toStdString()});
    }
    return out;
}

void OptionsDialog::showError(const QString& text) {
    error_->setText(text);
}

QString OptionsDialog::errorText() const {
    return error_->text();
}

bool OptionsDialog::tryAccept() {
    std::vector<PrefIssue> issues = parseIssues();
    if (issues.empty()) {
        viewmodel::PrefValidation opts;
        opts.checkPaths = true;
        issues = viewmodel::validate(edited(), opts);
    }
    if (!issues.empty()) {
        showError(qs(issues.front().message));
        // Show the page holding the problem.
        const auto& list = viewmodel::preferenceFields();
        for (std::size_t i = 0; i < list.size(); ++i) {
            if (issues.front().key == list[i].key) {
                categories_->setCurrentRow(static_cast<int>(list[i].category));
                break;
            }
        }
        return false;
    }
    error_->clear();
    accept();
    return true;
}

void OptionsDialog::restoreDefaults() {
    setPreferences(viewmodel::PreferencesModel::defaults());
}

bool OptionsDialog::setFieldText(const std::string& key, const QString& text) {
    QWidget* w = field(key);
    if (auto* e = qobject_cast<QLineEdit*>(w)) {
        e->setText(text);
        return true;
    }
    if (auto* c = qobject_cast<QComboBox*>(w)) {
        const int i = c->findText(text);
        if (i >= 0) {
            c->setCurrentIndex(i);
            return true;
        }
    }
    return false;
}

}  // namespace carvetto::app
