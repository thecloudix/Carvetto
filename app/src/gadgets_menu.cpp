#include "gadgets_menu.hpp"

#include <QAction>
#include <QFileDialog>
#include <QKeySequence>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPushButton>
#include <QSettings>
#include <QStandardPaths>
#include <QStatusBar>

#include "gadget_dialogs.hpp"
#include "main_window.hpp"
#include "toolpaths_panel.hpp"
#include "carvetto/viewmodel/file_session.hpp"

namespace carvetto::app {

namespace {

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

}  // namespace

std::filesystem::path defaultGadgetsFolder() {
    const QString base = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    if (base.isEmpty()) {
        return {};
    }
    return viewmodel::pathFromUtf8(base.toStdString()) / "gadgets";
}

GadgetsMenu::GadgetsMenu(MainWindow& window) : window_(window), manager_(defaultGadgetsFolder()) {
    menu_ = window.menuBar()->addMenu(QStringLiteral("&Gadgets"));
    menu_->setObjectName(QStringLiteral("gadgetsMenu"));
    installAction_ = new QAction(QStringLiteral("&Install New Gadget..."), menu_);
    installAction_->setObjectName(QStringLiteral("installGadgetAction"));
    QObject::connect(installAction_, &QAction::triggered, menu_, [this]() { installGadget(); });
    shortcutsAction_ = new QAction(QStringLiteral("Gadget &Shortcuts..."), menu_);
    shortcutsAction_->setObjectName(QStringLiteral("gadgetShortcutsAction"));
    QObject::connect(shortcutsAction_, &QAction::triggered, menu_, [this]() { shortcutsDialog(); });
    rebuild();
}

GadgetsMenu::~GadgetsMenu() = default;

void GadgetsMenu::setFolder(const std::filesystem::path& folder) {
    manager_ = viewmodel::GadgetManager(folder);
    if (persist_) {
        loadOverrides();
    }
    rebuild();
}

void GadgetsMenu::setPersist(bool persist) {
    persist_ = persist;
    if (persist) {
        loadOverrides();
        rebuild();
    }
}

void GadgetsMenu::loadOverrides() {
    QSettings settings;
    scripting::ShortcutOverrides o;
    settings.beginGroup(QStringLiteral("gadgetShortcuts"));
    for (const QString& key : settings.childKeys()) {
        o[key.toStdString()] = settings.value(key).toString().toStdString();
    }
    settings.endGroup();
    manager_.setOverrides(std::move(o));
    manager_.rescan();
}

void GadgetsMenu::storeOverrides() {
    if (!persist_) {
        return;
    }
    QSettings settings;
    settings.remove(QStringLiteral("gadgetShortcuts"));
    settings.beginGroup(QStringLiteral("gadgetShortcuts"));
    for (const auto& kv : manager_.overrides()) {
        settings.setValue(qs(kv.first), qs(kv.second));
    }
    settings.endGroup();
}

void GadgetsMenu::addItems(QMenu* into, const std::vector<viewmodel::GadgetMenuItem>& items) {
    for (const viewmodel::GadgetMenuItem& item : items) {
        if (item.gadget < 0) {
            QMenu* sub = into->addMenu(qs(viewmodel::escapeMenuMnemonics(item.label)));
            addItems(sub, item.children);
            continue;
        }
        QAction* action = into->addAction(qs(viewmodel::escapeMenuMnemonics(item.label)));
        action->setEnabled(item.enabled);
        if (!item.tooltip.empty()) {
            action->setToolTip(qs(item.tooltip));
            action->setStatusTip(qs(item.tooltip));
        }
        if (!item.shortcut.empty()) {
            action->setShortcut(QKeySequence(qs(item.shortcut)));
        }
        action->setData(item.gadget);
        const std::size_t index = static_cast<std::size_t>(item.gadget);
        QObject::connect(action, &QAction::triggered, menu_, [this, index]() { runGadget(index); });
        gadgetActions_.push_back(action);
    }
}

void GadgetsMenu::rebuild() {
    // Install and Shortcuts persist across rebuilds: a rebuild can run inside their own slots.
    menu_->removeAction(installAction_);
    menu_->removeAction(shortcutsAction_);
    menu_->clear();
    gadgetActions_.clear();
    const std::vector<viewmodel::GadgetMenuItem> items = manager_.menu();
    addItems(menu_, items);
    if (items.empty()) {
        QAction* none = menu_->addAction(QStringLiteral("(no gadgets installed)"));
        none->setEnabled(false);
    }
    menu_->addSeparator();
    menu_->addAction(installAction_);
    menu_->addAction(shortcutsAction_);
}

std::vector<QAction*> GadgetsMenu::gadgetActions() const {
    return gadgetActions_;
}

scripting::GadgetUi GadgetsMenu::realUi() {
    scripting::GadgetUi ui;
    QWidget* parent = &window_;
    ui.form = [this, parent](const scripting::FormSpec& spec) -> std::optional<scripting::ScriptValue::Map> {
        GadgetFormChoices choices;
        const document::Document& doc = window_.currentDocument();
        for (const document::Layer& l : doc.layers()) {
            choices.layers.emplace_back(l.id.value, l.name);
        }
        for (const auto& kv : window_.toolpaths()->session().toolDatabase().tools()) {
            choices.tools.emplace_back(kv.first, kv.second.name);
        }
        GadgetFormDialog dialog(spec, choices, parent);
        dialog.onAbort = [this] { cancel_.store(true); };
        if (dialog.exec() != QDialog::Accepted) {
            return std::nullopt;
        }
        return dialog.values();
    };
    // Every box has an "Abort gadget" button: the deadline does not run while a box is open, so this
    // (with the per-run caps of GadgetLimits) is how the user ends a gadget that keeps asking.
    const auto box = [this, parent](const std::string& text, QMessageBox::Icon icon, bool yesNo) {
        QMessageBox mb(icon, QStringLiteral("Gadget"), qs(text), QMessageBox::NoButton, parent);
        QPushButton* yes = mb.addButton(yesNo ? QMessageBox::Yes : QMessageBox::Ok);
        if (yesNo) {
            mb.addButton(QMessageBox::No);
        }
        QPushButton* abort = mb.addButton(QStringLiteral("Abort gadget"), QMessageBox::DestructiveRole);
        mb.setDefaultButton(yes);
        mb.exec();
        if (mb.clickedButton() == abort) {
            cancel_.store(true);
            return false;
        }
        return mb.clickedButton() == yes;
    };
    ui.message = [box](const std::string& text) { box(text, QMessageBox::Information, false); };
    ui.confirm = [box](const std::string& text) { return box(text, QMessageBox::Question, true); };
    return ui;
}

viewmodel::GadgetRunReport GadgetsMenu::runGadget(std::size_t index) {
    viewmodel::GadgetRunReport report;
    if (window_.toolpaths()->busy()) {
        report.error = viewmodel::GadgetErrorReport{"Gadget", "Wait for the toolpath calculation to finish.", "",
                                                    scripting::ScriptStatus::RuntimeError, 0};
        if (!window_.quiet()) {
            QMessageBox::information(&window_, QStringLiteral("Gadget"), qs(report.error->summary));
        }
        last_ = report;
        return report;
    }
    scripting::GadgetUi ui = uiOverride_ ? *uiOverride_ : realUi();
    cancel_.store(false);
    report = manager_.run(index, window_.currentDocument(), window_.toolpaths()->session(), std::move(ui), limits_,
                          &cancel_);
    window_.documentChanged();
    if (report.ok) {
        if (!window_.quiet()) {
            window_.statusBar()->showMessage(qs(report.summary), 6000);
        }
    } else if (report.error && !window_.quiet()) {
        showGadgetError(&window_, *report.error);
    }
    last_ = report;
    return report;
}

std::optional<scripting::InstallResult> GadgetsMenu::installGadget(const std::filesystem::path& folder) {
    std::filesystem::path source = folder;
    if (source.empty()) {
        const QString picked = QFileDialog::getExistingDirectory(&window_, QStringLiteral("Choose the gadget folder"));
        if (picked.isEmpty()) {
            return std::nullopt;
        }
        source = viewmodel::pathFromUtf8(picked.toStdString());
    }
    std::function<bool(const QString&)> ask;
    if (window_.quiet()) {
        ask = [](const QString&) { return true; };
    }
    auto result = installGadgetDialog(&window_, manager_, source, ask);
    if (result && result->ok) {
        rebuild();
    }
    return result;
}

void GadgetsMenu::shortcutsDialog() {
    GadgetShortcutsDialog dialog(manager_, &window_);
    dialog.onChanged = [this]() {
        storeOverrides();
        rebuild();
    };
    dialog.exec();
}

}  // namespace carvetto::app
