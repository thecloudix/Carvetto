#include "gadget_dialogs.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QVariant>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>

namespace carvetto::app {

namespace {

using scripting::FormFieldKind;
using scripting::ScriptValue;

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

constexpr double kBig = 1.0e9;

std::vector<std::string> shortcutChoices() {
    std::vector<std::string> out;
    out.emplace_back();  // none
    for (const char* prefix : {"Ctrl+F", "Ctrl+Shift+F"}) {
        for (int i = 1; i <= 12; ++i) {
            out.push_back(prefix + std::to_string(i));
        }
    }
    return out;
}

}  // namespace

GadgetFormDialog::GadgetFormDialog(const scripting::FormSpec& spec, const GadgetFormChoices& choices, QWidget* parent)
    : QDialog(parent) {
    setWindowTitle(qs(spec.title));
    setObjectName(QStringLiteral("gadgetFormDialog"));
    auto* layout = new QVBoxLayout(this);
    auto* form = new QFormLayout();
    layout->addLayout(form);
    for (const scripting::FormField& f : spec.fields) {
        QWidget* w = nullptr;
        switch (f.kind) {
            case FormFieldKind::Number: {
                auto* box = new QDoubleSpinBox(this);
                box->setDecimals(6);
                box->setKeyboardTracking(false);
                box->setRange(f.min.value_or(-kBig), f.max.value_or(kBig));
                box->setValue(f.defaultValue.asNumber());
                w = box;
                break;
            }
            case FormFieldKind::Text: {
                auto* edit = new QLineEdit(qs(f.defaultValue.asString()), this);
                w = edit;
                break;
            }
            case FormFieldKind::Choice: {
                auto* combo = new QComboBox(this);
                for (const std::string& c : f.choices) {
                    combo->addItem(qs(c));
                }
                combo->setCurrentText(qs(f.defaultValue.asString()));
                w = combo;
                break;
            }
            case FormFieldKind::Bool: {
                auto* check = new QCheckBox(this);
                check->setChecked(f.defaultValue.asBool());
                w = check;
                break;
            }
            case FormFieldKind::Layer: {
                auto* combo = new QComboBox(this);
                for (const auto& l : choices.layers) {
                    combo->addItem(qs(l.second), QVariant::fromValue<qulonglong>(l.first));
                    if (static_cast<std::int64_t>(l.first) == f.defaultValue.asInt()) {
                        combo->setCurrentIndex(combo->count() - 1);
                    }
                }
                w = combo;
                break;
            }
            case FormFieldKind::Tool: {
                auto* combo = new QComboBox(this);
                combo->addItem(QStringLiteral("(none)"), QString());
                for (const auto& t : choices.tools) {
                    combo->addItem(qs(t.second), qs(t.first));
                    if (t.first == f.defaultValue.asString()) {
                        combo->setCurrentIndex(combo->count() - 1);
                    }
                }
                w = combo;
                break;
            }
        }
        w->setObjectName(qs("gadgetField_" + f.name));
        form->addRow(qs(f.label), w);
        rows_.push_back(Row{f, w});
    }
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    abort_ = buttons->addButton(QStringLiteral("Abort gadget"), QDialogButtonBox::DestructiveRole);
    connect(abort_, &QPushButton::clicked, this, [this] {
        if (onAbort) {
            onAbort();
        }
        reject();
    });
    layout->addWidget(buttons);
}

ScriptValue::Map GadgetFormDialog::values() const {
    ScriptValue::Map out;
    for (const Row& r : rows_) {
        ScriptValue v;
        switch (r.field.kind) {
            case FormFieldKind::Number: v = ScriptValue(static_cast<QDoubleSpinBox*>(r.widget)->value()); break;
            case FormFieldKind::Text: v = ScriptValue(static_cast<QLineEdit*>(r.widget)->text().toStdString()); break;
            case FormFieldKind::Choice: v = ScriptValue(static_cast<QComboBox*>(r.widget)->currentText().toStdString()); break;
            case FormFieldKind::Bool: v = ScriptValue(static_cast<QCheckBox*>(r.widget)->isChecked()); break;
            case FormFieldKind::Layer: {
                auto* combo = static_cast<QComboBox*>(r.widget);
                v = ScriptValue(static_cast<std::int64_t>(combo->currentData().toULongLong()));
                break;
            }
            case FormFieldKind::Tool: v = ScriptValue(static_cast<QComboBox*>(r.widget)->currentData().toString().toStdString()); break;
        }
        out.set(r.field.name, std::move(v));
    }
    return out;
}

bool GadgetFormDialog::setValue(const std::string& name, const ScriptValue& value) {
    for (const Row& r : rows_) {
        if (r.field.name != name) {
            continue;
        }
        switch (r.field.kind) {
            case FormFieldKind::Number:
                if (!value.isNumber()) return false;
                static_cast<QDoubleSpinBox*>(r.widget)->setValue(value.asNumber());
                return true;
            case FormFieldKind::Text:
                if (!value.isString()) return false;
                static_cast<QLineEdit*>(r.widget)->setText(qs(value.asString()));
                return true;
            case FormFieldKind::Choice: {
                if (!value.isString()) return false;
                auto* combo = static_cast<QComboBox*>(r.widget);
                const int i = combo->findText(qs(value.asString()));
                if (i < 0) return false;
                combo->setCurrentIndex(i);
                return true;
            }
            case FormFieldKind::Bool:
                if (!value.isBool()) return false;
                static_cast<QCheckBox*>(r.widget)->setChecked(value.asBool());
                return true;
            case FormFieldKind::Layer: {
                if (!value.isInt()) return false;
                auto* combo = static_cast<QComboBox*>(r.widget);
                const int i = combo->findData(QVariant::fromValue<qulonglong>(static_cast<std::uint64_t>(value.asInt())));
                if (i < 0) return false;
                combo->setCurrentIndex(i);
                return true;
            }
            case FormFieldKind::Tool: {
                if (!value.isString()) return false;
                auto* combo = static_cast<QComboBox*>(r.widget);
                const int i = combo->findData(qs(value.asString()));
                if (i < 0) return false;
                combo->setCurrentIndex(i);
                return true;
            }
        }
    }
    return false;
}

void showGadgetError(QWidget* parent, const viewmodel::GadgetErrorReport& report) {
    QMessageBox box(QMessageBox::Warning, qs(report.title), qs(report.summary), QMessageBox::Ok, parent);
    box.setObjectName(QStringLiteral("gadgetErrorDialog"));
    if (!report.detail.empty()) {
        box.setDetailedText(qs(report.detail));
    }
    box.exec();
}

GadgetShortcutsDialog::GadgetShortcutsDialog(viewmodel::GadgetManager& manager, QWidget* parent)
    : QDialog(parent), manager_(manager) {
    setWindowTitle(QStringLiteral("Gadget Shortcuts"));
    setObjectName(QStringLiteral("gadgetShortcutsDialog"));
    auto* layout = new QVBoxLayout(this);
    top_ = layout;
    layout->addWidget(new QLabel(QStringLiteral("Choose a Ctrl+F-key for each gadget. A key another gadget uses is refused."), this));
    body_ = new QWidget(this);
    layout->addWidget(body_);
    message_ = new QLabel(this);
    layout->addWidget(message_);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close, this);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    layout->addWidget(buttons);
    reload();
}

void GadgetShortcutsDialog::reload() {
    // A fresh body replaces the old one (deleted later: this runs inside a combo box's own signal).
    loading_ = true;
    combos_.clear();
    QWidget* fresh = new QWidget(this);
    auto* form = new QFormLayout(fresh);
    const std::vector<std::string> choices = shortcutChoices();
    for (const auto& row : manager_.shortcuts()) {
        auto* combo = new QComboBox(fresh);
        for (const std::string& c : choices) {
            combo->addItem(c.empty() ? QStringLiteral("(none)") : qs(c), qs(c));
        }
        combo->setCurrentIndex(std::max(0, combo->findData(qs(row.shortcut))));
        const std::size_t index = combos_.size();
        connect(combo, &QComboBox::activated, this, [this, index, combo](int i) {
            if (loading_) {
                return;
            }
            setShortcut(index, combo->itemData(i).toString().toStdString());
        });
        QString label = qs(row.name);
        if (!row.note.empty()) {
            label += QStringLiteral(" (") + qs(row.note) + QStringLiteral(")");
        }
        form->addRow(label, combo);
        combos_.push_back(combo);
    }
    top_->replaceWidget(body_, fresh);
    body_->hide();
    body_->deleteLater();
    body_ = fresh;
    loading_ = false;
}

std::string GadgetShortcutsDialog::setShortcut(std::size_t row, const std::string& shortcut) {
    const std::string err = manager_.assignShortcut(row, shortcut);
    if (!err.empty()) {
        message_->setText(qs(err));
        if (row < combos_.size()) {
            const std::string current = manager_.gadgets()[row].shortcut;
            combos_[row]->setCurrentIndex(std::max(0, combos_[row]->findData(qs(current))));
        }
        return err;
    }
    message_->setText(QString());
    reload();
    if (onChanged) {
        onChanged();
    }
    return {};
}

QString GadgetShortcutsDialog::message() const {
    return message_->text();
}

std::string GadgetShortcutsDialog::shortcutOf(std::size_t row) const {
    return row < combos_.size() ? combos_[row]->currentData().toString().toStdString() : std::string();
}

std::optional<scripting::InstallResult> installGadgetDialog(QWidget* parent, viewmodel::GadgetManager& manager,
                                                            const std::filesystem::path& folder,
                                                            const std::function<bool(const QString&)>& ask) {
    const auto confirm = [&](const QString& title, const QString& text) {
        if (ask) {
            return ask(text);
        }
        return QMessageBox::question(parent, title, text, QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes;
    };
    const scripting::InstallResult seen = manager.inspect(folder);
    if (!seen.ok) {
        if (!ask) {
            QMessageBox::warning(parent, QStringLiteral("Install New Gadget"),
                                 QStringLiteral("This folder is not a gadget: ") + qs(seen.error));
        }
        return std::nullopt;
    }
    const scripting::GadgetManifest& m = seen.gadget.manifest;
    QString text = QStringLiteral("Install gadget \"") + qs(m.name) + QStringLiteral("\" (version ") + qs(m.version) +
                   QStringLiteral(")?\n\nMenu: Gadgets > ") + qs(m.menu);
    if (!m.shortcut.empty()) {
        text += QStringLiteral("\nShortcut: ") + qs(m.shortcut);
    }
    if (!m.description.empty()) {
        text += QStringLiteral("\n\n") + qs(m.description);
    }
    if (!seen.gadget.problem.empty()) {
        text += QStringLiteral("\n\nIt cannot run here: ") + qs(seen.gadget.problem) + QStringLiteral(".");
    }
    text += QStringLiteral("\n\nA gadget is a script that can change your drawing. It cannot read your files or run programs, "
                           "and one run is one Undo step. Install only gadgets you trust.");
    if (!confirm(QStringLiteral("Install New Gadget"), text)) {
        return std::nullopt;
    }
    scripting::InstallResult r = manager.install(folder, false);
    if (!r.ok && r.error.find("already installed") != std::string::npos) {
        if (!confirm(QStringLiteral("Install New Gadget"),
                     qs(r.error) + QStringLiteral(". Replace it with this version?"))) {
            return std::nullopt;
        }
        r = manager.install(folder, true);
    }
    if (!r.ok && !ask) {
        QMessageBox::warning(parent, QStringLiteral("Install New Gadget"), qs(r.error));
    }
    return r;
}

}  // namespace carvetto::app
