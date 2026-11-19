#include "rotary_layout_dialog.hpp"

#include <QCoreApplication>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QVariant>
#include <utility>
#include <vector>

namespace carvetto::app {

namespace {

QString qs(const std::string& s) {
    return QString::fromUtf8(s.data(), static_cast<qsizetype>(s.size()));
}

}  // namespace

RotaryLayoutDialog::RotaryLayoutDialog(viewmodel::RotaryLayoutForm form, PreviewFn previewFn, QWidget* parent)
    : QDialog(parent), form_(std::move(form)), previewFn_(std::move(previewFn)) {
    const bool fluting = form_.kind() == viewmodel::RotaryLayoutKind::Fluting;
    setObjectName(QStringLiteral("rotaryLayoutDialog"));
    setWindowTitle(fluting ? QCoreApplication::translate("RotaryLayoutDialog", "Wrapped Fluting Layout") : QCoreApplication::translate("RotaryLayoutDialog", "Wrapped Spiral Layout"));

    auto* layout = new QVBoxLayout(this);
    status_ = new QLabel(this);
    status_->setObjectName(QStringLiteral("layoutStatus"));
    status_->setWordWrap(true);
    layout->addWidget(status_);
    error_ = new QLabel(this);
    error_->setObjectName(QStringLiteral("layoutError"));
    error_->setWordWrap(true);
    error_->setStyleSheet(QStringLiteral("color: #b00020;"));
    layout->addWidget(error_);
    buttons_ = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons_, &QDialogButtonBox::accepted, this, [this]() {
        if (preview_.ok) {
            accept();
        }
    });
    connect(buttons_, &QDialogButtonBox::rejected, this, [this]() { reject(); });
    layout->addWidget(buttons_);

    rebuildRows();
    refreshPreview();
}

void RotaryLayoutDialog::setPreviewSink(PreviewSink sink) {
    sink_ = std::move(sink);
    if (sink_) {
        sink_(preview_);
    }
}

QString RotaryLayoutDialog::message() const {
    return error_->text();
}

QLineEdit* RotaryLayoutDialog::fieldEdit(const std::string& id) const {
    const auto it = edits_.find(id);
    return it == edits_.end() ? nullptr : it->second;
}

QComboBox* RotaryLayoutDialog::choiceBox(const std::string& id) const {
    const auto it = combos_.find(id);
    return it == combos_.end() ? nullptr : it->second;
}

void RotaryLayoutDialog::rebuildRows() {
    building_ = true;
    // A fresh container each time: the old one may hold the combo box whose signal led here, so it goes later.
    QWidget* fresh = new QWidget(this);
    if (rows_ != nullptr) {
        rows_->hide();
        static_cast<QVBoxLayout*>(this->layout())->replaceWidget(rows_, fresh);
        rows_->deleteLater();
    } else {
        static_cast<QVBoxLayout*>(this->layout())->insertWidget(0, fresh);
    }
    rows_ = fresh;
    edits_.clear();
    combos_.clear();
    auto* form = new QFormLayout(rows_);
    // Choices first (the mode decides which fields exist), then the numeric fields.
    for (const viewmodel::ChoiceField& c : form_.choices()) {
        auto* box = new QComboBox(rows_);
        box->setObjectName(qs("choice_" + c.id));
        for (const std::string& o : c.options) {
            box->addItem(qs(o));
        }
        box->setCurrentIndex(c.selected);
        form->addRow(qs(c.label + ":"), box);
        combos_[c.id] = box;
        const std::string id = c.id;
        connect(box, &QComboBox::activated, this, [this, id](int index) { chooseOption(id, index); });
    }
    for (const viewmodel::NumericField& f : form_.fields()) {
        auto* edit = new QLineEdit(qs(f.text), rows_);
        edit->setObjectName(qs("field_" + f.id));
        form->addRow(qs(f.label + ":"), edit);
        edits_[f.id] = edit;
        const std::string id = f.id;
        connect(edit, &QLineEdit::editingFinished, this, [this, id, edit]() {
            if (!building_) {
                typeField(id, edit->text().toStdString());
            }
        });
    }
    fresh->show();
    building_ = false;
}

void RotaryLayoutDialog::syncFromForm() {
    // Show the form's own (normalised) text again, e.g. after a refused entry.
    for (const viewmodel::NumericField& f : form_.fields()) {
        QLineEdit* e = fieldEdit(f.id);
        if (e != nullptr && e->text() != qs(f.text)) {
            e->setText(qs(f.text));
        }
    }
}

bool RotaryLayoutDialog::typeField(const std::string& id, const std::string& text) {
    QLineEdit* edit = fieldEdit(id);
    if (edit == nullptr) {
        return false;
    }
    if (edit->text().toStdString() != text) {
        edit->setText(qs(text));
    }
    const viewmodel::FieldResult r = form_.setField(id, text);
    fieldMessage_ = r.ok ? QString() : qs(r.message);
    if (!r.ok) {
        syncFromForm();
    }
    refreshPreview();
    return r.ok;
}

bool RotaryLayoutDialog::chooseOption(const std::string& id, int option) {
    QComboBox* box = choiceBox(id);
    if (box == nullptr) {
        return false;
    }
    const viewmodel::FieldResult r = form_.setChoice(id, option);
    fieldMessage_ = r.ok ? QString() : qs(r.message);
    if (r.ok) {
        // A choice (the spiral's mode) can change which fields exist.
        rebuildRows();
    } else if (box->currentIndex() != option) {
        syncFromForm();
    }
    refreshPreview();
    return r.ok;
}

void RotaryLayoutDialog::refreshPreview() {
    preview_ = previewFn_ ? previewFn_(form_) : viewmodel::RotaryLayoutPreview{};
    QString shown = fieldMessage_;
    if (!preview_.ok) {
        shown = shown.isEmpty() ? qs(preview_.message) : shown + QStringLiteral("\n") + qs(preview_.message);
        status_->setText(QString());
    } else {
        QString text = QCoreApplication::translate("RotaryLayoutDialog", "%1 vectors").arg(preview_.contours.size());
        if (preview_.turns > 0.0) {
            text += QCoreApplication::translate("RotaryLayoutDialog", ", %1 turns").arg(preview_.turns, 0, 'g', 6);
        }
        if (preview_.pitch > 0.0) {
            text += QCoreApplication::translate("RotaryLayoutDialog", ", pitch %1 mm").arg(preview_.pitch, 0, 'g', 6);
        }
        for (const std::string& w : preview_.warnings) {
            text += QStringLiteral("\n") + qs(w);
        }
        status_->setText(text);
    }
    error_->setText(shown);
    if (QPushButton* ok = buttons_->button(QDialogButtonBox::Ok)) {
        ok->setEnabled(preview_.ok);
    }
    if (sink_) {
        sink_(preview_);
    }
}

}  // namespace carvetto::app
