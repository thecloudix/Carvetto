#include "properties_panel.hpp"

#include <QComboBox>
#include <QFormLayout>
#include <QLabel>
#include <QLayoutItem>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>
#include <utility>

namespace carvetto::app {

namespace {

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

const char* const kBadStyle = "border: 1px solid red;";

}  // namespace

PropertiesPanel::PropertiesPanel(DocumentGetter getter, QWidget* parent) : PanelBase(std::move(getter), parent) {
    auto* outer = new QVBoxLayout(this);
    title_ = new QLabel(this);
    outer->addWidget(title_);
    auto* host = new QWidget(this);
    form_ = new QFormLayout(host);
    form_->setContentsMargins(0, 0, 0, 0);
    outer->addWidget(host);
    status_ = new QLabel(this);
    status_->setWordWrap(true);
    outer->addWidget(status_);
    outer->addStretch(1);
    refresh();
}

void PropertiesPanel::clearRows() {
    // Rows are handed to deleteLater: this can run from inside the signal of one of their own widgets.
    while (form_->rowCount() > 0) {
        const QFormLayout::TakeRowResult taken = form_->takeRow(0);
        for (QLayoutItem* item : {taken.labelItem, taken.fieldItem}) {
            if (item != nullptr) {
                if (item->widget() != nullptr) {
                    item->widget()->hide();
                    item->widget()->deleteLater();
                }
                delete item;
            }
        }
    }
    rows_.clear();
}

void PropertiesPanel::setStatus(const std::string& text) {
    status_->setText(qs(text));
    status_->setStyleSheet(text.empty() ? QString() : QStringLiteral("color: #b00020;"));
}

std::vector<std::string> PropertiesPanel::fieldIds() const {
    std::vector<std::string> ids;
    for (const Row& r : rows_) {
        ids.push_back(r.id);
    }
    return ids;
}

std::string PropertiesPanel::textOf(const std::string& id) const {
    for (const Row& r : rows_) {
        if (r.id == id) {
            if (auto* line = qobject_cast<QLineEdit*>(r.input)) {
                return line->text().toStdString();
            }
            if (auto* box = qobject_cast<QPlainTextEdit*>(r.input)) {
                return box->toPlainText().toStdString();
            }
            if (auto* combo = qobject_cast<QComboBox*>(r.input)) {
                return combo->currentText().toStdString();
            }
        }
    }
    return {};
}

void PropertiesPanel::rebuild(const viewmodel::PropertyView& view) {
    clearRows();
    for (const viewmodel::PropertyField& f : view.fields) {
        Row row;
        row.id = f.id;
        row.kind = f.kind;
        QString label = qs(f.label);
        if (!f.suffix.empty()) {
            label += QStringLiteral(" (") + qs(f.suffix) + QStringLiteral(")");
        }
        const std::string id = f.id;
        if (f.kind == viewmodel::PropertyKind::Choice) {
            auto* combo = new QComboBox(this);
            for (const std::string& c : f.choices) {
                combo->addItem(qs(c));
            }
            combo->setCurrentIndex(f.choice);
            combo->setEnabled(f.editable);
            connect(combo, &QComboBox::activated, this, [this, id](int) { commitRow(id); });
            row.editor = row.input = combo;
        } else if (f.multiline) {
            auto* host = new QWidget(this);
            auto* box = new QVBoxLayout(host);
            box->setContentsMargins(0, 0, 0, 0);
            auto* edit = new QPlainTextEdit(host);
            edit->setPlainText(qs(f.text));
            edit->setMaximumHeight(90);
            edit->setEnabled(f.editable);
            auto* apply = new QPushButton(QStringLiteral("Apply text"), host);
            apply->setEnabled(f.editable);
            box->addWidget(edit);
            box->addWidget(apply);
            connect(apply, &QPushButton::clicked, this, [this, id]() { commitRow(id); });
            row.editor = host;
            row.input = edit;
        } else {
            auto* edit = new QLineEdit(qs(f.text), this);
            edit->setEnabled(f.editable);
            connect(edit, &QLineEdit::editingFinished, this, [this, id]() { commitRow(id); });
            row.editor = row.input = edit;
        }
        row.shown = f.kind == viewmodel::PropertyKind::Choice ? std::to_string(f.choice) : f.text;
        form_->addRow(label, row.editor);
        rows_.push_back(std::move(row));
    }
}

void PropertiesPanel::refresh() {
    const document::Document* d = doc();
    const viewmodel::PropertyView view = d != nullptr ? viewmodel::describeSelection(*d) : viewmodel::PropertyView{};
    titleText_ = view.title.empty() ? "No selection" : view.title;
    title_->setText(QStringLiteral("<b>%1</b>").arg(qs(titleText_).toHtmlEscaped()));

    bool same = view.fields.size() == rows_.size();
    for (std::size_t i = 0; same && i < rows_.size(); ++i) {
        same = rows_[i].id == view.fields[i].id && rows_[i].kind == view.fields[i].kind;
        if (same && view.fields[i].kind == viewmodel::PropertyKind::Choice) {
            auto* combo = qobject_cast<QComboBox*>(rows_[i].input);
            same = combo != nullptr && static_cast<std::size_t>(combo->count()) == view.fields[i].choices.size();
        }
    }
    updating_ = true;
    if (!same) {
        rebuild(view);
    } else {
        for (std::size_t i = 0; i < rows_.size(); ++i) {
            const viewmodel::PropertyField& f = view.fields[i];
            Row& row = rows_[i];
            row.input->setEnabled(f.editable);
            row.input->setStyleSheet(QString());
            if (row.input->hasFocus()) {
                continue;  // never overwrite what the user is typing
            }
            if (auto* line = qobject_cast<QLineEdit*>(row.input)) {
                line->setText(qs(f.text));
                row.shown = f.text;
            } else if (auto* box = qobject_cast<QPlainTextEdit*>(row.input)) {
                box->setPlainText(qs(f.text));
                row.shown = f.text;
            } else if (auto* combo = qobject_cast<QComboBox*>(row.input)) {
                combo->setCurrentIndex(f.choice);
                row.shown = std::to_string(f.choice);
            }
        }
    }
    updating_ = false;
}

bool PropertiesPanel::commit(const std::string& id, const std::string& text) {
    for (Row& row : rows_) {
        if (row.id != id) {
            continue;
        }
        if (auto* line = qobject_cast<QLineEdit*>(row.input)) {
            line->setText(qs(text));
        } else if (auto* box = qobject_cast<QPlainTextEdit*>(row.input)) {
            box->setPlainText(qs(text));
        } else if (auto* combo = qobject_cast<QComboBox*>(row.input)) {
            combo->setCurrentIndex(text.empty() ? -1 : qs(text).toInt());
        }
        return commitRow(id);
    }
    return false;
}

bool PropertiesPanel::commitRow(const std::string& id) {
    if (updating_) {
        return false;
    }
    document::Document* d = doc();
    if (d == nullptr) {
        return false;
    }
    Row* found = nullptr;
    for (Row& r : rows_) {
        if (r.id == id) {
            found = &r;
        }
    }
    if (found == nullptr) {
        return false;
    }
    std::string text;
    if (auto* line = qobject_cast<QLineEdit*>(found->input)) {
        text = line->text().toStdString();
    } else if (auto* box = qobject_cast<QPlainTextEdit*>(found->input)) {
        text = box->toPlainText().toStdString();
    } else if (auto* combo = qobject_cast<QComboBox*>(found->input)) {
        text = std::to_string(combo->currentIndex());
    }
    if (text == found->shown) {
        found->input->setStyleSheet(QString());
        return true;  // nothing changed (also when the field was only tabbed through)
    }
    QWidget* input = found->input;
    const viewmodel::EditResult r = viewmodel::setProperty(*d, id, text);
    if (!r.ok) {
        input->setStyleSheet(QString::fromUtf8(kBadStyle));
        setStatus(r.message);
        if (onMessage) {
            onMessage(r.message);
        }
        refresh();  // puts the old value back (the red mark is cleared with it; the status stays)
        input->setStyleSheet(QString::fromUtf8(kBadStyle));
        return false;
    }
    setStatus({});
    if (r.changed && onEdited) {
        onEdited();  // the window refreshes every view, this panel included
    } else {
        refresh();
    }
    return true;
}

}  // namespace carvetto::app
