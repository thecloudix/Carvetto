#include "tool_panel.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QSpinBox>
#include <QString>
#include <QVBoxLayout>

#include "canvas_widget.hpp"
#include "carvetto/document/units.hpp"

namespace carvetto::app {

using viewmodel::FieldKind;
using viewmodel::ToolKind;

namespace {

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

}  // namespace

ToolPanel::ToolPanel(CanvasWidget* canvas, QWidget* parent) : QWidget(parent), canvas_(canvas) {
    auto* outer = new QVBoxLayout(this);
    title_ = new QLabel(this);
    outer->addWidget(title_);
    auto* formHost = new QWidget(this);
    form_ = new QFormLayout(formHost);
    form_->setContentsMargins(0, 0, 0, 0);
    outer->addWidget(formHost);
    status_ = new QLabel(this);
    status_->setWordWrap(true);
    outer->addWidget(status_);
    outer->addStretch(1);
    rebuild();
}

void ToolPanel::clearForm() {
    edits_.clear();
    while (form_->rowCount() > 0) {
        form_->removeRow(0);
    }
}

void ToolPanel::addOptionRows(ToolKind tool) {
    viewmodel::ToolController& ctl = canvas_->tools();
    if (tool == ToolKind::Rectangle) {
        auto* corner = new QComboBox(this);
        corner->addItems({QStringLiteral("Square"), QStringLiteral("Rounded"), QStringLiteral("Chamfered")});
        corner->setCurrentIndex(static_cast<int>(ctl.options().cornerStyle));
        form_->addRow(QStringLiteral("Corners"), corner);
        connect(corner, &QComboBox::currentIndexChanged, this, [this](int i) {
            canvas_->tools().options().cornerStyle = static_cast<document::RectangleParams::CornerStyle>(i);
            canvas_->update();
        });
        auto* size = new QLineEdit(this);
        size->setPlaceholderText(QStringLiteral("corner size"));
        form_->addRow(QStringLiteral("Corner size"), size);
        connect(size, &QLineEdit::textEdited, this, [this, size](const QString& t) {
            const viewmodel::ParsedNumber n = viewmodel::parseNumber(
                t.toStdString(), FieldKind::Length, canvas_->tools().displayUnits());
            const bool ok = n.ok && n.value >= 0.0;
            size->setStyleSheet(ok || t.isEmpty() ? QString() : QStringLiteral("border: 1px solid red;"));
            if (ok) {
                canvas_->tools().options().cornerSizeMm = n.value;
                canvas_->update();
            }
        });
    } else if (tool == ToolKind::Polygon) {
        auto* sides = new QSpinBox(this);
        sides->setRange(3, 1000);
        sides->setValue(ctl.options().polygonSides);
        form_->addRow(QStringLiteral("Sides"), sides);
        connect(sides, &QSpinBox::valueChanged, this, [this](int v) {
            canvas_->tools().options().polygonSides = v;
            canvas_->update();
        });
    } else if (tool == ToolKind::Star) {
        auto* points = new QSpinBox(this);
        points->setRange(3, 1000);
        points->setValue(ctl.options().starPoints);
        form_->addRow(QStringLiteral("Points"), points);
        connect(points, &QSpinBox::valueChanged, this, [this](int v) {
            canvas_->tools().options().starPoints = v;
            canvas_->update();
        });
        auto* inner = new QSpinBox(this);
        inner->setRange(1, 99);
        inner->setSuffix(QStringLiteral(" %"));
        inner->setValue(static_cast<int>(ctl.options().starInnerPercent));
        form_->addRow(QStringLiteral("Inner radius"), inner);
        connect(inner, &QSpinBox::valueChanged, this, [this](int v) {
            canvas_->tools().options().starInnerPercent = static_cast<double>(v);
            canvas_->update();
        });
    } else if (tool == ToolKind::Arc) {
        auto* mode = new QComboBox(this);
        mode->addItems({QStringLiteral("Through 3 points"), QStringLiteral("Centre, start, end"),
                        QStringLiteral("Start, end, radius"), QStringLiteral("Start, end, height")});
        mode->setCurrentIndex(static_cast<int>(ctl.options().arcMode));
        form_->addRow(QStringLiteral("Arc"), mode);
        connect(mode, &QComboBox::currentIndexChanged, this, [this](int i) {
            canvas_->tools().key(viewmodel::ToolKey::Escape);
            canvas_->tools().options().arcMode = static_cast<viewmodel::ArcMode>(i);
            rebuild();
            canvas_->update();
        });
        auto* dir = new QComboBox(this);
        dir->addItems({QStringLiteral("Counter-clockwise"), QStringLiteral("Clockwise")});
        dir->setCurrentIndex(ctl.options().arcDirection == geometry::ArcDirection::CCW ? 0 : 1);
        form_->addRow(QStringLiteral("Direction"), dir);
        connect(dir, &QComboBox::currentIndexChanged, this, [this](int i) {
            canvas_->tools().options().arcDirection =
                i == 0 ? geometry::ArcDirection::CCW : geometry::ArcDirection::CW;
            canvas_->update();
        });
        auto* large = new QCheckBox(QStringLiteral("Larger arc"), this);
        large->setChecked(ctl.options().arcLarge);
        form_->addRow(QString(), large);
        connect(large, &QCheckBox::toggled, this, [this](bool on) {
            canvas_->tools().options().arcLarge = on;
            canvas_->update();
        });
    }
}

void ToolPanel::rebuild() {
    clearForm();
    viewmodel::ToolController& ctl = canvas_->tools();
    title_->setText(QStringLiteral("<b>%1</b>").arg(QString::fromUtf8(viewmodel::toolName(ctl.tool()))));
    status_->clear();
    addOptionRows(ctl.tool());
    const document::Units units = ctl.displayUnits();
    for (const viewmodel::NumericField& f : ctl.fields()) {
        auto* edit = new QLineEdit(this);
        QString label = qs(f.label);
        if (f.kind == FieldKind::Length) {
            label += QStringLiteral(" (%1)").arg(QString::fromUtf8(document::unitSuffix(units).data()));
        } else if (f.kind == FieldKind::Angle) {
            label += QStringLiteral(" (deg)");
        }
        form_->addRow(label, edit);
        edits_[f.id] = edit;
        const std::string id = f.id;
        connect(edit, &QLineEdit::textEdited, this, [this, id, edit](const QString& t) {
            const viewmodel::FieldResult r = canvas_->tools().setField(id, t.toStdString());
            edit->setStyleSheet(r.ok ? QString() : QStringLiteral("border: 1px solid red;"));
            edit->setToolTip(qs(r.message));
            status_->setText(qs(r.message));
            canvas_->update();
        });
        connect(edit, &QLineEdit::returnPressed, this, [this]() {
            const viewmodel::ToolResult r = canvas_->tools().commitNumeric();
            status_->setText(qs(r.message));
            if (r.committed) {
                for (auto& [key, e] : edits_) {
                    e->clear();
                    e->setStyleSheet(QString());
                }
                if (canvas_->onDocumentEdited) {
                    canvas_->onDocumentEdited();
                }
            }
            refreshValues();
            canvas_->update();
        });
    }
    refreshValues();
}

void ToolPanel::refreshValues() {
    for (const viewmodel::NumericField& f : canvas_->tools().fields()) {
        const auto it = edits_.find(f.id);
        if (it == edits_.end()) {
            continue;
        }
        QLineEdit* edit = it->second;
        edit->setPlaceholderText(qs(f.text));
        if (!f.typed && !edit->hasFocus() && !edit->text().isEmpty()) {
            edit->clear();  // the value was cleared by the controller (shape finished or cancelled)
            edit->setStyleSheet(QString());
        }
    }
}

bool ToolPanel::typeInto(const std::string& id, const std::string& text) {
    const auto it = edits_.find(id);
    if (it == edits_.end()) {
        return false;
    }
    it->second->setText(qs(text));
    const viewmodel::FieldResult r = canvas_->tools().setField(id, text);
    it->second->setStyleSheet(r.ok ? QString() : QStringLiteral("border: 1px solid red;"));
    return r.ok;
}

}  // namespace carvetto::app
