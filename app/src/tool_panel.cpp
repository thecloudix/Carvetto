#include "tool_panel.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QString>
#include <QVBoxLayout>

#include "canvas_widget.hpp"
#include "dimension_menu.hpp"
#include "carvetto/document/units.hpp"
#include "carvetto/viewmodel/file_session.hpp"
#include "carvetto/viewmodel/starter_font.hpp"
#include "carvetto/viewmodel/text_tool.hpp"

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
    } else if (tool == ToolKind::NodeEdit) {
        viewmodel::NodeEditor& editor = ctl.nodeEditor();
        auto* mirror = new QComboBox(this);
        mirror->addItems({QStringLiteral("Off"), QStringLiteral("Horizontal"), QStringLiteral("Vertical")});
        mirror->setCurrentIndex(static_cast<int>(editor.options().mirror));
        form_->addRow(QStringLiteral("Mirror mode"), mirror);
        connect(mirror, &QComboBox::currentIndexChanged, this, [this](int i) {
            canvas_->tools().nodeEditor().setMirror(static_cast<viewmodel::MirrorMode>(i));
        });
        auto* keep = new QCheckBox(QStringLiteral("Keep Bezier tangency"), this);
        keep->setChecked(editor.options().keepBezierTangency);
        form_->addRow(QString(), keep);
        connect(keep, &QCheckBox::toggled, this,
                [this](bool on) { canvas_->tools().nodeEditor().options().keepBezierTangency = on; });
    } else if (tool == ToolKind::Text) {
        addTextRows();
    } else if (tool == ToolKind::Dimension) {
        addDimensionRows();
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

void ToolPanel::addDimensionRows() {
    viewmodel::DimensionTool& dt = canvas_->tools().dimensionTool();
    auto* kind = new QComboBox(this);
    kind->setObjectName(QStringLiteral("dimensionKindCombo"));
    const std::pair<const char*, document::DimensionKind> kinds[] = {
        {"Aligned", document::DimensionKind::Aligned},
        {"Horizontal", document::DimensionKind::Linear},
        {"Vertical", document::DimensionKind::Linear},
        {"Angular", document::DimensionKind::Angular},
        {"Radius", document::DimensionKind::Radial},
        {"Diameter", document::DimensionKind::Radial}};
    for (const auto& [name, k] : kinds) {
        kind->addItem(QString::fromUtf8(name), static_cast<int>(k));
    }
    // Rows 1 and 5 are the vertical linear and the diameter variants.
    int row = 0;
    switch (dt.options().kind) {
        case document::DimensionKind::Aligned: row = 0; break;
        case document::DimensionKind::Linear: row = dt.options().vertical ? 2 : 1; break;
        case document::DimensionKind::Angular: row = 3; break;
        case document::DimensionKind::Radial: row = dt.options().diameter ? 5 : 4; break;
    }
    kind->setCurrentIndex(row);
    form_->addRow(QStringLiteral("Kind"), kind);
    connect(kind, &QComboBox::currentIndexChanged, this, [this, kind](int index) {
        viewmodel::DimensionOptions o = canvas_->tools().dimensionTool().options();
        o.kind = static_cast<document::DimensionKind>(kind->itemData(index).toInt());
        o.vertical = index == 2;
        o.diameter = index == 5;
        status_->setText(QString::fromStdString(canvas_->tools().dimensionTool().setOptions(o).message));
        canvas_->update();
    });
    auto* options = new QPushButton(QStringLiteral("Options..."), this);
    options->setObjectName(QStringLiteral("dimensionOptionsButton"));
    form_->addRow(options);
    connect(options, &QPushButton::clicked, this, [this]() {
        auto& tool = canvas_->tools().dimensionTool();
        if (tool.document() == nullptr) {
            return;
        }
        DimensionDialog dialog(*tool.document(), tool.options(), false, this);
        if (dialog.exec() == QDialog::Accepted) {
            status_->setText(QString::fromStdString(tool.setOptions(dialog.options()).message));
            rebuild();
        }
    });
    form_->addRow(new QLabel(QStringLiteral("Click the points, then the line position.\nShift-click a dimension to edit it."), this));
}

void ToolPanel::addTextRows() {
    viewmodel::TextEditor& ed = canvas_->tools().textEditor();
    if (!ed.hasFont()) {
        const auto starter = viewmodel::starterFontBytes();
        ed.setFont(std::vector<std::uint8_t>(starter.begin(), starter.end()), viewmodel::starterFontName());
    }
    const auto report = [this](const viewmodel::TextEditResult& r) {
        status_->setText(qs(r.message));
        canvas_->update();
    };

    auto* fontRow = new QWidget(this);
    auto* fontLayout = new QHBoxLayout(fontRow);
    fontLayout->setContentsMargins(0, 0, 0, 0);
    auto* fontName = new QLabel(qs(ed.fontName()), fontRow);
    fontName->setObjectName(QStringLiteral("textFontName"));
    auto* choose = new QPushButton(QStringLiteral("Choose..."), fontRow);
    choose->setToolTip(QStringLiteral("Pick a TrueType or OpenType font file; it is embedded in the document"));
    fontLayout->addWidget(fontName, 1);
    fontLayout->addWidget(choose);
    form_->addRow(QStringLiteral("Font"), fontRow);
    connect(choose, &QPushButton::clicked, this, [this, fontName, report]() {
        const QString file = QFileDialog::getOpenFileName(this, QStringLiteral("Choose a font"), QString(),
                                                          QStringLiteral("Fonts (*.ttf *.otf *.ttc)"));
        if (file.isEmpty()) {
            return;
        }
        const viewmodel::TextEditResult r =
            canvas_->tools().textEditor().setFontFile(viewmodel::pathFromUtf8(file.toStdString()));
        if (r.ok) {
            fontName->setText(qs(canvas_->tools().textEditor().fontName()));
        }
        report(r);
    });

    auto* mode = new QComboBox(this);
    mode->addItems({QStringLiteral("Point (click)"), QStringLiteral("In a box (drag)"),
                    QStringLiteral("On a curve (click a curve)")});
    mode->setCurrentIndex(static_cast<int>(ed.settings().mode));
    form_->addRow(QStringLiteral("Mode"), mode);
    connect(mode, &QComboBox::currentIndexChanged, this, [this, report](int i) {
        report(canvas_->tools().textEditor().setMode(static_cast<document::TextMode>(i)));
        if (canvas_->onDocumentEdited) {
            canvas_->onDocumentEdited();
        }
    });

    const document::Units units = canvas_->tools().displayUnits();
    auto* size = new QDoubleSpinBox(this);
    size->setDecimals(3);
    size->setRange(0.01, 10000.0);
    size->setValue(document::toDisplay(ed.settings().sizeMm, units));
    size->setSuffix(QStringLiteral(" ") + QString::fromUtf8(document::unitSuffix(units).data()));
    form_->addRow(QStringLiteral("Size (em)"), size);
    connect(size, &QDoubleSpinBox::valueChanged, this, [this, report, units](double v) {
        report(canvas_->tools().textEditor().setSize(document::fromDisplay(v, units)));
    });

    auto* align = new QComboBox(this);
    align->addItems({QStringLiteral("Left"), QStringLiteral("Centre"), QStringLiteral("Right")});
    align->setCurrentIndex(static_cast<int>(ed.settings().align));
    form_->addRow(QStringLiteral("Alignment"), align);
    connect(align, &QComboBox::currentIndexChanged, this, [this, report](int i) {
        report(canvas_->tools().textEditor().setAlign(static_cast<document::TextAlign>(i)));
    });

    auto* letter = new QDoubleSpinBox(this);
    letter->setDecimals(3);
    letter->setRange(-1000.0, 1000.0);
    letter->setValue(ed.settings().letterSpacingMm);
    letter->setSuffix(QStringLiteral(" mm"));
    form_->addRow(QStringLiteral("Letter spacing"), letter);
    connect(letter, &QDoubleSpinBox::valueChanged, this,
            [this, report](double v) { report(canvas_->tools().textEditor().setLetterSpacing(v)); });

    auto* line = new QDoubleSpinBox(this);
    line->setDecimals(2);
    line->setRange(0.0, 100.0);
    line->setSingleStep(0.1);
    line->setValue(ed.settings().lineSpacing);
    form_->addRow(QStringLiteral("Line spacing"), line);
    connect(line, &QDoubleSpinBox::valueChanged, this,
            [this, report](double v) { report(canvas_->tools().textEditor().setLineSpacing(v)); });

    auto* kerning = new QCheckBox(QStringLiteral("Kerning"), this);
    kerning->setChecked(ed.settings().kerning);
    form_->addRow(QString(), kerning);
    connect(kerning, &QCheckBox::toggled, this,
            [this, report](bool on) { report(canvas_->tools().textEditor().setKerning(on)); });

    auto* place = new QPushButton(QStringLiteral("Place text (Ctrl+Enter)"), this);
    form_->addRow(QString(), place);
    connect(place, &QPushButton::clicked, this, [this, report]() {
        const viewmodel::ToolResult r = canvas_->tools().key(viewmodel::ToolKey::Enter);
        status_->setText(qs(r.message));
        if (r.committed && canvas_->onDocumentEdited) {
            canvas_->onDocumentEdited();
        }
        if (r.selectionChanged && canvas_->onSelectionChanged) {
            canvas_->onSelectionChanged();
        }
        canvas_->update();
        (void)report;
    });
    auto* convert = new QPushButton(QStringLiteral("Convert selected text to curves"), this);
    form_->addRow(QString(), convert);
    connect(convert, &QPushButton::clicked, this, [this, report]() {
        const viewmodel::TextEditResult r = canvas_->tools().textEditor().convertSelectionToCurves();
        if (r.committed && canvas_->onDocumentEdited) {
            canvas_->onDocumentEdited();
        }
        report(r);
    });
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
