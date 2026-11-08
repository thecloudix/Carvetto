#include "toolpath_form_dialog.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>
#include <algorithm>
#include <utility>

#include "carvetto/document/units.hpp"
#include "carvetto/viewmodel/tools.hpp"
#include "carvetto/viewmodel/units_format.hpp"

namespace carvetto::app {

using document::ToolpathOperation;
using viewmodel::ToolpathField;
using viewmodel::ToolpathFieldKind;

namespace {

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

const char* const kBadStyle = "border: 1px solid red;";

enum BaseKind { BaseAll = 0, BaseObjects = 1, BaseLayers = 2 };

std::string sourceLabel(const tooling::ToolDatabase& db, const document::ToolSource& s) {
    if (s.machine.empty() && s.material.empty()) {
        return "Tool default";
    }
    const tooling::Machine* m = db.findMachine(s.machine);
    const tooling::Material* mat = db.findMaterial(s.material);
    return (m != nullptr ? m->name : s.machine) + " / " + (mat != nullptr ? mat->name : s.material);
}

}  // namespace

ToolpathFormDialog::ToolpathFormDialog(viewmodel::ToolpathForm form, tooling::ToolDatabase db, FormDocumentInfo info,
                                       bool editing, QWidget* parent)
    : QDialog(parent), form_(std::move(form)), db_(std::move(db)), info_(std::move(info)), editing_(editing) {
    setObjectName(QStringLiteral("ToolpathFormDialog"));
    setWindowTitle(QStringLiteral("%1 toolpath").arg(QString::fromUtf8(viewmodel::operationName(form_.operation()))));
    form_.refreshTools(db_);
    buildStatic();
    rebuildParameters();
    refreshStatus();
    resize(560, 720);
}

void ToolpathFormDialog::buildStatic() {
    building_ = true;
    auto* outer = new QVBoxLayout(this);

    auto* top = new QFormLayout();
    nameEdit_ = new QLineEdit(qs(form_.name()), this);
    nameEdit_->setObjectName(QStringLiteral("toolpathName"));
    top->addRow(QStringLiteral("Name"), nameEdit_);
    connect(nameEdit_, &QLineEdit::textEdited, this, [this](const QString& t) {
        const viewmodel::FieldResult r = form_.setName(t.toStdString());
        nameEdit_->setStyleSheet(r.ok ? QString() : QString::fromUtf8(kBadStyle));
        if (r.ok) {
            badFields_.erase("name");
        } else {
            badFields_.insert("name");
        }
        setMessage(r.message, !r.ok);
    });

    // Tool picker.
    toolCombo_ = new QComboBox(this);
    toolCombo_->setObjectName(QStringLiteral("toolCombo"));
    toolCombo_->addItem(QStringLiteral("(choose a tool)"), QString());
    const bool vcarve = form_.operation() == ToolpathOperation::VCarving;
    for (const tooling::Tool* t : viewmodel::toolsFor(db_, form_.operation(), false)) {
        toolCombo_->addItem(qs(t->name), qs(t->id));
    }
    top->addRow(vcarve ? QStringLiteral("Carving tool") : QStringLiteral("Tool"), toolCombo_);
    dataCombo_ = new QComboBox(this);
    dataCombo_->setObjectName(QStringLiteral("cuttingDataCombo"));
    top->addRow(QStringLiteral("Cutting data"), dataCombo_);
    toolInfo_ = new QLabel(this);
    toolInfo_->setWordWrap(true);
    top->addRow(QString(), toolInfo_);
    connect(toolCombo_, &QComboBox::currentIndexChanged, this, [this](int) {
        if (!building_) {
            applyTool();
        }
    });
    connect(dataCombo_, &QComboBox::currentIndexChanged, this, [this](int row) {
        if (building_ || row < 0) {
            return;
        }
        document::ToolSource source;
        source.toolId = toolCombo_->currentData().toString().toStdString();
        const tooling::Tool* tool = db_.findTool(source.toolId);
        if (tool == nullptr) {
            return;
        }
        const auto choices = viewmodel::cuttingDataChoices(*tool);
        if (static_cast<std::size_t>(row) >= choices.size()) {
            return;
        }
        const viewmodel::FieldResult r = form_.setTool(db_, choices[static_cast<std::size_t>(row)]);
        setMessage(r.message, !r.ok);
        refreshPlaceholders();
    });
    outer->addLayout(top);

    // Clearance tools (V-carve only).
    clearanceBox_ = new QGroupBox(QStringLiteral("Clearance tools (flat areas, in order)"), this);
    auto* cbox = new QVBoxLayout(clearanceBox_);
    clearanceList_ = new QListWidget(clearanceBox_);
    clearanceList_->setObjectName(QStringLiteral("clearanceList"));
    clearanceList_->setMaximumHeight(80);
    cbox->addWidget(clearanceList_);
    auto* crow = new QHBoxLayout();
    clearanceCombo_ = new QComboBox(clearanceBox_);
    clearanceCombo_->setObjectName(QStringLiteral("clearanceCombo"));
    for (const tooling::Tool* t : viewmodel::toolsFor(db_, form_.operation(), true)) {
        clearanceCombo_->addItem(qs(t->name), qs(t->id));
    }
    crow->addWidget(clearanceCombo_, 1);
    auto* add = new QPushButton(QStringLiteral("Add"), clearanceBox_);
    auto* remove = new QPushButton(QStringLiteral("Remove"), clearanceBox_);
    auto* up = new QPushButton(QStringLiteral("Up"), clearanceBox_);
    auto* down = new QPushButton(QStringLiteral("Down"), clearanceBox_);
    add->setObjectName(QStringLiteral("clearanceAdd"));
    for (QPushButton* b : {add, remove, up, down}) {
        crow->addWidget(b);
    }
    cbox->addLayout(crow);
    outer->addWidget(clearanceBox_);
    clearanceBox_->setVisible(vcarve);
    auto edit = [this](const std::function<void(std::vector<document::ToolSource>&)>& change) {
        std::vector<document::ToolSource> list = form_.clearanceTools();
        change(list);
        const viewmodel::FieldResult r = form_.setClearanceTools(db_, list);
        setMessage(r.message, !r.ok);
        applyClearance();
    };
    connect(add, &QPushButton::clicked, this, [this, edit]() {
        const std::string id = clearanceCombo_->currentData().toString().toStdString();
        if (id.empty()) {
            return;
        }
        edit([&](std::vector<document::ToolSource>& l) { l.push_back(document::ToolSource{id, "", ""}); });
    });
    connect(remove, &QPushButton::clicked, this, [this, edit]() {
        const int row = clearanceList_->currentRow();
        if (row < 0) {
            return;
        }
        edit([&](std::vector<document::ToolSource>& l) {
            if (static_cast<std::size_t>(row) < l.size()) {
                l.erase(l.begin() + row);
            }
        });
    });
    connect(up, &QPushButton::clicked, this, [this, edit]() {
        const int row = clearanceList_->currentRow();
        if (row <= 0) {
            return;
        }
        edit([&](std::vector<document::ToolSource>& l) {
            if (static_cast<std::size_t>(row) < l.size()) {
                std::swap(l[static_cast<std::size_t>(row)], l[static_cast<std::size_t>(row) - 1]);
            }
        });
        clearanceList_->setCurrentRow(row - 1);
    });
    connect(down, &QPushButton::clicked, this, [this, edit]() {
        const int row = clearanceList_->currentRow();
        if (row < 0) {
            return;
        }
        edit([&](std::vector<document::ToolSource>& l) {
            if (static_cast<std::size_t>(row) + 1 < l.size()) {
                std::swap(l[static_cast<std::size_t>(row)], l[static_cast<std::size_t>(row) + 1]);
            }
        });
        clearanceList_->setCurrentRow(row + 1);
    });

    // Vectors (the selector).
    auto* vbox = new QGroupBox(QStringLiteral("Vectors"), this);
    auto* vform = new QFormLayout(vbox);
    baseCombo_ = new QComboBox(vbox);
    baseCombo_->setObjectName(QStringLiteral("vectorBase"));
    baseCombo_->addItems({QStringLiteral("All vectors"), QStringLiteral("Selected objects"), QStringLiteral("Layers")});
    vform->addRow(QStringLiteral("Machine"), baseCombo_);
    layerList_ = new QListWidget(vbox);
    layerList_->setObjectName(QStringLiteral("vectorLayers"));
    layerList_->setMaximumHeight(70);
    for (const FormDocumentInfo::LayerInfo& l : info_.layers) {
        auto* item = new QListWidgetItem(qs(l.name), layerList_);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(Qt::Unchecked);
        item->setData(Qt::UserRole, QVariant::fromValue(static_cast<qulonglong>(l.id.value)));
    }
    vform->addRow(QString(), layerList_);
    openCheck_ = new QCheckBox(QStringLiteral("Open vectors"), vbox);
    closedCheck_ = new QCheckBox(QStringLiteral("Closed vectors"), vbox);
    circlesCheck_ = new QCheckBox(QStringLiteral("Only circles"), vbox);
    diameterEdit_ = new QLineEdit(vbox);
    diameterEdit_->setPlaceholderText(QStringLiteral("any diameter"));
    diameterEdit_->setObjectName(QStringLiteral("circleDiameter"));
    vform->addRow(openCheck_);
    vform->addRow(closedCheck_);
    vform->addRow(circlesCheck_);
    vform->addRow(QStringLiteral("Circle diameter"), diameterEdit_);
    countLabel_ = new QLabel(vbox);
    countLabel_->setObjectName(QStringLiteral("vectorCount"));
    vform->addRow(countLabel_);
    outer->addWidget(vbox);

    const viewmodel::VectorSelector& sel = form_.selector();
    openCheck_->setChecked(sel.includeOpen);
    closedCheck_->setChecked(sel.includeClosed);
    circlesCheck_->setChecked(sel.onlyCircles);
    if (sel.circleDiameterMm > 0.0) {
        diameterEdit_->setText(qs(viewmodel::formatLength(sel.circleDiameterMm, form_.units())));
    }
    int base = BaseObjects;
    if (sel.base.all) {
        base = BaseAll;
    } else if (!sel.base.layers.empty()) {
        base = BaseLayers;
    }
    baseCombo_->setCurrentIndex(base);
    for (int i = 0; i < layerList_->count(); ++i) {
        QListWidgetItem* item = layerList_->item(i);
        const document::LayerId id{item->data(Qt::UserRole).toULongLong()};
        item->setCheckState(std::find(sel.base.layers.begin(), sel.base.layers.end(), id) != sel.base.layers.end()
                                ? Qt::Checked
                                : Qt::Unchecked);
    }
    connect(baseCombo_, &QComboBox::currentIndexChanged, this, [this](int) { applySelectorFromWidgets(); });
    connect(layerList_, &QListWidget::itemChanged, this, [this](QListWidgetItem*) { applySelectorFromWidgets(); });
    for (QCheckBox* c : {openCheck_, closedCheck_, circlesCheck_}) {
        connect(c, &QCheckBox::toggled, this, [this](bool) { applySelectorFromWidgets(); });
    }
    connect(diameterEdit_, &QLineEdit::textEdited, this, [this](const QString&) { applySelectorFromWidgets(); });

    // Parameters (generated).
    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    auto* scrollContent = new QWidget(scroll);
    paramLayout_ = new QVBoxLayout(scrollContent);
    paramLayout_->setContentsMargins(0, 0, 0, 0);
    scroll->setWidget(scrollContent);
    outer->addWidget(scroll, 1);

    status_ = new QLabel(this);
    status_->setObjectName(QStringLiteral("formStatus"));
    status_->setWordWrap(true);
    outer->addWidget(status_);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    ok_ = buttons->button(QDialogButtonBox::Ok);
    ok_->setText(editing_ ? QStringLiteral("Save and recalculate") : QStringLiteral("Add and calculate"));
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    outer->addWidget(buttons);

    // Initial tool state.
    const std::string current = form_.tool().toolId;
    const int row = toolCombo_->findData(qs(current));
    toolCombo_->setCurrentIndex(row >= 0 ? row : 0);
    refreshCuttingData();
    applyClearance();
    refreshVectors();
    building_ = false;
}

void ToolpathFormDialog::refreshCuttingData() {
    const bool was = building_;
    building_ = true;
    dataCombo_->clear();
    const tooling::Tool* tool = db_.findTool(toolCombo_->currentData().toString().toStdString());
    if (tool != nullptr) {
        int current = 0;
        const auto choices = viewmodel::cuttingDataChoices(*tool);
        for (std::size_t i = 0; i < choices.size(); ++i) {
            dataCombo_->addItem(qs(sourceLabel(db_, choices[i])));
            if (choices[i] == form_.tool()) {
                current = static_cast<int>(i);
            }
        }
        dataCombo_->setCurrentIndex(current);
    }
    dataCombo_->setEnabled(dataCombo_->count() > 1);
    const viewmodel::ToolSummary& s = form_.toolSummary();
    if (s.valid) {
        toolInfo_->setText(QStringLiteral("Diameter %1 %2; feed %3, plunge %4 mm/min; spindle %5 rpm%6")
                               .arg(qs(viewmodel::formatLength(s.diameterMm, form_.units())))
                               .arg(QString::fromUtf8(document::unitSuffix(form_.units()).data()))
                               .arg(s.data.feedRate)
                               .arg(s.data.plungeRate)
                               .arg(s.data.spindleSpeed)
                               .arg(s.fallback ? QStringLiteral(" (tool default: no data for that pair)") : QString()));
    } else {
        toolInfo_->clear();
    }
    building_ = was;
}

void ToolpathFormDialog::applyTool() {
    const std::string id = toolCombo_->currentData().toString().toStdString();
    if (id.empty()) {
        // No tool: keep the form's last tool out of the picture by resetting to an empty source.
        viewmodel::ToolpathDefinition def = form_.definition();
        def.tool = document::ToolSource{};
        form_ = viewmodel::ToolpathForm(def, form_.units());
        form_.refreshTools(db_);
        refreshCuttingData();
        refreshPlaceholders();
        refreshStatus();
        return;
    }
    const tooling::Tool* tool = db_.findTool(id);
    if (tool == nullptr) {
        return;
    }
    const viewmodel::FieldResult r = form_.setTool(db_, viewmodel::cuttingDataChoices(*tool).front());
    setMessage(r.message, !r.ok);
    refreshCuttingData();
    refreshPlaceholders();
}

void ToolpathFormDialog::applyClearance() {
    const bool was = building_;
    building_ = true;
    clearanceList_->clear();
    for (const document::ToolSource& s : form_.clearanceTools()) {
        const tooling::Tool* t = db_.findTool(s.toolId);
        clearanceList_->addItem(qs(t != nullptr ? t->name : s.toolId));
    }
    building_ = was;
    refreshStatus();
}

void ToolpathFormDialog::refreshVectors() {
    const int base = baseCombo_->currentIndex();
    layerList_->setVisible(base == BaseLayers);
    diameterEdit_->setEnabled(circlesCheck_->isChecked());
    if (info_.countVectors) {
        const std::size_t n = info_.countVectors(form_.selector());
        countLabel_->setText(QStringLiteral("%1 vector(s) selected now").arg(static_cast<qulonglong>(n)));
    }
    refreshStatus();
}

void ToolpathFormDialog::applySelectorFromWidgets() {
    if (building_) {
        return;
    }
    viewmodel::VectorSelector sel = form_.selector();
    const int base = baseCombo_->currentIndex();
    if (base == BaseAll) {
        sel.base = document::VectorSelection::allVectors();
    } else if (base == BaseLayers) {
        sel.base = document::VectorSelection{};
        for (int i = 0; i < layerList_->count(); ++i) {
            if (layerList_->item(i)->checkState() == Qt::Checked) {
                sel.base.layers.push_back(document::LayerId{layerList_->item(i)->data(Qt::UserRole).toULongLong()});
            }
        }
    } else {
        std::vector<document::ObjectId> objects = form_.selector().base.objects;
        if (objects.empty()) {
            objects = info_.selection;
        }
        sel.base = document::VectorSelection{false, {}, objects};
    }
    sel.includeOpen = openCheck_->isChecked();
    sel.includeClosed = closedCheck_->isChecked();
    sel.onlyCircles = circlesCheck_->isChecked();
    bool diameterOk = true;
    const std::string text = diameterEdit_->text().toStdString();
    if (text.empty()) {
        sel.circleDiameterMm = 0.0;
    } else {
        const viewmodel::ParsedNumber n =
            viewmodel::parseNumber(text, viewmodel::FieldKind::Length, form_.units());
        diameterOk = n.ok && n.value >= 0.0;
        if (diameterOk) {
            sel.circleDiameterMm = n.value;
        }
    }
    diameterEdit_->setStyleSheet(diameterOk ? QString() : QString::fromUtf8(kBadStyle));
    if (diameterOk) {
        badFields_.erase("circleDiameter");
    } else {
        badFields_.insert("circleDiameter");
    }
    form_.setSelector(std::move(sel));
    refreshVectors();
    setMessage(diameterOk ? std::string() : std::string("circle diameter: enter a length (0 or empty = any)"),
               !diameterOk);
}

void ToolpathFormDialog::rebuildParameters() {
    // Remove the old content (deferred: this may run inside one of its own widgets' signals).
    while (QLayoutItem* item = paramLayout_->takeAt(0)) {
        if (QWidget* w = item->widget()) {
            w->hide();
            w->deleteLater();
        }
        delete item;
    }
    edits_.clear();
    choiceWidgets_.clear();
    for (auto it = badFields_.begin(); it != badFields_.end();) {
        it = (*it == "name" || *it == "circleDiameter") ? std::next(it) : badFields_.erase(it);
    }
    const bool was = building_;
    building_ = true;

    const auto fields = form_.fields();
    const auto choices = form_.choices();
    std::vector<std::string> groups;
    const auto note = [&groups](const std::string& g) {
        if (std::find(groups.begin(), groups.end(), g) == groups.end()) {
            groups.push_back(g);
        }
    };
    for (const auto& f : fields) {
        note(f.group);
    }
    for (const auto& c : choices) {
        note(c.group);
    }
    const std::string unit(document::unitSuffix(form_.units()));
    for (const std::string& g : groups) {
        auto* box = new QGroupBox(qs(g), this);
        auto* layout = new QFormLayout(box);
        for (const auto& c : choices) {
            if (c.group != g) {
                continue;
            }
            const std::string id = c.id;
            if (c.isBool) {
                auto* check = new QCheckBox(qs(c.label), box);
                check->setObjectName(qs("choice_" + id));
                check->setChecked(c.current == 1);
                layout->addRow(check);
                choiceWidgets_[id] = check;
                connect(check, &QCheckBox::toggled, this, [this, id](bool on) {
                    if (building_) {
                        return;
                    }
                    const viewmodel::FieldResult r = form_.setChoice(id, on ? 1 : 0);
                    setMessage(r.message, !r.ok);
                    rebuildParameters();
                    refreshStatus();
                });
            } else {
                auto* combo = new QComboBox(box);
                combo->setObjectName(qs("choice_" + id));
                for (const std::string& o : c.options) {
                    combo->addItem(qs(o));
                }
                combo->setCurrentIndex(c.current);
                layout->addRow(qs(c.label), combo);
                choiceWidgets_[id] = combo;
                connect(combo, &QComboBox::currentIndexChanged, this, [this, id](int row) {
                    if (building_) {
                        return;
                    }
                    const viewmodel::FieldResult r = form_.setChoice(id, row);
                    setMessage(r.message, !r.ok);
                    rebuildParameters();
                    refreshStatus();
                });
            }
        }
        for (const auto& f : fields) {
            if (f.group != g) {
                continue;
            }
            auto* edit = new QLineEdit(box);
            edit->setObjectName(qs("field_" + f.id));
            QString label = qs(f.label);
            if (f.kind == ToolpathFieldKind::Length) {
                label += QStringLiteral(" (%1)").arg(qs(unit));
            } else if (f.kind == ToolpathFieldKind::Angle) {
                label += QStringLiteral(" (deg)");
            } else if (f.kind == ToolpathFieldKind::Seconds) {
                label += QStringLiteral(" (s)");
            }
            edit->setText(f.typed ? qs(f.text) : QString());
            edit->setPlaceholderText(qs(f.text));
            layout->addRow(label, edit);
            edits_[f.id] = edit;
            const std::string id = f.id;
            connect(edit, &QLineEdit::textEdited, this, [this, id, edit](const QString& t) {
                const viewmodel::FieldResult r = form_.setField(id, t.toStdString());
                edit->setStyleSheet(r.ok ? QString() : QString::fromUtf8(kBadStyle));
                edit->setToolTip(qs(r.message));
                if (r.ok) {
                    badFields_.erase(id);
                } else {
                    badFields_.insert(id);
                }
                setMessage(r.message, !r.ok);
                refreshPlaceholders();
            });
        }
        paramLayout_->addWidget(box);
    }
    paramLayout_->addStretch(1);
    building_ = was;
}

void ToolpathFormDialog::refreshPlaceholders() {
    for (const ToolpathField& f : form_.fields()) {
        const auto it = edits_.find(f.id);
        if (it != edits_.end() && !badFields_.contains(f.id)) {
            it->second->setPlaceholderText(qs(f.text));
        }
    }
    refreshStatus();
}

void ToolpathFormDialog::setMessage(const std::string& text, bool error) {
    message_ = error ? qs(text) : QString();
    refreshStatus();
}

bool ToolpathFormDialog::canAccept() const {
    if (!badFields_.empty() || !form_.problems().empty()) {
        return false;
    }
    if (info_.countVectors && info_.countVectors(form_.selector()) == 0) {
        return false;
    }
    return true;
}

void ToolpathFormDialog::refreshStatus() {
    if (status_ == nullptr || ok_ == nullptr) {
        return;
    }
    QString text;
    if (!message_.isEmpty()) {
        text = message_;
    } else {
        QStringList parts;
        for (const std::string& p : form_.problems()) {
            parts << qs(p);
        }
        if (info_.countVectors && info_.countVectors(form_.selector()) == 0) {
            parts << QStringLiteral("no vectors are selected");
        }
        text = parts.join(QStringLiteral("; "));
    }
    status_->setText(text);
    status_->setStyleSheet(text.isEmpty() ? QString() : QStringLiteral("color: #b00020;"));
    ok_->setEnabled(canAccept());
}

bool ToolpathFormDialog::typeField(const std::string& id, const std::string& text) {
    QLineEdit* edit = fieldEdit(id);
    if (edit == nullptr) {
        return false;
    }
    edit->setText(qs(text));
    // setText does not emit textEdited; run the same handler the user's typing would.
    Q_EMIT edit->textEdited(qs(text));
    return !badFields_.contains(id);
}

bool ToolpathFormDialog::chooseOption(const std::string& id, int option) {
    const auto it = choiceWidgets_.find(id);
    if (it == choiceWidgets_.end()) {
        return false;
    }
    if (auto* combo = qobject_cast<QComboBox*>(it->second)) {
        combo->setCurrentIndex(option);
    } else if (auto* check = qobject_cast<QCheckBox*>(it->second)) {
        check->setChecked(option == 1);
    } else {
        return false;
    }
    return true;
}

bool ToolpathFormDialog::pickTool(const std::string& toolId) {
    const int row = toolCombo_->findData(qs(toolId));
    if (row < 0) {
        return false;
    }
    toolCombo_->setCurrentIndex(row);
    return form_.tool().toolId == toolId;
}

QLineEdit* ToolpathFormDialog::fieldEdit(const std::string& id) const {
    const auto it = edits_.find(id);
    return it == edits_.end() ? nullptr : it->second;
}

}  // namespace carvetto::app
