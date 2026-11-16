#include "toolpath_form_dialog.hpp"

#include <QAction>
#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMenu>
#include <QPushButton>
#include <QScrollArea>
#include <QHeaderView>
#include <QTableWidget>
#include <QVBoxLayout>
#include <algorithm>
#include <initializer_list>
#include <utility>
#include <variant>

#include "carvetto/document/units.hpp"
#include "carvetto/viewmodel/calculation.hpp"
#include "carvetto/viewmodel/tools.hpp"
#include "carvetto/viewmodel/units_format.hpp"

namespace carvetto::app {

using document::ToolpathOperation;
using viewmodel::ToolpathField;
using viewmodel::ToolpathFieldKind;

const char* operationLabel(ToolpathOperation op) {
    switch (op) {
        case ToolpathOperation::Profile:
            return "Profile";
        case ToolpathOperation::Pocket:
            return "Pocket";
        case ToolpathOperation::Drill:
            return "Drill";
        case ToolpathOperation::Engrave:
            return "Engrave";
        case ToolpathOperation::VCarving:
            return "V-Carve";
        case ToolpathOperation::Chamfer:
            return "Chamfer";
        case ToolpathOperation::ThreadMill:
            return "Thread Mill";
        case ToolpathOperation::Inlay:
            return "Inlay";
        case ToolpathOperation::Fluting:
            return "Fluting";
        case ToolpathOperation::Prism:
            return "Prism";
        case ToolpathOperation::ImageEngrave:
            return "Image Engrave";
        case ToolpathOperation::VCarveInlay:
            return "V-Carve Inlay";
        case ToolpathOperation::Rough3d:
            return "3D Rough";
        case ToolpathOperation::Finish3d:
            return "3D Finish";
        case ToolpathOperation::Moulding:
            return "Moulding";
        case ToolpathOperation::Project3d:
            return "Project onto Model";
        case ToolpathOperation::LaserPicture:
            return "Laser Picture";
        case ToolpathOperation::LaserSketch:
            return "Laser Sketch Engraving";
        case ToolpathOperation::LaserCutFill:
            return "Laser Cut / Fill";
        case ToolpathOperation::DrillBank:
            return "Drill Bank";
        case ToolpathOperation::Rounding:
            return "Rounding";
    }
    return "?";
}

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
    setWindowTitle(QStringLiteral("%1 toolpath").arg(QString::fromUtf8(operationLabel(form_.operation()))));
    form_.refreshTools(db_);
    if (isPhoto() && form_.selector().base.all) {
        // The image engrave reads the selected bitmap: start from the current selection.
        form_.setSelector(viewmodel::VectorSelector::fromObjects(info_.selection));
    }
    buildStatic();
    rebuildParameters();
    refreshStatus();
    reportVariables();
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
    const bool vBitMode = viewmodel::hasClearanceTools(form_.operation());
    const bool restMode = viewmodel::hasRestTools(form_.operation());  // 3D rough / finish, moulding (T-0314)
    const bool mouldingMode = form_.operation() == document::ToolpathOperation::Moulding;
    for (const tooling::Tool* t : viewmodel::toolsFor(db_, form_.operation(), false)) {
        toolCombo_->addItem(qs(t->name), qs(t->id));
    }
    top->addRow(restMode ? QStringLiteral("Tool (the last, finest)")
                         : vBitMode ? QStringLiteral("Carving tool")
                                    : QStringLiteral("Tool"),
                toolCombo_);
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
    clearanceBox_ = new QGroupBox(mouldingMode ? QStringLiteral("Larger area clearance tool (cuts first; at most one)")
                                  : restMode   ? QStringLiteral("Earlier tools (cut first; the tool above cuts what they leave)")
                                               : QStringLiteral("Clearance tools (flat areas, in order)"),
                                  this);
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
    clearanceBox_->setVisible(vBitMode);
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

    // Prior toolpaths (3D finish only, T-0314): the toolpaths whose result is the starting stock.
    priorsBox_ = new QGroupBox(QStringLiteral("Prior toolpaths (ids, comma separated; the finish only cuts what they left)"), this);
    auto* pbox = new QVBoxLayout(priorsBox_);
    priorsEdit_ = new QLineEdit(priorsBox_);
    priorsEdit_->setObjectName(QStringLiteral("priorToolpaths"));
    {
        QStringList ids;
        for (const std::uint64_t id : form_.priorToolpaths()) {
            ids.push_back(QString::number(static_cast<qulonglong>(id)));
        }
        priorsEdit_->setText(ids.join(QStringLiteral(", ")));
    }
    pbox->addWidget(priorsEdit_);
    outer->addWidget(priorsBox_);
    priorsBox_->setVisible(form_.operation() == document::ToolpathOperation::Finish3d);
    connect(priorsEdit_, &QLineEdit::textEdited, this, [this](const QString& text) { applyPriorsText(text); });

    // Spindles of a drill bank (T-0335): one row per spindle of the head.
    bankBox_ = new QGroupBox(QStringLiteral("Drill bank spindles (mm; blank = the tool's diameter / the cut depth)"), this);
    auto* bankLayout = new QVBoxLayout(bankBox_);
    bankTable_ = new QTableWidget(0, 4, bankBox_);
    bankTable_->setObjectName(QStringLiteral("bankSpindleTable"));
    bankTable_->setHorizontalHeaderLabels({QStringLiteral("Drill"), QStringLiteral("Active"),
                                           QStringLiteral("Diameter"), QStringLiteral("Depth")});
    bankTable_->verticalHeader()->setVisible(false);
    bankTable_->horizontalHeader()->setStretchLastSection(true);
    bankTable_->setMinimumHeight(140);
    bankLayout->addWidget(bankTable_);
    outer->addWidget(bankBox_);
    bankBox_->setVisible(form_.operation() == document::ToolpathOperation::DrillBank);
    connect(bankTable_, &QTableWidget::itemChanged, this, [this](QTableWidgetItem*) {
        if (!building_ && !bankBuilding_) {
            applyBankTable();
        }
    });

    // Vectors (the selector).
    auto* vbox = new QGroupBox(isPhoto() ? QStringLiteral("Bitmap") : QStringLiteral("Vectors"), this);
    auto* vform = new QFormLayout(vbox);
    vectorsBox_ = vbox;
    vectorsForm_ = vform;
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

bool ToolpathFormDialog::applyPriorsText(const QString& text) {
    std::vector<std::uint64_t> ids;
    bool ok = true;
    for (const QString& part : text.split(QLatin1Char(','), Qt::SkipEmptyParts)) {
        bool num = false;
        const qulonglong v = part.trimmed().toULongLong(&num);
        if (!num) {
            ok = false;
            break;
        }
        ids.push_back(static_cast<std::uint64_t>(v));
    }
    if (ok) {
        const viewmodel::FieldResult r = form_.setPriorToolpaths(ids);
        ok = r.ok;
        setMessage(r.message, !r.ok);
    } else {
        setMessage("the prior toolpaths are toolpath ids separated by commas", true);
    }
    priorsEdit_->setStyleSheet(ok ? QString() : QString::fromUtf8(kBadStyle));
    if (ok) {
        badFields_.erase("priorToolpaths");
    } else {
        badFields_.insert("priorToolpaths");
    }
    refreshStatus();
    return ok;
}

bool ToolpathFormDialog::typePriors(const std::string& text) {
    priorsEdit_->setText(qs(text));
    return applyPriorsText(qs(text));
}

bool ToolpathFormDialog::addClearanceTool(const std::string& toolId) {
    std::vector<document::ToolSource> list = form_.clearanceTools();
    list.push_back(document::ToolSource{toolId, "", ""});
    const viewmodel::FieldResult r = form_.setClearanceTools(db_, list);
    setMessage(r.message, !r.ok);
    applyClearance();
    return r.ok;
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
    if (isPhoto() && form_.selector().base.all) {
        // The source became a bitmap (a laser sketch's source choice): start from the current selection.
        form_.setSelector(viewmodel::VectorSelector::fromObjects(info_.selection));
        const bool was = building_;
        building_ = true;
        baseCombo_->setCurrentIndex(BaseObjects);
        building_ = was;
    }
    const int base = baseCombo_->currentIndex();
    // The source of a bitmap toolpath is the selected bitmap (and, for Selected Vectors, the selected vectors): no
    // layer or open/closed/circle choices.
    const bool photo = isPhoto();
    vectorsBox_->setTitle(photo ? QStringLiteral("Bitmap") : QStringLiteral("Vectors"));
    for (QWidget* w : std::initializer_list<QWidget*>{baseCombo_, openCheck_, closedCheck_, circlesCheck_, diameterEdit_}) {
        w->setVisible(!photo);
    }
    vectorsForm_->labelForField(baseCombo_)->setVisible(!photo);
    vectorsForm_->labelForField(diameterEdit_)->setVisible(!photo);
    layerList_->setVisible(!photo && base == BaseLayers);
    diameterEdit_->setEnabled(circlesCheck_->isChecked());
    if (isPhoto()) {
        const std::string problem = info_.bitmapProblem ? info_.bitmapProblem(form_.selector()) : std::string();
        QString text = problem.empty() ? QStringLiteral("A bitmap is selected.") : qs(problem);
        if (info_.countVectors && problem.empty() && viewmodel::boundaryUsesVectors(form_.definition().params)) {
            text += QStringLiteral(" %1 vector(s) to follow.")
                        .arg(static_cast<qulonglong>(info_.countVectors(form_.selector())));
        }
        countLabel_->setText(text);
    } else if (form_.operation() == document::ToolpathOperation::Rounding) {
        countLabel_->setText(QStringLiteral("Turns the cylinder of the rotary job down to a radius; no vectors are needed."));
    } else if (form_.operation() == document::ToolpathOperation::Project3d) {
        countLabel_->setText(QStringLiteral("Re-heights the chosen toolpath onto the relief model of the active sheet."));
    } else if (isModel()) {
        QString text = QStringLiteral("Machines the relief model of the active sheet.");
        if (info_.countVectors) {
            text += QStringLiteral(" %1 vector(s) selected now (used by the Selected vectors boundary).")
                        .arg(static_cast<qulonglong>(info_.countVectors(form_.selector())));
        }
        countLabel_->setText(text);
    } else if (info_.countVectors) {
        const std::size_t n = info_.countVectors(form_.selector());
        QString text = QStringLiteral("%1 vector(s) selected now").arg(static_cast<qulonglong>(n));
        if (form_.operation() == document::ToolpathOperation::Moulding) {
            text += QStringLiteral(" (the profile is the section, an open vector; the others are the rails)");
        }
        countLabel_->setText(text);
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
                    refreshVectors();  // the strategy decides what an image engrave needs selected
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
                // A calculation shows its result as the tooltip; a refused one shows why.
                if (r.ok && viewmodel::isExpressionText(t.toStdString())) {
                    const viewmodel::Calculation calc = viewmodel::calculate(t.toStdString(), form_.units());
                    edit->setToolTip(calc.ok ? qs("= " + viewmodel::numberText(calc.value)) : QString());
                }
                setMessage(r.message, !r.ok);
                refreshPlaceholders();
                reportVariables();
                if (r.ok && (id == "horizontalDrills" || id == "verticalDrills")) {
                    refreshBankTable();  // the head changed: one row per spindle
                }
            });
            if (info_.createVariable) {
                edit->setContextMenuPolicy(Qt::CustomContextMenu);
                connect(edit, &QLineEdit::customContextMenuRequested, this, [this, id, edit](const QPoint& pos) {
                    QMenu* menu = edit->createStandardContextMenu();
                    menu->addSeparator();
                    QAction* insert = menu->addAction(QStringLiteral("Insert New Document Variable..."));
                    connect(insert, &QAction::triggered, this, [this, id]() {
                        bool ok = false;
                        const QString name = QInputDialog::getText(this, QStringLiteral("New Document Variable"),
                                                                   QStringLiteral("Name"), QLineEdit::Normal,
                                                                   QString(), &ok);
                        if (!ok) {
                            return;
                        }
                        const QString value = QInputDialog::getText(this, QStringLiteral("New Document Variable"),
                                                                    QStringLiteral("Value"), QLineEdit::Normal,
                                                                    QString(), &ok);
                        if (!ok) {
                            return;
                        }
                        const std::string err = insertNewVariable(id, name.toStdString(), value.toStdString());
                        if (!err.empty()) {
                            setMessage(err, true);
                        }
                    });
                    menu->setAttribute(Qt::WA_DeleteOnClose);
                    menu->popup(edit->mapToGlobal(pos));
                });
            }
        }
        paramLayout_->addWidget(box);
    }
    paramLayout_->addStretch(1);
    building_ = was;
    refreshBankTable();
}

namespace {

QString mmText(double v) {
    return v > 0.0 ? QString::number(v, 'g', 12) : QString();
}

}  // namespace

void ToolpathFormDialog::refreshBankTable() {
    if (bankTable_ == nullptr || form_.operation() != document::ToolpathOperation::DrillBank) {
        return;
    }
    const auto* p = std::get_if<toolpath::DrillBankParams>(&form_.definition().params);
    if (p == nullptr) {
        return;
    }
    const int count = std::clamp(p->horizontalDrills + p->verticalDrills, 0, 64);
    bankBuilding_ = true;
    bankTable_->setRowCount(count);
    for (int i = 0; i < count; ++i) {
        const std::size_t k = static_cast<std::size_t>(i);
        auto* number = new QTableWidgetItem(QString::number(i + 1));
        number->setFlags(Qt::ItemIsEnabled);
        auto* active = new QTableWidgetItem();
        active->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
        active->setCheckState(std::find(p->inactiveSpindles.begin(), p->inactiveSpindles.end(), i + 1) ==
                                      p->inactiveSpindles.end()
                                  ? Qt::Checked
                                  : Qt::Unchecked);
        auto* diameter = new QTableWidgetItem(k < p->spindleDiameters.size() ? mmText(p->spindleDiameters[k]) : QString());
        auto* depth = new QTableWidgetItem(k < p->spindleDepths.size() ? mmText(p->spindleDepths[k]) : QString());
        bankTable_->setItem(i, 0, number);
        bankTable_->setItem(i, 1, active);
        bankTable_->setItem(i, 2, diameter);
        bankTable_->setItem(i, 3, depth);
    }
    bankBuilding_ = false;
    // Lists that no longer fit the head (its size changed) are brought back to it from the rows.
    const bool fits = (p->spindleDiameters.empty() || p->spindleDiameters.size() == static_cast<std::size_t>(count)) &&
                      (p->spindleDepths.empty() || p->spindleDepths.size() == static_cast<std::size_t>(count)) &&
                      std::all_of(p->inactiveSpindles.begin(), p->inactiveSpindles.end(),
                                  [count](int n) { return n >= 1 && n <= count; });
    if (!fits) {
        applyBankTable();
    }
}

bool ToolpathFormDialog::applyBankTable() {
    std::vector<int> inactive;
    std::vector<double> diameters;
    std::vector<double> depths;
    bool anyDiameter = false;
    bool anyDepth = false;
    bool ok = true;
    const int rows = bankTable_->rowCount();
    const auto cell = [this](int row, int column, double& out, bool& present) {
        const QTableWidgetItem* item = bankTable_->item(row, column);
        const QString text = item != nullptr ? item->text().trimmed() : QString();
        present = !text.isEmpty();
        out = 0.0;
        if (!present) {
            return true;
        }
        bool parsed = false;
        out = text.toDouble(&parsed);
        return parsed;
    };
    for (int r = 0; r < rows; ++r) {
        const QTableWidgetItem* active = bankTable_->item(r, 1);
        if (active != nullptr && active->checkState() != Qt::Checked) {
            inactive.push_back(r + 1);
        }
        double d = 0.0;
        double h = 0.0;
        bool hasD = false;
        bool hasH = false;
        ok = cell(r, 2, d, hasD) && ok;
        ok = cell(r, 3, h, hasH) && ok;
        anyDiameter = anyDiameter || hasD;
        anyDepth = anyDepth || hasH;
        diameters.push_back(d);
        depths.push_back(h);
    }
    if (!ok) {
        badFields_.insert("bankSpindles");
        setMessage("drill diameters and depths are numbers in mm", true);
        return false;
    }
    if (anyDiameter) {
        // All or nothing in the form: a blank diameter then stands for the tool's.
        const double tool = form_.toolSummary().diameterMm;
        for (double& d : diameters) {
            d = d > 0.0 ? d : tool;
        }
    } else {
        diameters.clear();
    }
    if (!anyDepth) {
        depths.clear();
    }
    const viewmodel::FieldResult r = form_.setBankSpindles(inactive, diameters, depths);
    if (r.ok) {
        badFields_.erase("bankSpindles");
    } else {
        badFields_.insert("bankSpindles");
    }
    setMessage(r.message, !r.ok);
    return r.ok;
}

bool ToolpathFormDialog::setBankSpindle(int number, bool active, double diameterMm, double depthMm) {
    if (bankTable_ == nullptr || number < 1 || number > bankTable_->rowCount()) {
        return false;
    }
    const int row = number - 1;
    // Each assignment is one edit, as the user's: the table's change handler applies it to the form.
    bankTable_->item(row, 2)->setText(mmText(diameterMm));
    bankTable_->item(row, 3)->setText(mmText(depthMm));
    bankTable_->item(row, 1)->setCheckState(active ? Qt::Checked : Qt::Unchecked);
    return !badFields_.contains("bankSpindles");
}

int ToolpathFormDialog::bankSpindleRows() const {
    return bankTable_ != nullptr && bankBox_ != nullptr && form_.operation() == document::ToolpathOperation::DrillBank
               ? bankTable_->rowCount()
               : 0;
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
    return sourceProblem().empty();
}

std::string ToolpathFormDialog::sourceProblem() const {
    if (isPhoto()) {
        if (info_.bitmapProblem) {
            if (std::string bad = info_.bitmapProblem(form_.selector()); !bad.empty()) {
                return bad;
            }
        }
        if (viewmodel::boundaryUsesVectors(form_.definition().params) && info_.countVectors &&
            info_.countVectors(form_.selector()) == 0) {
            return std::holds_alternative<toolpath::LaserSketchParams>(form_.definition().params)
                       ? "the Vectors boundary needs closed vectors: select some besides the bitmap"
                       : "Selected Vectors needs vectors to follow: select some besides the bitmap";
        }
        return {};
    }
    if (isModel()) {
        if (info_.modelProblem) {
            if (std::string bad = info_.modelProblem(); !bad.empty()) {
                return bad;
            }
        }
        if (form_.operation() == document::ToolpathOperation::Project3d) {
            return {};  // the vectors are not used; the form reports a missing source
        }
        if (viewmodel::boundaryUsesVectors(form_.definition().params) && info_.countVectors &&
            info_.countVectors(form_.selector()) == 0) {
            return "the Selected vectors boundary needs closed vectors: select some";
        }
        return {};
    }
    if (form_.operation() == document::ToolpathOperation::Moulding && info_.countVectors &&
        info_.countVectors(form_.selector()) < 2) {
        return "a moulding needs a profile and at least one rail: select two or more vectors";
    }
    if (info_.countVectors && viewmodel::needsVectors(form_.definition().params) &&
        info_.countVectors(form_.selector()) == 0) {
        return "no vectors are selected";
    }
    return {};
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
        if (const std::string bad = sourceProblem(); !bad.empty()) {
            parts << qs(bad);
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

std::string ToolpathFormDialog::insertNewVariable(const std::string& id, const std::string& name,
                                                  const std::string& value) {
    QLineEdit* edit = fieldEdit(id);
    if (edit == nullptr) {
        return "no such field";
    }
    if (!info_.createVariable) {
        return "variables are not available";
    }
    const std::string err = info_.createVariable(name, value);
    if (!err.empty()) {
        return err;
    }
    QString text = edit->text();
    int pos = edit->cursorPosition();
    const QString token = qs("{" + name + "}");
    if (!viewmodel::isExpressionText(text.toStdString())) {
        text = QStringLiteral("=") + text;
        ++pos;
    }
    text.insert(pos, token);
    edit->setText(text);
    edit->setCursorPosition(pos + static_cast<int>(token.size()));
    Q_EMIT edit->textEdited(text);
    return {};
}

void ToolpathFormDialog::reportVariables() {
    if (info_.variablesChanged) {
        info_.variablesChanged(form_.usedVariables());
    }
}

QLineEdit* ToolpathFormDialog::fieldEdit(const std::string& id) const {
    const auto it = edits_.find(id);
    return it == edits_.end() ? nullptr : it->second;
}

}  // namespace carvetto::app
