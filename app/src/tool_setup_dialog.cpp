#include "tool_setup_dialog.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRegularExpression>
#include <QSpinBox>
#include <QTabWidget>
#include <QTableWidget>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>

#include "carvetto/geometry/vec2.hpp"
#include "carvetto/viewmodel/file_session.hpp"
#include "carvetto/viewmodel/tool_setup.hpp"

namespace carvetto::app {

namespace {

constexpr int kIdRole = Qt::UserRole;
constexpr int kMmRole = Qt::UserRole + 1;
constexpr double kRadToDeg = 180.0 / geometry::kPi;

QString qs(const std::string& s) {
    return QString::fromUtf8(s.data(), static_cast<qsizetype>(s.size()));
}

std::string ss(const QString& s) {
    return s.toStdString();
}

QDoubleSpinBox* makeBox(QWidget* parent, const char* name, double max = viewmodel::kMaxSetupMagnitude) {
    auto* b = new QDoubleSpinBox(parent);
    b->setObjectName(QString::fromLatin1(name));
    b->setRange(0.0, max);
    b->setDecimals(3);
    b->setKeyboardTracking(false);
    return b;
}

QTreeWidgetItem* findItem(QTreeWidget* tree, const std::string& id) {
    for (int i = 0; i < tree->topLevelItemCount(); ++i) {
        QTreeWidgetItem* item = tree->topLevelItem(i);
        if (ss(item->data(0, kIdRole).toString()) == id) {
            return item;
        }
    }
    return nullptr;
}

std::string idOf(const QTreeWidget* tree) {
    const QTreeWidgetItem* item = tree->currentItem();
    return item == nullptr ? std::string() : ss(item->data(0, kIdRole).toString());
}

QPushButton* button(QWidget* parent, const char* name, const QString& text) {
    auto* b = new QPushButton(text, parent);
    b->setObjectName(QString::fromLatin1(name));
    return b;
}

}  // namespace

ToolSetupDialog::ToolSetupDialog(tooling::ToolDatabase db, std::string activeMachineId, document::Units units,
                                 std::function<std::vector<std::string>(const std::string&)> toolpathUses,
                                 QWidget* parent)
    : QDialog(parent),
      db_(std::move(db)),
      activeMachineId_(std::move(activeMachineId)),
      units_(units),
      uses_(std::move(toolpathUses)) {
    setWindowTitle(QStringLiteral("Tools, Machines and Materials"));
    setObjectName(QStringLiteral("ToolSetupDialog"));
    resize(980, 640);

    auto* outer = new QVBoxLayout(this);
    auto* top = new QHBoxLayout();
    top->addWidget(new QLabel(QStringLiteral("Show lengths in:"), this));
    unitsBox_ = new QComboBox(this);
    unitsBox_->setObjectName(QStringLiteral("setupUnits"));
    unitsBox_->addItem(QStringLiteral("Millimetres"), static_cast<int>(document::Units::Millimetres));
    unitsBox_->addItem(QStringLiteral("Inches"), static_cast<int>(document::Units::Inches));
    unitsBox_->setCurrentIndex(units_ == document::Units::Inches ? 1 : 0);
    top->addWidget(unitsBox_);
    top->addStretch(1);
    QPushButton* importButton = button(this, "setupImport", QStringLiteral("Import tools..."));
    QPushButton* exportButton = button(this, "setupExport", QStringLiteral("Export tools..."));
    importButton->setToolTip(QStringLiteral("Add the tools, machines and materials of a tool database file that are not here yet"));
    exportButton->setToolTip(QStringLiteral("Save the whole tool database to a file"));
    top->addWidget(importButton);
    top->addWidget(exportButton);
    outer->addLayout(top);

    tabs_ = new QTabWidget(this);
    tabs_->setObjectName(QStringLiteral("setupTabs"));
    auto* toolsPage = new QWidget(tabs_);
    auto* machinesPage = new QWidget(tabs_);
    auto* materialsPage = new QWidget(tabs_);
    buildTools(toolsPage);
    buildMachines(machinesPage);
    buildMaterials(materialsPage);
    tabs_->addTab(toolsPage, QStringLiteral("Tools"));
    tabs_->addTab(machinesPage, QStringLiteral("Machines"));
    tabs_->addTab(materialsPage, QStringLiteral("Materials"));
    outer->addWidget(tabs_, 1);

    errors_ = new QLabel(this);
    errors_->setObjectName(QStringLiteral("setupErrors"));
    errors_->setWordWrap(true);
    errors_->setStyleSheet(QStringLiteral("color: #b00020;"));
    outer->addWidget(errors_);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    buttons->setObjectName(QStringLiteral("setupButtons"));
    connect(buttons, &QDialogButtonBox::accepted, this, &ToolSetupDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    outer->addWidget(buttons);

    connect(unitsBox_, &QComboBox::currentIndexChanged, this, [this](int) {
        setUnits(static_cast<document::Units>(unitsBox_->currentData().toInt()));
    });
    connect(importButton, &QPushButton::clicked, this, [this]() {
        const QString file = QFileDialog::getOpenFileName(this, QStringLiteral("Import tool database"), QString(),
                                                          QStringLiteral("Tool database (*.json);;All files (*)"));
        if (!file.isEmpty()) {
            importFile(viewmodel::pathFromUtf8(ss(file)));
        }
    });
    connect(exportButton, &QPushButton::clicked, this, [this]() {
        const QString file = QFileDialog::getSaveFileName(this, QStringLiteral("Export tool database"),
                                                          QStringLiteral("tools.json"),
                                                          QStringLiteral("Tool database (*.json)"));
        if (!file.isEmpty()) {
            exportFile(viewmodel::pathFromUtf8(ss(file)));
        }
    });

    refreshAll();
}

// ------------------------------------------------------------------------------------- building

void ToolSetupDialog::buildTools(QWidget* page) {
    auto* row = new QHBoxLayout(page);

    auto* left = new QVBoxLayout();
    toolList_ = new QTreeWidget(page);
    toolList_->setObjectName(QStringLiteral("toolList"));
    toolList_->setColumnCount(2);
    toolList_->setHeaderLabels({QStringLiteral("Tool"), QStringLiteral("Type")});
    toolList_->setRootIsDecorated(false);
    left->addWidget(toolList_, 1);
    auto* buttons = new QHBoxLayout();
    auto* add = button(page, "toolAdd", QStringLiteral("Add"));
    auto* addMenu = new QMenu(add);
    for (const tooling::ToolType type : tooling::allToolTypes()) {
        QAction* a = addMenu->addAction(qs(viewmodel::toolTypeLabel(type)));
        a->setObjectName(QStringLiteral("toolAdd_") + qs(std::string(tooling::toString(type))));
        connect(a, &QAction::triggered, this, [this, type]() { addTool(type); });
    }
    add->setMenu(addMenu);
    toolDuplicate_ = button(page, "toolDuplicate", QStringLiteral("Duplicate"));
    toolDelete_ = button(page, "toolDelete", QStringLiteral("Delete"));
    buttons->addWidget(add);
    buttons->addWidget(toolDuplicate_);
    buttons->addWidget(toolDelete_);
    left->addLayout(buttons);
    row->addLayout(left, 2);

    toolForm_ = new QWidget(page);
    auto* right = new QVBoxLayout(toolForm_);
    auto* form = new QFormLayout();
    toolName_ = new QLineEdit(toolForm_);
    toolName_->setObjectName(QStringLiteral("toolName"));
    toolType_ = new QComboBox(toolForm_);
    toolType_->setObjectName(QStringLiteral("toolType"));
    for (const tooling::ToolType type : tooling::allToolTypes()) {
        toolType_->addItem(qs(viewmodel::toolTypeLabel(type)), static_cast<int>(type));
    }
    toolDiameter_ = makeBox(toolForm_, "toolDiameter");
    toolAngle_ = makeBox(toolForm_, "toolAngle", 179.0);
    toolAngle_->setSuffix(QStringLiteral(" deg"));
    toolTip_ = makeBox(toolForm_, "toolTip");
    toolCorner_ = makeBox(toolForm_, "toolCorner");
    toolFlutes_ = new QSpinBox(toolForm_);
    toolFlutes_->setObjectName(QStringLiteral("toolFlutes"));
    toolFlutes_->setRange(1, 1000);
    toolProfile_ = new QPlainTextEdit(toolForm_);
    toolProfile_->setObjectName(QStringLiteral("toolProfile"));
    toolProfile_->setMaximumHeight(70);
    toolProfile_->setToolTip(QStringLiteral("One point per line: radius, height above the tip (right-hand half profile)"));
    toolNotes_ = new QLineEdit(toolForm_);
    toolNotes_->setObjectName(QStringLiteral("toolNotes"));
    const auto addRow = [&](const QString& label, QWidget* w, std::vector<QWidget*>* group) {
        auto* l = new QLabel(label, toolForm_);
        form->addRow(l, w);
        if (group != nullptr) {
            group->push_back(l);
            group->push_back(w);
        }
    };
    addRow(QStringLiteral("Name"), toolName_, nullptr);
    addRow(QStringLiteral("Type"), toolType_, nullptr);
    addRow(QStringLiteral("Diameter"), toolDiameter_, &diameterWidgets_);
    addRow(QStringLiteral("Included angle"), toolAngle_, &angleWidgets_);
    addRow(QStringLiteral("Tip / ball diameter"), toolTip_, &tipWidgets_);
    addRow(QStringLiteral("Corner radius"), toolCorner_, &cornerWidgets_);
    addRow(QStringLiteral("Profile"), toolProfile_, &profileWidgets_);
    addRow(QStringLiteral("Flutes"), toolFlutes_, nullptr);
    addRow(QStringLiteral("Notes"), toolNotes_, nullptr);
    right->addLayout(form);

    right->addWidget(new QLabel(QStringLiteral("Feeds and speeds (default and per machine and material):"), toolForm_));
    cuttingTable_ = new QTableWidget(toolForm_);
    cuttingTable_->setObjectName(QStringLiteral("cuttingTable"));
    right->addWidget(cuttingTable_, 1);
    auto* entry = new QHBoxLayout();
    entryMachine_ = new QComboBox(toolForm_);
    entryMachine_->setObjectName(QStringLiteral("entryMachine"));
    entryMaterial_ = new QComboBox(toolForm_);
    entryMaterial_->setObjectName(QStringLiteral("entryMaterial"));
    auto* addEntry = button(toolForm_, "entryAdd", QStringLiteral("Add entry"));
    auto* removeEntry = button(toolForm_, "entryRemove", QStringLiteral("Remove entry"));
    entry->addWidget(entryMachine_);
    entry->addWidget(entryMaterial_);
    entry->addWidget(addEntry);
    entry->addWidget(removeEntry);
    right->addLayout(entry);
    row->addWidget(toolForm_, 5);

    connect(toolList_, &QTreeWidget::currentItemChanged, this, [this]() { loadTool(); });
    connect(toolDuplicate_, &QPushButton::clicked, this, [this]() { duplicateTool(); });
    connect(toolDelete_, &QPushButton::clicked, this, [this]() { deleteTool(); });
    connect(addEntry, &QPushButton::clicked, this, [this]() { addCuttingEntry(); });
    connect(removeEntry, &QPushButton::clicked, this, [this]() { removeCuttingEntry(); });
    const auto commit = [this]() { commitTool(); };
    connect(toolName_, &QLineEdit::textEdited, this, commit);
    connect(toolNotes_, &QLineEdit::textEdited, this, commit);
    connect(toolType_, &QComboBox::activated, this, [this](int) {
        // A new type shows other fields: keep the values that still apply, fill the others sensibly.
        const auto type = static_cast<tooling::ToolType>(toolType_->currentData().toInt());
        const tooling::Tool fresh = viewmodel::defaultTool(type);
        const viewmodel::ToolFields fields = viewmodel::toolFields(type);
        loading_ = true;
        if (fields.includedAngle || fields.tipDiameter) {
            toolAngle_->setValue(fresh.includedAngle * kRadToDeg);
        }
        if (fields.tipDiameter) {
            toolTip_->setValue(lengthToUi(fresh.tipDiameter));
            toolTip_->setProperty("mm", fresh.tipDiameter);
            toolTip_->setProperty("ui", toolTip_->value());
        }
        if (fields.cornerRadius) {
            toolCorner_->setValue(lengthToUi(fresh.cornerRadius));
            toolCorner_->setProperty("mm", fresh.cornerRadius);
            toolCorner_->setProperty("ui", toolCorner_->value());
        }
        if (fields.profile) {
            QString text;
            for (const geometry::Vec2& p : fresh.profile) {
                text += QString::number(lengthToUi(p.x), 'g', 8) + QStringLiteral(", ") +
                        QString::number(lengthToUi(p.y), 'g', 8) + QStringLiteral("\n");
            }
            toolProfile_->setPlainText(text);
        }
        loading_ = false;
        updateFieldVisibility();
        commitTool();
    });
    connect(toolDiameter_, &QDoubleSpinBox::valueChanged, this, commit);
    connect(toolAngle_, &QDoubleSpinBox::valueChanged, this, commit);
    connect(toolTip_, &QDoubleSpinBox::valueChanged, this, commit);
    connect(toolCorner_, &QDoubleSpinBox::valueChanged, this, commit);
    connect(toolFlutes_, &QSpinBox::valueChanged, this, commit);
    connect(toolProfile_, &QPlainTextEdit::textChanged, this, commit);
    connect(cuttingTable_, &QTableWidget::itemChanged, this, commit);
}

void ToolSetupDialog::buildMachines(QWidget* page) {
    auto* row = new QHBoxLayout(page);
    auto* left = new QVBoxLayout();
    machineList_ = new QTreeWidget(page);
    machineList_->setObjectName(QStringLiteral("machineList"));
    machineList_->setColumnCount(2);
    machineList_->setHeaderLabels({QStringLiteral("Machine"), QStringLiteral("")});
    machineList_->setRootIsDecorated(false);
    left->addWidget(machineList_, 1);
    auto* buttons = new QHBoxLayout();
    auto* add = button(page, "machineAdd", QStringLiteral("Add"));
    machineDuplicate_ = button(page, "machineDuplicate", QStringLiteral("Duplicate"));
    machineDelete_ = button(page, "machineDelete", QStringLiteral("Delete"));
    buttons->addWidget(add);
    buttons->addWidget(machineDuplicate_);
    buttons->addWidget(machineDelete_);
    left->addLayout(buttons);
    row->addLayout(left, 2);

    machineForm_ = new QWidget(page);
    auto* right = new QVBoxLayout(machineForm_);
    auto* form = new QFormLayout();
    machineName_ = new QLineEdit(machineForm_);
    machineName_->setObjectName(QStringLiteral("machineName"));
    workX_ = makeBox(machineForm_, "machineWorkX");
    workY_ = makeBox(machineForm_, "machineWorkY");
    workZ_ = makeBox(machineForm_, "machineWorkZ");
    rapid_ = makeBox(machineForm_, "machineRapid");
    accelX_ = makeBox(machineForm_, "machineAccelX");
    accelY_ = makeBox(machineForm_, "machineAccelY");
    accelZ_ = makeBox(machineForm_, "machineAccelZ");
    accelAngular_ = makeBox(machineForm_, "machineAccelAngular");
    accelAngular_->setSuffix(QStringLiteral(" deg/s2"));
    maxSpindle_ = makeBox(machineForm_, "machineMaxSpindle");
    maxSpindle_->setSuffix(QStringLiteral(" rpm"));
    maxFeed_ = makeBox(machineForm_, "machineMaxFeed");
    for (QDoubleSpinBox* b : {workX_, workY_, workZ_, rapid_, accelX_, accelY_, accelZ_, maxSpindle_, maxFeed_, accelAngular_}) {
        b->setSpecialValueText(QStringLiteral("not set"));
    }
    posts_ = new QLineEdit(machineForm_);
    posts_->setObjectName(QStringLiteral("machinePosts"));
    posts_->setToolTip(QStringLiteral("Names of the posts usable with this machine, separated by commas"));
    defaultPost_ = new QComboBox(machineForm_);
    defaultPost_->setObjectName(QStringLiteral("machineDefaultPost"));
    form->addRow(QStringLiteral("Name"), machineName_);
    form->addRow(QStringLiteral("Work area X"), workX_);
    form->addRow(QStringLiteral("Work area Y"), workY_);
    form->addRow(QStringLiteral("Work area Z"), workZ_);
    form->addRow(QStringLiteral("Rapid rate"), rapid_);
    form->addRow(QStringLiteral("Acceleration X"), accelX_);
    form->addRow(QStringLiteral("Acceleration Y"), accelY_);
    form->addRow(QStringLiteral("Acceleration Z"), accelZ_);
    form->addRow(QStringLiteral("Rotary acceleration"), accelAngular_);
    form->addRow(QStringLiteral("Spindle speed limit"), maxSpindle_);
    form->addRow(QStringLiteral("Feed rate limit"), maxFeed_);
    form->addRow(QStringLiteral("Associated posts"), posts_);
    form->addRow(QStringLiteral("Default post"), defaultPost_);
    right->addLayout(form);
    machineUse_ = button(machineForm_, "machineUse", QStringLiteral("Use this machine"));
    machineUse_->setToolTip(QStringLiteral("The machine whose rapid rate and acceleration are used for the time estimate"));
    right->addWidget(machineUse_);
    right->addStretch(1);
    row->addWidget(machineForm_, 5);

    connect(machineList_, &QTreeWidget::currentItemChanged, this, [this]() { loadMachine(); });
    connect(add, &QPushButton::clicked, this, [this]() { addMachine(); });
    connect(machineDuplicate_, &QPushButton::clicked, this, [this]() { duplicateMachine(); });
    connect(machineDelete_, &QPushButton::clicked, this, [this]() { deleteMachine(); });
    connect(machineUse_, &QPushButton::clicked, this, [this]() {
        const std::string id = currentMachineId();
        if (!id.empty()) {
            activeMachineId_ = id;
            refreshMachines();
        }
    });
    const auto commit = [this]() { commitMachine(); };
    connect(machineName_, &QLineEdit::textEdited, this, commit);
    for (QDoubleSpinBox* b : {workX_, workY_, workZ_, rapid_, accelX_, accelY_, accelZ_, maxSpindle_, maxFeed_, accelAngular_}) {
        connect(b, &QDoubleSpinBox::valueChanged, this, commit);
    }
    connect(posts_, &QLineEdit::textEdited, this, [this]() {
        const QString keep = defaultPost_->currentText();
        loading_ = true;
        defaultPost_->clear();
        defaultPost_->addItem(QString());
        for (const QString& p : posts_->text().split(QLatin1Char(','), Qt::SkipEmptyParts)) {
            defaultPost_->addItem(p.trimmed());
        }
        const int at = defaultPost_->findText(keep);
        defaultPost_->setCurrentIndex(at < 0 ? 0 : at);
        loading_ = false;
        commitMachine();
    });
    connect(defaultPost_, &QComboBox::activated, this, [this](int) { commitMachine(); });
}

void ToolSetupDialog::buildMaterials(QWidget* page) {
    auto* col = new QVBoxLayout(page);
    materialList_ = new QTreeWidget(page);
    materialList_->setObjectName(QStringLiteral("materialList"));
    materialList_->setColumnCount(1);
    materialList_->setHeaderLabels({QStringLiteral("Material (double-click to rename)")});
    materialList_->setRootIsDecorated(false);
    col->addWidget(materialList_, 1);
    auto* buttons = new QHBoxLayout();
    auto* add = button(page, "materialAdd", QStringLiteral("Add"));
    materialDuplicate_ = button(page, "materialDuplicate", QStringLiteral("Duplicate"));
    materialDuplicate_->setToolTip(QStringLiteral("Copy the material together with every tool's feeds and speeds for it"));
    materialDelete_ = button(page, "materialDelete", QStringLiteral("Delete"));
    buttons->addWidget(add);
    buttons->addWidget(materialDuplicate_);
    buttons->addWidget(materialDelete_);
    buttons->addStretch(1);
    col->addLayout(buttons);

    connect(add, &QPushButton::clicked, this, [this]() { addMaterial(); });
    connect(materialDuplicate_, &QPushButton::clicked, this, [this]() { duplicateMaterial(); });
    connect(materialDelete_, &QPushButton::clicked, this, [this]() { deleteMaterial(); });
    connect(materialList_, &QTreeWidget::itemChanged, this, [this](QTreeWidgetItem* item, int) {
        if (loading_) {
            return;
        }
        const std::string name = ss(item->text(0).trimmed());
        if (name.empty()) {
            setError(QStringLiteral("A material needs a name."));
            loading_ = true;
            const auto* m = db_.findMaterial(ss(item->data(0, kIdRole).toString()));
            item->setText(0, m == nullptr ? QString() : qs(m->name));
            loading_ = false;
            return;
        }
        setError(QString());
        db_.renameMaterial(ss(item->data(0, kIdRole).toString()), name);
        refreshTools();  // the entry combos show the names
    });
    connect(materialList_, &QTreeWidget::currentItemChanged, this, [this]() {
        materialDuplicate_->setEnabled(!currentMaterialId().empty());
        materialDelete_->setEnabled(!currentMaterialId().empty());
    });
}

// ------------------------------------------------------------------------------------- helpers

double ToolSetupDialog::lengthToUi(double mm) const {
    return document::toDisplay(mm, units_);
}

double ToolSetupDialog::lengthFromUi(double value) const {
    return document::fromDisplay(value, units_);
}

std::string ToolSetupDialog::currentToolId() const {
    return idOf(toolList_);
}
std::string ToolSetupDialog::currentMachineId() const {
    return idOf(machineList_);
}
std::string ToolSetupDialog::currentMaterialId() const {
    return idOf(materialList_);
}

QString ToolSetupDialog::errorText() const {
    return errors_->text();
}

void ToolSetupDialog::setError(const QString& text) {
    errors_->setText(text);
}

bool ToolSetupDialog::confirm(const QString& title, const QString& text) {
    if (quiet_) {
        return true;
    }
    return QMessageBox::question(this, title, text, QMessageBox::Yes | QMessageBox::No, QMessageBox::No) ==
           QMessageBox::Yes;
}

namespace {

/// Sets a length box (value in the display units) and remembers the mm value it came from so an
/// untouched box never changes the stored number through unit rounding.
void setLengthBox(QDoubleSpinBox* box, double mm, document::Units units, double scale = 1.0) {
    box->setProperty("mm", mm);
    box->setValue(document::toDisplay(mm, units) * scale);
    box->setProperty("ui", box->value());
}

double lengthOf(const QDoubleSpinBox* box, document::Units units, double scale = 1.0) {
    if (box->value() == box->property("ui").toDouble()) {
        return box->property("mm").toDouble();
    }
    return document::fromDisplay(box->value() / scale, units);
}

}  // namespace

void ToolSetupDialog::setUnits(document::Units units) {
    units_ = units;
    const bool in = units == document::Units::Inches;
    const QString len = in ? QStringLiteral(" in") : QStringLiteral(" mm");
    const int decimals = in ? 4 : 3;
    for (QDoubleSpinBox* b : {toolDiameter_, toolTip_, toolCorner_, workX_, workY_, workZ_}) {
        b->setSuffix(len);
        b->setDecimals(decimals);
    }
    for (QDoubleSpinBox* b : {rapid_, maxFeed_}) {
        b->setSuffix(len + QStringLiteral("/min"));
        b->setDecimals(in ? 3 : 1);
    }
    for (QDoubleSpinBox* b : {accelX_, accelY_, accelZ_}) {
        b->setSuffix(len + QStringLiteral("/s2"));
        b->setDecimals(in ? 3 : 1);
    }
    loadTool();
    loadMachine();
}

// ------------------------------------------------------------------------------------ refreshing

void ToolSetupDialog::refreshAll() {
    setUnits(units_);
    refreshMaterials();
    refreshMachines();
    refreshTools();
}

void ToolSetupDialog::refreshTools() {
    const std::string keep = currentToolId();
    loading_ = true;
    toolList_->clear();
    std::vector<const tooling::Tool*> tools;
    for (const auto& entry : db_.tools()) {
        tools.push_back(&entry.second);
    }
    std::sort(tools.begin(), tools.end(), [](const tooling::Tool* a, const tooling::Tool* b) {
        return QString::compare(qs(a->name), qs(b->name), Qt::CaseInsensitive) < 0;
    });
    for (const tooling::Tool* t : tools) {
        auto* item = new QTreeWidgetItem(toolList_);
        item->setText(0, qs(t->name));
        item->setText(1, qs(viewmodel::toolTypeLabel(t->type)));
        item->setData(0, kIdRole, qs(t->id));
    }
    entryMachine_->clear();
    for (const auto& [id, m] : db_.machines()) {
        entryMachine_->addItem(qs(m.name), qs(id));
    }
    entryMaterial_->clear();
    for (const auto& [id, m] : db_.materials()) {
        entryMaterial_->addItem(qs(m.name), qs(id));
    }
    loading_ = false;
    if (QTreeWidgetItem* item = findItem(toolList_, keep)) {
        toolList_->setCurrentItem(item);
    } else if (toolList_->topLevelItemCount() > 0) {
        toolList_->setCurrentItem(toolList_->topLevelItem(0));
    }
    loadTool();
}

void ToolSetupDialog::refreshMachines() {
    const std::string keep = currentMachineId();
    loading_ = true;
    machineList_->clear();
    for (const auto& [id, m] : db_.machines()) {
        auto* item = new QTreeWidgetItem(machineList_);
        item->setText(0, qs(m.name));
        item->setText(1, id == activeMachineId_ ? QStringLiteral("in use") : QString());
        item->setData(0, kIdRole, qs(id));
    }
    loading_ = false;
    if (QTreeWidgetItem* item = findItem(machineList_, keep)) {
        machineList_->setCurrentItem(item);
    } else if (QTreeWidgetItem* active = findItem(machineList_, activeMachineId_)) {
        machineList_->setCurrentItem(active);
    } else if (machineList_->topLevelItemCount() > 0) {
        machineList_->setCurrentItem(machineList_->topLevelItem(0));
    }
    loadMachine();
}

void ToolSetupDialog::refreshMaterials() {
    const std::string keep = currentMaterialId();
    loading_ = true;
    materialList_->clear();
    for (const auto& [id, m] : db_.materials()) {
        auto* item = new QTreeWidgetItem(materialList_);
        item->setText(0, qs(m.name));
        item->setData(0, kIdRole, qs(id));
        item->setFlags(item->flags() | Qt::ItemIsEditable);
    }
    loading_ = false;
    if (QTreeWidgetItem* item = findItem(materialList_, keep)) {
        materialList_->setCurrentItem(item);
    } else if (materialList_->topLevelItemCount() > 0) {
        materialList_->setCurrentItem(materialList_->topLevelItem(0));
    }
    materialDuplicate_->setEnabled(!currentMaterialId().empty());
    materialDelete_->setEnabled(!currentMaterialId().empty());
}

// ------------------------------------------------------------------------------------------ tools

namespace {

const char* const kColumns[] = {"Pass depth", "Stepover", "Final stepover", "Clearance stepover", "Spindle (rpm)",
                                "Feed", "Plunge", "Max burn", "Tool no."};

}  // namespace

void ToolSetupDialog::updateFieldVisibility() {
    const auto type = static_cast<tooling::ToolType>(toolType_->currentData().toInt());
    const viewmodel::ToolFields f = viewmodel::toolFields(type);
    const auto show = [](const std::vector<QWidget*>& ws, bool on) {
        for (QWidget* w : ws) {
            w->setVisible(on);
        }
    };
    show(diameterWidgets_, f.diameter);
    show(angleWidgets_, f.includedAngle);
    show(tipWidgets_, f.tipDiameter);
    show(cornerWidgets_, f.cornerRadius);
    show(profileWidgets_, f.profile);
}

void ToolSetupDialog::fillCuttingTable(const tooling::Tool& tool) {
    const bool in = units_ == document::Units::Inches;
    const QString feedUnit = in ? QStringLiteral("in/min") : QStringLiteral("mm/min");
    const QString lenUnit = in ? QStringLiteral("in") : QStringLiteral("mm");
    cuttingTable_->clear();
    cuttingTable_->setColumnCount(9);
    QStringList headers;
    for (int c = 0; c < 9; ++c) {
        QString h = QString::fromLatin1(kColumns[c]);
        if (c <= 3) {
            h += QStringLiteral(" (") + lenUnit + QStringLiteral(")");
        } else if (c >= 5 && c <= 7) {
            h += QStringLiteral(" (") + feedUnit + QStringLiteral(")");
        }
        headers << h;
    }
    cuttingTable_->setHorizontalHeaderLabels(headers);
    cuttingTable_->horizontalHeader()->setStretchLastSection(true);

    struct RowData {
        QString label;
        QString machineId;
        QString materialId;
        const tooling::CuttingData* data;
    };
    std::vector<RowData> rows;
    rows.push_back({QStringLiteral("Default"), QString(), QString(), &tool.defaultCuttingData});
    for (const auto& [key, entryData] : tool.cuttingData) {
        const tooling::Machine* m = db_.findMachine(key.machineId);
        const tooling::Material* mat = db_.findMaterial(key.materialId);
        rows.push_back({(m != nullptr ? qs(m->name) : qs(key.machineId)) + QStringLiteral(" / ") +
                            (mat != nullptr ? qs(mat->name) : qs(key.materialId)),
                        qs(key.machineId), qs(key.materialId), &entryData});
    }
    cuttingTable_->setRowCount(static_cast<int>(rows.size()));
    for (int r = 0; r < static_cast<int>(rows.size()); ++r) {
        const RowData& rd = rows[static_cast<std::size_t>(r)];
        auto* header = new QTableWidgetItem(rd.label);
        header->setData(kIdRole, rd.machineId);
        header->setData(kIdRole + 2, rd.materialId);
        cuttingTable_->setVerticalHeaderItem(r, header);
        const tooling::CuttingData& d = *rd.data;
        const double mm[8] = {d.passDepth, d.stepover, d.finalPassStepover, d.clearancePassStepover,
                              d.spindleSpeed, d.feedRate, d.plungeRate, d.maxBurnRate};
        for (int c = 0; c < 9; ++c) {
            const bool length = c <= 3 || (c >= 5 && c <= 7);
            const double value = c == 8 ? static_cast<double>(d.toolNumber) : mm[c];
            auto* item = new QTableWidgetItem(
                QString::number(length ? lengthToUi(value) : value, 'g', 8));
            item->setData(kMmRole, value);
            item->setData(kMmRole + 1, item->text());
            cuttingTable_->setItem(r, c, item);
        }
    }
}

bool ToolSetupDialog::readCuttingTable(tooling::Tool& tool, QString& error) {
    tool.cuttingData.clear();
    for (int r = 0; r < cuttingTable_->rowCount(); ++r) {
        double v[9] = {};
        for (int c = 0; c < 9; ++c) {
            const QTableWidgetItem* item = cuttingTable_->item(r, c);
            if (item == nullptr) {
                error = QStringLiteral("A feeds and speeds cell is empty.");
                return false;
            }
            if (item->text() == item->data(kMmRole + 1).toString()) {
                v[c] = item->data(kMmRole).toDouble();
                continue;
            }
            bool ok = false;
            double x = item->text().trimmed().toDouble(&ok);
            if (!ok || !std::isfinite(x) || x < 0.0) {
                error = QStringLiteral("Feeds and speeds must be numbers that are not negative.");
                return false;
            }
            if (c <= 3 || (c >= 5 && c <= 7)) {
                x = lengthFromUi(x);
            }
            v[c] = x;
        }
        tooling::CuttingData d;
        d.passDepth = v[0];
        d.stepover = v[1];
        d.finalPassStepover = v[2];
        d.clearancePassStepover = v[3];
        d.spindleSpeed = v[4];
        d.feedRate = v[5];
        d.plungeRate = v[6];
        d.maxBurnRate = v[7];
        d.toolNumber = static_cast<int>(std::min(v[8], 1.0e6));
        const QTableWidgetItem* header = cuttingTable_->verticalHeaderItem(r);
        const std::string machine = header != nullptr ? ss(header->data(kIdRole).toString()) : std::string();
        if (r == 0) {
            tool.defaultCuttingData = d;
        } else {
            tool.cuttingData[tooling::CuttingDataKey{machine, ss(header->data(kIdRole + 2).toString())}] = d;
        }
    }
    return true;
}

void ToolSetupDialog::loadTool() {
    if (loading_) {
        return;
    }
    const tooling::Tool* tool = db_.findTool(currentToolId());
    toolForm_->setEnabled(tool != nullptr);
    toolDuplicate_->setEnabled(tool != nullptr);
    toolDelete_->setEnabled(tool != nullptr);
    if (tool == nullptr) {
        return;
    }
    loading_ = true;
    toolName_->setText(qs(tool->name));
    toolNotes_->setText(qs(tool->notes));
    toolType_->setCurrentIndex(toolType_->findData(static_cast<int>(tool->type)));
    setLengthBox(toolDiameter_, tool->diameter, units_);
    toolAngle_->setValue(tool->includedAngle * kRadToDeg);
    toolAngle_->setProperty("rad", tool->includedAngle);
    toolAngle_->setProperty("ui", toolAngle_->value());
    setLengthBox(toolTip_, tool->tipDiameter, units_);
    setLengthBox(toolCorner_, tool->cornerRadius, units_);
    toolFlutes_->setValue(tool->flutes);
    QString profile;
    for (const geometry::Vec2& p : tool->profile) {
        profile += QString::number(lengthToUi(p.x), 'g', 8) + QStringLiteral(", ") +
                   QString::number(lengthToUi(p.y), 'g', 8) + QStringLiteral("\n");
    }
    toolProfile_->setPlainText(profile);
    toolProfile_->setProperty("text", profile);
    fillCuttingTable(*tool);
    loading_ = false;
    updateFieldVisibility();
}

void ToolSetupDialog::commitTool() {
    if (loading_) {
        return;
    }
    const tooling::Tool* current = db_.findTool(currentToolId());
    if (current == nullptr) {
        return;
    }
    tooling::Tool t = *current;
    t.name = ss(toolName_->text());
    t.notes = ss(toolNotes_->text());
    t.type = static_cast<tooling::ToolType>(toolType_->currentData().toInt());
    t.diameter = lengthOf(toolDiameter_, units_);
    t.includedAngle = toolAngle_->value() == toolAngle_->property("ui").toDouble()
                          ? toolAngle_->property("rad").toDouble()
                          : toolAngle_->value() / kRadToDeg;
    t.tipDiameter = lengthOf(toolTip_, units_);
    t.cornerRadius = lengthOf(toolCorner_, units_);
    t.flutes = toolFlutes_->value();
    const viewmodel::ToolFields fields = viewmodel::toolFields(t.type);
    if (fields.profile) {
        if (toolProfile_->toPlainText() != toolProfile_->property("text").toString()) {
            std::vector<geometry::Vec2> profile;
            for (const QString& line : toolProfile_->toPlainText().split(QLatin1Char('\n'), Qt::SkipEmptyParts)) {
                const QStringList parts = line.split(QRegularExpression(QStringLiteral("[,;\\s]+")), Qt::SkipEmptyParts);
                bool okX = false;
                bool okY = false;
                const double x = parts.size() == 2 ? parts[0].toDouble(&okX) : 0.0;
                const double y = parts.size() == 2 ? parts[1].toDouble(&okY) : 0.0;
                if (!okX || !okY || !std::isfinite(x) || !std::isfinite(y)) {
                    setError(QStringLiteral("Each profile line needs two numbers: radius, height."));
                    return;
                }
                profile.push_back({lengthFromUi(x), lengthFromUi(y)});
            }
            t.profile = std::move(profile);
        }
        double maxR = 0.0;
        for (const geometry::Vec2& p : t.profile) {
            maxR = std::max(maxR, p.x);
        }
        t.diameter = 2.0 * maxR;
    } else {
        t.profile.clear();
    }
    // Fields that do not apply to the type keep neutral values.
    if (!fields.includedAngle && !fields.tipDiameter) {
        t.includedAngle = 0.0;
    }
    if (!fields.tipDiameter) {
        t.tipDiameter = 0.0;
    }
    if (!fields.cornerRadius) {
        t.cornerRadius = 0.0;
    }
    QString tableError;
    if (!readCuttingTable(t, tableError)) {
        setError(tableError);
        return;
    }
    if (const auto problem = viewmodel::validateToolForEdit(t)) {
        setError(qs(*problem));
        return;
    }
    setError(QString());
    if (t == *current) {
        return;
    }
    const bool relist = t.name != current->name || t.type != current->type;
    db_.updateTool(t);
    if (relist) {
        // Keep the form as typed: only the list text changes.
        if (QTreeWidgetItem* item = toolList_->currentItem()) {
            loading_ = true;
            item->setText(0, qs(t.name));
            item->setText(1, qs(viewmodel::toolTypeLabel(t.type)));
            loading_ = false;
        }
    }
}

void ToolSetupDialog::addTool(tooling::ToolType type) {
    tooling::Tool t = viewmodel::defaultTool(type);
    t.id = viewmodel::newToolId(db_);
    t.name = "New " + viewmodel::toolTypeLabel(type);
    if (db_.addTool(t)) {
        setError(QString());
        refreshTools();
        selectTool(t.id);
    }
}

void ToolSetupDialog::duplicateTool() {
    const tooling::Tool* t = db_.findTool(currentToolId());
    if (t == nullptr) {
        return;
    }
    const std::string id = viewmodel::newToolId(db_);
    if (db_.copyTool(t->id, id, viewmodel::copyNameForTool(db_, t->name))) {
        refreshTools();
        selectTool(id);
    }
}

void ToolSetupDialog::deleteTool() {
    const tooling::Tool* t = db_.findTool(currentToolId());
    if (t == nullptr) {
        return;
    }
    QString text = QStringLiteral("Delete the tool \"%1\"?").arg(qs(t->name));
    if (uses_) {
        const std::vector<std::string> used = uses_(t->id);
        if (!used.empty()) {
            text += QStringLiteral("\n\nToolpaths made with it keep their moves but can no longer be recalculated: ");
            for (std::size_t i = 0; i < used.size() && i < 5; ++i) {
                text += (i == 0 ? QString() : QStringLiteral(", ")) + qs(used[i]);
            }
            if (used.size() > 5) {
                text += QStringLiteral(" and %1 more").arg(used.size() - 5);
            }
            text += QStringLiteral(".");
        }
    }
    if (!confirm(QStringLiteral("Delete tool"), text)) {
        return;
    }
    db_.removeTool(t->id);
    setError(QString());
    refreshTools();
}

void ToolSetupDialog::addCuttingEntry() {
    const tooling::Tool* current = db_.findTool(currentToolId());
    if (current == nullptr || entryMachine_->currentIndex() < 0 || entryMaterial_->currentIndex() < 0) {
        setError(QStringLiteral("Add a machine and a material first."));
        return;
    }
    const std::string machine = ss(entryMachine_->currentData().toString());
    const std::string material = ss(entryMaterial_->currentData().toString());
    if (current->findCuttingData(machine, material) != nullptr) {
        setError(QStringLiteral("That machine and material already have an entry."));
        return;
    }
    tooling::Tool t = *current;
    t.setCuttingData(machine, material, current->defaultCuttingData);
    db_.updateTool(t);
    setError(QString());
    loadTool();
}

void ToolSetupDialog::removeCuttingEntry() {
    const tooling::Tool* current = db_.findTool(currentToolId());
    const int row = cuttingTable_->currentRow();
    if (current == nullptr || row < 1) {
        setError(QStringLiteral("Select a machine/material row to remove (the default row stays)."));
        return;
    }
    const QTableWidgetItem* header = cuttingTable_->verticalHeaderItem(row);
    tooling::Tool t = *current;
    t.removeCuttingData(ss(header->data(kIdRole).toString()), ss(header->data(kIdRole + 2).toString()));
    db_.updateTool(t);
    setError(QString());
    loadTool();
}

void ToolSetupDialog::selectTool(const std::string& id) {
    if (QTreeWidgetItem* item = findItem(toolList_, id)) {
        toolList_->setCurrentItem(item);
    }
}

// --------------------------------------------------------------------------------------- machines

void ToolSetupDialog::loadMachine() {
    if (loading_) {
        return;
    }
    const tooling::Machine* m = db_.findMachine(currentMachineId());
    machineForm_->setEnabled(m != nullptr);
    machineDuplicate_->setEnabled(m != nullptr);
    machineDelete_->setEnabled(m != nullptr);
    if (m == nullptr) {
        return;
    }
    loading_ = true;
    machineName_->setText(qs(m->name));
    setLengthBox(workX_, m->workAreaX, units_);
    setLengthBox(workY_, m->workAreaY, units_);
    setLengthBox(workZ_, m->workAreaZ, units_);
    setLengthBox(rapid_, m->rapidRate, units_);
    setLengthBox(accelX_, m->accelerationX, units_);
    setLengthBox(accelY_, m->accelerationY, units_);
    setLengthBox(accelZ_, m->accelerationZ, units_);
    accelAngular_->setValue(m->accelerationAngular * kRadToDeg);
    accelAngular_->setProperty("rad", m->accelerationAngular);
    accelAngular_->setProperty("ui", accelAngular_->value());
    maxSpindle_->setValue(m->maxSpindleSpeed);
    setLengthBox(maxFeed_, m->maxFeedRate, units_);
    QStringList posts;
    for (const std::string& p : m->associatedPosts) {
        posts << qs(p);
    }
    posts_->setText(posts.join(QStringLiteral(", ")));
    defaultPost_->clear();
    defaultPost_->addItem(QString());
    for (const std::string& p : m->associatedPosts) {
        defaultPost_->addItem(qs(p));
    }
    const int at = defaultPost_->findText(qs(m->defaultPost));
    defaultPost_->setCurrentIndex(at < 0 ? 0 : at);
    loading_ = false;
    machineUse_->setEnabled(m->id != activeMachineId_);
}

void ToolSetupDialog::commitMachine() {
    if (loading_) {
        return;
    }
    const tooling::Machine* current = db_.findMachine(currentMachineId());
    if (current == nullptr) {
        return;
    }
    tooling::Machine m = *current;
    m.name = ss(machineName_->text());
    m.workAreaX = lengthOf(workX_, units_);
    m.workAreaY = lengthOf(workY_, units_);
    m.workAreaZ = lengthOf(workZ_, units_);
    m.rapidRate = lengthOf(rapid_, units_);
    m.accelerationX = lengthOf(accelX_, units_);
    m.accelerationY = lengthOf(accelY_, units_);
    m.accelerationZ = lengthOf(accelZ_, units_);
    m.accelerationAngular = accelAngular_->value() == accelAngular_->property("ui").toDouble()
                                ? accelAngular_->property("rad").toDouble()
                                : accelAngular_->value() / kRadToDeg;
    m.maxSpindleSpeed = maxSpindle_->value();
    m.maxFeedRate = lengthOf(maxFeed_, units_);
    m.associatedPosts.clear();
    for (const QString& p : posts_->text().split(QLatin1Char(','), Qt::SkipEmptyParts)) {
        const std::string name = ss(p.trimmed());
        if (!name.empty() && std::find(m.associatedPosts.begin(), m.associatedPosts.end(), name) == m.associatedPosts.end()) {
            m.associatedPosts.push_back(name);
        }
    }
    m.defaultPost = ss(defaultPost_->currentText());
    if (const auto problem = viewmodel::validateMachine(m)) {
        setError(qs(*problem));
        return;
    }
    setError(QString());
    if (m == *current) {
        return;
    }
    db_.updateMachine(m);
    if (QTreeWidgetItem* item = machineList_->currentItem()) {
        loading_ = true;
        item->setText(0, qs(m.name));
        loading_ = false;
    }
    // Cutting-data entry combos show machine names.
    loading_ = true;
    for (int i = 0; i < entryMachine_->count(); ++i) {
        if (ss(entryMachine_->itemData(i).toString()) == m.id) {
            entryMachine_->setItemText(i, qs(m.name));
        }
    }
    loading_ = false;
}

void ToolSetupDialog::addMachine() {
    tooling::Machine m = viewmodel::defaultMachine("New machine");
    m.id = viewmodel::newMachineId(db_);
    if (db_.addMachine(m)) {
        setError(QString());
        if (activeMachineId_.empty() || db_.findMachine(activeMachineId_) == nullptr) {
            activeMachineId_ = m.id;
        }
        refreshMachines();
        selectMachine(m.id);
        refreshTools();
    }
}

void ToolSetupDialog::duplicateMachine() {
    const tooling::Machine* m = db_.findMachine(currentMachineId());
    if (m == nullptr) {
        return;
    }
    const std::string id = viewmodel::newMachineId(db_);
    if (db_.duplicateMachine(m->id, id, m->name + " copy", true)) {
        refreshMachines();
        selectMachine(id);
        refreshTools();
    }
}

void ToolSetupDialog::deleteMachine() {
    const tooling::Machine* m = db_.findMachine(currentMachineId());
    if (m == nullptr) {
        return;
    }
    if (!confirm(QStringLiteral("Delete machine"),
                 QStringLiteral("Delete the machine \"%1\" and every tool's feeds and speeds for it?").arg(qs(m->name)))) {
        return;
    }
    const std::string id = m->id;
    db_.removeMachine(id, true);
    if (activeMachineId_ == id) {
        const tooling::Machine* next = viewmodel::activeMachine(db_, std::string());
        activeMachineId_ = next != nullptr ? next->id : std::string();
    }
    setError(QString());
    refreshMachines();
    refreshTools();
}

void ToolSetupDialog::selectMachine(const std::string& id) {
    if (QTreeWidgetItem* item = findItem(machineList_, id)) {
        machineList_->setCurrentItem(item);
    }
}

// -------------------------------------------------------------------------------------- materials

void ToolSetupDialog::addMaterial() {
    tooling::Material m{viewmodel::newMaterialId(db_), "New material"};
    if (db_.addMaterial(m)) {
        setError(QString());
        refreshMaterials();
        if (QTreeWidgetItem* item = findItem(materialList_, m.id)) {
            materialList_->setCurrentItem(item);
        }
        refreshTools();
    }
}

void ToolSetupDialog::duplicateMaterial() {
    const tooling::Material* m = db_.findMaterial(currentMaterialId());
    if (m == nullptr) {
        return;
    }
    const std::string id = viewmodel::newMaterialId(db_);
    if (db_.duplicateMaterial(m->id, id, m->name + " copy", true)) {
        refreshMaterials();
        if (QTreeWidgetItem* item = findItem(materialList_, id)) {
            materialList_->setCurrentItem(item);
        }
        refreshTools();
    }
}

void ToolSetupDialog::deleteMaterial() {
    const tooling::Material* m = db_.findMaterial(currentMaterialId());
    if (m == nullptr) {
        return;
    }
    if (!confirm(QStringLiteral("Delete material"),
                 QStringLiteral("Delete the material \"%1\" and every tool's feeds and speeds for it?").arg(qs(m->name)))) {
        return;
    }
    db_.removeMaterial(m->id);
    setError(QString());
    refreshMaterials();
    refreshTools();
}

// ------------------------------------------------------------------------------ import and export

bool ToolSetupDialog::importFile(const std::filesystem::path& path) {
    const tooling::ImportResult r = viewmodel::readToolDatabaseFile(path);
    if (!r.ok) {
        setError(qs(r.error));
        if (!quiet_) {
            QMessageBox::warning(this, QStringLiteral("Import tools"), qs(r.error));
        }
        return false;
    }
    const tooling::MergeReport report = db_.merge(r.database, tooling::MergePolicy::KeepExisting);
    setError(QString());
    refreshAll();
    if (!quiet_) {
        QMessageBox::information(
            this, QStringLiteral("Import tools"),
            QStringLiteral("Added %1 tool(s), %2 machine(s) and %3 material(s); items that were already here were kept.")
                .arg(report.tools.added)
                .arg(report.machines.added)
                .arg(report.materials.added));
    }
    return true;
}

bool ToolSetupDialog::exportFile(const std::filesystem::path& path) {
    std::string error;
    if (!viewmodel::writeToolDatabaseFile(path, db_, error)) {
        setError(qs(error));
        if (!quiet_) {
            QMessageBox::warning(this, QStringLiteral("Export tools"), qs(error));
        }
        return false;
    }
    setError(QString());
    return true;
}

void ToolSetupDialog::accept() {
    if (const auto problem = viewmodel::validateSetup(db_)) {
        setError(qs(*problem));
        return;
    }
    QDialog::accept();
}

}  // namespace carvetto::app
