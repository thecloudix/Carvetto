#include "relief_panels.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDropEvent>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
#include <limits>

namespace carvetto::app {

namespace {

constexpr int kRoleId = Qt::UserRole;
constexpr int kRoleIsLevel = Qt::UserRole + 1;
constexpr int kColName = 0;
constexpr int kColMode = 1;
constexpr int kColShow = 2;

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

QPushButton* button(const char* text, const char* tip, QWidget* parent) {
    auto* b = new QPushButton(QString::fromUtf8(text), parent);
    b->setToolTip(QString::fromUtf8(tip));
    return b;
}

std::uint64_t idOf(const QTreeWidgetItem* item) {
    return item->data(kColName, kRoleId).toULongLong();
}

bool isLevel(const QTreeWidgetItem* item) {
    return item->data(kColName, kRoleIsLevel).toBool();
}

void fillModeBox(QComboBox* box) {
    for (const relief::CombineMode m : viewmodel::allCombineModes()) {
        box->addItem(QString::fromUtf8(viewmodel::combineModeName(m)), static_cast<int>(m));
    }
}

}  // namespace

// --- Tree -------------------------------------------------------------------------------------

void ReliefTreeWidget::dropEvent(QDropEvent* event) {
    QTreeWidgetItem* dragged = currentItem();
    const QPoint pos = event->position().toPoint();
    QTreeWidgetItem* target = itemAt(pos);
    event->ignore();  // the tree is rebuilt from the document, never edited by Qt's own move
    if (dragged == nullptr || isLevel(dragged) || target == nullptr || !onDrop) {
        return;
    }
    const document::ComponentId id{idOf(dragged)};
    // Items list top of the stack first, so dropping onto a component puts it just above that
    // component (bottom-first index + 1) or below it when released in the lower half.
    if (isLevel(target)) {
        onDrop(id, document::LevelId{idOf(target)}, std::numeric_limits<std::size_t>::max());
        return;
    }
    QTreeWidgetItem* parent = target->parent();
    if (parent == nullptr) {
        return;
    }
    const int row = parent->indexOfChild(target);
    const int count = parent->childCount();
    const bool lowerHalf = pos.y() > visualItemRect(target).center().y();
    // Bottom-first index of the target component is count - 1 - row.
    std::size_t index = static_cast<std::size_t>(count - 1 - row);
    if (!lowerHalf) {
        index += 1;
    }
    onDrop(id, document::LevelId{idOf(parent)}, index);
}

ComponentTreePanel::ComponentTreePanel(DocumentGetter getter, QWidget* parent)
    : PanelBase(std::move(getter), parent) {
    auto* outer = new QVBoxLayout(this);
    tree_ = new ReliefTreeWidget(this);
    tree_->setColumnCount(3);
    tree_->setHeaderLabels({QStringLiteral("Component"), QStringLiteral("Mode"), QStringLiteral("Show")});
    tree_->setUniformRowHeights(true);
    tree_->setSelectionMode(QAbstractItemView::ExtendedSelection);
    tree_->setDragDropMode(QAbstractItemView::InternalMove);
    tree_->setDragEnabled(true);
    tree_->setAcceptDrops(true);
    tree_->setToolTip(QStringLiteral("Levels and components, top of the stack first. Double-click to rename; "
                                     "drag a component to reorder it or move it to another level."));
    tree_->header()->setStretchLastSection(false);
    tree_->header()->setSectionResizeMode(kColName, QHeaderView::Stretch);
    outer->addWidget(tree_, 1);

    auto* row1 = new QHBoxLayout();
    QPushButton* addLevelBtn = button("Add level", "Add a level on top", this);
    QPushButton* dup = button("Duplicate", "Duplicate the selected component", this);
    QPushButton* del = button("Delete", "Delete the selected components", this);
    for (QPushButton* b : {addLevelBtn, dup, del}) {
        row1->addWidget(b);
    }
    outer->addLayout(row1);
    auto* row2 = new QHBoxLayout();
    QPushButton* up = button("Up", "Move up the stack", this);
    QPushButton* down = button("Down", "Move down the stack", this);
    QPushButton* only = button("Show only", "Show only the selected component", this);
    QPushButton* all = button("Show all", "Show every component", this);
    for (QPushButton* b : {up, down, only, all}) {
        row2->addWidget(b);
    }
    outer->addLayout(row2);
    auto* row3 = new QHBoxLayout();
    mode_ = new QComboBox(this);
    mode_->setToolTip(QStringLiteral("Combine mode of the selected component"));
    fillModeBox(mode_);
    moveTo_ = new QComboBox(this);
    moveTo_->setToolTip(QStringLiteral("Move the selected component to this level"));
    row3->addWidget(mode_);
    row3->addWidget(moveTo_, 1);
    outer->addLayout(row3);

    connect(addLevelBtn, &QPushButton::clicked, this, [this]() { addLevel(); });
    connect(dup, &QPushButton::clicked, this, [this]() { duplicateSelected(); });
    connect(del, &QPushButton::clicked, this, [this]() { deleteSelected(); });
    connect(up, &QPushButton::clicked, this, [this]() { moveSelected(true); });
    connect(down, &QPushButton::clicked, this, [this]() { moveSelected(false); });
    connect(only, &QPushButton::clicked, this, [this]() {
        if (document::Document* d = doc(); d != nullptr && d->componentSelection().single()) {
            finish(viewmodel::showOnlyReliefComponent(*d, d->componentSelection().single()));
        }
    });
    connect(all, &QPushButton::clicked, this, [this]() {
        if (document::Document* d = doc(); d != nullptr) {
            finish(viewmodel::setAllReliefComponentsVisible(*d, d->activeSheet(), true));
        }
    });
    connect(mode_, &QComboBox::activated, this, [this](int index) {
        if (!refreshing_ && index >= 0) {
            setSelectedMode(static_cast<relief::CombineMode>(mode_->itemData(index).toInt()));
        }
    });
    connect(moveTo_, &QComboBox::activated, this, [this](int index) {
        if (!refreshing_ && index >= 0) {
            moveSelectedToLevel(document::LevelId{moveTo_->itemData(index).toULongLong()});
        }
    });
    connect(tree_, &QTreeWidget::itemChanged, this, [this](QTreeWidgetItem* item, int column) { itemEdited(item, column); });
    connect(tree_, &QTreeWidget::itemSelectionChanged, this, [this]() { selectionEdited(); });
    connect(tree_, &QTreeWidget::currentItemChanged, this, [this](QTreeWidgetItem* item, QTreeWidgetItem*) {
        if (!refreshing_ && onLevelPicked) {
            onLevelPicked(item != nullptr && isLevel(item) ? document::LevelId{idOf(item)} : document::LevelId{});
        }
    });
    tree_->onDrop = [this](document::ComponentId id, document::LevelId level, std::size_t index) {
        drop(id, level, index);
    };
}

int ComponentTreePanel::topLevelCount() const {
    return tree_->topLevelItemCount();
}

QTreeWidgetItem* ComponentTreePanel::levelItem(document::LevelId id) const {
    for (int i = 0; i < tree_->topLevelItemCount(); ++i) {
        if (idOf(tree_->topLevelItem(i)) == id.value) {
            return tree_->topLevelItem(i);
        }
    }
    return nullptr;
}

QTreeWidgetItem* ComponentTreePanel::componentItem(document::ComponentId id) const {
    for (int i = 0; i < tree_->topLevelItemCount(); ++i) {
        QTreeWidgetItem* level = tree_->topLevelItem(i);
        for (int c = 0; c < level->childCount(); ++c) {
            if (idOf(level->child(c)) == id.value) {
                return level->child(c);
            }
        }
    }
    return nullptr;
}

void ComponentTreePanel::refresh() {
    const document::Document* d = doc();
    refreshing_ = true;
    const QSignalBlocker blocker(tree_);
    const std::vector<viewmodel::ReliefTreeRow> rows =
        d != nullptr ? viewmodel::buildReliefTree(*d, d->activeSheet()) : std::vector<viewmodel::ReliefTreeRow>{};

    // Same structure as the items: update in place (a refresh can run from inside this tree's own
    // itemChanged signal, so no item may be destroyed then). Otherwise rebuild.
    bool same = true;
    std::size_t k = 0;
    for (int i = 0; i < tree_->topLevelItemCount() && same; ++i) {
        const QTreeWidgetItem* level = tree_->topLevelItem(i);
        same = k < rows.size() && rows[k].isLevel && rows[k].level.value == idOf(level);
        ++k;
        for (int c = 0; c < level->childCount() && same; ++c, ++k) {
            same = k < rows.size() && !rows[k].isLevel && rows[k].component.value == idOf(level->child(c));
        }
    }
    same = same && k == rows.size();
    if (!same) {
        tree_->clear();
        QTreeWidgetItem* current = nullptr;
        for (const viewmodel::ReliefTreeRow& row : rows) {
            QTreeWidgetItem* item = row.isLevel ? new QTreeWidgetItem(tree_) : new QTreeWidgetItem(current);
            item->setData(kColName, kRoleId, static_cast<qulonglong>(row.isLevel ? row.level.value : row.component.value));
            item->setData(kColName, kRoleIsLevel, row.isLevel);
            item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsEditable | Qt::ItemIsUserCheckable |
                           (row.isLevel ? Qt::ItemFlags{} : Qt::ItemIsDragEnabled));
            if (row.isLevel) {
                current = item;
                item->setExpanded(true);
            }
        }
    }
    QTreeWidgetItem* level = nullptr;
    int childRow = 0;
    int levelRow = -1;
    for (const viewmodel::ReliefTreeRow& row : rows) {
        QTreeWidgetItem* item = nullptr;
        if (row.isLevel) {
            level = tree_->topLevelItem(++levelRow);
            childRow = 0;
            item = level;
        } else {
            item = level->child(childRow++);
        }
        item->setText(kColName, qs(row.name));
        item->setText(kColMode, QString::fromUtf8(viewmodel::combineModeName(row.mode)));
        item->setCheckState(kColShow, row.visible ? Qt::Checked : Qt::Unchecked);
        item->setSelected(!row.isLevel && row.selected);
        QFont f = item->font(kColName);
        f.setBold(row.isLevel);
        item->setFont(kColName, f);
    }
    updateMoveTargets();
    if (d != nullptr) {
        const document::ReliefComponent* sel = d->findReliefComponent(d->componentSelection().single());
        if (sel != nullptr) {
            const int idx = mode_->findData(static_cast<int>(sel->data.mode));
            mode_->setCurrentIndex(idx);
        }
    }
    refreshing_ = false;
}

void ComponentTreePanel::updateMoveTargets() {
    moveTo_->clear();
    moveTo_->addItem(QStringLiteral("Move to level..."), 0);
    const document::Document* d = doc();
    if (d == nullptr) {
        return;
    }
    for (const viewmodel::ReliefTreeRow& row : viewmodel::buildReliefTree(*d, d->activeSheet())) {
        if (row.isLevel) {
            moveTo_->addItem(qs(row.name), static_cast<qulonglong>(row.level.value));
        }
    }
}

void ComponentTreePanel::itemEdited(QTreeWidgetItem* item, int column) {
    document::Document* d = doc();
    if (refreshing_ || d == nullptr || item == nullptr) {
        return;
    }
    const bool level = isLevel(item);
    viewmodel::EditResult r;
    if (column == kColName) {
        const std::string name = item->text(kColName).toStdString();
        r = level ? viewmodel::renameReliefLevel(*d, document::LevelId{idOf(item)}, name)
                  : viewmodel::renameReliefComponent(*d, document::ComponentId{idOf(item)}, name);
    } else if (column == kColShow) {
        const bool on = item->checkState(kColShow) == Qt::Checked;
        r = level ? viewmodel::setReliefLevelVisible(*d, document::LevelId{idOf(item)}, on)
                  : viewmodel::setReliefComponentVisible(*d, document::ComponentId{idOf(item)}, on);
    } else {
        return;
    }
    if (!finish(r)) {
        refresh();  // restores the text or check box the user changed
    }
}

void ComponentTreePanel::selectionEdited() {
    document::Document* d = doc();
    if (refreshing_ || d == nullptr) {
        return;
    }
    std::vector<document::ComponentId> ids;
    for (QTreeWidgetItem* item : tree_->selectedItems()) {
        if (!isLevel(item)) {
            ids.push_back(document::ComponentId{idOf(item)});
        }
    }
    viewmodel::selectReliefComponents(*d, ids);
    if (onSelectionChanged) {
        onSelectionChanged();
    }
}

bool ComponentTreePanel::addLevel(const std::string& name) {
    document::Document* d = doc();
    return d != nullptr && finish(viewmodel::addReliefLevel(*d, d->activeSheet(), name));
}

bool ComponentTreePanel::deleteSelected() {
    document::Document* d = doc();
    if (d == nullptr) {
        return false;
    }
    if (d->componentSelection().empty()) {
        // A current level row deletes that level.
        QTreeWidgetItem* cur = tree_->currentItem();
        if (cur != nullptr && isLevel(cur)) {
            return finish(viewmodel::deleteReliefLevel(*d, document::LevelId{idOf(cur)}));
        }
    }
    return finish(viewmodel::deleteReliefComponents(*d, d->componentSelection().ids()));
}

bool ComponentTreePanel::duplicateSelected() {
    document::Document* d = doc();
    if (d == nullptr) {
        return false;
    }
    const document::ComponentId id = d->componentSelection().single();
    if (!id) {
        if (onMessage) {
            onMessage("Select one component to duplicate");
        }
        return false;
    }
    document::ComponentId copy;
    const bool ok = finish(viewmodel::duplicateReliefComponent(*d, id, &copy));
    if (ok && copy) {
        viewmodel::selectReliefComponents(*d, {copy});
        refresh();
    }
    return ok;
}

bool ComponentTreePanel::moveSelected(bool up) {
    document::Document* d = doc();
    if (d == nullptr) {
        return false;
    }
    const document::ComponentId id = d->componentSelection().single();
    if (id) {
        return finish(viewmodel::moveReliefComponent(*d, id, up));
    }
    QTreeWidgetItem* cur = tree_->currentItem();
    if (cur != nullptr && isLevel(cur)) {
        return finish(viewmodel::moveReliefLevel(*d, document::LevelId{idOf(cur)}, up));
    }
    return false;
}

bool ComponentTreePanel::setSelectedMode(relief::CombineMode mode) {
    document::Document* d = doc();
    if (d == nullptr) {
        return false;
    }
    const document::ComponentId id = d->componentSelection().single();
    if (id) {
        return finish(viewmodel::setReliefComponentMode(*d, id, mode));
    }
    QTreeWidgetItem* cur = tree_->currentItem();
    if (cur != nullptr && isLevel(cur)) {
        return finish(viewmodel::setReliefLevelMode(*d, document::LevelId{idOf(cur)}, mode));
    }
    return false;
}

bool ComponentTreePanel::toggleVisible(document::ComponentId id) {
    document::Document* d = doc();
    const document::ReliefComponent* c = d != nullptr ? d->findReliefComponent(id) : nullptr;
    return c != nullptr && finish(viewmodel::setReliefComponentVisible(*d, id, !c->data.visible));
}

bool ComponentTreePanel::toggleLevelVisible(document::LevelId id) {
    document::Document* d = doc();
    const document::ReliefLevel* l = d != nullptr ? d->findReliefLevel(id) : nullptr;
    return l != nullptr && finish(viewmodel::setReliefLevelVisible(*d, id, !l->visible));
}

bool ComponentTreePanel::rename(document::ComponentId id, const std::string& name) {
    document::Document* d = doc();
    return d != nullptr && finish(viewmodel::renameReliefComponent(*d, id, name));
}

bool ComponentTreePanel::moveSelectedToLevel(document::LevelId level) {
    document::Document* d = doc();
    if (d == nullptr || !level) {
        return false;
    }
    const document::ComponentId id = d->componentSelection().single();
    return id && finish(viewmodel::moveReliefComponentToLevel(*d, id, level));
}

bool ComponentTreePanel::drop(document::ComponentId id, document::LevelId level, std::size_t index) {
    document::Document* d = doc();
    return d != nullptr && finish(viewmodel::dropReliefComponent(*d, id, level, index));
}

void ComponentTreePanel::selectComponents(const std::vector<document::ComponentId>& ids) {
    document::Document* d = doc();
    if (d == nullptr) {
        return;
    }
    viewmodel::selectReliefComponents(*d, ids);
    refresh();
    if (onSelectionChanged) {
        onSelectionChanged();
    }
}

// --- Properties -------------------------------------------------------------------------------

ComponentPropertiesPanel::ComponentPropertiesPanel(DocumentGetter getter, QWidget* parent)
    : PanelBase(std::move(getter), parent) {
    auto* outer = new QVBoxLayout(this);
    auto* form = new QFormLayout();
    auto edit = [this, form](const char* label, const char* tip) {
        auto* e = new QLineEdit(this);
        e->setObjectName(QString::fromUtf8(label));
        e->setToolTip(QString::fromUtf8(tip));
        form->addRow(QString::fromUtf8(label), e);
        return e;
    };
    name_ = edit("Name", "Name of the component");
    mode_ = new QComboBox(this);
    mode_->setObjectName(QStringLiteral("Combine mode"));
    fillModeBox(mode_);
    form->addRow(QStringLiteral("Combine mode"), mode_);
    shape_ = edit("Shape height %", "Scale of the stored height, 0 to 1000 %");
    base_ = edit("Base height", "Z offset added after the shape height, in mm");
    fadeOn_ = new QCheckBox(QStringLiteral("Fade"), this);
    form->addRow(fadeOn_);
    fadePct_ = edit("Fade %", "Height lost at the second point");
    fadeAx_ = edit("Fade anchor X", "Fade anchor point, mm");
    fadeAy_ = edit("Fade anchor Y", "");
    fadeBx_ = edit("Fade second X", "Second point of the fade, mm");
    fadeBy_ = edit("Fade second Y", "");
    tiltOn_ = new QCheckBox(QStringLiteral("Tilt"), this);
    form->addRow(tiltOn_);
    tiltAngle_ = edit("Tilt angle", "Degrees, below 89");
    tiltPx_ = edit("Tilt pivot X", "Pivot point, mm");
    tiltPy_ = edit("Tilt pivot Y", "");
    tiltRx_ = edit("Tilt raise X", "Point to raise, mm");
    tiltRy_ = edit("Tilt raise Y", "");
    outer->addLayout(form);
    info_ = new QLabel(this);
    outer->addWidget(info_);
    apply_ = button("Apply", "Write the values to the model (one undo step)", this);
    outer->addWidget(apply_);
    outer->addStretch(1);
    connect(apply_, &QPushButton::clicked, this, [this]() { apply(); });
    refresh();
}

QLineEdit* ComponentPropertiesPanel::field(const QString& label) const {
    return findChild<QLineEdit*>(label);
}

void ComponentPropertiesPanel::setLevel(document::LevelId level) {
    level_ = level;
    refresh();
}

void ComponentPropertiesPanel::refresh() {
    const document::Document* d = doc();
    component_ = d != nullptr ? d->componentSelection().single() : document::ComponentId{};
    if (component_ && d->findReliefComponent(component_) == nullptr) {
        component_ = {};
    }
    if (!component_ && level_ && (d == nullptr || d->findReliefLevel(level_) == nullptr)) {
        level_ = {};
    }
    fill();
}

void ComponentPropertiesPanel::fill() {
    refreshing_ = true;
    const document::Document* d = doc();
    viewmodel::ReliefPropertiesForm form;
    showing_ = d != nullptr && component_ && viewmodel::describeReliefComponent(*d, component_, form);
    const bool levelMode = !showing_ && d != nullptr && level_ && d->findReliefLevel(level_) != nullptr;
    const bool detail = showing_;
    for (QWidget* w : std::initializer_list<QWidget*>{shape_, fadePct_, fadeAx_, fadeAy_, fadeBx_, fadeBy_, tiltAngle_,
                                                      tiltPx_, tiltPy_, tiltRx_, tiltRy_, fadeOn_, tiltOn_}) {
        w->setEnabled(detail);
    }
    for (QWidget* w : std::initializer_list<QWidget*>{name_, mode_, base_, apply_}) {
        w->setEnabled(showing_ || levelMode);
    }
    if (levelMode) {
        const document::ReliefLevel* l = d->findReliefLevel(level_);
        name_->setText(qs(l->name));
        mode_->setCurrentIndex(mode_->findData(static_cast<int>(l->mode)));
        base_->setText(QString::number(l->baseHeight, 'g', 10));
        info_->setText(QStringLiteral("Level: name, combine mode and base height"));
    } else if (showing_) {
        name_->setText(qs(form.name));
        mode_->setCurrentIndex(mode_->findData(static_cast<int>(form.mode)));
        shape_->setText(qs(form.shapeHeightPercent));
        base_->setText(qs(form.baseHeight));
        fadeOn_->setChecked(form.fadeEnabled);
        fadePct_->setText(qs(form.fadePercent));
        fadeAx_->setText(qs(form.fadeAnchorX));
        fadeAy_->setText(qs(form.fadeAnchorY));
        fadeBx_->setText(qs(form.fadeSecondX));
        fadeBy_->setText(qs(form.fadeSecondY));
        tiltOn_->setChecked(form.tiltEnabled);
        tiltAngle_->setText(qs(form.tiltAngleDeg));
        tiltPx_->setText(qs(form.tiltPivotX));
        tiltPy_->setText(qs(form.tiltPivotY));
        tiltRx_->setText(qs(form.tiltRaiseX));
        tiltRy_->setText(qs(form.tiltRaiseY));
        info_->setText(qs(form.fieldInfo));
    } else {
        for (QLineEdit* e : {name_, shape_, base_, fadePct_, fadeAx_, fadeAy_, fadeBx_, fadeBy_, tiltAngle_, tiltPx_,
                             tiltPy_, tiltRx_, tiltRy_}) {
            e->clear();
        }
        info_->setText(QStringLiteral("Select a component or a level"));
    }
    refreshing_ = false;
}

bool ComponentPropertiesPanel::apply() {
    document::Document* d = doc();
    if (d == nullptr || refreshing_) {
        return false;
    }
    const relief::CombineMode mode = static_cast<relief::CombineMode>(mode_->currentData().toInt());
    if (showing_) {
        viewmodel::ReliefPropertiesForm f;
        if (!viewmodel::describeReliefComponent(*d, component_, f)) {
            return false;
        }
        f.name = name_->text().toStdString();
        f.mode = mode;
        f.shapeHeightPercent = shape_->text().toStdString();
        f.baseHeight = base_->text().toStdString();
        f.fadeEnabled = fadeOn_->isChecked();
        f.fadePercent = fadePct_->text().toStdString();
        f.fadeAnchorX = fadeAx_->text().toStdString();
        f.fadeAnchorY = fadeAy_->text().toStdString();
        f.fadeSecondX = fadeBx_->text().toStdString();
        f.fadeSecondY = fadeBy_->text().toStdString();
        f.tiltEnabled = tiltOn_->isChecked();
        f.tiltAngleDeg = tiltAngle_->text().toStdString();
        f.tiltPivotX = tiltPx_->text().toStdString();
        f.tiltPivotY = tiltPy_->text().toStdString();
        f.tiltRaiseX = tiltRx_->text().toStdString();
        f.tiltRaiseY = tiltRy_->text().toStdString();
        const bool ok = finish(viewmodel::applyReliefProperties(*d, component_, f));
        if (!ok) {
            return false;
        }
        return true;
    }
    if (level_ && d->findReliefLevel(level_) != nullptr) {
        return finish(viewmodel::applyReliefLevelProperties(*d, level_, name_->text().toStdString(), mode,
                                                            base_->text().toStdString()));
    }
    return false;
}

// --- 3D view ----------------------------------------------------------------------------------

ModelView3D::ModelView3D(DocumentGetter getter, QWidget* parent) : QWidget(parent), getter_(std::move(getter)) {
    setMinimumSize(200, 150);
    setMouseTracking(false);
    setFocusPolicy(Qt::StrongFocus);
    setToolTip(QStringLiteral("Right-drag orbits, the wheel zooms, middle-drag or Ctrl+right-drag pans"));
    auto* bar = new QHBoxLayout(this);
    bar->setContentsMargins(6, 6, 6, 6);
    auto view = [this, bar](const char* text, void (viewmodel::OrbitCamera::*fn)()) {
        auto* b = new QPushButton(QString::fromUtf8(text), this);
        connect(b, &QPushButton::clicked, this, [this, fn]() {
            (camera_.*fn)();
            cameraChanged();
        });
        bar->addWidget(b);
    };
    view("Top", &viewmodel::OrbitCamera::viewTop);
    view("Front", &viewmodel::OrbitCamera::viewFront);
    view("Right", &viewmodel::OrbitCamera::viewRight);
    view("Fit", &viewmodel::OrbitCamera::reset);
    materialBox_ = new QComboBox(this);
    for (const viewmodel::ModelMaterial m :
         {viewmodel::ModelMaterial::SolidColour, viewmodel::ModelMaterial::Wood, viewmodel::ModelMaterial::Metal,
          viewmodel::ModelMaterial::Stone, viewmodel::ModelMaterial::Plastic}) {
        materialBox_->addItem(QString::fromUtf8(viewmodel::modelMaterialName(m)), static_cast<int>(m));
    }
    materialBox_->setCurrentIndex(materialBox_->findData(static_cast<int>(options_.material)));
    connect(materialBox_, &QComboBox::activated, this, [this](int index) {
        setMaterial(static_cast<viewmodel::ModelMaterial>(materialBox_->itemData(index).toInt()));
    });
    bar->addWidget(materialBox_);
    bar->addStretch(1);
    bar->setAlignment(Qt::AlignTop);
}

void ModelView3D::refresh() {
    document::Document* d = getter_ ? getter_() : nullptr;
    if (previewing_) {
        surface_ = previewSurface_;
    } else if (d != nullptr) {
        surface_ = viewmodel::composeModelSurface(*d, d->activeSheet());
    } else {
        surface_ = viewmodel::ModelSurface{};
    }
    invalidate();
}

void ModelView3D::setPreview(const relief::HeightField& field) {
    viewmodel::ModelSurface s;
    const std::size_t longest = std::max(field.width(), field.height());
    if (field.empty()) {
        s.problem = "Nothing to preview";
    } else if (longest <= viewmodel::kMaxSurfaceDimension) {
        s.field = field;
    } else {
        s.stride = (longest + viewmodel::kMaxSurfaceDimension - 1) / viewmodel::kMaxSurfaceDimension;
        const relief::Grid g{(field.width() + s.stride - 1) / s.stride, (field.height() + s.stride - 1) / s.stride,
                             field.cellSize() * static_cast<double>(s.stride), field.origin()};
        if (field.resample(g, s.field) != relief::Status::Ok) {
            s.problem = "The preview is too large to show";
        }
    }
    previewSurface_ = std::move(s);
    previewing_ = true;
    surface_ = previewSurface_;
    invalidate();
}

void ModelView3D::clearPreview() {
    if (previewing_) {
        previewing_ = false;
        refresh();
    }
}

void ModelView3D::setMaterial(viewmodel::ModelMaterial material) {
    options_.material = material;
    const int idx = materialBox_->findData(static_cast<int>(material));
    if (idx >= 0) {
        materialBox_->setCurrentIndex(idx);
    }
    invalidate();
}

void ModelView3D::cameraChanged() {
    invalidate();
}

void ModelView3D::invalidate() {
    dirty_ = true;
    update();
}

const QImage& ModelView3D::image() {
    const std::size_t w = static_cast<std::size_t>(std::max(width(), 1));
    const std::size_t h = static_cast<std::size_t>(std::max(height(), 1));
    if (!dirty_ && image_.width() == static_cast<int>(w) && image_.height() == static_cast<int>(h)) {
        return image_;
    }
    options_.width = w;
    options_.height = h;
    const viewmodel::Raster raster = viewmodel::renderModel(surface_, camera_, options_);
    QImage img(static_cast<int>(raster.width), static_cast<int>(raster.height), QImage::Format_ARGB32);
    for (std::size_t y = 0; y < raster.height; ++y) {
        auto* line = reinterpret_cast<QRgb*>(img.scanLine(static_cast<int>(y)));
        for (std::size_t x = 0; x < raster.width; ++x) {
            line[x] = raster.pixels[y * raster.width + x];
        }
    }
    image_ = img;
    dirty_ = false;
    return image_;
}

void ModelView3D::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.drawImage(0, 0, image());
    if (!surface_.ok()) {
        p.setPen(QColor(90, 90, 90));
        p.drawText(rect(), Qt::AlignCenter | Qt::TextWordWrap,
                   surface_.problem.empty() ? QStringLiteral("No model") : qs(surface_.problem));
    }
}

void ModelView3D::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    invalidate();
}

void ModelView3D::mousePressEvent(QMouseEvent* event) {
    last_ = event->pos();
    setFocus();
}

void ModelView3D::mouseMoveEvent(QMouseEvent* event) {
    const QPoint delta = event->pos() - last_;
    last_ = event->pos();
    const bool pan = (event->buttons() & Qt::MiddleButton) ||
                     ((event->buttons() & Qt::RightButton) && (event->modifiers() & Qt::ControlModifier)) ||
                     ((event->buttons() & Qt::LeftButton) && (event->buttons() & Qt::RightButton));
    if (pan) {
        camera_.pan(delta.x(), delta.y());
    } else if (event->buttons() & Qt::RightButton) {
        camera_.orbit(delta.x(), delta.y());
    } else {
        return;
    }
    cameraChanged();
}

void ModelView3D::wheelEvent(QWheelEvent* event) {
    camera_.zoomBy(std::pow(1.0015, event->angleDelta().y()));
    cameraChanged();
}

}  // namespace carvetto::app
