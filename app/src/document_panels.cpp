#include "document_panels.hpp"
#include "variable_menu.hpp"

#include <QBrush>
#include <QColor>
#include <QColorDialog>
#include <QComboBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QString>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <cstdint>
#include <initializer_list>
#include <optional>
#include <string>
#include <utility>

#include "carvetto/viewmodel/layers_sheets.hpp"
#include "carvetto/viewmodel/tools.hpp"
#include "carvetto/viewmodel/units_format.hpp"

namespace carvetto::app {

namespace {

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

QPushButton* button(const char* text, const char* tip, QWidget* parent) {
    auto* b = new QPushButton(QString::fromUtf8(text), parent);
    b->setToolTip(QString::fromUtf8(tip));
    return b;
}

QColor toQColor(document::Colour c) {
    return QColor(c.r, c.g, c.b);
}

constexpr int kColName = 0;
constexpr int kColVisible = 1;
constexpr int kColLocked = 2;
constexpr int kColColour = 3;

}  // namespace

bool PanelBase::finish(const viewmodel::EditResult& r) {
    if (!r.ok) {
        if (onMessage) {
            onMessage(r.message);
        }
        return false;
    }
    if (r.changed && onEdited) {
        onEdited();
    }
    return true;
}

// --- Layers -----------------------------------------------------------------------------------

LayersPanel::LayersPanel(DocumentGetter getter, QWidget* parent) : PanelBase(std::move(getter), parent) {
    auto* outer = new QVBoxLayout(this);
    tree_ = new QTreeWidget(this);
    tree_->setColumnCount(4);
    tree_->setHeaderLabels({QStringLiteral("Layer"), QStringLiteral("Show"), QStringLiteral("Lock"),
                            QStringLiteral("Colour")});
    tree_->setRootIsDecorated(false);
    tree_->setUniformRowHeights(true);
    tree_->setToolTip(QStringLiteral("Stacking order, bottom first. Double-click a name to rename it."));
    tree_->header()->setStretchLastSection(false);
    tree_->header()->setSectionResizeMode(kColName, QHeaderView::Stretch);
    outer->addWidget(tree_, 1);

    auto* row1 = new QHBoxLayout();
    QPushButton* add = button("Add", "Add a layer on top", this);
    QPushButton* del = button("Delete", "Delete the selected layer", this);
    QPushButton* up = button("Up", "Move the layer down the stacking order (towards the bottom)", this);
    QPushButton* down = button("Down", "Move the layer up the stacking order (towards the top)", this);
    for (QPushButton* b : {add, del, up, down}) {
        row1->addWidget(b);
    }
    outer->addLayout(row1);
    auto* row2 = new QHBoxLayout();
    QPushButton* colour = button("Colour...", "Choose the colour of the selected layer", this);
    QPushButton* moveHere = button("Move selection here", "Move the selected objects to this layer", this);
    row2->addWidget(colour);
    row2->addWidget(moveHere);
    outer->addLayout(row2);

    connect(add, &QPushButton::clicked, this, [this]() { addLayer(); });
    connect(del, &QPushButton::clicked, this, [this]() { askDelete(); });
    connect(up, &QPushButton::clicked, this, [this]() { moveLayer(currentRow(), currentRow() - 1); });
    connect(down, &QPushButton::clicked, this, [this]() { moveLayer(currentRow(), currentRow() + 1); });
    connect(colour, &QPushButton::clicked, this, [this]() { askColour(); });
    connect(moveHere, &QPushButton::clicked, this, [this]() { moveSelectionHere(currentRow()); });
    connect(tree_, &QTreeWidget::itemChanged, this, [this](QTreeWidgetItem* item, int column) { itemEdited(item, column); });
    connect(tree_, &QTreeWidget::currentItemChanged, this, [this](QTreeWidgetItem* item, QTreeWidgetItem*) {
        if (!refreshing_ && item != nullptr && onPick) {
            onPick(tree_->indexOfTopLevelItem(item));
        }
    });
}

int LayersPanel::rowCount() const {
    return tree_->topLevelItemCount();
}

int LayersPanel::currentRow() const {
    return tree_->currentItem() != nullptr ? tree_->indexOfTopLevelItem(tree_->currentItem()) : -1;
}

void LayersPanel::setCurrentRow(int row) {
    if (row >= 0 && row < rowCount()) {
        tree_->setCurrentItem(tree_->topLevelItem(row));
    }
}

document::LayerId LayersPanel::idAt(int row) const {
    const document::Document* d = doc();
    if (d == nullptr || row < 0 || static_cast<std::size_t>(row) >= d->layers().size()) {
        return {};
    }
    return d->layers()[static_cast<std::size_t>(row)].id;
}

void LayersPanel::refresh() {
    // Updated in place (rows added or removed only at the end): a refresh can run from inside the
    // itemChanged signal of an edit in this very tree, so no item may be destroyed under it.
    const document::Document* d = doc();
    refreshing_ = true;
    const int want = d != nullptr ? static_cast<int>(d->layers().size()) : 0;
    while (tree_->topLevelItemCount() > want) {
        delete tree_->takeTopLevelItem(tree_->topLevelItemCount() - 1);
    }
    while (tree_->topLevelItemCount() < want) {
        auto* item = new QTreeWidgetItem(tree_);
        item->setFlags(item->flags() | Qt::ItemIsEditable | Qt::ItemIsUserCheckable);
        item->setToolTip(kColColour, QStringLiteral("Use the Colour button to change it"));
    }
    if (d != nullptr) {
        int active = 0;
        for (int i = 0; i < want; ++i) {
            const document::Layer& layer = d->layers()[static_cast<std::size_t>(i)];
            QTreeWidgetItem* item = tree_->topLevelItem(i);
            item->setText(kColName, qs(layer.name));
            item->setCheckState(kColVisible, layer.visible ? Qt::Checked : Qt::Unchecked);
            item->setCheckState(kColLocked, layer.locked ? Qt::Checked : Qt::Unchecked);
            item->setBackground(kColColour, QBrush(toQColor(layer.colour)));
            if (layer.id == d->activeLayer()) {
                active = i;
            }
        }
        if (want > 0 && currentRow() != active) {
            tree_->setCurrentItem(tree_->topLevelItem(active));
        }
    }
    refreshing_ = false;
}

void LayersPanel::itemEdited(QTreeWidgetItem* item, int column) {
    if (refreshing_ || item == nullptr) {
        return;
    }
    const int row = tree_->indexOfTopLevelItem(item);
    if (column == kColName) {
        if (!renameLayer(row, item->text(kColName).toStdString())) {
            refresh();  // put the old name back
        }
    } else if (column == kColVisible) {
        setVisible(row, item->checkState(kColVisible) == Qt::Checked);
    } else if (column == kColLocked) {
        setLocked(row, item->checkState(kColLocked) == Qt::Checked);
    }
}

bool LayersPanel::addLayer(const std::string& name) {
    document::Document* d = doc();
    return d != nullptr && finish(viewmodel::addLayer(*d, name));
}

bool LayersPanel::renameLayer(int row, const std::string& name) {
    document::Document* d = doc();
    return d != nullptr && finish(viewmodel::renameLayer(*d, idAt(row), name));
}

bool LayersPanel::setVisible(int row, bool visible) {
    document::Document* d = doc();
    return d != nullptr && finish(viewmodel::setLayerVisible(*d, idAt(row), visible));
}

bool LayersPanel::setLocked(int row, bool locked) {
    document::Document* d = doc();
    return d != nullptr && finish(viewmodel::setLayerLocked(*d, idAt(row), locked));
}

bool LayersPanel::setColour(int row, document::Colour colour) {
    document::Document* d = doc();
    return d != nullptr && finish(viewmodel::setLayerColour(*d, idAt(row), colour));
}

bool LayersPanel::deleteLayer(int row, bool deleteContents) {
    document::Document* d = doc();
    return d != nullptr && finish(viewmodel::deleteLayer(*d, idAt(row), deleteContents));
}

bool LayersPanel::moveLayer(int row, int newRow) {
    document::Document* d = doc();
    if (d == nullptr || row < 0 || newRow < 0) {
        return false;
    }
    return finish(viewmodel::moveLayer(*d, idAt(row), static_cast<std::size_t>(newRow)));
}

bool LayersPanel::moveSelectionHere(int row) {
    document::Document* d = doc();
    return d != nullptr && finish(viewmodel::moveSelectionToLayer(*d, idAt(row)));
}

void LayersPanel::askDelete() {
    const document::Document* d = doc();
    const int row = currentRow();
    const document::LayerId id = idAt(row);
    if (d == nullptr || id.isNull()) {
        return;
    }
    bool deleteContents = false;
    if (!d->layerIsEmpty(id)) {
        QMessageBox box(QMessageBox::Question, QStringLiteral("Delete layer"),
                        QStringLiteral("The layer \"%1\" holds objects.").arg(qs(d->layers()[static_cast<std::size_t>(row)].name)),
                        QMessageBox::NoButton, this);
        QPushButton* move = box.addButton(QStringLiteral("Move objects to the first layer"), QMessageBox::AcceptRole);
        QPushButton* erase = box.addButton(QStringLiteral("Delete the objects too"), QMessageBox::DestructiveRole);
        box.addButton(QMessageBox::Cancel);
        box.exec();
        if (box.clickedButton() == erase) {
            deleteContents = true;
        } else if (box.clickedButton() != move) {
            return;
        }
    }
    deleteLayer(row, deleteContents);  // `d` may be stale after the dialog's event loop: re-fetched inside
}

void LayersPanel::askColour() {
    const document::Document* d = doc();
    const int row = currentRow();
    if (d == nullptr || idAt(row).isNull()) {
        return;
    }
    const QColor initial = toQColor(d->layers()[static_cast<std::size_t>(row)].colour);
    const document::LayerId id = idAt(row);
    const QColor picked = QColorDialog::getColor(initial, this, QStringLiteral("Layer colour"));
    if (!picked.isValid()) {
        return;
    }
    // The dialog ran an event loop: find the layer again by id before editing.
    const document::Document* after = doc();
    if (after == nullptr || !after->findLayer(id)) {
        return;
    }
    const std::optional<std::size_t> index = after->layerIndex(id);
    setColour(static_cast<int>(*index),
              document::Colour{static_cast<std::uint8_t>(picked.red()), static_cast<std::uint8_t>(picked.green()),
                               static_cast<std::uint8_t>(picked.blue())});
}

// --- Sheets -----------------------------------------------------------------------------------

SheetsPanel::SheetsPanel(DocumentGetter getter, QWidget* parent) : PanelBase(std::move(getter), parent) {
    auto* outer = new QVBoxLayout(this);
    list_ = new QListWidget(this);
    list_->setToolTip(QStringLiteral("Double-click a name to rename the sheet."));
    outer->addWidget(list_, 1);
    auto* row = new QHBoxLayout();
    QPushButton* add = button("Add", "Add a sheet with the same material block", this);
    QPushButton* del = button("Delete", "Delete the selected sheet and the objects on it", this);
    row->addWidget(add);
    row->addWidget(del);
    outer->addLayout(row);

    auto* form = new QFormLayout();
    width_ = new QLineEdit(this);
    height_ = new QLineEdit(this);
    thickness_ = new QLineEdit(this);
    for (QLineEdit* e : {width_, height_, thickness_}) {
        attachVariableMenu(e);
    }
    zZero_ = new QComboBox(this);
    zZero_->addItems({QStringLiteral("Material surface"), QStringLiteral("Machine bed"), QStringLiteral("Cylinder axis")});
    datum_ = new QComboBox(this);
    datum_->addItems({QStringLiteral("Bottom left"), QStringLiteral("Bottom right"), QStringLiteral("Top left"),
                      QStringLiteral("Top right"), QStringLiteral("Centre")});
    form->addRow(QStringLiteral("Material width"), width_);
    form->addRow(QStringLiteral("Material height"), height_);
    form->addRow(QStringLiteral("Thickness"), thickness_);
    form->addRow(QStringLiteral("Z zero"), zZero_);
    form->addRow(QStringLiteral("XY datum"), datum_);
    outer->addLayout(form);

    connect(add, &QPushButton::clicked, this, [this]() { addSheet(); });
    connect(del, &QPushButton::clicked, this, [this]() { askDelete(); });
    for (QLineEdit* edit : {width_, height_, thickness_}) {
        connect(edit, &QLineEdit::editingFinished, this, [this]() { applyForm(); });
    }
    connect(zZero_, &QComboBox::activated, this, [this](int) { applyForm(); });
    connect(datum_, &QComboBox::activated, this, [this](int) { applyForm(); });
    connect(list_, &QListWidget::currentRowChanged, this, [this](int r) {
        if (refreshing_) {
            return;
        }
        if (onPick) {
            onPick(r);
        }
        fillForm();
    });
    list_->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::EditKeyPressed);
    connect(list_, &QListWidget::itemChanged, this, [this](QListWidgetItem* item) {
        if (refreshing_ || item == nullptr) {
            return;
        }
        if (!renameSheet(list_->row(item), item->text().toStdString())) {
            refresh();
        }
    });
}

int SheetsPanel::rowCount() const {
    return list_->count();
}

int SheetsPanel::currentRow() const {
    return list_->currentRow();
}

void SheetsPanel::setCurrentRow(int row) {
    if (row >= 0 && row < rowCount()) {
        list_->setCurrentRow(row);
    }
}

document::SheetId SheetsPanel::idAt(int row) const {
    const document::Document* d = doc();
    if (d == nullptr || row < 0 || static_cast<std::size_t>(row) >= d->sheets().size()) {
        return {};
    }
    return d->sheets()[static_cast<std::size_t>(row)].id;
}

void SheetsPanel::refresh() {
    // In place, like `LayersPanel::refresh` (it can run from inside this list's own signals).
    const document::Document* d = doc();
    refreshing_ = true;
    const int want = d != nullptr ? static_cast<int>(d->sheets().size()) : 0;
    while (list_->count() > want) {
        delete list_->takeItem(list_->count() - 1);
    }
    while (list_->count() < want) {
        auto* item = new QListWidgetItem(list_);
        item->setFlags(item->flags() | Qt::ItemIsEditable);
    }
    if (d != nullptr) {
        int active = 0;
        for (int i = 0; i < want; ++i) {
            const document::Sheet& sheet = d->sheets()[static_cast<std::size_t>(i)];
            list_->item(i)->setText(qs(sheet.name));
            if (sheet.id == d->activeSheet()) {
                active = i;
            }
        }
        if (want > 0 && list_->currentRow() != active) {
            list_->setCurrentRow(active);
        }
    }
    refreshing_ = false;
    fillForm();
}

void SheetsPanel::fillForm() {
    const document::Document* d = doc();
    const int row = list_->currentRow();
    const bool have = d != nullptr && !idAt(row).isNull();
    for (QWidget* w : std::initializer_list<QWidget*>{width_, height_, thickness_, zZero_, datum_}) {
        w->setEnabled(have);
    }
    if (!have) {
        return;
    }
    const document::Sheet& sheet = d->sheets()[static_cast<std::size_t>(row)];
    const document::Units u = d->job().displayUnits;
    const QString suffix = QStringLiteral(" ") + QString::fromUtf8(document::unitSuffix(u).data());
    const bool wasRefreshing = refreshing_;
    refreshing_ = true;
    width_->setText(qs(viewmodel::formatLength(sheet.sizeMm.x, u)) + suffix);
    height_->setText(qs(viewmodel::formatLength(sheet.sizeMm.y, u)) + suffix);
    thickness_->setText(qs(viewmodel::formatLength(sheet.sizeMm.z, u)) + suffix);
    zZero_->setCurrentIndex(static_cast<int>(sheet.zZero));
    datum_->setCurrentIndex(static_cast<int>(sheet.datum));
    refreshing_ = wasRefreshing;
}

void SheetsPanel::applyForm() {
    if (refreshing_) {
        return;
    }
    setMaterial(list_->currentRow(), width_->text().toStdString(), height_->text().toStdString(),
                thickness_->text().toStdString(), zZero_->currentIndex(), datum_->currentIndex());
}

bool SheetsPanel::addSheet(const std::string& name) {
    document::Document* d = doc();
    return d != nullptr && finish(viewmodel::addSheet(*d, name));
}

bool SheetsPanel::renameSheet(int row, const std::string& name) {
    document::Document* d = doc();
    return d != nullptr && finish(viewmodel::renameSheet(*d, idAt(row), name));
}

bool SheetsPanel::deleteSheet(int row) {
    document::Document* d = doc();
    return d != nullptr && finish(viewmodel::deleteSheet(*d, idAt(row)));
}

bool SheetsPanel::setMaterial(int row, const std::string& width, const std::string& height,
                              const std::string& thickness, int zZero, int datum) {
    document::Document* d = doc();
    if (d == nullptr) {
        return false;
    }
    const document::Units units = d->job().displayUnits;
    viewmodel::SheetMaterialEdit edit;
    auto parse = [&](const std::string& text, std::optional<double>* out, const char* what) {
        if (text.empty()) {
            return true;
        }
        const viewmodel::ParsedNumber n = viewmodel::parseNumber(text, viewmodel::FieldKind::Length, units);
        if (!n.ok) {
            if (onMessage) {
                onMessage(std::string("Material ") + what + ": " + n.error);
            }
            return false;
        }
        *out = n.value;
        return true;
    };
    if (!parse(width, &edit.widthMm, "width") || !parse(height, &edit.heightMm, "height") ||
        !parse(thickness, &edit.thicknessMm, "thickness")) {
        fillForm();
        return false;
    }
    if (zZero >= 0 && zZero <= static_cast<int>(document::ZZero::CylinderAxis)) {
        edit.zZero = static_cast<document::ZZero>(zZero);
    }
    if (datum >= 0 && datum <= static_cast<int>(document::Datum::Centre)) {
        edit.datum = static_cast<document::Datum>(datum);
    }
    const bool ok = finish(viewmodel::setSheetMaterial(*d, idAt(row), edit));
    if (!ok) {
        fillForm();
    }
    return ok;
}

void SheetsPanel::askDelete() {
    const document::Document* d = doc();
    const int row = list_->currentRow();
    const document::SheetId id = idAt(row);
    if (d == nullptr || id.isNull()) {
        return;
    }
    if (!d->objectsOnSheet(id).empty()) {
        const QMessageBox::StandardButton answer = QMessageBox::question(
            this, QStringLiteral("Delete sheet"),
            QStringLiteral("The sheet \"%1\" holds objects. Delete it together with them?")
                .arg(qs(d->sheets()[static_cast<std::size_t>(row)].name)),
            QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel);
        if (answer != QMessageBox::Yes) {
            return;
        }
    }
    const document::Document* after = doc();  // the dialog ran an event loop
    if (after == nullptr || after->findSheet(id) == nullptr) {
        return;
    }
    deleteSheet(static_cast<int>(*after->sheetIndex(id)));
}

// --- History ----------------------------------------------------------------------------------

HistoryPanel::HistoryPanel(DocumentGetter getter, QWidget* parent) : PanelBase(std::move(getter), parent) {
    auto* outer = new QVBoxLayout(this);
    list_ = new QListWidget(this);
    list_->setToolTip(QStringLiteral("Click a step to return to the state after it. Steps in grey were undone."));
    outer->addWidget(list_, 1);
    connect(list_, &QListWidget::itemClicked, this, [this](QListWidgetItem* item) {
        if (!refreshing_ && item != nullptr) {
            jumpTo(static_cast<std::size_t>(list_->row(item)));
        }
    });
}

int HistoryPanel::rowCount() const {
    return list_->count();
}

int HistoryPanel::currentRow() const {
    return list_->currentRow();
}

void HistoryPanel::refresh() {
    // In place: a refresh runs from inside this list's own itemClicked signal (a jump).
    const document::Document* d = doc();
    refreshing_ = true;
    const std::vector<viewmodel::HistoryEntry> entries =
        d != nullptr ? viewmodel::historyEntries(*d) : std::vector<viewmodel::HistoryEntry>{};
    const int want = d != nullptr ? static_cast<int>(entries.size()) + 1 : 0;
    while (list_->count() > want) {
        delete list_->takeItem(list_->count() - 1);
    }
    while (list_->count() < want) {
        new QListWidgetItem(list_);
    }
    if (d != nullptr) {
        list_->item(0)->setText(QStringLiteral("(start)"));
        for (std::size_t i = 0; i < entries.size(); ++i) {
            QListWidgetItem* item = list_->item(static_cast<int>(i) + 1);
            item->setText(qs(entries[i].text));
            item->setForeground(QBrush(entries[i].applied ? list_->palette().text().color() : QColor(Qt::gray)));
        }
        list_->setCurrentRow(static_cast<int>(d->undoStack().index()));
    }
    refreshing_ = false;
}

bool HistoryPanel::jumpTo(std::size_t applied) {
    document::Document* d = doc();
    if (d == nullptr) {
        return false;
    }
    if (historyBlocked && historyBlocked()) {
        if (onMessage) {
            onMessage("The history cannot change while a toolpath is being calculated.");
        }
        refresh();
        return false;
    }
    return finish(viewmodel::jumpToHistory(*d, applied));
}

}  // namespace carvetto::app
