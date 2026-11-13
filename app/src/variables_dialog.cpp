#include "variables_dialog.hpp"

#include <QFileDialog>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>
#include <utility>

namespace carvetto::app {

namespace {

QString toQ(const std::string& s) {
    return QString::fromUtf8(s.data(), static_cast<qsizetype>(s.size()));
}

}  // namespace

VariablesDialog::VariablesDialog(viewmodel::VariablesModel& model, QWidget* parent)
    : QDialog(parent), model_(model) {
    setObjectName(QStringLiteral("VariablesDialog"));
    setWindowTitle(QStringLiteral("Document Variables"));
    auto* outer = new QVBoxLayout(this);
    outer->addWidget(new QLabel(
        QStringLiteral("Use a variable in any numeric box as {Name}, after an = (for example =W/2-{Margin})."), this));

    table_ = new QTableWidget(0, 3, this);
    table_->setObjectName(QStringLiteral("variablesTable"));
    table_->setHorizontalHeaderLabels({QStringLiteral("Name"), QStringLiteral("Value"), QStringLiteral("Used by")});
    table_->horizontalHeader()->setStretchLastSection(true);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setSelectionMode(QAbstractItemView::SingleSelection);
    table_->verticalHeader()->setVisible(false);
    outer->addWidget(table_, 1);

    auto* entry = new QHBoxLayout();
    nameEdit_ = new QLineEdit(this);
    nameEdit_->setObjectName(QStringLiteral("variableName"));
    nameEdit_->setPlaceholderText(QStringLiteral("Name"));
    valueEdit_ = new QLineEdit(this);
    valueEdit_->setObjectName(QStringLiteral("variableValue"));
    valueEdit_->setPlaceholderText(QStringLiteral("Value (a number, or =calculation)"));
    set_ = new QPushButton(QStringLiteral("Set"), this);
    set_->setObjectName(QStringLiteral("variableSet"));
    set_->setToolTip(QStringLiteral("Create the variable, or change its value"));
    entry->addWidget(nameEdit_, 1);
    entry->addWidget(valueEdit_, 1);
    entry->addWidget(set_);
    outer->addLayout(entry);

    auto* buttons = new QHBoxLayout();
    delete_ = new QPushButton(QStringLiteral("Delete"), this);
    delete_->setObjectName(QStringLiteral("variableDelete"));
    rename_ = new QPushButton(QStringLiteral("Rename..."), this);
    rename_->setObjectName(QStringLiteral("variableRename"));
    auto* import = new QPushButton(QStringLiteral("Import..."), this);
    import->setObjectName(QStringLiteral("variableImport"));
    auto* exportButton = new QPushButton(QStringLiteral("Export..."), this);
    exportButton->setObjectName(QStringLiteral("variableExport"));
    auto* close = new QPushButton(QStringLiteral("Close"), this);
    close->setDefault(false);
    close->setAutoDefault(false);
    buttons->addWidget(delete_);
    buttons->addWidget(rename_);
    buttons->addSpacing(12);
    buttons->addWidget(import);
    buttons->addWidget(exportButton);
    buttons->addStretch(1);
    buttons->addWidget(close);
    outer->addLayout(buttons);

    status_ = new QLabel(this);
    status_->setObjectName(QStringLiteral("variablesStatus"));
    status_->setWordWrap(true);
    outer->addWidget(status_);

    connect(set_, &QPushButton::clicked, this, [this]() {
        report(model_.set(nameEdit_->text().toStdString(), valueEdit_->text().toStdString()));
    });
    connect(valueEdit_, &QLineEdit::returnPressed, set_, &QPushButton::click);
    connect(nameEdit_, &QLineEdit::returnPressed, set_, &QPushButton::click);
    connect(delete_, &QPushButton::clicked, this, [this]() { deleteSelected(); });
    connect(rename_, &QPushButton::clicked, this, [this]() { renameSelected(); });
    connect(import, &QPushButton::clicked, this, [this]() { importDialog(); });
    connect(exportButton, &QPushButton::clicked, this, [this]() { exportDialog(); });
    connect(close, &QPushButton::clicked, this, &QDialog::accept);
    connect(table_, &QTableWidget::itemSelectionChanged, this, [this]() {
        const QString name = selectedName();
        if (!name.isEmpty() && !loading_) {
            nameEdit_->setText(name);
            const auto rows = model_.rows();
            for (const auto& r : rows) {
                if (toQ(r.name) == name) {
                    valueEdit_->setText(toQ(r.valueText));
                }
            }
        }
    });
    connect(table_, &QTableWidget::itemChanged, this, [this](QTableWidgetItem* item) {
        if (loading_ || item->column() != 1) {
            return;
        }
        const QTableWidgetItem* nameItem = table_->item(item->row(), 0);
        if (nameItem != nullptr) {
            report(model_.set(nameItem->text().toStdString(), item->text().toStdString()));
        }
    });
    resize(560, 420);
    reload();
}

void VariablesDialog::reload() {
    loading_ = true;
    const auto rows = model_.rows();
    table_->setRowCount(static_cast<int>(rows.size()));
    for (int i = 0; i < static_cast<int>(rows.size()); ++i) {
        const viewmodel::VariableRow& r = rows[static_cast<std::size_t>(i)];
        auto* name = new QTableWidgetItem(toQ(r.name));
        name->setFlags(name->flags() & ~Qt::ItemIsEditable);
        auto* value = new QTableWidgetItem(toQ(r.valueText));
        auto* used = new QTableWidgetItem(r.references == 0 ? QString() : QString::number(r.references));
        used->setFlags(used->flags() & ~Qt::ItemIsEditable);
        table_->setItem(i, 0, name);
        table_->setItem(i, 1, value);
        table_->setItem(i, 2, used);
    }
    loading_ = false;
}

void VariablesDialog::report(const viewmodel::VariableEdit& result) {
    lastOk_ = result.ok;
    status_->setText(toQ(result.message));
    status_->setStyleSheet(result.ok ? QString() : QStringLiteral("color: #b00020;"));
    reload();
    if (result.ok && onEdited) {
        onEdited();
    }
    if (result.ok && result.recalculationPending && onRecalculate) {
        onRecalculate();
    }
}

QString VariablesDialog::message() const {
    return status_->text();
}

int VariablesDialog::rowCount() const {
    return table_->rowCount();
}

QString VariablesDialog::selectedName() const {
    const int row = table_->currentRow();
    if (row < 0 || table_->item(row, 0) == nullptr) {
        return {};
    }
    return table_->item(row, 0)->text();
}

void VariablesDialog::deleteSelected() {
    const QString name = selectedName();
    if (name.isEmpty()) {
        viewmodel::VariableEdit none;
        none.message = "select a variable first";
        report(none);
        return;
    }
    report(model_.remove(name.toStdString()));
}

void VariablesDialog::renameSelected() {
    const QString name = selectedName();
    if (name.isEmpty()) {
        viewmodel::VariableEdit none;
        none.message = "select a variable first";
        report(none);
        return;
    }
    bool ok = false;
    const QString next = QInputDialog::getText(this, QStringLiteral("Rename Variable"), QStringLiteral("New name"),
                                               QLineEdit::Normal, name, &ok);
    if (ok) {
        report(model_.rename(name.toStdString(), next.toStdString()));
    }
}

void VariablesDialog::importDialog() {
    const QString file = QFileDialog::getOpenFileName(this, QStringLiteral("Import Variables"), QString(),
                                                      QStringLiteral("Text files (*.txt);;All files (*)"));
    if (!file.isEmpty()) {
        importFrom(std::filesystem::path(file.toStdString()));
    }
}

void VariablesDialog::exportDialog() {
    const QString file = QFileDialog::getSaveFileName(this, QStringLiteral("Export Variables"), QString(),
                                                      QStringLiteral("Text files (*.txt);;All files (*)"));
    if (!file.isEmpty()) {
        exportTo(std::filesystem::path(file.toStdString()));
    }
}

bool VariablesDialog::setVariable(const std::string& name, const std::string& value) {
    nameEdit_->setText(toQ(name));
    valueEdit_->setText(toQ(value));
    set_->click();
    return lastOk_;
}

bool VariablesDialog::deleteVariable(const std::string& name) {
    for (int i = 0; i < table_->rowCount(); ++i) {
        if (table_->item(i, 0)->text() == toQ(name)) {
            table_->selectRow(i);
            table_->setCurrentCell(i, 0);
            delete_->click();
            return lastOk_;
        }
    }
    return false;
}

bool VariablesDialog::editValueCell(const std::string& name, const std::string& value) {
    for (int i = 0; i < table_->rowCount(); ++i) {
        if (table_->item(i, 0)->text() == toQ(name)) {
            table_->item(i, 1)->setText(toQ(value));  // emits itemChanged, as an edit does
            return lastOk_;
        }
    }
    return false;
}

bool VariablesDialog::importFrom(const std::filesystem::path& path) {
    report(model_.importFile(path));
    return lastOk_;
}

bool VariablesDialog::exportTo(const std::filesystem::path& path) {
    const viewmodel::VariableEdit r = model_.exportFile(path);
    lastOk_ = r.ok;
    status_->setText(toQ(r.message));
    status_->setStyleSheet(r.ok ? QString() : QStringLiteral("color: #b00020;"));
    return r.ok;
}

}  // namespace carvetto::app
