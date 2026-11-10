#include "vector_ops_menu.hpp"

#include <QAction>
#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QMenu>
#include <QMessageBox>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>
#include <QVariant>
#include <string>

#include "carvetto/document/units.hpp"
#include "carvetto/viewmodel/vector_validator.hpp"
#include "main_window.hpp"

namespace carvetto::app {

using viewmodel::BooleanKind;
using viewmodel::OffsetDirection;
using viewmodel::OffsetOptions;
using viewmodel::VectorOpResult;

OffsetDialog::OffsetDialog(document::Units units, const OffsetOptions& initial, QWidget* parent)
    : QDialog(parent), units_(units) {
    setWindowTitle(QStringLiteral("Offset Vectors"));
    distance_ = new QDoubleSpinBox(this);
    distance_->setDecimals(4);
    distance_->setRange(0.0001, 100000.0);
    distance_->setSuffix(QStringLiteral(" ") + QString::fromUtf8(document::unitSuffix(units).data(),
                                                                  static_cast<int>(document::unitSuffix(units).size())));
    direction_ = new QComboBox(this);
    direction_->addItem(QStringLiteral("Outwards (open: right)"), static_cast<int>(OffsetDirection::Outwards));
    direction_->addItem(QStringLiteral("Inwards (open: left)"), static_cast<int>(OffsetDirection::Inwards));
    join_ = new QComboBox(this);
    join_->addItem(QStringLiteral("Round corners"), static_cast<int>(geometry::JoinType::Round));
    join_->addItem(QStringLiteral("Sharp corners"), static_cast<int>(geometry::JoinType::Sharp));
    join_->addItem(QStringLiteral("Square corners"), static_cast<int>(geometry::JoinType::Square));
    deleteOriginals_ = new QCheckBox(QStringLiteral("Delete original vectors"), this);

    auto* form = new QFormLayout;
    form->addRow(QStringLiteral("Distance"), distance_);
    form->addRow(QStringLiteral("Direction"), direction_);
    form->addRow(QStringLiteral("Corners"), join_);
    form->addRow(deleteOriginals_);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, [this]() { accept(); });
    connect(buttons, &QDialogButtonBox::rejected, this, [this]() { reject(); });
    auto* layout = new QVBoxLayout(this);
    layout->addLayout(form);
    layout->addWidget(buttons);
    setOptions(initial);
}

OffsetOptions OffsetDialog::options() const {
    OffsetOptions o;
    o.distanceMm = document::fromDisplay(distance_->value(), units_);
    o.direction = static_cast<OffsetDirection>(direction_->currentData().toInt());
    o.join = static_cast<geometry::JoinType>(join_->currentData().toInt());
    o.deleteOriginals = deleteOriginals_->isChecked();
    return o;
}

void OffsetDialog::setOptions(const OffsetOptions& o) {
    distance_->setValue(document::toDisplay(o.distanceMm, units_));
    direction_->setCurrentIndex(direction_->findData(static_cast<int>(o.direction)));
    join_->setCurrentIndex(join_->findData(static_cast<int>(o.join)));
    deleteOriginals_->setChecked(o.deleteOriginals);
}

void OffsetDialog::accept() {
    const std::string problem = viewmodel::validateOffsetOptions(options());
    if (!problem.empty()) {
        QMessageBox::warning(this, windowTitle(), QString::fromStdString(problem));
        return;
    }
    QDialog::accept();
}

VectorOpsMenu::VectorOpsMenu(MainWindow& window, QMenu* editMenu) : window_(window) {
    QMenu* menu = editMenu->addMenu(QStringLiteral("&Vectors"));
    weld_ = menu->addAction(QStringLiteral("&Weld"));
    subtract_ = menu->addAction(QStringLiteral("&Subtract (last selected from the others)"));
    intersect_ = menu->addAction(QStringLiteral("&Overlap (intersect with last selected)"));
    exclude_ = menu->addAction(QStringLiteral("&Exclude"));
    menu->addSeparator();
    offset_ = menu->addAction(QStringLiteral("&Offset..."));
    QObject::connect(weld_, &QAction::triggered, menu, [this]() { runBoolean(BooleanKind::Weld); });
    QObject::connect(subtract_, &QAction::triggered, menu, [this]() { runBoolean(BooleanKind::Subtract); });
    QObject::connect(intersect_, &QAction::triggered, menu, [this]() { runBoolean(BooleanKind::Intersect); });
    QObject::connect(exclude_, &QAction::triggered, menu, [this]() { runBoolean(BooleanKind::Exclude); });
    QObject::connect(offset_, &QAction::triggered, menu, [this]() {
        OffsetDialog dialog(window_.currentDocument().job().displayUnits, lastOffset_, &window_);
        if (dialog.exec() == QDialog::Accepted) {
            lastOffset_ = dialog.options();
            runOffset(lastOffset_);
        }
    });
    menu->addSeparator();
    QAction* validate = menu->addAction(QStringLiteral("&Validate Vectors..."));
    QAction* selectDuplicates = menu->addAction(QStringLiteral("Select All Duplicate Vectors"));
    QObject::connect(validate, &QAction::triggered, menu, [this]() { runValidator(); });
    QObject::connect(selectDuplicates, &QAction::triggered, menu, [this]() {
        document::Document& doc = window_.currentDocument();
        viewmodel::ValidatorOptions options;
        options.scope = viewmodel::defaultValidatorScope(doc);
        const std::size_t n = viewmodel::selectDuplicateVectors(doc, options);
        if (n > 0) {
            window_.documentChanged();
        } else {
            QMessageBox::information(&window_, QStringLiteral("Vectors"), QStringLiteral("No duplicate vectors found."));
        }
    });
    QObject::connect(menu, &QMenu::aboutToShow, menu, [this]() { refresh(); });
    for (QAction* action : {weld_, subtract_, intersect_, exclude_, offset_}) {
        action->setEnabled(false);  // until the menu opens (the window may not have a document yet)
    }
}

void VectorOpsMenu::refresh() {
    const document::Document& doc = window_.currentDocument();
    weld_->setEnabled(viewmodel::canWeld(doc));
    subtract_->setEnabled(viewmodel::canSubtract(doc));
    intersect_->setEnabled(viewmodel::canIntersect(doc));
    exclude_->setEnabled(viewmodel::canExclude(doc));
    offset_->setEnabled(viewmodel::canOffset(doc));
}

QAction* VectorOpsMenu::booleanAction(BooleanKind kind) const {
    switch (kind) {
        case BooleanKind::Weld:
            return weld_;
        case BooleanKind::Subtract:
            return subtract_;
        case BooleanKind::Intersect:
            return intersect_;
        case BooleanKind::Exclude:
            return exclude_;
    }
    return nullptr;
}

VectorOpResult VectorOpsMenu::runBoolean(BooleanKind kind, bool showErrors) {
    VectorOpResult result = viewmodel::applyBoolean(window_.currentDocument(), kind);
    report(result, showErrors);
    return result;
}

VectorOpResult VectorOpsMenu::runOffset(const OffsetOptions& options, bool showErrors) {
    VectorOpResult result = viewmodel::applyOffset(window_.currentDocument(), options);
    report(result, showErrors);
    return result;
}

void VectorOpsMenu::runValidator() {
    document::Document& doc = window_.currentDocument();
    viewmodel::ValidatorOptions options;
    options.scope = viewmodel::defaultValidatorScope(doc);
    QMessageBox box(&window_);
    box.setWindowTitle(QStringLiteral("Vector Validator"));
    auto* vcarve = new QCheckBox(QStringLiteral("V-carving mode (ignore crossings inside text)"), &box);
    vcarve->setChecked(lastVCarvingMode_);
    box.setCheckBox(vcarve);
    QPushButton* fix = box.addButton(QStringLiteral("Fix"), QMessageBox::AcceptRole);
    QPushButton* select = box.addButton(QStringLiteral("Select Offenders"), QMessageBox::ActionRole);
    QPushButton* search = box.addButton(QStringLiteral("Search"), QMessageBox::ActionRole);
    box.addButton(QMessageBox::Close);
    const QString scope = options.scope == viewmodel::ValidatorScope::Selection
                              ? QStringLiteral("Checking the selected vectors.")
                              : QStringLiteral("Checking all visible vectors on this sheet.");
    for (;;) {
        options.vCarvingMode = vcarve->isChecked();
        lastVCarvingMode_ = options.vCarvingMode;
        const viewmodel::ValidatorReport r = viewmodel::validateVectors(doc, options);
        box.setText(scope + QStringLiteral("\n\n") + QString::fromStdString(r.summary()));
        fix->setEnabled(r.report.fixableCount() > 0);
        select->setEnabled(r.report.total() > 0);
        box.exec();
        if (box.clickedButton() == fix) {
            const viewmodel::ValidatorFixResult f = viewmodel::fixVectors(doc, options);
            if (f.objectsChanged > 0 || f.objectsRemoved > 0) {
                window_.documentChanged();
            }
        } else if (box.clickedButton() == select) {
            if (viewmodel::selectOffenders(doc, r) > 0) {
                window_.documentChanged();
            }
            return;
        } else if (box.clickedButton() != search) {
            return;
        }
    }
}

void VectorOpsMenu::report(const VectorOpResult& result, bool showErrors) {
    if (result.ok) {
        window_.documentChanged();
    } else if (showErrors) {
        QMessageBox::warning(&window_, QStringLiteral("Vectors"), QString::fromStdString(result.message));
    }
}

}  // namespace carvetto::app
