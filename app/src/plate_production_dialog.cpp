#include "plate_production_dialog.hpp"
#include "vm_text.hpp"

#include <QCheckBox>
#include <QCoreApplication>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QVariant>
#include <algorithm>

#include "carvetto/io/delimited.hpp"
#include "main_window.hpp"

namespace carvetto::app {

using viewmodel::DataMergeOptions;
using viewmodel::DataMergeResult;

namespace {

QDoubleSpinBox* lengthBox(QWidget* parent, document::Units units, double maxMm) {
    auto* box = new QDoubleSpinBox(parent);
    box->setDecimals(4);
    box->setRange(0.0, document::toDisplay(maxMm, units));
    const std::string_view suffix = document::unitSuffix(units);
    box->setSuffix(QStringLiteral(" ") + QString::fromUtf8(suffix.data(), static_cast<int>(suffix.size())));
    return box;
}

constexpr double kCounterLimit = 1.0e12;

}  // namespace

PlateProductionDialog::PlateProductionDialog(const document::Document& doc, QWidget* parent)
    : QDialog(parent), doc_(doc), units_(doc.job().displayUnits) {
    setWindowTitle(QCoreApplication::translate("PlateProductionDialog", "Plate Production"));
    auto* root = new QVBoxLayout(this);

    root->addWidget(new QLabel(
        QCoreApplication::translate("PlateProductionDialog", "The selected objects are the template. One plate is made per data row; use !!NAME!! in "
                       "template text for a column value."),
        this));
    csv_ = new QPlainTextEdit(this);
    csv_->setPlaceholderText(QCoreApplication::translate("PlateProductionDialog", "Paste data here or import a file (one plate per row)"));
    csv_->setMinimumHeight(100);
    root->addWidget(csv_);
    auto* importRow = new QHBoxLayout();
    auto* importButton = new QPushButton(QCoreApplication::translate("PlateProductionDialog", "Import from file..."), this);
    importRow->addWidget(importButton);
    separator_ = new QComboBox(this);
    separator_->addItem(QCoreApplication::translate("PlateProductionDialog", "Comma"), QVariant(QLatin1Char(',')));
    separator_->addItem(QCoreApplication::translate("PlateProductionDialog", "Tab"), QVariant(QLatin1Char('\t')));
    separator_->addItem(QCoreApplication::translate("PlateProductionDialog", "Semicolon"), QVariant(QLatin1Char(';')));
    separator_->addItem(QCoreApplication::translate("PlateProductionDialog", "Space"), QVariant(QLatin1Char(' ')));
    importRow->addWidget(new QLabel(QCoreApplication::translate("PlateProductionDialog", "Separator"), this));
    importRow->addWidget(separator_);
    firstRowNames_ = new QCheckBox(QCoreApplication::translate("PlateProductionDialog", "First row is column names"), this);
    firstRowNames_->setChecked(true);
    importRow->addWidget(firstRowNames_);
    importRow->addStretch(1);
    root->addLayout(importRow);

    assignmentForm_ = new QFormLayout();
    root->addWidget(new QLabel(QCoreApplication::translate("PlateProductionDialog", "Variable assignment"), this));
    root->addLayout(assignmentForm_);

    auto* counterForm = new QFormLayout();
    counterEnabled_ = new QCheckBox(QCoreApplication::translate("PlateProductionDialog", "Serial number counter"), this);
    counterName_ = new QLineEdit(QCoreApplication::translate("PlateProductionDialog", "SERIAL"), this);
    counterStart_ = new QDoubleSpinBox(this);
    counterStart_->setDecimals(0);
    counterStart_->setRange(-kCounterLimit, kCounterLimit);
    counterStart_->setValue(1.0);
    counterStep_ = new QDoubleSpinBox(this);
    counterStep_->setDecimals(0);
    counterStep_->setRange(-kCounterLimit, kCounterLimit);
    counterStep_->setValue(1.0);
    counterWidth_ = new QSpinBox(this);
    counterWidth_->setRange(1, viewmodel::kMaxCounterWidth);
    counterWidth_->setValue(3);
    counterPrefix_ = new QLineEdit(this);
    counterSuffix_ = new QLineEdit(this);
    counterForm->addRow(counterEnabled_);
    counterForm->addRow(QCoreApplication::translate("PlateProductionDialog", "Placeholder name"), counterName_);
    counterForm->addRow(QCoreApplication::translate("PlateProductionDialog", "Start"), counterStart_);
    counterForm->addRow(QCoreApplication::translate("PlateProductionDialog", "Step"), counterStep_);
    counterForm->addRow(QCoreApplication::translate("PlateProductionDialog", "Digits"), counterWidth_);
    counterForm->addRow(QCoreApplication::translate("PlateProductionDialog", "Prefix"), counterPrefix_);
    counterForm->addRow(QCoreApplication::translate("PlateProductionDialog", "Suffix"), counterSuffix_);
    root->addLayout(counterForm);

    auto* layoutForm = new QFormLayout();
    layout_ = new QComboBox(this);
    layout_->addItem(QCoreApplication::translate("PlateProductionDialog", "Fill sheets (overflow to new sheets)"),
                     static_cast<int>(viewmodel::MergeLayout::Grid));
    layout_->addItem(QCoreApplication::translate("PlateProductionDialog", "One plate per sheet"), static_cast<int>(viewmodel::MergeLayout::OnePerSheet));
    marginLeft_ = lengthBox(this, units_, viewmodel::kMaxMergeLengthMm);
    marginRight_ = lengthBox(this, units_, viewmodel::kMaxMergeLengthMm);
    marginTop_ = lengthBox(this, units_, viewmodel::kMaxMergeLengthMm);
    marginBottom_ = lengthBox(this, units_, viewmodel::kMaxMergeLengthMm);
    spacing_ = lengthBox(this, units_, viewmodel::kMaxMergeLengthMm);
    const DataMergeOptions defaults;
    marginLeft_->setValue(document::toDisplay(defaults.marginLeftMm, units_));
    marginRight_->setValue(document::toDisplay(defaults.marginRightMm, units_));
    marginTop_->setValue(document::toDisplay(defaults.marginTopMm, units_));
    marginBottom_->setValue(document::toDisplay(defaults.marginBottomMm, units_));
    spacing_->setValue(document::toDisplay(defaults.spacingMm, units_));
    newLayer_ = new QCheckBox(QCoreApplication::translate("PlateProductionDialog", "Put the plates on a new layer"), this);
    newLayer_->setChecked(defaults.newLayer);
    removeTemplate_ = new QCheckBox(QCoreApplication::translate("PlateProductionDialog", "Delete the template afterwards"), this);
    skipBadRows_ = new QCheckBox(QCoreApplication::translate("PlateProductionDialog", "Skip rows with problems"), this);
    layoutForm->addRow(QCoreApplication::translate("PlateProductionDialog", "Layout"), layout_);
    layoutForm->addRow(QCoreApplication::translate("PlateProductionDialog", "Left margin"), marginLeft_);
    layoutForm->addRow(QCoreApplication::translate("PlateProductionDialog", "Right margin"), marginRight_);
    layoutForm->addRow(QCoreApplication::translate("PlateProductionDialog", "Top margin"), marginTop_);
    layoutForm->addRow(QCoreApplication::translate("PlateProductionDialog", "Bottom margin"), marginBottom_);
    layoutForm->addRow(QCoreApplication::translate("PlateProductionDialog", "Spacing"), spacing_);
    layoutForm->addRow(newLayer_);
    layoutForm->addRow(removeTemplate_);
    layoutForm->addRow(skipBadRows_);
    root->addLayout(layoutForm);

    summary_ = new QLabel(this);
    root->addWidget(summary_);
    problems_ = new QPlainTextEdit(this);
    problems_->setReadOnly(true);
    problems_->setMaximumHeight(90);
    root->addWidget(problems_);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    auto* previewButton = buttons->addButton(QCoreApplication::translate("PlateProductionDialog", "Preview"), QDialogButtonBox::ActionRole);
    connect(buttons, &QDialogButtonBox::accepted, this, [this]() { accept(); });
    connect(buttons, &QDialogButtonBox::rejected, this, [this]() { reject(); });
    connect(previewButton, &QPushButton::clicked, this, [this]() { refreshPreview(); });
    connect(importButton, &QPushButton::clicked, this, [this]() { importFile(); });
    root->addWidget(buttons);

    placeholders_ = viewmodel::templatePlaceholders(doc_);
    rebuildAssignments();
    refreshPreview();
}

QString PlateProductionDialog::csvText() const { return csv_->toPlainText(); }

void PlateProductionDialog::setCsvText(const QString& text) {
    csv_->setPlainText(text);
    rebuildAssignments();
}

void PlateProductionDialog::setSeparator(char separator) {
    const int index = separator_->findData(QVariant(QLatin1Char(separator)));
    if (index >= 0) {
        separator_->setCurrentIndex(index);
    }
}

void PlateProductionDialog::setSkipBadRows(bool skip) { skipBadRows_->setChecked(skip); }

QString PlateProductionDialog::summary() const { return summary_->text(); }
QString PlateProductionDialog::problems() const { return problems_->toPlainText(); }

void PlateProductionDialog::importFile() {
    const QString path = QFileDialog::getOpenFileName(this, QCoreApplication::translate("PlateProductionDialog", "Data file"), QString(),
                                                      QCoreApplication::translate("PlateProductionDialog", "Data files (*.csv *.txt *.tsv);;All files (*)"));
    if (path.isEmpty()) {
        return;
    }
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        QMessageBox::warning(this, windowTitle(), QCoreApplication::translate("PlateProductionDialog", "The file could not be opened."));
        return;
    }
    // One byte over the cap is enough for the parser to refuse an oversized file.
    const QByteArray bytes = file.read(static_cast<qint64>(io::kDelimitedMaxBytes) + 1);
    setCsvText(QString::fromUtf8(bytes));
    refreshPreview();
}

void PlateProductionDialog::rebuildAssignments() {
    const DataMergeOptions o = options();
    const std::vector<std::string> headers =
        o.firstRowNames ? viewmodel::csvHeaders(csv_->toPlainText().toStdString(), o.separator)
                        : std::vector<std::string>{};
    // Remember what the user chose.
    std::vector<viewmodel::VariableAssignment> previous;
    for (std::size_t i = 0; i < assignmentBoxes_.size(); ++i) {
        const int col = assignmentBoxes_[i]->currentData().toInt();
        if (col >= 0) {
            previous.push_back({placeholders_[i], col});
        }
    }
    for (QComboBox* box : assignmentBoxes_) {
        delete box;
    }
    for (QLabel* label : assignmentLabels_) {
        delete label;
    }
    assignmentBoxes_.clear();
    assignmentLabels_.clear();
    const std::vector<viewmodel::VariableAssignment> automatic = viewmodel::autoAssign(placeholders_, headers);
    io::DelimitedOptions csvOptions;
    csvOptions.separator = o.separator;
    const std::size_t columns = io::parseDelimited(csv_->toPlainText().toStdString(), csvOptions).columns;
    for (const std::string& name : placeholders_) {
        auto* box = new QComboBox(this);
        box->addItem(QCoreApplication::translate("PlateProductionDialog", "(none)"), -1);
        for (std::size_t c = 0; c < columns; ++c) {
            const QString header = c < headers.size() ? QString::fromStdString(headers[c]) : QString();
            box->addItem(QCoreApplication::translate("PlateProductionDialog", "Column %1 %2").arg(c + 1).arg(header).trimmed(), static_cast<int>(c));
        }
        int chosen = -1;
        for (const auto& a : previous) {
            if (a.name == name) {
                chosen = a.column;
            }
        }
        if (chosen < 0) {
            for (const auto& a : automatic) {
                if (a.name == name) {
                    chosen = a.column;
                }
            }
        }
        box->setCurrentIndex(box->findData(chosen));
        if (box->currentIndex() < 0) {
            box->setCurrentIndex(0);
        }
        auto* label = new QLabel(QStringLiteral("!!%1!!").arg(QString::fromStdString(name)), this);
        assignmentForm_->addRow(label, box);
        assignmentBoxes_.push_back(box);
        assignmentLabels_.push_back(label);
    }
}

DataMergeOptions PlateProductionDialog::options() const {
    DataMergeOptions o;
    o.separator = static_cast<char>(separator_->currentData().toChar().toLatin1());
    o.firstRowNames = firstRowNames_->isChecked();
    const std::string counterName = counterName_->text().toStdString();
    for (std::size_t i = 0; i < assignmentBoxes_.size(); ++i) {
        const int col = assignmentBoxes_[i]->currentData().toInt();
        const bool counted = counterEnabled_->isChecked() && placeholders_[i] == counterName;
        if (col >= 0 && !counted) {
            o.assignments.push_back({placeholders_[i], col});
        }
    }
    if (counterEnabled_->isChecked()) {
        viewmodel::CounterSpec c;
        c.name = counterName;
        c.start = static_cast<std::int64_t>(counterStart_->value());
        c.step = static_cast<std::int64_t>(counterStep_->value());
        c.width = counterWidth_->value();
        c.prefix = counterPrefix_->text().toStdString();
        c.suffix = counterSuffix_->text().toStdString();
        o.counters.push_back(std::move(c));
    }
    o.layout = static_cast<viewmodel::MergeLayout>(layout_->currentData().toInt());
    o.marginLeftMm = document::fromDisplay(marginLeft_->value(), units_);
    o.marginRightMm = document::fromDisplay(marginRight_->value(), units_);
    o.marginTopMm = document::fromDisplay(marginTop_->value(), units_);
    o.marginBottomMm = document::fromDisplay(marginBottom_->value(), units_);
    o.spacingMm = document::fromDisplay(spacing_->value(), units_);
    o.newLayer = newLayer_->isChecked();
    o.removeTemplate = removeTemplate_->isChecked();
    o.skipBadRows = skipBadRows_->isChecked();
    return o;
}

void PlateProductionDialog::refreshPreview() {
    rebuildAssignments();
    const DataMergeResult r = viewmodel::previewDataMerge(doc_, csv_->toPlainText().toStdString(), options());
    QStringList lines;
    for (const viewmodel::MergeDiagnostic& d : r.diagnostics) {
        lines << (d.line > 0 ? QCoreApplication::translate("PlateProductionDialog", "line %1: ").arg(d.line) : QString()) +
                     QString::fromStdString(d.message);
    }
    for (const viewmodel::MergeDiagnostic& d : r.warnings) {
        lines << QCoreApplication::translate("PlateProductionDialog", "line %1: note: ").arg(d.line) + QString::fromStdString(d.message);
    }
    problems_->setPlainText(lines.join(QLatin1Char('\n')));
    if (r.ok) {
        summary_->setText(QCoreApplication::translate("PlateProductionDialog", "%1 plate(s) on %2 sheet(s), %3 per sheet.")
                              .arg(r.plates)
                              .arg(r.sheetsUsed)
                              .arg(r.platesPerSheet));
    } else {
        summary_->setText(vmText(r.message));
    }
}

void PlateProductionDialog::accept() {
    const DataMergeResult r = viewmodel::previewDataMerge(doc_, csv_->toPlainText().toStdString(), options());
    refreshPreview();
    if (!r.ok) {
        QMessageBox::warning(this, windowTitle(), QString::fromStdString(r.message));
        return;
    }
    QDialog::accept();
}

DataMergeResult runPlateProduction(MainWindow& window, const std::string& csvText, const DataMergeOptions& options,
                                   bool showErrors) {
    DataMergeResult result = viewmodel::applyDataMerge(window.currentDocument(), csvText, options);
    if (result.ok) {
        window.documentChanged();
    } else if (showErrors) {
        QMessageBox::warning(&window, QCoreApplication::translate("PlateProductionDialog", "Plate Production"), QString::fromStdString(result.message));
    }
    return result;
}

void showPlateProduction(MainWindow& window) {
    PlateProductionDialog dialog(window.currentDocument(), &window);
    if (dialog.exec() == QDialog::Accepted) {
        runPlateProduction(window, dialog.csvText().toStdString(), dialog.options());
    }
}

}  // namespace carvetto::app
