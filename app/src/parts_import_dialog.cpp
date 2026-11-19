#include "parts_import_dialog.hpp"

#include <QCoreApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>
#include <algorithm>
#include <cstddef>
#include <utility>

#include "carvetto/io/parts_import.hpp"
#include "carvetto/viewmodel/file_session.hpp"

namespace carvetto::app {

namespace {

QString qs(const std::string& s) { return QString::fromStdString(s); }

constexpr int kMaxShownDiagnostics = 500;

}  // namespace

PartsImportDialog::PartsImportDialog(tooling::ToolDatabase db, QWidget* parent) : QDialog(parent), db_(std::move(db)) {
    setObjectName(QStringLiteral("partsImportDialog"));
    setWindowTitle(QCoreApplication::translate("PartsImportDialog", "Import Parts List"));
    resize(760, 720);
    auto* layout = new QVBoxLayout(this);

    const auto addSource = [&](const QString& title, const char* editName, const char* browseName, bool mapping) {
        auto* box = new QGroupBox(title, this);
        auto* v = new QVBoxLayout(box);
        auto* row = new QHBoxLayout;
        auto* openButton = new QPushButton(QCoreApplication::translate("PartsImportDialog", "Open file..."), box);
        openButton->setObjectName(QString::fromLatin1(browseName));
        connect(openButton, &QPushButton::clicked, this, [this, mapping]() { browse(mapping); });
        row->addWidget(openButton);
        row->addStretch(1);
        v->addLayout(row);
        auto* edit = new QPlainTextEdit(box);
        edit->setObjectName(QString::fromLatin1(editName));
        edit->setLineWrapMode(QPlainTextEdit::NoWrap);
        edit->setMaximumBlockCount(0);
        v->addWidget(edit);
        layout->addWidget(box, 1);
        return edit;
    };
    mappingEdit_ = addSource(QCoreApplication::translate("PartsImportDialog", "Mapping file (or paste it)"), "partsMappingEdit", "partsMappingBrowse", true);
    partsEdit_ = addSource(QCoreApplication::translate("PartsImportDialog", "Parts list, CSV (or paste it)"), "partsListEdit", "partsListBrowse", false);
    fileNote_ = new QLabel(this);
    fileNote_->setObjectName(QStringLiteral("partsFileNote"));
    fileNote_->setWordWrap(true);
    fileNote_->setTextFormat(Qt::PlainText);
    layout->addWidget(fileNote_);

    auto* tools = new QGroupBox(QCoreApplication::translate("PartsImportDialog", "Tools and options"), this);
    auto* tv = new QVBoxLayout(tools);
    const auto addCombo = [&](const QString& label, const char* name) {
        auto* row = new QHBoxLayout;
        row->addWidget(new QLabel(label, tools));
        auto* combo = new QComboBox(tools);
        combo->setObjectName(QString::fromLatin1(name));
        combo->addItem(QCoreApplication::translate("PartsImportDialog", "(none)"), QString());
        for (const auto& [id, tool] : db_.tools()) {
            combo->addItem(qs(tool.name.empty() ? id : tool.name), qs(id));
        }
        row->addWidget(combo, 1);
        tv->addLayout(row);
        connect(combo, &QComboBox::currentIndexChanged, this, [this]() { refresh(); });
        return combo;
    };
    profileCombo_ = addCombo(QCoreApplication::translate("PartsImportDialog", "Profile tool"), "partsProfileTool");
    drillCombo_ = addCombo(QCoreApplication::translate("PartsImportDialog", "Drilling tool"), "partsDrillTool");
    pocketCombo_ = addCombo(QCoreApplication::translate("PartsImportDialog", "Pocket (dado) tool"), "partsPocketTool");
    toolpathsCheck_ = new QCheckBox(QCoreApplication::translate("PartsImportDialog", "Create toolpaths"), tools);
    toolpathsCheck_->setObjectName(QStringLiteral("partsCreateToolpaths"));
    toolpathsCheck_->setChecked(true);
    partialCheck_ = new QCheckBox(QCoreApplication::translate("PartsImportDialog", "Import the rows without errors (allow a partial import)"), tools);
    partialCheck_->setObjectName(QStringLiteral("partsAllowPartial"));
    tv->addWidget(toolpathsCheck_);
    tv->addWidget(partialCheck_);
    layout->addWidget(tools);
    connect(toolpathsCheck_, &QCheckBox::toggled, this, [this]() { refresh(); });
    connect(partialCheck_, &QCheckBox::toggled, this, [this]() { refresh(); });

    summary_ = new QLabel(this);
    summary_->setObjectName(QStringLiteral("partsSummary"));
    summary_->setTextFormat(Qt::PlainText);
    summary_->setWordWrap(true);
    layout->addWidget(summary_);
    diagnostics_ = new QListWidget(this);
    diagnostics_->setObjectName(QStringLiteral("partsDiagnostics"));
    diagnostics_->setWordWrap(true);
    layout->addWidget(diagnostics_, 1);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    ok_ = buttons->button(QDialogButtonBox::Ok);
    ok_->setText(QCoreApplication::translate("PartsImportDialog", "Import"));
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);

    connect(mappingEdit_, &QPlainTextEdit::textChanged, this, [this]() { refresh(); });
    connect(partsEdit_, &QPlainTextEdit::textChanged, this, [this]() { refresh(); });
    refresh();
}

QComboBox* PartsImportDialog::toolCombo(document::ToolpathOperation operation) const {
    switch (operation) {
        case document::ToolpathOperation::Profile:
            return profileCombo_;
        case document::ToolpathOperation::Drill:
            return drillCombo_;
        case document::ToolpathOperation::Pocket:
            return pocketCombo_;
        default:
            return nullptr;
    }
}

void PartsImportDialog::refresh() {
    if (loading_) {
        return;
    }
    form_.setMappingText(mappingEdit_->toPlainText().toStdString());
    form_.setPartsText(partsEdit_->toPlainText().toStdString());
    for (const document::ToolpathOperation op :
         {document::ToolpathOperation::Profile, document::ToolpathOperation::Drill, document::ToolpathOperation::Pocket}) {
        document::ToolSource source;
        source.toolId = toolCombo(op)->currentData().toString().toStdString();
        form_.setTool(op, source);
    }
    form_.setCreateToolpaths(toolpathsCheck_->isChecked());
    form_.setAllowPartial(partialCheck_->isChecked());
    const viewmodel::PartsImportPreview& p = form_.preview();

    diagnostics_->clear();
    int shown = 0;
    for (const io::PartsDiagnostic& d : p.diagnostics) {
        if (shown >= kMaxShownDiagnostics) {
            diagnostics_->addItem(QCoreApplication::translate("PartsImportDialog", "... more diagnostics not shown"));
            break;
        }
        const QString where = d.line == 0 ? qs(d.source) : QCoreApplication::translate("PartsImportDialog", "%1 line %2").arg(qs(d.source)).arg(d.line);
        diagnostics_->addItem(QStringLiteral("%1 (%2): %3")
                                  .arg(d.severity == io::PartsDiagnostic::Severity::Error ? QCoreApplication::translate("PartsImportDialog", "Error")
                                                                                          : QCoreApplication::translate("PartsImportDialog", "Warning"),
                                       where, qs(d.text)));
        ++shown;
    }
    QString text;
    if (!p.mappingOk) {
        text = QCoreApplication::translate("PartsImportDialog", "The mapping is not valid yet.");
    } else if (!p.partsRead) {
        text = QCoreApplication::translate("PartsImportDialog", "The parts list could not be read.");
    } else {
        text = QCoreApplication::translate("PartsImportDialog", "%1 part row(s), %2 piece(s); %3 row(s) skipped; %4 error(s), %5 warning(s).")
                   .arg(p.parts)
                   .arg(p.instances)
                   .arg(p.rowsSkipped)
                   .arg(p.errors)
                   .arg(p.warnings);
        if (p.errors > 0 && !p.canApply) {
            text += QCoreApplication::translate("PartsImportDialog", " Fix the errors or allow a partial import.");
        }
    }
    summary_->setText(text);
    ok_->setEnabled(p.canApply);
}

bool PartsImportDialog::canApply() const { return ok_->isEnabled(); }

QString PartsImportDialog::loadInto(const QString& path, bool mapping) {
    std::string err;
    const auto text = io::readBoundedFile(path.toStdString(), mapping ? io::kMaxMappingBytes : io::kMaxPartsFileBytes, &err);
    if (!text) {
        return qs(err.empty() ? QCoreApplication::translate("PartsImportDialog", "the file could not be read").toStdString() : err);
    }
    (mapping ? mappingEdit_ : partsEdit_)->setPlainText(QString::fromUtf8(text->data(), static_cast<qsizetype>(text->size())));
    return {};
}

void PartsImportDialog::browse(bool mapping) {
    const QString path =
        QFileDialog::getOpenFileName(this, mapping ? QCoreApplication::translate("PartsImportDialog", "Mapping file") : QCoreApplication::translate("PartsImportDialog", "Parts list"),
                                     QString(), mapping ? QCoreApplication::translate("PartsImportDialog", "Mapping files (*.map *.txt *.ini);;All files (*)")
                                                        : QCoreApplication::translate("PartsImportDialog", "CSV files (*.csv *.txt);;All files (*)"));
    if (path.isEmpty()) {
        return;
    }
    const QString why = loadInto(path, mapping);
    fileNote_->setText(why.isEmpty() ? QString() : QCoreApplication::translate("PartsImportDialog", "Could not read %1: %2").arg(path, why));
}

std::string PartsImportDialog::chooseMappingFile(const QString& path) {
    const QString why = loadInto(path, true);
    fileNote_->setText(why);
    return why.toStdString();
}

std::string PartsImportDialog::choosePartsFile(const QString& path) {
    const QString why = loadInto(path, false);
    fileNote_->setText(why);
    return why.toStdString();
}

void PartsImportDialog::typeMapping(const QString& text) { mappingEdit_->setPlainText(text); }
void PartsImportDialog::typeParts(const QString& text) { partsEdit_->setPlainText(text); }

bool PartsImportDialog::pickTool(document::ToolpathOperation operation, const std::string& toolId) {
    QComboBox* combo = toolCombo(operation);
    if (combo == nullptr) {
        return false;
    }
    const int index = combo->findData(qs(toolId));
    if (index < 0) {
        return false;
    }
    combo->setCurrentIndex(index);
    return true;
}

void PartsImportDialog::setAllowPartial(bool on) { partialCheck_->setChecked(on); }
void PartsImportDialog::setCreateToolpaths(bool on) { toolpathsCheck_->setChecked(on); }

}  // namespace carvetto::app
