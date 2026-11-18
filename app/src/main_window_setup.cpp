// Setup menu of the main window (T-0235): job setup and the tool / machine / material editor.
// Kept apart from main_window.cpp; the window's other code only wires the actions.

#include <QComboBox>
#include <QDir>
#include <QMessageBox>
#include <QStandardPaths>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/commands.hpp"
#include "carvetto/viewmodel/double_sided_ops.hpp"
#include "carvetto/viewmodel/file_session.hpp"
#include "carvetto/viewmodel/new_job_form.hpp"
#include "carvetto/viewmodel/rotary_material.hpp"
#include "carvetto/viewmodel/tool_setup.hpp"
#include "carvetto/viewmodel/variables_model.hpp"
#include "canvas_widget.hpp"
#include "main_window.hpp"
#include "options_dialog.hpp"
#include "preview_panel.hpp"
#include "ruler_widget.hpp"
#include "new_job_dialog.hpp"
#include "relief_panels.hpp"
#include "rotary_layout_dialog.hpp"
#include "rotary_material_dialog.hpp"
#include "tool_setup_dialog.hpp"
#include "variables_dialog.hpp"
#include "toolpaths_panel.hpp"

namespace carvetto::app {

namespace {

QString toQ(const std::string& s) {
    return QString::fromUtf8(s.data(), static_cast<qsizetype>(s.size()));
}

}  // namespace

void MainWindow::jobSetupDialog() {
    NewJobDialog dialog(viewmodel::formFromJob(session_.document().job()), this);
    dialog.setWindowTitle(QStringLiteral("Job Setup"));
    if (dialog.exec() != QDialog::Accepted || !dialog.job().has_value()) {
        return;
    }
    // Double-sided jobs pair or unpair their sheets together with the job type (T-0261).
    const viewmodel::EditResult result = viewmodel::applyJobSetup(session_.document(), *dialog.job());
    if (!result.ok && !quiet_) {
        QMessageBox::warning(this, QStringLiteral("Job Setup"), toQ(result.message));
    }
    if (result.changed) {
        toolpaths_->invalidate();
        documentChanged();
    }
}

void MainWindow::materialSetupDialog() {
    const std::optional<viewmodel::RotaryMaterialForm> initial =
        viewmodel::rotaryMaterialFromJob(session_.document().job());
    if (!initial) {
        return;
    }
    RotaryMaterialDialog dialog(*initial, this);
    if (dialog.exec() != QDialog::Accepted || !dialog.result().has_value()) {
        return;
    }
    const viewmodel::EditResult result = viewmodel::applyRotaryMaterial(session_.document(), *dialog.result());
    if (!result.ok && !quiet_) {
        QMessageBox::warning(this, QStringLiteral("Material Setup"), toQ(result.message));
    }
    if (result.changed) {
        toolpaths_->invalidate();
        documentChanged();
    }
}

void MainWindow::toolSetupDialog() {
    if (toolpaths_->busy()) {
        if (!quiet_) {
            QMessageBox::information(this, QStringLiteral("Tools, Machines and Materials"),
                                     QStringLiteral("Wait for the toolpath calculation to finish."));
        }
        return;
    }
    const document::Document& doc = session_.document();
    ToolSetupDialog dialog(
        toolpaths_->toolDatabase(), toolpaths_->activeMachineId(), doc.job().displayUnits,
        [&doc](const std::string& toolId) {
            std::vector<std::string> names;
            for (const viewmodel::ToolpathUse& use : viewmodel::toolpathsUsingTool(doc, toolId)) {
                names.push_back(use.name);
            }
            return names;
        },
        this);
    if (dialog.exec() != QDialog::Accepted) {
        return;
    }
    toolpaths_->applySetup(dialog.database(), dialog.activeMachineId());
    if (!toolpaths_->setupSaveError().empty() && !quiet_) {
        QMessageBox::warning(this, QStringLiteral("Tools, Machines and Materials"),
                             QStringLiteral("The setup could not be saved: ") + toQ(toolpaths_->setupSaveError()));
    }
    documentChanged();
}

void MainWindow::variablesDialog() {
    if (toolpaths_->busy()) {
        if (!quiet_) {
            QMessageBox::information(this, QStringLiteral("Document Variables"),
                                     QStringLiteral("Wait for the toolpath calculation to finish."));
        }
        return;
    }
    viewmodel::VariablesModel model(&session_.document(), &toolpaths_->session());
    model.setDeferRecalculation(true);  // toolpaths recalculate in the background (T-0271)
    model.setBusyCheck([this]() { return toolpaths_->busy(); });
    VariablesDialog dialog(model, this);
    dialog.onEdited = [this]() { documentChanged(); };
    dialog.onRecalculate = [this]() { toolpaths_->recalculateBound(); };
    dialog.exec();
}

void MainWindow::loadUserSetup(const QString& activeMachine) {
    const QString folder = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
    if (folder.isEmpty() && prefs_.current().toolDatabasePath.empty()) {
        return;
    }
    std::filesystem::path file = viewmodel::pathFromUtf8(folder.toStdString()) / viewmodel::kSetupFileName;
    if (!prefs_.current().toolDatabasePath.empty()) {
        file = viewmodel::pathFromUtf8(prefs_.current().toolDatabasePath);  // the Options preference wins
    }
    const std::string message = toolpaths_->useSetupFile(file, activeMachine.toStdString());
    if (!message.empty() && !quiet_) {
        QMessageBox::warning(this, QStringLiteral("Tool setup"), toQ(message));
    }
}

bool MainWindow::rulersHidden() const {
    return hRuler_->isHidden() && vRuler_->isHidden();
}

QString MainWindow::dialogStart(const std::string& folderUtf8, const QString& name) const {
    if (folderUtf8.empty()) {
        return name;
    }
    return QDir(toQ(folderUtf8)).filePath(name);
}

std::vector<viewmodel::PrefIssue> MainWindow::setPreferences(const viewmodel::Preferences& p) {
    std::vector<viewmodel::PrefIssue> issues = prefs_.apply(p);
    if (!issues.empty()) {
        return issues;
    }
    applyPreferences(prefs_.current());
    if (persistLayout_) {
        QSettingsPreferencesStore store;
        prefs_.save(store);
    }
    return issues;
}

void MainWindow::applyPreferences(const viewmodel::Preferences& p) {
    const bool first = !applied_.has_value();
    const viewmodel::Preferences before = applied_.value_or(viewmodel::Preferences{});
    applied_ = p;

    DisplayOptions& display = displayOptions();
    display.background = QColor::fromRgb(p.canvasBackground);
    display.material = QColor::fromRgb(p.materialFill);
    display.antialiasing = p.antialiasing;
    hRuler_->setVisible(p.showRulers);
    vRuler_->setVisible(p.showRulers);
    rulerCorner_->setVisible(p.showRulers);
    canvas_->refresh();

    session_.document().undoStack().setLimit(static_cast<std::size_t>(p.undoDepth));
    if (first || p.simulationQuality != before.simulationQuality) {
        // The preference is the starting quality; the Preview panel's own choice follows it when it changes.
        QComboBox* combo = preview_->qualityCombo();
        const int i = combo->findData(static_cast<int>(p.simulationQuality));
        if (i >= 0) {
            combo->setCurrentIndex(i);
        }
    }
    if (!first && persistLayout_ && p.toolDatabasePath != before.toolDatabasePath) {
        loadUserSetup(QString::fromStdString(toolpaths_->activeMachineId()));
    }
}

void MainWindow::optionsDialog() {
    OptionsDialog dialog(prefs_.current(), this);
    if (dialog.exec() != QDialog::Accepted) {
        return;
    }
    const std::vector<viewmodel::PrefIssue> issues = setPreferences(dialog.edited());
    if (!issues.empty() && !quiet_) {
        QMessageBox::warning(this, QStringLiteral("Options"), toQ(issues.front().message));
    }
}

}  // namespace carvetto::app

namespace carvetto::app {

RotaryLayoutDialog* MainWindow::makeRotaryLayoutDialog(viewmodel::RotaryLayoutKind kind) {
    document::Document& doc = session_.document();
    if (doc.job().type != document::JobType::Rotary) {
        return nullptr;
    }
    viewmodel::RotaryLayoutForm form(kind, doc.job().displayUnits);
    form.startFrom(doc);
    auto* dialog = new RotaryLayoutDialog(
        std::move(form),
        [this](const viewmodel::RotaryLayoutForm& f) { return f.preview(session_.document()); }, this);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    // The wrapped Model view shows the vectors while the dialog is open.
    ModelView3D* view = modelView_;
    dialog->setPreviewSink([view](const viewmodel::RotaryLayoutPreview& p) {
        if (p.ok) {
            view->setLayoutPreview(p.contours);
        } else {
            view->clearLayoutPreview();
        }
    });
    connect(dialog, &QObject::destroyed, this, [view]() { view->clearLayoutPreview(); });
    return dialog;
}

void MainWindow::rotaryLayoutDialog(viewmodel::RotaryLayoutKind kind) {
    RotaryLayoutDialog* dialog = makeRotaryLayoutDialog(kind);
    if (dialog == nullptr) {
        return;
    }
    if (dialog->exec() == QDialog::Accepted) {
        applyRotaryLayout(dialog->form());
    }
    delete dialog;
}

bool MainWindow::applyRotaryLayout(const viewmodel::RotaryLayoutForm& form) {
    document::Document& doc = session_.document();
    const viewmodel::RotaryLayoutResult result = form.apply(doc);
    if (!result.ok) {
        if (!quiet_) {
            QMessageBox::warning(this, QStringLiteral("Rotary layout"), toQ(result.message));
        }
        return false;
    }
    modelView_->clearLayoutPreview();
    if (form.kind() == viewmodel::RotaryLayoutKind::Fluting && result.created.has_value()) {
        // A Fluting toolpath on this layout takes its depth from it.
        toolpaths_->session().setLayoutFlutingDepth(*result.created, form.flutingParams(doc).depth);
    }
    documentChanged();
    return true;
}

}  // namespace carvetto::app
