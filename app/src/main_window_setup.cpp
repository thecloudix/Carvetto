// Setup menu of the main window (T-0235): job setup and the tool / machine / material editor.
// Kept apart from main_window.cpp; the window's other code only wires the actions.

#include <QMessageBox>
#include <QStandardPaths>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "carvetto/document/commands.hpp"
#include "carvetto/viewmodel/file_session.hpp"
#include "carvetto/viewmodel/new_job_form.hpp"
#include "carvetto/viewmodel/tool_setup.hpp"
#include "carvetto/viewmodel/variables_model.hpp"
#include "main_window.hpp"
#include "new_job_dialog.hpp"
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
    if (session_.document().execute(std::make_unique<document::SetJobCommand>(*dialog.job()))) {
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
    VariablesDialog dialog(model, this);
    dialog.onEdited = [this]() { documentChanged(); };
    dialog.exec();
}

void MainWindow::loadUserSetup(const QString& activeMachine) {
    const QString folder = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
    if (folder.isEmpty()) {
        return;
    }
    const std::filesystem::path file =
        viewmodel::pathFromUtf8(folder.toStdString()) / viewmodel::kSetupFileName;
    const std::string message = toolpaths_->useSetupFile(file, activeMachine.toStdString());
    if (!message.empty() && !quiet_) {
        QMessageBox::warning(this, QStringLiteral("Tool setup"), toQ(message));
    }
}

}  // namespace carvetto::app
