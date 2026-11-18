// 3D import and export (T-0301): the MainWindow side of File > Import 3D Model and Export 3D Model.
#include <QApplication>
#include <QFileDialog>
#include <QMessageBox>
#include <fstream>

#include "carvetto/io/mesh.hpp"
#include "carvetto/viewmodel/file_session.hpp"
#include "carvetto/viewmodel/model_view.hpp"
#include "main_window.hpp"
#include "model_io_dialogs.hpp"
#include "toolpaths_panel.hpp"

namespace carvetto::app {

namespace {
QString qs(const std::string& s) { return QString::fromStdString(s); }
}  // namespace

bool MainWindow::importMeshFile(const std::filesystem::path& path, std::optional<viewmodel::MeshImportSettings> settings) {
    modelIoError_.clear();
    auto fail = [this](const QString& title) {
        if (!quiet_) {
            QMessageBox::critical(this, title, qs(modelIoError_));
        }
        return false;
    };
    auto finish = [this, &fail](const io::TriangleMesh& m, const viewmodel::MeshImportSettings& s,
                                const std::string& componentName) {
        document::Document& doc = currentDocument();
        const viewmodel::MeshImportOutcome r =
            viewmodel::importMeshIntoDocument(doc, doc.activeSheet(), m, s, componentName);
        if (!r.ok) {
            modelIoError_ = r.error;
            return fail(QStringLiteral("Import failed"));
        }
        documentChanged();
        return true;
    };
    auto mesh = io::readMeshFile(viewmodel::pathToUtf8(path));
    if (!mesh.ok()) {
        modelIoError_ = mesh.error();
        return fail(QStringLiteral("Import failed"));
    }
    const QString fileName = qs(viewmodel::pathToUtf8(path));
    viewmodel::MeshImportSettings use = settings.value_or(viewmodel::MeshImportSettings{});
    std::string name = path.stem().string();
    if (!settings && !quiet_) {
        MeshImportDialog dialog(std::move(mesh.value()), fileName, this);
        if (dialog.exec() != QDialog::Accepted) {
            return false;
        }
        use = dialog.settings();
        name = dialog.componentName();
        return finish(dialog.mesh(), use, name);
    }
    return finish(mesh.value(), use, name);
}

bool MainWindow::createComponentFromPreview(const viewmodel::PreviewComponentRequest& request) {
    modelIoError_.clear();
    if (toolpaths_ == nullptr) {
        modelIoError_ = "there are no toolpaths";
        return false;
    }
    const viewmodel::PreviewComponentOutcome out =
        viewmodel::createComponentFromToolpathPreview(currentDocument(), toolpaths_->session(), request);
    if (!out.ok) {
        modelIoError_ = out.message;
        return false;
    }
    documentChanged();
    return true;
}

void MainWindow::createComponentFromPreviewDialog() {
    // Shift keeps only the cells the tool cut (the rest is transparent); Ctrl subtracts the result instead of adding
    // it (readings of "Shift or Ctrl changes the result", ADR-0091).
    const Qt::KeyboardModifiers mods = QApplication::keyboardModifiers();
    viewmodel::PreviewComponentRequest request;
    if (mods.testFlag(Qt::ShiftModifier)) {
        request.cells = simulate::PreviewCells::CutOnly;
    }
    if (mods.testFlag(Qt::ControlModifier)) {
        request.mode = relief::CombineMode::Subtract;
    }
    QApplication::setOverrideCursor(Qt::WaitCursor);
    const bool ok = createComponentFromPreview(request);
    QApplication::restoreOverrideCursor();
    if (!ok && !quiet_) {
        QMessageBox::warning(this, QStringLiteral("Create component from toolpath preview"), qs(modelIoError_));
    }
}

bool MainWindow::exportModelFile(const std::filesystem::path& path, const viewmodel::ModelExportSettings& settings) {
    modelIoError_.clear();
    const document::Document& doc = currentDocument();
    const viewmodel::ModelExportResult r = viewmodel::exportSheetModel(doc, doc.activeSheet(), settings);
    bool ok = r.ok;
    if (!ok) {
        modelIoError_ = r.error;
    } else {
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        out.write(r.data.data(), static_cast<std::streamsize>(r.data.size()));
        out.flush();
        if (!out) {
            modelIoError_ = "The file could not be written.";
            ok = false;
        }
    }
    if (!ok && !quiet_) {
        QMessageBox::critical(this, QStringLiteral("Export failed"), qs(modelIoError_));
    }
    return ok;
}

void MainWindow::importMeshDialog() {
    const QString name = QFileDialog::getOpenFileName(this, QStringLiteral("Import 3D Model"), dialogStart(preferences().openFolder),
                                                      QStringLiteral("3D models (*.stl *.obj);;All files (*)"));
    if (name.isEmpty()) {
        return;
    }
    importMeshFile(viewmodel::pathFromUtf8(name.toStdString()));
}

void MainWindow::exportModelDialog(viewmodel::ModelExportKind kind) {
    const document::Document& doc = currentDocument();
    const viewmodel::ModelSurface surface = viewmodel::composeModelSurface(doc, doc.activeSheet());
    if (!surface.ok()) {
        QMessageBox::information(this, QStringLiteral("Export 3D Model"),
                                 qs(surface.problem.empty() ? "There is no model to export." : surface.problem));
        return;
    }
    ModelExportDialog options(kind, viewmodel::defaultBackZ(surface.field), this);
    if (options.exec() != QDialog::Accepted) {
        return;
    }
    const viewmodel::ModelExportSettings settings = options.settings();
    const QString name = QFileDialog::getSaveFileName(this, QStringLiteral("Export 3D Model"), dialogStart(preferences().exportFolder),
                                                      QString::fromUtf8(viewmodel::exportFilter(kind)));
    if (name.isEmpty()) {
        return;
    }
    std::filesystem::path path = viewmodel::pathFromUtf8(name.toStdString());
    if (!path.has_extension()) {
        path += viewmodel::exportExtension(kind, settings.heightmap);
    }
    exportModelFile(path, settings);
}

}  // namespace carvetto::app
