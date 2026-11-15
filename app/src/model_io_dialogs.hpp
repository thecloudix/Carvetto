#pragma once

#include <QDialog>
#include <string>

#include "carvetto/io/mesh.hpp"
#include "carvetto/viewmodel/model_io.hpp"

class QCheckBox;
class QComboBox;
class QDialogButtonBox;
class QDoubleSpinBox;
class QLabel;

namespace carvetto::app {

class MainWindow;
/// Self-test of the 3D import and export dialogs (T-0301); empty on success.
std::string modelIoSelfTest(MainWindow& window);

/// Orientate Model: the dialog File > Import 3D Model opens after the mesh file was read. Units,
/// top face, rotation, size with ratio lock, centring and the zero plane are edited with a live
/// top-down preview in which undercut cells (material hidden below the top surface, which a
/// height field cannot hold) are red. `settings()` is what the import uses.
class MeshImportDialog : public QDialog {
   public:
    MeshImportDialog(io::TriangleMesh mesh, const QString& fileName, QWidget* parent = nullptr);

    viewmodel::MeshImportSettings settings() const;
    void setSettings(const viewmodel::MeshImportSettings& s);
    const io::TriangleMesh& mesh() const { return mesh_; }
    const viewmodel::MeshImportPreview& preview() const { return preview_; }
    /// Name for the new component (the file name without extension).
    std::string componentName() const { return name_; }
    QLabel* undercutLabel() const { return undercuts_; }
    QLabel* previewLabel() const { return picture_; }
    QDialogButtonBox* buttons() const { return buttons_; }

   private:
    void update();

    io::TriangleMesh mesh_;
    std::string name_;
    viewmodel::MeshImportPreview preview_;
    QComboBox *units_, *top_, *rotation_, *driver_;
    QCheckBox *resize_, *lock_, *centre_, *zeroBottom_, *discard_;
    QDoubleSpinBox *sizeX_, *sizeY_, *sizeZ_, *zero_, *cell_;
    QLabel *picture_, *undercuts_, *info_;
    QDialogButtonBox* buttons_;
    bool updating_{false};
};

/// Export Model options: back face (open surface or closed solid with a flat back at a given Z,
/// automatic by default) and the heightmap format. Opened by the three File > Export actions.
class ModelExportDialog : public QDialog {
   public:
    ModelExportDialog(viewmodel::ModelExportKind kind, double autoBackZ, QWidget* parent = nullptr);

    viewmodel::ModelExportSettings settings() const;
    void setSettings(const viewmodel::ModelExportSettings& s);

   private:
    void sync();

    viewmodel::ModelExportKind kind_;
    QComboBox *back_, *format_;
    QCheckBox* auto_;
    QDoubleSpinBox* backZ_;
    double autoZ_;
};

}  // namespace carvetto::app
