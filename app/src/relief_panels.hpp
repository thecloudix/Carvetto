#pragma once

#include <QImage>
#include <QTreeWidget>
#include <QWidget>
#include <functional>
#include <string>

#include "carvetto/document/ids.hpp"
#include "carvetto/viewmodel/model_view.hpp"
#include "carvetto/viewmodel/relief_tree.hpp"
#include "document_panels.hpp"

class QCheckBox;
class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;

namespace carvetto::app {

class MainWindow;
/// Self-test of the Component Tree, Component Properties and Model 3D view (T-0299); empty on success.
std::string reliefPanelsSelfTest(MainWindow& window);

/// Drag and drop of the tree: the component moves to a level at a position (bottom-first index).
class ReliefTreeWidget : public QTreeWidget {
   public:
    using QTreeWidget::QTreeWidget;
    std::function<void(document::ComponentId, document::LevelId, std::size_t)> onDrop;

   protected:
    void dropEvent(QDropEvent* event) override;
};

/// Component Tree panel (T-0299, ADR-0077): levels with their components of the active sheet
/// (top of the stack first), show/hide, rename, combine mode, reorder (buttons and drag and drop),
/// duplicate, delete and move to level. Every edit is one undoable command through
/// `viewmodel/relief_tree.hpp`.
class ComponentTreePanel : public PanelBase {
   public:
    explicit ComponentTreePanel(DocumentGetter getter, QWidget* parent = nullptr);

    /// Called with the level the user picked (null when a component or nothing is current).
    std::function<void(document::LevelId)> onLevelPicked;
    /// Called after the component selection changed.
    std::function<void()> onSelectionChanged;

    void refresh();

    // Actions (buttons and self-test).
    bool addLevel(const std::string& name = {});
    bool deleteSelected();
    bool duplicateSelected();
    bool moveSelected(bool up);
    bool setSelectedMode(relief::CombineMode mode);
    bool toggleVisible(document::ComponentId id);
    bool toggleLevelVisible(document::LevelId id);
    bool rename(document::ComponentId id, const std::string& name);
    bool moveSelectedToLevel(document::LevelId level);
    bool drop(document::ComponentId id, document::LevelId level, std::size_t index);
    void selectComponents(const std::vector<document::ComponentId>& ids);

    QTreeWidget* tree() const { return tree_; }
    /// Rows as drawn: level and component items in order, for the self-test.
    int topLevelCount() const;
    QTreeWidgetItem* levelItem(document::LevelId id) const;
    QTreeWidgetItem* componentItem(document::ComponentId id) const;

   private:
    void itemEdited(QTreeWidgetItem* item, int column);
    void selectionEdited();
    void updateMoveTargets();

    ReliefTreeWidget* tree_;
    QComboBox* mode_;
    QComboBox* moveTo_;
    bool refreshing_{false};
};

/// Component Properties form: name, combine mode, shape height, base height, fade and tilt of a
/// component (levels: name, combine mode, base height). Apply writes one undo step.
class ComponentPropertiesPanel : public PanelBase {
   public:
    explicit ComponentPropertiesPanel(DocumentGetter getter, QWidget* parent = nullptr);

    /// Shows this level when no single component is selected.
    void setLevel(document::LevelId level);
    void refresh();
    /// Writes the form into the document (the Apply button).
    bool apply();

    /// Form fields by label, for the self-test.
    QLineEdit* field(const QString& label) const;
    QCheckBox* fadeEnabled() const { return fadeOn_; }
    QCheckBox* tiltEnabled() const { return tiltOn_; }
    QComboBox* modeBox() const { return mode_; }
    bool showingComponent() const { return showing_; }

   private:
    void fill();

    QLineEdit* name_;
    QComboBox* mode_;
    QLineEdit *shape_, *base_, *fadePct_, *fadeAx_, *fadeAy_, *fadeBx_, *fadeBy_;
    QLineEdit *tiltAngle_, *tiltPx_, *tiltPy_, *tiltRx_, *tiltRy_;
    QCheckBox* fadeOn_;
    QCheckBox* tiltOn_;
    QLabel* info_;
    QPushButton* apply_;
    document::LevelId level_;
    document::ComponentId component_;
    bool showing_{false};
    bool refreshing_{false};
};

/// The 3D view of the composed model: shaded surface, orbit (right-drag), zoom (wheel), pan
/// (middle-drag or Ctrl+right-drag), standard views and a material choice. Draws the Qt-free
/// raster of `viewmodel/model_view.hpp`.
class ModelView3D : public QWidget {
   public:
    explicit ModelView3D(DocumentGetter getter, QWidget* parent = nullptr);

    /// Recomposes the model from the document (after an edit) and redraws.
    void refresh();
    void setMaterial(viewmodel::ModelMaterial material);
    viewmodel::OrbitCamera& camera() { return camera_; }
    void cameraChanged();

    /// Renders now at the current size (the paint handler does this lazily).
    const QImage& image();
    const viewmodel::ModelSurface& surface() const { return surface_; }
    viewmodel::ModelMaterial material() const { return options_.material; }

   protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

   private:
    void invalidate();

    DocumentGetter getter_;
    viewmodel::ModelSurface surface_;
    viewmodel::OrbitCamera camera_;
    viewmodel::ModelRenderOptions options_;
    QImage image_;
    bool dirty_{true};
    QPoint last_;
    QComboBox* materialBox_;
};

}  // namespace carvetto::app
