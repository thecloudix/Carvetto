#pragma once

#include <QImage>
#include <QTreeWidget>
#include <QWidget>
#include <array>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/ids.hpp"
#include "carvetto/viewmodel/draw_3d.hpp"
#include "carvetto/viewmodel/model_view.hpp"
#include "carvetto/viewmodel/wrapped_view.hpp"
#include "carvetto/viewmodel/relief_tree.hpp"
#include "document_panels.hpp"

class QCheckBox;
class QComboBox;
class QLabel;
class QLineEdit;
class QPainter;
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
    /// Appearance controls (T-0305), for the self-test.
    QComboBox* appearanceBox() const { return appearance_; }
    QComboBox* materialBox() const { return appearanceMaterial_; }
    QLineEdit* colourEdit() const { return appearanceColour_; }
    bool showingComponent() const { return showing_; }

   private:
    void fill();

    QLineEdit* name_;
    QComboBox* mode_;
    QComboBox* appearance_;
    QComboBox* appearanceMaterial_;
    QLineEdit* appearanceColour_;
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
    /// Shows `field` alone instead of the document's model (the modelling forms' preview, T-0300);
    /// `clearPreview` returns to the model. `refresh` keeps showing the preview until cleared.
    void setPreview(const relief::HeightField& field);
    /// Shows an already composed surface (the sculpt preview merged with the model, T-0316).
    void setPreviewSurface(const viewmodel::ModelSurface& surface);
    void clearPreview();
    bool previewing() const { return previewing_; }
    /// Brush painting (T-0302): when set, a left-button press, drag and release on the model call
    /// this with the design-space point under the pointer (Press, Move) or an empty point (Release).
    /// Press returns false when nothing is hit.
    enum class BrushPhase { Press, Move, Release };
    std::function<void(std::optional<relief::Point>, BrushPhase)> onBrush;
    /// Design-space point of the model under the widget position `pos`.
    std::optional<relief::Point> pickAt(QPoint pos);

    /// Drawing in the 3D view (T-0345): with a shape set, left-click or left-drag on the zero plane draws it
    /// as an ordinary document vector (one undo step); Enter / double-click finish a polyline, Escape cancels.
    /// `onDrawCommitted` is called after each vector is added so the window can refresh.
    void setDrawShape(std::optional<viewmodel::Draw3DShape> shape);
    std::optional<viewmodel::Draw3DShape> drawShape() const { return drawShape_; }
    viewmodel::Draw3DController& drawController() { return draw_; }
    std::function<void()> onDrawCommitted;
    /// Widget position of the zero-plane point (x, y) mm (self-test); nullopt without a surface.
    std::optional<viewmodel::ScreenPoint> planePixel(double x, double y);
    /// Last refusal message of a draw pick (empty when none); shown as the view's tooltip.
    const QString& drawMessage() const { return drawMessage_; }
    viewmodel::OrbitCamera& camera() { return camera_; }
    void cameraChanged();

    /// The Auto-Wrapping View (T-0334): for a rotary job the model, the visible vectors and the toolpaths are drawn
    /// round the cylinder (`setWrapped(true)`, the default) or flat. Brush picking and the reference overlay
    /// need the flat view. `onWrappedToggled` is called when the user flips the view's own toggle button.
    void setWrapped(bool on);
    bool wrappedRequested() const { return rotaryView_.mode == viewmodel::RotaryViewMode::Wrapped; }
    /// The job is rotary (the toggle is offered).
    bool wrappedAvailable() const { return rotaryAvailable_; }
    /// The picture is the wrapped one now.
    bool showingWrapped() const { return rotaryAvailable_ && wrappedRequested(); }
    std::function<void(bool)> onWrappedToggled;
    /// A layout being set up (T-0340): its flat contours are drawn on the cylinder (rotary jobs) in an accent colour
    /// until cleared. Not part of the document.
    void setLayoutPreview(std::vector<geometry::Contour> contours);
    void clearLayoutPreview();
    bool hasLayoutPreview() const { return !layoutPreview_.empty(); }
    /// Number of path runs of the vectors/toolpaths/preview drawn on the wrapped cylinder now (self-test).
    std::size_t wrappedPathRuns() const;

    /// Reference image laid over the model on the design plane at the model's top, drawn with
    /// `opacity` (0..1) (T-0309). It is a plane drawn over the surface, not occluded by it.
    void setOverlay(const QImage& image, double opacity);
    void clearOverlay();
    bool hasOverlay() const { return !overlay_.isNull(); }
    double overlayOpacity() const { return overlayOpacity_; }
    /// Screen position of design-space corner `index` (0 top-left, 1 top-right, 2 bottom-left in
    /// image terms) of the overlay; nullopt without an overlay (self-test).
    std::optional<viewmodel::ScreenPoint> overlayCorner(int index);

    /// Brush ring cursor (T-0309): while this returns a diameter in mm, hovering the model draws a
    /// ring of that size on the surface under the pointer. Unset: no ring.
    std::function<std::optional<double>()> brushDiameter;
    /// The ring last drawn, in pixels (empty when none).
    std::vector<viewmodel::ScreenPoint> ringOutline();
    /// Shows the ring at design point `at` (what hovering does).
    void setRingCentre(std::optional<relief::Point> at);

    /// Renders now at the current size (the paint handler does this lazily).
    const QImage& image();
    const viewmodel::ModelSurface& surface() const { return surface_; }
    viewmodel::ModelMaterial material() const { return options_.material; }

   protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void leaveEvent(QEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

   private:
    void invalidate();
    viewmodel::Draw3DView drawView();
    void finishDrawEvent(const viewmodel::Draw3DResult& r);
    void paintEditHandles(QPainter& p);

    DocumentGetter getter_;
    viewmodel::Draw3DController draw_;
    std::optional<viewmodel::Draw3DShape> drawShape_;
    bool drawing_{false};
    QString drawMessage_;
    QComboBox* drawBox_{nullptr};
    viewmodel::ModelSurface surface_;
    viewmodel::OrbitCamera camera_;
    viewmodel::ModelRenderOptions options_;
    QImage image_;
    bool dirty_{true};
    bool previewing_{false};
    viewmodel::ModelSurface previewSurface_;
    QPoint last_;
    bool painting_{false};
    QImage overlay_;
    double overlayOpacity_{0.5};
    std::optional<relief::Point> ringCentre_;
    QComboBox* materialBox_;
    QPushButton* wrappedButton_{nullptr};
    viewmodel::RotaryViewState rotaryView_;
    bool rotaryAvailable_{false};
    std::vector<viewmodel::ModelPath> wrappedPaths_;
    std::vector<geometry::Contour> layoutPreview_;
};

}  // namespace carvetto::app
