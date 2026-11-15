#pragma once

#include <QWidget>
#include <functional>
#include <string>

#include "carvetto/viewmodel/relief_effects.hpp"
#include "carvetto/viewmodel/relief_sculpt.hpp"
#include "document_panels.hpp"

class QCheckBox;
class QComboBox;
class QLabel;
class QPushButton;
class QSlider;

namespace carvetto::app {

class MainWindow;
/// Self-test of the Sculpt mode and the level effect actions (T-0302); empty on success.
std::string sculptSelfTest(MainWindow& window);

/// Docked Sculpt panel (T-0302, ADR-0084): the brush palette (Deposit, Remove, Smooth, Smudge, Undo
/// Brush), diameter / strength / smoothness sliders, Normal / Raise / Lower mode, preserve
/// transparency, presets, and Undo Stroke / Keep / Discard. While sculpting, the working copy of the
/// chosen component is shown alone in the Model view (`onPreview`) and left-drag there paints
/// (`paint*`, called by the view with design-space points). Nothing reaches the document until Keep,
/// which is one undo step. The logic is `viewmodel/relief_sculpt.hpp`.
class SculptPanel : public PanelBase {
   public:
    explicit SculptPanel(DocumentGetter getter, QWidget* parent = nullptr);

    /// Called with the working field after every change, or null when sculpting ended.
    std::function<void(const relief::HeightField*)> onPreview;
    /// Called when sculpting starts / ends (the window shows the Model view and the brush cursor).
    std::function<void(bool)> onActiveChanged;

    bool startSculpting();   ///< The "Start Sculpting" button: the selected (or only) component.
    void stopSculpting();    ///< Leaves the mode; strokes not kept are dropped.
    bool sculpting() const { return controller_.active(); }
    bool keep();             ///< Writes the working copy into the model (one undo step).
    void discard();          ///< Back to the last kept state.
    bool undoStroke();

    // Brush: the same state the widgets hold.
    void setBrush(const viewmodel::BrushSettings& s);
    viewmodel::BrushSettings brush() const;
    bool applyPreset(const std::string& name);
    bool savePreset(const std::string& name);

    // Painting, called by the Model view with design-space points.
    bool paintPress(relief::Point at);
    bool paintMove(relief::Point at);
    void paintRelease();

    /// Re-syncs after the document changed (component deleted, or an outside undo).
    void refresh();

    viewmodel::SculptController& controller() { return controller_; }
    viewmodel::BrushPresets& presets() { return presets_; }
    QPushButton* brushButton(relief::BrushKind kind) const;
    QSlider* slider(const QString& name) const;
    QComboBox* combo(const QString& name) const;
    QCheckBox* check(const QString& name) const;
    QLabel* status() const { return status_; }

   private:
    void widgetsChanged();
    void writeWidgets(const viewmodel::BrushSettings& s);
    void updateButtons();
    void message(const std::string& text, bool error);
    void showWorking();
    void fillPresets();

    viewmodel::SculptController controller_;
    viewmodel::BrushPresets presets_;
    viewmodel::BrushSettings brush_;
    QLabel* status_;
    QLabel* readout_;
    QPushButton* startButton_;
    QPushButton* keepButton_;
    QPushButton* discardButton_;
    QPushButton* undoButton_;
    QComboBox* presetBox_;
    bool writing_{false};
};

/// Level effect actions (T-0302): Apply / Update / Remove Clipping from the selected closed
/// vectors and the Mirror Mode choice of the current level, plus Bake. Every action is one undo
/// step through `viewmodel/relief_effects.hpp`. The level is the one picked in the Component Tree,
/// else the level of the selected component, else the top level.
class LevelEffectsPanel : public PanelBase {
   public:
    explicit LevelEffectsPanel(DocumentGetter getter, QWidget* parent = nullptr);

    void setLevel(document::LevelId level) { picked_ = level; refresh(); }
    document::LevelId level() const;
    void refresh();

    bool applyClip();
    bool removeClip();
    bool setMirror(relief::MirrorMode mode);
    bool bake();

    QLabel* summary() const { return summary_; }
    QComboBox* mirrorBox() const { return mirror_; }
    QPushButton* applyButton() const { return apply_; }
    QPushButton* removeButton() const { return remove_; }

   private:
    QLabel* summary_;
    QComboBox* mirror_;
    QPushButton* apply_;
    QPushButton* remove_;
    QPushButton* bake_;
    document::LevelId picked_;
    bool refreshing_{false};
};

}  // namespace carvetto::app
