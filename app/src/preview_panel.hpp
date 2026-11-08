#pragma once

#include <QElapsedTimer>
#include <QString>
#include <QTimer>
#include <QWidget>
#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include "carvetto/viewmodel/playback.hpp"
#include "carvetto/viewmodel/simulation_controller.hpp"
#include "carvetto/viewmodel/time_readout.hpp"
#include "carvetto/viewmodel/toolpath_preview.hpp"

class QAction;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QListWidget;
class QProgressBar;
class QPushButton;
class QSlider;
class QTreeWidget;

namespace carvetto::app {

class CanvasWidget;
class MainWindow;
class ReliefView;
class ToolpathsPanel;

/// The Preview dock (T-0223, ADR-0032 amendment): what the 2D canvas shows of the toolpaths (on /
/// off, per-toolpath visibility, selected only, move-class legend), the machining-time readout, and
/// the simulation (quality, colouring, Simulate / Stop / Clear, progress, playback scrubber with
/// play / pause / step / speed, diagnostics). All logic is Qt-free in `carvetto_viewmodel`
/// (`PreviewModel`, `makeTimeReadout`, `SimulationController`, `PlaybackModel`); this class is
/// widgets and a 30 ms timer that polls the simulation worker.
///
/// Threading and lifetime: the simulation runs on a worker over COPIES (shared immutable move lists,
/// tool shapes, the stock), never the Document. The panel reads the document only on the UI thread
/// inside `refresh()` (through the toolpaths panel's session) to keep the preview and the "is the
/// simulation still current" check up to date; a result that no longer matches the document is
/// shown with an "out of date" banner and without tool/playback marks on the canvas. Replacing the
/// document (`documentReplaced`) stops the simulation at once.
class PreviewPanel : public QWidget {
   public:
    PreviewPanel(ToolpathsPanel* toolpaths, CanvasWidget* canvas, ReliefView* relief, QWidget* parent = nullptr);
    ~PreviewPanel() override;

    /// Re-reads the toolpaths after the document or the toolpath list changed.
    void refresh();
    /// The toolpaths panel was rebound to another document: stops the simulation and forgets the
    /// scene (call after `ToolpathsPanel::setDocument`; nothing of the old document is touched).
    void documentReplaced();
    /// Stops the simulation worker (window closing).
    void shutdown();

    viewmodel::PreviewModel& model() { return model_; }
    const viewmodel::PreviewModel& model() const { return model_; }
    viewmodel::SimulationController& simulation() { return sim_; }
    viewmodel::PlaybackModel& playback() { return playback_; }

    /// The "Show toolpaths" action (View menu / toolbar) and the check box mirror each other.
    QAction* showAction() const { return showAction_; }
    QCheckBox* showCheck() const { return show_; }
    QCheckBox* selectedOnlyCheck() const { return selectedOnly_; }
    QTreeWidget* toolpathList() const { return list_; }
    QCheckBox* classCheck(viewmodel::MoveClass cls) const { return classChecks_[static_cast<std::size_t>(cls)]; }
    QLabel* timeLabel() const { return time_; }
    QDoubleSpinBox* rapidRate() const { return rapid_; }
    QComboBox* qualityCombo() const { return quality_; }
    QComboBox* colourCombo() const { return colour_; }
    QComboBox* speedCombo() const { return speed_; }
    QPushButton* simulateButton() const { return simulate_; }
    QPushButton* stopButton() const { return stop_; }
    QPushButton* clearButton() const { return clear_; }
    QPushButton* playButton() const { return play_; }
    QPushButton* stepBackButton() const { return stepBack_; }
    QPushButton* stepForwardButton() const { return stepForward_; }
    QSlider* scrubber() const { return scrubber_; }
    QProgressBar* progressBar() const { return progress_; }
    QLabel* statusLabel() const { return status_; }
    QListWidget* diagnostics() const { return diagnostics_; }

    void setShowPreview(bool on);
    /// False when a simulation exists and the document no longer matches it (or none exists).
    bool simulationCurrent() const { return simActive() && simCurrent_; }
    bool simActive() const { return sim_.active(); }
    /// Move count the scrubber is at.
    std::size_t playbackPosition() const { return playback_.position(); }
    /// Runs the event loop until the simulation has produced the frame for the playback position (or
    /// failed). Self-test. False on timeout or failure.
    bool waitSimulation(int timeoutMs);

    /// Starts a simulation of the enabled toolpaths (as the button does). Returns false and sets
    /// the status text when it cannot start.
    bool startSimulation();
    void clearSimulation();
    /// Moves the scrubber to `move` (as dragging it does).
    void scrubTo(std::size_t move);

   private:
    void build();
    void tick();
    void showFrame(const std::shared_ptr<const viewmodel::SimulationFrame>& frame);
    void updateCanvasState();
    void updateControls();
    void updateTime();
    void rebuildToolpathList();
    void updateStatus();
    void seekToPlayback();
    void applyRapidRate();
    void onSliderMoved(int value);
    geometry::Box2 stockBox() const;

    ToolpathsPanel* toolpaths_{nullptr};
    CanvasWidget* canvas_{nullptr};
    ReliefView* relief_{nullptr};

    viewmodel::PreviewModel model_;
    viewmodel::SimulationController sim_;
    viewmodel::PlaybackModel playback_;
    bool playbackReady_{false};
    bool simCurrent_{false};
    std::string simReason_;
    std::uint64_t shownSerial_{0};
    std::size_t lastSeek_{static_cast<std::size_t>(-1)};
    std::string listSignature_;
    bool updatingUi_{false};
    QString setupProblem_;
    std::vector<std::string> setupNotes_;
    QTimer timer_;
    QElapsedTimer clock_;

    QAction* showAction_{nullptr};
    QCheckBox* show_{nullptr};
    QCheckBox* selectedOnly_{nullptr};
    std::array<QCheckBox*, viewmodel::kMoveClassCount> classChecks_{};
    QTreeWidget* list_{nullptr};
    QLabel* time_{nullptr};
    QDoubleSpinBox* rapid_{nullptr};
    QComboBox* quality_{nullptr};
    QComboBox* colour_{nullptr};
    QComboBox* speed_{nullptr};
    QPushButton* simulate_{nullptr};
    QPushButton* stop_{nullptr};
    QPushButton* clear_{nullptr};
    QPushButton* play_{nullptr};
    QPushButton* stepBack_{nullptr};
    QPushButton* stepForward_{nullptr};
    QSlider* scrubber_{nullptr};
    QProgressBar* progress_{nullptr};
    QLabel* status_{nullptr};
    QLabel* notes_{nullptr};
    QListWidget* diagnostics_{nullptr};
};

/// Drives the Preview dock of `window` through its real widgets (self-test, offscreen): toggling the
/// preview, per-toolpath visibility, the time readout, simulation start / scrub / step / play / colour,
/// staleness after edits and recalculation, a million-move toolpath, and the document replaced or the
/// window closed while a simulation runs. Returns an empty string on success, else the first failure.
/// Replaces the window's document and leaves a simulation running (the caller closes the window).
std::string previewSelfTest(MainWindow& window);

}  // namespace carvetto::app
