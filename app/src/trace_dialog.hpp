#pragma once

#include <QDialog>
#include <QString>
#include <memory>
#include <optional>
#include <string>

#include "carvetto/document/ids.hpp"
#include "carvetto/document/units.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/io/raster.hpp"
#include "carvetto/viewmodel/bitmap_trace.hpp"

class QAction;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QMenu;
class QProgressBar;
class QPushButton;
class QSpinBox;
class QTimer;

namespace carvetto::app {

class CanvasWidget;
class MainWindow;

/// The Trace Bitmap dialog (T-0260, ADR-0038): mode, threshold/invert, colours, noise filter, corner
/// fit, size, a live preview and a Trace button. Hosts widgets only: validation, the shrunken
/// preview image, the worker thread and the cancel flag come from `viewmodel/bitmap_trace.hpp`.
/// Preview and the final trace both run on a worker thread; changing a setting cancels the preview
/// in progress and restarts it. "Trace" runs the full-size trace; Cancel stops it (or closes the
/// dialog when idle). `result()` holds the layers once the dialog was accepted.
class TraceBitmapDialog : public QDialog {
   public:
    TraceBitmapDialog(io::RasterImage image, const QString& fileName, document::Units units, QWidget* parent = nullptr);
    ~TraceBitmapDialog() override;

    // --- State the self-test drives and reads -------------------------------------------------
    viewmodel::TraceForm form() const;
    void setForm(const viewmodel::TraceForm& form);
    /// Runs the event loop until the preview is up to date (or ~20 s). True when idle.
    bool waitIdle();
    /// Starts the full trace and waits for it; true when it produced layers (the dialog is then accepted).
    bool traceNow();
    bool previewRunning() const;
    std::size_t previewLayerCount() const { return previewLayers_; }
    QString problemShown() const;
    const viewmodel::TraceOutcome& result() const { return result_; }
    io::TraceMode resultMode() const { return resultMode_; }
    bool tracing() const { return tracing_; }
    void startTrace();
    void cancelTrace();

    /// Result code of `exec()` when the user pressed "Pick from 2D View...": the caller lets the
    /// user click a bitmap on the canvas and opens a new dialog for it (T-0274).
    static constexpr int kPickRequested = 2;
    /// The image comes from a bitmap placed in the drawing: the result is positioned at
    /// `originMm`, and Bitmap Fading (dimming that bitmap on `canvas` while the dialog is open)
    /// is available.
    void setSourceFromView(geometry::Vec2 originMm, CanvasWidget* canvas);
    bool sourceFromView() const { return canvas_ != nullptr; }
    /// Bitmap Fading in percent (0 = shown as is, 100 = invisible). Applies to the canvas at once.
    int fadePercent() const;
    void setFadePercent(int percent);
    bool fadeEnabled() const;
    /// Same as pressing "Pick from 2D View...": closes the dialog with `kPickRequested`.
    void requestPick();

    void accept() override;
    void reject() override;
    void done(int code) override;

   private:
    void settingsChanged();
    void refreshPreview();
    void onWorkerDone();
    void showPreview(const viewmodel::TraceOutcome& outcome);
    void updateEnabling();
    void applyFade();
    void restoreFade();

    io::RasterImage fullStorage_;
    std::shared_ptr<const io::RasterImage> full_;
    std::shared_ptr<const io::RasterImage> small_;
    double previewScale_{1.0};
    document::Units units_;
    viewmodel::TraceRunner runner_;
    viewmodel::TraceOutcome result_;
    io::TraceMode resultMode_{io::TraceMode::BlackWhite};
    bool tracing_{false};
    bool pendingRestart_{false};
    bool previewRunning_{false};
    bool closing_{false};
    std::size_t previewLayers_{0};

    QComboBox* mode_{nullptr};
    QSpinBox* threshold_{nullptr};
    QCheckBox* invert_{nullptr};
    QSpinBox* colours_{nullptr};
    QSpinBox* noise_{nullptr};
    QSpinBox* cornerFit_{nullptr};  // percent
    QComboBox* sizeBasis_{nullptr};
    QDoubleSpinBox* size_{nullptr};
    QLabel* sizeInfo_{nullptr};
    QLabel* preview_{nullptr};
    QLabel* status_{nullptr};
    QProgressBar* busy_{nullptr};
    QPushButton* traceButton_{nullptr};
    QPushButton* cancelButton_{nullptr};
    QTimer* debounce_{nullptr};
    QPushButton* pickButton_{nullptr};
    QSpinBox* fade_{nullptr};
    CanvasWidget* canvas_{nullptr};
    double fadeBefore_{1.0};
    bool fadeApplied_{false};
    geometry::Vec2 origin_{0, 0};
    int imageWidth_{0};
    int imageHeight_{0};
};

/// File > Trace Bitmap... and Edit > Trace Bitmap...: choose an image, decode it (`decodeImageFile`),
/// run the dialog, add the result as one undo step (`viewmodel::applyTrace`).
class TraceBitmapMenu {
   public:
    /// Adds the action to both menus (the File one before its last separator, if any).
    TraceBitmapMenu(MainWindow& window, QMenu* fileMenu, QMenu* editMenu);

    QAction* fileAction() const { return fileAction_; }
    QAction* editAction() const { return editAction_; }

    /// The whole command for `path` (asks for a file when empty): decode, run the modal dialog, add
    /// the result. Refusals are shown in a message box when `showErrors`; `error` (optional)
    /// receives the message either way. True when objects were added.
    bool run(const QString& path, QString* error = nullptr, bool showErrors = true);
    /// Starts with Pick from 2D View instead of a file (Edit > Trace Bitmap from Drawing...).
    bool runFromView(QString* error = nullptr, bool showErrors = true);
    QAction* viewAction() const { return viewAction_; }
    /// Lets the user click a bitmap on the canvas (Esc cancels); waits in a nested event loop.
    /// The id of the bitmap, or none. `error` says why when a click found none.
    std::optional<document::ObjectId> pickBitmap(QString* error = nullptr);
    /// Opens the dialog for the bitmap object `id` (decoding its embedded image) and adds the
    /// result like `run`. The loop of "pick again" requests is handled here.
    bool traceBitmapObject(document::ObjectId id, QString* error = nullptr, bool showErrors = true);
    /// Adds an accepted dialog's result to the window's document (one undo step) and refreshes the
    /// window. False with `error` set when nothing was added.
    bool addResult(const TraceBitmapDialog& dialog, QString* error = nullptr, bool showErrors = true);

   private:
    MainWindow& window_;
    QAction* fileAction_{nullptr};
    QAction* editAction_{nullptr};
    QAction* viewAction_{nullptr};
    QString lastFolder_;
};

/// Self-test of Trace Bitmap (T-0260): decodes real PNG/BMP files, refuses oversize and damaged
/// ones, drives the dialog widgets and the menu command, checks one undo step. Returns "" on success.
std::string traceSelfTest(MainWindow& window);

}  // namespace carvetto::app
