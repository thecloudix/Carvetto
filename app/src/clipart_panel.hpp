#pragma once

#include <QWidget>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "carvetto/geometry/vec2.hpp"
#include "carvetto/viewmodel/clipart_insert.hpp"
#include "carvetto/viewmodel/clipart_library.hpp"
#include "carvetto/viewmodel/clipart_thumbnail.hpp"
#include "document_panels.hpp"

class QCheckBox;
class QComboBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;
class QTimer;

namespace carvetto::app {

class MainWindow;

/// The Library dock's clipart browser (T-0333, ADR-0109): user-chosen local folders of SVG / DXF /
/// PNG files shown as thumbnails with a name search and favourites. Thumbnails are made on a worker
/// thread (`viewmodel::ClipartThumbnailService`) and fetched by a timer; double-click or Insert puts
/// a file at the material centre, dragging it onto the canvas drops it at the pointer, each as one
/// undo step. No network, no shipped clipart. Thin glue over `viewmodel/clipart_*.hpp`.
class ClipartPanel : public PanelBase {
   public:
    explicit ClipartPanel(DocumentGetter getter, QWidget* parent = nullptr);
    ~ClipartPanel() override;

    /// Switches between a scratch store (tests) and the application settings (the real app).
    void setPersist(bool persist);

    /// Adds a library folder (UTF-8 path); the message is shown when refused.
    bool addFolder(const std::string& path);
    bool removeFolder(const std::string& path);
    /// Rescans the folders and forgets cached thumbnails.
    void refreshFolders();
    void setSearch(const std::string& text);
    void setFavouritesOnly(bool only);
    /// Marks or unmarks the file shown in `row`.
    bool toggleFavouriteRow(int row);
    /// Inserts the file of `row` at `at` (the material centre when empty); one undo step.
    viewmodel::ClipartInsertOutcome insertRow(int row, std::optional<geometry::Vec2> at = std::nullopt);
    /// Inserts a file by path (drag and drop from the canvas or the file manager).
    viewmodel::ClipartInsertOutcome insertPath(const std::string& path, std::optional<geometry::Vec2> at);

    /// Queues thumbnails for the rows in view (all rows when `all`), and shows those that are done.
    /// The timer does this on its own; the self-test calls it and `waitThumbnails`.
    void pumpThumbnails(bool all = false);
    /// Blocks until the worker is idle, then shows the finished thumbnails (self-test).
    void waitThumbnails();

    int rowCount() const;
    std::string pathOfRow(int row) const;
    bool rowHasThumbnail(int row) const;
    std::string rowToolTip(int row) const;
    QListWidget* list() const { return list_; }
    viewmodel::ClipartLibrary& library() { return *library_; }
    viewmodel::ClipartThumbnailService& thumbnails() { return *service_; }
    std::string statusText() const;
    /// Called with the path of a PNG/SVG/DXF dropped on the canvas.
    void dropOnCanvas(const std::string& path, geometry::Vec2 mm) { insertPath(path, mm); }

   private:
    class Store;
    void rebuildList();
    void applyCompleted();
    void updateStatus(const std::string& extra = {});
    std::unique_ptr<Store> store_;
    std::unique_ptr<viewmodel::ClipartLibrary> library_;
    std::unique_ptr<viewmodel::ClipartThumbnailService> service_;
    QLineEdit* search_{nullptr};
    QCheckBox* favouritesOnly_{nullptr};
    QComboBox* folders_{nullptr};
    QListWidget* list_{nullptr};
    QLabel* status_{nullptr};
    QPushButton* insert_{nullptr};
    QPushButton* favourite_{nullptr};
    QTimer* timer_{nullptr};
    std::vector<std::size_t> shown_;  ///< Library entry index of each row.
};

/// Drives the clipart browser of `window` like a user (T-0333); returns an error text or empty.
std::string clipartSelfTest(MainWindow& window);

}  // namespace carvetto::app
