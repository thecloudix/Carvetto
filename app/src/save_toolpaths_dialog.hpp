#pragma once

#include <QDialog>
#include <atomic>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/tooling/machine.hpp"
#include "carvetto/viewmodel/save_toolpaths.hpp"

class QCheckBox;
class QComboBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QThread;
class QTimer;

namespace carvetto::app {

/// Where the shipped posts live: next to the executable (`posts`), in `../share/carvetto/posts`,
/// or the source tree's `posts` (development builds). Empty when none is found.
std::filesystem::path shippedPostsDir();
/// The user's own posts: `<app data>/posts`.
std::filesystem::path userPostsDir();

/// The Save Toolpaths dialog (T-0224, ADR-0034): post, machine, units, one or several files,
/// file name template, a preview of the first lines and the errors. Only hosts widgets; all
/// decisions and the G-code come from `viewmodel/save_toolpaths.hpp`. The G-code is generated and
/// written on a worker thread from a copy of the toolpaths, so the dialog never reads the
/// document after construction and the window stays responsive.
class SaveToolpathsDialog : public QDialog {
   public:
    /// `ids` empty = every toolpath. `jobName` fills {job}. `machines` may be empty.
    SaveToolpathsDialog(const document::Document& doc, const std::vector<document::ToolpathId>& ids,
                        const std::string& jobName, const std::vector<tooling::Machine>& machines,
                        const std::filesystem::path& startFolder, QWidget* parent = nullptr);
    ~SaveToolpathsDialog() override;

    // --- State the self-test drives and reads -------------------------------------------------
    void setFolder(const std::filesystem::path& folder);
    void setOutputMode(viewmodel::OutputMode mode);
    void setNameTemplate(const std::string& tmpl);
    void setOverwrite(bool overwrite);
    /// Selects the post whose file name is `fileName` (e.g. "grbl_mm.pp") if it is listed.
    bool selectPost(const std::string& fileName);
    int postCount() const;
    /// Runs the event loop until no generation or write is running (or ~10 s). True when idle.
    bool waitIdle();
    /// The generated output of the current settings.
    const viewmodel::SaveOutput& output() const { return output_; }
    std::string previewShown() const;
    std::string messageShown() const;
    bool canSave() const;
    /// Writes the files as the Save button does and waits for the result. True on success.
    bool saveNow();
    const std::vector<std::filesystem::path>& written() const { return written_; }

    void accept() override;
    void reject() override;

   private:
    void fillPosts();
    void settingsChanged();
    /// Text typed into the folder or name field: the post runs 300 ms after the last keystroke.
    void typingChanged();
    void startGeneration();
    void generationDone(viewmodel::SaveOutput out, unsigned generation);
    void writeDone(viewmodel::WriteResult result);
    void startWrite();
    void showState();
    void setBusy(bool busy);
    viewmodel::SaveOptions currentOptions() const;
    void finishThread();

    viewmodel::SaveSource source_;
    std::string jobName_;
    std::vector<tooling::Machine> machines_;
    std::vector<viewmodel::PostInfo> posts_;
    std::vector<std::size_t> listed_;  ///< Indices into posts_ shown in the post combo.

    QComboBox* machine_{nullptr};
    QComboBox* units_{nullptr};
    QComboBox* post_{nullptr};
    QComboBox* mode_{nullptr};
    QLineEdit* folder_{nullptr};
    QLineEdit* template_{nullptr};
    QCheckBox* overwrite_{nullptr};
    QLabel* summary_{nullptr};
    QLabel* message_{nullptr};
    QPlainTextEdit* preview_{nullptr};
    QPushButton* save_{nullptr};

    viewmodel::SaveOutput output_;
    bool outputFresh_{false};
    std::vector<std::filesystem::path> written_;
    std::string writeError_;

    QThread* thread_{nullptr};
    bool running_{false};
    bool writing_{false};
    bool again_{false};
    bool closeAfterWrite_{false};
    unsigned generation_{0};
    bool updating_{false};
    QTimer* debounce_{nullptr};
    std::shared_ptr<std::atomic<bool>> cancel_;
};

}  // namespace carvetto::app
