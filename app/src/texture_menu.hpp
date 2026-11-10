#pragma once

#include <QDialog>
#include <map>
#include <string>

#include "carvetto/viewmodel/texture_form.hpp"

class QAction;
class QComboBox;
class QLabel;
class QLineEdit;
class QMenu;
class QPushButton;
class QWidget;

namespace carvetto::app {

class MainWindow;

/// Draws a texture preview (the lines over the region's bounding box) scaled to fit.
class TexturePreviewWidget : public QWidget {
   public:
    explicit TexturePreviewWidget(QWidget* parent = nullptr);
    void setPreview(const viewmodel::TexturePreview& preview);

   protected:
    void paintEvent(QPaintEvent* event) override;

   private:
    viewmodel::TexturePreview preview_;
};

/// Create Vector Texture dialog (T-0254). Only hosts widgets: every entry is validated by
/// `viewmodel::TextureForm` as it is typed (a refused entry is flagged and keeps the previous
/// value), the live preview and its statistics come from `TextureForm::preview`. OK stays open
/// while an entry is refused or the texture cannot be made.
class TextureDialog : public QDialog {
   public:
    TextureDialog(viewmodel::TextureForm& form, const document::Document& doc, QWidget* parent = nullptr);

    /// Types `text` into the entry `id` (as a user would); returns the form's verdict.
    viewmodel::FieldResult setEntry(const std::string& id, const std::string& text);
    /// Chooses the target layer by name; false when there is none of that name.
    bool setTargetLayer(const std::string& name);
    /// The preview as last computed.
    const viewmodel::TexturePreview& preview() const { return preview_; }
    /// Text of the status line under the preview.
    std::string statusText() const;
    /// Re-reads the typed entries and recomputes the preview.
    void refreshPreview();

    void accept() override;

   private:
    void entryChanged(const std::string& id);

    viewmodel::TextureForm& form_;
    const document::Document& doc_;
    std::map<std::string, QLineEdit*> edits_;
    std::map<std::string, std::string> errors_;
    QComboBox* layer_{nullptr};
    QLabel* status_{nullptr};
    TexturePreviewWidget* canvas_{nullptr};
    viewmodel::TexturePreview preview_;
};

/// Edit > Create Vector Texture...: enabled while closed vectors are selected. Thin glue over
/// `viewmodel/texture_form.hpp`.
class TextureMenu {
   public:
    TextureMenu(MainWindow& window, QMenu* editMenu);

    QAction* action() const { return action_; }
    viewmodel::TextureForm& form() { return form_; }
    /// Opens the modal dialog and, when accepted, adds the texture (one undo step).
    void openDialog();
    /// Adds the texture for the form as it is; refreshes the window on success.
    viewmodel::TextureResult run(bool showErrors = true);

   private:
    MainWindow& window_;
    QAction* action_{nullptr};
    viewmodel::TextureForm form_;
};

/// Drives the real window: opens the dialog from its menu action, types entries (good and bad),
/// checks the live preview against the selected plate with a hole, accepts, and undoes. Returns an
/// empty string on success, else the failure reason. Leaves the document as it found it.
std::string textureSelfTest(MainWindow& window);

}  // namespace carvetto::app
