#pragma once

#include <QDialog>
#include <functional>
#include <map>
#include <string>

#include "carvetto/viewmodel/rotary_layout_form.hpp"

class QComboBox;
class QDialogButtonBox;
class QLabel;
class QLineEdit;

namespace carvetto::app {

/// The Wrapped Fluting Layout and Wrapped Spiral Layout dialog (T-0340): one entry per field of a
/// `viewmodel::RotaryLayoutForm` and a combo box per choice, all validated by the form (a bad entry shows its
/// message and the form keeps the old value). `previewFn` is asked for the vectors after every accepted change and
/// OK is disabled while it reports a problem; `onPreview` receives each result so the owner can draw it wrapped.
/// The dialog never touches the document: the owner applies `form()` after OK.
class RotaryLayoutDialog : public QDialog {
   public:
    using PreviewFn = std::function<viewmodel::RotaryLayoutPreview(const viewmodel::RotaryLayoutForm&)>;
    using PreviewSink = std::function<void(const viewmodel::RotaryLayoutPreview&)>;

    RotaryLayoutDialog(viewmodel::RotaryLayoutForm form, PreviewFn previewFn, QWidget* parent = nullptr);

    const viewmodel::RotaryLayoutForm& form() const { return form_; }
    void setPreviewSink(PreviewSink sink);
    const viewmodel::RotaryLayoutPreview& lastPreview() const { return preview_; }
    bool canAccept() const { return preview_.ok; }
    QString message() const;

    // Scripted access (the self-test drives the real widgets).
    bool typeField(const std::string& id, const std::string& text);
    bool chooseOption(const std::string& id, int option);
    QLineEdit* fieldEdit(const std::string& id) const;
    QComboBox* choiceBox(const std::string& id) const;
    QDialogButtonBox* buttons() const { return buttons_; }

   private:
    void rebuildRows();
    void refreshPreview();
    void syncFromForm();

    viewmodel::RotaryLayoutForm form_;
    PreviewFn previewFn_;
    PreviewSink sink_;
    viewmodel::RotaryLayoutPreview preview_;
    std::map<std::string, QLineEdit*> edits_;
    std::map<std::string, QComboBox*> combos_;
    QLabel* status_{nullptr};
    QLabel* error_{nullptr};
    QDialogButtonBox* buttons_{nullptr};
    QWidget* rows_{nullptr};
    QString fieldMessage_;
    bool building_{false};
};

}  // namespace carvetto::app
