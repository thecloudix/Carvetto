#pragma once

#include <map>
#include <string>
#include <vector>

#include "document_panels.hpp"

class QFormLayout;
class QLabel;
class QWidget;

namespace carvetto::app {

/// Properties of the selection (T-0210): name, layer, position, size, rotation and the parameters
/// of the selected shape or text. A thin form over `viewmodel::describeSelection()` /
/// `setProperty()`: the fields, validation and undo steps all come from the viewmodel. A value is
/// applied when the entry is committed (Enter or leaving the field); a refused one is marked red,
/// the reason is shown and the old value comes back. Nothing here keeps a document pointer.
class PropertiesPanel : public PanelBase {
   public:
    PropertiesPanel(DocumentGetter getter, QWidget* parent = nullptr);

    /// Rebuilds the form when the selection's field set changed, otherwise only updates values
    /// (never the one being typed into).
    void refresh();
    /// Test hook: enters `text` into field `id` as if typed and committed. False when refused.
    bool commit(const std::string& id, const std::string& text);
    /// Ids of the fields currently shown, in order.
    std::vector<std::string> fieldIds() const;
    /// Text currently shown in a field ("" when absent).
    std::string textOf(const std::string& id) const;
    const std::string& title() const { return titleText_; }

   private:
    struct Row {
        std::string id;
        QWidget* editor{nullptr};  ///< The widget placed in the form (may hold an Apply button).
        QWidget* input{nullptr};   ///< The line edit, text edit or combo box the user types into.
        std::string shown;         ///< The value last shown (to skip a commit that changes nothing).
        viewmodel::PropertyKind kind{viewmodel::PropertyKind::Text};
    };
    void rebuild(const viewmodel::PropertyView& view);
    void clearRows();
    /// Applies the entry of field `id`; true when it was applied or changed nothing.
    bool commitRow(const std::string& id);
    void setStatus(const std::string& text);

    QLabel* title_;
    QLabel* status_;
    QFormLayout* form_;
    std::vector<Row> rows_;
    std::string titleText_;
    bool updating_{false};
};

class MainWindow;

/// Drives the real window through the Properties, Layers, Sheets and History panels (T-0210).
/// Empty on success, otherwise the failure.
std::string panelsSelfTest(MainWindow& window);

}  // namespace carvetto::app
