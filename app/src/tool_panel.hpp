#pragma once

#include <QWidget>
#include <map>
#include <string>
#include <vector>

#include "carvetto/viewmodel/tools.hpp"

class QCheckBox;
class QComboBox;
class QFormLayout;
class QLabel;
class QLineEdit;
class QSpinBox;

namespace carvetto::app {

class CanvasWidget;

/// Numeric entry and options of the active drawing tool (T-0204). A thin form over
/// `viewmodel::ToolController`: the labels, validation and values all come from the controller
/// (`fields()`, `setField()`); this widget only hosts line edits. Typing a value overrides the
/// pointer for that field; an invalid entry is marked red and changes nothing; Enter in a field
/// completes the shape.
class ToolPanel : public QWidget {
   public:
    ToolPanel(CanvasWidget* canvas, QWidget* parent = nullptr);

    /// Rebuilds the form for the controller's current tool (call after switching tools).
    void rebuild();
    /// Updates the shown pointer-derived values without disturbing what the user is typing.
    void refreshValues();
    /// Test hook: enters `text` into the field `id` as if typed. Returns the edit's validity.
    bool typeInto(const std::string& id, const std::string& text);

   private:
    void addOptionRows(viewmodel::ToolKind tool);
    void addTextRows();
    void addDimensionRows();
    void clearForm();

    CanvasWidget* canvas_;
    QFormLayout* form_;
    QLabel* title_;
    QLabel* status_;
    std::map<std::string, QLineEdit*> edits_;
};

}  // namespace carvetto::app
