#pragma once

#include <QWidget>
#include <cstddef>
#include <functional>
#include <string>

#include "carvetto/document/document.hpp"
#include "carvetto/document/layer.hpp"
#include "carvetto/viewmodel/properties_model.hpp"

class QComboBox;
class QLineEdit;
class QListWidget;
class QPushButton;
class QTreeWidget;
class QTreeWidgetItem;

namespace carvetto::app {

/// Returns the document the panels edit right now. The panels never keep the pointer: every
/// action asks again, so a document replaced by File > New / Open (or freed between a click and
/// its handler) cannot be reached through a stale reference (T-0210).
using DocumentGetter = std::function<document::Document*()>;

/// What the three panels below share: the getter, and the hooks the main window supplies.
class PanelBase : public QWidget {
   public:
    /// Called after an edit changed the document (the window refreshes every view).
    std::function<void()> onEdited;
    /// Called with a refusal text to show to the user (status bar).
    std::function<void(const std::string&)> onMessage;
    /// Optional: true while history jumps must be refused (a toolpath calculation is running).
    std::function<bool()> historyBlocked;

   protected:
    PanelBase(DocumentGetter getter, QWidget* parent) : QWidget(parent), getter_(std::move(getter)) {}
    document::Document* doc() const { return getter_ ? getter_() : nullptr; }
    /// Reports `r`: the message when refused, `onEdited` when the document changed.
    bool finish(const viewmodel::EditResult& r);

   private:
    DocumentGetter getter_;
};

/// Layers panel: stacking-order list with name (double-click to rename), visibility and lock
/// check boxes and a colour swatch, plus add, delete, reorder, colour and "move selection here".
/// Selecting a row makes that layer the active one (`onPick`). All edits go through
/// `viewmodel/layers_sheets.hpp`.
class LayersPanel : public PanelBase {
   public:
    LayersPanel(DocumentGetter getter, QWidget* parent = nullptr);

    /// Called with the row of the layer the user picked as the active layer.
    std::function<void(int)> onPick;

    void refresh();

    // Actions, used by the buttons (after their dialogs) and by the self-test.
    bool addLayer(const std::string& name = {});
    bool renameLayer(int row, const std::string& name);
    bool setVisible(int row, bool visible);
    bool setLocked(int row, bool locked);
    bool setColour(int row, document::Colour colour);
    bool deleteLayer(int row, bool deleteContents);
    /// Moves the layer at `row` to position `newRow` of the list (the stacking order).
    bool moveLayer(int row, int newRow);
    bool moveSelectionHere(int row);

    int rowCount() const;
    int currentRow() const;
    void setCurrentRow(int row);
    QTreeWidget* tree() const { return tree_; }

   private:
    void askDelete();
    void askColour();
    void itemEdited(QTreeWidgetItem* item, int column);
    document::LayerId idAt(int row) const;

    QTreeWidget* tree_;
    bool refreshing_{false};
};

/// Sheets panel: the sheet list (double-click to rename) with add and delete, and the per-sheet
/// material block (width, height, thickness, Z zero, datum) of the selected sheet.
class SheetsPanel : public PanelBase {
   public:
    SheetsPanel(DocumentGetter getter, QWidget* parent = nullptr);

    /// Called with the row of the sheet the user picked as the active sheet.
    std::function<void(int)> onPick;

    void refresh();

    bool addSheet(const std::string& name = {});
    bool renameSheet(int row, const std::string& name);
    bool deleteSheet(int row);
    /// Material block of the sheet at `row`; sizes as typed (display units, "mm"/"in" suffix
    /// allowed). An empty text keeps that value. `zZero` / `datum` are choice indexes, -1 keeps.
    bool setMaterial(int row, const std::string& width, const std::string& height, const std::string& thickness,
                     int zZero = -1, int datum = -1);

    int rowCount() const;
    int currentRow() const;
    void setCurrentRow(int row);
    QListWidget* list() const { return list_; }

   private:
    void askDelete();
    void applyForm();
    void fillForm();
    document::SheetId idAt(int row) const;

    QListWidget* list_;
    QLineEdit* width_;
    QLineEdit* height_;
    QLineEdit* thickness_;
    QComboBox* zZero_;
    QComboBox* datum_;
    bool refreshing_{false};
};

/// Undo history list: one row per recorded step (the first row is the untouched start); the
/// current state is highlighted, steps after it can be redone. Clicking a row jumps there.
class HistoryPanel : public PanelBase {
   public:
    HistoryPanel(DocumentGetter getter, QWidget* parent = nullptr);

    void refresh();
    /// Jumps to the state after `applied` steps (0 = start).
    bool jumpTo(std::size_t applied);

    int rowCount() const;
    int currentRow() const;
    QListWidget* list() const { return list_; }

   private:
    QListWidget* list_;
    bool refreshing_{false};
};

}  // namespace carvetto::app
