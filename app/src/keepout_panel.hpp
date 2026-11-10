#pragma once

#include <QWidget>
#include <functional>
#include <string>

#include "document_panels.hpp"

class QAction;
class QCheckBox;
class QDoubleSpinBox;
class QLabel;
class QMenu;
class QPushButton;

namespace carvetto::app {

class MainWindow;

/// Keep-out zones (T-0256): the "Keep-out Zones" dock (clearance, show/hide, Create from Selection,
/// Clear Zones, a status line with the routing result of the preview) and the Edit > Keep-out
/// Zones submenu with the same actions. Thin glue over `viewmodel/keepout.hpp`: each edit is one
/// undo step through the T-0241 commands.
class KeepOutPanel : public PanelBase {
   public:
    explicit KeepOutPanel(DocumentGetter getter, QWidget* parent = nullptr);

    /// Adds the Edit submenu (the actions below) to `edit`.
    void buildMenu(QMenu* edit);
    /// Re-reads the document: field values, enabled state of the actions, status line.
    void refresh();
    /// Text shown under the status line (routed / unroutable counts); set by the window.
    std::function<std::string()> routingText;

    // The actions (used by the menu, the buttons and the self-test).
    bool createFromSelection();
    bool clearZones();
    /// Display-unit value, as typed in the field.
    bool setClearance(double mm);
    void setShown(bool shown);

    QAction* createAction() const { return create_; }
    QAction* clearAction() const { return clear_; }
    QAction* showAction() const { return show_; }
    QDoubleSpinBox* clearanceField() const { return clearance_; }

   private:
    QAction* create_{nullptr};
    QAction* clear_{nullptr};
    QAction* show_{nullptr};
    QDoubleSpinBox* clearance_{nullptr};
    QLabel* status_{nullptr};
    bool refreshing_{false};
};

/// Drives the keep-out UI of `window` like a user (T-0256); returns an error text or empty.
std::string keepOutSelfTest(MainWindow& window);

}  // namespace carvetto::app
