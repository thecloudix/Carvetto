#pragma once

#include <QString>
#include <string>

class QAction;
class QMenu;

namespace carvetto::app {

class MainWindow;

/// The Edit > Double-sided submenu (T-0261, ADR-0039): Swap Sides, Top Side (1), Bottom Side (2),
/// Multi-Sided View (=), Copy / Move to Other Side for the selected objects and for the current
/// toolpath, and Single-Sided Job (switch back). All logic is `viewmodel/double_sided_ops.hpp` and
/// `layers_sheets.hpp`; this class wires actions and refreshes the window after each edit.
class DoubleSidedMenu {
   public:
    DoubleSidedMenu(MainWindow& window, QMenu* edit);

    /// Enables the actions for the open document (double-sided or not). Called by the window after
    /// every document change.
    void refresh();

    QAction* swapSidesAction() const { return swap_; }
    QAction* topSideAction() const { return top_; }
    QAction* bottomSideAction() const { return bottom_; }
    QAction* multiSidedAction() const { return multi_; }
    QAction* copyObjectsAction() const { return copyObjects_; }
    QAction* moveObjectsAction() const { return moveObjects_; }
    QAction* copyToolpathAction() const { return copyToolpath_; }
    QAction* moveToolpathAction() const { return moveToolpath_; }
    QAction* singleSidedAction() const { return single_; }
    /// What the last action said (also shown in the status bar).
    const QString& lastMessage() const { return lastMessage_; }

   private:
    void afterEdit(bool changed, const std::string& message);
    void toolpathToOtherSide(bool move);

    MainWindow& window_;
    QAction* swap_{nullptr};
    QAction* top_{nullptr};
    QAction* bottom_{nullptr};
    QAction* multi_{nullptr};
    QAction* copyObjects_{nullptr};
    QAction* moveObjects_{nullptr};
    QAction* copyToolpath_{nullptr};
    QAction* moveToolpath_{nullptr};
    QAction* single_{nullptr};
    QString lastMessage_;
};

/// Self-test of double-sided jobs (T-0261): Job Setup, side actions, multi-sided view, copy to the
/// other side, per-side Save Toolpaths and a .carv round trip. Empty on success, else the failure.
std::string doubleSidedSelfTest(MainWindow& window);

}  // namespace carvetto::app
