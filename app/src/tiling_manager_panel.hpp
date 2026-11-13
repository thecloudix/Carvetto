#pragma once

#include <QWidget>
#include <functional>
#include <string>

#include "carvetto/viewmodel/tiling_form.hpp"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QTreeWidget;

namespace carvetto::app {

class TilePreviewWidget;

/// The Tiling Manager (T-0266, ADR-0043): the "Tile Toolpaths" check box of the Save Toolpaths
/// dialog with its settings (strategy, tile size, overlap, order, original position), a list of
/// the tiles and a drawing of the tiles over the job (saved tiles green, tiles larger than the
/// machine work area red, empty tiles grey). Only hosts widgets: the form, the preview and all
/// checks are `viewmodel/tiling_form.hpp`; the dialog feeds the preview back in.
class TilingManagerPanel : public QWidget {
   public:
    explicit TilingManagerPanel(QWidget* parent = nullptr);

    /// Called after every user change of the form (immediately for check box and combos).
    std::function<void()> onChanged;
    /// Called after a change of a number field (the owner may debounce).
    std::function<void()> onTyped;

    viewmodel::TilingForm form() const;
    void setForm(const viewmodel::TilingForm& form);
    /// Ticks or clears "Tile Toolpaths" and shows or hides the settings.
    void setTilingOn(bool on);
    bool tilingOn() const;
    /// The machine's work area: the tile size follows it until the user edits a size.
    void setMachineWorkArea(double x, double y);
    /// Shows `preview` (tile list, drawing, status line).
    void showPreview(const viewmodel::TilePreview& preview);

    // --- State the self-test reads ---------------------------------------------------------
    int tileRows() const;
    std::string statusShown() const;
    bool settingsShown() const;
    const viewmodel::TilePreview& shownPreview() const { return preview_; }

   private:
    void changed(bool typed);
    void updateEnabledFields();

    QCheckBox* tile_{nullptr};
    QWidget* settings_{nullptr};
    QComboBox* strategy_{nullptr};
    QDoubleSpinBox* width_{nullptr};
    QDoubleSpinBox* height_{nullptr};
    QDoubleSpinBox* overlap_{nullptr};
    QCheckBox* smallestFirst_{nullptr};
    QCheckBox* originalPosition_{nullptr};
    QPushButton* useWorkArea_{nullptr};
    QLabel* status_{nullptr};
    QTreeWidget* list_{nullptr};
    TilePreviewWidget* drawing_{nullptr};

    viewmodel::TilePreview preview_;
    double areaX_{0.0};
    double areaY_{0.0};
    bool sizeEdited_{false};
    bool updating_{false};
};

}  // namespace carvetto::app
