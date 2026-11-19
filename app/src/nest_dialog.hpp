#pragma once

#include <QDialog>

#include "carvetto/document/document.hpp"
#include "carvetto/document/units.hpp"
#include "carvetto/viewmodel/nest_selection.hpp"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QSpinBox;

namespace carvetto::app {

/// Modal form for Nest Selected Vectors (T-0337). Lengths are entered in display units and stored
/// in millimetres, angles in degrees and stored in radians. OK stays open while the view model's
/// validation fails. The boundary layer combo lists the document's layers.
class NestDialog : public QDialog {
   public:
    NestDialog(const document::Document& doc, const viewmodel::NestSelectionOptions& initial,
               QWidget* parent = nullptr);
    viewmodel::NestSelectionOptions options() const;
    void setOptions(const viewmodel::NestSelectionOptions& options);
    void accept() override;

   private:
    void updateEnabled();

    document::Units units_;
    QComboBox* area_{nullptr};
    QDoubleSpinBox* sheetLength_{nullptr};
    QDoubleSpinBox* sheetWidth_{nullptr};
    QComboBox* boundaryLayer_{nullptr};
    QDoubleSpinBox* toolDia_{nullptr};
    QDoubleSpinBox* clearance_{nullptr};
    QDoubleSpinBox* border_{nullptr};
    QDoubleSpinBox* rotation_{nullptr};
    QCheckBox* mirror_{nullptr};
    QCheckBox* inside_{nullptr};
    QComboBox* corner_{nullptr};
    QComboBox* direction_{nullptr};
    QSpinBox* copies_{nullptr};
    QCheckBox* remove_{nullptr};
};

}  // namespace carvetto::app
