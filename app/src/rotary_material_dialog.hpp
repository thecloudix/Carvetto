#pragma once

#include <QDialog>
#include <optional>

#include "carvetto/viewmodel/rotary_material.hpp"

class QComboBox;
class QDoubleSpinBox;
class QLabel;

namespace carvetto::app {

/// Material Setup for a rotary job: diameter, Gap Outside / Gap Inside Model, Z-zero (cylinder surface
/// or axis) and the read-only model thickness. Validation is `viewmodel::validateRotaryMaterial`; OK
/// stays open while it reports errors (T-0319).
class RotaryMaterialDialog : public QDialog {
   public:
    explicit RotaryMaterialDialog(const viewmodel::RotaryMaterialForm& initial, QWidget* parent = nullptr);

    /// The accepted form (set once the dialog closed with OK).
    const std::optional<viewmodel::RotaryMaterialForm>& result() const { return result_; }
    viewmodel::RotaryMaterialForm form() const;
    void setForm(const viewmodel::RotaryMaterialForm& form);

    void accept() override;

   private:
    void updateThickness();

    QDoubleSpinBox* diameter_{nullptr};
    QDoubleSpinBox* gapOutside_{nullptr};
    QDoubleSpinBox* gapInside_{nullptr};
    QComboBox* zZero_{nullptr};
    QLabel* thickness_{nullptr};
    QLabel* errors_{nullptr};
    document::Units units_{document::Units::Millimetres};
    std::optional<viewmodel::RotaryMaterialForm> result_;
};

}  // namespace carvetto::app
