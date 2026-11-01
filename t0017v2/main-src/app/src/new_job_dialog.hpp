#pragma once

#include <QDialog>
#include <optional>

#include "carvetto/document/job.hpp"
#include "carvetto/document/units.hpp"
#include "carvetto/viewmodel/new_job_form.hpp"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;

namespace carvetto::app {

/// File > New job form: job type, size, units, Z-zero and XY datum. Validation and the mapping
/// to a `document::Job` are done by `viewmodel::validateNewJob`; OK stays open while it reports
/// errors.
class NewJobDialog : public QDialog {
   public:
    explicit NewJobDialog(const viewmodel::NewJobForm& initial, QWidget* parent = nullptr);

    /// The accepted job (set once the dialog closed with OK).
    const std::optional<document::Job>& job() const { return job_; }
    /// Current form contents.
    viewmodel::NewJobForm form() const;
    /// Replaces the form contents.
    void setForm(const viewmodel::NewJobForm& form);

    void accept() override;

   private:
    void updateEnabledFields();
    void unitsChanged();

    QComboBox* type_{nullptr};
    QComboBox* units_{nullptr};
    QDoubleSpinBox* width_{nullptr};
    QDoubleSpinBox* height_{nullptr};
    QDoubleSpinBox* thickness_{nullptr};
    QDoubleSpinBox* length_{nullptr};
    QDoubleSpinBox* diameter_{nullptr};
    QComboBox* rotaryAxis_{nullptr};
    QComboBox* flip_{nullptr};
    QComboBox* zZero_{nullptr};
    QComboBox* datum_{nullptr};
    QCheckBox* useOffset_{nullptr};
    QDoubleSpinBox* offsetX_{nullptr};
    QDoubleSpinBox* offsetY_{nullptr};
    QLabel* errors_{nullptr};

    document::Units shownUnits_{document::Units::Millimetres};
    bool updating_{false};
    std::optional<document::Job> job_;
};

}  // namespace carvetto::app
