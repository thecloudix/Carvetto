#pragma once

#include <QDialog>
#include <QString>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/units.hpp"
#include "carvetto/viewmodel/data_merge.hpp"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QSpinBox;

namespace carvetto::app {

class MainWindow;

/// Plate Production (data merge) form (T-0347). The selected objects are the template; CSV rows
/// become plates. Thin glue over `viewmodel/data_merge.hpp`: the preview line and the problem
/// list come from `previewDataMerge`; OK stays open while the options or the data are refused.
class PlateProductionDialog : public QDialog {
   public:
    PlateProductionDialog(const document::Document& doc, QWidget* parent = nullptr);

    QString csvText() const;
    void setCsvText(const QString& text);
    /// Options from the widgets (lengths converted from display units to millimetres).
    viewmodel::DataMergeOptions options() const;
    /// Re-reads the headers and placeholders, re-runs the dry run and updates the summary.
    void refreshPreview();
    /// Summary line of the last dry run.
    QString summary() const;
    /// The problem list of the last dry run (one `line N: ...` per row).
    QString problems() const;
    /// Number of placeholder rows in the variable assignment table.
    int assignmentRows() const { return static_cast<int>(assignmentBoxes_.size()); }
    void setSeparator(char separator);
    void setSkipBadRows(bool skip);
    void accept() override;

   private:
    void importFile();
    void rebuildAssignments();

    const document::Document& doc_;
    document::Units units_;
    QPlainTextEdit* csv_{nullptr};
    QComboBox* separator_{nullptr};
    QCheckBox* firstRowNames_{nullptr};
    QFormLayout* assignmentForm_{nullptr};
    std::vector<std::string> placeholders_;
    std::vector<QComboBox*> assignmentBoxes_;
    std::vector<QLabel*> assignmentLabels_;
    QCheckBox* counterEnabled_{nullptr};
    QLineEdit* counterName_{nullptr};
    QDoubleSpinBox* counterStart_{nullptr};
    QDoubleSpinBox* counterStep_{nullptr};
    QSpinBox* counterWidth_{nullptr};
    QLineEdit* counterPrefix_{nullptr};
    QLineEdit* counterSuffix_{nullptr};
    QComboBox* layout_{nullptr};
    QDoubleSpinBox* marginLeft_{nullptr};
    QDoubleSpinBox* marginRight_{nullptr};
    QDoubleSpinBox* marginTop_{nullptr};
    QDoubleSpinBox* marginBottom_{nullptr};
    QDoubleSpinBox* spacing_{nullptr};
    QCheckBox* newLayer_{nullptr};
    QCheckBox* removeTemplate_{nullptr};
    QCheckBox* skipBadRows_{nullptr};
    QLabel* summary_{nullptr};
    QPlainTextEdit* problems_{nullptr};
};

/// Runs the merge on the window's document, refreshes the window and (when `showErrors`) shows the
/// refusal in a message box. Returns the viewmodel's result.
viewmodel::DataMergeResult runPlateProduction(MainWindow& window, const std::string& csvText,
                                              const viewmodel::DataMergeOptions& options, bool showErrors = true);

/// Opens the dialog for the window's document and applies the merge when accepted.
void showPlateProduction(MainWindow& window);

}  // namespace carvetto::app
