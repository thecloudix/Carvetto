#pragma once

#include <QWidget>
#include <functional>
#include <string>

#include "carvetto/relief/relief.hpp"
#include "carvetto/viewmodel/relief_tree.hpp"
#include "carvetto/viewmodel/relief_modelling.hpp"
#include "document_panels.hpp"

class QCheckBox;
class QComboBox;
class QFormLayout;
class QLabel;
class QLineEdit;
class QPushButton;
class QStackedWidget;

namespace carvetto::app {

class MainWindow;
/// Self-test of the modelling forms (T-0300); empty on success.
std::string modellingSelfTest(MainWindow& window);

/// Docked modelling forms (T-0300, ADR-0080; T-0306, ADR-0083): Create Shape, Bitmap, Zero Plane, Scale Model Z
/// and the modify tools (replace below, reset, offset, smooth, emboss, slice, draft, split, boundary, cross
/// section, distort),
/// each with the common Preview / Apply / Reset / Start New Component buttons. The logic lives in
/// `viewmodel/relief_modelling.hpp`; this widget only reads and writes the text of the forms.
/// Preview shows the result alone in the Model view (`onPreview`), Apply is one undo step; Apply
/// again re-applies to the same component until Start New Component.
class ModellingPanel : public PanelBase {
   public:
    /// `Modify` stands for every page of `viewmodel::ModifyTool` (T-0306); see `modifyTool()`.
    enum class Tool { CreateShape = 0, Bitmap = 1, ZeroPlane = 2, ScaleZ = 3, Modify = 4 };

    explicit ModellingPanel(DocumentGetter getter, QWidget* parent = nullptr);

    /// Called with the previewed field, or null when the preview is cleared.
    std::function<void(const relief::HeightField*)> onPreview;
    /// Decodes PNG/JPEG/... for the bitmap form (the app supplies QImage decoding).
    document::BitmapDecoder decoder;

    void setTool(Tool tool);
    Tool tool() const;
    void setModifyTool(viewmodel::ModifyTool tool);
    viewmodel::ModifyTool modifyTool() const;

    bool preview();      ///< Builds the result of the current form and shows it; false with a message.
    bool apply();        ///< The Apply button.
    void reset();        ///< The Reset button: the form returns to its defaults, the preview is cleared.
    void startNew();     ///< The Start New Component button.
    /// Fills the bitmap form from the selected bitmap object.
    void useSelectedBitmap();
    void clearPreview();

    QLineEdit* field(const QString& label) const;
    QComboBox* combo(const QString& label) const;
    QCheckBox* check(const QString& label) const;
    QLabel* status() const { return status_; }
    viewmodel::ModellingSession& session() { return session_; }

   private:
    QWidget* buildShapePage();
    QWidget* buildBitmapPage();
    QWidget* buildZeroPlanePage();
    QWidget* buildScaleZPage();
    QWidget* buildModifyPage(viewmodel::ModifyTool tool);
    void writeModify(const viewmodel::ModifyForm& f);
    viewmodel::ModifyForm readModify() const;
    void writeShape(const viewmodel::CreateShapeForm& f);
    void writeBitmap(const viewmodel::BitmapReliefForm& f);
    void writeZeroPlane(const viewmodel::ZeroPlaneForm& f);
    void writeScaleZ(const viewmodel::ScaleZForm& f);
    viewmodel::CreateShapeForm readShape() const;
    viewmodel::BitmapReliefForm readBitmap() const;
    viewmodel::ZeroPlaneForm readZeroPlane() const;
    viewmodel::ScaleZForm readScaleZ() const;
    void message(const std::string& text, bool error);
    QLineEdit* addEdit(QFormLayout* form, const QString& prefix, const QString& label, const char* tip);

    QComboBox* toolBox_;
    QStackedWidget* stack_;
    QLabel* status_;
    viewmodel::ModifyResult lastModify_;   ///< The previewed modify result (kept alive for the shown field).
    QPushButton* previewButton_;
    QPushButton* applyButton_;
    QPushButton* resetButton_;
    QPushButton* newButton_;
    viewmodel::ModellingSession session_;
};

}  // namespace carvetto::app
