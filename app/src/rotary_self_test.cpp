// Self-test of rotary jobs (T-0319): drives the Job Setup page, the Material Setup page and the
// sheet editor of the real MainWindow offscreen, then saves and reloads the job.

#include "rotary_self_test.hpp"

#include <QAction>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <cmath>
#include <memory>
#include <span>
#include <string>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/job.hpp"
#include "carvetto/viewmodel/double_sided_ops.hpp"
#include "carvetto/viewmodel/layers_sheets.hpp"
#include "carvetto/viewmodel/new_job_form.hpp"
#include "carvetto/viewmodel/rotary_material.hpp"
#include "document_panels.hpp"
#include "main_window.hpp"
#include "new_job_dialog.hpp"
#include "rotary_material_dialog.hpp"

namespace carvetto::app {

namespace {

#define RT_REQUIRE(cond, text)                    \
    do {                                          \
        if (!(cond)) {                            \
            return std::string("rotary: ") + (text); \
        }                                         \
    } while (false)

bool near(double a, double b) {
    return std::abs(a - b) < 1e-6;
}

}  // namespace

std::string rotarySelfTest(MainWindow& window) {
    window.setQuiet(true);
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(200.0, 100.0, 18.0, document::Units::Millimetres)));
    document::Document& doc = window.currentDocument();
    RT_REQUIRE(window.materialSetupAction() != nullptr && !window.materialSetupAction()->isEnabled(),
               "Material Setup is enabled for a flat job");

    // 1. Job Setup rotary page: the controls exist, the type switch picks the cylinder-axis zero.
    NewJobDialog dialog(viewmodel::formFromJob(doc.job()), &window);
    auto* axis = dialog.findChild<QComboBox*>(QStringLiteral("rotaryAxis"));
    auto* flipDesign = dialog.findChild<QCheckBox*>(QStringLiteral("flipDesign"));
    auto* zZero = dialog.findChild<QComboBox*>(QStringLiteral("zZero"));
    auto* length = dialog.findChild<QDoubleSpinBox*>(QStringLiteral("rotaryLength"));
    auto* diameter = dialog.findChild<QDoubleSpinBox*>(QStringLiteral("rotaryDiameter"));
    RT_REQUIRE(axis && flipDesign && zZero && length && diameter, "Job Setup has no rotary controls");
    RT_REQUIRE(!flipDesign->isEnabled() && !length->isEnabled(), "rotary controls are enabled for a flat job");
    viewmodel::NewJobForm form = dialog.form();
    form.type = document::JobType::Rotary;
    form.zZero = document::ZZero::CylinderAxis;
    form.length = 300.0;
    form.diameter = 80.0;
    form.rotaryAxis = document::RotaryAxis::Y;
    form.flipDesign = true;
    dialog.setForm(form);
    RT_REQUIRE(flipDesign->isEnabled() && flipDesign->isChecked() && length->isEnabled() && diameter->isEnabled(),
               "the rotary controls are not editable for a rotary job");
    dialog.accept();
    RT_REQUIRE(dialog.job().has_value() && dialog.job()->type == document::JobType::Rotary &&
                   dialog.job()->flipDesign && dialog.job()->rotaryAxis == document::RotaryAxis::Y,
               "the dialog did not produce the rotary job");
    const std::size_t steps = doc.undoStack().count();
    RT_REQUIRE(viewmodel::applyJobSetup(doc, *dialog.job()).ok && doc.undoStack().count() == steps + 1,
               "Job Setup is not one undo step");
    window.documentChanged();
    const document::Sheet& sheet = doc.sheets()[0];
    RT_REQUIRE(near(sheet.sizeMm.y, 300.0) && near(sheet.sizeMm.x, 3.14159265358979323846 * 80.0) &&
                   near(sheet.sizeMm.z, 80.0) && sheet.zZero == document::ZZero::CylinderAxis,
               "the sheet did not follow the cylinder");
    RT_REQUIRE(window.materialSetupAction()->isEnabled(), "Material Setup is disabled for a rotary job");

    // 2. Material Setup page: diameter, gaps and the read-only model thickness.
    const auto initial = viewmodel::rotaryMaterialFromJob(doc.job());
    RT_REQUIRE(initial.has_value(), "no Material Setup form for a rotary job");
    RotaryMaterialDialog material(*initial, &window);
    auto* gapOutside = material.findChild<QDoubleSpinBox*>(QStringLiteral("gapOutside"));
    auto* gapInside = material.findChild<QDoubleSpinBox*>(QStringLiteral("gapInside"));
    auto* thickness = material.findChild<QLabel*>(QStringLiteral("modelThickness"));
    RT_REQUIRE(gapOutside && gapInside && thickness, "Material Setup has no gap controls");
    gapOutside->setValue(5.0);
    gapInside->setValue(15.0);
    RT_REQUIRE(thickness->text().startsWith(QStringLiteral("20.0000")), "the model thickness did not follow the gaps");
    material.accept();
    RT_REQUIRE(material.result().has_value(), "Material Setup refused valid values");
    RT_REQUIRE(viewmodel::applyRotaryMaterial(doc, *material.result()).ok, "Material Setup could not be applied");
    window.documentChanged();
    RT_REQUIRE(near(doc.job().gapOutsideMm, 5.0) && near(doc.job().modelThicknessMm(), 20.0) &&
                   near(doc.job().rotaryModelTopZ(), 35.0),
               "the job did not take the gaps");
    RotaryMaterialDialog bad(*initial, &window);
    bad.setForm({initial->units, 80.0, 30.0, 20.0, document::ZZero::CylinderAxis});
    bad.accept();
    RT_REQUIRE(!bad.result().has_value(), "Material Setup accepted gaps larger than the radius");

    // 3. Edit Sheet: the length edits the sheet, the circumference and diameter are read-only.
    SheetsPanel* panel = window.sheetsPanel();
    RT_REQUIRE(panel != nullptr, "the Sheets panel is missing");
    panel->refresh();
    RT_REQUIRE(panel->setMaterial(0, "150", "", "", -1, -1), "Edit Sheet refused a rotary length");
    window.documentChanged();
    RT_REQUIRE(near(doc.sheets()[0].sizeMm.y, 150.0) && near(doc.sheets()[0].sizeMm.z, 80.0),
               "Edit Sheet did not set the length alone");
    RT_REQUIRE(!panel->setMaterial(0, "-5", "", "", -1, -1), "Edit Sheet accepted a negative length");

    // 4. Save and reload: every field survives.
    const auto saved = document::saveCarv(doc, document::CarvExtras{});
    RT_REQUIRE(saved.ok(), "the rotary job could not be saved");
    const auto loaded = document::loadCarv(std::span<const std::uint8_t>(saved.value()));
    RT_REQUIRE(loaded.ok(), "the rotary job could not be reloaded");
    RT_REQUIRE(loaded.value().document.job() == doc.job() && loaded.value().document.sheets() == doc.sheets(),
               "the reloaded rotary job differs");
    RT_REQUIRE(loaded.value().document.job().flipDesign && near(loaded.value().document.job().gapInsideMm, 15.0),
               "the reloaded rotary job lost flip design or gaps");
    return {};
}

}  // namespace carvetto::app
