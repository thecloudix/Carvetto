// Self-test of Trace Bitmap (T-0260): real PNG/BMP files through QImageReader, hostile sizes,
// the dialog widgets (preview on a worker thread, cancel), and the undoable result.

#include <QAction>
#include <QColor>
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QImage>
#include <QPainter>
#include <QPushButton>
#include <QTemporaryDir>
#include <cmath>
#include <memory>
#include <string>

#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "main_window.hpp"
#include "trace_dialog.hpp"
#include "trace_image.hpp"

namespace carvetto::app {

namespace {

#define TR_REQUIRE(cond, text)                         \
    do {                                               \
        if (!(cond)) {                                 \
            return std::string("trace bitmap: ") + (text); \
        }                                              \
    } while (false)

bool writeFile(const QString& path, const QByteArray& bytes) {
    QFile f(path);
    return f.open(QIODevice::WriteOnly) && f.write(bytes) == bytes.size();
}

/// A BMP whose header claims `w` x `h` 24-bit pixels but which holds no pixel data.
QByteArray hostileBmp(int w, int h) {
    QByteArray b(54, '\0');
    auto put32 = [&](int off, quint32 v) {
        for (int i = 0; i < 4; ++i) b[off + i] = static_cast<char>((v >> (8 * i)) & 0xFF);
    };
    b[0] = 'B';
    b[1] = 'M';
    put32(2, 54);
    put32(10, 54);
    put32(14, 40);
    put32(18, static_cast<quint32>(w));
    put32(22, static_cast<quint32>(h));
    b[26] = 1;
    b[28] = 24;
    return b;
}

double netArea(const io::TraceLayer& l) {
    double a = 0;
    for (const auto& c : l.contours) a += geometry::signedArea(c);
    return a;
}

}  // namespace

std::string traceSelfTest(MainWindow& window) {
    window.setQuiet(true);
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(300.0, 300.0, 18.0, document::Units::Millimetres)));
    document::Document& doc = window.currentDocument();
    TraceBitmapMenu* menu = window.traceBitmap();
    TR_REQUIRE(menu != nullptr && menu->fileAction() != nullptr && menu->editAction() != nullptr, "the Trace Bitmap menu entries are missing");

    QTemporaryDir dir;
    TR_REQUIRE(dir.isValid(), "no temporary directory");
    // A 120 x 80 picture: a black 60 x 40 block and a red disc on white.
    QImage picture(120, 80, QImage::Format_RGB32);
    picture.fill(Qt::white);
    {
        QPainter p(&picture);
        p.fillRect(10, 20, 60, 40, Qt::black);
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(220, 20, 20));
        p.drawEllipse(QPoint(95, 30), 15, 15);
    }
    const QString png = dir.filePath(QStringLiteral("picture.png"));
    const QString bmp = dir.filePath(QStringLiteral("picture.bmp"));
    TR_REQUIRE(picture.save(png, "PNG") && picture.save(bmp, "BMP"), "could not write the test images");

    // --- Decoding and refusals --------------------------------------------------------------
    for (const QString& path : {png, bmp}) {
        const DecodedImage d = decodeImageFile(path);
        TR_REQUIRE(d.ok && d.image.width == 120 && d.image.height == 80 && d.image.valid(), "a PNG/BMP did not decode to 120 x 80");
        TR_REQUIRE(d.image.rgb[(25 * 120 + 20) * 3] == 0, "the black block is not black after decoding");
    }
    const QString huge = dir.filePath(QStringLiteral("huge.bmp"));
    TR_REQUIRE(writeFile(huge, hostileBmp(20000, 20000)), "could not write the hostile image");
    const DecodedImage refused = decodeImageFile(huge);
    TR_REQUIRE(!refused.ok && !refused.error.isEmpty(), (std::string("an oversize image was not refused with a message: ") + refused.error.toStdString()).c_str());
    const QString wide = dir.filePath(QStringLiteral("wide.bmp"));
    TR_REQUIRE(writeFile(wide, hostileBmp(16384, 1024)), "could not write the wide image");  // 16.7 Mpx > 16 Mpx cap
    TR_REQUIRE(!decodeImageFile(wide).ok, "an image over the pixel cap was not refused");
    const QString junk = dir.filePath(QStringLiteral("junk.png"));
    TR_REQUIRE(writeFile(junk, QByteArray("this is not an image at all")), "could not write the junk file");
    TR_REQUIRE(!decodeImageFile(junk).ok, "a non-image was accepted");
    TR_REQUIRE(!decodeImageFile(dir.filePath(QStringLiteral("missing.png"))).ok, "a missing file was accepted");
    QFile whole(png);
    TR_REQUIRE(whole.open(QIODevice::ReadOnly), "cannot reread the PNG");
    const QByteArray pngBytes = whole.readAll();
    whole.close();
    const QString cut = dir.filePath(QStringLiteral("cut.png"));
    TR_REQUIRE(writeFile(cut, pngBytes.left(pngBytes.size() / 2)), "could not write the truncated PNG");
    TR_REQUIRE(!decodeImageFile(cut).ok, "a truncated PNG was accepted");
    QString message;
    const std::size_t stepsBefore = doc.undoStack().count();
    TR_REQUIRE(!menu->run(huge, &message, false) && !message.isEmpty(), "the command did not report the oversize image");
    TR_REQUIRE(doc.undoStack().count() == stepsBefore, "a refused image changed the document");

    // --- Dialog: preview on a worker thread, then the full trace ------------------------------
    DecodedImage decoded = decodeImageFile(png);
    TR_REQUIRE(decoded.ok, "decoding the test picture failed");
    TraceBitmapDialog dialog(std::move(decoded.image), png, document::Units::Millimetres, &window);
    dialog.show();
    TR_REQUIRE(dialog.waitIdle(), "the preview did not finish");
    TR_REQUIRE(dialog.previewLayerCount() == 1, "the black and white preview has no layer");
    viewmodel::TraceForm form = dialog.form();
    form.sizeBasis = viewmodel::TraceSizeBasis::TargetWidth;
    form.sizeValue = 60.0;  // 120 px -> 60 mm: 0.5 mm per pixel
    form.noiseFilter = 0;
    dialog.setForm(form);
    TR_REQUIRE(dialog.waitIdle() && dialog.previewLayerCount() == 1, "the preview did not follow a setting change");
    // An invalid size disables Trace and says why; nothing runs.
    form.sizeValue = 0.0;
    dialog.setForm(form);
    TR_REQUIRE(!dialog.problemShown().isEmpty() && !dialog.traceNow(), "an invalid size was not reported");
    form.sizeValue = 60.0;
    dialog.setForm(form);
    TR_REQUIRE(dialog.waitIdle(), "the preview did not settle");

    const std::size_t objectsBefore = doc.objectCount();
    const std::size_t layersBefore = doc.layers().size();
    const std::size_t stepsNow = doc.undoStack().count();
    TR_REQUIRE(dialog.traceNow() && dialog.result().ok, "the full trace failed");
    TR_REQUIRE(dialog.result().layers.size() == 1, "black and white did not give one layer");
    const double expected = 60.0 * 40.0 * 0.25 + 0.0;  // the 60x40 px block (red disc is dark in luma too: see below)
    const double got = netArea(dialog.result().layers[0]);
    TR_REQUIRE(got > expected && got < expected * 1.5, "the traced area is not plausible");
    TR_REQUIRE(menu->addResult(dialog, &message, false), "adding the result failed");
    TR_REQUIRE(doc.objectCount() == objectsBefore + 1 && doc.layers().size() == layersBefore + 1, "the result is not one object on one new layer");
    TR_REQUIRE(doc.undoStack().count() == stepsNow + 1, "the trace is not one undo step");
    window.undo();
    TR_REQUIRE(doc.objectCount() == objectsBefore && doc.layers().size() == layersBefore, "undo did not remove the trace");
    window.redo();
    TR_REQUIRE(doc.objectCount() == objectsBefore + 1, "redo did not restore the trace");
    window.undo();

    // --- Colour mode: one layer per colour, one undo step --------------------------------------
    DecodedImage again = decodeImageFile(png);
    TraceBitmapDialog colour(std::move(again.image), png, document::Units::Inches, &window);
    colour.show();
    viewmodel::TraceForm cf = colour.form();
    cf.mode = io::TraceMode::Colour;
    cf.colourCount = 3;
    cf.noiseFilter = 0;
    cf.sizeBasis = viewmodel::TraceSizeBasis::PerPixel;
    cf.sizeValue = 0.02;  // inches per pixel
    colour.setForm(cf);
    TR_REQUIRE(colour.waitIdle() && colour.previewLayerCount() == 3, "the colour preview does not have three layers");
    TR_REQUIRE(colour.traceNow() && colour.result().layers.size() == 3, "the colour trace did not give three layers");
    const std::size_t stepsColour = doc.undoStack().count();
    TR_REQUIRE(menu->addResult(colour, &message, false), "adding the colour result failed");
    TR_REQUIRE(doc.layers().size() == layersBefore + 3 && doc.objectCount() == objectsBefore + 3, "colour mode did not add one layer and object per colour");
    TR_REQUIRE(doc.undoStack().count() == stepsColour + 1, "the colour trace is not one undo step");
    window.undo();
    TR_REQUIRE(doc.layers().size() == layersBefore && doc.objectCount() == objectsBefore, "undo did not remove the colour trace");

    // --- Cancel: the trace stops (or had already finished); the dialog never gets stuck ---------
    DecodedImage third = decodeImageFile(png);
    TraceBitmapDialog stopping(std::move(third.image), png, document::Units::Millimetres, &window);
    stopping.show();
    TR_REQUIRE(stopping.waitIdle(), "the preview did not finish");
    stopping.startTrace();
    stopping.reject();  // the Cancel button while tracing
    TR_REQUIRE(stopping.waitIdle() && !stopping.tracing(), "cancelling left the trace running");
    TR_REQUIRE(stopping.result().ok == (stopping.result().layers.size() > 0), "a cancelled trace left a partial result");
    stopping.reject();  // idle: closes
    TR_REQUIRE(!stopping.isVisible(), "Cancel on an idle dialog did not close it");
    TR_REQUIRE(doc.objectCount() == objectsBefore, "cancelling changed the document");
    dialog.close();
    return {};
}

}  // namespace carvetto::app
