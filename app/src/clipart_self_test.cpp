// Self-test of the clipart browser (T-0333): drives the real MainWindow offscreen through the Library
// dock like a user: add a folder of mixed files, thumbnails from the worker thread, search, favourites,
// insert (one undo step each) and a drop on the canvas; hostile files are refused with messages.

#include <QCoreApplication>
#include <QDockWidget>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QImage>
#include <QMimeData>
#include <QTemporaryDir>
#include <QUrl>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>

#include "canvas_widget.hpp"
#include "carvetto/document/document.hpp"
#include "clipart_panel.hpp"
#include "main_window.hpp"

namespace carvetto::app {

namespace {

#define CL_REQUIRE(cond, text)                          \
    do {                                                \
        if (!(cond)) {                                  \
            return std::string("clipart: ") + (text);   \
        }                                               \
    } while (false)

bool writeFile(const QString& path, const std::string& content) {
    std::ofstream out(path.toStdString(), std::ios::binary);
    out.write(content.data(), static_cast<std::streamsize>(content.size()));
    return out.good();
}

int rowOf(ClipartPanel& panel, const QString& fileName) {
    for (int row = 0; row < panel.rowCount(); ++row) {
        if (QString::fromStdString(panel.pathOfRow(row)).endsWith(fileName)) {
            return row;
        }
    }
    return -1;
}

}  // namespace

std::string clipartSelfTest(MainWindow& window) {
    window.setQuiet(true);
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(300.0, 300.0, 18.0, document::Units::Millimetres)));
    document::Document& doc = window.currentDocument();
    ClipartPanel* panel = window.clipartPanel();
    CL_REQUIRE(panel != nullptr && window.libraryDock() != nullptr && window.libraryDock()->widget() == panel,
               "the Library dock does not hold the clipart browser");

    QTemporaryDir dir;
    CL_REQUIRE(dir.isValid(), "could not create a temporary directory");
    const QString root = dir.path();
    const std::string svg =
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"20mm\" height=\"10mm\" viewBox=\"0 0 20 10\">"
        "<rect x=\"0\" y=\"0\" width=\"20\" height=\"10\"/><circle cx=\"10\" cy=\"5\" r=\"3\"/></svg>";
    const std::string dxf = "0\nSECTION\n2\nENTITIES\n0\nLINE\n8\nSTAR\n10\n0\n20\n0\n11\n10\n21\n10\n"
                            "0\nLINE\n8\nSTAR\n10\n10\n20\n10\n11\n20\n21\n0\n0\nENDSEC\n0\nEOF\n";
    QImage png(60, 30, QImage::Format_RGB32);
    png.fill(QColor(200, 40, 40));
    CL_REQUIRE(writeFile(root + QStringLiteral("/fish.svg"), svg), "could not write the SVG");
    CL_REQUIRE(writeFile(root + QStringLiteral("/star.dxf"), dxf), "could not write the DXF");
    CL_REQUIRE(png.save(root + QStringLiteral("/pic.png")), "could not write the PNG");
    CL_REQUIRE(writeFile(root + QStringLiteral("/notes.txt"), "not clipart"), "could not write the note");
    CL_REQUIRE(QDir().mkpath(root + QStringLiteral("/sub")), "could not create the sub-folder");
    CL_REQUIRE(writeFile(root + QStringLiteral("/sub/bomb.svg"), std::string("PK\x03\x04 pretend archive")),
               "could not write the hostile file");

    // 1. Browsing a folder of mixed files lists the clipart files and nothing else.
    CL_REQUIRE(panel->addFolder(root.toStdString()), "adding the folder was refused");
    CL_REQUIRE(panel->rowCount() == 4, "the folder should list 4 clipart files");
    CL_REQUIRE(rowOf(*panel, QStringLiteral("notes.txt")) < 0, "a text file is listed as clipart");
    CL_REQUIRE(!panel->addFolder(root.toStdString()), "adding the same folder twice was accepted");

    // 2. Thumbnails come from the worker thread, are cached, and a hostile file only gets a message.
    panel->pumpThumbnails(true);
    panel->waitThumbnails();
    const int svgRow = rowOf(*panel, QStringLiteral("fish.svg"));
    const int dxfRow = rowOf(*panel, QStringLiteral("star.dxf"));
    const int pngRow = rowOf(*panel, QStringLiteral("pic.png"));
    const int bombRow = rowOf(*panel, QStringLiteral("bomb.svg"));
    CL_REQUIRE(svgRow >= 0 && dxfRow >= 0 && pngRow >= 0 && bombRow >= 0, "an expected file is missing from the list");
    CL_REQUIRE(panel->rowHasThumbnail(svgRow) && panel->rowHasThumbnail(dxfRow) && panel->rowHasThumbnail(pngRow),
               "a good file has no thumbnail");
    CL_REQUIRE(!panel->rowHasThumbnail(bombRow) && panel->rowToolTip(bombRow).find("zip") != std::string::npos,
               "the hostile file was not refused with a message");
    CL_REQUIRE(panel->thumbnails().processed() == 4, "the worker did not process the four files");
    CL_REQUIRE(panel->thumbnails().cacheBytes() > 0 && panel->thumbnails().cacheBytes() <= 32U * 1024U * 1024U,
               "the thumbnail cache is empty or over its bound");
    CL_REQUIRE(panel->rowToolTip(svgRow).find("20.0 x 10.0 mm") != std::string::npos, "the SVG tooltip lacks its size");

    // 3. Search and favourites.
    panel->setSearch("FISH");
    CL_REQUIRE(panel->rowCount() == 1 && panel->pathOfRow(0).find("fish.svg") != std::string::npos,
               "searching for 'fish' should leave only fish.svg");
    panel->setSearch("");
    CL_REQUIRE(panel->rowCount() == 4, "clearing the search should list all files");
    CL_REQUIRE(panel->toggleFavouriteRow(rowOf(*panel, QStringLiteral("star.dxf"))), "marking a favourite failed");
    panel->setFavouritesOnly(true);
    CL_REQUIRE(panel->rowCount() == 1 && panel->pathOfRow(0).find("star.dxf") != std::string::npos,
               "the favourites filter should show only star.dxf");
    panel->setFavouritesOnly(false);
    CL_REQUIRE(panel->rowCount() == 4, "leaving the favourites filter should list all files");

    // 4. Insert: one undo step each, scaled to fit and centred on the material.
    const auto insertAndCheck = [&](const QString& name, std::size_t expectBitmaps) -> std::string {
        const std::size_t steps = doc.undoStack().index();
        const std::size_t objects = doc.objectCount();
        const viewmodel::ClipartInsertOutcome r = panel->insertRow(rowOf(*panel, name));
        if (!r.ok) {
            return "inserting " + name.toStdString() + " failed: " + r.error;
        }
        if (doc.undoStack().index() != steps + 1 || doc.undoStack().undoText() != "Insert clipart") {
            return "inserting " + name.toStdString() + " is not one undo step";
        }
        if (doc.objectCount() != objects + r.objectsAdded || doc.bitmaps().size() != expectBitmaps) {
            return "inserting " + name.toStdString() + " added the wrong objects";
        }
        if (std::fabs(r.placedBox.centre().x - 150.0) > 1e-6 || std::fabs(r.placedBox.centre().y - 150.0) > 1e-6 ||
            std::fabs(std::max(r.placedBox.width(), r.placedBox.height()) - 150.0) > 1e-6) {
            return name.toStdString() + " was not scaled to fit and centred";
        }
        if (!doc.undo() || doc.objectCount() != objects || doc.undoStack().index() != steps) {
            return "undoing the insert of " + name.toStdString() + " did not remove it in one step";
        }
        return {};
    };
    for (const auto& [name, bitmaps] : {std::pair<QString, std::size_t>{QStringLiteral("fish.svg"), 0},
                                        std::pair<QString, std::size_t>{QStringLiteral("star.dxf"), 0},
                                        std::pair<QString, std::size_t>{QStringLiteral("pic.png"), 1}}) {
        if (const std::string why = insertAndCheck(name, bitmaps); !why.empty()) {
            return "clipart: " + why;
        }
    }
    CL_REQUIRE(doc.objectCount() == 0 && doc.bitmaps().empty(), "the inserts were not all undone");

    // 5. A hostile file is refused with a clear message and the document is untouched.
    const std::size_t steps = doc.undoStack().index();
    const viewmodel::ClipartInsertOutcome refused = panel->insertRow(bombRow);
    CL_REQUIRE(!refused.ok && refused.error.find("zip") != std::string::npos, "the archive was not refused by name");
    CL_REQUIRE(doc.undoStack().index() == steps && doc.objectCount() == 0, "a refused insert changed the document");

    // 6. A file dropped on the canvas lands at the drop point.
    CanvasWidget* canvas = window.canvas();
    const geometry::Vec2 px = canvas->view().worldToScreen(geometry::Vec2{60.0, 220.0});
    QMimeData mime;
    mime.setUrls({QUrl::fromLocalFile(root + QStringLiteral("/fish.svg"))});
    QDragEnterEvent enter(QPoint(static_cast<int>(px.x), static_cast<int>(px.y)), Qt::CopyAction, &mime, Qt::LeftButton,
                          Qt::NoModifier);
    QCoreApplication::sendEvent(canvas, &enter);
    CL_REQUIRE(enter.isAccepted(), "the canvas does not accept a dragged clipart file");
    QDropEvent drop(QPointF(px.x, px.y), Qt::CopyAction, &mime, Qt::LeftButton, Qt::NoModifier);
    const std::size_t before = doc.undoStack().index();
    QCoreApplication::sendEvent(canvas, &drop);
    CL_REQUIRE(drop.isAccepted() && doc.undoStack().index() == before + 1 && doc.objectCount() == 2,
               "dropping the file on the canvas did not insert it as one undo step (accepted " +
                   std::to_string(drop.isAccepted()) + ", steps " + std::to_string(doc.undoStack().index() - before) +
                   ", objects " + std::to_string(doc.objectCount()) + ")");
    const geometry::Vec2 expected = canvas->view().screenToWorld(px);
    const auto bounds = doc.objectBounds(doc.objectIds().back());
    CL_REQUIRE(bounds.has_value(), "the dropped clipart has no bounds");
    geometry::Box2 all;
    for (const document::ObjectId id : doc.objectIds()) {
        all.expand(*doc.objectBounds(id));
    }
    CL_REQUIRE(std::fabs(all.centre().x - expected.x) < 1e-6 && std::fabs(all.centre().y - expected.y) < 1e-6,
               "the dropped clipart is not centred on the drop point");
    // A dropped file of another type is ignored.
    QMimeData other;
    other.setUrls({QUrl::fromLocalFile(root + QStringLiteral("/notes.txt"))});
    QDropEvent ignored(QPointF(px.x, px.y), Qt::CopyAction, &other, Qt::LeftButton, Qt::NoModifier);
    QCoreApplication::sendEvent(canvas, &ignored);
    CL_REQUIRE(doc.undoStack().index() == before + 1, "a dropped text file changed the document");
    CL_REQUIRE(doc.undo() && doc.objectCount() == 0, "undoing the drop did not remove the clipart");

    // 7. Removing the folder empties the list.
    CL_REQUIRE(panel->removeFolder(root.toStdString()) && panel->rowCount() == 0, "removing the folder did not empty the list");
    return {};
}

}  // namespace carvetto::app
