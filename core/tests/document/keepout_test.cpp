// Keep-out zones in the document (T-0241): create from selection, undo/redo, persistence.

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "carv_zip.hpp"
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/keepout_commands.hpp"
#include "carvetto/geometry/contour.hpp"

using namespace carvetto::document;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::Vec2;

namespace {

Contour rect(double x0, double y0, double x1, double y1) {
    return Contour::rectangle(Box2{Vec2{x0, y0}, Vec2{x1, y1}});
}

ObjectId addVector(Document& doc, std::vector<Contour> contours) {
    auto cmd = std::make_unique<AddObjectCommand>(Object::vector(std::move(contours), doc.activeLayer(), doc.activeSheet()));
    AddObjectCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    return raw->id();
}

}  // namespace

TEST_CASE("create from selection uses closed vectors only and is not linked", "[keepout][document]") {
    Document doc;
    Contour open = rect(50, 50, 60, 60);
    open.closed = false;
    const ObjectId a = addVector(doc, {rect(0, 0, 10, 10), open});
    const ObjectId b = addVector(doc, {Contour::circle({30, 30}, 5)});
    auto cmd = std::make_unique<CreateKeepOutZonesCommand>(doc, std::vector<ObjectId>{a, b});
    const auto* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    REQUIRE(doc.keepOutZones().size() == 2);
    CHECK(raw->createdIds().size() == 2);
    CHECK(doc.keepOutZones()[0].outline == rect(0, 0, 10, 10));

    // Not linked: deleting the source vectors leaves the zones.
    REQUIRE(doc.execute(std::make_unique<RemoveObjectsCommand>(std::vector<ObjectId>{a, b})));
    CHECK(doc.keepOutZones().size() == 2);
}

TEST_CASE("create with nothing usable is a no-op", "[keepout][document]") {
    Document doc;
    Contour open = rect(0, 0, 10, 10);
    open.closed = false;
    const ObjectId a = addVector(doc, {open});
    CHECK_FALSE(doc.execute(std::make_unique<CreateKeepOutZonesCommand>(doc, std::vector<ObjectId>{a})));
    CHECK_FALSE(doc.execute(std::make_unique<CreateKeepOutZonesCommand>(std::vector<Contour>{})));
    CHECK(doc.keepOutZones().empty());
}

TEST_CASE("create, delete and clear undo and redo exactly", "[keepout][document]") {
    Document doc;
    REQUIRE(doc.execute(std::make_unique<CreateKeepOutZonesCommand>(
        std::vector<Contour>{rect(0, 0, 1, 1), rect(2, 2, 3, 3), rect(4, 4, 5, 5)})));
    const std::vector<KeepOutZone> three = doc.keepOutZones();
    REQUIRE(three.size() == 3);

    REQUIRE(doc.execute(std::make_unique<RemoveKeepOutZonesCommand>(std::vector<KeepOutZoneId>{three[1].id})));
    CHECK(doc.keepOutZones().size() == 2);
    CHECK_FALSE(doc.findKeepOutZone(three[1].id));
    REQUIRE(doc.undo());
    CHECK(doc.keepOutZones() == three);  // same ids, same order
    REQUIRE(doc.redo());
    CHECK(doc.keepOutZones().size() == 2);
    REQUIRE(doc.undo());

    auto clear = RemoveKeepOutZonesCommand::all();
    REQUIRE(doc.execute(std::make_unique<RemoveKeepOutZonesCommand>(std::move(clear))));
    CHECK(doc.keepOutZones().empty());
    CHECK_FALSE(doc.execute(std::make_unique<RemoveKeepOutZonesCommand>(RemoveKeepOutZonesCommand::all())));
    REQUIRE(doc.undo());
    CHECK(doc.keepOutZones() == three);

    REQUIRE(doc.undo());  // the create
    CHECK(doc.keepOutZones().empty());
    REQUIRE(doc.redo());
    CHECK(doc.keepOutZones() == three);
    // Unknown ids are skipped.
    CHECK_FALSE(doc.execute(std::make_unique<RemoveKeepOutZonesCommand>(std::vector<KeepOutZoneId>{KeepOutZoneId{987654}})));
}

TEST_CASE("zone ids share the document id space", "[keepout][document]") {
    Document doc;
    REQUIRE(doc.execute(std::make_unique<CreateKeepOutZonesCommand>(std::vector<Contour>{rect(0, 0, 1, 1)})));
    const std::uint64_t id = doc.keepOutZones()[0].id.value;
    CHECK(doc.hasObject(ObjectId{id}) == false);
    const ObjectId o = addVector(doc, {rect(0, 0, 1, 1)});
    CHECK(o.value != id);
}

TEST_CASE("clearance command validates, coalesces and undoes", "[keepout][document]") {
    Document doc;
    CHECK(doc.keepOutClearance() == 0.0);
    CHECK_FALSE(doc.execute(std::make_unique<SetKeepOutClearanceCommand>(-1.0)));
    CHECK_FALSE(doc.execute(std::make_unique<SetKeepOutClearanceCommand>(std::nan(""))));
    CHECK_FALSE(doc.execute(std::make_unique<SetKeepOutClearanceCommand>(1e9)));
    CHECK_FALSE(doc.execute(std::make_unique<SetKeepOutClearanceCommand>(0.0)));
    REQUIRE(doc.execute(std::make_unique<SetKeepOutClearanceCommand>(2.5)));
    CHECK(doc.keepOutClearance() == 2.5);
    REQUIRE(doc.execute(std::make_unique<SetKeepOutClearanceCommand>(4.0)));
    REQUIRE(doc.undo());
    CHECK(doc.keepOutClearance() == 0.0);  // both edits were one step
    REQUIRE(doc.redo());
    CHECK(doc.keepOutClearance() == 4.0);
}

TEST_CASE("zones, clearance and visibility survive save and load", "[keepout][document][carv]") {
    Document doc;
    REQUIRE(doc.execute(std::make_unique<CreateKeepOutZonesCommand>(
        std::vector<Contour>{rect(0, 0, 10, 10), Contour::circle({30, 30}, 5)})));
    REQUIRE(doc.execute(std::make_unique<SetKeepOutClearanceCommand>(1.5)));
    doc.setKeepOutVisible(false);

    auto bytes = saveCarv(doc);
    REQUIRE(bytes.ok());
    auto loaded = loadCarv(std::span<const std::uint8_t>(bytes.value()));
    REQUIRE(loaded.ok());
    const Document& d = loaded.value().document;
    CHECK(loaded.value().formatMinor == 4);
    CHECK(loaded.value().formatMinor == kCarvFormatMinorKeepOut);
    CHECK(d.keepOutZones() == doc.keepOutZones());
    CHECK(d.keepOutClearance() == 1.5);
    CHECK_FALSE(d.keepOutVisible());

    // Byte-for-byte stable.
    auto again = saveCarv(d, loaded.value().extras);
    REQUIRE(again.ok());
    CHECK(again.value() == bytes.value());

    // New ids do not collide with the loaded zones.
    Document copy = std::move(loaded.value().document);
    REQUIRE(copy.execute(std::make_unique<CreateKeepOutZonesCommand>(std::vector<Contour>{rect(0, 0, 1, 1)})));
    CHECK(copy.keepOutZones().size() == 3);
    CHECK(copy.keepOutZones()[2].id.value > doc.keepOutZones()[1].id.value);
}

TEST_CASE("a document without zones keeps its old format version and has no keepout record", "[keepout][document][carv]") {
    Document doc;
    auto bytes = saveCarv(doc);
    REQUIRE(bytes.ok());
    auto loaded = loadCarv(std::span<const std::uint8_t>(bytes.value()));
    REQUIRE(loaded.ok());
    CHECK(loaded.value().formatMinor == kCarvFormatMinorBase);
    CHECK(loaded.value().document.keepOutZones().empty());
    CHECK(loaded.value().document.keepOutVisible());
}

namespace {

std::string jobJson(const std::vector<std::uint8_t>& archive) {
    auto r = carvetto::document::detail::readZip(archive);
    REQUIRE(r.ok);
    for (const auto& m : r.members) {
        if (m.name == "job.json") {
            return {m.data.begin(), m.data.end()};
        }
    }
    FAIL("no job.json");
    return {};
}

}  // namespace

TEST_CASE("unknown keys inside the keepout record are preserved on re-save (T-0258)", "[keepout][document][carv]") {
    Document doc;
    REQUIRE(doc.execute(std::make_unique<CreateKeepOutZonesCommand>(std::vector<Contour>{rect(0, 0, 10, 10)})));
    auto bytes = saveCarv(doc);
    REQUIRE(bytes.ok());
    auto r = carvetto::document::detail::readZip(bytes.value());
    REQUIRE(r.ok);
    for (auto& m : r.members) {
        if (m.name == "job.json") {
            std::string text(m.data.begin(), m.data.end());
            const std::size_t pos = text.find("\"keepout\": {");
            REQUIRE(pos != std::string::npos);
            text.insert(pos + 13, "\"future_key\": {\"a\": [1, 2]}, ");
            m.data.assign(text.begin(), text.end());
        }
    }
    auto w = carvetto::document::detail::writeZip(r.members);
    REQUIRE(w.ok);

    auto loaded = loadCarv(std::span<const std::uint8_t>(w.bytes));
    REQUIRE(loaded.ok());
    CHECK(loaded.value().document.keepOutZones().size() == 1);
    auto again = saveCarv(loaded.value().document, loaded.value().extras);
    REQUIRE(again.ok());
    const std::string out = jobJson(again.value());
    CHECK(out.find("\"future_key\"") != std::string::npos);
    CHECK(out.find("\"a\"") != std::string::npos);

    // Without the extras the key is gone (it is not invented).
    auto plain = saveCarv(loaded.value().document);
    REQUIRE(plain.ok());
    CHECK(jobJson(plain.value()).find("future_key") == std::string::npos);
}
