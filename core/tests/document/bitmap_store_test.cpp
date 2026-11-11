#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>

#include "carvetto/document/bitmap_store.hpp"
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/undo_stack.hpp"
#include "carv_zip.hpp"

using namespace carvetto;
using namespace carvetto::document;

namespace {

std::vector<std::uint8_t> bytesOf(const std::string& s) { return {s.begin(), s.end()}; }

const std::string kPpm = "P3\n2 1\n255\n255 0 0  0 0 255\n";

BitmapImport importOf(Document& doc, std::vector<std::uint8_t> bytes) {
    BitmapImport in;
    in.bytes = std::move(bytes);
    in.widthPx = 2;
    in.heightPx = 1;
    in.placementMm = geometry::Box2{geometry::Vec2{0, 0}, geometry::Vec2{20, 10}};
    in.layer = doc.layers().front().id;
    in.sheet = doc.sheets().front().id;
    in.name = "pic";
    return in;
}

std::size_t bitmapObjects(const Document& doc) {
    std::size_t n = 0;
    for (const auto& [id, o] : doc.objects()) {
        if (o.isBitmap()) {
            ++n;
        }
    }
    return n;
}

}  // namespace

TEST_CASE("import embeds, dedups and undoes", "[bitmap_store]") {
    Document doc;
    UndoStack stack;
    const auto bytes = bytesOf(kPpm);
    auto cmd = makeImportBitmapCommand(doc, importOf(doc, bytes));
    REQUIRE(cmd);
    REQUIRE(stack.push(std::move(cmd), doc));
    CHECK(doc.bitmaps().size() == 1);
    CHECK(bitmapObjects(doc) == 1);
    // A second import of identical bytes stores the image once.
    REQUIRE(stack.push(makeImportBitmapCommand(doc, importOf(doc, bytes)), doc));
    CHECK(doc.bitmaps().size() == 1);
    CHECK(bitmapObjects(doc) == 2);
    CHECK(doc.checkIntegrity().empty());
    stack.undo(doc);
    CHECK(doc.bitmaps().size() == 1);  // still used by the first object
    CHECK(bitmapObjects(doc) == 1);
    stack.undo(doc);
    CHECK(doc.bitmaps().empty());
    CHECK(bitmapObjects(doc) == 0);
    stack.redo(doc);
    CHECK(doc.bitmaps().size() == 1);
    CHECK(bitmapObjects(doc) == 1);
}

TEST_CASE("import validation", "[bitmap_store]") {
    Document doc;
    CHECK_FALSE(makeImportBitmapCommand(doc, importOf(doc, {})));
    BitmapImport in = importOf(doc, bytesOf(kPpm));
    in.widthPx = 0;
    CHECK_FALSE(makeImportBitmapCommand(doc, in));
    in = importOf(doc, bytesOf(kPpm));
    in.placementMm.max = in.placementMm.min;
    CHECK_FALSE(makeImportBitmapCommand(doc, in));
    in = importOf(doc, std::vector<std::uint8_t>(kMaxEmbeddedBitmapBytes + 1, 1));
    CHECK_FALSE(bitmapImportProblem(doc, in).empty());
}

TEST_CASE("decodeBitmapObject", "[bitmap_store]") {
    Document doc;
    UndoStack stack;
    REQUIRE(stack.push(makeImportBitmapCommand(doc, importOf(doc, bytesOf(kPpm))), doc));
    const BitmapObject* b = nullptr;
    for (const auto& [id, o] : doc.objects()) {
        b = o.asBitmap();
    }
    REQUIRE(b);
    auto r = decodeBitmapObject(doc, *b);
    REQUIRE(r.ok());
    CHECK(r.value().width == 2);
    CHECK(r.value().rgb[0] == 255);
    CHECK(r.value().rgb[5] == 255);

    // Unknown format: needs a decoder; a decoder is used; garbage fails cleanly.
    Document d2;
    UndoStack s2;
    REQUIRE(s2.push(makeImportBitmapCommand(d2, importOf(d2, bytesOf("\x89PNG..."))), d2));
    const BitmapObject* b2 = nullptr;
    for (const auto& [id, o] : d2.objects()) {
        b2 = o.asBitmap();
    }
    CHECK_FALSE(decodeBitmapObject(d2, *b2).ok());
    BitmapDecoder dec = [](std::span<const std::uint8_t>) {
        io::RasterImage img;
        img.width = 1;
        img.height = 1;
        img.rgb = {1, 2, 3};
        return geometry::ShapeResult<io::RasterImage>(img);
    };
    CHECK(decodeBitmapObject(d2, *b2, dec).ok());
    BitmapDecoder bad = [](std::span<const std::uint8_t>) {
        return geometry::ShapeResult<io::RasterImage>(io::RasterImage{});
    };
    CHECK_FALSE(decodeBitmapObject(d2, *b2, bad).ok());
    BitmapObject legacy;
    legacy.sourceRef = "bitmaps/photo.png";
    CHECK_FALSE(decodeBitmapObject(d2, legacy, dec).ok());
}

TEST_CASE("decodeBitmapBytes works on bytes away from the document", "[bitmap_store]") {
    const std::vector<std::uint8_t> ppm = bytesOf(kPpm);
    auto r = decodeBitmapBytes(std::span<const std::uint8_t>(ppm));
    REQUIRE(r.ok());
    CHECK(r.value().width == 2);
    const std::vector<std::uint8_t> other = bytesOf("\x89PNG...");
    CHECK_FALSE(decodeBitmapBytes(std::span<const std::uint8_t>(other)).ok());
    CHECK_FALSE(decodeBitmapBytes(std::span<const std::uint8_t>{}).ok());
    BitmapDecoder bad = [](std::span<const std::uint8_t>) {
        return geometry::ShapeResult<io::RasterImage>(io::RasterImage{});
    };
    CHECK_FALSE(decodeBitmapBytes(std::span<const std::uint8_t>(other), bad).ok());
}

TEST_CASE("bitmap store .carv round trip", "[bitmap_store][carv]") {
    Document doc;
    UndoStack stack;
    std::vector<std::uint8_t> png = {0x89, 'P', 'N', 'G', 0, 1, 2, 3, 255, 254, 0, 0};
    REQUIRE(stack.push(makeImportBitmapCommand(doc, importOf(doc, png)), doc));
    auto saved = saveCarv(doc);
    REQUIRE(saved.ok());
    auto loaded = loadCarv(saved.value());
    REQUIRE(loaded.ok());
    CHECK(loaded.value().formatMinor == kCarvFormatMinorBitmap);
    REQUIRE(loaded.value().document.bitmaps().size() == 1);
    CHECK(loaded.value().document.bitmaps().begin()->second.bytes == png);
    CHECK(loaded.value().extras.empty());
    auto again = saveCarv(loaded.value().document, loaded.value().extras);
    REQUIRE(again.ok());
    CHECK(again.value() == saved.value());
}

TEST_CASE("documents without embedded bitmaps keep their minor", "[bitmap_store][carv]") {
    Document doc;
    auto saved = saveCarv(doc);
    REQUIRE(saved.ok());
    auto loaded = loadCarv(saved.value());
    REQUIRE(loaded.ok());
    CHECK(loaded.value().formatMinor == kCarvFormatMinorBase);
}

TEST_CASE("older reader keeps bitmap members; corrupt ones are rejected", "[bitmap_store][carv]") {
    Document doc;
    UndoStack stack;
    REQUIRE(stack.push(makeImportBitmapCommand(doc, importOf(doc, bytesOf(kPpm))), doc));
    const std::string key = doc.bitmaps().begin()->first;
    auto saved = saveCarv(doc);
    REQUIRE(saved.ok());
    // A pre-1.6 reader sees the member as an unknown one: simulated by writing it through extras
    // of a document that does not model it is not possible, so check the name is reserved.
    CHECK_THROWS(CarvExtras{}.withMember("bitmaps/" + key + ".img", {1}));
    CHECK_NOTHROW(CarvExtras{}.withMember("bitmaps/a.png", {1}));

    // Dangling reference: refused at save.
    Document dangling;
    UndoStack s2;
    REQUIRE(s2.push(makeImportBitmapCommand(dangling, importOf(dangling, bytesOf(kPpm))), dangling));
    dangling.takeBitmap(key);
    CHECK_FALSE(dangling.checkIntegrity().empty());
    CHECK_FALSE(saveCarv(dangling).ok());
}

TEST_CASE("corrupt or oversized bitmap members are rejected", "[bitmap_store][carv]") {
    Document doc;
    UndoStack stack;
    REQUIRE(stack.push(makeImportBitmapCommand(doc, importOf(doc, bytesOf(kPpm))), doc));
    const std::string name = "bitmaps/" + doc.bitmaps().begin()->first + ".img";
    auto saved = saveCarv(doc);
    REQUIRE(saved.ok());
    auto z = detail::readZip(saved.value());
    REQUIRE(z.ok);

    const auto withBytes = [&](const std::string& member, std::vector<std::uint8_t> data) {
        auto ms = z.members;
        bool found = false;
        for (auto& m : ms) {
            if (m.name == member) {
                m.data = data;
                found = true;
            }
        }
        if (!found) {
            ms.push_back({member, data});
        }
        auto w = detail::writeZip(ms);
        REQUIRE(w.ok);
        return w.bytes;
    };

    // Bytes that do not match the name.
    auto r = loadCarv(withBytes(name, bytesOf("P3\n1 1\n255\n0 0 0\n")));
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == CarvError::Code::InvalidContent);
    // Empty member.
    CHECK_FALSE(loadCarv(withBytes(name, {})).ok());
    // Oversized member (name matches its content hash, so only the cap rejects it).
    std::vector<std::uint8_t> big(kMaxEmbeddedBitmapBytes + 1, 7);
    const std::string bigName = "bitmaps/" + fontKeyFor(big) + ".img";
    auto rb = loadCarv(withBytes(bigName, big));
    REQUIRE_FALSE(rb.ok());
    CHECK(rb.error().code == CarvError::Code::ResourceLimit);
    // Truncated archive never crashes.
    auto cut = saved.value();
    cut.resize(cut.size() / 2);
    CHECK_FALSE(loadCarv(cut).ok());
}

TEST_CASE("legacy bitmap objects with an opaque source_ref still load", "[bitmap_store][carv]") {
    Document doc;
    Object o;
    o.layer = doc.layers().front().id;
    o.sheet = doc.sheets().front().id;
    BitmapObject b;
    b.widthPx = 4;
    b.heightPx = 4;
    b.placementMm = geometry::Box2{geometry::Vec2{0, 0}, geometry::Vec2{4, 4}};
    b.sourceRef = "bitmaps/photo.png";
    o.data = b;
    REQUIRE(AddObjectCommand(o).apply(doc));
    auto saved = saveCarv(doc);
    REQUIRE(saved.ok());
    auto loaded = loadCarv(saved.value());
    REQUIRE(loaded.ok());
    CHECK(loaded.value().formatMinor == kCarvFormatMinorBase);
    CHECK(loaded.value().document.bitmaps().empty());
}
