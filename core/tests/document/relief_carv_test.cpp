// T-0282: relief persistence in `.carv` (format 1.9, ADR-0061): byte-stable round trips, the
// `components/<id>.hf` members and hostile or damaged input.

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "carv_relief.hpp"
#include "carv_zip.hpp"
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/relief_commands.hpp"

using namespace carvetto::document;
namespace relief = carvetto::relief;
using Bytes = std::vector<std::uint8_t>;
using Code = CarvError::Code;

namespace {

relief::HeightField makeField(std::size_t w, std::size_t h, double cell, relief::Point origin, float seed) {
    std::vector<float> d(w * h);
    for (std::size_t i = 0; i < d.size(); ++i) {
        d[i] = (i % 5 == 3) ? relief::kTransparent : seed + 0.37F * static_cast<float>(i) - 1.5F;
    }
    relief::HeightField f;
    REQUIRE(relief::HeightField::fromData(relief::Grid{w, h, cell, origin}, std::move(d), f) == relief::Status::Ok);
    return f;
}

bool sameField(const relief::HeightField& a, const relief::HeightField& b) {
    if (a.width() != b.width() || a.height() != b.height() || a.cellSize() != b.cellSize() ||
        a.origin().x != b.origin().x || a.origin().y != b.origin().y) {
        return false;
    }
    for (std::size_t i = 0; i < a.data().size(); ++i) {
        std::uint32_t x = 0;
        std::uint32_t y = 0;
        std::memcpy(&x, &a.data()[i], 4);
        std::memcpy(&y, &b.data()[i], 4);
        if (x != y) {
            return false;
        }
    }
    return true;
}

struct Built {
    Document doc;
    SheetId s1, s2;
    LevelId l1, l2, l3;
    ComponentId a, b, c;
};

void build(Built& t) {
    Document& doc = t.doc;
    t.s1 = doc.activeSheet();
    t.s2 = doc.insertSheet(Sheet::fromJob(doc.job(), "Two"));
    const auto level = [&](SheetId s, const char* name) {
        auto cmd = std::make_unique<AddReliefLevelCommand>(s, name);
        auto* p = cmd.get();
        REQUIRE(doc.execute(std::move(cmd)));
        return p->createdId();
    };
    t.l1 = level(t.s1, "Base \xC3\xA9");
    t.l2 = level(t.s2, "Other sheet");
    t.l3 = level(t.s1, "Top");
    const auto comp = [&](LevelId l, const char* name, relief::HeightField f) {
        relief::Component c;
        c.name = name;
        c.field = std::move(f);
        auto cmd = std::make_unique<AddReliefComponentCommand>(l, std::move(c));
        auto* p = cmd.get();
        REQUIRE(doc.execute(std::move(cmd)));
        return p->createdId();
    };
    t.a = comp(t.l1, "A", makeField(7, 5, 0.25, {1.5, -2.0}, 1.0F));
    t.b = comp(t.l1, "B", makeField(3, 4, 0.5, {0.0, 0.0}, -4.0F));
    t.c = comp(t.l2, "C", makeField(6, 6, 1.0, {10.0, 10.0}, 0.1F));
    ReliefComponentProperties p;
    p.shapeHeightPercent = 75.5;
    p.baseHeight = -0.125;
    p.fade = {true, {0.1, 0.2}, {3.0, 4.5}, 33.0};
    p.tilt = {true, {1.0, 1.0}, {1.0, 9.0}, 0.0123456789};
    p.visible = false;
    REQUIRE(doc.execute(std::make_unique<SetReliefComponentPropertiesCommand>(t.b, p)));
    REQUIRE(doc.execute(std::make_unique<SetReliefComponentModeCommand>(t.a, relief::CombineMode::MergeLow)));
    REQUIRE(doc.execute(std::make_unique<SetReliefLevelPropertiesCommand>(t.l3, relief::CombineMode::Multiply, 2.75, false)));
}

Bytes save(const Document& doc) {
    auto r = saveCarv(doc);
    REQUIRE(r.ok());
    return r.value();
}

LoadedCarv load(const Bytes& bytes) {
    auto r = loadCarv(bytes);
    if (!r.ok()) {
        FAIL(r.error().message);
    }
    return std::move(r.value());
}

std::vector<detail::ZipMember> members(const Bytes& bytes) {
    auto z = detail::readZip(bytes);
    REQUIRE(z.ok);
    return z.members;
}

detail::ZipMember* find(std::vector<detail::ZipMember>& ms, const std::string& name) {
    for (auto& m : ms) {
        if (m.name == name) {
            return &m;
        }
    }
    return nullptr;
}

Bytes pack(const std::vector<detail::ZipMember>& ms) {
    auto z = detail::writeZip(ms, 1);
    REQUIRE(z.ok);
    return z.bytes;
}

CarvError loadError(const Bytes& bytes) {
    auto r = loadCarv(bytes);
    REQUIRE_FALSE(r.ok());
    return r.error();
}

void replaceAll(std::string& s, const std::string& from, const std::string& to) {
    const auto at = s.find(from);
    REQUIRE(at != std::string::npos);
    s.replace(at, from.size(), to);
}

std::string text(const detail::ZipMember& m) {
    return {m.data.begin(), m.data.end()};
}

}  // namespace

TEST_CASE("relief carv: round trip keeps levels, components and fields, and is byte stable",
          "[document][relief][carv]") {
    Built t;
    build(t);
    const Bytes first = save(t.doc);
    LoadedCarv loaded = load(first);
    CHECK(loaded.formatMajor == 1);
    CHECK(loaded.formatMinor == kCarvFormatMinorRelief);
    Document& d = loaded.document;
    REQUIRE(d.checkIntegrity().empty());
    REQUIRE(d.reliefLevels().size() == 3);
    for (std::size_t i = 0; i < 3; ++i) {
        const ReliefLevel& x = t.doc.reliefLevels()[i];
        const ReliefLevel& y = d.reliefLevels()[i];
        CHECK(x.id == y.id);
        CHECK(x.sheet == y.sheet);
        CHECK(x.name == y.name);
        CHECK(x.mode == y.mode);
        CHECK(x.baseHeight == y.baseHeight);
        CHECK(x.visible == y.visible);
        REQUIRE(x.components.size() == y.components.size());
        for (std::size_t j = 0; j < x.components.size(); ++j) {
            const relief::Component& p = x.components[j].data;
            const relief::Component& q = y.components[j].data;
            CHECK(x.components[j].id == y.components[j].id);
            CHECK(p.name == q.name);
            CHECK(p.mode == q.mode);
            CHECK(p.shapeHeightPercent == q.shapeHeightPercent);
            CHECK(p.baseHeight == q.baseHeight);
            CHECK(p.visible == q.visible);
            CHECK(p.fade.enabled == q.fade.enabled);
            CHECK(p.fade.anchor.x == q.fade.anchor.x);
            CHECK(p.fade.anchor.y == q.fade.anchor.y);
            CHECK(p.fade.second.x == q.fade.second.x);
            CHECK(p.fade.second.y == q.fade.second.y);
            CHECK(p.fade.percent == q.fade.percent);
            CHECK(p.tilt.enabled == q.tilt.enabled);
            CHECK(p.tilt.pivot.x == q.tilt.pivot.x);
            CHECK(p.tilt.pivot.y == q.tilt.pivot.y);
            CHECK(p.tilt.raise.x == q.tilt.raise.x);
            CHECK(p.tilt.raise.y == q.tilt.raise.y);
            CHECK(p.tilt.angleRad == q.tilt.angleRad);
            CHECK(sameField(p.field, q.field));
        }
    }
    // Ids keep their place in the generator: nothing allocated later can collide.
    CHECK(d.idGenerator().peek() == t.doc.idGenerator().peek());
    CHECK(save(d) == first);
    CHECK(save(load(save(d)).document) == first);
}

TEST_CASE("relief carv: archive layout", "[document][relief][carv]") {
    Built t;
    build(t);
    const Bytes bytes = save(t.doc);
    auto ms = members(bytes);
    CHECK(find(ms, "components/" + std::to_string(t.a.value) + ".hf") != nullptr);
    CHECK(find(ms, "components/" + std::to_string(t.b.value) + ".hf") != nullptr);
    CHECK(find(ms, "components/" + std::to_string(t.c.value) + ".hf") != nullptr);
    // The member is the documented 40 byte header plus four bytes per cell.
    CHECK(find(ms, "components/" + std::to_string(t.a.value) + ".hf")->data.size() == 40 + 4 * 35);
    CHECK(text(*find(ms, "manifest.json")).find("\"minor\": 9") != std::string::npos);
    CHECK(text(*find(ms, "job.json")).find("\"relief\"") != std::string::npos);
}

TEST_CASE("relief carv: a document without relief is written as before", "[document][relief][carv]") {
    Document doc;
    const Bytes bytes = save(doc);
    auto ms = members(bytes);
    CHECK(text(*find(ms, "job.json")).find("relief") == std::string::npos);
    CHECK(text(*find(ms, "manifest.json")).find("\"minor\": 1") != std::string::npos);
    for (const auto& m : ms) {
        CHECK(m.name.find("components/") == std::string::npos);
    }
    LoadedCarv loaded = load(bytes);
    CHECK(loaded.document.reliefLevels().empty());
    // An emptied relief goes back to the old format too.
    Built t;
    build(t);
    REQUIRE(t.doc.execute(std::make_unique<DeleteReliefLevelCommand>(t.l1)));
    REQUIRE(t.doc.execute(std::make_unique<DeleteReliefLevelCommand>(t.l2)));
    REQUIRE(t.doc.execute(std::make_unique<DeleteReliefLevelCommand>(t.l3)));
    CHECK(load(save(t.doc)).formatMinor == kCarvFormatMinorBase);
}

TEST_CASE("relief carv: a level without components, and every combine mode, round trip", "[document][relief][carv]") {
    Document doc;
    REQUIRE(doc.execute(std::make_unique<AddReliefLevelCommand>(doc.activeSheet(), "Empty")));
    const relief::CombineMode modes[] = {relief::CombineMode::Add, relief::CombineMode::Subtract,
                                         relief::CombineMode::MergeHigh, relief::CombineMode::MergeLow,
                                         relief::CombineMode::Multiply};
    auto lvl = std::make_unique<AddReliefLevelCommand>(doc.activeSheet(), "Modes");
    auto* lp = lvl.get();
    REQUIRE(doc.execute(std::move(lvl)));
    for (const relief::CombineMode m : modes) {
        relief::Component c;
        c.name = "m";
        c.mode = m;
        c.field = makeField(2, 2, 1.0, {0, 0}, 1.0F);
        REQUIRE(doc.execute(std::make_unique<AddReliefComponentCommand>(lp->createdId(), std::move(c))));
    }
    const Bytes first = save(doc);
    const LoadedCarv loaded = load(first);
    REQUIRE(loaded.document.reliefLevels().size() == 2);
    CHECK(loaded.document.reliefLevels()[0].components.empty());
    for (std::size_t i = 0; i < 5; ++i) {
        CHECK(loaded.document.reliefLevels()[1].components[i].data.mode == modes[i]);
    }
}

TEST_CASE("relief carv: transparent cells and exotic floats survive; NaN payloads are canonicalised",
          "[document][relief][carv]") {
    Document doc;
    auto lvl = std::make_unique<AddReliefLevelCommand>(doc.activeSheet(), "L");
    auto* lp = lvl.get();
    REQUIRE(doc.execute(std::move(lvl)));
    std::uint32_t oddNanBits = 0x7FA12345U;
    float oddNan = 0.0F;
    std::memcpy(&oddNan, &oddNanBits, 4);
    const std::vector<float> values = {relief::kTransparent, -0.0F, 0.0F, 1e-40F /*denormal*/, 3.4e38F,
                                       -3.4e38F,              oddNan, 123.456F};
    relief::Component c;
    c.name = "x";
    REQUIRE(relief::HeightField::fromData(relief::Grid{4, 2, 1.0, {0, 0}}, values, c.field) == relief::Status::Ok);
    REQUIRE(doc.execute(std::make_unique<AddReliefComponentCommand>(lp->createdId(), std::move(c))));
    const LoadedCarv loaded = load(save(doc));
    const auto& f = loaded.document.reliefLevels()[0].components[0].data.field;
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (std::isnan(values[i])) {
            CHECK(std::isnan(f.data()[i]));
        } else {
            std::uint32_t a = 0;
            std::uint32_t b = 0;
            std::memcpy(&a, &values[i], 4);
            std::memcpy(&b, &f.data()[i], 4);
            CHECK(a == b);  // -0.0 and denormals bit-exact
        }
    }
}

TEST_CASE("relief carv: a flat field compresses far past the usual ratio limit and still loads",
          "[document][relief][carv]") {
    Document doc;
    auto lvl = std::make_unique<AddReliefLevelCommand>(doc.activeSheet(), "L");
    auto* lp = lvl.get();
    REQUIRE(doc.execute(std::move(lvl)));
    relief::Component c;
    c.name = "flat";
    REQUIRE(relief::HeightField::create(relief::Grid{1500, 1500, 0.1, {0, 0}}, 2.0F, c.field) == relief::Status::Ok);
    REQUIRE(doc.execute(std::make_unique<AddReliefComponentCommand>(lp->createdId(), std::move(c))));
    const Bytes bytes = save(doc);
    CHECK(bytes.size() < 100000);  // 9 MB of cells in well under 100 kB: more than 20:1
    const LoadedCarv loaded = load(bytes);
    CHECK(loaded.document.reliefLevels()[0].components[0].data.field.at(1499, 1499) == 2.0F);
}

TEST_CASE("relief carv: saveCarv refuses unstorable relief", "[document][relief][carv]") {
    Built t;
    build(t);
    t.doc.findReliefComponent(t.a)->data.baseHeight = std::nan("");
    auto r = saveCarv(t.doc);
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == Code::InvalidContent);
    t.doc.findReliefComponent(t.a)->data.baseHeight = 0.0;
    t.doc.findReliefComponent(t.a)->data.field.set(0, 0, INFINITY);
    r = saveCarv(t.doc);
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == Code::InvalidContent);
    t.doc.findReliefComponent(t.a)->data.field.set(0, 0, 1.0F);
    t.doc.findReliefLevel(t.l1)->baseHeight = INFINITY;
    CHECK_FALSE(saveCarv(t.doc).ok());
    t.doc.findReliefLevel(t.l1)->baseHeight = 0.0;
    t.doc.findReliefLevel(t.l1)->name = std::string(kMaxReliefNameBytes + 1, 'x');
    CHECK_FALSE(saveCarv(t.doc).ok());
    t.doc.findReliefLevel(t.l1)->name = "ok";
    CHECK(saveCarv(t.doc).ok());
}

TEST_CASE("relief carv: height-field members are reserved names for extras", "[document][relief][carv]") {
    CHECK_THROWS(CarvExtras{}.withMember("components/12.hf", {1}));
    CHECK_NOTHROW(CarvExtras{}.withMember("components/012.hf", {1}));  // not a canonical name
    CHECK_NOTHROW(CarvExtras{}.withMember("components/x.hf", {1}));
}

// --- Hostile and damaged input -------------------------------------------------------------------

namespace {

/// A saved document with one 3x2 component, and the member name of its field.
struct OneField {
    Document doc;
    ComponentId id;
    std::string member;
    Bytes bytes;
    OneField() {
        auto lvl = std::make_unique<AddReliefLevelCommand>(doc.activeSheet(), "L");
        auto* lp = lvl.get();
        REQUIRE(doc.execute(std::move(lvl)));
        relief::Component c;
        c.name = "c";
        c.field = makeField(3, 2, 1.0, {0, 0}, 1.0F);
        auto add = std::make_unique<AddReliefComponentCommand>(lp->createdId(), std::move(c));
        auto* ap = add.get();
        REQUIRE(doc.execute(std::move(add)));
        id = ap->createdId();
        member = "components/" + std::to_string(id.value) + ".hf";
        bytes = save(doc);
    }
    /// Loads the archive with the field member replaced by `data`.
    CarvError withField(const Bytes& data) const {
        auto ms = members(bytes);
        find(ms, member)->data = data;
        return loadError(pack(ms));
    }
    Bytes field() const {
        auto ms = members(bytes);
        return find(ms, member)->data;
    }
};

void putU32(Bytes& b, std::size_t at, std::uint32_t v) {
    for (int i = 0; i < 4; ++i) {
        b[at + static_cast<std::size_t>(i)] = static_cast<std::uint8_t>((v >> (8 * i)) & 0xFF);
    }
}

}  // namespace

TEST_CASE("relief carv: damaged height fields fail with a clear error", "[document][relief][carv][hostile]") {
    const OneField f;
    const Bytes good = f.field();
    REQUIRE(good.size() == 40 + 24);

    {
        Bytes b = good;
        b[0] = 'X';
        const CarvError e = f.withField(b);
        CHECK(e.code == Code::InvalidContent);
        CHECK(e.message.find(f.member) != std::string::npos);
        CHECK(e.message.find("signature") != std::string::npos);
    }
    {
        Bytes b = good;
        putU32(b, 4, 2);
        const CarvError e = f.withField(b);
        CHECK(e.code == Code::InvalidContent);
        CHECK(e.message.find("version") != std::string::npos);
    }
    {  // truncated payload: the size no longer matches the header
        Bytes b = good;
        b.pop_back();
        const CarvError e = f.withField(b);
        CHECK(e.code == Code::InvalidContent);
        CHECK(e.message.find("takes 64") != std::string::npos);
    }
    {  // trailing bytes
        Bytes b = good;
        b.push_back(0);
        CHECK(f.withField(b).code == Code::InvalidContent);
    }
    {  // shorter than a header
        const CarvError e = f.withField(Bytes(10, 0));
        CHECK(e.code == Code::InvalidContent);
    }
    {  // empty member
        CHECK(f.withField({}).code == Code::InvalidContent);
    }
    {  // zero width
        Bytes b = good;
        putU32(b, 8, 0);
        CHECK(f.withField(b).code == Code::InvalidContent);
    }
    {  // zero height
        Bytes b = good;
        putU32(b, 12, 0);
        CHECK(f.withField(b).code == Code::InvalidContent);
    }
    {  // width x height disagrees with the member size
        Bytes b = good;
        putU32(b, 8, 4);
        CHECK(f.withField(b).code == Code::InvalidContent);
    }
    {  // cell size 0
        Bytes b = good;
        std::memset(&b[16], 0, 8);
        CHECK(f.withField(b).code == Code::InvalidContent);
    }
    {  // negative cell size
        Bytes b = good;
        b[23] = 0xBF;  // -1.0: 0xBFF0000000000000, bytes 16..23 little-endian
        std::memset(&b[16], 0, 7);
        b[22] = 0xF0;
        CHECK(f.withField(b).code == Code::InvalidContent);
    }
    {  // NaN origin
        Bytes b = good;
        std::memset(&b[24], 0xFF, 8);
        CHECK(f.withField(b).code == Code::InvalidContent);
    }
    {  // an infinite cell: exponent bytes 0x7F800000 in the last (fourth) plane for cell 0
        Bytes b = good;
        b[40 + 0] = 0x00;
        b[40 + 6 + 0] = 0x00;
        b[40 + 12 + 0] = 0x80;
        b[40 + 18 + 0] = 0x7F;
        const CarvError e = f.withField(b);
        CHECK(e.code == Code::InvalidContent);
        CHECK(e.message.find("infinite") != std::string::npos);
    }
}

TEST_CASE("relief carv: oversized fields are refused from the header, before any storage is allocated",
          "[document][relief][carv][hostile]") {
    const OneField f;
    const Bytes good = f.field();
    {  // 2^16 x 2^16 cells in a 64-byte member
        Bytes b = good;
        putU32(b, 8, 65536);
        putU32(b, 12, 65536);
        const CarvError e = f.withField(b);
        CHECK(e.code == Code::InvalidContent);
        CHECK(e.message.find("33554432") != std::string::npos);
    }
    {  // 8193 x 4096 = kMaxCells + 4096: too many cells although the height alone is fine
        Bytes b = good;
        putU32(b, 8, 8193);
        putU32(b, 12, 4096);
        CHECK(f.withField(b).message.find("33554432") != std::string::npos);
    }
    {  // exactly kMaxCells + 1
        Bytes b = good;
        putU32(b, 8, 1);
        putU32(b, 12, static_cast<std::uint32_t>(relief::kMaxCells + 1));
        CHECK(f.withField(b).message.find("33554432") != std::string::npos);
    }
    {  // within the cell cap but the member is nowhere near that big: size mismatch, not an allocation
        Bytes b = good;
        putU32(b, 8, 8192);
        putU32(b, 12, 4096);
        const CarvError e = f.withField(b);
        CHECK(e.code == Code::InvalidContent);
        CHECK(e.message.find("takes") != std::string::npos);
    }
    {  // 0xFFFFFFFF x 0xFFFFFFFF must not overflow the cell count
        Bytes b = good;
        putU32(b, 8, 0xFFFFFFFFU);
        putU32(b, 12, 0xFFFFFFFFU);
        CHECK(f.withField(b).code == Code::InvalidContent);
    }
}

TEST_CASE("relief carv: decodeHeightField never reads past the declared size", "[document][relief][carv][hostile]") {
    // Stream-level: the decoder consults the declared size, not the stream, to size its storage.
    const OneField f;
    auto ms = members(f.bytes);
    const Bytes good = find(ms, f.member)->data;
    detail::ZipArchive zip(f.bytes);
    std::string error;
    bool limit = false;
    REQUIRE(zip.open({}, error, limit));
    const detail::ZipEntry* e = zip.find(f.member);
    REQUIRE(e != nullptr);
    CHECK(e->reliefField);
    relief::HeightField out;
    {
        auto s = zip.stream(*e, error);
        REQUIRE(s);
        CHECK(detail::decodeHeightField(*s, e->size, out).empty());
        CHECK(out.width() == 3);
    }
    {
        auto s = zip.stream(*e, error);
        REQUIRE(s);
        CHECK_FALSE(detail::decodeHeightField(*s, e->size + 4, out).empty());  // lying declared size
        CHECK(out.empty());
    }
    {
        auto s = zip.stream(*e, error);
        REQUIRE(s);
        CHECK_FALSE(detail::decodeHeightField(*s, 39, out).empty());
    }
}

TEST_CASE("relief carv: a missing height-field member, a stale one and a corrupted one",
          "[document][relief][carv][hostile]") {
    const OneField f;
    {
        auto ms = members(f.bytes);
        ms.erase(std::remove_if(ms.begin(), ms.end(), [&](const auto& m) { return m.name == f.member; }), ms.end());
        const CarvError e = loadError(pack(ms));
        CHECK(e.code == Code::MissingMember);
        CHECK(e.message.find(f.member) != std::string::npos);
    }
    {  // a field no component names is ignored (not even inflated) and not carried into the next save
        auto ms = members(f.bytes);
        ms.push_back({"components/99999.hf", Bytes(5, 7)});
        LoadedCarv loaded = load(pack(ms));
        CHECK(loaded.extras.empty());
        CHECK(save(loaded.document) == f.bytes);
    }
    {  // a flipped payload byte is caught by the archive CRC
        auto z = members(f.bytes);
        Bytes archive = f.bytes;
        // Stored/deflated data: damage the middle of the archive until the CRC or inflate notices.
        bool failed = false;
        for (std::size_t at = 30; at + 8 < archive.size() && !failed; at += 7) {
            Bytes damaged = archive;
            damaged[at] ^= 0x55;
            auto r = loadCarv(damaged);
            if (!r.ok()) {
                failed = true;
                CHECK((r.error().code == Code::InvalidArchive || r.error().code == Code::InvalidContent ||
                       r.error().code == Code::InvalidJson || r.error().code == Code::MissingMember));
            }
        }
        CHECK(failed);
    }
}

TEST_CASE("relief carv: malformed relief records in job.json", "[document][relief][carv][hostile]") {
    Built t;
    build(t);
    const Bytes bytes = save(t.doc);
    const auto mutate = [&](const std::string& from, const std::string& to, const std::string& renameMember = "",
                            const std::string& toMember = "") {
        auto ms = members(bytes);
        if (!renameMember.empty()) {
            find(ms, renameMember)->name = toMember;
        }
        std::string job = text(*find(ms, "job.json"));
        replaceAll(job, from, to);
        find(ms, "job.json")->data.assign(job.begin(), job.end());
        return pack(ms);
    };
    {
        const CarvError e = loadError(mutate("\"merge_low\"", "\"bogus\""));
        CHECK(e.code == Code::InvalidContent);
        CHECK(e.message.find("relief[0].components[0].mode") != std::string::npos);
    }
    {  // a level on a sheet that does not exist
        const CarvError e = loadError(mutate("\"sheet\": " + std::to_string(t.s2.value), "\"sheet\": 31337"));
        CHECK(e.code == Code::InvalidContent);
        CHECK(e.message.find("sheet 31337 does not exist") != std::string::npos);
    }
    {  // two components with one id
        const CarvError e = loadError(
            mutate("\"id\": " + std::to_string(t.c.value) + ",", "\"id\": " + std::to_string(t.a.value) + ","));
        CHECK(e.code == Code::InvalidContent);
        CHECK(e.message.find("duplicate id") != std::string::npos);
    }
    {  // a component id that is also a layer's id
        const std::string layer = std::to_string(t.doc.defaultLayer().value);
        const CarvError e =
            loadError(mutate("\"id\": " + std::to_string(t.c.value) + ",", "\"id\": " + layer + ",",
                             "components/" + std::to_string(t.c.value) + ".hf", "components/" + layer + ".hf"));
        CHECK(e.code == Code::InvalidContent);
        CHECK(e.message.find("duplicate id") != std::string::npos);
    }
    {  // an enabled fade whose direction is degenerate
        const CarvError e = loadError(mutate("\"second\": [3.0, 4.5]", "\"second\": [0.1, 0.2]"));
        CHECK(e.code == Code::InvalidContent);
        CHECK(e.message.find("relief[0].components[1]") != std::string::npos);
        CHECK(e.message.find("degenerate") != std::string::npos);
    }
    {  // a non-finite number
        const CarvError e = loadError(mutate("\"percent\": 33.0", "\"percent\": \"NaN\""));
        CHECK(e.code == Code::InvalidContent);
        CHECK(e.message.find("percent") != std::string::npos);
    }
    {  // an over-long name
        const CarvError e = loadError(mutate("\"name\": \"A\"", "\"name\": \"" + std::string(300, 'n') + "\""));
        CHECK(e.code == Code::ResourceLimit);
    }
    {  // a missing required key
        const CarvError e = loadError(mutate("\"visible\": false\n", "\"visibl\": false\n"));
        CHECK(e.code == Code::InvalidContent);
        CHECK(e.message.find("missing 'visible'") != std::string::npos);
    }
    {  // a sheet that is not a number
        CHECK(loadError(mutate("\"sheet\": " + std::to_string(t.s2.value), "\"sheet\": \"x\"")).code ==
              Code::InvalidContent);
    }
    {  // keys of a later minor version are ignored, the rest still loads
        auto ms = members(bytes);
        std::string job = text(*find(ms, "job.json"));
        replaceAll(job, "\"mode\": \"merge_low\"", "\"mode\": \"merge_low\", \"future\": [1, 2]");
        replaceAll(job, "\"enabled\": true, \"percent\"", "\"enabled\": true, \"future\": 1, \"percent\"");
        find(ms, "job.json")->data.assign(job.begin(), job.end());
        const LoadedCarv loaded = load(pack(ms));
        CHECK(loaded.document.reliefLevels()[0].components[0].data.mode == relief::CombineMode::MergeLow);
        CHECK(save(loaded.document) == bytes);
    }
}

TEST_CASE("relief carv: the height-field encoding is the documented one", "[document][relief][carv]") {
    relief::HeightField f;
    REQUIRE(relief::HeightField::fromData(relief::Grid{2, 1, 0.5, {1.0, -1.0}}, {1.0F, -2.0F}, f) == relief::Status::Ok);
    Bytes out;
    CHECK(detail::encodeHeightField(f, out).empty());
    const Bytes expected = {'C',  'V',  'H',  'F',  1,    0,    0,    0,    2,    0,    0,    0,    1,    0,    0,
                            0,    0,    0,    0,    0,    0,    0,    0xE0, 0x3F, 0,    0,    0,    0,    0,    0,
                            0xF0, 0x3F, 0,    0,    0,    0,    0,    0,    0xF0, 0xBF,
                            // byte planes of 0x3F800000 and 0xC0000000
                            0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x3F, 0xC0};
    CHECK(out == expected);
    // Appends: a second call keeps what was there.
    Bytes twice = {9};
    CHECK(detail::encodeHeightField(f, twice).empty());
    CHECK(twice.size() == 1 + expected.size());
    CHECK(twice[0] == 9);
    relief::HeightField none;
    CHECK_FALSE(detail::encodeHeightField(none, out).empty());
    // Every NaN is written as the same bits, so equal documents give equal bytes.
    std::uint32_t oddBits = 0xFFA12345U;
    float odd = 0.0F;
    std::memcpy(&odd, &oddBits, 4);
    REQUIRE(std::isnan(odd));
    relief::HeightField a;
    relief::HeightField b;
    REQUIRE(relief::HeightField::fromData(relief::Grid{2, 1, 0.5, {1.0, -1.0}}, {odd, 1.0F}, a) == relief::Status::Ok);
    REQUIRE(relief::HeightField::fromData(relief::Grid{2, 1, 0.5, {1.0, -1.0}}, {relief::kTransparent, 1.0F}, b) ==
            relief::Status::Ok);
    Bytes ba;
    Bytes bb;
    CHECK(detail::encodeHeightField(a, ba).empty());
    CHECK(detail::encodeHeightField(b, bb).empty());
    CHECK(ba == bb);
}

TEST_CASE("relief carv: member names", "[document][relief][carv]") {
    CHECK(detail::reliefFieldIdOfName("components/7.hf") == 7);
    CHECK(detail::reliefFieldIdOfName("components/4611686018427387903.hf") == 4611686018427387903ULL);
    CHECK(detail::reliefFieldIdOfName("components/4611686018427387904.hf") == 0);  // 2^62: not a valid id
    CHECK(detail::reliefFieldIdOfName("components/99999999999999999999.hf") == 0);
    CHECK(detail::reliefFieldIdOfName("components/0.hf") == 0);
    CHECK(detail::reliefFieldIdOfName("components/07.hf") == 0);
    CHECK(detail::reliefFieldIdOfName("components/-7.hf") == 0);
    CHECK(detail::reliefFieldIdOfName("components/.hf") == 0);
    CHECK(detail::reliefFieldIdOfName("components/7.hfx") == 0);
    CHECK(detail::reliefFieldIdOfName("component/7.hf") == 0);
    CHECK(detail::reliefFieldIdOfName("components/7/8.hf") == 0);
    CHECK(detail::reliefFieldMemberName(123) == "components/123.hf");
    CHECK_THROWS(CarvExtras{}.withMember("components/4611686018427387903.hf", {1}));
    CHECK_NOTHROW(CarvExtras{}.withMember("components/4611686018427387904.hf", {1}));
}

TEST_CASE("relief carv: a field of the per-field cell maximum round trips", "[document][relief][carv][large]") {
    Document doc;
    auto lvl = std::make_unique<AddReliefLevelCommand>(doc.activeSheet(), "L");
    auto* lp = lvl.get();
    REQUIRE(doc.execute(std::move(lvl)));
    relief::Component c;
    c.name = "max";
    REQUIRE(relief::HeightField::create(relief::Grid{8192, 4096, 1.0, {0, 0}}, 1.5F, c.field) == relief::Status::Ok);
    c.field.set(8191, 4095, 9.0F);
    c.field.set(0, 4095, relief::kTransparent);
    REQUIRE(doc.execute(std::make_unique<AddReliefComponentCommand>(lp->createdId(), std::move(c))));
    LoadedCarv loaded = load(save(doc));
    const auto& f = loaded.document.reliefLevels()[0].components[0].data.field;
    CHECK(f.width() * f.height() == relief::kMaxCells);
    CHECK(f.at(8191, 4095) == 9.0F);
    CHECK(f.at(1, 4095) == 1.5F);
    CHECK(std::isnan(f.at(0, 4095)));
}
