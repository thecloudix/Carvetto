#include "carvetto/viewmodel/recovery.hpp"

#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "carvetto/document/commands.hpp"
#include "carvetto/geometry/contour.hpp"

using namespace carvetto;
using namespace carvetto::document;
using carvetto::viewmodel::FileSession;
using carvetto::viewmodel::RecoveryStore;
namespace fs = std::filesystem;

namespace {

class TempDir {
   public:
    TempDir() {
        static std::atomic<int> counter{0};
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        dir_ = fs::temp_directory_path() / ("carvetto_t0357_" + std::to_string(stamp) + "_" + std::to_string(counter++));
        fs::create_directories(dir_);
    }
    ~TempDir() {
        std::error_code ignored;
        fs::remove_all(dir_, ignored);
    }
    TempDir(const TempDir&) = delete;
    TempDir& operator=(const TempDir&) = delete;
    fs::path operator/(const std::string& name) const { return dir_ / name; }

   private:
    fs::path dir_;
};

std::vector<std::uint8_t> readAll(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

void writeAll(const fs::path& p, const std::vector<std::uint8_t>& b) {
    std::ofstream out(p, std::ios::binary | std::ios::trunc);
    out.write(reinterpret_cast<const char*>(b.data()), static_cast<std::streamsize>(b.size()));
}

void addCircle(Document& doc, double r) {
    const geometry::Vec2 c{50.0, 50.0};
    REQUIRE(doc.execute(std::make_unique<AddObjectCommand>(
        Object::vector(geometry::Contour::circle(c, r), doc.activeLayer(), doc.activeSheet(), CircleParams{c, r}))));
}

std::size_t objectCount(const FileSession& s) { return s.document().objects().size(); }

}  // namespace

TEST_CASE("T-0357: a killed session is recovered with its path and unsaved state", "[viewmodel][recovery]") {
    TempDir t;
    const fs::path file = t / "job.carv";
    {
        FileSession s;
        addCircle(s.document(), 5.0);
        REQUIRE(s.saveAs(file));
        addCircle(s.document(), 7.0);  // unsaved edit
        RecoveryStore store(t / "recovery");
        REQUIRE(store.tick(s));
        // The process "dies" here: no discard.
    }
    RecoveryStore next(t / "recovery");
    const auto pending = next.pending();
    REQUIRE(pending.has_value());
    CHECK(pending->displayName == "job.carv");

    FileSession fresh;
    REQUIRE(next.restore(fresh));
    CHECK(objectCount(fresh) == 2);
    CHECK(fresh.path() == file);
    CHECK(fresh.isDirty());
    CHECK_FALSE(next.pending().has_value());
    // The saved file on disk still has the older state until the user saves.
    FileSession onDisk;
    REQUIRE(onDisk.open(file));
    CHECK(objectCount(onDisk) == 1);
    // Saving the recovered session makes it clean.
    REQUIRE(fresh.save());
    CHECK_FALSE(fresh.isDirty());
}

TEST_CASE("T-0357: an untitled session recovers as untitled", "[viewmodel][recovery]") {
    TempDir t;
    {
        FileSession s;
        addCircle(s.document(), 5.0);
        RecoveryStore store(t / "r");
        REQUIRE(store.snapshot(s));
    }
    RecoveryStore next(t / "r");
    REQUIRE(next.pending().has_value());
    CHECK(next.pending()->displayName == "Untitled");
    FileSession fresh;
    REQUIRE(next.restore(fresh));
    CHECK_FALSE(fresh.hasPath());
    CHECK(fresh.isDirty());
    CHECK(objectCount(fresh) == 1);
}

TEST_CASE("T-0357: a clean exit leaves nothing to recover; a clean document drops the snapshot", "[viewmodel][recovery]") {
    TempDir t;
    FileSession s;
    RecoveryStore store(t / "r");
    CHECK_FALSE(store.tick(s));  // clean: nothing written
    CHECK_FALSE(store.pending().has_value());
    addCircle(s.document(), 5.0);
    REQUIRE(store.tick(s));
    REQUIRE(store.pending().has_value());
    REQUIRE(s.saveAs(t / "a.carv"));
    CHECK_FALSE(store.tick(s));  // saved: the stale snapshot goes
    CHECK_FALSE(store.pending().has_value());
    addCircle(s.document(), 6.0);
    REQUIRE(store.tick(s));
    store.discard();  // clean exit
    CHECK_FALSE(store.pending().has_value());
}

TEST_CASE("T-0357: a damaged recovery file is refused with a message and removed", "[viewmodel][recovery]") {
    TempDir t;
    {
        FileSession s;
        addCircle(s.document(), 5.0);
        RecoveryStore store(t / "r");
        REQUIRE(store.snapshot(s));
    }
    auto bytes = readAll(t / "r" / "autosave.carv");
    bytes.resize(bytes.size() / 2);
    writeAll(t / "r" / "autosave.carv", bytes);
    RecoveryStore next(t / "r");
    FileSession fresh;
    addCircle(fresh.document(), 3.0);
    CHECK_FALSE(next.restore(fresh));
    CHECK_FALSE(next.lastError().empty());
    CHECK(objectCount(fresh) == 1);  // untouched
    CHECK_FALSE(next.pending().has_value());
}

TEST_CASE("T-0357: save keeps a backup and an injected failure leaves a valid file", "[viewmodel][recovery]") {
    TempDir t;
    const fs::path file = t / "job.carv";
    FileSession s;
    addCircle(s.document(), 5.0);
    REQUIRE(s.saveAs(file));
    const auto first = readAll(file);
    addCircle(s.document(), 6.0);
    REQUIRE(s.save());
    CHECK(readAll(t / "job.carv.bak") == first);
    const auto second = readAll(file);

    addCircle(s.document(), 8.0);
    for (const auto point : {AtomicFailPoint::AfterTempWrite, AtomicFailPoint::BeforeRename}) {
        s.setSaveFailPointForTest(point);
        CHECK_FALSE(s.save());
        CHECK_FALSE(s.lastError().empty());
        CHECK(s.isDirty());  // still unsaved
        CHECK(readAll(file) == second);
        FileSession check;
        REQUIRE(check.open(file));
        CHECK(objectCount(check) == 2);
        CHECK_FALSE(fs::exists(t / "job.carv.tmp"));
    }
    s.setSaveFailPointForTest(AtomicFailPoint::None);
    REQUIRE(s.save());
    CHECK_FALSE(s.isDirty());
}

TEST_CASE("T-0357: corrupt and truncated .carv files give an error, never a crash or a changed session",
          "[viewmodel][recovery][fuzz]") {
    TempDir t;
    const fs::path good = t / "good.carv";
    {
        FileSession s;
        addCircle(s.document(), 5.0);
        addCircle(s.document(), 9.0);
        REQUIRE(s.saveAs(good));
    }
    const auto bytes = readAll(good);
    REQUIRE(bytes.size() > 64);
    FileSession session;
    addCircle(session.document(), 4.0);
    const fs::path bad = t / "bad.carv";

    // Every truncation (stepped so the corpus stays fast) is refused.
    const std::size_t step = bytes.size() / 60 + 1;
    for (std::size_t len = 0; len < bytes.size(); len += step) {
        writeAll(bad, std::vector<std::uint8_t>(bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(len)));
        CHECK_FALSE(session.open(bad));
        CHECK_FALSE(session.lastError().empty());
        CHECK(objectCount(session) == 1);
    }
    // Deterministic byte damage: a result either loads or reports an error; nothing may crash.
    std::uint32_t rng = 12345u;
    for (int i = 0; i < 80; ++i) {
        auto copy = bytes;
        for (int k = 0; k < 3; ++k) {
            rng = rng * 1664525u + 1013904223u;
            copy[(rng >> 8) % copy.size()] ^= static_cast<std::uint8_t>(1u << ((rng >> 4) % 8));
        }
        writeAll(bad, copy);
        FileSession probe;
        if (!probe.open(bad)) {
            CHECK_FALSE(probe.lastError().empty());
        }
    }
    // Garbage, an empty file and a folder.
    writeAll(bad, std::vector<std::uint8_t>(500, 0xAB));
    CHECK_FALSE(session.open(bad));
    writeAll(bad, {});
    CHECK_FALSE(session.open(bad));
    CHECK_FALSE(session.open(t / "missing.carv"));
    CHECK_FALSE(session.open(t / "recovery-folder-does-not-exist"));
    CHECK(objectCount(session) == 1);
}
