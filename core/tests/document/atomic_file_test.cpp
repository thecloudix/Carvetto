#include "carvetto/document/atomic_file.hpp"

#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;
using carvetto::document::AtomicFailPoint;
using carvetto::document::AtomicWriteOptions;
using carvetto::document::writeFileAtomic;

std::vector<std::uint8_t> bytesOf(const std::string& s) { return {s.begin(), s.end()}; }

std::string slurp(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

struct TempDir {
    fs::path dir = fs::temp_directory_path() / "carvetto_atomic_file_test";
    TempDir() {
        fs::remove_all(dir);
        fs::create_directories(dir);
    }
    ~TempDir() {
        std::error_code ec;
        fs::remove_all(dir, ec);
    }
};

}  // namespace

TEST_CASE("T-0357: atomic write replaces the file and leaves no temporary", "[document][atomic]") {
    TempDir t;
    const fs::path f = t.dir / "a.bin";
    std::string why;
    REQUIRE(writeFileAtomic(f, bytesOf("one"), why));
    REQUIRE(writeFileAtomic(f, bytesOf("two"), why));
    CHECK(slurp(f) == "two");
    CHECK_FALSE(fs::exists(t.dir / "a.bin.tmp"));
    CHECK_FALSE(fs::exists(t.dir / "a.bin.bak"));
}

TEST_CASE("T-0357: atomic write keeps the previous file as a backup on request", "[document][atomic]") {
    TempDir t;
    const fs::path f = t.dir / "a.bin";
    std::string why;
    AtomicWriteOptions opt;
    opt.keepBackup = true;
    REQUIRE(writeFileAtomic(f, bytesOf("one"), why, opt));
    CHECK_FALSE(fs::exists(t.dir / "a.bin.bak"));  // nothing to back up yet
    REQUIRE(writeFileAtomic(f, bytesOf("two"), why, opt));
    CHECK(slurp(f) == "two");
    CHECK(slurp(t.dir / "a.bin.bak") == "one");
}

TEST_CASE("T-0357: an injected failure leaves the old file valid", "[document][atomic]") {
    TempDir t;
    const fs::path f = t.dir / "a.bin";
    std::string why;
    REQUIRE(writeFileAtomic(f, bytesOf("good"), why));
    for (const AtomicFailPoint point : {AtomicFailPoint::AfterTempWrite, AtomicFailPoint::BeforeRename}) {
        AtomicWriteOptions opt;
        opt.keepBackup = true;
        opt.failAt = point;
        why.clear();
        CHECK_FALSE(writeFileAtomic(f, bytesOf("new content"), why, opt));
        CHECK_FALSE(why.empty());
        CHECK(slurp(f) == "good");
        CHECK_FALSE(fs::exists(t.dir / "a.bin.tmp"));
    }
}

TEST_CASE("T-0357: writing into a missing folder fails cleanly", "[document][atomic]") {
    TempDir t;
    std::string why;
    CHECK_FALSE(writeFileAtomic(t.dir / "nope" / "a.bin", bytesOf("x"), why));
    CHECK_FALSE(why.empty());
}
