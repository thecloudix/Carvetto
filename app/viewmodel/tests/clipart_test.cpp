// Clipart library browser: file recognition and limits, folder scan, library model, thumbnails and
// insertion (T-0333, ADR-0109).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/io/png_guard.hpp"
#include "carvetto/viewmodel/clipart_files.hpp"
#include "carvetto/viewmodel/clipart_insert.hpp"
#include "carvetto/viewmodel/clipart_library.hpp"
#include "carvetto/viewmodel/clipart_thumbnail.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using Catch::Approx;
namespace fs = std::filesystem;

namespace {

/// A scratch directory under the system temp dir, removed on scope exit.
struct TempDir {
    explicit TempDir(const std::string& name) {
        path = fs::temp_directory_path() / ("carvetto_clipart_" + name);
        std::error_code ec;
        fs::remove_all(path, ec);
        fs::create_directories(path);
    }
    ~TempDir() {
        std::error_code ec;
        fs::remove_all(path, ec);
    }
    std::string str() const { return path.string(); }
    std::string file(const std::string& rel) const { return (path / rel).string(); }
    fs::path path;
};

void write(const std::string& path, const std::string& content) {
    fs::create_directories(fs::path(path).parent_path());
    std::ofstream out(path, std::ios::binary);
    out.write(content.data(), static_cast<std::streamsize>(content.size()));
}

std::vector<std::uint8_t> bytesOf(const std::string& s) { return std::vector<std::uint8_t>(s.begin(), s.end()); }

std::string svgRect(const std::string& w, const std::string& h) {
    return "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" + w + "\" height=\"" + h + "\" viewBox=\"0 0 " +
           w.substr(0, w.size() - 2) + " " + h.substr(0, h.size() - 2) + "\"><rect x=\"0\" y=\"0\" width=\"" +
           w.substr(0, w.size() - 2) + "\" height=\"" + h.substr(0, h.size() - 2) + "\"/></svg>";
}

std::string dxfLines(int count, const std::string& layer = "CLIP") {
    std::string s = "0\nSECTION\n2\nENTITIES\n";
    for (int i = 0; i < count; ++i) {
        s += "0\nLINE\n8\n" + layer + "\n10\n" + std::to_string(i * 10) + "\n20\n0\n11\n" + std::to_string(i * 10 + 5) +
             "\n21\n10\n";
    }
    return s + "0\nENDSEC\n0\nEOF\n";
}

void put32be(std::string& s, std::uint32_t v) {
    s.push_back(static_cast<char>(v >> 24));
    s.push_back(static_cast<char>(v >> 16));
    s.push_back(static_cast<char>(v >> 8));
    s.push_back(static_cast<char>(v));
}

/// zlib stream (one fixed-Huffman block) inflating to `n` (>= 1) zero bytes at about 13 bits per 258
/// bytes: a tiny file that is a decompression bomb when `n` is far above what a header declares.
std::string zeroZlib(std::uint64_t n) {
    std::string z("\x78\x01", 2);
    std::uint32_t acc = 0;
    int bits = 0;
    const auto emit = [&](std::uint32_t value, int count, bool msbFirst) {
        for (int i = 0; i < count; ++i) {
            const std::uint32_t bit = msbFirst ? (value >> (count - 1 - i)) & 1U : (value >> i) & 1U;
            acc |= bit << bits;
            if (++bits == 8) {
                z.push_back(static_cast<char>(acc));
                acc = 0;
                bits = 0;
            }
        }
    };
    emit(1, 1, false);    // BFINAL
    emit(1, 2, false);    // BTYPE: fixed Huffman
    emit(0x30, 8, true);  // literal 0
    std::uint64_t left = n - 1;
    while (left >= 258) {
        emit(0xC5, 8, true);  // length 258
        emit(0, 5, true);     // distance 1
        left -= 258;
    }
    for (; left > 0; --left) {
        emit(0x30, 8, true);
    }
    emit(0, 7, true);  // end of block
    if (bits > 0) {
        z.push_back(static_cast<char>(acc));
    }
    put32be(z, 1);
    return z;
}

/// A PNG (RGB, 8 bit) with the given header size. IDAT inflates to `rawBytes` zero bytes (the size
/// the header declares by default); a header declaring more than 64 MiB gets no IDAT unless `rawBytes` is given.
std::string pngHeader(std::uint32_t w, std::uint32_t h, std::uint64_t rawBytes = 0) {
    std::string s("\x89PNG\r\n\x1a\n", 8);
    put32be(s, 13);
    s += "IHDR";
    put32be(s, w);
    put32be(s, h);
    s += std::string("\x08\x02\0\0\0", 5) + std::string(4, '\0');
    const std::uint64_t declared = static_cast<std::uint64_t>(h) * (1 + 3ULL * w);
    const std::uint64_t raw = rawBytes != 0 ? rawBytes : declared;
    if (w != 0 && h != 0 && (rawBytes != 0 || raw <= (64ULL << 20))) {
        const std::string z = zeroZlib(raw);
        put32be(s, static_cast<std::uint32_t>(z.size()));
        s += "IDAT" + z;
        put32be(s, 0);
        put32be(s, 0);
        s += "IEND";
        put32be(s, 0);
    }
    return s;
}

/// Decoder that ignores the bytes and returns a solid `w` x `h` image; counts its calls.
struct FakeDecoder {
    int w{4};
    int h{2};
    std::uint8_t value{100};
    std::shared_ptr<std::atomic<int>> calls = std::make_shared<std::atomic<int>>(0);
    document::BitmapDecoder fn() const {
        return [*this](std::span<const std::uint8_t>) -> geometry::ShapeResult<io::RasterImage> {
            ++*calls;
            io::RasterImage r;
            r.width = w;
            r.height = h;
            r.rgb.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 3U, value);
            return r;
        };
    }
};

}  // namespace

TEST_CASE("clipart: kind from extension", "[clipart]") {
    CHECK(clipartKindFromExtension("a/b/Fish.SVG") == ClipartKind::Svg);
    CHECK(clipartKindFromExtension("fish.dxf") == ClipartKind::Dxf);
    CHECK(clipartKindFromExtension("C:\\x\\fish.Png") == ClipartKind::Png);
    CHECK_FALSE(clipartKindFromExtension("fish.svgz").has_value());
    CHECK_FALSE(clipartKindFromExtension("fish.txt").has_value());
    CHECK_FALSE(clipartKindFromExtension("svg").has_value());
    CHECK_FALSE(clipartKindFromExtension(".svg").has_value());
    CHECK_FALSE(clipartKindFromExtension("dir.svg/readme").has_value());
}

TEST_CASE("clipart: sniffing by content", "[clipart]") {
    CHECK(sniffClipart(bytesOf(svgRect("20mm", "10mm"))).kind == ClipartKind::Svg);
    CHECK(sniffClipart(bytesOf("\xEF\xBB\xBF  \n<?xml version=\"1.0\"?><svg/>")).kind == ClipartKind::Svg);
    CHECK(sniffClipart(bytesOf(dxfLines(1))).kind == ClipartKind::Dxf);
    CHECK(sniffClipart(bytesOf("  999\r\ncomment\r\n  0\r\nSECTION\r\n")).kind == ClipartKind::Dxf);
    CHECK(sniffClipart(bytesOf("AutoCAD Binary DXF\r\n\x1a\0")).kind == ClipartKind::Dxf);
    CHECK(sniffClipart(bytesOf(pngHeader(1, 1))).kind == ClipartKind::Png);

    const auto refused = [](const std::string& s) { return sniffClipart(bytesOf(s)); };
    CHECK_FALSE(refused("").kind.has_value());
    CHECK(refused("").refusal.find("empty") != std::string::npos);
    CHECK(refused("\x1f\x8b\x08\0 gz").refusal.find("gzip") != std::string::npos);
    CHECK(refused("PK\x03\x04 zip").refusal.find("zip") != std::string::npos);
    CHECK(refused(std::string("7z\xbc\xaf\x27\x1c", 6)).refusal.find("7z") != std::string::npos);
    CHECK(refused(std::string("\xfd" "7zXZ\0", 6)).refusal.find("xz") != std::string::npos);
    CHECK(refused("BZh91AY").refusal.find("bzip2") != std::string::npos);
    CHECK(refused("\x28\xb5\x2f\xfd").refusal.find("zstd") != std::string::npos);
    CHECK(refused("<html><body/></html>").refusal.find("not SVG") != std::string::npos);
    CHECK(refused("   \n\t ").refusal.find("blank") != std::string::npos);
    CHECK(refused("hello world").refusal.find("not SVG, DXF or PNG") != std::string::npos);
    CHECK_FALSE(refused("12345\nSECTION").kind.has_value());   // five digits is not a group code
    CHECK_FALSE(refused("0\nno such word").kind.has_value());  // no SECTION in the head
    CHECK_FALSE(refused("0 x\nSECTION").kind.has_value());     // the first line must end after the digits
    CHECK(refused("0 \nSECTION").kind == ClipartKind::Dxf);
    // <svg only counts within the first 4 KiB.
    CHECK_FALSE(refused("<!--" + std::string(5000, 'x') + "--><svg/>").kind.has_value());
    CHECK(refused("<!--" + std::string(4000, 'x') + "--><svg/>").kind == ClipartKind::Svg);
    // The window is exactly 4096 bytes: "<svg" ending at byte 4096 counts, one byte later does not.
    CHECK(refused("<" + std::string(4091, ' ') + "<svg").kind == ClipartKind::Svg);
    CHECK_FALSE(refused("<" + std::string(4092, ' ') + "<svg").kind.has_value());
    // The same window limits the SECTION search of an ASCII DXF.
    CHECK(refused("0\n" + std::string(4096 - 2 - 7, ' ') + "SECTION").kind == ClipartKind::Dxf);
    CHECK_FALSE(refused("0\n" + std::string(4096 - 2 - 6, ' ') + "SECTION").kind.has_value());
}

TEST_CASE("clipart: PNG dimension limits are checked on the header", "[clipart][hostile]") {
    ClipartLimits lim;
    const auto check = [&](std::uint32_t w, std::uint32_t h) {
        return checkClipartBytes(bytesOf(pngHeader(w, h)), "x.png", lim);
    };
    const auto dims = pngDimensions(bytesOf(pngHeader(640, 480)));
    REQUIRE(dims.has_value());
    CHECK(dims->first == 640);
    CHECK(dims->second == 480);
    CHECK_FALSE(pngDimensions(bytesOf(pngHeader(1, 1).substr(0, 23))).has_value());
    CHECK(check(8192, 2048).ok);
    CHECK(check(4096, 4096).ok);  // exactly the pixel limit
    CHECK_FALSE(check(8193, 1).ok);
    CHECK_FALSE(check(1, 8193).ok);
    CHECK_FALSE(check(4097, 4096).ok);  // one row over the pixel limit
    CHECK_FALSE(check(0, 10).ok);
    CHECK_FALSE(check(10, 0).ok);
    const ClipartFile huge = check(0xFFFFFFFFU, 0xFFFFFFFFU);
    CHECK_FALSE(huge.ok);
    CHECK(huge.error.find("pixels") != std::string::npos);
    CHECK(huge.error.rfind("x.png: ", 0) == 0);
    std::string bad = pngHeader(1, 1);
    bad[12] = 'X';
    CHECK(checkClipartBytes(bytesOf(bad), "x.png", lim).error.find("damaged") != std::string::npos);
    // Decompression bombs: the header is inside the limits but the pixel data is not what it says.
    const std::uint64_t declared = 4ULL * (1 + 3 * 4);  // 4 x 4 RGB8
    CHECK(checkClipartBytes(bytesOf(pngHeader(4, 4, declared + io::kPngInflateSlackBytes)), "x.png", lim).ok);
    const ClipartFile bomb = checkClipartBytes(bytesOf(pngHeader(4, 4, declared + io::kPngInflateSlackBytes + 1)), "x.png", lim);
    CHECK_FALSE(bomb.ok);
    CHECK(bomb.error.find("bomb") != std::string::npos);
    const std::string tiny = pngHeader(1, 1, 256ULL << 20);  // a 256 MiB stream in under 2 MiB: refused
    CHECK(tiny.size() < 2 * 1024 * 1024);
    CHECK_FALSE(checkClipartBytes(bytesOf(tiny), "x.png", lim).ok);
    // A header declaring more raw bytes than the cap is refused before any inflating.
    lim.maxPngRawBytes = 4 * 4 * 3 + 4;
    CHECK(checkClipartBytes(bytesOf(pngHeader(4, 4)), "x.png", lim).ok);
    lim.maxPngRawBytes = 4 * 4 * 3 + 3;
    const ClipartFile over = checkClipartBytes(bytesOf(pngHeader(4, 4)), "x.png", lim);
    CHECK_FALSE(over.ok);
    CHECK(over.error.find("limit") != std::string::npos);
    lim.maxPngRawBytes = ClipartLimits{}.maxPngRawBytes;
    // The default cap admits the largest image the pixel limits allow (RGBA 8 bit = 4 bytes a pixel).
    CHECK(ClipartLimits{}.maxPngRawBytes >= ClipartLimits{}.maxImagePixels * 4 + ClipartLimits{}.maxImageEdge);
    // Damaged pixel data (no IDAT) is refused.
    std::string noIdat = pngHeader(4, 4).substr(0, 33);
    CHECK(checkClipartBytes(bytesOf(noIdat), "x.png", lim).error.find("damaged") != std::string::npos);
    // The extension has to agree with the content.
    const ClipartFile mism = checkClipartBytes(bytesOf(svgRect("1mm", "1mm")), "x.png", lim);
    CHECK_FALSE(mism.ok);
    CHECK(mism.error.find("rename") != std::string::npos);
    CHECK_FALSE(checkClipartBytes(bytesOf(svgRect("1mm", "1mm")), "x.txt", lim).ok);
}

TEST_CASE("clipart: reading a file enforces the size limit before reading", "[clipart][hostile]") {
    TempDir dir("read");
    ClipartLimits lim;
    lim.maxVectorFileBytes = 200;
    const std::string png = pngHeader(2, 2);
    lim.maxPngFileBytes = png.size();
    const std::string svg = svgRect("1mm", "1mm");
    REQUIRE(svg.size() < 200);
    write(dir.file("small.svg"), svg);
    CHECK(readClipartFile(dir.file("small.svg"), lim).ok);
    const std::string padded = svg + std::string(200 - svg.size(), ' ');
    write(dir.file("exact.svg"), padded);
    CHECK(readClipartFile(dir.file("exact.svg"), lim).ok);
    write(dir.file("over.svg"), padded + " ");
    const ClipartFile over = readClipartFile(dir.file("over.svg"), lim);
    CHECK_FALSE(over.ok);
    CHECK(over.error.find("limit") != std::string::npos);
    CHECK(over.error.rfind("over.svg: ", 0) == 0);
    write(dir.file("p.png"), png);
    CHECK(readClipartFile(dir.file("p.png"), lim).ok);  // exactly the limit
    write(dir.file("p2.png"), png + "x");
    CHECK_FALSE(readClipartFile(dir.file("p2.png"), lim).ok);  // one byte over
    write(dir.file("empty.svg"), "");
    CHECK(readClipartFile(dir.file("empty.svg"), lim).error.find("empty") != std::string::npos);
    CHECK(readClipartFile(dir.file("missing.svg"), lim).error.find("regular file") != std::string::npos);
    CHECK_FALSE(readClipartFile(dir.str(), lim).ok);
    write(dir.file("n.txt"), "x");
    CHECK_FALSE(readClipartFile(dir.file("n.txt"), lim).ok);
    write(dir.file("zip.svg"), "PK\x03\x04 not an svg");
    CHECK(readClipartFile(dir.file("zip.svg"), lim).error.find("zip") != std::string::npos);
}

TEST_CASE("clipart: hostile vector files are refused by the parsers' budgets", "[clipart][hostile]") {
    ClipartLimits lim;
    // Deep nesting.
    std::string deep = "<svg xmlns=\"http://www.w3.org/2000/svg\">";
    for (int i = 0; i < 200; ++i) {
        deep += "<g>";
    }
    deep += "<rect width=\"1\" height=\"1\"/>";
    for (int i = 0; i < 200; ++i) {
        deep += "</g>";
    }
    deep += "</svg>";
    const ClipartGeometry d = parseClipartVectors(bytesOf(deep), ClipartKind::Svg, lim);
    CHECK_FALSE(d.ok);
    CHECK(d.error.find("SVG") != std::string::npos);
    // The same nesting passes with a looser budget (so the refusal is the budget's).
    ClipartLimits loose = lim;
    loose.svgMaxDepth = 400;
    CHECK(parseClipartVectors(bytesOf(deep), ClipartKind::Svg, loose).ok);

    // Entity-expansion / DOCTYPE.
    const std::string laughs =
        "<?xml version=\"1.0\"?><!DOCTYPE svg [<!ENTITY a \"aaaaaaaaaa\"><!ENTITY b \"&a;&a;&a;&a;\">]>"
        "<svg xmlns=\"http://www.w3.org/2000/svg\"><text>&b;</text></svg>";
    CHECK_FALSE(parseClipartVectors(bytesOf(laughs), ClipartKind::Svg, lim).ok);

    // <use> amplification: 8 levels of 10 uses each = 10^8 visits without a budget.
    std::string use = "<svg xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\"><defs>"
                      "<rect id=\"u0\" width=\"1\" height=\"1\"/>";
    for (int i = 1; i <= 12; ++i) {
        use += "<g id=\"u" + std::to_string(i) + "\">";
        for (int k = 0; k < 10; ++k) {
            use += "<use xlink:href=\"#u" + std::to_string(i - 1) + "\"/>";
        }
        use += "</g>";
    }
    use += "</defs><use xlink:href=\"#u12\"/></svg>";
    const ClipartGeometry u = parseClipartVectors(bytesOf(use), ClipartKind::Svg, lim);
    CHECK_FALSE(u.ok);

    // Contour count: exactly the limit passes, one more does not.
    ClipartLimits few;
    few.maxContours = 3;
    CHECK(parseClipartVectors(bytesOf(dxfLines(3)), ClipartKind::Dxf, few).ok);
    const ClipartGeometry four = parseClipartVectors(bytesOf(dxfLines(4)), ClipartKind::Dxf, few);
    CHECK_FALSE(four.ok);
    CHECK(four.error.find("4 shapes") != std::string::npos);
    CHECK(four.contours.empty());

    // DXF span budget.
    ClipartLimits spans;
    spans.dxfMaxSpans = 2;
    CHECK_FALSE(parseClipartVectors(bytesOf(dxfLines(5)), ClipartKind::Dxf, spans).ok);

    // Not XML at all and not a vector kind.
    CHECK_FALSE(parseClipartVectors(bytesOf("<svg"), ClipartKind::Svg, lim).ok);
    CHECK_FALSE(parseClipartVectors(bytesOf("x"), ClipartKind::Png, lim).ok);
    // An empty drawing is fine but has no contours.
    const ClipartGeometry empty = parseClipartVectors(bytesOf("<svg xmlns=\"http://www.w3.org/2000/svg\"/>"), ClipartKind::Svg, lim);
    CHECK(empty.ok);
    CHECK(empty.contours.empty());
}

TEST_CASE("clipart: folder scan of mixed files", "[clipart][scan]") {
    TempDir dir("scan");
    write(dir.file("b_fish.svg"), svgRect("1mm", "1mm"));
    write(dir.file("A_bird.DXF"), dxfLines(1));
    write(dir.file("c_star.png"), pngHeader(2, 2));
    write(dir.file("notes.txt"), "x");
    write(dir.file(".hidden.svg"), svgRect("1mm", "1mm"));
    write(dir.file(".hiddendir/x.svg"), svgRect("1mm", "1mm"));
    write(dir.file("animals/cat.svg"), svgRect("1mm", "1mm"));
    write(dir.file("animals/deep/dog.svg"), svgRect("1mm", "1mm"));
    write(dir.file("animals/deep/er/owl.svg"), svgRect("1mm", "1mm"));

    ClipartScanLimits sl;
    sl.maxDepth = 2;
    const ClipartScan scan = scanClipartFolder(dir.str(), sl, ClipartLimits{});
    REQUIRE(scan.entries.size() == 5);
    CHECK_FALSE(scan.truncated);
    // Sorted by folder, then name, case-insensitively.
    CHECK(scan.entries[0].name == "A_bird");
    CHECK(scan.entries[0].folder.empty());
    CHECK(scan.entries[0].kind == ClipartKind::Dxf);
    CHECK(scan.entries[1].name == "b_fish");
    CHECK(scan.entries[2].name == "c_star");
    CHECK(scan.entries[2].kind == ClipartKind::Png);
    CHECK(scan.entries[3].folder == "animals");
    CHECK(scan.entries[3].name == "cat");
    CHECK(scan.entries[4].folder == "animals/deep");
    CHECK(scan.entries[4].sizeBytes == svgRect("1mm", "1mm").size());
    // Depth 1 stops one level earlier; depth 0 lists only the folder's own files.
    sl.maxDepth = 1;
    CHECK(scanClipartFolder(dir.str(), sl, ClipartLimits{}).entries.size() == 4);
    sl.maxDepth = 0;
    CHECK(scanClipartFolder(dir.str(), sl, ClipartLimits{}).entries.size() == 3);
    // Deeper allowed: the owl appears.
    sl.maxDepth = 3;
    CHECK(scanClipartFolder(dir.str(), sl, ClipartLimits{}).entries.size() == 6);
}

TEST_CASE("clipart: scan bounds the file count and the entries examined", "[clipart][scan][hostile]") {
    TempDir dir("scanbound");
    for (int i = 0; i < 12; ++i) {
        write(dir.file("f" + std::to_string(100 + i) + ".svg"), svgRect("1mm", "1mm"));
    }
    for (int i = 0; i < 20; ++i) {
        write(dir.file("junk" + std::to_string(i) + ".txt"), "x");
    }
    ClipartScanLimits sl;
    sl.maxFiles = 5;
    ClipartScan s = scanClipartFolder(dir.str(), sl, ClipartLimits{});
    CHECK(s.entries.size() == 5);
    CHECK(s.truncated);
    REQUIRE_FALSE(s.messages.empty());
    CHECK(s.messages[0].find("5 clipart files") != std::string::npos);
    sl.maxFiles = 12;  // exactly all of them: not truncated
    s = scanClipartFolder(dir.str(), sl, ClipartLimits{});
    CHECK(s.entries.size() == 12);
    CHECK_FALSE(s.truncated);
    CHECK(s.visited == 32);

    ClipartScanLimits visits;
    visits.maxVisited = 7;
    s = scanClipartFolder(dir.str(), visits, ClipartLimits{});
    CHECK(s.visited == 7);
    CHECK(s.truncated);
    CHECK(s.messages[0].find("examining 7") != std::string::npos);
    visits.maxVisited = 32;
    s = scanClipartFolder(dir.str(), visits, ClipartLimits{});
    CHECK_FALSE(s.truncated);
    CHECK(s.visited == 32);

    // A folder that does not exist reports a message and lists nothing.
    s = scanClipartFolder(dir.file("nope"), ClipartScanLimits{}, ClipartLimits{});
    CHECK(s.entries.empty());
    REQUIRE(s.messages.size() == 1);
    CHECK(s.messages[0].find("cannot be read") != std::string::npos);
}

TEST_CASE("clipart: scan skips symbolic links and refuses oversize files", "[clipart][scan][hostile]") {
    TempDir dir("scanlink");
    write(dir.file("real/a.svg"), svgRect("1mm", "1mm"));
    std::error_code ec;
    fs::create_directory_symlink(dir.path, dir.path / "real" / "loop", ec);
    const bool linked = !ec;
    ec.clear();
    fs::create_symlink(dir.path / "real" / "a.svg", dir.path / "link.svg", ec);
    ClipartLimits lim;
    lim.maxVectorFileBytes = 150;
    write(dir.file("big.svg"), std::string(151, ' '));
    write(dir.file("zero.svg"), "");
    const ClipartScan s = scanClipartFolder(dir.str(), ClipartScanLimits{}, lim);
    if (linked) {
        CHECK(s.visited < 20);  // a followed loop would run to the depth limit and beyond
    }
    std::size_t refused = 0;
    std::size_t reals = 0;
    for (const ClipartEntry& e : s.entries) {
        CHECK(e.name != "link");
        if (!e.refusal.empty()) {
            ++refused;
            if (e.name == "big") {
                CHECK(e.refusal.find("limit") != std::string::npos);
            } else {
                CHECK(e.name == "zero");
                CHECK(e.refusal.find("empty") != std::string::npos);
            }
        } else {
            ++reals;
        }
    }
    CHECK(refused == 2);
    CHECK(reals == 1);
}

TEST_CASE("clipart: library folders, search and favourites persist through the store", "[clipart][library]") {
    TempDir dir("library");
    write(dir.file("one/Red Fish.svg"), svgRect("1mm", "1mm"));
    write(dir.file("one/blue_fish.svg"), svgRect("1mm", "1mm"));
    write(dir.file("one/bird.dxf"), dxfLines(1));
    write(dir.file("two/star.png"), pngHeader(2, 2));
    write(dir.file("two/FISH sketch.svg"), svgRect("1mm", "1mm"));

    MemoryClipartStore store;
    ClipartLibrary lib(store);
    CHECK(lib.entries().empty());
    CHECK_FALSE(lib.addFolder(dir.file("missing")).ok);
    CHECK_FALSE(lib.addFolder("").ok);
    CHECK_FALSE(lib.addFolder("bad\npath").ok);
    CHECK_FALSE(lib.addFolder(dir.file("one/bird.dxf")).ok);  // a file, not a folder
    REQUIRE(lib.addFolder(dir.file("one")).ok);
    const ClipartEdit dup = lib.addFolder(dir.file("one"));
    CHECK_FALSE(dup.ok);
    CHECK(dup.error.find("already") != std::string::npos);
    REQUIRE(lib.addFolder(dir.file("two")).ok);
    CHECK(lib.entries().size() == 5);
    CHECK(lib.visible().size() == 5);
    CHECK(store.state.folders.size() == 2);

    // Entries remember the folder they were found under.
    for (const ClipartEntry& e : lib.entries()) {
        CHECK(lib.folders()[e.root] == dir.file(e.path.find("one") != std::string::npos ? "one" : "two"));
    }

    lib.setQuery("FISH");
    CHECK(lib.visible().size() == 3);
    lib.setQuery("  fish   red ");
    REQUIRE(lib.visible().size() == 1);
    CHECK(lib.entries()[lib.visible()[0]].name == "Red Fish");
    lib.setQuery("fish zzz");
    CHECK(lib.visible().empty());
    lib.setQuery("");
    CHECK(lib.visible().size() == 5);
    lib.setQuery("   ");
    CHECK(lib.visible().size() == 5);

    // Favourites.
    const std::string birdPath = dir.file("one/bird.dxf");
    CHECK_FALSE(lib.isFavourite(birdPath));
    CHECK(lib.toggleFavourite(birdPath).ok);
    CHECK(lib.isFavourite(birdPath));
    CHECK(lib.favouriteCount() == 1);
    lib.setFavouritesOnly(true);
    REQUIRE(lib.visible().size() == 1);
    CHECK(lib.entries()[lib.visible()[0]].path == birdPath);
    lib.setQuery("fish");
    CHECK(lib.visible().empty());  // search and favourites combine
    lib.setQuery("");
    lib.setFavouritesOnly(false);
    CHECK(lib.toggleFavourite(birdPath).ok);
    CHECK_FALSE(lib.isFavourite(birdPath));
    CHECK(lib.toggleFavourite(birdPath).ok);
    CHECK_FALSE(lib.setFavourite("bad\tpath", true).ok);

    // A fresh library over the same store sees folders and favourites.
    ClipartLibrary again(store);
    CHECK(again.folders().size() == 2);
    CHECK(again.isFavourite(birdPath));
    again.rescan();
    CHECK(again.entries().size() == 5);

    // reload() re-reads the store.
    store.state.folders = {dir.file("one")};
    store.state.favourites = {};
    again.reload();
    CHECK(again.folders().size() == 1);
    CHECK_FALSE(again.isFavourite(birdPath));
    CHECK(again.entries().size() == 3);
    store.state.folders = {dir.file("one"), dir.file("two")};
    again.reload();

    // Removing a folder drops its entries.
    CHECK_FALSE(again.removeFolder(dir.file("nope")));
    CHECK(again.removeFolder(dir.file("two")));
    CHECK(again.entries().size() == 3);
    CHECK(store.state.folders.size() == 1);
}

TEST_CASE("clipart: library limits and store failures", "[clipart][library][hostile]") {
    TempDir dir("libraryedge");
    MemoryClipartStore store;
    for (std::size_t i = 0; i < kMaxClipartFolders + 5; ++i) {
        store.state.folders.push_back(dir.file("f" + std::to_string(i)));
    }
    store.state.folders.push_back("");
    store.state.folders.push_back("a\nb");
    store.state.folders.push_back(dir.file("f0"));  // duplicate
    store.state.favourites = {"", "a\x01", "ok.svg", "ok.svg"};
    ClipartLibrary lib(store);
    CHECK(lib.folders().size() == kMaxClipartFolders);
    CHECK(lib.favouriteCount() == 1);
    for (std::size_t i = 0; i < kMaxClipartFolders; ++i) {
        fs::create_directories(dir.file("f" + std::to_string(i)));
    }
    const ClipartEdit full = lib.addFolder(dir.file("extra"));
    CHECK_FALSE(full.ok);
    CHECK(full.error.find("32") != std::string::npos);

    MemoryClipartStore failing;
    failing.failSaves = true;
    ClipartLibrary l2(failing);
    const ClipartEdit fav = l2.setFavourite("x.svg", true);
    CHECK_FALSE(fav.ok);
    CHECK(fav.error.find("saved") != std::string::npos);
    CHECK(l2.isFavourite("x.svg"));  // kept for this session
    CHECK_FALSE(l2.messages().empty());

    // Favourites are bounded.
    MemoryClipartStore many;
    for (std::size_t i = 0; i < kMaxClipartFavourites + 3; ++i) {
        many.state.favourites.push_back("f" + std::to_string(i) + ".svg");
    }
    ClipartLibrary l3(many);
    CHECK(l3.favouriteCount() == kMaxClipartFavourites);
    CHECK_FALSE(l3.setFavourite("new.svg", true).ok);
    CHECK(l3.setFavourite("f0.svg", true).ok);  // already a favourite: allowed at the cap
    CHECK(l3.setFavourite("f0.svg", false).ok);
    CHECK(l3.setFavourite("new.svg", true).ok);
}

TEST_CASE("clipart: library scan budget is shared between folders", "[clipart][library]") {
    TempDir dir("libraryshare");
    for (int i = 0; i < 4; ++i) {
        write(dir.file("a/f" + std::to_string(i) + ".svg"), svgRect("1mm", "1mm"));
        write(dir.file("b/g" + std::to_string(i) + ".svg"), svgRect("1mm", "1mm"));
    }
    MemoryClipartStore store;
    ClipartScanLimits sl;
    sl.maxFiles = 6;
    ClipartLibrary lib(store, sl);
    REQUIRE(lib.addFolder(dir.file("a")).ok);
    REQUIRE(lib.addFolder(dir.file("b")).ok);
    CHECK(lib.entries().size() == 6);
    CHECK(lib.visited() == 8);
    bool truncated = false;
    for (const std::string& m : lib.messages()) {
        truncated = truncated || m.find("2 clipart files") != std::string::npos;
    }
    CHECK(truncated);
    // The same folder reachable through two library folders is listed once.
    MemoryClipartStore s2;
    ClipartLibrary l2(s2);
    REQUIRE(l2.addFolder(dir.str()).ok);
    REQUIRE(l2.addFolder(dir.file("a")).ok);
    CHECK(l2.entries().size() == 8);
}

TEST_CASE("clipart: thumbnail cache is bounded and evicts least recently used", "[clipart][thumbnail]") {
    const auto entry = [](std::size_t bytes) {
        ClipartCacheEntry e;
        auto t = std::make_shared<ClipartThumbnail>();
        t->width = 1;
        t->height = 1;
        t->rgb.assign(bytes, 0);
        e.thumb = t;
        return e;
    };
    // Each entry costs 64 + its pixels.
    ClipartThumbnailCache cache(3 * (64 + 100), 100);
    cache.put("a", entry(100));
    cache.put("b", entry(100));
    cache.put("c", entry(100));
    CHECK(cache.size() == 3);
    CHECK(cache.bytes() == 3 * 164);
    CHECK(cache.evictions() == 0);
    REQUIRE(cache.get("a") != nullptr);  // a becomes most recent; b is now the oldest
    cache.put("d", entry(100));
    CHECK(cache.size() == 3);
    CHECK_FALSE(cache.contains("b"));
    CHECK(cache.contains("a"));
    CHECK(cache.evictions() == 1);
    CHECK(cache.bytes() == 3 * 164);
    // An entry costing exactly the whole byte bound is kept; one byte more is refused.
    ClipartThumbnailCache tight(64 + 100, 10);
    tight.put("x", entry(100));
    CHECK(tight.contains("x"));
    CHECK(tight.bytes() == 164);
    ClipartThumbnailCache tighter(64 + 100, 10);
    tighter.put("x", entry(101));
    CHECK_FALSE(tighter.contains("x"));
    CHECK(tighter.bytes() == 0);
    // Replacing a key does not double count.
    cache.put("a", entry(100));
    CHECK(cache.bytes() == 3 * 164);
    CHECK(cache.size() == 3);
    // One bigger entry pushes out several.
    cache.put("big", entry(300));
    CHECK(cache.bytes() <= 3 * 164);
    CHECK(cache.contains("big"));
    CHECK(cache.size() == 1);
    // An entry that can never fit is not stored (and does not flush the cache).
    const std::size_t before = cache.size();
    cache.put("huge", entry(10'000));
    CHECK_FALSE(cache.contains("huge"));
    CHECK(cache.size() == before);
    CHECK(cache.get("missing") == nullptr);
    cache.clear();
    CHECK(cache.size() == 0);
    CHECK(cache.bytes() == 0);

    // The entry-count bound alone also evicts.
    ClipartThumbnailCache few(1'000'000, 2);
    few.put("a", entry(1));
    few.put("b", entry(1));
    few.put("c", entry(1));
    CHECK(few.size() == 2);
    CHECK_FALSE(few.contains("a"));
    // Failures are cheap but not free.
    ClipartCacheEntry fail;
    fail.error = std::string(36, 'e');
    few.put("f", fail);
    CHECK(few.bytes() >= 100);
}

TEST_CASE("clipart: vector thumbnails are drawn Y up inside the bounds", "[clipart][thumbnail]") {
    // An "L": a vertical bar on the left and a horizontal bar at the bottom of a 20 x 20 mm box.
    const std::string svg =
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"20mm\" height=\"20mm\" viewBox=\"0 0 20 20\">"
        "<path d=\"M0 0 L0 20 M0 20 L20 20\"/></svg>";  // a bar on the left and one along the bottom
    const ClipartRender r = renderClipartThumbnail(bytesOf(svg), ClipartKind::Svg, 64, ClipartLimits{}, {});
    REQUIRE(r.ok);
    CHECK(r.thumb.width == 64);  // 57 px of drawing + 1 + 2 * 3 px of padding
    CHECK(r.thumb.height == r.thumb.width);
    CHECK(r.thumb.rgb.size() == static_cast<std::size_t>(r.thumb.width) * static_cast<std::size_t>(r.thumb.height) * 3U);
    CHECK(r.info.find("20.0 x 20.0 mm") != std::string::npos);
    CHECK(r.info.find("2 shapes") != std::string::npos);
    CHECK(r.segmentsDrawn == 2);
    CHECK_FALSE(r.truncated);
    const auto px = [&](int x, int y) { return r.thumb.rgb[(static_cast<std::size_t>(y) * static_cast<std::size_t>(r.thumb.width) + static_cast<std::size_t>(x)) * 3U]; };
    CHECK(px(0, 0) == 245);                                    // padding is background
    CHECK(px(3, r.thumb.height / 2) == 40);                    // the left bar
    CHECK(px(r.thumb.width / 2, r.thumb.height - 4) == 40);    // the other bar lies along the bottom
    CHECK(px(r.thumb.width / 2, 3) == 245);                    // nothing along the top
    CHECK(px(r.thumb.width - 4, r.thumb.height / 2) == 245);   // nothing on the right

    // A non-square drawing keeps its aspect; the edge is clamped to 16..512.
    const ClipartRender wide = renderClipartThumbnail(bytesOf(svgRect("40mm", "10mm")), ClipartKind::Svg, 100, ClipartLimits{}, {});
    REQUIRE(wide.ok);
    CHECK(wide.thumb.width == 100);
    CHECK(wide.thumb.height == 30);  // 10/40 of 93 = 23.25 -> 23, + 7
    const ClipartRender tiny = renderClipartThumbnail(bytesOf(svg), ClipartKind::Svg, 1, ClipartLimits{}, {});
    REQUIRE(tiny.ok);
    CHECK(tiny.thumb.width == kMinThumbEdge);
    const ClipartRender big = renderClipartThumbnail(bytesOf(svg), ClipartKind::Svg, 100000, ClipartLimits{}, {});
    REQUIRE(big.ok);
    CHECK(big.thumb.width == kMaxThumbEdge);
}

TEST_CASE("clipart: thumbnails of empty, text-only and hostile drawings fail cleanly", "[clipart][thumbnail][hostile]") {
    const ClipartRender none = renderClipartThumbnail(bytesOf("<svg xmlns=\"http://www.w3.org/2000/svg\"/>"), ClipartKind::Svg, 64, {}, {});
    CHECK_FALSE(none.ok);
    CHECK(none.error.find("no vectors") != std::string::npos);
    const std::string textOnly = "0\nSECTION\n2\nENTITIES\n0\nTEXT\n8\n0\n10\n0\n20\n0\n40\n5\n1\nHI\n0\nENDSEC\n0\nEOF\n";
    const ClipartRender t = renderClipartThumbnail(bytesOf(textOnly), ClipartKind::Dxf, 64, {}, {});
    CHECK_FALSE(t.ok);
    CHECK(t.error.find("text") != std::string::npos);
    const ClipartRender bad = renderClipartThumbnail(bytesOf("<svg"), ClipartKind::Svg, 64, {}, {});
    CHECK_FALSE(bad.ok);
    // A single point still gets a thumbnail.
    const std::string dot = "0\nSECTION\n2\nENTITIES\n0\nLINE\n8\n0\n10\n5\n20\n5\n11\n5\n21\n5\n0\nENDSEC\n0\nEOF\n";
    (void)renderClipartThumbnail(bytesOf(dot), ClipartKind::Dxf, 64, {}, {});
    // Absurd coordinates are clipped, not walked pixel by pixel.
    const std::string distant = "0\nSECTION\n2\nENTITIES\n0\nLINE\n8\n0\n10\n0\n20\n0\n11\n1e12\n21\n1e12\n0\nENDSEC\n0\nEOF\n";
    const ClipartRender f = renderClipartThumbnail(bytesOf(distant), ClipartKind::Dxf, 64, {}, {});
    if (f.ok) {
        CHECK(f.segmentsDrawn == 1);
        CHECK(f.thumb.width <= kMaxThumbEdge);
    }
}

TEST_CASE("clipart: segment budget truncates a vector thumbnail", "[clipart][thumbnail][hostile]") {
    // A circle polygonised at a tight tolerance has far fewer than the budget, so use many shapes.
    std::string svg = "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"100mm\" height=\"100mm\" viewBox=\"0 0 100 100\">";
    ClipartLimits lim;
    lim.maxContours = 20'000;
    for (int i = 0; i < 40; ++i) {
        svg += "<rect x=\"" + std::to_string(i) + "\" y=\"1\" width=\"1\" height=\"1\"/>";
    }
    svg += "</svg>";
    const ClipartRender r = renderClipartThumbnail(bytesOf(svg), ClipartKind::Svg, 64, lim, {});
    REQUIRE(r.ok);
    CHECK(r.segmentsDrawn == 40 * 4);
    CHECK_FALSE(r.truncated);
}

TEST_CASE("clipart: PNG thumbnails are box-filtered and never enlarged", "[clipart][thumbnail]") {
    FakeDecoder dec;
    dec.w = 8;
    dec.h = 4;
    dec.value = 90;
    const std::string png = pngHeader(8, 4);
    ClipartRender r = renderClipartThumbnail(bytesOf(png), ClipartKind::Png, 16, ClipartLimits{}, dec.fn());
    REQUIRE(r.ok);
    CHECK(r.thumb.width == 8);  // smaller than the edge: unchanged
    CHECK(r.thumb.height == 4);
    CHECK(r.thumb.rgb[0] == 90);
    CHECK(r.info == "8 x 4 px");

    // Shrinking: a 32 x 16 source against the edge of 16.
    FakeDecoder big;
    big.w = 32;
    big.h = 16;
    r = renderClipartThumbnail(bytesOf(pngHeader(32, 16)), ClipartKind::Png, 16, ClipartLimits{}, big.fn());
    REQUIRE(r.ok);
    CHECK(r.thumb.width == 16);
    CHECK(r.thumb.height == 8);

    // Averaging: a 32 x 32 checker of 0/255 shrunk to 16 x 16 becomes mid grey.
    io::RasterImage checker;
    checker.width = 32;
    checker.height = 32;
    checker.rgb.resize(32U * 32U * 3U);
    for (std::size_t y = 0; y < 32; ++y) {
        for (std::size_t x = 0; x < 32; ++x) {
            const std::uint8_t v = ((x + y) % 2 == 0) ? 0 : 255;
            for (std::size_t k = 0; k < 3; ++k) {
                checker.rgb[(y * 32 + x) * 3 + k] = v;
            }
        }
    }
    const document::BitmapDecoder checkerDec = [&](std::span<const std::uint8_t>) -> geometry::ShapeResult<io::RasterImage> { return checker; };
    r = renderClipartThumbnail(bytesOf(pngHeader(32, 32)), ClipartKind::Png, 16, ClipartLimits{}, checkerDec);
    REQUIRE(r.ok);
    CHECK(r.thumb.width == 16);
    CHECK(r.thumb.rgb[0] == 128);  // (0+255+255+0 + 2) / 4 = 128

    // No decoder / decoder failure / unusable image.
    CHECK_FALSE(renderClipartThumbnail(bytesOf(png), ClipartKind::Png, 16, ClipartLimits{}, {}).ok);
    const document::BitmapDecoder failing = [](std::span<const std::uint8_t>) -> geometry::ShapeResult<io::RasterImage> {
        return std::string("damaged");
    };
    const ClipartRender fr = renderClipartThumbnail(bytesOf(png), ClipartKind::Png, 16, ClipartLimits{}, failing);
    CHECK_FALSE(fr.ok);
    CHECK(fr.error.find("damaged") != std::string::npos);
    const document::BitmapDecoder zero = [](std::span<const std::uint8_t>) -> geometry::ShapeResult<io::RasterImage> {
        return io::RasterImage{};
    };
    CHECK_FALSE(renderClipartThumbnail(bytesOf(png), ClipartKind::Png, 16, ClipartLimits{}, zero).ok);

    // A hostile header never reaches the decoder.
    FakeDecoder counting;
    CHECK_FALSE(renderClipartThumbnail(bytesOf(pngHeader(0xFFFFFFFFU, 3)), ClipartKind::Png, 16, ClipartLimits{}, counting.fn()).ok);
    CHECK_FALSE(renderClipartThumbnail(bytesOf(pngHeader(9000, 1)), ClipartKind::Png, 16, ClipartLimits{}, counting.fn()).ok);
    CHECK_FALSE(renderClipartThumbnail(bytesOf(pngHeader(0, 1)), ClipartKind::Png, 16, ClipartLimits{}, counting.fn()).ok);
    CHECK_FALSE(renderClipartThumbnail(bytesOf(pngHeader(1, 9000)), ClipartKind::Png, 16, ClipartLimits{}, counting.fn()).ok);
    CHECK_FALSE(renderClipartThumbnail(bytesOf(pngHeader(4097, 4096)), ClipartKind::Png, 16, ClipartLimits{}, counting.fn()).ok);
    CHECK_FALSE(renderClipartThumbnail(bytesOf("\x89PNG\r\n\x1a\n"), ClipartKind::Png, 16, ClipartLimits{}, counting.fn()).ok);
    // A tiny header over a stream that inflates far beyond it never reaches the decoder either.
    const ClipartRender bomb = renderClipartThumbnail(bytesOf(pngHeader(1, 1, 64ULL << 20)), ClipartKind::Png, 16,
                                                      ClipartLimits{}, counting.fn());
    CHECK_FALSE(bomb.ok);
    CHECK(bomb.error.find("bomb") != std::string::npos);
    CHECK(counting.calls->load() == 0);
}

TEST_CASE("clipart: thumbnail service works off the calling thread and bounds its queue", "[clipart][thumbnail][service]") {
    TempDir dir("service");
    write(dir.file("a.svg"), svgRect("10mm", "10mm"));
    write(dir.file("b.svg"), svgRect("10mm", "10mm"));
    write(dir.file("c.png"), pngHeader(4, 2));
    write(dir.file("bad.svg"), "PK\x03\x04");

    std::mutex m;
    std::condition_variable cv;
    bool entered = false;
    bool release = false;
    FakeDecoder inner;
    const document::BitmapDecoder gated = [&](std::span<const std::uint8_t> b) -> geometry::ShapeResult<io::RasterImage> {
        {
            std::unique_lock<std::mutex> lock(m);
            entered = true;
            cv.notify_all();
            cv.wait(lock, [&] { return release; });
        }
        return inner.fn()(b);
    };

    ClipartThumbnailService::Config cfg;
    cfg.edge = 64;
    cfg.maxPending = 2;
    ClipartThumbnailService svc(cfg, gated);
    CHECK(svc.state(dir.file("c.png")) == ClipartThumbState::Unknown);
    CHECK(svc.request(dir.file("c.png")));
    {
        std::unique_lock<std::mutex> lock(m);
        cv.wait(lock, [&] { return entered; });  // the worker holds c.png inside the decoder
    }
    CHECK(svc.state(dir.file("c.png")) == ClipartThumbState::Pending);
    CHECK(svc.request(dir.file("c.png")));  // idempotent
    CHECK(svc.request(dir.file("a.svg")));
    CHECK(svc.request(dir.file("b.svg")));
    CHECK_FALSE(svc.request(dir.file("bad.svg")));  // queue of 2 is full
    svc.cancelPending();                             // drops a and b
    CHECK(svc.state(dir.file("a.svg")) == ClipartThumbState::Unknown);
    CHECK(svc.request(dir.file("a.svg")));
    CHECK(svc.request(dir.file("bad.svg")));
    {
        std::lock_guard<std::mutex> lock(m);
        release = true;
    }
    cv.notify_all();
    svc.waitIdle();
    CHECK(svc.processed() == 3);
    CHECK(svc.state(dir.file("a.svg")) == ClipartThumbState::Ready);
    CHECK(svc.state(dir.file("bad.svg")) == ClipartThumbState::Failed);
    CHECK(svc.state(dir.file("b.svg")) == ClipartThumbState::Unknown);
    // c.png finished after cancelPending bumped the generation: its result is discarded.
    CHECK(svc.state(dir.file("c.png")) == ClipartThumbState::Unknown);
    const ClipartCacheEntry a = svc.lookup(dir.file("a.svg"));
    REQUIRE(a.thumb != nullptr);
    CHECK(a.thumb->width > 0);
    CHECK(a.info.find("10.0 x 10.0 mm") != std::string::npos);
    const ClipartCacheEntry bad = svc.lookup(dir.file("bad.svg"));
    CHECK(bad.thumb == nullptr);
    CHECK(bad.error.find("zip") != std::string::npos);
    const std::vector<std::string> done = svc.takeCompleted();
    CHECK(done.size() == 2);
    CHECK(svc.takeCompleted().empty());
    CHECK(svc.cacheBytes() > 0);

    // PNG now through the normal path, then clear() forgets it.
    CHECK(svc.request(dir.file("c.png")));
    svc.waitIdle();
    CHECK(svc.state(dir.file("c.png")) == ClipartThumbState::Ready);
    svc.clear();
    CHECK(svc.state(dir.file("c.png")) == ClipartThumbState::Unknown);
    CHECK(svc.cacheBytes() == 0);
    CHECK(svc.takeCompleted().empty());
}

TEST_CASE("clipart: thumbnail cache of the service stays within its byte bound", "[clipart][thumbnail][service]") {
    TempDir dir("servicebound");
    ClipartThumbnailService::Config cfg;
    cfg.edge = 64;
    cfg.cacheBytes = 1;  // clamped up to one mebibyte
    cfg.cacheEntries = 5;
    ClipartThumbnailService svc(cfg, {});
    for (int i = 0; i < 12; ++i) {
        write(dir.file("s" + std::to_string(i) + ".svg"), svgRect(std::to_string(i + 1) + "mm", "5mm"));
        CHECK(svc.request(dir.file("s" + std::to_string(i) + ".svg")));
    }
    svc.waitIdle();
    CHECK(svc.processed() == 12);
    CHECK(svc.cacheEvictions() == 7);
    CHECK(svc.cacheBytes() <= 1024U * 1024U);
    int ready = 0;
    for (int i = 0; i < 12; ++i) {
        ready += svc.state(dir.file("s" + std::to_string(i) + ".svg")) == ClipartThumbState::Ready ? 1 : 0;
    }
    CHECK(ready == 5);
    // The completed list is bounded by the entry bound too: the newest five, oldest first.
    const std::vector<std::string> done = svc.takeCompleted();
    REQUIRE(done.size() == 5);
    for (int i = 0; i < 5; ++i) {
        CHECK(done[static_cast<std::size_t>(i)] == dir.file("s" + std::to_string(i + 7) + ".svg"));
    }
    CHECK(svc.takeCompleted().empty());
}

TEST_CASE("clipart: inserting an SVG is one undo step, scaled to fit and centred", "[clipart][insert]") {
    document::Document doc;  // 300 x 300 mm material, bottom-left datum: centre (150, 150)
    const std::size_t steps = doc.undoStack().count();
    ClipartInsertOptions opt;
    const ClipartInsertOutcome r = insertClipartBytes(doc, bytesOf(svgRect("20mm", "10mm")), "box.svg", opt, {});
    REQUIRE(r.ok);
    CHECK(r.kind == ClipartKind::Svg);
    CHECK(r.objectsAdded == 1);
    CHECK(doc.undoStack().count() == steps + 1);
    CHECK(doc.undoStack().undoText() == "Insert clipart");
    CHECK(r.scale == Approx(7.5));  // 150 x 150 box: min(150/20, 150/10)
    CHECK(r.placedBox.width() == Approx(150.0));
    CHECK(r.placedBox.height() == Approx(75.0));
    CHECK(r.placedBox.centre().x == Approx(150.0));
    CHECK(r.placedBox.centre().y == Approx(150.0));
    CHECK(doc.selection().ids().size() == 1);
    CHECK(doc.objectCount() == 1);

    REQUIRE(doc.undo());
    CHECK(doc.objectCount() == 0);
    CHECK(doc.undoStack().count() == steps + 1);  // still recorded, now undone
    REQUIRE(doc.redo());
    REQUIRE(doc.objectCount() == 1);
    const auto id = doc.objects().begin()->first;
    CHECK(doc.objectBounds(id)->width() == Approx(150.0));
}

TEST_CASE("clipart: insertion point, fit fraction and non-square material", "[clipart][insert]") {
    document::Job job;
    job.sizeMm = {200.0, 100.0, 18.0};
    document::Document doc(job);
    ClipartInsertOptions opt;
    // 10 x 10 clipart into 100 x 50 (half the material): the height decides.
    ClipartInsertOutcome r = insertClipartBytes(doc, bytesOf(svgRect("10mm", "10mm")), "sq.svg", opt, {});
    REQUIRE(r.ok);
    CHECK(r.scale == Approx(5.0));
    CHECK(r.placedBox.centre().x == Approx(100.0));
    CHECK(r.placedBox.centre().y == Approx(50.0));
    // 40 x 10 clipart: the width decides (100/40 = 2.5 < 50/10 = 5).
    opt.at = geometry::Vec2{-20.0, 30.0};
    r = insertClipartBytes(doc, bytesOf(svgRect("40mm", "10mm")), "wide.svg", opt, {});
    REQUIRE(r.ok);
    CHECK(r.scale == Approx(2.5));
    CHECK(r.placedBox.centre().x == Approx(-20.0));
    CHECK(r.placedBox.centre().y == Approx(30.0));
    CHECK(r.placedBox.width() == Approx(100.0));
    // The fit fraction scales the target box.
    opt.fitFraction = 0.25;
    r = insertClipartBytes(doc, bytesOf(svgRect("40mm", "10mm")), "wide.svg", opt, {});
    REQUIRE(r.ok);
    CHECK(r.scale == Approx(1.25));
    opt.fitFraction = 1.0;
    r = insertClipartBytes(doc, bytesOf(svgRect("40mm", "10mm")), "wide.svg", opt, {});
    REQUIRE(r.ok);
    CHECK(r.scale == Approx(5.0));
    // Invalid fractions and positions are refused untouched.
    const std::uint64_t gen = doc.undoStack().generation();
    for (const double f : {0.0, -1.0, 1.5}) {
        opt.fitFraction = f;
        CHECK_FALSE(insertClipartBytes(doc, bytesOf(svgRect("1mm", "1mm")), "a.svg", opt, {}).ok);
    }
    opt.fitFraction = 0.5;
    opt.at = geometry::Vec2{std::nan(""), 0.0};
    CHECK_FALSE(insertClipartBytes(doc, bytesOf(svgRect("1mm", "1mm")), "a.svg", opt, {}).ok);
    CHECK(doc.undoStack().generation() == gen);
}

TEST_CASE("clipart: an empty material fits a 100 mm box", "[clipart][insert]") {
    document::Job job;
    job.sizeMm = {0.0, 0.0, 0.0};
    document::Document doc(job);
    REQUIRE(doc.findSheet(doc.activeSheet()) != nullptr);
    REQUIRE(doc.findSheet(doc.activeSheet())->materialBoundsMm().width() == Approx(0.0));
    ClipartInsertOptions opt;  // fit fraction 0.5 of 100 mm = 50 mm
    const ClipartInsertOutcome r = insertClipartBytes(doc, bytesOf(svgRect("10mm", "10mm")), "sq.svg", opt, {});
    REQUIRE(r.ok);
    CHECK(r.scale == Approx(5.0));
    CHECK(r.placedBox.width() == Approx(50.0));
    CHECK(r.placedBox.centre().x == Approx(0.0));
    CHECK(r.placedBox.centre().y == Approx(0.0));
}

TEST_CASE("clipart: a line on a non-square material is scaled by the matching side", "[clipart][insert]") {
    document::Job job;
    job.sizeMm = {200.0, 100.0, 18.0};  // fit box at 0.5: 100 x 50
    document::Document doc(job);
    ClipartInsertOptions opt;
    const std::string head = "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"10mm\" height=\"10mm\" viewBox=\"0 0 10 10\">";
    // A vertical line (no width) is as tall as the fit box is high: 50 / 10.
    ClipartInsertOutcome r = insertClipartBytes(doc, bytesOf(head + "<path d=\"M5 0 L5 10\"/></svg>"), "v.svg", opt, {});
    REQUIRE(r.ok);
    CHECK(r.scale == Approx(5.0));
    CHECK(r.placedBox.height() == Approx(50.0));
    // A horizontal line is as wide as the fit box is wide: 100 / 10.
    r = insertClipartBytes(doc, bytesOf(head + "<path d=\"M0 5 L10 5\"/></svg>"), "h.svg", opt, {});
    REQUIRE(r.ok);
    CHECK(r.scale == Approx(10.0));
    CHECK(r.placedBox.width() == Approx(100.0));
}

TEST_CASE("clipart: a failure after the macro recorded steps rolls the insert back exactly", "[clipart][insert][hostile]") {
    // Object ids are never reused (the `.carv` records the next free id), so a rolled-back insert
    // leaves the id counter advanced. The reference below advances it identically: the same insert
    // succeeds and is then discarded with `undoAndDiscard`; the failed one must end byte-identical.
    const auto build = [](document::Document& d) {
        REQUIRE(insertClipartBytes(d, bytesOf(svgRect("10mm", "10mm")), "first.svg", {}, {}).ok);
    };
    const std::string cases[2][2] = {{svgRect("20mm", "10mm"), "second.svg"}, {dxfLines(3, "NEWLAYER"), "lines.dxf"}};
    for (const auto& c : cases) {
        document::Document doc;
        build(doc);
        document::Document ref;
        build(ref);
        REQUIRE(insertClipartBytes(ref, bytesOf(c[0]), c[1], {}, {}).ok);
        REQUIRE(ref.undoStack().undoAndDiscard(ref));
        ref.selection().assign(doc.selection().ids());
        const auto expected = document::saveCarv(ref);
        REQUIRE(expected.ok());

        const std::size_t steps = doc.undoStack().count();
        const std::size_t index = doc.undoStack().index();
        const auto selection = doc.selection().ids();
        const std::size_t layers = doc.layers().size();
        const std::size_t objects = doc.objectCount();
        int hookCalls = 0;
        ClipartInsertOptions opt;
        opt.afterImport = [&](document::Document& d) {
            ++hookCalls;
            CHECK(d.undoStack().inMacro());
            CHECK(d.objectCount() > objects);  // the importer's objects are in the document by now
            throw std::runtime_error("boom");
        };
        const ClipartInsertOutcome r = insertClipartBytes(doc, bytesOf(c[0]), c[1], opt, {});
        CHECK(hookCalls == 1);
        CHECK_FALSE(r.ok);
        CHECK(r.error.find("boom") != std::string::npos);
        CHECK(doc.undoStack().count() == steps);
        CHECK(doc.undoStack().index() == index);
        CHECK_FALSE(doc.undoStack().inMacro());
        CHECK_FALSE(doc.undoStack().canRedo());  // discarded, not merely undone
        CHECK(doc.undoStack().undoText() == "Insert clipart");
        CHECK(doc.objectCount() == objects);
        CHECK(doc.layers().size() == layers);
        CHECK(doc.selection().ids() == selection);
        const auto after = document::saveCarv(doc);
        REQUIRE(after.ok());
        CHECK(after.value() == expected.value());
        // The document still works afterwards: a normal insert is one more step.
        REQUIRE(insertClipartBytes(doc, bytesOf(c[0]), c[1], {}, {}).ok);
        CHECK(doc.undoStack().count() == steps + 1);
    }
}

TEST_CASE("clipart: an exception thrown inside the importer step rolls the insert back", "[clipart][insert][hostile]") {
    // The bitmap importer calls the injected decoder while the insert's macro is open, so a throwing
    // decoder is a throw from inside the importer step (the afterImport hook only runs after it).
    document::Document doc;
    const std::size_t steps = doc.undoStack().count();
    const std::size_t index = doc.undoStack().index();
    const std::size_t layers = doc.layers().size();
    int calls = 0;
    const document::BitmapDecoder throwing = [&calls](std::span<const std::uint8_t>) -> geometry::ShapeResult<io::RasterImage> {
        ++calls;
        throw std::runtime_error("decoder exploded");
    };
    const ClipartInsertOutcome r = insertClipartBytes(doc, bytesOf(pngHeader(40, 10)), "pic.png", {}, throwing);
    CHECK(calls == 1);
    CHECK_FALSE(r.ok);
    CHECK(r.error.find("decoder exploded") != std::string::npos);
    CHECK_FALSE(doc.undoStack().inMacro());
    CHECK(doc.undoStack().count() == steps);
    CHECK(doc.undoStack().index() == index);
    CHECK_FALSE(doc.undoStack().canRedo());
    CHECK(doc.objectCount() == 0);
    CHECK(doc.bitmaps().empty());
    CHECK(doc.layers().size() == layers);
    FakeDecoder dec;  // the document still works afterwards
    REQUIRE(insertClipartBytes(doc, bytesOf(pngHeader(40, 10)), "pic.png", {}, dec.fn()).ok);
    CHECK(doc.undoStack().count() == steps + 1);
}

TEST_CASE("clipart: a line (zero width) is scaled by its length", "[clipart][insert]") {
    document::Document doc;
    const std::string svg =
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"10mm\" height=\"10mm\" viewBox=\"0 0 10 10\"><path d=\"M5 0 L5 10\"/></svg>";
    ClipartInsertOptions opt;
    ClipartInsertOutcome r = insertClipartBytes(doc, bytesOf(svg), "v.svg", opt, {});
    REQUIRE(r.ok);
    CHECK(r.scale == Approx(15.0));
    CHECK(r.placedBox.height() == Approx(150.0));
    const std::string hor =
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"10mm\" height=\"10mm\" viewBox=\"0 0 10 10\"><path d=\"M0 5 L10 5\"/></svg>";
    r = insertClipartBytes(doc, bytesOf(hor), "h.svg", opt, {});
    REQUIRE(r.ok);
    CHECK(r.scale == Approx(15.0));
    CHECK(r.placedBox.width() == Approx(150.0));
}

TEST_CASE("clipart: inserting a DXF adds layers inside the same single undo step", "[clipart][insert]") {
    document::Document doc;
    const std::size_t layers = doc.layers().size();
    const std::size_t steps = doc.undoStack().count();
    ClipartInsertOptions opt;
    opt.at = geometry::Vec2{50.0, 60.0};
    const ClipartInsertOutcome r = insertClipartBytes(doc, bytesOf(dxfLines(3, "CLIPLAYER")), "lines.dxf", opt, {});
    REQUIRE(r.ok);
    CHECK(r.kind == ClipartKind::Dxf);
    CHECK(r.objectsAdded == 3);
    CHECK(doc.layers().size() == layers + 1);
    CHECK(doc.undoStack().count() == steps + 1);
    CHECK(doc.undoStack().undoText() == "Insert clipart");
    CHECK(r.placedBox.centre().x == Approx(50.0));
    CHECK(r.placedBox.centre().y == Approx(60.0));
    CHECK(std::max(r.placedBox.width(), r.placedBox.height()) == Approx(150.0));
    CHECK(doc.selection().ids().size() == 3);
    REQUIRE(doc.undo());
    CHECK(doc.objectCount() == 0);
    CHECK(doc.layers().size() == layers);
    CHECK_FALSE(doc.undo());  // nothing more to undo: the whole insert was one step
}

TEST_CASE("clipart: inserting a PNG embeds it scaled to fit", "[clipart][insert]") {
    document::Document doc;
    FakeDecoder dec;
    dec.w = 40;
    dec.h = 10;
    const std::size_t steps = doc.undoStack().count();
    ClipartInsertOptions opt;
    opt.at = geometry::Vec2{10.0, 10.0};
    const ClipartInsertOutcome r = insertClipartBytes(doc, bytesOf(pngHeader(40, 10)), "dir/pic.png", opt, dec.fn());
    REQUIRE(r.ok);
    CHECK(r.kind == ClipartKind::Png);
    CHECK(r.objectsAdded == 1);
    CHECK(doc.undoStack().count() == steps + 1);
    CHECK(r.placedBox.width() == Approx(150.0));
    CHECK(r.placedBox.height() == Approx(37.5));
    CHECK(r.placedBox.centre().x == Approx(10.0));
    CHECK(r.placedBox.centre().y == Approx(10.0));
    CHECK(doc.bitmaps().size() == 1);
    const auto id = doc.objects().begin()->first;
    REQUIRE(doc.findObject(id)->asBitmap() != nullptr);
    CHECK(doc.findObject(id)->name == "pic.png");
    REQUIRE(doc.undo());
    CHECK(doc.objectCount() == 0);
    CHECK(doc.bitmaps().empty());
    CHECK_FALSE(doc.undo());
}

TEST_CASE("clipart: refused insertions leave the document untouched", "[clipart][insert][hostile]") {
    document::Document doc;
    const std::uint64_t gen = doc.undoStack().generation();
    const std::size_t steps = doc.undoStack().count();
    ClipartInsertOptions opt;
    FakeDecoder dec;
    const auto refused = [&](const std::string& bytes, const std::string& name) {
        const ClipartInsertOutcome r = insertClipartBytes(doc, bytesOf(bytes), name, opt, dec.fn());
        CHECK_FALSE(r.ok);
        CHECK_FALSE(r.error.empty());
        CHECK(doc.objectCount() == 0);
        CHECK(doc.undoStack().count() == steps);
        CHECK(doc.undoStack().generation() == gen);
        return r.error;
    };
    CHECK(refused("PK\x03\x04 bomb", "bomb.svg").find("zip") != std::string::npos);
    CHECK(refused(pngHeader(0xFFFFFFFFU, 0xFFFFFFFFU), "bomb.png").find("pixels") != std::string::npos);
    CHECK(refused("<svg xmlns=\"http://www.w3.org/2000/svg\"/>", "empty.svg").find("no vectors") != std::string::npos);
    CHECK(refused(dxfLines(0), "empty.dxf").find("no vectors") != std::string::npos);
    CHECK(refused("hello", "x.svg").find("not SVG, DXF or PNG") != std::string::npos);
    CHECK(refused(svgRect("1mm", "1mm"), "x.dxf").find("rename") != std::string::npos);
    CHECK(refused(svgRect("1mm", "1mm"), "x.pdf").find("supported") != std::string::npos);
    std::string deep = "<svg xmlns=\"http://www.w3.org/2000/svg\">";
    for (int i = 0; i < 300; ++i) {
        deep += "<g>";
    }
    deep += "<rect width=\"1\" height=\"1\"/>";
    CHECK(refused(deep, "deep.svg").find("SVG") != std::string::npos);
    // A drawing with too many shapes.
    opt.limits.maxContours = 2;
    CHECK(refused(dxfLines(3), "many.dxf").find("3 shapes") != std::string::npos);
    opt.limits.maxContours = 20'000;
    // A bitmap the decoder rejects: nothing remains of the attempt.
    const document::BitmapDecoder failing = [](std::span<const std::uint8_t>) -> geometry::ShapeResult<io::RasterImage> {
        return std::string("damaged");
    };
    const ClipartInsertOutcome bm = insertClipartBytes(doc, bytesOf(pngHeader(4, 4)), "bad.png", opt, failing);
    CHECK_FALSE(bm.ok);
    CHECK(doc.undoStack().count() == steps);
    CHECK(doc.bitmaps().empty());
    // Absurdly large geometry that is finite still scales to fit (and does not break).
    const std::string huge =
        "0\nSECTION\n2\nENTITIES\n0\nLINE\n8\n0\n10\n0\n20\n0\n11\n1e300\n21\n1e300\n0\nENDSEC\n0\nEOF\n";
    const ClipartInsertOutcome h = insertClipartBytes(doc, bytesOf(huge), "huge.dxf", opt, {});
    if (!h.ok) {
        CHECK(doc.objectCount() == 0);
        CHECK(doc.undoStack().count() == steps);
    }
}

TEST_CASE("clipart: a hidden or locked active layer refuses insertion", "[clipart][insert]") {
    document::Document doc;
    document::Layer l = *doc.findLayer(doc.activeLayer());
    l.locked = true;
    REQUIRE(doc.execute(std::make_unique<document::SetLayerPropertiesCommand>(l)));
    const std::size_t steps = doc.undoStack().count();
    const ClipartInsertOutcome r = insertClipartBytes(doc, bytesOf(svgRect("1mm", "1mm")), "a.svg", {}, {});
    CHECK_FALSE(r.ok);
    CHECK(r.error.find("locked") != std::string::npos);
    CHECK(doc.undoStack().count() == steps);
}

TEST_CASE("clipart: insertClipartFile reads from disk", "[clipart][insert]") {
    TempDir dir("insertfile");
    write(dir.file("a.svg"), svgRect("20mm", "10mm"));
    write(dir.file("zip.svg"), "PK\x03\x04");
    document::Document doc;
    CHECK(insertClipartFile(doc, dir.file("a.svg"), {}, {}).ok);
    const ClipartInsertOutcome bad = insertClipartFile(doc, dir.file("zip.svg"), {}, {});
    CHECK_FALSE(bad.ok);
    CHECK(bad.error.rfind("zip.svg: ", 0) == 0);
    CHECK_FALSE(insertClipartFile(doc, dir.file("missing.svg"), {}, {}).ok);
    CHECK(doc.objectCount() == 1);
}

TEST_CASE("clipart: a request made after clear() survives the stale result of the old one", "[clipart][thumbnail][service]") {
    TempDir dir("servicestale");
    write(dir.file("c.png"), pngHeader(4, 2));
    std::mutex m;
    std::condition_variable cv;
    int entered = 0;
    int allowed = 0;
    FakeDecoder inner;
    const document::BitmapDecoder gated = [&](std::span<const std::uint8_t> b) -> geometry::ShapeResult<io::RasterImage> {
        std::unique_lock<std::mutex> lock(m);
        ++entered;
        cv.notify_all();
        cv.wait(lock, [&] { return allowed >= entered; });
        lock.unlock();
        return inner.fn()(b);
    };
    ClipartThumbnailService svc({}, gated);
    const std::string path = dir.file("c.png");
    const auto waitEntered = [&](int n) {
        std::unique_lock<std::mutex> lock(m);
        cv.wait(lock, [&] { return entered >= n; });
    };
    const auto allow = [&](int n) {
        {
            std::lock_guard<std::mutex> lock(m);
            allowed = n;
        }
        cv.notify_all();
    };
    CHECK(svc.request(path));
    waitEntered(1);  // the first render is running
    svc.clear();
    CHECK(svc.state(path) == ClipartThumbState::Unknown);
    CHECK(svc.request(path));  // queued behind the running one
    allow(1);                  // the stale first render finishes
    waitEntered(2);            // the worker took the second request, so the stale result was handled
    CHECK(svc.state(path) == ClipartThumbState::Pending);
    CHECK(svc.processed() == 1);
    allow(2);
    svc.waitIdle();
    CHECK(svc.state(path) == ClipartThumbState::Ready);
    CHECK(svc.processed() == 2);
    CHECK(svc.takeCompleted().size() == 1);  // only the fresh result is reported
}
