// T-0331: gadget manifests, shortcuts, discovery and install (ADR-0108).
#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>
#include <set>
#include <vector>
#include <string>

#include "carvetto/scripting/gadget_api.hpp"
#include "carvetto/scripting/gadget_manifest.hpp"

using namespace carvetto::scripting;
namespace fs = std::filesystem;

namespace {

bool contains(const std::string& s, const char* part) {
    return s.find(part) != std::string::npos;
}

/// A fresh scratch folder named for its test case (test cases never run twice at once), removed
/// before and after.
struct Scratch {
    fs::path root;
    explicit Scratch(const std::string& name) : root(fs::temp_directory_path() / ("carvetto-t0331-" + name)) {
        std::error_code ec;
        fs::remove_all(root, ec);
        fs::create_directories(root);
    }
    ~Scratch() {
        std::error_code ec;
        fs::remove_all(root, ec);
    }
    Scratch(const Scratch&) = delete;
    Scratch& operator=(const Scratch&) = delete;
};

void write(const fs::path& p, const std::string& text) {
    fs::create_directories(p.parent_path());
    std::ofstream out(p, std::ios::binary);
    out << text;
}

std::string manifest(const std::string& name, const std::string& extra = "", int api = 1) {
    return "{\"name\": \"" + name + "\", \"api_version\": " + std::to_string(api) + extra + "}";
}

void makeGadget(const fs::path& root, const std::string& folder, const std::string& manifestText,
                const std::string& script = "return 1") {
    write(root / folder / kGadgetManifestFile, manifestText);
    write(root / folder / kGadgetScriptFile, script);
}

}  // namespace

TEST_CASE("gadget manifest: parsing and validation", "[gadget][manifest][t0331]") {
    SECTION("a minimal manifest") {
        const auto r = parseGadgetManifest(manifest("Weave"));
        REQUIRE(r.manifest.has_value());
        CHECK(r.manifest->name == "Weave");
        CHECK(r.manifest->menu == "Weave");  // defaults to the name
        CHECK(r.manifest->version == "1");
        CHECK(r.manifest->shortcut.empty());
    }
    SECTION("all fields, unknown keys ignored") {
        const auto r = parseGadgetManifest(
            manifest("Weave", ", \"version\": \"2.1\", \"menu\": \"Patterns/Weave\", \"shortcut\": \"ctrl+f5\", "
                              "\"description\": \"d\", \"future\": [1]"));
        REQUIRE(r.manifest.has_value());
        CHECK(r.manifest->version == "2.1");
        CHECK(r.manifest->shortcut == "Ctrl+F5");
        CHECK(r.manifest->description == "d");
        CHECK(menuPath(*r.manifest) == std::vector<std::string>{"Patterns", "Weave"});
    }
    SECTION("rejections carry a reason") {
        const char* bad[] = {
            "",
            "[]",
            "{",
            "{\"api_version\": 1}",
            "{\"name\": \"\", \"api_version\": 1}",
            "{\"name\": \"   \", \"api_version\": 1}",
            "{\"name\": 5, \"api_version\": 1}",
            "{\"name\": \"x\"}",
            "{\"name\": \"x\", \"api_version\": 0}",
            "{\"name\": \"x\", \"api_version\": 1.5}",
            "{\"name\": \"x\", \"api_version\": \"1\"}",
            "{\"name\": \"x\", \"api_version\": 1, \"shortcut\": \"Alt+F5\"}",
            "{\"name\": \"x\", \"api_version\": 1, \"shortcut\": \"F5\"}",
            "{\"name\": \"x\", \"api_version\": 1, \"menu\": \"a//b\"}",
            "{\"name\": \"x\", \"api_version\": 1, \"menu\": \"a/b/c/d/e\"}",
            "{\"name\": \"x\\u0001\", \"api_version\": 1}",
        };
        for (const char* b : bad) {
            INFO(b);
            const auto r = parseGadgetManifest(b);
            CHECK_FALSE(r.manifest.has_value());
            CHECK_FALSE(r.error.empty());
        }
        CHECK_FALSE(parseGadgetManifest(std::string(kMaxManifestBytes + 1, ' ')).manifest.has_value());
    }
}

TEST_CASE("gadget manifest: shortcuts are Ctrl+F1..F12, optionally with Shift", "[gadget][manifest][t0331]") {
    CHECK(normaliseShortcut("Ctrl+F1") == "Ctrl+F1");
    CHECK(normaliseShortcut("ctrl + f12") == "Ctrl+F12");
    CHECK(normaliseShortcut("CTRL+SHIFT+F9") == "Ctrl+Shift+F9");
    CHECK(normaliseShortcut("shift+ctrl+f10") == "Ctrl+Shift+F10");
    CHECK(normaliseShortcut("Ctrl+F13") == std::nullopt);
    CHECK(normaliseShortcut("Ctrl+F0") == std::nullopt);
    CHECK(normaliseShortcut("Ctrl+F01") == std::nullopt);
    CHECK(normaliseShortcut("Ctrl+F") == std::nullopt);
    CHECK(normaliseShortcut("Ctrl+A") == std::nullopt);
    CHECK(normaliseShortcut("Shift+F5") == std::nullopt);
    CHECK(normaliseShortcut("Ctrl+Ctrl+F5") == std::nullopt);
    CHECK(normaliseShortcut("Ctrl+Alt+F5") == std::nullopt);
    CHECK(normaliseShortcut("") == std::nullopt);
    CHECK(normaliseShortcut("Ctrl+") == std::nullopt);
    CHECK(normaliseShortcut("Ctrl+F1x") == std::nullopt);
}

TEST_CASE("gadget discovery: folders, order, problems, shortcut conflicts", "[gadget][manifest][t0331]") {
    Scratch s("discovery");
    makeGadget(s.root, "b-second", manifest("Second", ", \"shortcut\": \"Ctrl+F5\""));
    makeGadget(s.root, "a-first", manifest("First", ", \"shortcut\": \"Ctrl+F5\", \"menu\": \"Group/First\""));
    makeGadget(s.root, "c-third", manifest("Third", ", \"shortcut\": \"Ctrl+F6\""));
    makeGadget(s.root, "d-future", manifest("Future", ", \"shortcut\": \"Ctrl+F7\"", 99));
    write(s.root / "e-nomanifest" / kGadgetScriptFile, "return 1");
    write(s.root / "f-badjson" / kGadgetManifestFile, "{not json");
    write(s.root / "g-noscript" / kGadgetManifestFile, manifest("NoScript"));
    write(s.root / ".hidden" / kGadgetManifestFile, manifest("Hidden"));
    write(s.root / "stray.txt", "x");

    const GadgetDiscovery d = discoverGadgets(s.root);
    REQUIRE(d.gadgets.size() == 5);
    CHECK(d.gadgets[0].folderName == "a-first");
    CHECK(d.gadgets[1].folderName == "b-second");
    CHECK(d.gadgets[2].folderName == "c-third");
    CHECK(d.gadgets[3].folderName == "d-future");
    CHECK(d.gadgets[4].folderName == "g-noscript");

    CHECK(d.gadgets[0].shortcut == "Ctrl+F5");  // the first in folder order keeps it
    CHECK(d.gadgets[1].shortcut.empty());
    REQUIRE(d.gadgets[1].notes.size() == 1);
    CHECK(contains(d.gadgets[1].notes[0], "already used by \"First\""));
    CHECK(d.gadgets[1].runnable());
    CHECK(d.gadgets[2].shortcut == "Ctrl+F6");

    CHECK_FALSE(d.gadgets[3].runnable());  // needs a newer API
    CHECK(contains(d.gadgets[3].problem, "version 99"));
    CHECK(d.gadgets[3].shortcut.empty());
    CHECK_FALSE(d.gadgets[4].runnable());
    CHECK(contains(d.gadgets[4].problem, "gadget.lua is missing"));

    REQUIRE(d.skipped.size() == 2);
    CHECK(contains(d.skipped[0], "e-nomanifest"));
    CHECK(contains(d.skipped[1], "f-badjson"));
}

TEST_CASE("gadget discovery: an unrunnable gadget does not take a shortcut", "[gadget][manifest][t0331]") {
    Scratch s("shortcut-owner");
    makeGadget(s.root, "a", manifest("Future", ", \"shortcut\": \"Ctrl+F5\"", 50));
    makeGadget(s.root, "b", manifest("Real", ", \"shortcut\": \"Ctrl+F5\""));
    const GadgetDiscovery d = discoverGadgets(s.root);
    REQUIRE(d.gadgets.size() == 2);
    CHECK(d.gadgets[1].shortcut == "Ctrl+F5");
}

TEST_CASE("gadget discovery: missing folder, symlinks and sources", "[gadget][manifest][t0331]") {
    CHECK(discoverGadgets(fs::temp_directory_path() / "carvetto-t0331-does-not-exist").gadgets.empty());

    Scratch s("sources");
    makeGadget(s.root, "a", manifest("A"), "return 42");
    const GadgetDiscovery d = discoverGadgets(s.root);
    REQUIRE(d.gadgets.size() == 1);
    std::string text;
    CHECK(readGadgetSource(d.gadgets[0], 1000, &text).empty());
    CHECK(text == "return 42");
    CHECK_FALSE(readGadgetSource(d.gadgets[0], 3, &text).empty());  // over the limit

    std::error_code ec;
    fs::create_directory_symlink(s.root / "a", s.root / "link", ec);
    if (!ec) {
        CHECK(discoverGadgets(s.root).gadgets.size() == 1);  // the link is not followed
    }
}

TEST_CASE("gadget install: copies manifest and script, refuses duplicates", "[gadget][manifest][t0331]") {
    Scratch src("install-src");
    Scratch dst("install-dst");
    makeGadget(src.root, "pkg", manifest("My Cool Gadget!", ", \"shortcut\": \"Ctrl+F2\""), "return 7");
    write(src.root / "pkg" / "extra.bin", "do not copy");

    const InstallResult seen = inspectGadgetFolder(src.root / "pkg", 1000);
    REQUIRE(seen.ok);
    CHECK(seen.gadget.manifest.name == "My Cool Gadget!");
    CHECK(fs::is_empty(dst.root));  // inspecting copies nothing

    InstallResult r = installGadget(src.root / "pkg", dst.root / "gadgets", 1000);
    INFO(r.error);
    REQUIRE(r.ok);
    const std::string folder = gadgetFolderName("My Cool Gadget!");
    CHECK(r.gadget.folderName == folder);
    CHECK(fs::exists(dst.root / "gadgets" / folder / kGadgetManifestFile));
    CHECK(fs::exists(dst.root / "gadgets" / folder / kGadgetScriptFile));
    CHECK_FALSE(fs::exists(dst.root / "gadgets" / folder / "extra.bin"));
    CHECK(discoverGadgets(dst.root / "gadgets").gadgets.size() == 1);

    r = installGadget(src.root / "pkg", dst.root / "gadgets", 1000);
    CHECK_FALSE(r.ok);
    CHECK(contains(r.error, "already installed"));

    write(src.root / "pkg" / kGadgetScriptFile, "return 8");
    r = installGadget(src.root / "pkg", dst.root / "gadgets", 1000, true);
    REQUIRE(r.ok);
    std::string text;
    CHECK(readGadgetSource(r.gadget, 1000, &text).empty());
    CHECK(text == "return 8");

    // nothing half-installed is left behind
    for (const auto& e : fs::directory_iterator(dst.root / "gadgets")) {
        CHECK(e.path().filename().string()[0] != '.');
    }
}

TEST_CASE("gadget install: bad packages are refused before anything is copied", "[gadget][manifest][t0331]") {
    Scratch src("install-bad-src");
    Scratch dst("install-bad-dst");
    makeGadget(src.root, "noscript", manifest("X"));
    fs::remove(src.root / "noscript" / kGadgetScriptFile);
    makeGadget(src.root, "badmanifest", "{}");
    makeGadget(src.root, "big", manifest("Big"), std::string(2000, 'x'));
    makeGadget(src.root, "future", manifest("Future", "", 7));

    CHECK(contains(installGadget(src.root / "noscript", dst.root, 1000).error, "gadget.lua"));
    CHECK_FALSE(installGadget(src.root / "badmanifest", dst.root, 1000).ok);
    CHECK(contains(installGadget(src.root / "big", dst.root, 1000).error, "too large"));
    CHECK(contains(installGadget(src.root / "future", dst.root, 1000).error, "version 7"));
    CHECK_FALSE(installGadget(src.root / "missing", dst.root, 1000).ok);
    CHECK(discoverGadgets(dst.root).gadgets.empty());
}

TEST_CASE("gadget install: the folder name cannot escape the gadgets folder", "[gadget][manifest][t0331]") {
    Scratch src("install-escape-src");
    Scratch dst("install-escape-dst");
    makeGadget(src.root, "p", manifest("../../evil/.."), "return 1");
    const InstallResult r = installGadget(src.root / "p", dst.root / "g", 1000);
    REQUIRE(r.ok);
    CHECK(r.gadget.folder.parent_path() == dst.root / "g");
    CHECK(r.gadget.folderName.rfind("evil-", 0) == 0);
    makeGadget(src.root, "q", manifest("..."), "return 1");
    const InstallResult q = installGadget(src.root / "q", dst.root / "g", 1000);
    REQUIRE(q.ok);
    CHECK(q.gadget.folderName.rfind("gadget-", 0) == 0);  // a name that vanishes gets a hash
}

TEST_CASE("gadget install: folder names are unique, stable, ASCII and safe on Windows", "[gadget][manifest][t0331]") {
    // readable slug plus a 16-hex hash of the exact name
    const std::string weave = gadgetFolderName("Weave Creator");
    CHECK(weave.rfind("weave-creator-", 0) == 0);
    CHECK(weave.size() == std::string("weave-creator-").size() + 16);
    CHECK(weave == gadgetFolderName("Weave Creator"));  // stable
    // two different non-ASCII names do not share a folder
    const std::string a = gadgetFolderName("\xE3\x83\x84\xE3\x83\xBC\xE3\x83\xAB");
    const std::string b = gadgetFolderName("\xE3\x82\xAE\xE3\x83\xA3\xE3\x83\x89\xE3\x82\xB8\xE3\x82\xA7");
    CHECK(a != b);
    CHECK(a == gadgetFolderName("\xE3\x83\x84\xE3\x83\xBC\xE3\x83\xAB"));
    CHECK(gadgetFolderName("Caf\xC3\xA9") != gadgetFolderName("Caf\xC3\xA8"));
    CHECK(gadgetFolderName("Caf\xC3\xA9").rfind("caf-", 0) == 0);
    // every collision pair that plain sanitising used to merge
    const std::string longA = std::string(60, 'x') + "A";
    const std::string longB = std::string(60, 'x') + "B";
    const std::vector<std::string> distinct = {"My Gadget", "my-gadget", "My_Gadget", "my gadget!", "a/b", "a\\b", "CON",
                                               "con gadget", "con", "con-gadget", "A", "a", longA, longB, "", "...", "!!",
                                               "Caf\xC3\xA9", "Caf\xC3\xA8", "name", "name."};
    std::set<std::string> seen;
    for (const std::string& n : distinct) {
        INFO(n);
        const std::string f = gadgetFolderName(n);
        CHECK(seen.insert(f).second);
        CHECK(f.size() <= 57);
        CHECK_FALSE(f.empty());
        for (const char c : f) {
            CHECK(((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-'));
        }
        CHECK(f.back() != '-');
        CHECK(f.front() != '-');
    }
    CHECK(gadgetFolderName("").rfind("gadget-", 0) == 0);
    CHECK(gadgetFolderName(longA).size() <= 57);
    // Windows device names, whatever the case, never come out bare
    for (const char* n : {"CON", "nul", "Prn", "AUX", "com1", "COM9", "LPT1", "lpt9", "con.", "NUL  "}) {
        INFO(n);
        const std::string f = gadgetFolderName(n);
        CHECK(f.size() > 16);
        CHECK(f.find('-') != std::string::npos);
    }
    // two different non-ASCII gadgets install side by side
    Scratch src("install-unicode-src");
    Scratch dst("install-unicode-dst");
    makeGadget(src.root, "p", manifest("\xE3\x83\x84\xE3\x83\xBC\xE3\x83\xAB"), "return 1");
    makeGadget(src.root, "q", manifest("\xE3\x82\xAE\xE3\x83\xA3\xE3\x83\x89"), "return 2");
    CHECK(installGadget(src.root / "p", dst.root, 1000).ok);
    const InstallResult second = installGadget(src.root / "q", dst.root, 1000);
    INFO(second.error);
    CHECK(second.ok);
    CHECK(discoverGadgets(dst.root).gadgets.size() == 2);
}

TEST_CASE("gadget install: a failed replace keeps the old gadget", "[gadget][manifest][t0331]") {
    Scratch src("install-replace-src");
    Scratch dst("install-replace-dst");
    makeGadget(src.root, "pkg", manifest("Keep Me"), "return 'old'");
    REQUIRE(installGadget(src.root / "pkg", dst.root, 1000).ok);
    write(src.root / "pkg" / kGadgetScriptFile, "return 'new'");

    const auto scriptOf = [&](std::string* text) {
        const GadgetDiscovery d = discoverGadgets(dst.root);
        REQUIRE(d.gadgets.size() == 1);
        return readGadgetSource(d.gadgets[0], 1000, text);
    };
    // the move of the new gadget into place fails: the old one is still there, nothing is left over
    int calls = 0;
    const RenameFn failSecond = [&calls](const fs::path& from, const fs::path& to) {
        if (++calls == 2) {
            return std::make_error_code(std::errc::permission_denied);
        }
        std::error_code ec;
        fs::rename(from, to, ec);
        return ec;
    };
    InstallResult r = installGadget(src.root / "pkg", dst.root, 1000, true, failSecond);
    CHECK_FALSE(r.ok);
    CHECK(contains(r.error, "cannot move the gadget into place"));
    std::string text;
    CHECK(scriptOf(&text).empty());
    CHECK(text == "return 'old'");
    for (const auto& e : fs::directory_iterator(dst.root)) {
        CHECK(e.path().filename().string()[0] != '.');
    }
    // moving the old one aside fails: the old one is untouched
    const RenameFn failFirst = [](const fs::path&, const fs::path&) {
        return std::make_error_code(std::errc::permission_denied);
    };
    r = installGadget(src.root / "pkg", dst.root, 1000, true, failFirst);
    CHECK_FALSE(r.ok);
    CHECK(scriptOf(&text).empty());
    CHECK(text == "return 'old'");
    // and a good replace still works and leaves nothing behind
    r = installGadget(src.root / "pkg", dst.root, 1000, true);
    REQUIRE(r.ok);
    CHECK(scriptOf(&text).empty());
    CHECK(text == "return 'new'");
    for (const auto& e : fs::directory_iterator(dst.root)) {
        CHECK(e.path().filename().string()[0] != '.');
    }
}

TEST_CASE("gadget install: a different gadget in the same folder is never replaced", "[gadget][manifest][t0331]") {
    Scratch src("install-differ-src");
    Scratch dst("install-differ-dst");
    makeGadget(src.root, "one", manifest("My Gadget"), "return 1");
    makeGadget(src.root, "two", manifest("My_Gadget"), "return 2");
    REQUIRE(installGadget(src.root / "one", dst.root, 1000).ok);
    REQUIRE(installGadget(src.root / "two", dst.root, 1000).ok);  // different folders: side by side
    CHECK(discoverGadgets(dst.root).gadgets.size() == 2);

    // Force a folder shared by two different names (a hash collision, or a hand-edited folder):
    // the manifest name check refuses even with replace, and the installed gadget is untouched.
    const fs::path existing = dst.root / gadgetFolderName("Other One");
    write(existing / kGadgetManifestFile, manifest("Someone Else"));
    write(existing / kGadgetScriptFile, "return 'mine'");
    makeGadget(src.root, "three", manifest("Other One"), "return 3");
    for (const bool replace : {false, true}) {
        const InstallResult r = installGadget(src.root / "three", dst.root, 1000, replace);
        CHECK_FALSE(r.ok);
        CHECK(contains(r.error, "different gadget"));
        CHECK(contains(r.error, "Someone Else"));
        CHECK_FALSE(contains(r.error, "already installed"));  // so the UI offers no Replace
        std::string text;
        std::ifstream in(existing / kGadgetScriptFile);
        std::getline(in, text);
        CHECK(text == "return 'mine'");
    }
}
