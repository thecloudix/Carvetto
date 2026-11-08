// T-0224: the Save Toolpaths logic (post discovery, machine/units selection, file names, tool
// grouping, preview, errors, writing). The byte-identity with carvetto-cli is in
// save_toolpaths_cli_test.cpp.

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <cerrno>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

#include "carvetto/viewmodel/file_session.hpp"
#include "carvetto/viewmodel/save_toolpaths.hpp"
#include "toolpath_test_support.hpp"

namespace fs = std::filesystem;
using namespace carvetto;
using namespace carvetto::viewmodel;
using namespace carvetto::viewmodel::testing;

namespace {

fs::path workDir(const std::string& name) {
    const fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / ("save_" + name);
    fs::remove_all(dir);
    fs::create_directories(dir);
    return dir;
}

std::vector<fs::path> tempFiles(const fs::path& dir) {
    std::vector<fs::path> out;
    for (const auto& e : fs::directory_iterator(dir)) {
        if (e.path().filename().string().find(".carvetto-tmp") != std::string::npos) {
            out.push_back(e.path());
        }
    }
    return out;
}

SaveOutput oneFile(const fs::path& path, const std::string& text) {
    SaveOutput out;
    out.ok = true;
    OutputFile f;
    f.path = path;
    f.text = text;
    out.files.push_back(f);
    return out;
}

std::vector<PostInfo> shipped() {
    return discoverPosts(fs::path(CARVETTO_POSTS_DIR), {});
}

const PostInfo& postNamed(const std::vector<PostInfo>& posts, const char* file) {
    for (const PostInfo& p : posts) {
        if (p.path.filename() == file) {
            return p;
        }
    }
    FAIL("no post " << file);
    return posts.front();
}

/// A document with a profile (3 mm end mill), a pocket (6 mm end mill) and another profile
/// (3 mm end mill) on a square.
struct Fixture {
    document::Document doc;
    tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session{&doc};
    std::vector<document::ToolpathId> ids;

    Fixture() {
        session.setToolDatabase(db);
        addVector(doc, square(10, 10, 40));
        for (const ToolpathDefinition& d :
             {profileDefinition(db), pocketDefinition(db), profileDefinition(db, 2.0)}) {
            const ToolpathOutcome o = session.add(d);
            REQUIRE(o.ok);
            ids.push_back(o.ids.front());
        }
        doc.toolpaths();  // all three have moves
    }
};

SaveJob jobFor(Fixture& f, const PostInfo& post, OutputMode mode, const fs::path& dir) {
    SaveOptions o;
    o.mode = mode;
    o.directory = dir;
    o.jobName = "part";
    return prepareSave(f.doc, {}, post, o);
}

}  // namespace

TEST_CASE("shipped posts are discovered with their declared name, units and extension", "[save]") {
    const auto posts = shipped();
    REQUIRE(posts.size() == 8);
    for (const PostInfo& p : posts) {
        CHECK(p.usable());
        CHECK(p.shipped);
        CHECK(p.hasToolChange);
    }
    // Sorted by file name.
    CHECK(posts.front().path.filename() == "grbl_inch.pp");
    const PostInfo& grbl = postNamed(posts, "grbl_mm.pp");
    CHECK(grbl.name == "Grbl (mm) (*.nc)");
    CHECK(grbl.extension == "nc");
    CHECK(grbl.units == post::Units::Millimetres);
    CHECK(postNamed(posts, "grbl_inch.pp").units == post::Units::Inches);
    CHECK(postNamed(posts, "mach3_mm.pp").extension == "tap");
    CHECK(postNamed(posts, "linuxcnc_mm.pp").extension == "ngc");
    CHECK(postNamed(posts, "marlin_mm.pp").extension == "gcode");
    CHECK(postsWithUnits(posts, post::Units::Inches).size() == 3);
    CHECK(postsWithUnits(posts, post::Units::Millimetres).size() == 5);
}

TEST_CASE("user posts are listed after the shipped ones; bad ones carry their error", "[save]") {
    const fs::path user = workDir("userposts");
    {
        std::ofstream(user / "mine.pp") << "POST_NAME = \"Mine\"\nFILE_EXTENSION = \"abc\"\nUNITS = \"MM\"\n";
        std::ofstream(user / "broken.pp") << "begin HEADER\n\"[NO_SUCH_TAG]\"\nend\n";
        std::ofstream(user / "notes.txt") << "ignored";
        fs::create_directories(user / "dir.pp");
    }
    const auto posts = discoverPosts(fs::path(CARVETTO_POSTS_DIR), user);
    REQUIRE(posts.size() == 10);
    CHECK_FALSE(posts[8].usable());  // broken.pp sorts before mine.pp
    CHECK(posts[8].error.find("broken.pp") != std::string::npos);
    CHECK_FALSE(posts[8].shipped);
    CHECK(posts[9].name == "Mine");
    CHECK(posts[9].extension == "abc");
    CHECK_FALSE(posts[9].hasToolChange);
    CHECK(discoverPosts({}, fs::path(CARVETTO_VM_WORK_DIR) / "nonexistent").empty());
}

TEST_CASE("a machine selects its posts and default post; units filter", "[save]") {
    const auto posts = shipped();
    tooling::Machine m;
    CHECK(postsForMachine(posts, m).size() == 8);  // lists none: all
    CHECK_FALSE(defaultPostFor(posts, m));
    m.associatedPosts = {"mach3_mm", "Grbl (mm) (*.nc)", "nonexistent", "mach3_mm.pp"};
    m.defaultPost = "grbl_mm.pp";
    const auto assoc = postsForMachine(posts, m);
    REQUIRE(assoc.size() == 2);  // no duplicates, unknown dropped
    CHECK(posts[assoc[0]].path.filename() == "mach3_mm.pp");
    CHECK(posts[assoc[1]].path.filename() == "grbl_mm.pp");
    const auto def = defaultPostFor(posts, m);
    REQUIRE(def);
    CHECK(posts[*def].path.filename() == "grbl_mm.pp");
    m.defaultPost = "";
    CHECK_FALSE(defaultPostFor(posts, m));
}

TEST_CASE("file name stems are made safe", "[save]") {
    CHECK(sanitizeFileStem("part") == "part");
    CHECK(sanitizeFileStem("a/b\\c:d*e?\"f<g>h|i") == "a_b_c_d_e__f_g_h_i");
    CHECK(sanitizeFileStem("../../etc/passwd") == "_.._etc_passwd");
    CHECK(sanitizeFileStem("..") == "");
    CHECK(sanitizeFileStem("  . x . ") == "x");
    CHECK(sanitizeFileStem("con") == "_con");
    CHECK(sanitizeFileStem("NUL.txt") == "_NUL.txt");
    CHECK(sanitizeFileStem("COM1") == "_COM1");
    CHECK(sanitizeFileStem("console") == "console");
    CHECK(sanitizeFileStem(std::string("a\nb\0c", 5)) == "a_b_c");
    CHECK(sanitizeFileStem("caf\xC3\xA9") == "caf\xC3\xA9");
    CHECK(sanitizeFileStem("bad\xFF" "x") == "bad_x");
    CHECK(sanitizeFileStem("\xE2\x82") == "__");  // truncated sequence
    const std::string longName(300, 'x');
    CHECK(sanitizeFileStem(longName).size() == 100);
    std::string utf;
    for (int i = 0; i < 80; ++i) {
        utf += "\xC3\xA9";  // 2 bytes each
    }
    const std::string cut = sanitizeFileStem(utf);
    CHECK(cut.size() == 100);  // 50 whole characters
    CHECK(sanitizeFileStem("x").size() == 1);
}

TEST_CASE("single file: all toolpaths in job order, tool change counted", "[save]") {
    Fixture f;
    const auto posts = shipped();
    const fs::path dir = workDir("single");
    const SaveJob job = jobFor(f, postNamed(posts, "grbl_mm.pp"), OutputMode::SingleFile, dir);
    REQUIRE(job.error.empty());
    const SaveOutput out = generateOutput(job);
    INFO(out.error);
    REQUIRE(out.ok);
    REQUIRE(out.files.size() == 1);
    CHECK(out.files[0].path == dir / "part.nc");
    CHECK(out.files[0].toolpaths.size() == 3);
    CHECK(out.toolChanges == 2);  // 3 mm -> 6 mm -> 3 mm
    CHECK(out.files[0].text.find("G21") != std::string::npos);
    CHECK_FALSE(out.files[0].exists);
    // Deterministic.
    CHECK(generateOutput(job).files[0].text == out.files[0].text);
}

TEST_CASE("a subset of toolpaths is posted in document order", "[save]") {
    Fixture f;
    const auto posts = shipped();
    SaveOptions o;
    o.directory = workDir("subset");
    o.jobName = "p";
    const SaveJob job = prepareSave(f.doc, {f.ids[2], f.ids[0]}, postNamed(posts, "grbl_mm.pp"), o);
    const SaveOutput out = generateOutput(job);
    REQUIRE(out.ok);
    REQUIRE(out.files[0].toolpaths.size() == 2);
    CHECK(out.files[0].toolpaths[0] == f.doc.toolpaths()[0].name);
    CHECK(out.files[0].toolpaths[1] == f.doc.toolpaths()[2].name);
    CHECK(out.toolChanges == 0);
}

TEST_CASE("one file per toolpath: numbered names, collisions get a suffix", "[save]") {
    Fixture f;
    const auto posts = shipped();
    const fs::path dir = workDir("pertoolpath");
    SaveJob job = jobFor(f, postNamed(posts, "mach3_mm.pp"), OutputMode::FilePerToolpath, dir);
    SaveOutput out = generateOutput(job);
    REQUIRE(out.ok);
    REQUIRE(out.files.size() == 3);
    for (std::size_t i = 0; i < 3; ++i) {
        const std::string expect =
            "part_0" + std::to_string(i + 1) + "_" + sanitizeFileStem(f.doc.toolpaths()[i].name) + ".tap";
        CHECK(pathToUtf8(out.files[i].path.filename()) == expect);
        CHECK(out.files[i].toolpaths.size() == 1);
    }
    CHECK(out.toolChanges == 0);

    job.options.nameTemplate = "same";  // identical names collide
    out = generateOutput(job);
    REQUIRE(out.ok);
    CHECK(out.files[0].path.filename() == "same.tap");
    CHECK(out.files[1].path.filename() == "same_2.tap");
    CHECK(out.files[2].path.filename() == "same_3.tap");

    job.options.nameTemplate = "Same{n}";  // names differing only in case collide on Windows/macOS
    job.options.nameTemplate = "{toolpath}";
    CHECK(generateOutput(job).files.size() == 3);
}

TEST_CASE("one file per tool groups toolpaths of the same tool in job order", "[save]") {
    Fixture f;
    const auto posts = shipped();
    const fs::path dir = workDir("pertool");
    const SaveJob job = jobFor(f, postNamed(posts, "linuxcnc_mm.pp"), OutputMode::FilePerTool, dir);
    const SaveOutput out = generateOutput(job);
    REQUIRE(out.ok);
    REQUIRE(out.files.size() == 2);
    CHECK(out.files[0].toolpaths.size() == 2);  // the two profiles with the 3 mm end mill
    CHECK(out.files[0].toolpaths[0] == f.doc.toolpaths()[0].name);
    CHECK(out.files[0].toolpaths[1] == f.doc.toolpaths()[2].name);
    CHECK(out.files[1].toolpaths.size() == 1);
    CHECK(out.toolChanges == 0);
    CHECK(out.files[0].path.extension() == ".ngc");
    CHECK(pathToUtf8(out.files[0].path.filename()).rfind("part_01_", 0) == 0);
    CHECK(pathToUtf8(out.files[1].path.filename()).rfind("part_02_", 0) == 0);
}

TEST_CASE("template errors and dangerous values are caught", "[save]") {
    Fixture f;
    const auto posts = shipped();
    const fs::path dir = workDir("template");
    SaveJob job = jobFor(f, postNamed(posts, "grbl_mm.pp"), OutputMode::SingleFile, dir);

    job.options.nameTemplate = "{bogus}";
    SaveOutput out = generateOutput(job);
    CHECK_FALSE(out.ok);
    CHECK(out.error.find("{bogus}") != std::string::npos);
    CHECK(out.files.empty());

    job.options.nameTemplate = "abc{job";
    CHECK_FALSE(generateOutput(job).ok);
    job.options.nameTemplate = "..";
    out = generateOutput(job);
    CHECK_FALSE(out.ok);
    CHECK(out.error.find("empty") != std::string::npos);

    job.options.nameTemplate = "../{job}/x";
    job.options.jobName = "../../evil";
    out = generateOutput(job);
    REQUIRE(out.ok);
    CHECK(out.files[0].path.parent_path() == dir);  // stays in the chosen folder
    CHECK(out.files[0].path.filename() == "_.._.._evil_x.nc");
    CHECK(pathToUtf8(out.files[0].path.filename()).find('/') == std::string::npos);

    job.options.nameTemplate = "{job}";
    job.options.jobName = "CON";
    out = generateOutput(job);
    REQUIRE(out.ok);
    CHECK(out.files[0].path.filename() == "_CON.nc");
}

TEST_CASE("errors are reported, not thrown", "[save]") {
    Fixture f;
    const auto posts = shipped();
    const fs::path dir = workDir("errors");
    const PostInfo& grbl = postNamed(posts, "grbl_mm.pp");

    {  // no folder / missing folder
        SaveOptions o;
        CHECK(generateOutput(prepareSave(f.doc, {}, grbl, o)).error.find("no output folder") != std::string::npos);
        o.directory = dir / "missing";
        CHECK(generateOutput(prepareSave(f.doc, {}, grbl, o)).error.find("does not exist") != std::string::npos);
    }
    SaveOptions o;
    o.directory = dir;
    {  // no post
        const SaveOutput out = generateOutput(prepareSave(f.doc, {}, PostInfo{}, o));
        CHECK_FALSE(out.ok);
        CHECK(out.error.find("no post-processor") != std::string::npos);
    }
    {  // unusable post
        PostInfo bad = grbl;
        bad.error = "broken";
        CHECK(generateOutput(prepareSave(f.doc, {}, bad, o)).error == "broken");
    }
    {  // post file vanished
        PostInfo gone = grbl;
        gone.path = dir / "gone.pp";
        const SaveOutput out = generateOutput(prepareSave(f.doc, {}, gone, o));
        CHECK_FALSE(out.ok);
        CHECK(out.error.find("gone.pp") != std::string::npos);
    }
    {  // post that fails to run: an unknown tag in a block is a parse error
        std::ofstream(dir / "bad.pp") << "begin HEADER\n\"[NO_SUCH_TAG]\"\nend\n";
        PostInfo bad = grbl;
        bad.path = dir / "bad.pp";
        const SaveOutput out = generateOutput(prepareSave(f.doc, {}, bad, o));
        CHECK_FALSE(out.ok);
        CHECK(out.error.find("bad.pp") != std::string::npos);
        CHECK(out.error.find("line") != std::string::npos);
    }
    {  // no toolpaths
        document::Document empty;
        CHECK(generateOutput(prepareSave(empty, {}, grbl, o)).error.find("no toolpaths") != std::string::npos);
        CHECK(generateOutput(prepareSave(f.doc, {document::ToolpathId{9999}}, grbl, o)).error.find("no toolpaths") !=
              std::string::npos);
    }
    {  // cancelled
        std::atomic<bool> cancel{true};
        const SaveOutput out = generateOutput(prepareSave(f.doc, {}, grbl, o), &cancel);
        CHECK_FALSE(out.ok);
        CHECK(out.error == "cancelled");
    }
}

TEST_CASE("a spindle-less post without TOOLCHANGE warns about tool changes", "[save]") {
    Fixture f;
    const fs::path dir = workDir("notc");
    {  // the Grbl post without its TOOLCHANGE block
        std::string text = readTextFile(std::string(CARVETTO_POSTS_DIR) + "/grbl_mm.pp");
        const auto b = text.find("begin TOOLCHANGE");
        REQUIRE(b != std::string::npos);
        const auto e = text.find("\nend", b);
        REQUIRE(e != std::string::npos);
        text.erase(b, e + 4 - b);
        std::ofstream(dir / "nochange.pp", std::ios::binary) << text;
    }
    const auto posts = discoverPosts({}, dir);
    REQUIRE(posts.size() == 1);
    REQUIRE(posts[0].usable());
    SaveOptions o;
    o.directory = dir;
    const SaveOutput out = generateOutput(prepareSave(f.doc, {}, posts[0], o));
    INFO(out.error);
    REQUIRE(out.ok);
    bool warned = false;
    for (const std::string& w : out.warnings) {
        warned = warned || w.find("no TOOLCHANGE block") != std::string::npos;
    }
    CHECK(warned);
}

TEST_CASE("preview shows the first lines and counts the rest", "[save]") {
    SaveOutput out;
    out.ok = true;
    CHECK(previewText(out, 5).empty());
    out.files.push_back(OutputFile{});
    out.files[0].text = "a\nb\nc\nd\ne\n";
    CHECK(previewText(out, 3) == "a\nb\nc\n... (2 more lines)\n");
    CHECK(previewText(out, 5) == "a\nb\nc\nd\ne\n");
    CHECK(previewText(out, 50) == "a\nb\nc\nd\ne\n");
    CHECK(previewText(out, 0) == "... (5 more lines)\n");
    out.files[0].text = "a\nb";
    CHECK(previewText(out, 1) == "a\n... (1 more lines)\n");
    out.files[0].text = "no newline";
    CHECK(previewText(out, 4) == "no newline");

    Fixture f;
    const auto posts = shipped();
    const SaveOutput real =
        generateOutput(jobFor(f, postNamed(posts, "grbl_mm.pp"), OutputMode::SingleFile, workDir("preview")));
    REQUIRE(real.ok);
    const std::string p = previewText(real, 10);
    CHECK(real.files[0].text.rfind(p.substr(0, p.find("...")), 0) == 0);
    CHECK(p.find("more lines") != std::string::npos);
}

TEST_CASE("writing: bytes as generated, no overwrite by default, all or nothing", "[save]") {
    Fixture f;
    const auto posts = shipped();
    const fs::path dir = workDir("write");
    const SaveJob job = jobFor(f, postNamed(posts, "grbl_mm.pp"), OutputMode::FilePerToolpath, dir);
    SaveOutput out = generateOutput(job);
    REQUIRE(out.ok);
    WriteResult w = writeOutput(out, false);
    INFO(w.error);
    REQUIRE(w.ok);
    REQUIRE(w.written.size() == 3);
    for (const OutputFile& file : out.files) {
        CHECK(readTextFile(pathToUtf8(file.path)) == file.text);
    }
    for (const auto& e : fs::directory_iterator(dir)) {
        CHECK(e.path().extension() == ".nc");  // no staging files left
    }

    // The same again: files exist now.
    out = generateOutput(job);
    REQUIRE(out.ok);
    CHECK(out.files[0].exists);
    w = writeOutput(out, false);
    CHECK_FALSE(w.ok);
    CHECK(w.error.find("exists already") != std::string::npos);
    CHECK(w.written.empty());

    // Replace one file's content to see overwrite change it, and a refused write leave the rest.
    const std::string second = pathToUtf8(out.files[1].path);
    { std::ofstream(out.files[1].path, std::ios::binary) << "old"; }
    fs::remove(out.files[0].path);
    CHECK_FALSE(writeOutput(out, false).ok);  // file 1 still exists: nothing written, file 0 not created
    CHECK_FALSE(fs::exists(out.files[0].path));
    CHECK(readTextFile(second) == "old");
    w = writeOutput(out, true);
    REQUIRE(w.ok);
    CHECK(readTextFile(second) == out.files[1].text);

    // Missing folder, and a failed output.
    SaveOutput gone = out;
    gone.files[0].path = dir / "nope" / "x.nc";
    CHECK_FALSE(writeOutput(gone, true).ok);
    SaveOutput failed;
    failed.error = "boom";
    CHECK(writeOutput(failed, true).error == "boom");
    CHECK_FALSE(writeOutput(SaveOutput{}, true).ok);

    // A target that is a directory cannot be replaced: staged files are removed.
    SaveOutput blocked = out;
    const fs::path blockedPath = dir / "blocked.nc";
    fs::create_directories(blockedPath / "inner");
    blocked.files[0].path = blockedPath;
    w = writeOutput(blocked, true);
    CHECK_FALSE(w.ok);
    CHECK(tempFiles(dir).empty());
}

TEST_CASE("inch posts scale the numbers, mm posts do not", "[save]") {
    Fixture f;
    const auto posts = shipped();
    const SaveOutput mm =
        generateOutput(jobFor(f, postNamed(posts, "grbl_mm.pp"), OutputMode::SingleFile, workDir("units_mm")));
    const SaveOutput in =
        generateOutput(jobFor(f, postNamed(posts, "grbl_inch.pp"), OutputMode::SingleFile, workDir("units_in")));
    REQUIRE(mm.ok);
    REQUIRE(in.ok);
    CHECK(mm.files[0].text.find("G21") != std::string::npos);
    CHECK(in.files[0].text.find("G20") != std::string::npos);
    CHECK(mm.files[0].text != in.files[0].text);
}

TEST_CASE("uncalculated toolpaths are skipped with a warning", "[save]") {
    Fixture f;
    const auto posts = shipped();
    SaveJob job = jobFor(f, postNamed(posts, "grbl_mm.pp"), OutputMode::SingleFile, workDir("uncalc"));
    job.notComputed.push_back("Pending");
    const SaveOutput out = generateOutput(job);
    REQUIRE(out.ok);
    REQUIRE_FALSE(out.warnings.empty());
    CHECK(out.warnings.front().find("'Pending'") != std::string::npos);
}

TEST_CASE("only the session's enabled toolpaths are posted", "[save]") {
    Fixture f;
    const auto posts = shipped();
    REQUIRE(f.session.setEnabled(f.ids[1], false));
    const std::vector<document::ToolpathId> enabled = f.session.enabledToolpaths();
    REQUIRE(enabled.size() == 2);
    SaveOptions o;
    o.directory = workDir("enabled");
    const SaveOutput out = generateOutput(prepareSave(f.doc, enabled, postNamed(posts, "grbl_mm.pp"), o));
    REQUIRE(out.ok);
    REQUIRE(out.files[0].toolpaths.size() == 2);
    const std::string disabledName = f.doc.toolpaths()[1].name;
    for (const std::string& n : out.files[0].toolpaths) {
        CHECK(n != disabledName);
    }
    CHECK(out.files[0].text.find("(" + disabledName + ")") == std::string::npos);
    CHECK(out.toolChanges == 0);  // the 6 mm pocket between the two profiles is gone
}

TEST_CASE("stale and uncalculated enabled toolpaths are reported from the session", "[save]") {
    Fixture f;
    const auto posts = shipped();
    {
        const SaveSource clean = makeSaveSource(f.session);
        CHECK(clean.records.size() == 3);
        CHECK(clean.stale.empty());
        CHECK(clean.notComputed.empty());
    }
    addVector(f.doc, square(100, 100, 20));  // the vectors the toolpaths machine changed: all are stale now
    f.session.invalidate();
    document::ToolpathRecord pending;
    pending.name = "Pending";
    REQUIRE(f.doc.execute(std::make_unique<document::AddToolpathCommand>(pending)));
    const SaveSource src = makeSaveSource(f.session);
    REQUIRE(src.records.size() == 3);
    CHECK(src.stale.size() == 3);
    REQUIRE(src.notComputed.size() == 1);
    CHECK(src.notComputed[0] == "Pending");

    SaveOptions o;
    o.directory = workDir("stale");
    const SaveOutput out = generateOutput(prepareSave(src, postNamed(posts, "grbl_mm.pp"), o));
    REQUIRE(out.ok);
    CHECK(out.stale.size() == 3);
    bool staleWarning = false;
    bool pendingWarning = false;
    for (const std::string& w : out.warnings) {
        staleWarning = staleWarning || (w.find("3 toolpath(s) are out of date") != std::string::npos &&
                                        w.find("old moves") != std::string::npos);
        pendingWarning = pendingWarning || w.find("'Pending' has not been calculated") != std::string::npos;
    }
    CHECK(staleWarning);
    CHECK(pendingWarning);

    // A disabled stale toolpath is neither posted nor reported.
    REQUIRE(f.session.setEnabled(f.ids[0], false));
    CHECK(makeSaveSource(f.session).stale.size() == 2);
}

TEST_CASE("the commit falls back when hard links are not available", "[save][safety]") {
#ifdef _WIN32
    SKIP("POSIX commit steps");
#else
    const fs::path dir = workDir("nolink");
    struct Reset {
        ~Reset() { detail::setTestSeams(); }
    } reset;

    // 1. link() says EPERM (FAT, exFAT): a normal no-overwrite save still works, via the no-replace rename.
    detail::setTestSeams(EPERM);
    WriteResult w = writeOutput(oneFile(dir / "a.nc", "A"), false);
    INFO(w.error);
    REQUIRE(w.ok);
    CHECK(readTextFile(pathToUtf8(dir / "a.nc")) == "A");
    CHECK(tempFiles(dir).empty());

    // ... and still refuses to replace what is there.
    w = writeOutput(oneFile(dir / "a.nc", "B"), false);
    CHECK_FALSE(w.ok);
    CHECK(readTextFile(pathToUtf8(dir / "a.nc")) == "A");
    CHECK(tempFiles(dir).empty());

    // 2. Also without the no-replace rename (lstat + rename).
    for (int errnoValue : {EPERM, ENOTSUP, ENOSYS, EXDEV}) {
        detail::setTestSeams(errnoValue, true);
        const std::string name = "b" + std::to_string(errnoValue) + ".nc";
        w = writeOutput(oneFile(dir / name, "B"), false);
        INFO(errnoValue << ": " << w.error);
        REQUIRE(w.ok);
        CHECK(readTextFile(pathToUtf8(dir / name)) == "B");
        w = writeOutput(oneFile(dir / name, "C"), false);
        CHECK_FALSE(w.ok);
        CHECK(readTextFile(pathToUtf8(dir / name)) == "B");
    }
    CHECK(tempFiles(dir).empty());

    // 3. Symlinks, also dangling, are refused in every variant and never followed.
    const fs::path victim = dir / "victim.txt";
    { std::ofstream(victim) << "precious"; }
    std::error_code ec;
    fs::create_symlink(victim, dir / "link.nc", ec);
    std::error_code ec2;
    fs::create_symlink(dir / "nowhere", dir / "dangling.nc", ec2);
    REQUIRE_FALSE(ec);
    REQUIRE_FALSE(ec2);
    for (bool noRename : {false, true}) {
        detail::setTestSeams(EPERM, noRename);
        CHECK_FALSE(writeOutput(oneFile(dir / "link.nc", "x"), false).ok);
        CHECK_FALSE(writeOutput(oneFile(dir / "dangling.nc", "x"), false).ok);
        CHECK(readTextFile(pathToUtf8(victim)) == "precious");
        CHECK_FALSE(fs::exists(dir / "nowhere"));
        CHECK(tempFiles(dir).empty());
    }

    // 4. A file created between the check and the commit is not replaced (renameat2 path), and the
    // files this call already created are taken away again.
    detail::setTestSeams(EPERM);
    SaveOutput two = oneFile(dir / "first.nc", "1");
    OutputFile second;
    second.path = dir / "second.nc";
    second.text = "2";
    two.files.push_back(second);
    detail::setCommitHook([&](const std::vector<fs::path>&) { std::ofstream(dir / "second.nc") << "someone else"; });
    w = writeOutput(two, false);
    detail::setCommitHook({});
    CHECK_FALSE(w.ok);
    CHECK(w.error.find("exists already") != std::string::npos);
    CHECK(readTextFile(pathToUtf8(dir / "second.nc")) == "someone else");
    CHECK_FALSE(fs::exists(dir / "first.nc"));
    CHECK(tempFiles(dir).empty());

    // 5. Same with the lstat fallback (the file is there when it looks).
    fs::remove(dir / "second.nc");
    detail::setTestSeams(EPERM, true);
    detail::setCommitHook([&](const std::vector<fs::path>&) { std::ofstream(dir / "second.nc") << "someone else"; });
    w = writeOutput(two, false);
    detail::setCommitHook({});
    CHECK_FALSE(w.ok);
    CHECK(readTextFile(pathToUtf8(dir / "second.nc")) == "someone else");
    CHECK_FALSE(fs::exists(dir / "first.nc"));
    CHECK(tempFiles(dir).empty());

    // 6. With overwrite the fallback is not involved and "already replaced" is still reported.
    detail::setTestSeams(EPERM, true);
    fs::create_directories(dir / "dir.nc" / "inner");
    SaveOutput mixed = oneFile(dir / "ok.nc", "ok");
    OutputFile blocked;
    blocked.path = dir / "dir.nc";
    blocked.text = "x";
    mixed.files.push_back(blocked);
    w = writeOutput(mixed, true);
    CHECK_FALSE(w.ok);
    CHECK(w.error.find("already replaced: 'ok.nc'") != std::string::npos);
    CHECK(tempFiles(dir).empty());
#endif
}

TEST_CASE("staging: a collision on the temporary name is retried; a later failure removes earlier files", "[save][safety]") {
    const fs::path dir = workDir("staging");
    struct Reset {
        ~Reset() { detail::setTestSeams(); }
    } reset;

    // O_EXCL retry: the first random name is already taken by someone else's file.
    const std::string taken = ".x.nc.0123456789abcdef.carvetto-tmp";
    { std::ofstream(dir / taken) << "foreign"; }
    detail::setTestSeams(0, false, -1, {"0123456789abcdef", "fedcba9876543210"});
    WriteResult w = writeOutput(oneFile(dir / "x.nc", "X"), false);
    REQUIRE(w.ok);
    CHECK(readTextFile(pathToUtf8(dir / "x.nc")) == "X");
    CHECK(readTextFile(pathToUtf8(dir / taken)) == "foreign");  // untouched, not removed

    // The second of three files cannot be staged: the first temporary file is removed, nothing is written.
    fs::remove(dir / taken);
    SaveOutput three = oneFile(dir / "t1.nc", "1");
    for (const char* name : {"t2.nc", "t3.nc"}) {
        OutputFile of;
        of.path = dir / name;
        of.text = name;
        three.files.push_back(of);
    }
    detail::setTestSeams(0, false, 1);
    w = writeOutput(three, false);
    CHECK_FALSE(w.ok);
    CHECK(w.error.find("injected failure") != std::string::npos);
    CHECK(tempFiles(dir).empty());
    CHECK_FALSE(fs::exists(dir / "t1.nc"));
    CHECK_FALSE(fs::exists(dir / "t2.nc"));
}

TEST_CASE("a non-ASCII output folder works end to end", "[save]") {
    Fixture f;
    const fs::path base = workDir("unicode");
    const fs::path dir = base / pathFromUtf8("\xC3\xA9t\xC3\xA9 \xD0\x96");
    fs::create_directories(dir);
    // A post that prints the folder and path it is given.
    std::string text = readTextFile(std::string(CARVETTO_POSTS_DIR) + "/grbl_mm.pp");
    const auto at = text.find("begin HEADER\n");
    REQUIRE(at != std::string::npos);
    text.insert(at + 13, "\"([TP_DIR]) ([PATHNAME])\"\n");
    { std::ofstream(base / "echo.pp", std::ios::binary) << text; }
    const auto posts = discoverPosts({}, base);
    REQUIRE(posts.size() == 1);
    REQUIRE(posts[0].usable());
    SaveOptions o;
    o.directory = dir;
    o.jobName = "caf\xC3\xA9";
    const SaveOutput out = generateOutput(prepareSave(f.doc, {}, posts[0], o));
    INFO(out.error);
    REQUIRE(out.ok);
    CHECK(out.files[0].text.find("/ete _)") != std::string::npos);  // the post writes comments in ASCII
    CHECK(pathToUtf8(out.files[0].path.filename()) == "caf\xC3\xA9.nc");
    const WriteResult w = writeOutput(out, false);
    REQUIRE(w.ok);
    CHECK(fs::exists(dir / pathFromUtf8("caf\xC3\xA9.nc")));
}

TEST_CASE("a planted symlink at a predictable temp name is not followed", "[save][safety]") {
    const fs::path dir = workDir("symtmp");
    const fs::path victim = dir / "victim.txt";
    { std::ofstream(victim) << "precious"; }
    std::error_code ec;
    std::error_code ec2;
    fs::create_symlink(victim, dir / "job.nc.carvetto-tmp", ec);
    fs::create_symlink(victim, dir / ".job.nc.carvetto-tmp", ec2);
    if (ec || ec2) {  // e.g. Windows without the symlink privilege
        SKIP("symlinks are not available here");
    }
    const WriteResult w = writeOutput(oneFile(dir / "job.nc", "G-code"), false);
    REQUIRE(w.ok);
    CHECK(readTextFile(pathToUtf8(victim)) == "precious");
    CHECK_FALSE(fs::is_symlink(dir / "job.nc"));
    CHECK(readTextFile(pathToUtf8(dir / "job.nc")) == "G-code");
}

TEST_CASE("a symlinked target is refused, or replaced itself when overwriting", "[save][safety]") {
    const fs::path dir = workDir("symtarget");
    const fs::path victim = dir / "victim.txt";
    { std::ofstream(victim) << "precious"; }
    std::error_code ec;
    std::error_code ec2;
    fs::create_symlink(victim, dir / "job.nc", ec);
    fs::create_symlink(dir / "nowhere", dir / "dangling.nc", ec2);
    if (ec || ec2) {  // e.g. Windows without the symlink privilege
        SKIP("symlinks are not available here");
    }
    WriteResult w = writeOutput(oneFile(dir / "job.nc", "new"), false);
    CHECK_FALSE(w.ok);
    CHECK(readTextFile(pathToUtf8(victim)) == "precious");
    w = writeOutput(oneFile(dir / "dangling.nc", "new"), false);
    CHECK_FALSE(w.ok);
    CHECK_FALSE(fs::exists(dir / "nowhere"));

    w = writeOutput(oneFile(dir / "job.nc", "new"), true);
    REQUIRE(w.ok);
    CHECK(readTextFile(pathToUtf8(victim)) == "precious");  // the link was replaced, not its target
    CHECK_FALSE(fs::is_symlink(dir / "job.nc"));
    CHECK(readTextFile(pathToUtf8(dir / "job.nc")) == "new");
    CHECK(tempFiles(dir).empty());
}

TEST_CASE("a target created between the check and the commit is not replaced", "[save][safety]") {
    const fs::path dir = workDir("race");
    SaveOutput out = oneFile(dir / "a.nc", "A");
    OutputFile second;
    second.path = dir / "b.nc";
    second.text = "B";
    out.files.push_back(second);
    detail::setCommitHook([&](const std::vector<fs::path>&) { std::ofstream(dir / "b.nc") << "someone else"; });
    const WriteResult w = writeOutput(out, false);
    detail::setCommitHook({});
    CHECK_FALSE(w.ok);
    CHECK(w.error.find("exists already") != std::string::npos);
    CHECK(readTextFile(pathToUtf8(dir / "b.nc")) == "someone else");
    CHECK_FALSE(fs::exists(dir / "a.nc"));  // the file this call had created is taken away again
    CHECK(tempFiles(dir).empty());
    CHECK(w.written.empty());
}

TEST_CASE("every staged file has its own random name; concurrent saves do not clash", "[save][safety]") {
    const fs::path dir = workDir("concurrent");
    std::vector<fs::path> seen;
    detail::setCommitHook(
        [&](const std::vector<fs::path>& staged) { seen.insert(seen.end(), staged.begin(), staged.end()); });
    REQUIRE(writeOutput(oneFile(dir / "x.nc", "1"), true).ok);
    REQUIRE(writeOutput(oneFile(dir / "x.nc", "2"), true).ok);
    detail::setCommitHook({});
    REQUIRE(seen.size() == 2);
    CHECK(seen[0] != seen[1]);
    CHECK(seen[0].parent_path() == dir);

    // Concurrent saves to one target are a legitimate race: each write either succeeds completely or
    // fails cleanly (Windows may refuse a replace for a moment; the commit retries first). Whatever
    // happens, the target holds exactly one writer's complete content and no temp file is left.
    std::atomic<int> succeeded{0};
    std::atomic<int> malformed{0};
    std::vector<std::thread> threads;
    for (int t = 0; t < 4; ++t) {
        threads.emplace_back([&, t]() {
            for (int i = 0; i < 25; ++i) {
                const std::string text(2000, static_cast<char>('a' + t));
                const WriteResult r = writeOutput(oneFile(dir / "same.nc", text), true);
                if (r.ok) {
                    ++succeeded;
                } else if (r.error.empty() || !r.written.empty()) {
                    ++malformed;
                }
            }
        });
    }
    for (std::thread& th : threads) {
        th.join();
    }
    CHECK(malformed == 0);
    CHECK(succeeded > 0);
    const std::string final = readTextFile(pathToUtf8(dir / "same.nc"));
    CHECK(final.size() == 2000);
    CHECK(final.find_first_not_of(final[0]) == std::string::npos);  // one writer's complete content
    CHECK(tempFiles(dir).empty());
}

TEST_CASE("a failure while committing cleans up and names what was replaced", "[save][safety]") {
    const fs::path dir = workDir("commitfail");
    SaveOutput out = oneFile(dir / "a.nc", "A");
    OutputFile second;
    second.path = dir / "dir.nc";
    second.text = "B";
    out.files.push_back(second);
    fs::create_directories(dir / "dir.nc" / "inner");  // a directory cannot be replaced by a file
    { std::ofstream(dir / "a.nc") << "old"; }
    const WriteResult w = writeOutput(out, true);
    CHECK_FALSE(w.ok);
    CHECK(w.error.find("already replaced: 'a.nc'") != std::string::npos);
    CHECK(readTextFile(pathToUtf8(dir / "a.nc")) == "A");
    CHECK(tempFiles(dir).empty());
}

TEST_CASE("names that differ only in case collide, also beyond ASCII", "[save]") {
    Fixture f;
    const auto posts = shipped();
    SaveJob job = jobFor(f, postNamed(posts, "grbl_mm.pp"), OutputMode::FilePerToolpath, workDir("case"));
    job.records[0].name = "Part";
    job.records[1].name = "PART";
    job.records[2].name = "part";
    job.options.nameTemplate = "{toolpath}";
    SaveOutput out = generateOutput(job);
    REQUIRE(out.ok);
    CHECK(out.files[0].path.filename() == "Part.nc");
    CHECK(out.files[1].path.filename() == "PART_2.nc");
    CHECK(out.files[2].path.filename() == "part_3.nc");

    job.records[0].name = "\xC3\x89" "cole";  // E acute
    job.records[1].name = "\xC3\xA9" "cole";  // e acute
    job.records[2].name = "\xD0\x96";         // Cyrillic capital zhe
    out = generateOutput(job);
    REQUIRE(out.ok);
    CHECK(pathToUtf8(out.files[1].path.filename()) == "\xC3\xA9" "cole_2.nc");
    CHECK(pathToUtf8(out.files[2].path.filename()) == "\xD0\x96.nc");
}

TEST_CASE("reserved name variants and invisible characters are neutralised", "[save]") {
    CHECK(sanitizeFileStem("COM\xC2\xB9") == "_COM\xC2\xB9");
    CHECK(sanitizeFileStem("lpt\xC2\xB2") == "_lpt\xC2\xB2");
    CHECK(sanitizeFileStem("CON .x") == "_CON .x");
    CHECK(sanitizeFileStem("conin$") == "conin_");
    CHECK(sanitizeFileStem("a\xC2\x85" "b") == "a_b");           // C1 control (NEL)
    CHECK(sanitizeFileStem("a\xE2\x80\xAE" "b") == "a_b");       // right-to-left override
    CHECK(sanitizeFileStem("a\xE2\x81\xA6" "b") == "a_b");       // isolate
    CHECK(sanitizeFileStem("a\xE2\x80\x8B" "b") == "a_b");       // zero width space
    CHECK(sanitizeFileStem("a\xEF\xBB\xBF" "b") == "a_b");       // BOM
}
