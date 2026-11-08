// T-0224 AC: the G-code the Save Toolpaths logic generates is byte-identical to what
// `carvetto-cli post` writes for the same post, document and output path. Driven in process on the
// CLI end-to-end data (DXF -> drill + profile with tabs, as the T-0150 golden pipeline), with every
// shipped post, in single-file mode and per toolpath.

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "cli.hpp"
#include "carvetto/document/carv_io.hpp"
#include "carvetto/viewmodel/file_session.hpp"
#include "carvetto/viewmodel/save_toolpaths.hpp"

namespace fs = std::filesystem;
using namespace carvetto;
using namespace carvetto::viewmodel;

namespace {

const std::string kData = CARVETTO_CLI_DATA_DIR;

int runCli(const std::vector<std::string>& args, std::string* err = nullptr) {
    std::ostringstream out;
    std::ostringstream e;
    const int code = cli::run(args, out, e);
    if (err != nullptr) {
        *err = e.str();
    }
    return code;
}

std::string bytesOf(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    REQUIRE(in);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

/// Writes a params file with the tool database path made absolute.
void writeParams(const fs::path& dest, const std::string& sourceName) {
    std::string text = bytesOf(fs::path(kData) / sourceName);
    const std::string key = "\"tools.json\"";
    const auto at = text.find(key);
    REQUIRE(at != std::string::npos);
    text.replace(at, key.size(), "\"" + (fs::path(kData) / "tools.json").generic_string() + "\"");
    std::ofstream(dest, std::ios::binary) << text;
}

struct Pipeline {
    fs::path dir;
    document::LoadedCarv loaded;
};

Pipeline buildPipeline(const std::string& name) {
    Pipeline p;
    p.dir = fs::path(CARVETTO_VM_WORK_DIR) / ("save_cli_" + name);
    fs::remove_all(p.dir);
    fs::create_directories(p.dir);
    writeParams(p.dir / "drill.json", "drill.json");
    writeParams(p.dir / "profile.json", "profile.json");
    std::string err;
    REQUIRE(runCli({"import", kData + "/part.dxf", (p.dir / "part.carv").generic_string()}, &err) == 0);
    // drill only, profile only, both (drill then profile, as the golden pipeline).
    const std::string in = (p.dir / "part.carv").generic_string();
    REQUIRE(runCli({"toolpath", "drill", (p.dir / "drill.json").generic_string(), in,
                    (p.dir / "drill.carv").generic_string()}, &err) == 0);
    REQUIRE(runCli({"toolpath", "profile", (p.dir / "profile.json").generic_string(), in,
                    (p.dir / "profile.carv").generic_string()}, &err) == 0);
    REQUIRE(runCli({"toolpath", "profile", (p.dir / "profile.json").generic_string(),
                    (p.dir / "drill.carv").generic_string(), (p.dir / "both.carv").generic_string()}, &err) == 0);
    auto loaded = document::loadCarv(p.dir / "both.carv");
    REQUIRE(loaded);
    p.loaded = std::move(loaded.value());
    return p;
}

}  // namespace

TEST_CASE("single file output equals carvetto-cli post for every shipped post", "[save][cli]") {
    Pipeline p = buildPipeline("single");
    const auto posts = discoverPosts(fs::path(CARVETTO_POSTS_DIR), {});
    REQUIRE(posts.size() == 8);
    REQUIRE(p.loaded.document.toolpaths().size() >= 2);
    std::size_t bytes = 0;
    for (const PostInfo& post : posts) {
        const fs::path outPath = p.dir / ("out." + post.extension);
        const std::string pp = post.path.generic_string();
        std::string err;
        REQUIRE(runCli({"post", pp, (p.dir / "both.carv").generic_string(), outPath.generic_string()}, &err) == 0);

        SaveOptions o;
        o.directory = p.dir;
        o.jobName = "out";
        const SaveOutput out = generateOutput(prepareSave(p.loaded.document, {}, post, o));
        INFO(post.name << ": " << out.error);
        REQUIRE(out.ok);
        REQUIRE(out.files.size() == 1);
        CHECK(out.files[0].path == outPath);
        CHECK(out.files[0].text == bytesOf(outPath));
        CHECK(out.files[0].text.size() > 200);
        bytes += out.files[0].text.size();

        // And what is written is what the CLI wrote.
        fs::remove(outPath);
        const WriteResult w = writeOutput(out, false);
        REQUIRE(w.ok);
        CHECK(bytesOf(outPath) == out.files[0].text);
    }
    std::cout << "[T-0224 CLI identity] " << posts.size() << " posts, single file, " << bytes
              << " bytes compared (save = CLI)" << std::endl;
}

TEST_CASE("per toolpath and per tool output equals carvetto-cli post of that toolpath", "[save][cli]") {
    Pipeline p = buildPipeline("split");
    const auto posts = discoverPosts(fs::path(CARVETTO_POSTS_DIR), {});
    const auto& records = p.loaded.document.toolpaths();
    REQUIRE(records.size() == 2);  // drill, then profile
    const std::vector<std::string> alone = {"drill.carv", "profile.carv"};
    for (const PostInfo& post : posts) {
        for (OutputMode mode : {OutputMode::FilePerToolpath, OutputMode::FilePerTool}) {
            SaveOptions o;
            o.mode = mode;
            o.directory = p.dir;
            o.jobName = "job";
            o.nameTemplate = "f{n}";
            const SaveOutput out = generateOutput(prepareSave(p.loaded.document, {}, post, o));
            INFO(post.name << ": " << out.error);
            REQUIRE(out.ok);
            REQUIRE(out.files.size() == 2);  // two toolpaths, two different tools
            for (std::size_t i = 0; i < 2; ++i) {
                const fs::path expectPath = p.dir / ("f0" + std::to_string(i + 1) + "." + post.extension);
                CHECK(out.files[i].path == expectPath);
                REQUIRE(runCli({"post", post.path.generic_string(), (p.dir / alone[i]).generic_string(),
                                expectPath.generic_string()}) == 0);
                CHECK(out.files[i].text == bytesOf(expectPath));
                fs::remove(expectPath);
            }
        }
    }
}
