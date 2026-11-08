#include "carvetto/viewmodel/save_toolpaths.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iterator>
#include <set>
#include <sstream>
#include <system_error>

#include "carvetto/viewmodel/file_session.hpp"

namespace carvetto::viewmodel {

namespace fs = std::filesystem;

namespace {

constexpr std::uintmax_t kMaxPostBytes = 1024 * 1024;
constexpr std::size_t kMaxStemBytes = 100;

std::string lowerAscii(std::string s) {
    for (char& c : s) {
        if (c >= 'A' && c <= 'Z') {
            c = static_cast<char>(c - 'A' + 'a');
        }
    }
    return s;
}

/// Reads a whole regular file of at most `limit` bytes. Returns an error text or "".
std::string readSmallFile(const fs::path& path, std::uintmax_t limit, std::string& out) {
    std::error_code ec;
    const std::uintmax_t size = fs::file_size(path, ec);
    if (ec) {
        return "cannot read '" + pathToUtf8(path) + "'";
    }
    if (size > limit) {
        return "'" + pathToUtf8(path) + "' is too large for a post-processor (over 1 MiB)";
    }
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return "cannot read '" + pathToUtf8(path) + "'";
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    if (in.bad()) {
        return "cannot read '" + pathToUtf8(path) + "'";
    }
    out = ss.str();
    return {};
}

PostInfo describePost(const fs::path& path, bool shipped) {
    PostInfo info;
    info.path = path;
    info.shipped = shipped;
    info.name = pathToUtf8(path.filename());
    info.extension = "nc";
    std::string text;
    info.error = readSmallFile(path, kMaxPostBytes, text);
    if (!info.error.empty()) {
        return info;
    }
    auto pp = post::PostProcessor::parse(text);
    if (!pp) {
        info.error = pathToUtf8(path.filename()) + ": " + pp.error().describe();
        return info;
    }
    const post::PostSettings& s = pp.value().settings();
    if (!s.postName.empty()) {
        info.name = s.postName;
    }
    if (!s.fileExtension.empty()) {
        info.extension = s.fileExtension;
    }
    info.units = s.units;
    info.hasToolChange = pp.value().hasBlock("TOOLCHANGE");
    return info;
}

void listDirectory(const fs::path& dir, bool shipped, std::vector<PostInfo>& out) {
    if (dir.empty()) {
        return;
    }
    std::error_code ec;
    fs::directory_iterator it(dir, ec);
    if (ec) {
        return;
    }
    std::vector<fs::path> files;
    for (const fs::directory_iterator end; it != end; it.increment(ec)) {
        if (ec) {
            break;
        }
        std::error_code ec2;
        if (!it->is_regular_file(ec2) || ec2) {
            continue;
        }
        if (lowerAscii(pathToUtf8(it->path().extension())) != ".pp") {
            continue;
        }
        files.push_back(it->path());
    }
    std::sort(files.begin(), files.end(),
              [](const fs::path& a, const fs::path& b) { return pathToUtf8(a.filename()) < pathToUtf8(b.filename()); });
    for (const fs::path& f : files) {
        out.push_back(describePost(f, shipped));
    }
}

bool postMatches(const PostInfo& p, const std::string& key) {
    if (key.empty()) {
        return false;
    }
    return p.name == key || pathToUtf8(p.path.filename()) == key || pathToUtf8(p.path.stem()) == key;
}

// ---------------------------------------------------------------------------- file names

/// Length of the valid UTF-8 sequence at s[i], or 0.
std::size_t utf8Length(const std::string& s, std::size_t i) {
    const auto b = [&](std::size_t k) { return static_cast<unsigned char>(s[k]); };
    const unsigned char c = b(i);
    std::size_t n = 0;
    if (c < 0x80) {
        return 1;
    } else if (c >= 0xC2 && c <= 0xDF) {
        n = 2;
    } else if (c >= 0xE0 && c <= 0xEF) {
        n = 3;
    } else if (c >= 0xF0 && c <= 0xF4) {
        n = 4;
    } else {
        return 0;
    }
    if (i + n > s.size()) {
        return 0;
    }
    for (std::size_t k = 1; k < n; ++k) {
        if ((b(i + k) & 0xC0) != 0x80) {
            return 0;
        }
    }
    if (n == 3 && ((c == 0xE0 && b(i + 1) < 0xA0) || (c == 0xED && b(i + 1) >= 0xA0))) {
        return 0;
    }
    if (n == 4 && ((c == 0xF0 && b(i + 1) < 0x90) || (c == 0xF4 && b(i + 1) >= 0x90))) {
        return 0;
    }
    return n;
}

bool isReservedDeviceName(const std::string& stem) {
    const std::string head = lowerAscii(stem.substr(0, stem.find('.')));
    static const std::set<std::string> names = {"con",  "prn",  "aux",  "nul",  "com1", "com2", "com3", "com4",
                                                "com5", "com6", "com7", "com8", "com9", "lpt1", "lpt2", "lpt3",
                                                "lpt4", "lpt5", "lpt6", "lpt7", "lpt8", "lpt9"};
    return names.count(head) != 0;
}

/// Character-level mapping only (no trimming): what a template literal or a value may contain.
std::string mapName(const std::string& text) {
    std::string out;
    for (std::size_t i = 0; i < text.size();) {
        const std::size_t n = utf8Length(text, i);
        if (n == 0) {
            out += '_';
            ++i;
            continue;
        }
        if (n > 1) {
            out.append(text, i, n);
        } else {
            const char c = text[i];
            const bool ok = std::isalnum(static_cast<unsigned char>(c)) != 0 || c == ' ' || c == '-' || c == '_' ||
                            c == '.' || c == '(' || c == ')';
            out += ok ? c : '_';
        }
        i += n;
    }
    return out;
}

std::string finishStem(std::string s) {
    const auto first = s.find_first_not_of(" .");
    if (first == std::string::npos) {
        return {};
    }
    const auto last = s.find_last_not_of(" .");
    s = s.substr(first, last - first + 1);
    if (s.size() > kMaxStemBytes) {
        std::size_t cut = kMaxStemBytes;
        while (cut > 0 && (static_cast<unsigned char>(s[cut]) & 0xC0) == 0x80) {
            --cut;
        }
        s.resize(cut);
        const auto end = s.find_last_not_of(" .");
        s = end == std::string::npos ? std::string() : s.substr(0, end + 1);
    }
    if (!s.empty() && isReservedDeviceName(s)) {
        s.insert(s.begin(), '_');
    }
    return s;
}

std::string safeExtension(const std::string& ext) {
    std::string out;
    for (char c : ext) {
        if (std::isalnum(static_cast<unsigned char>(c)) != 0 && out.size() < 10) {
            out += c;
        }
    }
    return out.empty() ? std::string("nc") : out;
}

struct Group {
    std::vector<std::size_t> records;
    std::string tool;  ///< Name when every record has the same tool, else "".
    bool sameTool{true};
};

bool sameTool(const toolpath::ToolSnapshot& a, const toolpath::ToolSnapshot& b) {
    return a.toolNumber == b.toolNumber && a.name == b.name && a.diameter == b.diameter;
}

std::vector<Group> makeGroups(const SaveJob& job) {
    std::vector<Group> groups;
    const auto& recs = job.records;
    switch (job.options.mode) {
        case OutputMode::SingleFile: {
            Group g;
            for (std::size_t i = 0; i < recs.size(); ++i) {
                g.records.push_back(i);
            }
            groups.push_back(std::move(g));
            break;
        }
        case OutputMode::FilePerToolpath:
            for (std::size_t i = 0; i < recs.size(); ++i) {
                Group g;
                g.records.push_back(i);
                groups.push_back(std::move(g));
            }
            break;
        case OutputMode::FilePerTool:
            for (std::size_t i = 0; i < recs.size(); ++i) {
                bool placed = false;
                for (Group& g : groups) {
                    if (sameTool(recs[g.records.front()].tool, recs[i].tool)) {
                        g.records.push_back(i);
                        placed = true;
                        break;
                    }
                }
                if (!placed) {
                    Group g;
                    g.records.push_back(i);
                    groups.push_back(std::move(g));
                }
            }
            break;
    }
    for (Group& g : groups) {
        for (std::size_t k = 1; k < g.records.size(); ++k) {
            if (!sameTool(recs[g.records[0]].tool, recs[g.records[k]].tool)) {
                g.sameTool = false;
            }
        }
        g.tool = g.sameTool ? recs[g.records[0]].tool.name : std::string();
    }
    return groups;
}

/// Expands the name template for file number `n` (1-based) of a group. Empty string + error on a
/// bad template.
std::string expandName(const std::string& tmpl, const SaveJob& job, const Group& g, std::size_t n,
                       std::string& error) {
    std::string raw;
    for (std::size_t i = 0; i < tmpl.size();) {
        if (tmpl[i] != '{') {
            const std::size_t len = std::max<std::size_t>(utf8Length(tmpl, i), 1);
            raw += mapName(tmpl.substr(i, len));
            i += len;
            continue;
        }
        const auto close = tmpl.find('}', i);
        if (close == std::string::npos) {
            error = "file name template: '{' without a closing '}'";
            return {};
        }
        const std::string key = tmpl.substr(i + 1, close - i - 1);
        std::string value;
        if (key == "job") {
            value = job.options.jobName;
        } else if (key == "toolpath") {
            value = g.records.size() == 1 ? job.records[g.records[0]].name : std::string("toolpaths");
        } else if (key == "tool") {
            value = g.sameTool ? g.tool : std::string("tools");
        } else if (key == "n") {
            value = (n < 10 ? "0" : "") + std::to_string(n);
        } else {
            error = "file name template: unknown field '{" + key + "}' (use {job}, {toolpath}, {tool}, {n})";
            return {};
        }
        raw += mapName(value);
        i = close + 1;
    }
    std::string stem = finishStem(raw);
    if (stem.empty()) {
        error = "the file name template gives an empty file name";
    }
    return stem;
}

std::size_t countToolChanges(const std::vector<const document::ToolpathRecord*>& recs) {
    std::size_t n = 0;
    for (std::size_t i = 1; i < recs.size(); ++i) {
        if (!sameTool(recs[i - 1]->tool, recs[i]->tool)) {
            ++n;
        }
    }
    return n;
}

}  // namespace

std::vector<PostInfo> discoverPosts(const fs::path& shippedDir, const fs::path& userDir) {
    std::vector<PostInfo> out;
    listDirectory(shippedDir, true, out);
    listDirectory(userDir, false, out);
    return out;
}

std::optional<std::size_t> defaultPostFor(const std::vector<PostInfo>& posts, const tooling::Machine& machine) {
    for (std::size_t i = 0; i < posts.size(); ++i) {
        if (posts[i].usable() && postMatches(posts[i], machine.defaultPost)) {
            return i;
        }
    }
    return std::nullopt;
}

std::vector<std::size_t> postsForMachine(const std::vector<PostInfo>& posts, const tooling::Machine& machine) {
    std::vector<std::size_t> out;
    for (const std::string& key : machine.associatedPosts) {
        for (std::size_t i = 0; i < posts.size(); ++i) {
            if (posts[i].usable() && postMatches(posts[i], key) &&
                std::find(out.begin(), out.end(), i) == out.end()) {
                out.push_back(i);
            }
        }
    }
    if (machine.associatedPosts.empty()) {
        for (std::size_t i = 0; i < posts.size(); ++i) {
            if (posts[i].usable()) {
                out.push_back(i);
            }
        }
    }
    return out;
}

std::vector<std::size_t> postsWithUnits(const std::vector<PostInfo>& posts, post::Units units) {
    std::vector<std::size_t> out;
    for (std::size_t i = 0; i < posts.size(); ++i) {
        if (posts[i].usable() && posts[i].units == units) {
            out.push_back(i);
        }
    }
    return out;
}

std::string defaultNameTemplate(OutputMode mode) {
    switch (mode) {
        case OutputMode::SingleFile: return "{job}";
        case OutputMode::FilePerToolpath: return "{job}_{n}_{toolpath}";
        case OutputMode::FilePerTool: return "{job}_{n}_{tool}";
    }
    return "{job}";
}

std::string sanitizeFileStem(const std::string& text) {
    return finishStem(mapName(text));
}

SaveSource makeSaveSource(const document::Document& doc, const std::vector<document::ToolpathId>& ids) {
    SaveSource src;
    src.job = doc.job();
    const std::set<document::ToolpathId> wanted(ids.begin(), ids.end());
    for (const document::ToolpathRecord& r : doc.toolpaths()) {
        if (!ids.empty() && wanted.count(r.id) == 0) {
            continue;
        }
        if (!r.hasMoves()) {
            src.notComputed.push_back(r.name);
            continue;
        }
        src.records.push_back(r);
    }
    return src;
}

SaveJob prepareSave(const document::Document& doc, const std::vector<document::ToolpathId>& ids, const PostInfo& post,
                    const SaveOptions& options) {
    return prepareSave(makeSaveSource(doc, ids), post, options);
}

SaveJob prepareSave(const SaveSource& source, const PostInfo& post, const SaveOptions& options) {
    SaveJob job;
    job.job = source.job;
    job.records = source.records;
    job.notComputed = source.notComputed;
    job.post = post;
    job.options = options;
    if (!post.usable()) {
        job.error = post.error.empty() ? "no post-processor selected" : post.error;
    } else if (post.path.empty()) {
        job.error = "no post-processor selected";
    } else {
        const std::string err = readSmallFile(post.path, kMaxPostBytes, job.postText);
        if (!err.empty()) {
            job.error = "post-processor: " + err;
        }
    }
    if (job.error.empty() && job.records.empty()) {
        job.error = job.notComputed.empty() ? "there are no toolpaths to save"
                                            : "none of the toolpaths has been calculated yet (recalculate them first)";
    }
    return job;
}

SaveOutput generateOutput(const SaveJob& job, const std::atomic<bool>* cancel) {
    SaveOutput out;
    const auto fail = [&](std::string message) {
        out.ok = false;
        out.error = std::move(message);
        out.files.clear();
        return out;
    };
    if (!job.error.empty()) {
        return fail(job.error);
    }
    if (job.records.empty()) {
        return fail("there are no toolpaths to save");
    }
    if (job.options.directory.empty()) {
        return fail("no output folder chosen");
    }
    std::error_code ec;
    if (!fs::is_directory(job.options.directory, ec)) {
        return fail("the output folder '" + pathToUtf8(job.options.directory) + "' does not exist");
    }

    const std::string postLabel = pathToUtf8(job.post.path.filename());
    auto pp = post::PostProcessor::parse(job.postText);
    if (!pp) {
        return fail("post-processor '" + postLabel + "': " + pp.error().describe());
    }
    for (const post::Diagnostic& d : pp.value().warnings()) {
        out.warnings.push_back(postLabel + ": " + d.describe());
    }
    for (const std::string& name : job.notComputed) {
        out.warnings.push_back("toolpath '" + name + "' has not been calculated and is left out");
    }

    const std::string tmpl = job.options.nameTemplate.empty() ? defaultNameTemplate(job.options.mode)
                                                              : job.options.nameTemplate;
    const std::string ext = safeExtension(pp.value().settings().fileExtension);
    const std::vector<Group> groups = makeGroups(job);

    // File names first, so a bad template or a collision is reported before any post runs.
    std::vector<fs::path> paths;
    std::set<std::string> taken;
    for (std::size_t gi = 0; gi < groups.size(); ++gi) {
        std::string error;
        const std::string stem = expandName(tmpl, job, groups[gi], gi + 1, error);
        if (!error.empty()) {
            return fail(error);
        }
        std::string candidate = stem;
        for (std::size_t k = 2; taken.count(lowerAscii(candidate)) != 0; ++k) {
            candidate = stem + "_" + std::to_string(k);
        }
        taken.insert(lowerAscii(candidate));
        paths.push_back(job.options.directory / pathFromUtf8(candidate + "." + ext));
    }

    for (std::size_t gi = 0; gi < groups.size(); ++gi) {
        if (cancel != nullptr && cancel->load()) {
            return fail("cancelled");
        }
        const fs::path& outFs = paths[gi];
        std::vector<toolpath::Toolpath> toolpaths;
        std::vector<const document::ToolpathRecord*> recs;
        double safeZ = job.records[groups[gi].records.front()].safeZ();
        OutputFile file;
        for (std::size_t ri : groups[gi].records) {
            const document::ToolpathRecord& r = job.records[ri];
            safeZ = std::max(safeZ, r.safeZ());
            toolpaths.push_back(r.toolpath());
            recs.push_back(&r);
            file.toolpaths.push_back(r.name);
        }
        out.toolChanges += countToolChanges(recs);

        // The same context carvetto-cli builds (app/cli/src/cli.cpp, cmdPost).
        post::PostContext ctx;
        ctx.fileName = pathToUtf8(outFs.stem());
        ctx.fileExt = outFs.has_extension() ? pathToUtf8(outFs.extension()).substr(1) : std::string();
        ctx.fileDir = outFs.parent_path().generic_string();
        ctx.pathName = outFs.generic_string();
        ctx.safeZ = safeZ;
        ctx.home = {0.0, 0.0, safeZ};
        const document::Job& j = job.job;
        const geometry::Box2 material = j.materialBoundsMm();
        const bool bedZero = j.zZero == document::ZZero::MachineBed;
        ctx.materialMin = {material.min.x, material.min.y, bedZero ? 0.0 : -j.sizeMm.z};
        ctx.materialMax = {material.max.x, material.max.y, bedZero ? j.sizeMm.z : 0.0};
        ctx.rotaryDiameter = j.diameterMm;

        auto program = pp.value().run(toolpaths, ctx);
        if (!program) {
            return fail("post-processor '" + postLabel + "' failed" +
                        (groups.size() > 1 ? " for '" + pathToUtf8(outFs.filename()) + "'" : std::string()) + ": " +
                        program.error().describe());
        }
        for (const post::Diagnostic& d : program.value().warnings) {
            out.warnings.push_back(pathToUtf8(outFs.filename()) + ": " + d.describe());
        }
        file.path = outFs;
        file.text = std::move(program.value().text);
        std::error_code ec2;
        file.exists = fs::exists(outFs, ec2);
        out.files.push_back(std::move(file));
    }

    if (job.options.mode == OutputMode::SingleFile && out.toolChanges > 0 && !pp.value().hasBlock("TOOLCHANGE")) {
        out.warnings.push_back("the job changes tool " + std::to_string(out.toolChanges) + " time(s) but the post '" +
                               postLabel + "' has no TOOLCHANGE block: the file will not stop for the change");
    }
    out.ok = true;
    return out;
}

std::string previewText(const SaveOutput& output, std::size_t maxLines) {
    if (output.files.empty()) {
        return {};
    }
    const std::string& text = output.files.front().text;
    std::string preview;
    std::size_t pos = 0;
    std::size_t shown = 0;
    while (pos < text.size() && shown < maxLines) {
        const auto nl = text.find('\n', pos);
        const std::size_t end = nl == std::string::npos ? text.size() : nl + 1;
        preview.append(text, pos, end - pos);
        pos = end;
        ++shown;
    }
    if (pos < text.size()) {
        std::size_t more = 0;
        for (std::size_t i = pos; i < text.size(); ++i) {
            if (text[i] == '\n') {
                ++more;
            }
        }
        if (text.back() != '\n') {
            ++more;
        }
        if (!preview.empty() && preview.back() != '\n') {
            preview += '\n';
        }
        preview += "... (" + std::to_string(more) + " more lines)\n";
    }
    return preview;
}

WriteResult writeOutput(const SaveOutput& output, bool overwrite) {
    WriteResult res;
    if (!output.ok || output.files.empty()) {
        res.error = output.error.empty() ? "there is nothing to write" : output.error;
        return res;
    }
    // Checks first: nothing is staged when one of the targets is refused.
    for (const OutputFile& f : output.files) {
        std::error_code ec;
        if (!fs::is_directory(f.path.parent_path(), ec)) {
            res.error = "the output folder '" + pathToUtf8(f.path.parent_path()) + "' does not exist";
            return res;
        }
        if (!overwrite && fs::exists(f.path, ec)) {
            res.error = "'" + pathToUtf8(f.path) + "' exists already";
            return res;
        }
    }
    std::vector<fs::path> staged;
    const auto cleanup = [&]() {
        for (const fs::path& p : staged) {
            std::error_code ec;
            fs::remove(p, ec);
        }
    };
    for (const OutputFile& f : output.files) {
        fs::path tmp = f.path;
        tmp += ".carvetto-tmp";
        std::ofstream o(tmp, std::ios::binary | std::ios::trunc);
        if (!o) {
            res.error = "cannot write '" + pathToUtf8(f.path) + "'";
            cleanup();
            return res;
        }
        staged.push_back(tmp);
        o.write(f.text.data(), static_cast<std::streamsize>(f.text.size()));
        o.close();
        if (!o) {
            res.error = "cannot write '" + pathToUtf8(f.path) + "' (disk full?)";
            cleanup();
            return res;
        }
    }
    for (std::size_t i = 0; i < output.files.size(); ++i) {
        std::error_code ec;
        fs::rename(staged[i], output.files[i].path, ec);
        if (ec) {
            res.error = "cannot write '" + pathToUtf8(output.files[i].path) + "': " + ec.message();
            std::vector<fs::path> rest(staged.begin() + static_cast<std::ptrdiff_t>(i), staged.end());
            staged = rest;
            cleanup();
            return res;
        }
        res.written.push_back(output.files[i].path);
    }
    res.ok = true;
    return res;
}

}  // namespace carvetto::viewmodel
