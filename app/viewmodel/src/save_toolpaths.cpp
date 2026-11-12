#include "carvetto/viewmodel/save_toolpaths.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iterator>
#include <set>
#include <sstream>
#include <system_error>
#include <atomic>
#include <exception>
#include <chrono>
#include <thread>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <random>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>
#ifdef __APPLE__
#include <stdio.h>
#endif
#endif

#include "carvetto/document/double_sided.hpp"
#include "carvetto/toolpath/keepout.hpp"
#include "carvetto/viewmodel/file_session.hpp"
#include "carvetto/viewmodel/layers_sheets.hpp"

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
    std::string head = lowerAscii(stem.substr(0, stem.find('.')));
    while (!head.empty() && head.back() == ' ') {
        head.pop_back();
    }
    // COM1..9 / LPT1..9 also with the superscript digits (U+00B9, U+00B2, U+00B3).
    for (const char* sup : {"\xC2\xB9", "\xC2\xB2", "\xC2\xB3"}) {
        const std::string tail = sup;
        if (head.size() == 3 + tail.size() && head.compare(3, tail.size(), tail) == 0 &&
            (head.compare(0, 3, "com") == 0 || head.compare(0, 3, "lpt") == 0)) {
            return true;
        }
    }
    static const std::set<std::string> names = {"con",  "prn",  "aux",  "nul",    "conin$", "conout$", "com1", "com2",
                                                "com3", "com4", "com5", "com6",   "com7",   "com8",    "com9", "lpt1",
                                                "lpt2", "lpt3", "lpt4", "lpt5",   "lpt6",   "lpt7",    "lpt8", "lpt9"};
    return names.count(head) != 0;
}

/// The code point of the valid UTF-8 sequence of length n at s[i].
char32_t decodeAt(const std::string& s, std::size_t i, std::size_t n) {
    const auto b = [&](std::size_t k) { return static_cast<char32_t>(static_cast<unsigned char>(s[i + k])); };
    if (n == 1) {
        return b(0);
    }
    if (n == 2) {
        return ((b(0) & 0x1Fu) << 6) | (b(1) & 0x3Fu);
    }
    if (n == 3) {
        return ((b(0) & 0x0Fu) << 12) | ((b(1) & 0x3Fu) << 6) | (b(2) & 0x3Fu);
    }
    return ((b(0) & 0x07u) << 18) | ((b(1) & 0x3Fu) << 12) | ((b(2) & 0x3Fu) << 6) | (b(3) & 0x3Fu);
}

void encode(char32_t c, std::string& out) {
    if (c < 0x80) {
        out += static_cast<char>(c);
    } else if (c < 0x800) {
        out += static_cast<char>(0xC0 | (c >> 6));
        out += static_cast<char>(0x80 | (c & 0x3F));
    } else if (c < 0x10000) {
        out += static_cast<char>(0xE0 | (c >> 12));
        out += static_cast<char>(0x80 | ((c >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (c & 0x3F));
    } else {
        out += static_cast<char>(0xF0 | (c >> 18));
        out += static_cast<char>(0x80 | ((c >> 12) & 0x3F));
        out += static_cast<char>(0x80 | ((c >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (c & 0x3F));
    }
}

/// Simple case folding for Latin-1, Latin Extended-A, Greek and Cyrillic (what file systems that
/// ignore case commonly fold); other characters are unchanged.
char32_t foldCodePoint(char32_t c) {
    if (c >= 'A' && c <= 'Z') {
        return c + 0x20;
    }
    if ((c >= 0xC0 && c <= 0xDE && c != 0xD7)) {
        return c + 0x20;
    }
    if ((c >= 0x100 && c <= 0x137) || (c >= 0x14A && c <= 0x177)) {
        return (c % 2 == 0) ? c + 1 : c;
    }
    if ((c >= 0x139 && c <= 0x148)) {
        return (c % 2 == 1) ? c + 1 : c;
    }
    if (c >= 0x391 && c <= 0x3A9 && c != 0x3A2) {
        return c + 0x20;
    }
    if (c >= 0x410 && c <= 0x42F) {
        return c + 0x20;
    }
    if (c >= 0x400 && c <= 0x40F) {
        return c + 0x50;
    }
    return c;
}

/// A key equal for names that differ only in case.
std::string foldCase(const std::string& s) {
    std::string out;
    for (std::size_t i = 0; i < s.size();) {
        const std::size_t n = std::max<std::size_t>(utf8Length(s, i), 1);
        if (utf8Length(s, i) == 0) {
            out += s[i];
        } else {
            encode(foldCodePoint(decodeAt(s, i, n)), out);
        }
        i += n;
    }
    return out;
}

/// Characters that never belong in a file name: C1 controls, bidi controls, zero-width marks.
bool isHiddenCodePoint(char32_t c) {
    return (c >= 0x80 && c <= 0x9F) || c == 0x61C || (c >= 0x200B && c <= 0x200F) || (c >= 0x202A && c <= 0x202E) ||
           (c >= 0x2060 && c <= 0x2064) || (c >= 0x2066 && c <= 0x2069) || c == 0xFEFF;
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
            if (isHiddenCodePoint(decodeAt(text, i, n))) {
                out += '_';
            } else {
                out.append(text, i, n);
            }
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

/// The side a record is posted for (null: no side, posted as it is).
const SideInfo* sideOf(const SaveJob& job, const document::ToolpathRecord& r) {
    if (r.sheet.isNull()) {
        return nullptr;
    }
    for (const SideInfo& s : job.sides) {
        if (s.sheet == r.sheet) {
            return &s;
        }
    }
    return nullptr;
}

struct Group {
    const SideInfo* side{nullptr};  ///< Every record of the group is posted for this side.
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
            // One file per side (a file never mixes sides: each has its own Z zero).
            for (std::size_t i = 0; i < recs.size(); ++i) {
                const SideInfo* side = sideOf(job, recs[i]);
                const auto it = std::find_if(groups.begin(), groups.end(), [side](const Group& g) { return g.side == side; });
                if (it == groups.end()) {
                    Group g;
                    g.side = side;
                    g.records.push_back(i);
                    groups.push_back(std::move(g));
                } else {
                    it->records.push_back(i);
                }
            }
            break;
        }
        case OutputMode::FilePerToolpath:
            for (std::size_t i = 0; i < recs.size(); ++i) {
                Group g;
                g.side = sideOf(job, recs[i]);
                g.records.push_back(i);
                groups.push_back(std::move(g));
            }
            break;
        case OutputMode::FilePerTool:
            for (std::size_t i = 0; i < recs.size(); ++i) {
                bool placed = false;
                for (Group& g : groups) {
                    if (g.side == sideOf(job, recs[i]) && sameTool(recs[g.records.front()].tool, recs[i].tool)) {
                        g.records.push_back(i);
                        placed = true;
                        break;
                    }
                }
                if (!placed) {
                    Group g;
                    g.side = sideOf(job, recs[i]);
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
    if (g.side != nullptr) {
        raw += g.side->side == document::Side::Back ? "_back" : "_front";
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

static void copyKeepOut(const document::Document& doc, SaveSource& src) {
    for (const document::KeepOutZone& z : doc.keepOutZones()) {
        src.keepOutZones.push_back(z.outline);
    }
    src.keepOutClearance = doc.keepOutClearance();
    if (doc.job().type == document::JobType::DoubleSided) {
        for (const document::Sheet& s : doc.sheets()) {
            src.sides.push_back({s.id, s.side, s.name, sheetTopZ(doc, s.id), s.sizeMm.z});
        }
    }
}

SaveSource makeSaveSource(const document::Document& doc, const std::vector<document::ToolpathId>& ids) {
    SaveSource src;
    src.job = doc.job();
    copyKeepOut(doc, src);
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

SaveSource makeSaveSource(ToolpathSession& session) {
    SaveSource src;
    const document::Document* doc = session.document();
    if (doc == nullptr) {
        return src;
    }
    src.job = doc->job();
    copyKeepOut(*doc, src);
    std::set<document::ToolpathId> stale;
    std::set<document::ToolpathId> enabled;
    for (const ToolpathListItem& item : session.list()) {
        if (!item.enabled) {
            continue;
        }
        enabled.insert(item.id);
        if (item.status == ToolpathStatus::NeedsRecalculation) {
            stale.insert(item.id);
        }
    }
    for (const document::ToolpathRecord& r : doc->toolpaths()) {
        if (enabled.count(r.id) == 0) {
            continue;
        }
        if (!r.hasMoves()) {
            src.notComputed.push_back(r.name);
            continue;
        }
        if (stale.count(r.id) != 0) {
            src.stale.push_back(r.name);
        }
        src.records.push_back(r);
    }
    return src;
}

SaveJob prepareSave(const document::Document& doc, const std::vector<document::ToolpathId>& ids, const PostInfo& post,
                    const SaveOptions& options) {
    return prepareSave(makeSaveSource(doc, ids), post, options);
}

/// Replaces `job.records` by their tiles (tile-major, document order inside a tile). Sets
/// `job.error` when the tiling is invalid or a tile does not fit the work area.
static void tileRecords(SaveJob& job) {
    const toolpath::TileSpec& spec = *job.options.tiling;
    std::vector<toolpath::Toolpath> tps;
    std::vector<std::size_t> index;
    for (std::size_t i = 0; i < job.records.size(); ++i) {
        if (job.records[i].hasMoves()) {
            tps.push_back(job.records[i].toolpath());
            index.push_back(i);
        }
    }
    if (!spec.originalPosition && !job.keepOutZones.empty()) {
        job.error = "keep-out zones cannot be combined with tiles in tile coordinates (use the original position)";
        return;
    }
    const toolpath::TilingResult res = toolpath::tileToolpaths(tps, spec);
    if (!res.ok()) {
        job.error = "tiling: " + res.error;
        return;
    }
    std::vector<document::ToolpathRecord> out;
    std::size_t n = 0;
    for (const toolpath::Tile& tile : res.tiles) {
        ++n;
        if (!toolpath::tileFitsWorkArea(tile, job.options.workAreaX, job.options.workAreaY)) {
            job.error = "tiling: tile " + std::to_string(n) + " is larger than the machine work area";
            return;
        }
        for (std::size_t k = 0; k < tile.toolpaths.size(); ++k) {
            if (tile.toolpaths[k].isEmpty()) {
                continue;
            }
            document::ToolpathRecord rec = job.records[index[k]];
            rec.name += " (tile " + std::to_string(n) + ")";
            rec.moves = document::shareMoves(tile.toolpaths[k].moves);
            out.push_back(std::move(rec));
        }
    }
    job.records = std::move(out);
}

SaveJob prepareSave(const SaveSource& source, const PostInfo& post, const SaveOptions& options) {
    SaveJob job;
    job.job = source.job;
    job.records = source.records;
    job.notComputed = source.notComputed;
    job.stale = source.stale;
    job.keepOutZones = source.keepOutZones;
    job.keepOutClearance = source.keepOutClearance;
    job.sides = source.sides;
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
    if (job.error.empty() && job.options.tiling && !job.records.empty()) {
        tileRecords(job);
    }
    if (job.error.empty() && job.records.empty()) {
        job.error = job.notComputed.empty() ? "there are no toolpaths to save"
                                            : "none of the toolpaths has been calculated yet (recalculate them first)";
    }
    return job;
}

/// The path in UTF-8 with '/' separators: what `generic_string()` gives, without the narrow code
/// page conversion that throws for non-ASCII names on Windows.
static std::string genericUtf8(const fs::path& p) {
    std::string s = pathToUtf8(p);
#ifdef _WIN32
    std::replace(s.begin(), s.end(), '\\', '/');
#endif
    return s;
}

static SaveOutput generateOutputImpl(const SaveJob& job, const std::atomic<bool>* cancel);

SaveOutput generateOutput(const SaveJob& job, const std::atomic<bool>* cancel) {
    try {
        return generateOutputImpl(job, cancel);
    } catch (const std::exception& e) {
        SaveOutput out;
        out.error = std::string("saving failed: ") + e.what();
        return out;
    }
}

SaveOutput generateOutputImpl(const SaveJob& job, const std::atomic<bool>* cancel) {
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
    if (!job.stale.empty()) {
        std::string names;
        for (const std::string& n : job.stale) {
            names += (names.empty() ? "'" : ", '") + n + "'";
        }
        out.stale = job.stale;
        out.warnings.push_back(std::to_string(job.stale.size()) + " toolpath(s) are out of date and are posted from their old moves: " +
                               names + " (recalculate them first)");
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
        for (std::size_t k = 2; taken.count(foldCase(candidate)) != 0; ++k) {
            candidate = stem + "_" + std::to_string(k);
        }
        taken.insert(foldCase(candidate));
        paths.push_back(job.options.directory / pathFromUtf8(candidate + "." + ext));
    }

    std::optional<toolpath::KeepOutField> keepOut;
    if (!job.keepOutZones.empty()) {
        keepOut.emplace(job.keepOutZones, job.keepOutClearance);
    }
    for (std::size_t gi = 0; gi < groups.size(); ++gi) {
        if (cancel != nullptr && cancel->load()) {
            return fail("cancelled");
        }
        const fs::path& outFs = paths[gi];
        std::vector<toolpath::Toolpath> toolpaths;
        std::vector<const document::ToolpathRecord*> recs;
        const SideInfo* side = groups[gi].side;
        const double dz = side != nullptr ? side->topZ : 0.0;
        double safeZ = job.records[groups[gi].records.front()].safeZ() + dz;
        OutputFile file;
        for (std::size_t ri : groups[gi].records) {
            const document::ToolpathRecord& r = job.records[ri];
            safeZ = std::max(safeZ, r.safeZ() + dz);
            toolpaths.push_back(r.toolpath());
            if (side != nullptr) {
                // The side's Z zero (T-0261): moves were computed with the cut face at 0.
                toolpaths.back().moves = document::shiftMovesZ(toolpaths.back().moves, dz);
            }
            recs.push_back(&r);
            file.toolpaths.push_back(r.name);
        }
        out.toolChanges += countToolChanges(recs);

        // Keep-out zones (T-0241): rapids are routed around them. The tool starts at home and
        // each toolpath starts where the previous one ended.
        if (keepOut && !keepOut->empty()) {
            std::optional<geometry::Vec3> at = geometry::Vec3{0.0, 0.0, safeZ};
            for (toolpath::Toolpath& tp : toolpaths) {
                const toolpath::RoutingReport rep = toolpath::routeRapids(tp, *keepOut, at);
                if (rep.unroutable != 0) {
                    out.warnings.push_back(pathToUtf8(outFs.filename()) + ": toolpath '" + tp.name + "': " +
                                           std::to_string(rep.unroutable) +
                                           " rapid move(s) cannot be routed around the keep-out zones (an end point "
                                           "lies inside a zone) and are left straight");
                }
                if (rep.tooComplex != 0) {
                    out.warnings.push_back(pathToUtf8(outFs.filename()) + ": toolpath '" + tp.name + "': " +
                                           std::to_string(rep.tooComplex) +
                                           " rapid move(s) were not routed around the keep-out zones (the zones "
                                           "are too complex) and are left straight");
                }
                if (!tp.moves.empty()) {
                    at = tp.moves.back().to;
                }
            }
        }

        // The same context carvetto-cli builds (app/cli/src/cli.cpp, cmdPost).
        post::PostContext ctx;
        ctx.fileName = pathToUtf8(outFs.stem());
        ctx.fileExt = outFs.has_extension() ? pathToUtf8(outFs.extension()).substr(1) : std::string();
        ctx.fileDir = genericUtf8(outFs.parent_path());
        ctx.pathName = genericUtf8(outFs);
        ctx.safeZ = safeZ;
        ctx.home = {0.0, 0.0, safeZ};
        const document::Job& j = job.job;
        const geometry::Box2 material = j.materialBoundsMm();
        const bool bedZero = j.zZero == document::ZZero::MachineBed;
        ctx.materialMin = {material.min.x, material.min.y, bedZero ? 0.0 : -j.sizeMm.z};
        ctx.materialMax = {material.max.x, material.max.y, bedZero ? j.sizeMm.z : 0.0};
        if (side != nullptr) {
            ctx.materialMin.z = side->topZ - side->thicknessMm;
            ctx.materialMax.z = side->topZ;
        }
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

// ------------------------------------------------------------------------------ safe writing

namespace {

std::function<void(const std::vector<fs::path>&)>& commitHook() {
    static std::function<void(const std::vector<fs::path>&)> hook;
    return hook;
}

std::mutex& hookMutex() {
    static std::mutex m;
    return m;
}

/// A target "exists" when anything is at the path, a dangling symlink included.
bool occupied(const fs::path& p) {
    std::error_code ec;
    const fs::file_status st = fs::symlink_status(p, ec);
    return !ec && st.type() != fs::file_type::not_found;
}

/// Test seams (see detail::setTestSeams).
struct Seams {
    std::atomic<int> linkErrno{0};       ///< POSIX: the link() step fails with this errno
    std::atomic<int> noRenameNoReplace{0};  ///< POSIX: the no-replace rename is "unsupported"
    std::atomic<int> failStageAfter{-1};  ///< staging the file with this index fails
    std::mutex mutex;
    std::vector<std::string> randomNames;  ///< returned (front first) instead of random names
};

Seams& seams() {
    static Seams s;
    return s;
}

std::string randomHex() {
    {
        const std::lock_guard<std::mutex> lock(seams().mutex);
        if (!seams().randomNames.empty()) {
            std::string forced = seams().randomNames.front();
            seams().randomNames.erase(seams().randomNames.begin());
            return forced;
        }
    }
    static std::atomic<std::uint64_t> counter{0};
    std::random_device rd;
    std::uint64_t v =
        (static_cast<std::uint64_t>(rd()) << 32) ^ rd() ^ (counter.fetch_add(1) * 0x9E3779B97F4A7C15ull);
    std::string out;
    for (int i = 0; i < 16; ++i) {
        out += "0123456789abcdef"[v & 0xF];
        v >>= 4;
    }
    return out;
}

enum class Commit { Ok, Exists, Failed };

#ifdef _WIN32

/// Creates `path` (it must not exist), never following a link, and writes `data`.
bool createExclusive(const fs::path& path, const std::string& data, std::string& err) {
    HANDLE h = CreateFileW(path.wstring().c_str(), GENERIC_WRITE, 0, nullptr, CREATE_NEW,
                           FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OPEN_REPARSE_POINT, nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        const DWORD code = GetLastError();
        err = (code == ERROR_FILE_EXISTS || code == ERROR_ALREADY_EXISTS) ? "exists" : "cannot create";
        return false;
    }
    std::size_t done = 0;
    bool ok = true;
    while (done < data.size() && ok) {
        const DWORD chunk = static_cast<DWORD>(std::min<std::size_t>(data.size() - done, 1u << 20));
        DWORD wrote = 0;
        ok = WriteFile(h, data.data() + done, chunk, &wrote, nullptr) != 0 && wrote == chunk;
        done += wrote;
    }
    ok = ok && FlushFileBuffers(h) != 0;
    CloseHandle(h);
    if (!ok) {
        err = "cannot write";
        DeleteFileW(path.wstring().c_str());  // the file is ours: created just now with CREATE_NEW
    }
    return ok;
}

/// Moves the staged file to `target`; without `overwrite` an existing target is never replaced.
Commit commitFile(const fs::path& staged, const fs::path& target, bool overwrite, std::string& err) {
    const DWORD flags = overwrite ? MOVEFILE_REPLACE_EXISTING : 0;
    DWORD code = 0;
    // Another save replacing the same target (or a virus scanner holding the new file) makes the
    // move fail for a moment with a sharing or access error: retry briefly, then report.
    for (int attempt = 0; attempt < 20; ++attempt) {
        if (MoveFileExW(staged.wstring().c_str(), target.wstring().c_str(), flags) != 0) {
            return Commit::Ok;
        }
        code = GetLastError();
        if (!overwrite && (code == ERROR_ALREADY_EXISTS || code == ERROR_FILE_EXISTS)) {
            return Commit::Exists;
        }
        if (code != ERROR_ACCESS_DENIED && code != ERROR_SHARING_VIOLATION && code != ERROR_LOCK_VIOLATION) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    err = "error " + std::to_string(static_cast<unsigned long>(code));
    return Commit::Failed;
}

#else

bool createExclusive(const fs::path& path, const std::string& data, std::string& err) {
    const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0644);
    if (fd < 0) {
        err = errno == EEXIST ? "exists" : std::strerror(errno);
        return false;
    }
    std::size_t done = 0;
    bool ok = true;
    while (done < data.size()) {
        const ssize_t n = ::write(fd, data.data() + done, data.size() - done);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            err = std::strerror(errno);
            ok = false;
            break;
        }
        done += static_cast<std::size_t>(n);
    }
    if (ok && ::fsync(fd) != 0 && errno != EINVAL && errno != EROFS) {
        err = std::strerror(errno);
        ok = false;
    }
    if (::close(fd) != 0 && ok) {
        err = std::strerror(errno);
        ok = false;
    }
    if (!ok) {
        ::unlink(path.c_str());  // the file is ours: created just now with O_EXCL
    }
    return ok;
}

/// Moves `staged` to `target` only when nothing is at `target`, without hard links:
/// renameat2(RENAME_NOREPLACE) on Linux, renamex_np(RENAME_EXCL) on macOS. Returns 0 on success,
/// otherwise errno (ENOSYS/ENOTSUP/EINVAL: the system or file system cannot do it).
int renameNoReplace(const fs::path& staged, const fs::path& target) {
    if (seams().noRenameNoReplace.load() != 0) {
        return ENOSYS;
    }
#if defined(__linux__) && defined(SYS_renameat2)
    constexpr unsigned int kNoReplace = 1;  // RENAME_NOREPLACE
    if (::syscall(SYS_renameat2, AT_FDCWD, staged.c_str(), AT_FDCWD, target.c_str(), kNoReplace) == 0) {
        return 0;
    }
    return errno;
#elif defined(__APPLE__)
    if (::renamex_np(staged.c_str(), target.c_str(), RENAME_EXCL) == 0) {
        return 0;
    }
    return errno;
#else
    (void)staged;
    (void)target;
    return ENOSYS;
#endif
}

bool linkUnsupported(int code) {
    return code == EPERM || code == ENOTSUP || code == EOPNOTSUPP || code == ENOSYS || code == EXDEV ||
           code == EMLINK;
}

Commit commitFile(const fs::path& staged, const fs::path& target, bool overwrite, std::string& err) {
    if (overwrite) {
        // rename() replaces the directory entry itself: a symlink at the target is replaced, not followed.
        if (::rename(staged.c_str(), target.c_str()) != 0) {
            err = std::strerror(errno);
            return Commit::Failed;
        }
        return Commit::Ok;
    }
    // 1. link() fails when anything (a dangling symlink too) is at the target: no replace, no race.
    int code = 0;
    if (seams().linkErrno.load() != 0) {
        code = seams().linkErrno.load();
    } else if (::link(staged.c_str(), target.c_str()) != 0) {
        code = errno;
    }
    if (code == 0) {
        ::unlink(staged.c_str());
        return Commit::Ok;
    }
    if (code == EEXIST) {
        return Commit::Exists;
    }
    if (!linkUnsupported(code)) {
        err = std::strerror(code);
        return Commit::Failed;
    }
    // 2. No hard links here (FAT, exFAT, some network and FUSE file systems): an atomic no-replace rename.
    code = renameNoReplace(staged, target);
    if (code == 0) {
        return Commit::Ok;
    }
    if (code == EEXIST) {
        return Commit::Exists;
    }
    if (code != ENOSYS && code != ENOTSUP && code != EOPNOTSUPP && code != EINVAL && code != EPERM) {
        err = std::strerror(code);
        return Commit::Failed;
    }
    // 3. Last resort: look at the target (lstat: a symlink, dangling or not, counts as present) right
    // before rename(). A file created between the lstat and the rename can still be replaced; that
    // window is a few microseconds and only exists on file systems without a no-replace primitive.
    struct stat st {};
    if (::lstat(target.c_str(), &st) == 0) {
        return Commit::Exists;
    }
    if (errno != ENOENT) {
        err = std::strerror(errno);
        return Commit::Failed;
    }
    if (::rename(staged.c_str(), target.c_str()) != 0) {
        err = std::strerror(errno);
        return Commit::Failed;
    }
    return Commit::Ok;
}

#endif

void removeQuietly(const fs::path& p) {
    std::error_code ec;
    fs::remove(p, ec);
}

}  // namespace

namespace detail {
void setCommitHook(std::function<void(const std::vector<std::filesystem::path>&)> hook) {
    const std::lock_guard<std::mutex> lock(hookMutex());
    commitHook() = std::move(hook);
}

void setTestSeams(int linkErrno, bool noRenameNoReplace, int failStageAfter, std::vector<std::string> randomNames) {
    seams().linkErrno = linkErrno;
    seams().noRenameNoReplace = noRenameNoReplace ? 1 : 0;
    seams().failStageAfter = failStageAfter;
    const std::lock_guard<std::mutex> lock(seams().mutex);
    seams().randomNames = std::move(randomNames);
}
}  // namespace detail

static WriteResult writeOutputImpl(const SaveOutput& output, bool overwrite);

WriteResult writeOutput(const SaveOutput& output, bool overwrite) {
    try {
        return writeOutputImpl(output, overwrite);
    } catch (const std::exception& e) {
        WriteResult res;
        res.error = std::string("saving failed: ") + e.what();
        return res;
    }
}

static WriteResult writeOutputImpl(const SaveOutput& output, bool overwrite) {
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
        if (!overwrite && occupied(f.path)) {
            res.error = "'" + pathToUtf8(f.path) + "' exists already";
            return res;
        }
    }
    // Stage: every file under its own random name, created exclusively (never through a link).
    std::vector<fs::path> staged;
    const auto cleanup = [&](std::size_t from) {
        for (std::size_t i = from; i < staged.size(); ++i) {
            removeQuietly(staged[i]);
        }
    };
    for (std::size_t fileIndex = 0; fileIndex < output.files.size(); ++fileIndex) {
        const OutputFile& f = output.files[fileIndex];
        bool created = false;
        std::string err;
        fs::path tmp;
        if (seams().failStageAfter.load() >= 0 && static_cast<std::size_t>(seams().failStageAfter.load()) == fileIndex) {
            res.error = "cannot write '" + pathToUtf8(f.path) + "': injected failure";
            cleanup(0);
            return res;
        }
        for (int attempt = 0; attempt < 20 && !created; ++attempt) {
            tmp = f.path.parent_path() /
                  pathFromUtf8("." + pathToUtf8(f.path.filename()) + "." + randomHex() + ".carvetto-tmp");
            err.clear();
            created = createExclusive(tmp, f.text, err);
            if (!created && err != "exists") {
                break;
            }
        }
        if (!created) {
            res.error = "cannot write '" + pathToUtf8(f.path) + "': " + err;
            cleanup(0);
            return res;
        }
        staged.push_back(tmp);
    }
    {
        std::function<void(const std::vector<fs::path>&)> hook;
        {
            const std::lock_guard<std::mutex> lock(hookMutex());
            hook = commitHook();
        }
        if (hook) {
            hook(staged);
        }
    }
    // Commit. Without overwrite a file that appeared meanwhile is never replaced, and files this
    // call already created are taken away again (they are new, so nothing is lost).
    for (std::size_t i = 0; i < output.files.size(); ++i) {
        std::string err;
        const Commit c = commitFile(staged[i], output.files[i].path, overwrite, err);
        if (c != Commit::Ok) {
            std::string done;
            if (!overwrite) {
                for (const fs::path& w : res.written) {
                    removeQuietly(w);
                }
            } else {
                for (const fs::path& w : res.written) {
                    done += (done.empty() ? "'" : ", '") + pathToUtf8(w.filename()) + "'";
                }
            }
            res.error = c == Commit::Exists ? "'" + pathToUtf8(output.files[i].path) + "' exists already"
                                            : "cannot write '" + pathToUtf8(output.files[i].path) + "': " + err;
            if (!done.empty()) {
                res.error += " (already replaced: " + done + ")";
            }
            cleanup(i);
            res.written.clear();
            return res;
        }
        res.written.push_back(output.files[i].path);
    }
    res.ok = true;
    return res;
}

}  // namespace carvetto::viewmodel
