#include "toolpath_store.hpp"

#include <cmath>
#include <cstddef>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <utility>

#include "params.hpp"

namespace carvetto::cli {

namespace {

using json = nlohmann::json;  // std::map-backed: keys serialise sorted.
using toolpath::Move;
using toolpath::MoveKind;

const char* kindName(MoveKind k) {
    switch (k) {
        case MoveKind::Rapid: return "rapid";
        case MoveKind::Feed: return "feed";
        case MoveKind::Plunge: return "plunge";
        case MoveKind::Retract: return "retract";
        case MoveKind::ArcCW: return "arc_cw";
        case MoveKind::ArcCCW: return "arc_ccw";
        case MoveKind::Dwell: return "dwell";
    }
    return "rapid";
}

std::optional<MoveKind> parseKind(const std::string& s) {
    for (MoveKind k : {MoveKind::Rapid, MoveKind::Feed, MoveKind::Plunge, MoveKind::Retract, MoveKind::ArcCW,
                       MoveKind::ArcCCW, MoveKind::Dwell}) {
        if (s == kindName(k)) {
            return k;
        }
    }
    return std::nullopt;
}

/// Move encoding: ["kind", x, y, z] plus cx, cy for arcs or the seconds for a dwell, then an
/// optional {"feed": f} for a feed override.
json encodeMove(const Move& m) {
    json a = json::array({kindName(m.kind), m.to.x, m.to.y, m.to.z});
    if (toolpath::isArc(m.kind)) {
        a.push_back(m.centre.x);
        a.push_back(m.centre.y);
    } else if (m.kind == MoveKind::Dwell) {
        a.push_back(m.dwellSeconds);
    }
    if (m.feedOverride && toolpath::isCutting(m.kind)) {
        a.push_back(json{{"feed", *m.feedOverride}});
    }
    return a;
}

class Decoder {
   public:
    explicit Decoder(std::string member) : member_(std::move(member)) {}

    [[noreturn]] void fail(const std::string& what) const {
        throw CliError("toolpath member '" + member_ + "': " + what);
    }

    const json& field(const json& obj, const char* key) const {
        const auto it = obj.find(key);
        if (it == obj.end()) {
            fail(std::string("missing '") + key + "'");
        }
        return *it;
    }

    double number(const json& v, const std::string& what) const {
        if (!v.is_number()) {
            fail(what + " must be a number");
        }
        const double d = v.get<double>();
        if (!std::isfinite(d)) {
            fail(what + " must be finite");
        }
        return d;
    }

    /// A finite number greater than zero (tool diameter, feeds, dwell): a crafted file must not
    /// hand the post a zero or negative value.
    double positive(const json& v, const std::string& what) const {
        const double d = number(v, what);
        if (!(d > 0.0)) {
            fail(what + " must be greater than zero");
        }
        return d;
    }

    std::string string(const json& v, const std::string& what) const {
        if (!v.is_string()) {
            fail(what + " must be a string");
        }
        return v.get<std::string>();
    }

    Move move(const json& a, std::size_t i) const {
        const std::string where = "move " + std::to_string(i + 1);  // 1-based, as in post errors
        if (!a.is_array() || a.empty()) {
            fail(where + " must be a non-empty array");
        }
        const auto kind = parseKind(string(a[0], where + ": kind"));
        if (!kind) {
            fail(where + " has an unknown kind '" + a[0].get<std::string>() + "'");
        }
        Move m;
        m.kind = *kind;
        const std::size_t fixed = toolpath::isArc(m.kind) ? 6 : m.kind == MoveKind::Dwell ? 5 : 4;
        if (a.size() != fixed && a.size() != fixed + 1) {
            fail(where + " has " + std::to_string(a.size()) + " elements");
        }
        m.to = {number(a[1], where + ": x"), number(a[2], where + ": y"), number(a[3], where + ": z")};
        if (toolpath::isArc(m.kind)) {
            m.centre = {number(a[4], where + ": centre x"), number(a[5], where + ": centre y")};
        } else if (m.kind == MoveKind::Dwell) {
            m.dwellSeconds = positive(a[4], where + ": dwell seconds");
        }
        if (a.size() == fixed + 1) {
            const json& extra = a[fixed];
            if (!extra.is_object() || extra.size() != 1 || !extra.contains("feed") || !toolpath::isCutting(m.kind)) {
                fail(where + " has an invalid feed override");
            }
            m.feedOverride = positive(extra["feed"], where + ": feed override");
        }
        return m;
    }

   private:
    std::string member_;
};

}  // namespace

std::vector<std::uint8_t> encodeToolpath(const StoredToolpath& stored) {
    const toolpath::Toolpath& tp = stored.toolpath;
    json moves = json::array();
    for (const Move& m : tp.moves) {
        moves.push_back(encodeMove(m));
    }
    json j{
        {"format", kToolpathFormat},
        {"format_version", kToolpathFormatVersion},
        {"operation", stored.operation},
        {"name", tp.name},
        {"params", json::parse(stored.paramsJson)},
        {"safe_z", stored.safeZ},
        {"tool",
         {{"name", tp.tool.name},
          {"tool_number", tp.tool.toolNumber},
          {"diameter", tp.tool.diameter},
          {"feed_rate", tp.tool.feedRate},
          {"plunge_rate", tp.tool.plungeRate},
          {"spindle_speed", tp.tool.spindleSpeed}}},
        {"moves", std::move(moves)},
    };
    const std::string text = j.dump() + "\n";
    return {text.begin(), text.end()};
}

StoredToolpath decodeToolpath(std::string member, const std::vector<std::uint8_t>& bytes) {
    const Decoder d(member);
    json j;
    try {
        j = json::parse(bytes.begin(), bytes.end());
    } catch (const json::exception& e) {
        d.fail(std::string("not valid JSON: ") + e.what());
    }
    if (!j.is_object()) {
        d.fail("must hold a JSON object");
    }
    if (d.string(d.field(j, "format"), "'format'") != kToolpathFormat) {
        d.fail("is not a Carvetto toolpath");
    }
    const json& version = d.field(j, "format_version");
    if (!version.is_number_integer() || version.get<long long>() != kToolpathFormatVersion) {
        d.fail("has an unsupported format_version (this build reads version " +
               std::to_string(kToolpathFormatVersion) + ")");
    }
    StoredToolpath out;
    out.member = std::move(member);
    out.operation = d.string(d.field(j, "operation"), "'operation'");
    out.toolpath.name = d.string(d.field(j, "name"), "'name'");
    out.paramsJson = d.field(j, "params").dump();
    out.safeZ = d.number(d.field(j, "safe_z"), "'safe_z'");
    const json& tool = d.field(j, "tool");
    if (!tool.is_object()) {
        d.fail("'tool' must be an object");
    }
    toolpath::ToolSnapshot& t = out.toolpath.tool;
    t.name = d.string(d.field(tool, "name"), "'tool.name'");
    const json& number = d.field(tool, "tool_number");
    if (!number.is_number_integer() || number.get<long long>() < 0 || number.get<long long>() > 1'000'000) {
        d.fail("'tool.tool_number' must be a whole number from 0 to 1000000");
    }
    t.toolNumber = static_cast<int>(number.get<long long>());
    t.diameter = d.positive(d.field(tool, "diameter"), "'tool.diameter'");
    t.feedRate = d.positive(d.field(tool, "feed_rate"), "'tool.feed_rate'");
    t.plungeRate = d.positive(d.field(tool, "plunge_rate"), "'tool.plunge_rate'");
    t.spindleSpeed = d.number(d.field(tool, "spindle_speed"), "'tool.spindle_speed'");
    const json& moves = d.field(j, "moves");
    if (!moves.is_array()) {
        d.fail("'moves' must be an array");
    }
    out.toolpath.moves.reserve(moves.size());
    for (std::size_t i = 0; i < moves.size(); ++i) {
        out.toolpath.moves.push_back(d.move(moves[i], i));
    }
    return out;
}

bool isToolpathMember(std::string_view name) {
    constexpr std::string_view prefix = "toolpaths/";
    constexpr std::string_view suffix = ".json";
    if (name.size() != prefix.size() + 4 + suffix.size() || name.substr(0, prefix.size()) != prefix ||
        name.substr(prefix.size() + 4) != suffix) {
        return false;
    }
    for (char c : name.substr(prefix.size(), 4)) {
        if (c < '0' || c > '9') {
            return false;
        }
    }
    return true;
}

std::vector<StoredToolpath> storedToolpaths(const document::CarvExtras& extras) {
    std::vector<StoredToolpath> out;
    for (const std::string& name : extras.memberNames()) {  // sorted
        if (isToolpathMember(name)) {
            out.push_back(decodeToolpath(name, *extras.member(name)));
        }
    }
    return out;
}

std::string nextToolpathMember(const document::CarvExtras& extras) {
    int highest = 0;
    for (const std::string& name : extras.memberNames()) {
        if (isToolpathMember(name)) {
            highest = std::stoi(name.substr(10, 4));
        }
    }
    if (highest >= 9999) {
        throw CliError("the file already holds toolpath member toolpaths/9999.json; no more toolpaths can be added");
    }
    std::string digits = std::to_string(highest + 1);
    digits.insert(0, 4 - digits.size(), '0');
    return "toolpaths/" + digits + ".json";
}

}  // namespace carvetto::cli
