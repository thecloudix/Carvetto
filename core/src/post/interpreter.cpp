// Interpreter for parsed `.pp` posts (ADR-0004, ADR-0011).
//
// The interpreter walks every toolpath's move list carrying the current tool position, exactly
// as the controller will (ADR-0008: a move's start is the previous move's end). For each move it
// picks the most specific block the post defines (FIRST_* before the plain block, helical before
// plain arcs, PLUNGE/RETRACT before FEED) and expands that block's lines. Each `[tag]` is
// evaluated against a frame describing the move being written, converted to the post's units,
// multiplied, formatted and — for `C` (modal) variables — suppressed when the text is the same
// as the last one written. Modal memory is cleared at the start of every toolpath so the first
// moves after a tool change or new segment restate the full position and feed.
//
// Arcs are written with the arc blocks when the post has them; otherwise (and when the radius
// is outside MIN/MAX_ARC_RADIUS, or a rotary wrap is active) they are linearised into feed moves
// whose chordal deviation stays within `PostContext::arcTolerance`.
//
// Output safety (T-0133, ADR-0020) is enforced here, in the render path every tag goes
// through, so no post can bypass it: text values are sanitised for the comment context they
// land in (`detail::sanitiseText`), and every number is checked to be finite and within the
// supported machine range before it is formatted. Move data is checked up front as well, so a
// bad coordinate is refused even when the post never writes that axis.

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "carvetto/geometry/vec2.hpp"
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/move.hpp"
#include "text_util.hpp"

namespace carvetto::post {

namespace {

using geometry::kPi;
using geometry::kTwoPi;
using geometry::Vec2;
using geometry::Vec3;
using toolpath::Move;
using toolpath::MoveKind;
using toolpath::Toolpath;
using toolpath::ToolSnapshot;

constexpr double kMmPerInch = 25.4;
constexpr double kZeroLength = 1e-9;

/// Largest magnitude accepted for a length (mm), feed (mm/min), spindle speed (rpm), dwell (s)
/// or angle (degrees) before conversion to the post's units (ADR-0020).
constexpr double kMaxMachineValue = 1e6;
/// Largest inverse-time feed `[FI]` accepted (1/min); short moves make it legitimately large.
constexpr double kMaxInverseTime = 1e9;
/// Largest magnitude of any number as written (after unit conversion and the VAR multiplier).
constexpr double kMaxWrittenValue = 1e9;
/// Accepted range of `PostContext::arcTolerance` (mm): finer would ask for more chords than the
/// segment cap allows (and below ~1e-13 r the chord angle rounds to zero), coarser is no arc.
constexpr double kMinArcTolerance = 1e-6;
constexpr double kMaxArcTolerance = 1.0;
/// Most chords one linearised arc is split into.
constexpr double kMaxArcSegments = 100000.0;
/// Names quoted in error messages are cut to this many bytes.
constexpr std::size_t kMaxQuotedName = 64;

/// A number for an error message: shortest round-trip form, "NaN", "+inf" or "-inf".
std::string describeValue(double v) {
    if (std::isnan(v)) {
        return "NaN";
    }
    if (std::isinf(v)) {
        return v > 0.0 ? "+inf" : "-inf";
    }
    std::array<char, 64> buf{};
    const auto r = std::to_chars(buf.data(), buf.data() + buf.size(), v);
    return r.ec == std::errc{} ? std::string(buf.data(), r.ptr) : std::string("?");
}

/// A user-supplied name for an error message: quoted, sanitised and cut short.
std::string quoted(std::string_view name) {
    const std::string s = detail::sanitiseText(name.substr(0, kMaxQuotedName), {});
    return "'" + s + (name.size() > kMaxQuotedName ? "...'" : "'");
}

/// The supported range of a numeric tag's value before unit conversion (ADR-0020).
struct ValueLimit {
    double max;
    const char* unit;
};

/// The limit for a tag, or nullopt for the interpreter's own integer counters (N, T, TP, DB).
std::optional<ValueLimit> limitFor(const TagInfo& info) {
    switch (info.kind) {
        case TagKind::Length:
            return ValueLimit{kMaxMachineValue, "mm"};
        case TagKind::FeedRate:
            return ValueLimit{kMaxMachineValue, "mm/min"};
        case TagKind::Angle:
            return ValueLimit{kMaxMachineValue, "degrees"};
        case TagKind::Number:
            if (info.tag == "FI") return ValueLimit{kMaxInverseTime, "1/min"};
            if (info.tag == "S") return ValueLimit{kMaxMachineValue, "rpm"};
            if (info.tag == "DWELL") return ValueLimit{kMaxMachineValue, "s"};
            return std::nullopt;
        case TagKind::Text:
            return std::nullopt;
    }
    return std::nullopt;
}

/// Why `v` is not acceptable against `limit` ("" when it is).
std::string rangeProblem(double v, double limit, std::string_view unit) {
    if (!std::isfinite(v)) {
        return describeValue(v) + " is not a finite number";
    }
    if (std::fabs(v) > limit) {
        const std::string u = unit.empty() ? std::string{} : " " + std::string(unit);
        return describeValue(v) + u + " is outside the supported range -" + describeValue(limit) + ".." +
               describeValue(limit) + u;
    }
    return {};
}

/// The G-code of a line outside its comments: `( … )` (nesting counted) and `;` to the line end.
std::string liveCode(std::string_view line) {
    std::string code;
    int depth = 0;
    for (const char c : line) {
        if (c == ';' && depth == 0) {
            break;
        }
        if (c == '(') {
            ++depth;
        } else if (c == ')') {
            depth = depth > 0 ? depth - 1 : 0;
        } else if (depth == 0) {
            code += c;
        }
    }
    return code;
}

bool isLetter(char c) {
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z');
}

bool isDigit(char c) {
    return c >= '0' && c <= '9';
}

/// Length of the number starting at `i` (digits with an optional decimal part); 0 when none.
std::size_t numberLength(std::string_view s, std::size_t i) {
    std::size_t j = i;
    while (j < s.size() && (isDigit(s[j]) || s[j] == '.')) {
        ++j;
    }
    return j > i && isDigit(s[i]) ? j - i : 0;
}

/// True for an `M` word starting the spindle: M3 / M4 (also M03, M3.0 …).
bool isSpindleStartCode(std::string_view number) {
    double v = 0.0;
    const auto r = std::from_chars(number.data(), number.data() + number.size(), v);
    return r.ec == std::errc{} && (v == 3.0 || v == 4.0);
}

/// The first word in live G-code `code` that starts the spindle or sets a speed (M3, M4, S…),
/// or "" when there is none (T-0154). A word is a letter at the start of a word (not preceded by
/// another letter) directly followed by a number, so a message such as Marlin's
/// `M0 Drag tool 3 spindle off` holds none.
std::string spindleWord(std::string_view code) {
    for (std::size_t i = 0; i < code.size(); ++i) {
        const char c = code[i];
        if (!isLetter(c) || (i > 0 && isLetter(code[i - 1]))) {
            continue;
        }
        const char u = c >= 'a' ? static_cast<char>(c - 'a' + 'A') : c;
        if (u != 'M' && u != 'S') {
            continue;
        }
        const std::size_t n = numberLength(code, i + 1);
        if (n == 0) {
            continue;
        }
        if (u == 'S' || isSpindleStartCode(code.substr(i + 1, n))) {
            return std::string(code.substr(i, n + 1));
        }
    }
    return {};
}

/// True when a block line (template text) that writes `[S]` does nothing but start the spindle:
/// outside its comments it holds only `M3`/`M4` words, `[S]` (optionally after a literal `S`),
/// `[N]` and spaces. Such a line is left out for a toolpath that runs with the spindle off
/// (T-0154); any other line writing `[S]` there is an error, so leaving it out never drops a
/// tool change or another code.
bool spindleOnlyLine(std::string_view line) {
    const std::string code = liveCode(line);
    std::string_view s(code);
    std::size_t i = 0;
    while (i < s.size()) {
        const char c = s[i];
        if (detail::isSpace(c)) {
            ++i;
        } else if (c == '[') {
            const std::size_t close = s.find(']', i);
            if (close == std::string_view::npos) {
                return false;
            }
            const std::string_view tag = s.substr(i + 1, close - i - 1);
            if (tag != "S" && tag != "N") {
                return false;
            }
            i = close + 1;
        } else if ((c == 'S' || c == 's') && s.substr(i + 1, 3) == "[S]") {
            ++i;
        } else if (c == 'M' || c == 'm') {
            const std::size_t n = numberLength(s, i + 1);
            if (n == 0 || !isSpindleStartCode(s.substr(i + 1, n))) {
                return false;
            }
            i += 1 + n;
        } else {
            return false;
        }
    }
    return true;
}

/// What the tags of the block being written refer to.
struct Frame {
    Vec3 from;
    Vec3 to;
    const Move* move{nullptr};  // arc moves only: centre for I/J etc.
    double feed{0.0};           // mm/min
    double inverseTime{0.0};    // 1/min
    double dwell{0.0};          // seconds
    double length{0.0};         // cutting moves: path length (mm), for inverse time
    bool cutting{false};        // Feed/Plunge/Retract/Arc: [F] may be inverse time
};

class Interpreter {
   public:
    Interpreter(const PostProcessor& pp, const PostContext& ctx, const std::vector<Toolpath>& toolpaths)
        : pp_(pp), s_(pp.settings()), ctx_(ctx), toolpaths_(toolpaths) {}

    Result<PostOutput> run() {
        if ((s_.rotaryWrapX || s_.rotaryWrapY) && !(ctx_.rotaryDiameter > 0.0)) {
            return Error{
                0, "the post wraps a linear axis onto a rotary axis but the job has no rotary diameter"};
        }
        if (s_.rotaryWrapX || s_.rotaryWrapY) {
            if (const std::string problem = rangeProblem(ctx_.rotaryDiameter, kMaxMachineValue, "mm");
                !problem.empty()) {
                return Error{0, "cannot write G-code: job rotary diameter " + problem};
            }
        }
        if (!(ctx_.arcTolerance >= kMinArcTolerance && ctx_.arcTolerance <= kMaxArcTolerance)) {
            return Error{0, "cannot write G-code: arc tolerance " + describeValue(ctx_.arcTolerance) +
                                " mm is outside the supported range " + describeValue(kMinArcTolerance) +
                                ".." + describeValue(kMaxArcTolerance) + " mm"};
        }
        lineNumber_ = s_.lineNumberStart;
        radiusOnlyArcs_ = arcsUseRadiusOnly();
        for (const auto& [value, what] : {std::pair{ctx_.home.x, "home X"}, std::pair{ctx_.home.y, "home Y"},
                                          std::pair{ctx_.home.z, "home Z"}}) {
            if (const std::string problem = rangeProblem(value, kMaxMachineValue, "mm"); !problem.empty()) {
                return Error{0, "cannot write G-code: job " + std::string(what) + " " + problem};
            }
        }

        std::vector<const Toolpath*> active;
        for (const Toolpath& tp : toolpaths_) {
            if (!tp.isEmpty()) {
                active.push_back(&tp);
                if (std::find(toolsUsed_.begin(), toolsUsed_.end(), tp.tool.name) == toolsUsed_.end()) {
                    toolsUsed_.push_back(tp.tool.name);
                }
            }
        }

        // A toolpath that runs with the spindle off (T-0154) needs the post's SPINDLE_OFF_PAUSE
        // block: without it the post cannot stop the spindle and pause before the toolpath.
        if (!has("SPINDLE_OFF_PAUSE")) {
            for (const Toolpath* tp : active) {
                if (tp->tool.spindleOff) {
                    return Error{0, "cannot write G-code: toolpath " + quoted(tp->name) +
                                        " runs with the spindle off (tool " + quoted(tp->tool.name) +
                                        " must not turn), but the post has no SPINDLE_OFF_PAUSE block to stop "
                                        "the spindle and pause for the operator before it"};
                }
            }
        }
        // After the last spindle-off toolpath of a drag section, before a spinning toolpath can
        // start the spindle, the operator unlocks it (owner decision 2026-11-03): SPINDLE_UNLOCK_PAUSE.
        if (!has("SPINDLE_UNLOCK_PAUSE")) {
            if (!active.empty() && active.back()->tool.spindleOff) {
                // The next program could start a spindle that is still locked (director 2026-11-03).
                return Error{0, "cannot write G-code: the job ends with the spindle-off toolpath " +
                                    quoted(active.back()->name) +
                                    ", but the post has no SPINDLE_UNLOCK_PAUSE block to pause for the operator "
                                    "to unlock the spindle before the program ends"};
            }
            for (std::size_t i = 1; i < active.size(); ++i) {
                if (active[i - 1]->tool.spindleOff && !active[i]->tool.spindleOff) {
                    return Error{0, "cannot write G-code: toolpath " + quoted(active[i]->name) +
                                        " starts the spindle after the spindle-off toolpath " +
                                        quoted(active[i - 1]->name) +
                                        ", but the post has no SPINDLE_UNLOCK_PAUSE block to pause for the "
                                        "operator to unlock the spindle before it"};
                }
            }
        }

        pos_ = ctx_.home;
        if (!active.empty()) {
            tool_ = active.front()->tool;
            toolpath_ = active.front();
        }
        lastFeed_ = tool_.feedRate;
        setStationary();
        emit("HEADER");
        if (failure_) {
            return *failure_;
        }

        for (std::size_t i = 0; i < active.size(); ++i) {
            const Toolpath& tp = *active[i];
            // Going from a spindle-off toolpath to a spinning one (or back) with the same tool is a
            // tool change too, so the spindle is restarted only through TOOLCHANGE + SPINDLE_ON.
            const bool toolChanged = i > 0 && (tp.tool.toolNumber != tool_.toolNumber || tp.tool.name != tool_.name ||
                                               tp.tool.spindleOff != tool_.spindleOff);
            if (toolChanged && tool_.spindleOff && !tp.tool.spindleOff) {
                // End of a drag section (T-0154): still in the context of the last spindle-off
                // toolpath ([T], [TOOLNAME] are the drag tool's; no spindle word may be written).
                setStationary();
                emit("SPINDLE_UNLOCK_PAUSE");
                if (failure_) {
                    return *failure_;
                }
            }
            previousToolNumber_ = i > 0 ? tool_.toolNumber : 0;
            tool_ = tp.tool;
            toolpath_ = &tp;
            modal_.clear();
            lastFeedWritten_.reset();
            lastFeed_ = tool_.feedRate;
            firstRapid_ = firstFeed_ = firstPlunge_ = firstArc_ = true;
            setStationary();

            if (i == 0) {
                if (s_.firstToolpathNewSegment) {
                    emit("NEW_SEGMENT");
                }
            } else if (toolChanged) {
                emit("TOOLCHANGE");
                emit("SPINDLE_ON");
            } else {
                emit("NEW_SEGMENT");
            }
            if (tool_.spindleOff) {
                // T-0154: spindle stop and operator pause right before the spindle-off toolpath.
                emit("SPINDLE_OFF_PAUSE");
            }

            if (failure_) {
                return *failure_;
            }

            moveIndex_ = 0;
            for (const Move& m : tp.moves) {
                ++moveIndex_;
                if (auto err = validateMove(m)) {
                    return *err;
                }
                if (auto err = doMove(m)) {
                    return *err;
                }
                if (failure_) {
                    return *failure_;
                }
            }
            moveIndex_ = 0;
        }

        if (tool_.spindleOff && !active.empty()) {
            // A job ending with a drag section: unlock before the program ends (T-0154), still in
            // the last spindle-off toolpath's context.
            setStationary();
            emit("SPINDLE_UNLOCK_PAUSE");
            if (failure_) {
                return *failure_;
            }
        }
        inFooter_ = true;
        setStationary();
        emit("FOOTER");
        if (failure_) {
            return *failure_;
        }
        return PostOutput{std::move(out_), std::move(warnings_)};
    }

   private:
    // ---- block selection -------------------------------------------------------------------

    bool has(std::string_view name) const { return pp_.hasBlock(name); }

    /// True when the post's arc blocks give the arc by radius only (no I/J/IA/JA), so full
    /// circles must be split in two.
    bool arcsUseRadiusOnly() const {
        bool radius = false;
        bool centre = false;
        for (std::string_view dir : {"CW", "CCW"}) {
            for (std::string_view pattern :
                 {"FIRST_%_ARC_MOVE", "%_ARC_MOVE", "FIRST_%_HELICAL_ARC_PLUNGE_MOVE",
                  "FIRST_%_HELICAL_ARC_MOVE", "%_HELICAL_ARC_MOVE"}) {
                std::string name(pattern);
                name.replace(name.find('%'), 1, dir);
                if (const PostBlock* b = pp_.block(name)) {
                    for (const std::string& line : b->lines) {
                        radius = radius || line.find("[Radius]") != std::string::npos;
                        for (std::string_view t : {"[I]", "[J]", "[IA]", "[JA]"}) {
                            centre = centre || line.find(t) != std::string::npos;
                        }
                    }
                }
            }
        }
        return radius && !centre;
    }

    void warnOnce(const std::string& message) {
        for (const Diagnostic& d : warnings_) {
            if (d.message == message) {
                return;
            }
        }
        warnings_.push_back({0, message});
    }

    // ---- safety checks (ADR-0020) -----------------------------------------------------------

    /// Where the interpreter is, for error messages.
    std::string where() const {
        std::string w = "in " + std::string(block_);
        if (toolpath_ != nullptr && !inFooter_) {
            w += " of toolpath " + quoted(toolpath_->name);
            if (moveIndex_ > 0) {
                w += ", move " + std::to_string(moveIndex_);
            }
        }
        return w;
    }

    /// Refuses a move whose coordinates, arc centre, feed override or dwell are not finite or
    /// beyond the machine range, before anything is computed from it.
    std::optional<Error> validateMove(const Move& m) const {
        std::vector<std::pair<double, const char*>> values{{m.to.x, "X"}, {m.to.y, "Y"}, {m.to.z, "Z"}};
        if (m.kind == MoveKind::ArcCW || m.kind == MoveKind::ArcCCW) {
            values.emplace_back(m.centre.x, "arc centre X");
            values.emplace_back(m.centre.y, "arc centre Y");
        }
        for (const auto& [value, what] : values) {
            if (const std::string problem = rangeProblem(value, kMaxMachineValue, "mm"); !problem.empty()) {
                return Error{0, "cannot write G-code: toolpath " + quoted(toolpath_->name) + ", move " +
                                    std::to_string(moveIndex_) + ": " + what + " " + problem};
            }
        }
        std::optional<std::pair<double, const char*>> other;
        if (m.feedOverride && m.kind != MoveKind::Rapid && m.kind != MoveKind::Dwell) {
            other = std::pair{*m.feedOverride, "mm/min"};
        } else if (m.kind == MoveKind::Dwell) {
            other = std::pair{m.dwellSeconds, "s"};
        }
        if (other) {
            if (const std::string problem = rangeProblem(other->first, kMaxMachineValue, other->second);
                !problem.empty()) {
                return Error{0, "cannot write G-code: toolpath " + quoted(toolpath_->name) + ", move " +
                                    std::to_string(moveIndex_) + ": " +
                                    (m.kind == MoveKind::Dwell ? "dwell " : "feed ") + problem};
            }
        }
        if (m.kind == MoveKind::Dwell && !(m.dwellSeconds > 0.0)) {
            // T-0134 (owner decision 2026-11-02): a dwell of zero or less is no pause.
            return Error{0, "cannot write G-code: toolpath " + quoted(toolpath_->name) + ", move " +
                                std::to_string(moveIndex_) + ": dwell " + describeValue(m.dwellSeconds) +
                                " s must be greater than zero"};
        }
        return std::nullopt;
    }

    /// Checks a numeric tag value before conversion; records the first failure.
    bool acceptValue(const TagInfo& info, double v) {
        std::optional<ValueLimit> limit = limitFor(info);
        if (info.tag == "F" && s_.inverseTimeMode && frame_.cutting) {
            limit = ValueLimit{kMaxInverseTime, "1/min"};  // [F] writes [FI] here
        }
        if (!limit) {
            return true;
        }
        const std::string problem = rangeProblem(v, limit->max, limit->unit);
        if (problem.empty()) {
            return true;
        }
        fail(0, "[" + std::string(info.tag) + "] " + problem);
        return false;
    }

    /// Checks a numeric value as it will be written (after units and multiplier).
    bool acceptWritten(const VarDef& def, double v) {
        const std::string problem = rangeProblem(v, kMaxWrittenValue, "");
        if (problem.empty()) {
            return true;
        }
        // Blame the VAR line only when its multiplier took part.
        fail(def.multiplier != 1.0 ? def.line : 0, "[" + def.tag + "] would be written as " + problem +
                           (def.multiplier != 1.0 ? " (VAR multiplier " + describeValue(def.multiplier) + ")"
                                                  : std::string{}));
        return false;
    }

    /// Refuses a spindle speed that is not a finite number above zero while a toolpath is being
    /// written: checked wherever [S] is written and before SPINDLE_ON (owner decision
    /// 2026-11-02, ADR-0020). A manual spindle takes any positive nominal speed.
    bool acceptSpindleSpeed() {
        if (toolpath_ == nullptr || (std::isfinite(tool_.spindleSpeed) && tool_.spindleSpeed > 0.0)) {
            return true;
        }
        fail(0, "tool " + quoted(tool_.name) + " has spindle speed " + describeValue(tool_.spindleSpeed) +
                    " rpm; it must be greater than zero (for a manual spindle, set any positive nominal speed)");
        return false;
    }

    /// Refuses a feed, spindle speed or dwell above zero that the post's number format would
    /// write as zero (F0, P0.00 …), and a feed rate, plunge rate or feed override of zero or less
    /// (F0, F-5): a zero feed or dwell is never what the toolpath asked for (owner decisions
    /// 2026-11-02, ADR-0020). Inverse time is judged by the feed it comes from; only a cutting
    /// move of zero length (inverse time 0 by definition) is exempt. In a job without toolpaths
    /// nothing cuts, so nothing is checked (as for the spindle speed).
    bool acceptNonZero(const TagInfo& info, const VarDef& def, double raw, const std::string& body) {
        const bool rate =
            info.kind == TagKind::FeedRate || info.tag == "FI" || info.tag == "S" || info.tag == "DWELL";
        if (!rate || toolpath_ == nullptr) {
            return true;
        }
        const bool inverse = info.tag == "FI" || (info.tag == "F" && s_.inverseTimeMode && frame_.cutting);
        if (inverse && (!frame_.cutting || frame_.length <= kZeroLength)) {
            return true;  // [FI] outside a cut, or a zero-length move: inverse time 0 is right
        }
        const std::optional<ValueLimit> limit = limitFor(info);
        const std::string unit = inverse ? "1/min" : limit ? limit->unit : "";
        const std::string written = std::string(detail::trim(def.prefix)) + body;
        const bool zeroWritten = body.find_first_of("123456789") == std::string::npos;
        if (info.kind == TagKind::FeedRate || inverse) {
            const double feed = inverse ? frame_.feed : raw;
            if (!(feed > 0.0)) {
                fail(0, "[" + std::string(info.tag) + "] feed " + describeValue(feed) + " mm/min" +
                            (inverse ? " (inverse time " + describeValue(raw) + " 1/min)" : std::string()) +
                            " would be written as " + written + "; feeds must be greater than zero");
                return false;
            }
        }
        // Above zero (for inverse time: from a feed above zero, even if the quotient underflowed).
        if (!zeroWritten || !(inverse || raw > 0.0)) {
            return true;
        }
        fail(def.multiplier != 1.0 ? def.line : 0,
             "[" + std::string(info.tag) + "] " + describeValue(raw) + " " + unit + " would be written as " +
                 written + "; it is above zero but too small for the post's " +
                 std::to_string(def.format.decimals) + "-decimal format");
        return false;
    }

    /// True while the toolpath being written (from its HEADER, TOOLCHANGE or NEW_SEGMENT to its
    /// last move) runs with the spindle off (T-0154).
    bool spindleOff() const { return toolpath_ != nullptr && !inFooter_ && tool_.spindleOff; }

    /// What a spindle-off toolpath is called in error messages.
    std::string spindleOffWhat() const {
        return "the toolpath runs with the spindle off (tool " + quoted(tool_.name) + " must not turn)";
    }

    void fail(int line, const std::string& message) {
        if (!failure_) {
            failure_ = Error{line, "cannot write G-code " + where() + ": " + message};
        }
    }

    // ---- moves -----------------------------------------------------------------------------

    std::optional<Error> doMove(const Move& m) {
        switch (m.kind) {
            case MoveKind::Rapid:
            case MoveKind::Feed:
            case MoveKind::Plunge:
            case MoveKind::Retract:
                return linear(m.kind, m.to, m.feedOverride);
            case MoveKind::ArcCW:
            case MoveKind::ArcCCW:
                return arc(m);
            case MoveKind::Dwell:
                if (!has("DWELL_MOVE")) {
                    warnOnce("the post has no DWELL_MOVE block; dwells are omitted");
                    return std::nullopt;
                }
                setStationary();
                frame_.dwell = m.dwellSeconds;
                emit("DWELL_MOVE");
                return std::nullopt;
        }
        return std::nullopt;
    }

    double rateFor(MoveKind kind, std::optional<double> override) const {
        if (override) {
            return *override;
        }
        return kind == MoveKind::Plunge || kind == MoveKind::Retract ? tool_.plungeRate : tool_.feedRate;
    }

    void prepareCutting(MoveKind kind, Vec3 to, double length, std::optional<double> override) {
        const double feed = rateFor(kind, override);
        if (has("FEED_RATE_CHANGE") && (!lastFeedWritten_ || *lastFeedWritten_ != feed)) {
            setStationary();
            frame_.feed = feed;
            emit("FEED_RATE_CHANGE");
        }
        lastFeedWritten_ = feed;
        lastFeed_ = feed;
        frame_ = Frame{};
        frame_.from = pos_;
        frame_.to = to;
        frame_.feed = feed;
        frame_.inverseTime = length > kZeroLength ? feed / length : 0.0;
        frame_.length = length;
        frame_.cutting = true;
    }

    std::optional<Error> linear(MoveKind kind, Vec3 to, std::optional<double> override) {
        std::string_view name;
        switch (kind) {
            case MoveKind::Rapid:
                if (firstRapid_ && has("INITIAL_RAPID_MOVE")) {
                    name = "INITIAL_RAPID_MOVE";
                } else if (has("RAPID_MOVE")) {
                    name = "RAPID_MOVE";
                } else if (has("FEED_MOVE")) {
                    warnOnce("the post has no RAPID_MOVE block; rapids are written with FEED_MOVE");
                    name = "FEED_MOVE";
                }
                firstRapid_ = false;
                break;
            case MoveKind::Plunge:
                name = firstPlunge_ && has("FIRST_PLUNGE_MOVE") ? "FIRST_PLUNGE_MOVE"
                       : has("PLUNGE_MOVE")                     ? "PLUNGE_MOVE"
                                                                : "FEED_MOVE";
                firstPlunge_ = false;
                break;
            case MoveKind::Retract:
                name = has("RETRACT_MOVE") ? "RETRACT_MOVE" : "FEED_MOVE";
                break;
            default:
                name = firstFeed_ && has("FIRST_FEED_MOVE") ? "FIRST_FEED_MOVE" : "FEED_MOVE";
                firstFeed_ = false;
                break;
        }
        if (name.empty() || !has(name)) {
            return Error{0, "the post has no block to write a " + std::string(kindName(kind)) +
                                " move (FEED_MOVE is missing)"};
        }
        if (kind == MoveKind::Rapid) {
            frame_ = Frame{};
            frame_.from = pos_;
            frame_.to = to;
            frame_.feed = lastFeed_;
        } else {
            prepareCutting(kind, to, geometry::distance(pos_, to), override);
        }
        emit(name);
        pos_ = to;
        return std::nullopt;
    }

    static std::string_view kindName(MoveKind kind) {
        switch (kind) {
            case MoveKind::Rapid:
                return "rapid";
            case MoveKind::Feed:
                return "feed";
            case MoveKind::Plunge:
                return "plunge";
            case MoveKind::Retract:
                return "retract";
            case MoveKind::ArcCW:
            case MoveKind::ArcCCW:
                return "arc";
            case MoveKind::Dwell:
                return "dwell";
        }
        return "unknown";
    }

    std::optional<Error> arc(const Move& m) {
        const bool cw = m.kind == MoveKind::ArcCW;
        const Vec3 from = pos_;
        const double r = toolpath::arcRadius(m, from.xy());
        const double sweep = toolpath::arcSweep(m, from.xy());
        const bool wrapping = s_.rotaryWrapX.has_value() || s_.rotaryWrapY.has_value();
        const bool linearise = wrapping || !has(cw ? "CW_ARC_MOVE" : "CCW_ARC_MOVE") || r <= kZeroLength ||
                               r < s_.minArcRadius || (s_.maxArcRadius > 0.0 && r > s_.maxArcRadius);
        if (linearise) {
            return lineariseArc(m, from, r, sweep);
        }
        if (radiusOnlyArcs_ && std::fabs(sweep) > kTwoPi - 1e-9) {
            // A full circle cannot be written by radius alone: write two half circles.
            const double a0 = std::atan2(from.y - m.centre.y, from.x - m.centre.x);
            const double am = a0 + sweep / 2.0;
            const Vec3 mid{m.centre.x + r * std::cos(am), m.centre.y + r * std::sin(am),
                           (from.z + m.to.z) / 2.0};
            Move first = m;
            first.to = mid;
            writeArc(first, from, r, sweep / 2.0);
            writeArc(m, mid, r, sweep / 2.0);
            return std::nullopt;
        }
        writeArc(m, from, r, sweep);
        return std::nullopt;
    }

    void writeArc(const Move& m, Vec3 from, double r, double sweep) {
        const std::string dir = m.kind == MoveKind::ArcCW ? "CW" : "CCW";
        const bool helical = std::fabs(m.to.z - from.z) > kZeroLength;
        std::vector<std::string> candidates;
        if (helical) {
            if (firstArc_ && m.to.z < from.z) {
                candidates.push_back("FIRST_" + dir + "_HELICAL_ARC_PLUNGE_MOVE");
            }
            if (firstArc_) {
                candidates.push_back("FIRST_" + dir + "_HELICAL_ARC_MOVE");
            }
            candidates.push_back(dir + "_HELICAL_ARC_MOVE");
        }
        if (firstArc_) {
            candidates.push_back("FIRST_" + dir + "_ARC_MOVE");
        }
        candidates.push_back(dir + "_ARC_MOVE");
        firstArc_ = false;

        const double planar = std::fabs(sweep) * r;
        const double dz = m.to.z - from.z;
        prepareCutting(m.kind, m.to, std::sqrt(planar * planar + dz * dz), m.feedOverride);
        frame_.from = from;
        frame_.move = &m;
        for (const std::string& name : candidates) {
            if (has(name)) {
                emit(name);
                break;
            }
        }
        pos_ = m.to;
    }

    std::optional<Error> lineariseArc(const Move& m, Vec3 from, double r, double sweep) {
        const double tol = ctx_.arcTolerance;  // validated in run()
        double segments = 1.0;
        if (r > tol) {
            const double maxStep = 2.0 * std::acos(1.0 - tol / r);
            segments = std::ceil(std::fabs(sweep) / maxStep);
        } else if (r > kZeroLength) {
            segments = std::ceil(std::fabs(sweep) / (kPi / 2.0));
        }
        if (!(segments >= 1.0)) {
            segments = 1.0;  // zero sweep (or NaN, which validated inputs cannot produce)
        }
        // An infinite count (chord angle rounded to zero) is capped like any large one, never
        // collapsed to a single chord.
        const int n = static_cast<int>(std::min(segments, kMaxArcSegments));
        const double a0 = std::atan2(from.y - m.centre.y, from.x - m.centre.x);
        for (int k = 1; k <= n; ++k) {
            const double t = static_cast<double>(k) / static_cast<double>(n);
            Vec3 p = m.to;
            if (k < n) {
                const double a = a0 + sweep * t;
                p = Vec3{m.centre.x + r * std::cos(a), m.centre.y + r * std::sin(a),
                         from.z + (m.to.z - from.z) * t};
            }
            if (auto err = linear(MoveKind::Feed, p, m.feedOverride)) {
                return err;
            }
        }
        return std::nullopt;
    }

    void setStationary() {
        frame_ = Frame{};
        frame_.from = pos_;
        frame_.to = pos_;
        frame_.feed = lastFeed_;
    }

    // ---- expansion -------------------------------------------------------------------------

    void emit(std::string_view blockName) {
        const PostBlock* b = pp_.block(blockName);
        if (b == nullptr || failure_) {
            return;
        }
        const bool off = spindleOff();
        if (blockName == "SPINDLE_ON" && off) {
            return;  // T-0154: the spindle stays off; SPINDLE_OFF_PAUSE follows instead
        }
        // Blocks that start the spindle of the shipped posts on a line of its own (T-0154).
        const bool dropSpindleLines =
            off && (blockName == "HEADER" || blockName == "TOOLCHANGE" || blockName == "NEW_SEGMENT");
        if (blockName == "SPINDLE_ON") {
            const std::string_view outer = block_;
            block_ = b->name;
            const bool ok = acceptSpindleSpeed();
            block_ = outer;
            if (!ok) {
                return;
            }
        }
        const std::string_view outerBlock = block_;
        block_ = b->name;
        for (const std::string& line : b->lines) {
            if (dropSpindleLines && line.find("[S]") != std::string::npos) {
                if (spindleOnlyLine(line)) {
                    continue;  // the spindle start of a spinning toolpath: left out
                }
                fail(0, spindleOffWhat() + ", but the line " + quoted(line) +
                            " writes [S] together with other codes; start the spindle on a line of its own");
                break;
            }
            std::string lineText;
            bool hadTag = false;
            std::size_t i = 0;
            while (i < line.size()) {
                const std::size_t open = line.find('[', i);
                if (open == std::string::npos) {
                    lineText.append(line, i, std::string::npos);
                    break;
                }
                lineText.append(line, i, open - i);
                const std::size_t close = line.find(']', open);  // validated by the parser
                lineText += render(std::string_view(line).substr(open + 1, close - open - 1), lineText);
                hadTag = true;
                i = close + 1;
            }
            while (!lineText.empty() && detail::isSpace(lineText.back())) {
                lineText.pop_back();
            }
            if (failure_) {
                break;
            }
            if (lineText.empty() && hadTag) {
                continue;  // every variable on the line was suppressed
            }
            if (off) {
                // Second guard (T-0154): no spindle start or speed reaches the output of a
                // spindle-off toolpath, whatever the post's template text says.
                if (const std::string word = spindleWord(liveCode(lineText)); !word.empty()) {
                    fail(0, spindleOffWhat() + ", but the post writes " + word + " (line " + quoted(lineText) + ")");
                    break;
                }
            }
            out_ += lineText;
            out_ += s_.lineEnding;
        }
        block_ = outerBlock;
    }

    /// Rotary angle in degrees for a point, from whichever linear axis is wrapped.
    double rotaryAngle(Vec3 p) const {
        const std::optional<RotaryWrap>& w = s_.rotaryWrapX ? s_.rotaryWrapX : s_.rotaryWrapY;
        if (!w) {
            return 0.0;
        }
        const double linear = s_.rotaryWrapX ? p.x : p.y;
        const double deg = linear / (kPi * ctx_.rotaryDiameter) * 360.0;
        return w->negate ? -deg : deg;
    }

    double axisValue(std::string_view tag, Vec3 p) const {
        if (tag == "X") {
            return p.x;
        }
        if (tag == "Y") {
            return p.y;
        }
        if (tag == "Z") {
            return p.z;
        }
        return rotaryAngle(p);  // "A"
    }

    double number(std::string_view tag) const {
        const TagInfo* info = findTag(tag);
        if (info != nullptr && info->axis) {
            return axisValue(tag, frame_.to);
        }
        const Vec3 from = frame_.from;
        if (frame_.move != nullptr) {
            const Move& m = *frame_.move;
            const double r = geometry::distance(m.centre, from.xy());
            const double sweep = toolpath::arcSweep(m, from.xy());
            const double a0 = std::atan2(from.y - m.centre.y, from.x - m.centre.x);
            if (tag == "I") return m.centre.x - from.x;
            if (tag == "J") return m.centre.y - from.y;
            if (tag == "IA") return m.centre.x;
            if (tag == "JA") return m.centre.y;
            if (tag == "ArcStartX") return from.x;
            if (tag == "ArcStartY") return from.y;
            if (tag == "ArcMidX") return m.centre.x + r * std::cos(a0 + sweep / 2.0);
            if (tag == "ArcMidY") return m.centre.y + r * std::sin(a0 + sweep / 2.0);
            if (tag == "Radius") return std::fabs(sweep) > kPi + 1e-9 ? -r : r;
            if (tag == "Angle") return std::fabs(sweep) * 180.0 / kPi;
        }
        if (tag == "F") return s_.inverseTimeMode && frame_.cutting ? frame_.inverseTime : frame_.feed;
        if (tag == "FC") return tool_.feedRate;
        if (tag == "FP") return tool_.plungeRate;
        if (tag == "FI") return frame_.inverseTime;
        if (tag == "S") return tool_.spindleSpeed;
        if (tag == "T") return static_cast<double>(tool_.toolNumber);
        if (tag == "TP") return static_cast<double>(previousToolNumber_);
        if (tag == "TDIA") return tool_.diameter;
        if (tag == "N") return static_cast<double>(lineNumber_);
        if (tag == "XH") return ctx_.home.x;
        if (tag == "YH") return ctx_.home.y;
        if (tag == "ZH") return ctx_.home.z;
        if (tag == "SAFEZ") return ctx_.safeZ;
        if (tag == "XMIN") return ctx_.materialMin.x;
        if (tag == "XMAX") return ctx_.materialMax.x;
        if (tag == "XLENGTH") return ctx_.materialMax.x - ctx_.materialMin.x;
        if (tag == "YMIN") return ctx_.materialMin.y;
        if (tag == "YMAX") return ctx_.materialMax.y;
        if (tag == "YLENGTH") return ctx_.materialMax.y - ctx_.materialMin.y;
        if (tag == "ZMIN") return ctx_.materialMin.z;
        if (tag == "ZMAX") return ctx_.materialMax.z;
        if (tag == "ZLENGTH") return ctx_.materialMax.z - ctx_.materialMin.z;
        if (tag == "DWELL") return frame_.dwell;
        return 0.0;  // arc tags outside arcs, drill-bank tags
    }

    std::string text(std::string_view tag) const {
        if (tag == "TOOLNAME") return tool_.name;
        if (tag == "TOOLPATH_NAME") return toolpath_ != nullptr ? toolpath_->name : std::string{};
        if (tag == "TOOLS_USED") {
            // Joined with "; ", not ", ": a comma right after the first name would continue
            // its word run at the start of a comment and be neutralised to `_` (ADR-0020).
            std::string joined;
            for (const std::string& n : toolsUsed_) {
                joined += joined.empty() ? n : "; " + n;
            }
            return joined;
        }
        if (tag == "TIME") return ctx_.time;
        if (tag == "DATE") return ctx_.date;
        if (tag == "TP_FILENAME") return ctx_.fileName;
        if (tag == "TP_DIR") return ctx_.fileDir;
        if (tag == "TP_EXT") return ctx_.fileExt;
        if (tag == "PATHNAME") return ctx_.pathName;
        if (tag == "PRODUCT") return ctx_.product;
        if (tag == "FILE_NOTES") return ctx_.fileNotes;
        return {};  // TOOL_NOTES, CUTTER_COMP: no data yet
    }

    double toOutputUnits(const TagInfo& info, double v) const {
        if (s_.units != Units::Inches) {
            return v;
        }
        if (info.kind == TagKind::Length) {
            return v / kMmPerInch;
        }
        if (info.kind == TagKind::FeedRate && !(info.tag == "F" && s_.inverseTimeMode && frame_.cutting)) {
            return v / kMmPerInch;
        }
        return v;
    }

    /// Renders one `[tag]`. `before` is the output line written so far; it decides the comment
    /// context a text value is sanitised for.
    std::string render(std::string_view tag, std::string_view before) {
        const TagInfo* info = findTag(tag);  // never null: the parser rejected unknown tags
        const VarDef* defined = pp_.var(tag);
        const VarDef def = defined != nullptr ? *defined : defaultVarDef(tag);

        // A linear axis wrapped onto the rotary axis is consumed by [A].
        if ((tag == "X" && s_.rotaryWrapX) || (tag == "Y" && s_.rotaryWrapY)) {
            return {};
        }

        std::string body;
        if (info->kind == TagKind::Text) {
            body = text(tag);
            for (char& c : body) {
                for (const auto& [from, to] : s_.substitute) {
                    if (c == from) {
                        c = to;
                        break;
                    }
                }
            }
            // After SUBSTITUTE, so no post setting can reintroduce a control character or an
            // early comment close. The prefix is part of the line before the value.
            body = detail::sanitiseText(body, detail::commentPositionAt(std::string(before) + def.prefix));
        } else if (def.incremental) {
            const double scale = std::pow(10.0, def.format.decimals);
            const auto quantise = [&](double v) { return std::round(v * scale) / scale; };
            const double endRaw = axisValue(tag, frame_.to);
            const double startRaw = axisValue(tag, frame_.from);
            if (!acceptValue(*info, endRaw) || !acceptValue(*info, startRaw)) {
                return {};
            }
            const double end = toOutputUnits(*info, endRaw) * def.multiplier;
            const double start = toOutputUnits(*info, startRaw) * def.multiplier;
            if (!acceptWritten(def, end) || !acceptWritten(def, start)) {
                return {};
            }
            const double delta = quantise(end) - quantise(start);
            if (def.mode == OutputMode::Changed && std::fabs(delta) < 0.5 / scale) {
                return {};
            }
            return def.prefix + formatNumber(def.format, delta);
        } else {
            const double raw = number(tag);
            if (tag == "S" && spindleOff()) {
                fail(0, spindleOffWhat() + "; the post writes [S] here");
                return {};
            }
            if (tag == "S" && !acceptSpindleSpeed()) {
                return {};
            }
            if (!acceptValue(*info, raw)) {
                return {};
            }
            const double written = toOutputUnits(*info, raw) * def.multiplier;
            if (!acceptWritten(def, written)) {
                return {};
            }
            body = formatNumber(def.format, written);
            if (!acceptNonZero(*info, def, raw, body)) {
                return {};
            }
        }

        if (tag == "N") {
            lineNumber_ += s_.lineNumberIncrement;
            if (lineNumber_ > s_.lineNumberMaximum) {
                lineNumber_ = s_.lineNumberStart;
            }
        }
        if (def.mode == OutputMode::Changed) {
            const auto it = modal_.find(def.tag);
            if (it != modal_.end() && it->second == body) {
                return {};
            }
            modal_[def.tag] = body;
        }
        return def.prefix + body;
    }

    const PostProcessor& pp_;
    const PostSettings& s_;
    const PostContext& ctx_;
    const std::vector<Toolpath>& toolpaths_;

    std::string out_;
    std::vector<Diagnostic> warnings_;
    std::map<std::string, std::string> modal_;
    std::vector<std::string> toolsUsed_;

    Frame frame_;
    Vec3 pos_;
    ToolSnapshot tool_;
    const Toolpath* toolpath_{nullptr};
    int previousToolNumber_{0};
    long long lineNumber_{0};
    double lastFeed_{0.0};
    std::optional<double> lastFeedWritten_;
    std::optional<Error> failure_;     // first unsafe value met while rendering (ADR-0020)
    std::string_view block_{"the program"};  // block being expanded, for error messages
    std::size_t moveIndex_{0};         // 1-based move of the current toolpath; 0 = none
    bool inFooter_{false};
    bool radiusOnlyArcs_{false};
    bool firstRapid_{true};
    bool firstFeed_{true};
    bool firstPlunge_{true};
    bool firstArc_{true};
};

}  // namespace

Result<PostOutput> PostProcessor::run(const std::vector<toolpath::Toolpath>& toolpaths,
                                      const PostContext& context) const {
    return Interpreter(*this, context, toolpaths).run();
}

}  // namespace carvetto::post
