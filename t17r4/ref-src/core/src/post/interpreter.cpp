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

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <string_view>
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

/// What the tags of the block being written refer to.
struct Frame {
    Vec3 from;
    Vec3 to;
    const Move* move{nullptr};  // arc moves only: centre for I/J etc.
    double feed{0.0};           // mm/min
    double inverseTime{0.0};    // 1/min
    double dwell{0.0};          // seconds
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
        lineNumber_ = s_.lineNumberStart;
        radiusOnlyArcs_ = arcsUseRadiusOnly();

        std::vector<const Toolpath*> active;
        for (const Toolpath& tp : toolpaths_) {
            if (!tp.isEmpty()) {
                active.push_back(&tp);
                if (std::find(toolsUsed_.begin(), toolsUsed_.end(), tp.tool.name) == toolsUsed_.end()) {
                    toolsUsed_.push_back(tp.tool.name);
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

        for (std::size_t i = 0; i < active.size(); ++i) {
            const Toolpath& tp = *active[i];
            const bool toolChanged =
                i > 0 && (tp.tool.toolNumber != tool_.toolNumber || tp.tool.name != tool_.name);
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

            for (const Move& m : tp.moves) {
                if (auto err = doMove(m)) {
                    return *err;
                }
            }
        }

        setStationary();
        emit("FOOTER");
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
        const double tol = ctx_.arcTolerance > 0.0 ? ctx_.arcTolerance : 0.01;
        double segments = 1.0;
        if (r > tol) {
            const double maxStep = 2.0 * std::acos(1.0 - tol / r);
            segments = std::ceil(std::fabs(sweep) / maxStep);
        } else if (r > kZeroLength) {
            segments = std::ceil(std::fabs(sweep) / (kPi / 2.0));
        }
        const int n = static_cast<int>(std::clamp(segments, 1.0, 100000.0));
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
        if (b == nullptr) {
            return;
        }
        for (const std::string& line : b->lines) {
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
                lineText += render(std::string_view(line).substr(open + 1, close - open - 1));
                hadTag = true;
                i = close + 1;
            }
            while (!lineText.empty() && detail::isSpace(lineText.back())) {
                lineText.pop_back();
            }
            if (lineText.empty() && hadTag) {
                continue;  // every variable on the line was suppressed
            }
            out_ += lineText;
            out_ += s_.lineEnding;
        }
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
            std::string joined;
            for (const std::string& n : toolsUsed_) {
                joined += joined.empty() ? n : ", " + n;
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

    std::string render(std::string_view tag) {
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
        } else if (def.incremental) {
            const double scale = std::pow(10.0, def.format.decimals);
            const auto quantise = [&](double v) { return std::round(v * scale) / scale; };
            const double end = toOutputUnits(*info, axisValue(tag, frame_.to)) * def.multiplier;
            const double start = toOutputUnits(*info, axisValue(tag, frame_.from)) * def.multiplier;
            const double delta = quantise(end) - quantise(start);
            if (def.mode == OutputMode::Changed && std::fabs(delta) < 0.5 / scale) {
                return {};
            }
            return def.prefix + formatNumber(def.format, delta);
        } else {
            body = formatNumber(def.format, toOutputUnits(*info, number(tag)) * def.multiplier);
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
