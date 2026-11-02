// Parser for the `.pp` post-processor template grammar (ADR-0004, ADR-0011).
//
// The file is processed line by line. Outside a block a line is a comment (`+`), a VAR
// definition, `begin NAME`, or a `KEY = value` global setting. Inside a block it is a comment,
// a double-quoted output line, or `end`. Every `[tag]` reference in an output line is checked
// against the known tag table at parse time, so a typo is reported with its line number rather
// than surfacing as wrong G-code later.

#include <array>
#include <cstddef>
#include <string>
#include <string_view>

#include "carvetto/post/post_processor.hpp"
#include "text_util.hpp"

namespace carvetto::post {

namespace {

constexpr auto kKnownBlocks = std::to_array<std::string_view>({
    "HEADER",
    "FOOTER",
    "TOOLCHANGE",
    "NEW_SEGMENT",
    "SPINDLE_ON",
    "INITIAL_RAPID_MOVE",
    "RAPID_MOVE",
    "FIRST_RAPID_MOVE",
    "STRAIGHT_RAPID_MOVE",
    "FIRST_STRAIGHT_RAPID_MOVE",
    "INITIAL_STRAIGHT_RAPID_MOVE",
    "FIRST_FEED_MOVE",
    "FEED_MOVE",
    "FEED_RATE_CHANGE",
    "FIRST_CW_ARC_MOVE",
    "CW_ARC_MOVE",
    "FIRST_CCW_ARC_MOVE",
    "CCW_ARC_MOVE",
    "FIRST_CW_HELICAL_ARC_PLUNGE_MOVE",
    "FIRST_CCW_HELICAL_ARC_PLUNGE_MOVE",
    "FIRST_CW_HELICAL_ARC_MOVE",
    "FIRST_CCW_HELICAL_ARC_MOVE",
    "CW_HELICAL_ARC_MOVE",
    "CCW_HELICAL_ARC_MOVE",
    "FIRST_PLUNGE_MOVE",
    "PLUNGE_MOVE",
    "RETRACT_MOVE",
    "DWELL_MOVE",
    "JET_TOOL_POWER",
    "JET_TOOL_ON",
    "JET_TOOL_OFF",
    "SETUP_DRILL_BANK",
    "DRILL_BANK_PATTERN",
    "DRILL_BANK_EXTEND_DRILL",
    "DRILL_BANK_RETRACT_DRILL",
    "FINAL_DRILL_BANK_ACTION",
    "CUTTER_COMP_LEFT",
    "CUTTER_COMP_RIGHT",
    "CUTTER_COMP_OFF",
});

constexpr auto kKnownSettings = std::to_array<std::string_view>({
    "POST_NAME",
    "FILE_EXTENSION",
    "UNITS",
    "PRINT_DIRECT",
    "DIRECT_OUTPUT",
    "LINE_ENDING",
    "RAPID_PLUNGE_TO_STARTZ",
    "RAPID_SPEED",
    "MAXIMUM_LENGTH",
    "MAXIMUM_ROTATION",
    "MIN_ARC_RADIUS",
    "MAX_ARC_RADIUS",
    "ROTARY_WRAP_X",
    "ROTARY_WRAP_Y",
    "INVERSE_TIME_MODE",
    "SPINDLE_SPEED_RANGE",
    "LASER_SUPPORT",
    "SUBSTITUTE",
    "TAPE_SPLITTING",
    "FIRST_TOOLPATH_NEW_SEGMENT",
    "LINE_NUMBER_START",
    "LINE_NUMBER_INCREMENT",
    "LINE_NUMBER_MAXIMUM",
});

bool startsWithWord(std::string_view line, std::string_view word) {
    if (line.size() <= word.size() || !detail::iequals(line.substr(0, word.size()), word)) {
        return false;
    }
    return detail::isSpace(line[word.size()]);
}

std::string unquote(std::string_view v) {
    v = detail::trim(v);
    if (v.size() >= 2 && v.front() == '"' && v.back() == '"') {
        v = v.substr(1, v.size() - 2);
    }
    return std::string(v);
}

std::optional<bool> parseYesNo(std::string_view v) {
    const std::string u = detail::upper(detail::trim(v));
    if (u == "YES" || u == "TRUE" || u == "1" || u == "ON") {
        return true;
    }
    if (u == "NO" || u == "FALSE" || u == "0" || u == "OFF" || u.empty()) {
        return false;
    }
    return std::nullopt;
}

/// Decodes `[13][10]`-style character codes; other text is copied literally.
std::optional<std::string> decodeLineEnding(std::string_view v) {
    std::string out;
    std::size_t i = 0;
    while (i < v.size()) {
        if (v[i] == '[') {
            const std::size_t close = v.find(']', i);
            if (close == std::string_view::npos) {
                return std::nullopt;
            }
            const auto code = detail::parseInteger(v.substr(i + 1, close - i - 1));
            if (!code || *code < 1 || *code > 255) {
                return std::nullopt;
            }
            out.push_back(static_cast<char>(static_cast<unsigned char>(*code)));
            i = close + 1;
        } else {
            out.push_back(v[i]);
            ++i;
        }
    }
    return out;
}

/// Checks every `[tag]` reference in an output line.
std::optional<Error> checkReferences(std::string_view text, int line) {
    std::size_t i = 0;
    while ((i = text.find('[', i)) != std::string_view::npos) {
        const std::size_t close = text.find(']', i);
        if (close == std::string_view::npos) {
            return Error{line, "unterminated '[' in output line"};
        }
        const std::string_view tag = text.substr(i + 1, close - i - 1);
        if (findTag(tag) == nullptr) {
            return Error{line, "unknown variable tag '" + std::string(tag) + "'"};
        }
        i = close + 1;
    }
    return std::nullopt;
}

}  // namespace

bool PostProcessor::isKnownBlock(std::string_view name) {
    for (std::string_view b : kKnownBlocks) {
        if (b == name) {
            return true;
        }
    }
    return false;
}

const PostBlock* PostProcessor::block(std::string_view name) const {
    const auto it = blocks_.find(name);
    return it == blocks_.end() ? nullptr : &it->second;
}

const VarDef* PostProcessor::var(std::string_view tag) const {
    const auto it = vars_.find(tag);
    return it == vars_.end() ? nullptr : &it->second;
}

Result<PostProcessor> PostProcessor::parse(std::string_view source) {
    PostProcessor pp;
    PostSettings& s = pp.settings_;
    PostBlock* open = nullptr;
    PostBlock pending;
    int lineNo = 0;

    std::size_t pos = 0;
    while (pos <= source.size()) {
        const std::size_t nl = source.find('\n', pos);
        const std::string_view raw =
            source.substr(pos, nl == std::string_view::npos ? std::string_view::npos : nl - pos);
        pos = nl == std::string_view::npos ? source.size() + 1 : nl + 1;
        ++lineNo;

        const std::string_view t = detail::trim(raw);
        if (t.empty() || t.front() == '+') {
            continue;
        }

        if (open != nullptr) {
            if (detail::iequals(t, "end")) {
                const std::string name = pending.name;
                if (pp.blocks_.count(name) != 0) {
                    pp.warnings_.push_back(
                        {pending.line,
                         "block '" + name + "' defined more than once; the last definition is used"});
                }
                pp.blocks_[name] = std::move(pending);
                pending = PostBlock{};
                open = nullptr;
            } else if (t.front() == '"') {
                if (t.size() < 2 || t.back() != '"') {
                    return Error{lineNo, "output line is missing its closing quote"};
                }
                const std::string_view text = t.substr(1, t.size() - 2);
                if (auto err = checkReferences(text, lineNo)) {
                    return *err;
                }
                open->lines.emplace_back(text);
            } else if (startsWithWord(t, "begin")) {
                return Error{lineNo, "'begin' inside block '" + open->name + "' (missing 'end')"};
            } else {
                return Error{lineNo,
                             "expected a quoted output line or 'end' inside block '" + open->name + "'"};
            }
            continue;
        }

        if (startsWithWord(t, "begin")) {
            const std::string name(detail::trim(t.substr(5)));
            pending = PostBlock{name, lineNo, {}};
            open = &pending;
            if (!isKnownBlock(name)) {
                pp.warnings_.push_back({lineNo, "unknown block '" + name + "' is ignored"});
            }
            continue;
        }
        if (detail::iequals(t, "begin")) {
            return Error{lineNo, "'begin' without a block name"};
        }
        if (detail::iequals(t, "end")) {
            return Error{lineNo, "'end' without a matching 'begin'"};
        }

        const std::size_t eq = t.find('=');
        if (eq == std::string_view::npos) {
            return Error{lineNo, "unrecognised line '" + std::string(t) + "'"};
        }

        if (startsWithWord(t, "VAR")) {
            const std::string name(detail::trim(t.substr(3, eq - 3)));
            if (name.empty()) {
                return Error{lineNo, "VAR definition without a name"};
            }
            auto def = parseVarSpec(t.substr(eq + 1), lineNo);
            if (!def) {
                return def.error();
            }
            def.value().name = name;
            const std::string tag = def.value().tag;
            if (pp.vars_.count(tag) != 0) {
                pp.warnings_.push_back(
                    {lineNo, "tag '" + tag + "' already has a VAR definition; the last definition is used"});
            }
            pp.vars_[tag] = std::move(def.value());
            continue;
        }

        const std::string key(detail::trim(t.substr(0, eq)));
        const std::string value = unquote(t.substr(eq + 1));
        s.raw[key] = value;
        bool known = false;
        for (std::string_view k : kKnownSettings) {
            known = known || k == key;
        }
        if (!known) {
            pp.warnings_.push_back({lineNo, "unknown setting '" + key + "' is ignored"});
            continue;
        }

        const auto badValue = [&](std::string_view expected) {
            return Error{lineNo, "invalid value '" + value + "' for " + key + " (expected " +
                                     std::string(expected) + ")"};
        };
        const auto yesNo = [&](bool& out) -> std::optional<Error> {
            const auto b = parseYesNo(value);
            if (!b) {
                return badValue("YES or NO");
            }
            out = *b;
            return std::nullopt;
        };
        const auto number = [&](double& out) -> std::optional<Error> {
            const auto d = detail::parseDouble(value);
            if (!d) {
                return badValue("a number");
            }
            out = *d;
            return std::nullopt;
        };
        const auto integer = [&](long long& out) -> std::optional<Error> {
            const auto n = detail::parseInteger(value);
            if (!n) {
                return badValue("an integer");
            }
            out = *n;
            return std::nullopt;
        };
        const auto wrap = [&](std::optional<RotaryWrap>& out) -> std::optional<Error> {
            std::string u = detail::upper(detail::trim(value));
            if (u.empty() || u == "NO") {
                out.reset();
                return std::nullopt;
            }
            RotaryWrap w;
            if (u.front() == '-' || u.front() == '+') {
                w.negate = u.front() == '-';
                u.erase(0, 1);
            }
            if (u.size() != 1 || u[0] < 'A' || u[0] > 'Z') {
                return badValue("a rotary axis letter such as A or -A");
            }
            w.axis = u[0];
            out = w;
            return std::nullopt;
        };

        std::optional<Error> err;
        if (key == "POST_NAME") {
            s.postName = value;
        } else if (key == "FILE_EXTENSION") {
            s.fileExtension = value;
        } else if (key == "UNITS") {
            const std::string u = detail::upper(value);
            if (u == "MM" || u == "MILLIMETRES" || u == "MILLIMETERS" || u == "METRIC") {
                s.units = Units::Millimetres;
            } else if (u == "INCHES" || u == "INCH" || u == "IN") {
                s.units = Units::Inches;
            } else {
                err = badValue("MM or INCHES");
            }
        } else if (key == "LINE_ENDING") {
            const auto le = decodeLineEnding(value);
            if (!le) {
                err = badValue("text with [nnn] character codes");
            } else {
                s.lineEnding = *le;
            }
        } else if (key == "RAPID_PLUNGE_TO_STARTZ") {
            err = yesNo(s.rapidPlungeToStartZ);
        } else if (key == "INVERSE_TIME_MODE") {
            err = yesNo(s.inverseTimeMode);
        } else if (key == "FIRST_TOOLPATH_NEW_SEGMENT") {
            err = yesNo(s.firstToolpathNewSegment);
        } else if (key == "LASER_SUPPORT") {
            err = yesNo(s.laserSupport);
        } else if (key == "MIN_ARC_RADIUS") {
            err = number(s.minArcRadius);
        } else if (key == "MAX_ARC_RADIUS") {
            err = number(s.maxArcRadius);
        } else if (key == "ROTARY_WRAP_X") {
            err = wrap(s.rotaryWrapX);
        } else if (key == "ROTARY_WRAP_Y") {
            err = wrap(s.rotaryWrapY);
        } else if (key == "SUBSTITUTE") {
            if (value.size() % 2 != 0) {
                err = badValue("an even number of characters (from/to pairs)");
            } else {
                s.substitute.clear();
                for (std::size_t i = 0; i < value.size(); i += 2) {
                    s.substitute.emplace_back(value[i], value[i + 1]);
                }
            }
        } else if (key == "LINE_NUMBER_START") {
            err = integer(s.lineNumberStart);
        } else if (key == "LINE_NUMBER_INCREMENT") {
            err = integer(s.lineNumberIncrement);
        } else if (key == "LINE_NUMBER_MAXIMUM") {
            err = integer(s.lineNumberMaximum);
        }
        // Remaining known settings (PRINT_DIRECT, RAPID_SPEED, …) are stored in `raw` only.
        if (err) {
            return *err;
        }
    }

    if (open != nullptr) {
        return Error{pending.line, "block '" + pending.name + "' has no 'end'"};
    }
    if (s.rotaryWrapX && s.rotaryWrapY) {
        return Error{0, "ROTARY_WRAP_X and ROTARY_WRAP_Y cannot both be set"};
    }
    return pp;
}

}  // namespace carvetto::post
