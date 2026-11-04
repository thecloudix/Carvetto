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

/// Longest line written, on every post (T-0162, ADR-0020): characters of the line, comments
/// included, line ending excluded. LinuxCNC reads a line with `fgets(raw_line, LINELEN, …)`,
/// LINELEN 255, and refuses one whose read reaches 254 characters with NCE_COMMAND_TOO_LONG (so a
/// line plus its CR LF must stay below 254); grblHAL keeps comments in its 257-byte line buffer
/// (256 characters) and drops a longer line with error:overflow. 250 + CR LF is 252.
constexpr std::size_t kMaxLineLength = 250;
/// What a text value in a comment that is cut to fit `kMaxLineLength` ends with (T-0162).
constexpr std::string_view kEllipsis = "...";

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

/// A user-supplied name or post line for an error message: quoted, sanitised and cut short. A tab
/// is shown as `<TAB>` (T-0161): sanitising would turn it into a space, hiding the very character
/// a drag-section refusal is about.
std::string quoted(std::string_view name) {
    std::string_view rest = name.substr(0, kMaxQuotedName);
    std::string s;
    for (std::size_t tab = rest.find('\t'); tab != std::string_view::npos; tab = rest.find('\t')) {
        s += detail::sanitiseText(rest.substr(0, tab), {}) + "<TAB>";
        rest.remove_prefix(tab + 1);
    }
    s += detail::sanitiseText(rest, {});
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

/// The G-code of a line outside its comments, as Grbl and Mach read it: `(` opens a comment and
/// the FIRST `)` closes it (no nesting); with `semicolonComments` (posts whose comment style is
/// `;`, such as Marlin) `;` outside a comment makes the rest of the line a comment.
std::string liveCode(std::string_view line, bool semicolonComments) {
    std::string code;
    bool open = false;
    for (const char c : line) {
        if (open) {
            open = c != ')';
        } else if (c == '(') {
            open = true;
        } else if (c == ';' && semicolonComments) {
            break;
        } else {
            code += c;
        }
    }
    return code;
}

/// Why the parentheses of a line are not plain, unnested, closed comments ("" when they are):
/// a `(` inside an open comment, a `)` without one, or a comment left open (T-0154).
std::string parenProblem(std::string_view line, bool semicolonComments) {
    if (semicolonComments) {
        // ';'-style posts (Marlin): '(' is not a comment there (without PAREN_COMMENTS), so no
        // parenthesis may appear before the ';' comment.
        const std::string_view live = line.substr(0, line.find(';'));
        if (live.find_first_of("()") != std::string_view::npos) {
            return "a parenthesis outside the ';' comment (this post's comments start with ';')";
        }
        return {};
    }
    bool open = false;
    for (const char c : line) {
        if (open) {
            if (c == '(') {
                return "a '(' inside an open comment (comments end at the first ')')";
            }
            open = c != ')';
        } else if (c == '(') {
            open = true;
        } else if (c == ')') {
            return "a ')' without an open comment";
        } else if (c == ';' && semicolonComments) {
            return {};
        }
    }
    return open ? "a comment that is not closed on its line" : std::string();
}

/// The first byte that is not printable ASCII (0x20-0x7E), described; "" when none. Used only on
/// lines written in a drag section, where a tab is refused too (T-0154): Marlin does not treat a
/// tab as a separator ("N0<TAB>M0" is an unknown command, so the pause is skipped), and no shipped
/// post writes tabs.
std::string nonAsciiProblem(std::string_view text) {
    for (const char ch : text) {
        const auto b = static_cast<unsigned char>(ch);
        if (ch == '\t') {
            return "a tab (separate words with spaces)";
        }
        if (b < 0x20 || b > 0x7E) {
            static const char* const kHex = "0123456789ABCDEF";
            return std::string("the byte 0x") + kHex[b >> 4] + kHex[b & 0xF] + " (not printable ASCII)";
        }
    }
    return {};
}

bool isLetter(char c) {
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z');
}

bool isDigit(char c) {
    return c >= '0' && c <= '9';
}

char upperLetter(char c) {
    return c >= 'a' && c <= 'z' ? static_cast<char>(c - 'a' + 'A') : c;
}

/// One word of live G-code as a controller reads it: an address letter and its value.
struct GWord {
    char letter;       ///< upper case
    double value;      ///< signed value
    std::string text;  ///< as written, without spaces, e.g. "M03", "S.5", "M+3"
};

/// The words of live (comment-free) G-code, read the way RS274/NGC controllers and Grbl read
/// them (T-0154): spaces and tabs are ignored everywhere, so `M 3`, `M<TAB>03` and `S 1000` are
/// words; a value may have a sign and a leading `.` (`M+3`, `S.5`); letters are case-insensitive.
/// A letter without a value (message text such as Marlin's `M0 Drag tool 3 spindle off`) is
/// skipped, so every letter followed by a number counts, wherever it stands.
std::vector<GWord> gcodeWords(std::string_view code) {
    std::string c;
    for (const char ch : code) {
        if (ch != ' ' && ch != '\t') {
            c += ch;
        }
    }
    std::vector<GWord> words;
    for (std::size_t i = 0; i < c.size(); ++i) {
        if (!isLetter(c[i])) {
            continue;
        }
        std::size_t j = i + 1;
        bool negative = false;
        if (j < c.size() && (c[j] == '+' || c[j] == '-')) {
            negative = c[j] == '-';
            ++j;
        }
        const std::size_t numberStart = j;
        bool digit = false;
        while (j < c.size() && (isDigit(c[j]) || c[j] == '.')) {
            digit = digit || isDigit(c[j]);
            ++j;
        }
        if (!digit) {
            continue;
        }
        double v = 0.0;
        const auto r = std::from_chars(c.data() + numberStart, c.data() + j, v);
        if (r.ec != std::errc{}) {
            v = 0.0;  // malformed (e.g. "1.2.3"): still a word, its letter is what matters
        }
        words.push_back(GWord{upperLetter(c[i]), negative ? -v : v, c.substr(i, j - i)});
        i = j - 1;
    }
    return words;
}

bool isSpindleStart(const GWord& w) {
    return w.letter == 'M' && (w.value == 3.0 || w.value == 4.0);
}

/// True when a block line (template text) that writes `[S]` does nothing but start the spindle:
/// outside its comments it holds only `M3`/`M4` words, `[S]` (optionally after a literal `S`),
/// `[N]` and spaces. Such a line is left out for a toolpath that runs with the spindle off
/// (T-0154); any other line writing `[S]` there is an error, so leaving it out never drops a
/// tool change or another code.
bool spindleOnlyLine(std::string_view line, bool semicolonComments) {
    const std::string code = liveCode(line, semicolonComments);
    // Tags become markers: [S] -> '#', [N] -> nothing; any other tag disqualifies the line.
    std::string rest;
    for (std::size_t i = 0; i < code.size(); ++i) {
        if (code[i] == '[') {
            const std::size_t close = code.find(']', i);
            if (close == std::string::npos) {
                return false;
            }
            const std::string_view tag = std::string_view(code).substr(i + 1, close - i - 1);
            if (tag == "S") {
                rest += '#';
            } else if (tag != "N") {
                return false;
            }
            i = close;
        } else {
            rest += code[i];
        }
    }
    // Every remaining word must be M3/M4; besides words only '#', a literal S before it and spaces.
    for (const GWord& w : gcodeWords(rest)) {
        if (!isSpindleStart(w)) {
            return false;
        }
    }
    std::string stripped;
    for (const char ch : rest) {
        if (ch != ' ' && ch != '\t') {
            stripped += ch;
        }
    }
    for (std::size_t i = 0; i < stripped.size(); ++i) {
        const char ch = stripped[i];
        if (ch == '#') {
            continue;
        }
        if (upperLetter(ch) == 'S' && i + 1 < stripped.size() && stripped[i + 1] == '#') {
            continue;
        }
        if (upperLetter(ch) == 'M') {
            std::size_t j = i + 1;
            while (j < stripped.size() && (isDigit(stripped[j]) || stripped[j] == '.' || stripped[j] == '+')) {
                ++j;
            }
            if (j == i + 1) {
                return false;  // a bare M
            }
            i = j - 1;
            continue;
        }
        return false;
    }
    return true;
}

/// The first command of a line's words (a leading N line number skipped), alone.
std::vector<GWord> firstCommand(std::vector<GWord> words) {
    if (!words.empty() && words.front().letter == 'N') {
        words.erase(words.begin());
    }
    if (words.size() > 1) {
        words.resize(1);
    }
    return words;
}

/// True when an operator stop (M0 or M1) comes before the first M5 in `words` (T-0162): in a
/// pause block the spindle must be stopped before the program pauses for the operator.
bool pauseBeforeStop(const std::vector<GWord>& words) {
    for (const GWord& w : words) {
        if (w.letter == 'M' && w.value == 5.0) {
            return false;
        }
        if (w.letter == 'M' && (w.value == 0.0 || w.value == 1.0)) {
            return true;
        }
    }
    return false;
}

/// The words a block writes as template text, outside comments and with its tags left out
/// (used to check that a pause block stops the spindle and pauses). Lines that start with `/`
/// (block delete: the controller may skip them) do not count; with `firstWordOnly` (Marlin, one
/// command per line) only the first word of each line counts.
std::vector<GWord> blockWords(const PostBlock& block, bool semicolonComments, bool firstWordOnly) {
    std::vector<GWord> out;
    for (const std::string& line : block.lines) {
        const std::string_view trimmed = detail::trim(line);
        if (!trimmed.empty() && trimmed.front() == '/') {
            continue;
        }
        std::string code = liveCode(line, semicolonComments);
        std::string untagged;
        for (std::size_t i = 0; i < code.size(); ++i) {
            if (code[i] == '[') {
                const std::size_t close = code.find(']', i);
                i = close == std::string::npos ? code.size() : close;
                untagged += ' ';
            } else {
                untagged += code[i];
            }
        }
        std::vector<GWord> words = gcodeWords(untagged);
        if (firstWordOnly) {
            words = firstCommand(std::move(words));  // Marlin runs one command per line
        }
        for (GWord& w : words) {
            out.push_back(std::move(w));
        }
    }
    return out;
}

/// Why a block line (template text) may not be written while a spindle-off toolpath is active
/// ("" when it may), T-0154. Strict on purpose: printable ASCII only (no tab); no `#` (RS274/NGC
/// parameters); no M or S followed by a tag (`M[T]`); plain, unnested, closed comments; and no
/// M3/M4/S-number word anywhere in the template text, comments included (it is the post author's
/// fixed text and has no reason to mention them there). Tags count as spaces.
bool hasEmergencyCode(std::string_view text);

std::string templateProblem(std::string_view line, bool semicolonComments) {
    if (std::string bad = nonAsciiProblem(line); !bad.empty()) {
        return bad;
    }
    if (hasEmergencyCode(line)) {
        return "a Marlin emergency-parser code (M108, M112, M410, M876)";
    }
    if (line.find('#') != std::string_view::npos) {
        return "a '#' (parameters are not allowed here)";
    }
    if (std::string bad = parenProblem(line, semicolonComments); !bad.empty()) {
        return bad;
    }
    std::string untagged;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char u = upperLetter(line[i]);
        if (u == 'M' || u == 'S') {
            std::size_t j = i + 1;
            while (j < line.size() && line[j] == ' ') {
                ++j;
            }
            if (j < line.size() && line[j] == '[') {
                return std::string("an ") + u + " followed by a tag";
            }
        }
        if (line[i] == '[') {
            const std::size_t close = line.find(']', i);
            i = close == std::string_view::npos ? line.size() : close;
            untagged += ' ';
        } else {
            untagged += line[i];
        }
    }
    for (const GWord& w : gcodeWords(untagged)) {
        if (w.letter == 'S' || isSpindleStart(w)) {
            return "the spindle word " + w.text + " (refused anywhere in the template text, comments included)";
        }
    }
    return {};
}

/// The allowlist of the live G-code written in a drag section (T-0154, owner decision
/// 2026-11-03: an allowlist, not a blocklist). Exactly the codes the shipped posts write there.
/// G codes, by value: 0 1 2 3 4 17 20 21 40 43 49 80 90 91 91.1 94.
constexpr std::array<std::string_view, 16> kDragGCodes{"0",  "1",  "2",  "3",  "4",  "17", "20",   "21",
                                                       "40", "43", "49", "80", "90", "91", "91.1", "94"};
/// M codes, by integer value: 0 1 2 5 6 30 (never 3 or 4: the spindle stays off).
constexpr std::array<int, 6> kDragMCodes{0, 1, 2, 5, 6, 30};

bool allDigits(std::string_view v) {
    return !v.empty() && std::all_of(v.begin(), v.end(), [](char c) { return isDigit(c); });
}

/// `v` without leading zeros in its integer part and trailing zeros in its fraction ("091.10" ->
/// "91.1", "00" -> "0"); `v` is digits with at most one '.'.
std::string normalisedDecimal(std::string_view v) {
    const std::size_t dot = v.find('.');
    std::string_view whole = v.substr(0, dot);
    std::string_view frac = dot == std::string_view::npos ? std::string_view() : v.substr(dot + 1);
    while (whole.size() > 1 && whole.front() == '0') {
        whole.remove_prefix(1);
    }
    while (!frac.empty() && frac.back() == '0') {
        frac.remove_suffix(1);
    }
    return std::string(whole.empty() ? "0" : whole) + (frac.empty() ? "" : "." + std::string(frac));
}

/// True for digits with at most one decimal point, a digit first ("12", "0.100", "3."). With
/// `sign`, an optional leading '+' or '-'.
bool isPlainNumber(std::string_view v, bool sign) {
    if (sign && !v.empty() && (v.front() == '+' || v.front() == '-')) {
        v.remove_prefix(1);
    }
    if (v.empty() || !isDigit(v.front()) || std::count(v.begin(), v.end(), '.') > 1) {
        return false;
    }
    return std::all_of(v.begin(), v.end(), [](char c) { return isDigit(c) || c == '.'; });
}

/// Why one word of drag-section live G-code is not on the allowlist ("" when it is).
std::string wordProblem(char letter, std::string_view value) {
    const std::string word = std::string(1, letter) + std::string(value);
    // Digit caps (leading zeros included): codes and counters at most 5 digits (N: 9); numbers at most 8
    // integer and 6 decimal digits (controllers truncate or round longer ones).
    const std::size_t dot = value.find('.');
    std::string_view whole = value.substr(0, dot);
    if (!whole.empty() && (whole.front() == '+' || whole.front() == '-')) {
        whole.remove_prefix(1);
    }
    const std::size_t decimals = dot == std::string_view::npos ? 0 : value.size() - dot - 1;
    const bool number = letter == 'X' || letter == 'Y' || letter == 'Z' || letter == 'I' || letter == 'J' ||
                        letter == 'F' || letter == 'P';
    const std::size_t maxDigits = letter == 'N' ? 9 : 5;  // Mach3 numbers lines up to N999990
    if (number ? (whole.size() > 8 || decimals > 6) : (whole.size() + decimals > maxDigits)) {
        return "the word " + word + " (too many digits)";
    }
    switch (letter) {
        case 'X':
        case 'Y':
        case 'Z':
        case 'I':
        case 'J':
        case 'F':
        case 'P':
            return isPlainNumber(value, true) ? std::string() : "the word " + word + " (not a plain number)";
        case 'N':
            if (!allDigits(value)) {
                return "the word " + word + " (needs plain digits)";
            }
            // Grbl refuses a line number above MAX_LINE_NUMBER (10,000,000) and drops the line
            // (T-0161); the cap holds on every post because a post does not say which controller
            // it is for. Mach3 numbers lines up to N999990.
            return normalisedDecimal(value).size() <= 7 ? std::string()
                                                        : "the word " + word + " (line numbers above 9999999)";
        case 'T':
        case 'H':
            return allDigits(value) ? std::string() : "the word " + word + " (needs plain digits)";
        case 'G': {
            if (!isPlainNumber(value, false)) {
                return "the word " + word + " (not a plain number)";
            }
            const std::string n = normalisedDecimal(value);
            const bool ok = std::find(kDragGCodes.begin(), kDragGCodes.end(), n) != kDragGCodes.end();
            return ok ? std::string() : "the code " + word + " (not on the drag-section allowlist)";
        }
        case 'M': {
            if (!allDigits(value)) {
                return "the word " + word + " (M codes need plain digits)";
            }
            const std::string n = normalisedDecimal(value);
            const bool ok = n.size() <= 2 && std::find(kDragMCodes.begin(), kDragMCodes.end(), std::stoi(n)) !=
                                                 kDragMCodes.end();
            return ok ? std::string() : "the code " + word + " (not on the drag-section allowlist)";
        }
        default:
            return "the word " + word + " (the letter " + std::string(1, letter) +
                   " is not on the drag-section allowlist)";
    }
}

/// Why the live G-code of a line written in a drag section is not on the allowlist ("" when it
/// is). It must be only words LETTER + value separated by spaces: upper-case letters X Y Z I J F P
/// (signed plain numbers), N T H (plain digits), G (a code from `kDragGCodes`) and M (a code from
/// `kDragMCodes`, plain digits). Any other character refuses the line (tabs are refused before
/// this is called). A line with M0 or M1 holds nothing but an optional N word and the M0/M1
/// (`pauseLine` is set for it), on every post; with `bareStop` (the lines of the pause blocks,
/// T-0161) so does a line with M5.
std::string allowlistProblem(std::string_view live, bool bareStop, bool& pauseLine) {
    pauseLine = false;
    std::vector<std::pair<char, std::string>> words;
    // An M0/M1 line must be bare (owner decision 2026-11-03): an optional N word and the M0/M1,
    // nothing else (Marlin reads letters after M0 as parameters: "M0 UNLOCK UP 5" is a timeout;
    // Grbl and LinuxCNC drop spaces: "M0 3" is M03). In a pause block the M5 line is bare too
    // (T-0161): Grbl drops a line that errors as a whole, so "M5 M6" (unsupported) or
    // "M5 G0 G1" (modal-group violation) would lose the spindle stop.
    const auto bareCheck = [&words, &pauseLine, bareStop]() -> std::string {
        bool pause = false;
        bool stop = false;
        for (const auto& [l, v] : words) {
            if (l == 'M') {
                const std::string n = normalisedDecimal(v);
                pause = pause || n == "0" || n == "1";
                stop = stop || (bareStop && n == "5");
            }
        }
        if (!pause && !stop) {
            return {};
        }
        pauseLine = true;
        const std::size_t n = words.size();
        const bool bare = (n == 1) || (n == 2 && words[0].first == 'N');
        if (bare) {
            return {};
        }
        return pause ? "other words on a line with M0 or M1 (an M0/M1 line holds only [N] M0)"
                     : "other words on the M5 line of a pause block (it holds only [N] M5)";
    };
    std::size_t i = 0;
    while (true) {
        while (i < live.size() && live[i] == ' ') {
            ++i;
        }
        if (i >= live.size()) {
            return bareCheck();
        }
        const char letter = live[i];
        if (letter < 'A' || letter > 'Z') {
            return std::string("the character '") + letter + "' (only upper-case words are allowed here)";
        }
        ++i;
        const std::size_t start = i;
        while (i < live.size() && (isDigit(live[i]) || live[i] == '.' || live[i] == '+' || live[i] == '-')) {
            ++i;
        }
        const std::string_view value = live.substr(start, i - start);
        if (value.empty()) {
            return std::string("the letter ") + letter + " without a value";
        }
        if (std::string bad = wordProblem(letter, value); !bad.empty()) {
            return bad;
        }
        if (i < live.size() && live[i] != ' ') {
            return std::string("the character '") + live[i] + "' after " + letter + std::string(value);
        }
        words.emplace_back(letter, std::string(value));
    }
}

/// Longest live G-code (outside comments, spaces included, line ending excluded) of a line written
/// in a drag section (T-0161): Grbl's line buffer holds 79 characters (LINE_BUFFER_SIZE 80) and a
/// longer line is dropped whole with error:overflow. Grbl and Marlin do not buffer comments, so a
/// long tool name in a comment does not count.
constexpr std::size_t kMaxDragLiveCode = 79;

/// Why a written line (template text with its values) may not be written while a spindle-off
/// toolpath is active ("" when it may), T-0154. Anywhere in the line, comments included: only
/// printable ASCII, none of Grbl's realtime characters `~ ! ?`, and none of Marlin's
/// emergency-parser codes M108, M112, M410, M876. Parentheses must be plain comments (a comment
/// ends at the first `)`). The live G-code outside comments is at most `kMaxDragLiveCode`
/// characters and must pass `allowlistProblem` (`bareStop`: the line is in a pause block).
/// Values inside comments are already sanitised (ADR-0020), so a tool named "S12 drill" in a
/// comment is fine.
std::string renderedProblem(std::string_view text, bool semicolonComments, bool bareStop) {
    if (std::string bad = nonAsciiProblem(text); !bad.empty()) {
        return bad;
    }
    if (const std::size_t at = text.find_first_of("~!?"); at != std::string_view::npos) {
        return std::string("the Grbl realtime character '") + text[at] + "'";
    }
    if (std::string bad = parenProblem(text, semicolonComments); !bad.empty()) {
        return bad;
    }
    bool pauseLine = false;
    const std::string live = liveCode(text, semicolonComments);
    if (live.size() > kMaxDragLiveCode) {
        return std::to_string(live.size()) + " characters of G-code outside comments (Grbl's line buffer holds " +
               std::to_string(kMaxDragLiveCode) + "; a longer line is dropped)";
    }
    if (std::string bad = allowlistProblem(live, bareStop, pauseLine); !bad.empty()) {
        return bad;
    }
    if (semicolonComments && live.find('N') != std::string::npos) {
        // Hosts (OctoPrint, Printrun) prefix their own N...*checksum and Marlin skips only one N,
        // so "N57 N1 M0" would be an unknown command: no N words on ';'-style posts.
        return "an N word (line numbers are not allowed in a drag section on ';'-comment posts)";
    }
    if (pauseLine && !semicolonComments && std::count(text.begin(), text.end(), '(') > 1) {
        return "more than one comment on a line with M0 or M1 (or the M5 line of a pause block)";
    }
    return {};
}

/// The drag check on the exact bytes written for one line (text, VAR prefixes and values
/// included, plus the line ending): the ending must be LF, CR LF or CR and the rest must pass
/// `renderedProblem` (T-0154).
std::string writtenProblem(std::string_view written, std::string_view ending, bool semicolonComments,
                           bool bareStop) {
    if ((ending != "\n" && ending != "\r\n" && ending != "\r") || written.size() < ending.size() ||
        written.substr(written.size() - ending.size()) != ending) {
        return "a line ending other than LF, CR LF or CR";
    }
    return renderedProblem(written.substr(0, written.size() - ending.size()), semicolonComments, bareStop);
}

/// True for the Marlin emergency-parser codes M108, M112, M410, M876 anywhere in `text` (spaces
/// ignored, any case, leading zeros allowed). Called after `nonAsciiProblem`, so there is no tab.
bool hasEmergencyCode(std::string_view text) {
    std::string squeezed;
    for (const char c : text) {
        if (c != ' ') {
            squeezed += upperLetter(c);
        }
    }
    for (std::size_t at = squeezed.find('M'); at != std::string::npos; at = squeezed.find('M', at + 1)) {
        std::size_t j = at + 1;
        while (j < squeezed.size() && squeezed[j] == '0') {
            ++j;
        }
        std::size_t k = j;
        while (k < squeezed.size() && isDigit(squeezed[k])) {
            ++k;
        }
        const std::string_view code = std::string_view(squeezed).substr(j, k - j);
        if (code == "108" || code == "112" || code == "410" || code == "876") {
            return true;
        }
    }
    return false;
}


bool hasWord(const std::vector<GWord>& words, char letter, double value) {
    for (const GWord& w : words) {
        if (w.letter == letter && w.value == value) {
            return true;
        }
    }
    return false;
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
        semicolonComments_ = usesSemicolonComments();
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

        // A toolpath that runs with the spindle off (T-0154) must be a diamond drag tool's (the
        // snapshot records it; second guard after the record checks) and needs the post's
        // SPINDLE_OFF_PAUSE block, which must stop the spindle (M5) and pause (M0).
        for (const Toolpath* tp : active) {
            if (tp->tool.spindleOff && tp->tool.toolType != toolpath::kDiamondDragToolType) {
                return Error{0, "cannot write G-code: toolpath " + quoted(tp->name) +
                                    " is marked to run with the spindle off, but its tool " + quoted(tp->tool.name) +
                                    " is not a diamond drag tool; only diamond-drag toolpaths run with the spindle off"};
            }
        }
        const std::string offProblem = pauseBlockProblem("SPINDLE_OFF_PAUSE", true);
        if (!offProblem.empty()) {
            for (const Toolpath* tp : active) {
                if (tp->tool.spindleOff) {
                    return Error{0, "cannot write G-code: toolpath " + quoted(tp->name) +
                                        " runs with the spindle off (tool " + quoted(tp->tool.name) +
                                        " must not turn), but " + offProblem +
                                        " to stop the spindle and pause for the operator before it"};
                }
            }
        }
        // After the last spindle-off toolpath of a drag section, before a spinning toolpath can
        // start the spindle or the program ends, the operator unlocks it (owner and director
        // decisions 2026-11-03): SPINDLE_UNLOCK_PAUSE, which must stop the spindle (M5) and pause (M0).
        const std::string unlockProblem = pauseBlockProblem("SPINDLE_UNLOCK_PAUSE", true);
        if (!unlockProblem.empty()) {
            if (!active.empty() && active.back()->tool.spindleOff) {
                // The next program could start a spindle that is still locked (director 2026-11-03).
                return Error{0, "cannot write G-code: the job ends with the spindle-off toolpath " +
                                    quoted(active.back()->name) + ", but " + unlockProblem +
                                    " to pause for the operator to unlock the spindle before the program ends"};
            }
            for (std::size_t i = 1; i < active.size(); ++i) {
                if (active[i - 1]->tool.spindleOff && !active[i]->tool.spindleOff) {
                    return Error{0, "cannot write G-code: toolpath " + quoted(active[i]->name) +
                                        " starts the spindle after the spindle-off toolpath " +
                                        quoted(active[i - 1]->name) + ", but " + unlockProblem +
                                        " to pause for the operator to unlock the spindle before it"};
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

    /// Why the post's spindle-off pause block cannot serve (T-0154): missing, or (by the
    /// controller-style tokeniser) without an M0 pause, or without an M5 spindle stop when
    /// `needStop`. Empty when the block is usable.
    std::string pauseBlockProblem(std::string_view name, bool needStop) const {
        const PostBlock* b = pp_.block(name);
        if (b == nullptr) {
            return "the post has no " + std::string(name) + " block";
        }
        for (const std::string& line : b->lines) {
            if (const std::string bad = templateProblem(line, semicolonComments_); !bad.empty()) {
                return "the post's " + std::string(name) + " block line " + quoted(line) + " holds " + bad +
                       ", so it cannot serve";
            }
        }
        // Text after ';' never counts here (a comment on most controllers): conservative for a
        // block that must contain these words.
        const std::vector<GWord> words = blockWords(*b, true, semicolonComments_);
        if (needStop && !hasWord(words, 'M', 5.0)) {
            return "the post's " + std::string(name) + " block writes no M5 (spindle stop)";
        }
        if (!hasWord(words, 'M', 0.0)) {
            return "the post's " + std::string(name) + " block writes no M0 (program pause)";
        }
        if (needStop && pauseBeforeStop(words)) {
            return "the post's " + std::string(name) +
                   " block pauses (M0 or M1) before it stops the spindle (M5); write the M5 first";
        }
        return {};
    }

    /// True when the post's comment style is `;` (Marlin): some block line starts with `;` and no
    /// block line holds a `(`. Only then is text after `;` treated as a comment by the spindle-off
    /// checks (T-0154); for other posts it counts as G-code.
    bool usesSemicolonComments() const {
        bool semicolon = false;
        for (std::string_view name : PostProcessor::knownBlockNames()) {
            if (const PostBlock* b = pp_.block(name)) {
                for (const std::string& line : b->lines) {
                    if (line.find('(') != std::string::npos) {
                        return false;
                    }
                    const std::string_view t = detail::trim(line);
                    semicolon = semicolon || (!t.empty() && t.front() == ';');
                }
            }
        }
        return semicolon;
    }

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
        // T-0154: the pause blocks must write M5 and M0, judged on the lines actually written.
        const bool pauseBlock = off && (blockName == "SPINDLE_OFF_PAUSE" || blockName == "SPINDLE_UNLOCK_PAUSE");
        std::vector<GWord> pauseWords;
        for (const std::string& line : b->lines) {
            if (dropSpindleLines && line.find("[S]") != std::string::npos) {
                if (spindleOnlyLine(line, semicolonComments_)) {
                    continue;  // the spindle start of a spinning toolpath: left out
                }
                fail(0, spindleOffWhat() + ", but the line " + quoted(line) +
                            " writes [S] together with other codes; start the spindle on a line of its own");
                break;
            }
            if (off) {
                // T-0154 strict rule: the template text of every block written while a spindle-off
                // toolpath is active.
                if (const std::string bad = templateProblem(line, semicolonComments_); !bad.empty()) {
                    fail(0, spindleOffWhat() + ", but the line " + quoted(line) + " of the post holds " + bad);
                    break;
                }
            }
            bool hadTag = false;
            const std::string lineText = renderFittingLine(line, hadTag);
            if (failure_) {
                break;
            }
            if (lineText.empty() && hadTag) {
                continue;  // every variable on the line was suppressed
            }
            if (lineText.size() > kMaxLineLength) {
                // T-0162: only template text, live-code values or many text values on one line get
                // here; text values in comments were cut to fit by renderFittingLine.
                fail(0, "the line " + quoted(lineText) + " would be " + std::to_string(lineText.size()) +
                            " characters long; lines are limited to " + std::to_string(kMaxLineLength) +
                            " (LinuxCNC and grblHAL refuse longer lines)");
                break;
            }
            const std::string written = lineText + s_.lineEnding;
            if (off) {
                // T-0154: the exact bytes written for the line (values and line ending included)
                // must pass the drag-section allowlist.
                if (const std::string bad = writtenProblem(written, s_.lineEnding, semicolonComments_, pauseBlock);
                    !bad.empty()) {
                    fail(0, spindleOffWhat() + ", but the post writes " + bad + " (line " + quoted(lineText) + ")");
                    break;
                }
                if (pauseBlock) {
                    std::vector<GWord> words = gcodeWords(liveCode(lineText, true));
                    if (semicolonComments_) {
                        words = firstCommand(std::move(words));  // Marlin runs one command per line
                    }
                    pauseWords.insert(pauseWords.end(), words.begin(), words.end());
                }
            }
            out_ += written;
        }
        if (pauseBlock && !failure_ && (!hasWord(pauseWords, 'M', 5.0) || !hasWord(pauseWords, 'M', 0.0))) {
            fail(0, spindleOffWhat() + ", but the post's " + std::string(blockName) +
                        " block did not write both M5 and M0 (as the first command of a line on ';'-style posts)");
        }
        if (pauseBlock && !failure_ && pauseBeforeStop(pauseWords)) {
            // T-0162: the operator pause comes after the spindle stop, in both pause blocks.
            fail(0, spindleOffWhat() + ", but the post's " + std::string(blockName) +
                        " block wrote M0 or M1 before M5; the spindle must be stopped before the pause");
        }
        block_ = outerBlock;
    }

    /// Expands one template line (trailing spaces trimmed). `hadTag` is set when it holds a tag.
    std::string renderLine(const std::string& line, bool& hadTag) {
        std::string lineText;
        hadTag = false;
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
        return lineText;
    }

    /// True when `line` holds a text tag (`[TOOLNAME]`, `[TOOLPATH_NAME]` …).
    static bool hasTextTag(const std::string& line) {
        for (std::size_t open = line.find('['); open != std::string::npos; open = line.find('[', open + 1)) {
            const std::size_t close = line.find(']', open);
            if (close == std::string::npos) {
                break;
            }
            const TagInfo* info = findTag(std::string_view(line).substr(open + 1, close - open - 1));
            if (info != nullptr && info->kind == TagKind::Text) {
                return true;
            }
        }
        return false;
    }

    /// Expands one template line so that it fits `kMaxLineLength` when its text values can make it
    /// fit (T-0162, ADR-0020). A line longer than the limit is expanded again with every text
    /// value that lands in a comment cut to at most `cap` characters, the cut ones ending in
    /// `kEllipsis`; `cap` is the largest that fits (binary search, at least the ellipsis). The
    /// counters and modal memory are restored before each expansion, so the line is written once.
    /// When even the smallest cap does not fit, the line comes back too long and the caller
    /// refuses it.
    std::string renderFittingLine(const std::string& line, bool& hadTag) {
        if (!hasTextTag(line)) {
            return renderLine(line, hadTag);
        }
        const long long lineNumber = lineNumber_;
        const std::map<std::string, std::string> modal = modal_;
        std::string text = renderLine(line, hadTag);
        if (failure_ || text.size() <= kMaxLineLength) {
            return text;
        }
        const auto renderCapped = [&](std::size_t cap) {
            lineNumber_ = lineNumber;
            modal_ = modal;
            textCap_ = cap;
            std::string capped = renderLine(line, hadTag);
            textCap_.reset();
            return capped;
        };
        std::size_t lo = kEllipsis.size();  // the smallest cap; may still not fit
        std::size_t hi = text.size();       // no value is longer than the whole line
        while (lo < hi) {
            const std::size_t mid = lo + (hi - lo + 1) / 2;
            if (renderCapped(mid).size() <= kMaxLineLength) {
                lo = mid;
            } else {
                hi = mid - 1;
            }
        }
        return renderCapped(lo);
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
            const detail::CommentPosition pos = detail::commentPositionAt(std::string(before) + def.prefix);
            body = detail::sanitiseText(body, pos);
            if (textCap_ && pos.context != detail::CommentContext::None && body.size() > *textCap_) {
                // T-0162: cut to fit the line limit, visibly. A prefix of the sanitised value stays
                // safe: nothing in it can close the comment or form an active one.
                body = body.substr(0, *textCap_ - kEllipsis.size()) + std::string(kEllipsis);
            }
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
    std::optional<std::size_t> textCap_;  // longest text value in a comment, while a line is cut to fit (T-0162)
    std::string_view block_{"the program"};  // block being expanded, for error messages
    std::size_t moveIndex_{0};         // 1-based move of the current toolpath; 0 = none
    bool inFooter_{false};
    bool radiusOnlyArcs_{false};
    bool semicolonComments_{false};  // the post's comment style is ';' (T-0154)
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
