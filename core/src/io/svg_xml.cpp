#include "svg_xml.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace carvetto::io::detail {

namespace {

bool isSpace(char c) {
    return c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

bool isNameStart(char c) {
    const auto u = static_cast<unsigned char>(c);
    return (u >= 'a' && u <= 'z') || (u >= 'A' && u <= 'Z') || u == '_' || u == ':' || u >= 0x80;
}

bool isNameChar(char c) {
    const auto u = static_cast<unsigned char>(c);
    return isNameStart(c) || (u >= '0' && u <= '9') || u == '-' || u == '.';
}

bool validCodePoint(std::uint32_t cp) {
    return cp == 0x9 || cp == 0xA || cp == 0xD || (cp >= 0x20 && cp <= 0xD7FF) ||
           (cp >= 0xE000 && cp <= 0xFFFD) || (cp >= 0x10000 && cp <= 0x10FFFF);
}

void appendUtf8(std::string& out, std::uint32_t cp) {
    if (cp < 0x80) {
        out += static_cast<char>(cp);
    } else if (cp < 0x800) {
        out += static_cast<char>(0xC0 | (cp >> 6));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    } else if (cp < 0x10000) {
        out += static_cast<char>(0xE0 | (cp >> 12));
        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    } else {
        out += static_cast<char>(0xF0 | (cp >> 18));
        out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    }
}

class Parser {
   public:
    Parser(std::string_view text, const XmlLimits& limits) : s_(text), lim_(limits) {}

    SvgResult<XmlDoc> run() {
        if (s_.size() > lim_.maxInputBytes) {
            return fail(SvgError::Code::ResourceLimit, "input exceeds the size limit", 0);
        }
        if (s_.size() >= 3 && s_.substr(0, 3) == "\xEF\xBB\xBF") {
            pos_ = 3;
        }
        std::vector<std::size_t> stack;
        bool haveRoot = false;
        while (pos_ < s_.size()) {
            if (s_[pos_] != '<') {
                const std::size_t next = std::min(s_.find('<', pos_), s_.size());
                if (stack.empty()) {
                    for (std::size_t i = pos_; i < next; ++i) {
                        if (!isSpace(s_[i])) {
                            return failAt(SvgError::Code::InvalidFormat, "text outside the root element", i);
                        }
                    }
                }
                pos_ = next;
                continue;
            }
            const std::string_view rest = s_.substr(pos_);
            if (rest.starts_with("<!--")) {
                const std::size_t end = s_.find("-->", pos_ + 4);
                if (end == std::string_view::npos) {
                    return failAt(SvgError::Code::InvalidFormat, "unterminated comment", pos_);
                }
                pos_ = end + 3;
            } else if (rest.starts_with("<![CDATA[")) {
                if (stack.empty()) {
                    return failAt(SvgError::Code::InvalidFormat, "CDATA outside the root element", pos_);
                }
                const std::size_t end = s_.find("]]>", pos_ + 9);
                if (end == std::string_view::npos) {
                    return failAt(SvgError::Code::InvalidFormat, "unterminated CDATA section", pos_);
                }
                pos_ = end + 3;
            } else if (rest.starts_with("<?")) {
                const std::size_t end = s_.find("?>", pos_ + 2);
                if (end == std::string_view::npos) {
                    return failAt(SvgError::Code::InvalidFormat, "unterminated processing instruction", pos_);
                }
                pos_ = end + 2;
            } else if (rest.starts_with("<!")) {
                return failAt(SvgError::Code::Unsupported,
                              "DOCTYPE and entity declarations are not supported", pos_);
            } else if (rest.starts_with("</")) {
                pos_ += 2;
                std::string_view name;
                if (!readName(name)) {
                    return failAt(SvgError::Code::InvalidFormat, "malformed end tag", pos_);
                }
                skipSpace();
                if (pos_ >= s_.size() || s_[pos_] != '>') {
                    return failAt(SvgError::Code::InvalidFormat, "malformed end tag", pos_);
                }
                ++pos_;
                if (stack.empty() || doc_.nodes[stack.back()].name != name) {
                    return failAt(SvgError::Code::InvalidFormat, "mismatched end tag </" + std::string(name) + ">",
                                  pos_);
                }
                stack.pop_back();
            } else {
                if (stack.empty() && haveRoot) {
                    return failAt(SvgError::Code::InvalidFormat, "more than one root element", pos_);
                }
                if (stack.size() >= lim_.maxDepth) {
                    return failAt(SvgError::Code::ResourceLimit, "element nesting is too deep", pos_);
                }
                if (doc_.nodes.size() >= lim_.maxElements) {
                    return failAt(SvgError::Code::ResourceLimit, "too many elements", pos_);
                }
                const std::size_t tagPos = pos_;
                ++pos_;
                XmlNode node;
                std::string_view name;
                if (!readName(name)) {
                    return failAt(SvgError::Code::InvalidFormat, "malformed start tag", pos_);
                }
                node.name = std::string(name);
                node.line = lineAt(tagPos);
                bool selfClosing = false;
                if (auto err = readAttributes(node, selfClosing)) {
                    return *err;
                }
                const std::size_t idx = doc_.nodes.size();
                if (!stack.empty()) {
                    node.parent = stack.back();
                    XmlNode& p = doc_.nodes[stack.back()];
                    if (p.lastChild == kNoNode) {
                        p.firstChild = idx;
                    } else {
                        doc_.nodes[p.lastChild].nextSibling = idx;
                    }
                    p.lastChild = idx;
                }
                doc_.nodes.push_back(std::move(node));
                haveRoot = true;
                if (!selfClosing) {
                    stack.push_back(idx);
                }
            }
        }
        if (!stack.empty()) {
            return failAt(SvgError::Code::InvalidFormat, "unexpected end of input inside <" + doc_.nodes[stack.back()].name + ">",
                          s_.size());
        }
        if (!haveRoot) {
            return failAt(SvgError::Code::InvalidFormat, "no root element", s_.size());
        }
        return std::move(doc_);
    }

   private:
    SvgError makeError(SvgError::Code code, std::string msg, std::size_t line) const {
        SvgError e;
        e.code = code;
        e.message = std::move(msg);
        e.line = line;
        return e;
    }
    SvgError fail(SvgError::Code code, std::string msg, std::size_t line) const {
        return makeError(code, std::move(msg), line);
    }
    SvgError failAt(SvgError::Code code, std::string msg, std::size_t offset) {
        return makeError(code, std::move(msg), lineAt(offset));
    }

    /// 1-based line of a byte offset (offsets are queried in non-decreasing order, or fall back).
    std::size_t lineAt(std::size_t offset) {
        offset = std::min(offset, s_.size());
        if (offset < lineOffset_) {
            lineOffset_ = 0;
            line_ = 1;
        }
        for (; lineOffset_ < offset; ++lineOffset_) {
            if (s_[lineOffset_] == '\n') {
                ++line_;
            }
        }
        return line_;
    }

    void skipSpace() {
        while (pos_ < s_.size() && isSpace(s_[pos_])) {
            ++pos_;
        }
    }

    bool readName(std::string_view& out) {
        const std::size_t start = pos_;
        if (pos_ >= s_.size() || !isNameStart(s_[pos_])) {
            return false;
        }
        while (pos_ < s_.size() && isNameChar(s_[pos_])) {
            ++pos_;
            if (pos_ - start > lim_.maxNameLength) {
                return false;
            }
        }
        out = s_.substr(start, pos_ - start);
        return true;
    }

    /// Reads attributes up to and including '>' or '/>'.
    std::optional<SvgError> readAttributes(XmlNode& node, bool& selfClosing) {
        std::set<std::string_view> seen;
        for (;;) {
            const std::size_t before = pos_;
            skipSpace();
            if (pos_ >= s_.size()) {
                return makeError(SvgError::Code::InvalidFormat, "unterminated start tag", lineAt(s_.size()));
            }
            if (s_[pos_] == '>') {
                ++pos_;
                selfClosing = false;
                return std::nullopt;
            }
            if (s_[pos_] == '/') {
                if (pos_ + 1 < s_.size() && s_[pos_ + 1] == '>') {
                    pos_ += 2;
                    selfClosing = true;
                    return std::nullopt;
                }
                return makeError(SvgError::Code::InvalidFormat, "malformed start tag", lineAt(pos_));
            }
            if (pos_ == before) {
                return makeError(SvgError::Code::InvalidFormat, "missing whitespace between attributes",
                                 lineAt(pos_));
            }
            std::string_view name;
            if (!readName(name)) {
                return makeError(SvgError::Code::InvalidFormat, "malformed attribute name", lineAt(pos_));
            }
            skipSpace();
            if (pos_ >= s_.size() || s_[pos_] != '=') {
                return makeError(SvgError::Code::InvalidFormat, "attribute without a value", lineAt(pos_));
            }
            ++pos_;
            skipSpace();
            if (pos_ >= s_.size() || (s_[pos_] != '"' && s_[pos_] != '\'')) {
                return makeError(SvgError::Code::InvalidFormat, "attribute value must be quoted", lineAt(pos_));
            }
            const char quote = s_[pos_];
            ++pos_;
            const std::size_t end = s_.find(quote, pos_);
            if (end == std::string_view::npos) {
                return makeError(SvgError::Code::InvalidFormat, "unterminated attribute value", lineAt(pos_));
            }
            const std::string_view raw = s_.substr(pos_, end - pos_);
            const bool longOk = name == "d" || name == "points";
            if (raw.size() > (longOk ? lim_.maxLongAttributeLength : lim_.maxAttributeLength)) {
                return makeError(SvgError::Code::ResourceLimit,
                                 "attribute '" + std::string(name) + "' is longer than the limit", lineAt(pos_));
            }
            if (node.attrs.size() >= lim_.maxAttributes) {
                return makeError(SvgError::Code::ResourceLimit, "too many attributes on <" + node.name + ">",
                                 lineAt(pos_));
            }
            if (!seen.insert(name).second) {
                return makeError(SvgError::Code::InvalidFormat, "duplicate attribute '" + std::string(name) + "'",
                                 lineAt(pos_));
            }
            XmlAttr attr;
            attr.name = std::string(name);
            std::string why;
            if (!decodeValue(raw, attr.value, why)) {
                return makeError(SvgError::Code::InvalidFormat, why, lineAt(pos_));
            }
            node.attrs.push_back(std::move(attr));
            pos_ = end + 1;
        }
    }

    static bool decodeValue(std::string_view raw, std::string& out, std::string& why) {
        out.reserve(raw.size());
        for (std::size_t i = 0; i < raw.size(); ++i) {
            const char c = raw[i];
            if (c == '<') {
                why = "'<' is not allowed in an attribute value";
                return false;
            }
            if (c != '&') {
                out += c;
                continue;
            }
            const std::size_t semi = raw.find(';', i);
            if (semi == std::string_view::npos || semi - i > 12) {
                why = "malformed entity reference";
                return false;
            }
            const std::string_view ent = raw.substr(i + 1, semi - i - 1);
            if (ent == "lt") {
                out += '<';
            } else if (ent == "gt") {
                out += '>';
            } else if (ent == "amp") {
                out += '&';
            } else if (ent == "quot") {
                out += '"';
            } else if (ent == "apos") {
                out += '\'';
            } else if (ent.size() >= 2 && ent[0] == '#') {
                const bool hex = ent[1] == 'x';
                const std::size_t first = hex ? 2 : 1;
                if (first >= ent.size()) {
                    why = "malformed character reference";
                    return false;
                }
                std::uint32_t cp = 0;
                for (std::size_t k = first; k < ent.size(); ++k) {
                    const char d = ent[k];
                    std::uint32_t digit = 0;
                    if (d >= '0' && d <= '9') {
                        digit = static_cast<std::uint32_t>(d - '0');
                    } else if (hex && d >= 'a' && d <= 'f') {
                        digit = static_cast<std::uint32_t>(d - 'a' + 10);
                    } else if (hex && d >= 'A' && d <= 'F') {
                        digit = static_cast<std::uint32_t>(d - 'A' + 10);
                    } else {
                        why = "malformed character reference";
                        return false;
                    }
                    cp = cp * (hex ? 16U : 10U) + digit;
                    if (cp > 0x10FFFF) {
                        why = "character reference out of range";
                        return false;
                    }
                }
                if (!validCodePoint(cp)) {
                    why = "character reference is not a valid XML character";
                    return false;
                }
                appendUtf8(out, cp);
            } else {
                why = "entity '&" + std::string(ent) + ";' is not supported";
                return false;
            }
            i = semi;
        }
        return true;
    }

    std::string_view s_;
    XmlLimits lim_;
    std::size_t pos_{0};
    std::size_t lineOffset_{0};
    std::size_t line_{1};
    XmlDoc doc_;
};

}  // namespace

SvgResult<XmlDoc> parseXml(std::string_view text, const XmlLimits& limits) {
    return Parser(text, limits).run();
}

}  // namespace carvetto::io::detail
