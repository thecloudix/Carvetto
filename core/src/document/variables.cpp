#include "carvetto/document/variables.hpp"

#include <array>
#include <charconv>
#include <cstddef>
#include <string>
#include <string_view>
#include <system_error>

namespace carvetto::document {

namespace {

bool isAsciiLetter(char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
}

bool isAsciiDigit(char c) {
    return c >= '0' && c <= '9';
}

std::string_view trim(std::string_view s) {
    const auto notSpace = [](char c) { return c != ' ' && c != '\t' && c != '\r'; };
    while (!s.empty() && !notSpace(s.front())) {
        s.remove_prefix(1);
    }
    while (!s.empty() && !notSpace(s.back())) {
        s.remove_suffix(1);
    }
    return s;
}

// Shortest round-trip representation of a double.
std::string formatDouble(double v) {
    std::array<char, 64> buf{};
    const auto res = std::to_chars(buf.data(), buf.data() + buf.size(), v);
    return std::string(buf.data(), res.ptr);
}

bool parseDouble(std::string_view s, double& out) {
    const auto res = std::from_chars(s.data(), s.data() + s.size(), out);
    return res.ec == std::errc{} && res.ptr == s.data() + s.size();
}

}  // namespace

bool DocumentVariables::isValidName(std::string_view name) {
    if (name.empty() || !isAsciiLetter(name.front())) {
        return false;
    }
    for (char c : name.substr(1)) {
        if (!isAsciiLetter(c) && !isAsciiDigit(c) && c != '_') {
            return false;
        }
    }
    return true;
}

bool DocumentVariables::set(const std::string& name, double value) {
    if (!isValidName(name)) {
        return false;
    }
    values_[name] = value;
    return true;
}

std::optional<double> DocumentVariables::get(std::string_view name) const {
    const auto it = values_.find(name);
    if (it == values_.end()) {
        return std::nullopt;
    }
    return it->second;
}

bool DocumentVariables::contains(std::string_view name) const {
    return values_.find(name) != values_.end();
}

bool DocumentVariables::remove(std::string_view name) {
    const auto it = values_.find(name);
    if (it == values_.end() || referenceCount(name) > 0) {
        return false;
    }
    values_.erase(it);
    return true;
}

void DocumentVariables::forceRemove(std::string_view name) {
    const auto it = values_.find(name);
    if (it != values_.end()) {
        values_.erase(it);
    }
}

void DocumentVariables::addReference(std::string_view name) {
    if (!contains(name)) {
        return;
    }
    const auto it = references_.find(name);
    if (it == references_.end()) {
        references_.emplace(std::string(name), 1);
    } else {
        ++it->second;
    }
}

void DocumentVariables::removeReference(std::string_view name) {
    const auto it = references_.find(name);
    if (it == references_.end()) {
        return;
    }
    if (--it->second == 0) {
        references_.erase(it);
    }
}

std::size_t DocumentVariables::referenceCount(std::string_view name) const {
    const auto it = references_.find(name);
    return it == references_.end() ? 0 : it->second;
}

std::vector<std::string> DocumentVariables::names() const {
    std::vector<std::string> out;
    out.reserve(values_.size());
    for (const auto& [name, value] : values_) {
        out.push_back(name);
    }
    return out;
}

std::string DocumentVariables::toText() const {
    std::string out;
    for (const auto& [name, value] : values_) {
        out += name;
        out += " = ";
        out += formatDouble(value);
        out += '\n';
    }
    return out;
}

DocumentVariables::ImportResult DocumentVariables::importText(std::string_view text) {
    ImportResult result;
    std::size_t lineNo = 0;
    while (!text.empty()) {
        ++lineNo;
        const std::size_t nl = text.find('\n');
        std::string_view line = text.substr(0, nl);
        text = (nl == std::string_view::npos) ? std::string_view{} : text.substr(nl + 1);
        line = trim(line);
        if (line.empty() || line.front() == '#') {
            continue;
        }
        const std::size_t eq = line.find('=');
        if (eq == std::string_view::npos) {
            result.rejectedLines.push_back(lineNo);
            continue;
        }
        const std::string_view name = trim(line.substr(0, eq));
        const std::string_view valueText = trim(line.substr(eq + 1));
        double value = 0.0;
        if (!isValidName(name) || !parseDouble(valueText, value)) {
            result.rejectedLines.push_back(lineNo);
            continue;
        }
        values_[std::string(name)] = value;
        ++result.imported;
    }
    return result;
}

}  // namespace carvetto::document
