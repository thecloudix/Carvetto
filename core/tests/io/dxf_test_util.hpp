#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "carvetto/io/dxf.hpp"
#include "dxf_pairs.hpp"

namespace dxftest {

using Bytes = std::vector<std::uint8_t>;

inline Bytes readFixture(const std::string& name) {
    std::ifstream in(std::string(CARVETTO_DXF_FIXTURE_DIR) + "/" + name, std::ios::binary);
    return Bytes(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

inline Bytes toBytes(std::string_view s) {
    return Bytes(s.begin(), s.end());
}

inline std::string toString(const Bytes& b) {
    return std::string(b.begin(), b.end());
}

inline carvetto::io::DxfResult<carvetto::io::DxfDrawing> importText(
    std::string_view text, const carvetto::io::DxfImportOptions& o = {}) {
    const Bytes b = toBytes(text);
    return carvetto::io::importDxf(b, o);
}

/// Splits ASCII DXF text into (code, value) pairs (test fixtures only; no validation).
inline std::vector<std::pair<int, std::string>> asciiPairs(const std::string& text) {
    std::vector<std::pair<int, std::string>> out;
    std::istringstream in(text);
    std::string code;
    std::string value;
    while (std::getline(in, code) && std::getline(in, value)) {
        if (!value.empty() && value.back() == '\r') {
            value.pop_back();
        }
        out.emplace_back(std::stoi(code), value);
    }
    return out;
}

/// Re-encodes ASCII DXF pairs as binary DXF: R13+ layout (2-byte codes) or R12 layout
/// (1-byte codes, 255 escape). Comments (999) are dropped as binary files do not carry them.
inline Bytes toBinary(const std::string& asciiText, bool wideCodes) {
    using carvetto::io::detail::ValueType;
    Bytes out(carvetto::io::detail::kBinarySentinel.begin(), carvetto::io::detail::kBinarySentinel.end());
    auto le = [&out](std::uint64_t v, int n) {
        for (int i = 0; i < n; ++i) {
            out.push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xFFu));
        }
    };
    for (const auto& [code, value] : asciiPairs(asciiText)) {
        if (code == 999) {
            continue;
        }
        if (wideCodes) {
            le(static_cast<std::uint16_t>(code), 2);
        } else if (code < 255) {
            out.push_back(static_cast<std::uint8_t>(code));
        } else {
            out.push_back(255);
            le(static_cast<std::uint16_t>(code), 2);
        }
        const ValueType type = carvetto::io::detail::valueTypeFor(code).value_or(ValueType::String);
        switch (type) {
            case ValueType::Double: {
                const double d = std::stod(value);
                std::uint64_t bits = 0;
                std::memcpy(&bits, &d, sizeof bits);
                le(bits, 8);
                break;
            }
            case ValueType::Int16:
                le(static_cast<std::uint64_t>(std::stoll(value)), 2);
                break;
            case ValueType::Int32:
                le(static_cast<std::uint64_t>(std::stoll(value)), 4);
                break;
            case ValueType::Int64:
                le(static_cast<std::uint64_t>(std::stoll(value)), 8);
                break;
            case ValueType::Bool:
                le(static_cast<std::uint64_t>(std::stoll(value)), 1);
                break;
            case ValueType::Binary:
                out.push_back(static_cast<std::uint8_t>(value.size() / 2));
                for (std::size_t i = 0; i + 1 < value.size(); i += 2) {
                    out.push_back(static_cast<std::uint8_t>(std::stoi(value.substr(i, 2), nullptr, 16)));
                }
                break;
            case ValueType::String:
                out.insert(out.end(), value.begin(), value.end());
                out.push_back(0);
                break;
        }
    }
    return out;
}

/// Index of the layer called `name`, if any.
inline std::optional<std::size_t> layerIndex(const carvetto::io::DxfDrawing& d, std::string_view name) {
    for (std::size_t i = 0; i < d.layers.size(); ++i) {
        if (d.layers[i].name == name) {
            return i;
        }
    }
    return std::nullopt;
}

inline bool hasWarning(const carvetto::io::DxfDrawing& d, std::string_view fragment, std::size_t count = 0) {
    for (const auto& w : d.warnings) {
        if (w.message.find(fragment) != std::string::npos && (count == 0 || w.count == count)) {
            return true;
        }
    }
    return false;
}

}  // namespace dxftest
