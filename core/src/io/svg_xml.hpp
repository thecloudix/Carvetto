#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "carvetto/io/svg.hpp"

/// Internal: the bounded XML subset parser used by the SVG importer (ADR-0026).
namespace carvetto::io::detail {

inline constexpr std::size_t kNoNode = std::numeric_limits<std::size_t>::max();

struct XmlAttr {
    std::string name;
    std::string value;
};

/// An element. Children are linked through `firstChild` / `nextSibling` (document order).
struct XmlNode {
    std::string name;
    std::vector<XmlAttr> attrs;
    std::size_t parent{kNoNode};
    std::size_t firstChild{kNoNode};
    std::size_t lastChild{kNoNode};
    std::size_t nextSibling{kNoNode};
    std::size_t line{0};

    /// The attribute value, or nullptr.
    const std::string* attr(std::string_view attrName) const {
        for (const XmlAttr& a : attrs) {
            if (a.name == attrName) {
                return &a.value;
            }
        }
        return nullptr;
    }
};

/// A parsed document; `nodes[0]` is the root element.
struct XmlDoc {
    std::vector<XmlNode> nodes;
};

struct XmlLimits {
    std::size_t maxInputBytes{32U * 1024U * 1024U};
    std::size_t maxDepth{128};
    std::size_t maxElements{1'000'000};
    std::size_t maxAttributes{256};
    std::size_t maxAttributeLength{64U * 1024U};
    /// Applies to the attributes named `d` and `points`.
    std::size_t maxLongAttributeLength{16U * 1024U * 1024U};
    std::size_t maxNameLength{256};
};

/// Parses a document. Supported: elements, attributes, comments, CDATA, processing
/// instructions and the five predefined plus numeric character references in attribute values.
/// Refused: DOCTYPE and any other `<!` declaration (so no entity can be defined or expanded),
/// undefined entity references in attribute values, duplicate attributes, mismatched tags,
/// several root elements. Text content is not stored. Nothing is ever fetched or read from disk.
SvgResult<XmlDoc> parseXml(std::string_view text, const XmlLimits& limits);

}  // namespace carvetto::io::detail
