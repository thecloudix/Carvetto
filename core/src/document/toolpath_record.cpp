#include "carvetto/document/toolpath_record.hpp"

#include <variant>

namespace carvetto::document {

double ToolpathRecord::safeZ() const {
    return std::visit([](const auto& p) { return p.safeZ; }, params);
}

toolpath::Toolpath ToolpathRecord::toolpath() const {
    toolpath::Toolpath tp;
    tp.name = name;
    tp.tool = tool;
    if (moves) {
        tp.moves = *moves;
    }
    return tp;
}

bool ToolpathRecord::operator==(const ToolpathRecord& other) const {
    if (!(id == other.id && name == other.name && params == other.params && selection == other.selection &&
          tool == other.tool && toolSource == other.toolSource)) {
        return false;
    }
    if (moves == other.moves) {
        return true;  // same list, or both null
    }
    return moves && other.moves && *moves == *other.moves;
}

}  // namespace carvetto::document
