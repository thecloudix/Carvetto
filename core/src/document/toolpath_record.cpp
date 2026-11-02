#include "carvetto/document/toolpath_record.hpp"

#include <memory>
#include <utility>
#include <variant>
#include <vector>

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

SharedMoves shareMoves(std::vector<toolpath::Move> moves) {
    for (toolpath::Move& m : moves) {
        if (!toolpath::isArc(m.kind)) {
            m.centre = {};
        }
        if (m.kind != toolpath::MoveKind::Dwell) {
            m.dwellSeconds = 0.0;
        }
        if (!toolpath::isCutting(m.kind)) {
            m.feedOverride.reset();
        }
    }
    return std::make_shared<const std::vector<toolpath::Move>>(std::move(moves));
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
