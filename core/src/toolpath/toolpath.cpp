#include "carvetto/toolpath/toolpath.hpp"

#include <algorithm>

namespace carvetto::toolpath {

void Toolpath::append(const Toolpath& other) {
    moves.insert(moves.end(), other.moves.begin(), other.moves.end());
}

std::optional<Vec3> Toolpath::endPoint() const {
    if (moves.empty()) {
        return std::nullopt;
    }
    return moves.back().to;
}

Vec3 Toolpath::startOf(std::size_t i, std::optional<Vec3> from) const {
    if (i == 0) {
        return from.value_or(moves.empty() ? Vec3{} : moves.front().to);
    }
    return moves[i - 1].to;
}

LengthByKind Toolpath::lengthByKind(std::optional<Vec3> from) const {
    LengthByKind out;
    for (std::size_t i = 0; i < moves.size(); ++i) {
        const Move& m = moves[i];
        const double len = moveLength(m, startOf(i, from));
        switch (m.kind) {
            case MoveKind::Rapid:
                out.rapid += len;
                break;
            case MoveKind::Feed:
            case MoveKind::ArcCW:
            case MoveKind::ArcCCW:
                out.feed += len;
                break;
            case MoveKind::Plunge:
                out.plunge += len;
                break;
            case MoveKind::Retract:
                out.retract += len;
                break;
            case MoveKind::Dwell:
                break;
        }
    }
    return out;
}

Box2 Toolpath::bounds(std::optional<Vec3> from) const {
    Box2 b;
    for (std::size_t i = 0; i < moves.size(); ++i) {
        b.expand(moveBounds(moves[i], startOf(i, from)));
    }
    return b;
}

std::optional<double> Toolpath::minZ() const {
    if (moves.empty()) {
        return std::nullopt;
    }
    return std::ranges::min_element(moves, {}, [](const Move& m) { return m.to.z; })->to.z;
}

std::optional<double> Toolpath::maxZ() const {
    if (moves.empty()) {
        return std::nullopt;
    }
    return std::ranges::max_element(moves, {}, [](const Move& m) { return m.to.z; })->to.z;
}

std::size_t Toolpath::count(MoveKind kind) const {
    return static_cast<std::size_t>(
        std::ranges::count_if(moves, [kind](const Move& m) { return m.kind == kind; }));
}

}  // namespace carvetto::toolpath
