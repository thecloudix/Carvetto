#include "carvetto/toolpath/cutter_comp.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>

#include "carvetto/geometry/vec2.hpp"

namespace carvetto::toolpath {

namespace {

using geometry::Vec2;
using geometry::Vec3;

constexpr double kMeet = 1e-6;       // moved moves that end this close are taken to meet
constexpr double kPlane = 1e-9;      // a move shorter than this in the plane has no direction
constexpr double kNoRadius = 1e-9;   // a moved arc with a radius up to this is a point
constexpr double kFarRadii = 10.0;   // an intersection farther than this many radii from the corner: refused

/// One move of a run that travels in the plane, with its moved (programmed) form.
struct Prim {
    std::size_t move{0};
    bool arc{false};
    bool cw{false};
    Vec2 centre;
    double radius{0.0};   // tool-centre path
    Vec2 s, t;            // tool-centre path ends
    Vec2 dir;             // line: unit direction
    double movedRadius{0.0};
    bool point{false};    // moved arc with no radius
};

std::string at(std::size_t moveIndex, const std::string& what) {
    return "move " + std::to_string(moveIndex + 1) + ": cutter compensation: " + what;
}

bool linearKind(MoveKind k) {
    return k == MoveKind::Feed || k == MoveKind::Plunge || k == MoveKind::Retract;
}

/// `p` of the tool-centre path moved back to the programmed path.
Vec2 moved(const Prim& pr, Vec2 p, double signedRadius) {
    if (!pr.arc) {
        return p - pr.dir.perp() * signedRadius;
    }
    if (pr.point) {
        return pr.centre;
    }
    const Vec2 d = p - pr.centre;
    const double len = d.length();
    return pr.centre + d * (pr.movedRadius / len);
}

enum class Hit { None, Found };

Hit lineLine(Vec2 p1, Vec2 u1, Vec2 p2, Vec2 u2, Vec2& out) {
    const double d = u1.cross(u2);
    if (std::fabs(d) < 1e-9) {
        return Hit::None;
    }
    const double tt = (p2 - p1).cross(u2) / d;
    out = p1 + u1 * tt;
    return Hit::Found;
}

Hit lineCircle(Vec2 p, Vec2 u, Vec2 c, double r, Vec2 ref, Vec2& out) {
    const Vec2 f = p - c;
    const double b = f.dot(u);
    const double disc = b * b - (f.dot(f) - r * r);
    if (disc < -1e-9) {
        return Hit::None;
    }
    const double root = std::sqrt(std::max(disc, 0.0));
    const Vec2 a = p + u * (-b - root);
    const Vec2 e = p + u * (-b + root);
    out = geometry::distance(a, ref) <= geometry::distance(e, ref) ? a : e;
    return Hit::Found;
}

Hit circleCircle(Vec2 c1, double r1, Vec2 c2, double r2, Vec2 ref, Vec2& out) {
    const Vec2 d = c2 - c1;
    const double dist = d.length();
    if (dist < 1e-12) {
        return Hit::None;
    }
    const double a = (r1 * r1 - r2 * r2 + dist * dist) / (2.0 * dist);
    const double h2 = r1 * r1 - a * a;
    if (h2 < -1e-9) {
        return Hit::None;
    }
    const double h = std::sqrt(std::max(h2, 0.0));
    const Vec2 base = c1 + d * (a / dist);
    const Vec2 n = d.perp() * (h / dist);
    const Vec2 p = base + n;
    const Vec2 q = base - n;
    out = geometry::distance(p, ref) <= geometry::distance(q, ref) ? p : q;
    return Hit::Found;
}

/// Where the moved forms of `a` and `b` meet (nearest `ref`).
Hit meet(const Prim& a, const Prim& b, double signedRadius, Vec2 ref, Vec2& out) {
    if (a.point || b.point) {
        return Hit::None;
    }
    if (!a.arc && !b.arc) {
        return lineLine(moved(a, a.t, signedRadius), a.dir, moved(b, b.s, signedRadius), b.dir, out);
    }
    if (!a.arc) {
        return lineCircle(moved(a, a.t, signedRadius), a.dir, b.centre, b.movedRadius, ref, out);
    }
    if (!b.arc) {
        return lineCircle(moved(b, b.s, signedRadius), b.dir, a.centre, a.movedRadius, ref, out);
    }
    return circleCircle(a.centre, a.movedRadius, b.centre, b.movedRadius, ref, out);
}

/// The programmed path of the run `[a, b)` (all moves with the same side), written into `out`.
std::string deriveRun(const std::vector<Move>& moves, std::size_t a, std::size_t b, double radius, Vec3 from,
                      std::vector<ProgramMove>& out) {
    const double sigma = moves[a].comp == CutterComp::Left ? 1.0 : -1.0;
    const double signedRadius = sigma * radius;
    const Vec3 start = a == 0 ? from : moves[a - 1].to;
    std::vector<Prim> prims;
    std::vector<bool> primOf(b - a, false);
    Vec3 pos = start;
    for (std::size_t k = a; k < b; ++k) {
        const Move& m = moves[k];
        if (m.kind == MoveKind::Rapid || m.kind == MoveKind::Dwell) {
            return at(k, "a rapid or dwell inside a compensated run");
        }
        Prim p;
        p.move = k;
        p.s = pos.xy();
        p.t = m.to.xy();
        if (isArc(m.kind)) {
            p.arc = true;
            p.cw = m.kind == MoveKind::ArcCW;
            p.centre = m.centre;
            p.radius = geometry::distance(p.s, p.centre);
            if (!(p.radius > kPlane)) {
                return at(k, "an arc without a radius");
            }
            // CCW: the left of the travel is towards the centre; CW: away from it.
            const bool leftIsInward = !p.cw;
            const bool inward = (sigma > 0.0) == leftIsInward;
            p.movedRadius = inward ? p.radius + radius : p.radius - radius;
            if (p.movedRadius < -kNoRadius) {
                return at(k, "an arc of radius " + std::to_string(p.radius) +
                                 " mm would need a radius below zero: it is smaller than the tool radius (" +
                                 std::to_string(radius) + " mm)");
            }
            p.point = p.movedRadius <= kNoRadius;
            if (p.point) {
                p.movedRadius = 0.0;
            }
            primOf[k - a] = true;
            prims.push_back(p);
        } else {
            const Vec2 d = p.t - p.s;
            const double len = d.length();
            if (len > kPlane) {
                p.dir = d / len;
                primOf[k - a] = true;
                prims.push_back(p);
            }
        }
        pos = m.to;
    }
    if (prims.empty() || prims.front().move != a || prims.front().arc) {
        return at(a, "the compensation must be engaged by a straight move in the plane");
    }
    if (b >= moves.size()) {
        return at(b - 1, "the compensation is never released");
    }
    const Move& rel = moves[b];
    if (rel.comp != CutterComp::Off) {
        return at(b, "the side changes without releasing the compensation first");
    }
    if (!linearKind(rel.kind) || geometry::distance(moves[b - 1].to.xy(), rel.to.xy()) <= kPlane) {
        return at(b, "the compensation must be released by a straight move in the plane");
    }

    // End point of each primitive in the programmed path.
    std::vector<Vec2> end(prims.size());
    for (std::size_t i = 0; i < prims.size(); ++i) {
        const Prim& p = prims[i];
        const Vec2 own = moved(p, p.t, signedRadius);
        if (i + 1 == prims.size()) {
            end[i] = own;  // release: the compensation vector is perpendicular to this move
            continue;
        }
        const Prim& q = prims[i + 1];
        const Vec2 next = moved(q, q.s, signedRadius);
        if (i == 0) {
            end[i] = next;  // engage: perpendicular to the move after the engaging one
            continue;
        }
        if (geometry::distance(own, next) <= kMeet) {
            end[i] = own;
            continue;
        }
        const Vec2 ref = (own + next) * 0.5;
        Vec2 corner;
        if (meet(p, q, signedRadius, ref, corner) == Hit::None) {
            return at(q.move, "an inside corner whose two moves do not meet");
        }
        if (geometry::distance(corner, ref) > kFarRadii * radius + kMeet) {
            return at(q.move, "an inside corner too sharp to compensate (the moves meet more than ten tool radii away)");
        }
        end[i] = corner;
    }

    // Write the programmed moves.
    std::size_t next = 0;
    Vec2 cur = start.xy();
    for (std::size_t k = a; k < b; ++k) {
        ProgramMove& pm = out[k];
        pm.move = moves[k];
        if (primOf[k - a]) {
            const Prim& p = prims[next];
            if (p.point) {
                pm.omit = true;
            } else {
                cur = end[next];
            }
            ++next;
        }
        pm.move.to = Vec3{cur, moves[k].to.z};
    }
    return {};
}

}  // namespace

bool usesCutterComp(const std::vector<Move>& moves) {
    return std::any_of(moves.begin(), moves.end(), [](const Move& m) { return m.comp != CutterComp::Off; });
}

CutterCompProgram cutterCompProgram(const std::vector<Move>& moves, double radius, Vec3 from) {
    CutterCompProgram result;
    if (!usesCutterComp(moves)) {
        result.moves.reserve(moves.size());
        for (const Move& m : moves) {
            result.moves.push_back({false, m});
        }
        return result;
    }
    if (!std::isfinite(radius) || !(radius > 0.0)) {
        result.error = "cutter compensation needs a tool radius above zero";
        return result;
    }
    result.moves.resize(moves.size());
    std::size_t k = 0;
    while (k < moves.size()) {
        if (moves[k].comp == CutterComp::Off) {
            result.moves[k] = {false, moves[k]};
            ++k;
            continue;
        }
        std::size_t b = k;
        while (b < moves.size() && moves[b].comp == moves[k].comp) {
            ++b;
        }
        if (std::string err = deriveRun(moves, k, b, radius, from, result.moves); !err.empty()) {
            result.moves.clear();
            result.error = std::move(err);
            return result;
        }
        k = b;
    }
    return result;
}

}  // namespace carvetto::toolpath
