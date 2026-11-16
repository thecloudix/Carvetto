#include "carvetto/toolpath/drill_bank.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/toolpath/box_index.hpp"
#include "carvetto/toolpath/cut_order.hpp"
#include "carvetto/toolpath/move.hpp"
#include "carvetto/toolpath/pass_plan.hpp"

namespace carvetto::toolpath {

namespace {

using geometry::Contour;
using geometry::Vec2;

constexpr int kMaxLine = 32;                  // spindles per row or column
constexpr double kMaxMoves = 20000000.0;      // moves per toolpath
constexpr double kMaxLevels = 10000.0;        // passes per stroke
constexpr double kMaxLength = 1.0e6;          // mm: pitch, offsets, diameters
constexpr double kMaxDepth = 1.0e5;           // mm
constexpr std::size_t kMaxReported = 100;     // per-hole diagnostics of unmatched holes
constexpr std::size_t kPollEvery = 1024;      // placement evaluations between two polls of the cancel flag

std::string num(double v) {
    std::array<char, 64> buf{};
    const auto res = std::to_chars(buf.data(), buf.data() + buf.size(), v);
    return std::string(buf.data(), res.ptr);
}

std::string holeName(const DrillHole& h) {
    return "vector " + std::to_string(h.contour) + " (hole at " + num(h.centre.x) + ", " + num(h.centre.y) + ")";
}

bool finiteIn(double v, double lo, double hi) {
    return std::isfinite(v) && v >= lo && v <= hi;
}

int spindleCount(const DrillBankParams& p) {
    return p.horizontalDrills + p.verticalDrills;
}

}  // namespace

std::vector<DrillBankSpindle> drillBankSpindles(const DrillBankParams& p) {
    std::vector<DrillBankSpindle> out;
    const int h = std::max(p.horizontalDrills, 0);
    const int v = std::max(p.verticalDrills, 0);
    out.reserve(static_cast<std::size_t>(h + v));
    for (int i = 0; i < h; ++i) {
        DrillBankSpindle s;
        s.number = static_cast<int>(out.size()) + 1;
        s.offset = {(static_cast<double>(i) - 0.5 * static_cast<double>(h - 1)) * p.pitch, p.rowOffset};
        out.push_back(s);
    }
    for (int j = 0; j < v; ++j) {
        DrillBankSpindle s;
        s.number = static_cast<int>(out.size()) + 1;
        s.offset = {p.columnOffset, (static_cast<double>(j) - 0.5 * static_cast<double>(v - 1)) * p.pitch};
        out.push_back(s);
    }
    for (DrillBankSpindle& s : out) {
        const std::size_t k = static_cast<std::size_t>(s.number - 1);
        if (k < p.spindleDiameters.size()) {
            s.diameter = p.spindleDiameters[k];
        }
        if (k < p.spindleDepths.size()) {
            s.depth = p.spindleDepths[k];
        }
        s.active = std::find(p.inactiveSpindles.begin(), p.inactiveSpindles.end(), s.number) ==
                   p.inactiveSpindles.end();
    }
    return out;
}

std::vector<BankSpindleSpec> drillBankHead(const DrillBankParams& p, double toolDiameter) {
    const std::vector<DrillBankSpindle> all = drillBankSpindles(p);
    std::vector<BankSpindleSpec> out;
    if (all.empty()) {
        return out;
    }
    const std::size_t ref =
        p.referenceDrill >= 1 && static_cast<std::size_t>(p.referenceDrill) <= all.size()
            ? static_cast<std::size_t>(p.referenceDrill - 1)
            : 0;
    out.reserve(all.size());
    for (const DrillBankSpindle& s : all) {
        out.push_back({s.offset - all[ref].offset, s.diameter > 0.0 ? s.diameter : toolDiameter,
                       s.depth > 0.0 ? s.depth : p.cutDepth});
    }
    return out;
}

std::uint64_t drillBankSpindleBit(int number) {
    return number >= 1 && number <= 64 ? std::uint64_t{1} << (number - 1) : 0;
}

std::string drillBankParamProblem(const DrillBankParams& p) {
    if (p.horizontalDrills < 0 || p.horizontalDrills > kMaxLine || p.verticalDrills < 0 ||
        p.verticalDrills > kMaxLine) {
        return "the head has 0 to " + std::to_string(kMaxLine) + " drills on its row and on its column";
    }
    const int count = spindleCount(p);
    if (count < 1) {
        return "the head has no drills";
    }
    if (!(std::isfinite(p.pitch) && p.pitch > 0.0 && p.pitch <= kMaxLength)) {
        return "the pitch must be a finite number greater than zero";
    }
    if (!finiteIn(p.rowOffset, -kMaxLength, kMaxLength) || !finiteIn(p.columnOffset, -kMaxLength, kMaxLength)) {
        return "the row and column offsets must be finite numbers";
    }
    if (p.referenceDrill < 1 || p.referenceDrill > count) {
        return "the reference drill must be a drill number from 1 to " + std::to_string(count);
    }
    std::vector<bool> off(static_cast<std::size_t>(count), false);
    for (const int n : p.inactiveSpindles) {
        if (n < 1 || n > count) {
            return "inactive drill " + std::to_string(n) + " does not exist";
        }
        if (off[static_cast<std::size_t>(n - 1)]) {
            return "inactive drill " + std::to_string(n) + " is listed twice";
        }
        off[static_cast<std::size_t>(n - 1)] = true;
    }
    if (static_cast<int>(p.inactiveSpindles.size()) >= count) {
        return "every drill is switched off";
    }
    if (!p.spindleDiameters.empty() && p.spindleDiameters.size() != static_cast<std::size_t>(count)) {
        return "give a diameter for every drill or for none";
    }
    for (const double d : p.spindleDiameters) {
        if (!(std::isfinite(d) && d > 0.0 && d <= kMaxLength)) {
            return "drill diameters must be finite numbers greater than zero";
        }
    }
    if (!p.spindleDepths.empty() && p.spindleDepths.size() != static_cast<std::size_t>(count)) {
        return "give a depth for every drill or for none";
    }
    for (const double d : p.spindleDepths) {
        if (!finiteIn(d, 0.0, kMaxDepth)) {
            return "drill depths must be finite numbers from 0 to " + num(kMaxDepth);
        }
    }
    if (!finiteIn(p.pitchTolerance, 1e-6, kMaxLength)) {
        return "the pitch tolerance must be at least 0.000001 mm";
    }
    if (!finiteIn(p.diameterTolerance, 0.0, kMaxLength)) {
        return "the diameter tolerance must not be negative";
    }
    const int active = count - static_cast<int>(p.inactiveSpindles.size());
    if (p.minHoles < 1 || p.minHoles > active) {
        return "the minimum number of holes per stroke must be from 1 to the number of active drills (" +
               std::to_string(active) + ")";
    }
    // Neighbouring tolerance zones must not overlap, so every hole belongs to at most one spindle of a stroke.
    const std::vector<DrillBankSpindle> sp = drillBankSpindles(p);
    double closest = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < sp.size(); ++i) {
        for (std::size_t j = i + 1; j < sp.size(); ++j) {
            closest = std::min(closest, geometry::distance(sp[i].offset, sp[j].offset));
        }
    }
    if (!(closest > 2.0 * p.pitchTolerance)) {
        return "the pitch tolerance must be less than half the distance between the two closest drills (" +
               num(closest) + " mm; drills at the same place are not allowed)";
    }
    return {};
}

double drillBankHoleDiameter(const Contour& contour) {
    return 2.0 * std::sqrt(std::fabs(geometry::signedArea(contour)) / std::numbers::pi);
}

namespace {

/// A candidate stroke: the reference spindle placed so that active spindle `spindle` (index in the active list)
/// is exactly over hole `anchor`, which no stroke has drilled yet.
struct Candidate {
    std::size_t gain{0};
    std::size_t anchor{0};
    std::size_t spindle{0};
};

/// The priority queue's "less": lower gain, then the higher anchor, then the higher spindle come out later.
struct CandidateLess {
    bool operator()(const Candidate& a, const Candidate& b) const {
        if (a.gain != b.gain) {
            return a.gain < b.gain;
        }
        if (a.anchor != b.anchor) {
            return a.anchor > b.anchor;
        }
        return a.spindle > b.spindle;
    }
};

struct Claim {
    std::size_t spindle{0};  // index in the active list
    std::size_t hole{0};
    double distance{0.0};
};

}  // namespace

DrillBankResult drillBankToolpath(const std::vector<Contour>& contours, const tooling::Tool& tool,
                                  const std::optional<tooling::CuttingData>& cuttingData,
                                  const DrillBankParams& params, const ToolpathRunControl& control) {
    DrillBankResult result;
    result.toolpath.name = params.name;
    auto say = [&result](ToolpathIssue issue, std::optional<std::size_t> contour, std::string message) {
        result.diagnostics.push_back(ToolpathDiagnostic{issue, contour, std::move(message), std::nullopt});
    };
    const auto cancelled = [&control] { return control.cancel != nullptr && control.cancel->load(); };
    const auto stop = [&]() {
        result.toolpath.moves.clear();
        result.placements.clear();
        result.unmatched.clear();
        say(ToolpathIssue::Cancelled, std::nullopt, "cancelled");
        return result;
    };

    if (const std::string bad = drillParamProblem(params); !bad.empty()) {
        say(ToolpathIssue::InvalidParameters, std::nullopt, bad);
        return result;
    }
    if (const std::string bad = drillBankParamProblem(params); !bad.empty()) {
        say(ToolpathIssue::InvalidParameters, std::nullopt, bad);
        return result;
    }
    const geometry::Tolerance& tol = params.tolerance;
    const std::vector<DrillBankSpindle> all = drillBankSpindles(params);
    double deepest = params.cutDepth;
    for (const DrillBankSpindle& s : all) {
        if (s.active && s.depth > 0.0) {
            deepest = std::max(deepest, s.depth);
        }
    }
    // The deepest hole sets the tool's cutting width (as `drillToolpath` does for its depth) and the checks.
    const PassTool pt = passTool(tool, effectiveCuttingData(tool, cuttingData), params.startDepth + deepest);
    result.toolpath.tool = pt.snapshot;
    if (const std::string bad = drillToolProblem(tool, pt, params); !bad.empty()) {
        say(ToolpathIssue::InvalidTool, std::nullopt, bad);
        return result;
    }
    const double toolDiameter = pt.snapshot.diameter;

    // The active spindles with their effective diameter and depth, and positions relative to the reference.
    struct Active {
        int number{0};
        Vec2 rel;
        double diameter{0.0};
        double depth{0.0};
    };
    std::vector<Active> active;
    const Vec2 refOffset = all[static_cast<std::size_t>(params.referenceDrill - 1)].offset;
    for (const DrillBankSpindle& s : all) {
        if (s.active) {
            active.push_back({s.number, s.offset - refOffset, s.diameter > 0.0 ? s.diameter : toolDiameter,
                              s.depth > 0.0 ? s.depth : params.cutDepth});
        }
    }
    const std::size_t minHoles = static_cast<std::size_t>(params.minHoles);

    // Strokes are made of passes: cap the passes of the deepest stroke before anything else.
    const auto levelsFor = [&](double depth) {
        PassPlan plan;
        plan.startDepth = params.startDepth;
        plan.cutDepth = depth;
        plan.passDepth = params.peck ? pt.passDepth : 0.0;
        plan.maintainExactStep = true;
        return plan.levels();
    };
    if (params.peck && !(std::ceil(deepest / pt.passDepth) <= kMaxLevels + 1.0)) {
        say(ToolpathIssue::WorkLimitExceeded, std::nullopt,
            "more than " + num(kMaxLevels) + " passes per hole (pass depth " + num(pt.passDepth) +
                " mm too small for the cut depth)");
        return result;
    }
    if (static_cast<double>(levelsFor(deepest).size()) > kMaxLevels) {
        say(ToolpathIssue::WorkLimitExceeded, std::nullopt,
            "more than " + num(kMaxLevels) + " passes per hole (pass depth " + num(pt.passDepth) +
                " mm too small for the cut depth)");
        return result;
    }
    if (cancelled()) {
        return stop();
    }

    // Holes (the radius 0 skips the tool-wider-than-vector check: the spindles' drills differ).
    const std::vector<DrillHole> holes = drillHoles(contours, 0.0, tol, result.diagnostics);
    if (holes.empty()) {
        if (result.diagnostics.empty()) {
            say(ToolpathIssue::NoRegion, std::nullopt, "no vectors to drill");
        }
        return result;
    }
    std::vector<double> holeDiameter;
    holeDiameter.reserve(holes.size());
    for (const DrillHole& h : holes) {
        holeDiameter.push_back(drillBankHoleDiameter(contours[h.contour]));
    }

    const std::uint64_t limit = control.workLimit != 0 ? control.workLimit : kDrillBankWorkLimit;
    const double estimate = static_cast<double>(holes.size()) * static_cast<double>(active.size()) *
                            static_cast<double>(active.size());
    if (!(estimate <= static_cast<double>(limit))) {
        say(ToolpathIssue::WorkLimitExceeded, std::nullopt,
            "matching " + std::to_string(holes.size()) + " holes to " + std::to_string(active.size()) +
                " active drills is more work than the limit allows");
        return result;
    }

    std::vector<geometry::Box2> boxes;
    boxes.reserve(holes.size());
    for (const DrillHole& h : holes) {
        boxes.push_back(geometry::Box2{h.centre, h.centre});
    }
    const BoxIndex index(std::move(boxes));
    std::vector<bool> matched(holes.size(), false);
    const double ptol = params.pitchTolerance;
    const Vec2 reach{ptol, ptol};

    const auto compatible = [&](const Active& s, std::size_t hole) {
        return !params.matchDiameters || std::fabs(s.diameter - holeDiameter[hole]) <= params.diameterTolerance;
    };

    bool overLimit = false;
    // The holes a stroke with the reference spindle at `at` drills now (at most one per active spindle).
    const auto evaluate = [&](Vec2 at, std::vector<Claim>* claims) {
        ++result.stats.placementEvaluations;
        std::size_t count = 0;
        for (std::size_t a = 0; a < active.size(); ++a) {
            const Vec2 pos = at + active[a].rel;
            ++result.stats.spindleQueries;
            std::optional<Claim> best;
            for (const std::size_t j : index.query(geometry::Box2{pos - reach, pos + reach})) {
                if (matched[j] || !compatible(active[a], j)) {
                    continue;
                }
                const double d = geometry::distance(pos, holes[j].centre);
                if (d <= ptol && (!best || d < best->distance)) {
                    best = Claim{a, j, d};
                }
            }
            if (best) {
                ++count;
                if (claims != nullptr) {
                    claims->push_back(*best);
                }
            }
        }
        if (result.stats.spindleQueries > limit) {
            overLimit = true;
        }
        return count;
    };

    std::priority_queue<Candidate, std::vector<Candidate>, CandidateLess> queue;
    for (std::size_t h = 0; h < holes.size(); ++h) {
        for (std::size_t a = 0; a < active.size(); ++a) {
            if (!compatible(active[a], h)) {
                continue;
            }
            const std::size_t gain = evaluate(holes[h].centre - active[a].rel, nullptr);
            if (overLimit) {
                break;
            }
            if (gain >= minHoles) {
                queue.push({gain, h, a});
            }
            if (result.stats.placementEvaluations % kPollEvery == 0 && cancelled()) {
                return stop();
            }
        }
        if (overLimit) {
            break;
        }
    }

    // Greedy cover. Drilling a hole only lowers the gain of other candidates, so a candidate whose gain, looked
    // at again, equals the stored one is a best one (lazy evaluation).
    std::vector<DrillBankPlacement> placements;
    while (!overLimit && !queue.empty()) {
        const Candidate top = queue.top();
        queue.pop();
        if (matched[top.anchor]) {
            // The anchor was drilled by a stroke already: whatever this stroke still drills is also a candidate
            // anchored on one of those holes.
            continue;
        }
        const Vec2 at = holes[top.anchor].centre - active[top.spindle].rel;
        std::vector<Claim> claims;
        const std::size_t gain = evaluate(at, &claims);
        if (result.stats.placementEvaluations % kPollEvery == 0 && cancelled()) {
            return stop();
        }
        if (gain < minHoles) {
            continue;
        }
        if (gain < top.gain) {
            queue.push({gain, top.anchor, top.spindle});
            continue;
        }
        DrillBankPlacement pl;
        pl.reference = at;
        for (const Claim& c : claims) {
            matched[c.hole] = true;
            DrillBankHole dh;
            dh.centre = holes[c.hole].centre;
            dh.drilledAt = at + active[c.spindle].rel;
            dh.contour = holes[c.hole].contour;
            dh.spindle = active[c.spindle].number;
            dh.depth = active[c.spindle].depth;
            dh.deviation = c.distance;
            pl.depth = std::max(pl.depth, dh.depth);
            pl.holes.push_back(dh);
        }
        placements.push_back(std::move(pl));
    }
    if (overLimit) {
        say(ToolpathIssue::WorkLimitExceeded, std::nullopt,
            "matching the holes to the drills is more work than the limit allows (" +
                std::to_string(holes.size()) + " holes, " + std::to_string(active.size()) + " active drills)");
        return result;
    }

    // Holes no stroke drills.
    for (std::size_t j = 0; j < holes.size(); ++j) {
        if (matched[j]) {
            continue;
        }
        bool anyFits = false;
        for (const Active& s : active) {
            anyFits = anyFits || compatible(s, j);
        }
        result.unmatched.push_back({holes[j].centre, holes[j].contour, !anyFits});
        if (result.unmatched.size() <= kMaxReported) {
            say(ToolpathIssue::BankHoleUnmatched, holes[j].contour,
                !anyFits ? holeName(holes[j]) + ": its " + num(holeDiameter[j]) +
                               " mm diameter fits no active drill within " + num(params.diameterTolerance) + " mm"
                         : holeName(holes[j]) + ": no stroke of the head drills it together with at least " +
                               std::to_string(minHoles - 1) + " other hole(s)");
        }
    }
    if (!result.unmatched.empty()) {
        const std::string count = std::to_string(result.unmatched.size()) + " hole(s) fit no stroke of the drill bank";
        if (params.unmatched == BankUnmatched::Refuse) {
            say(ToolpathIssue::BankHoleUnmatched, std::nullopt, count + "; nothing is cut");
            result.placements.clear();
            return result;
        }
        say(ToolpathIssue::BankHoleUnmatched, std::nullopt, count + "; they are not drilled");
    }
    if (placements.empty()) {
        if (result.diagnostics.empty()) {
            say(ToolpathIssue::NoRegion, std::nullopt, "no vectors to drill");
        }
        return result;
    }

    // Holes drilled off their centre.
    {
        std::size_t moved = 0;
        const DrillBankHole* worst = nullptr;
        for (const DrillBankPlacement& pl : placements) {
            for (const DrillBankHole& h : pl.holes) {
                if (h.deviation > tol.distance) {
                    ++moved;
                    if (worst == nullptr || h.deviation > worst->deviation) {
                        worst = &h;
                    }
                }
            }
        }
        if (moved > 0) {
            say(ToolpathIssue::BankHoleMoved, worst->contour,
                std::to_string(moved) + " hole(s) are drilled off their centre by the spindle pattern, by up to " +
                    num(worst->deviation) + " mm (vector " + std::to_string(worst->contour) + ")");
        }
    }

    // Order of the strokes.
    std::vector<OrderItem> items;
    items.reserve(placements.size());
    for (std::size_t i = 0; i < placements.size(); ++i) {
        OrderItem item;
        geometry::Contour path;
        path.append(geometry::LineSpan{placements[i].reference, placements[i].reference});
        item.path = std::move(path);
        item.selectionIndex = i;
        item.start = placements[i].reference;
        item.end = placements[i].reference;
        items.push_back(std::move(item));
    }
    const std::vector<std::size_t> order = orderCuts(items, params.order, params.startPosition, tol);

    // Moves.
    double movesNeeded = 0.0;
    std::vector<std::pair<double, std::vector<double>>> levelCache;
    const auto levelsOf = [&](double depth) -> const std::vector<double>& {
        for (const auto& entry : levelCache) {
            if (entry.first == depth) {
                return entry.second;
            }
        }
        levelCache.emplace_back(depth, levelsFor(depth));
        return levelCache.back().second;
    };
    for (const DrillBankPlacement& pl : placements) {
        const std::vector<double>& lv = levelsOf(pl.depth);
        if (lv.empty() || static_cast<double>(lv.size()) > kMaxLevels) {
            say(ToolpathIssue::InvalidParameters, std::nullopt, "the cut depth yields no passes");
            return result;
        }
        movesNeeded += 2.0 + static_cast<double>(lv.size()) * (params.dwellTime > 0.0 ? 3.0 : 2.0);
    }
    if (!(movesNeeded <= kMaxMoves)) {
        say(ToolpathIssue::WorkLimitExceeded, std::nullopt,
            "the toolpath would have more than " + num(kMaxMoves) + " moves (too many strokes or passes)");
        return result;
    }

    const double startZ = -params.startDepth;
    const double safeZ = params.safeZ;
    result.toolpath.bankHead = drillBankHead(params, toolDiameter);
    std::vector<Move>& moves = result.toolpath.moves;
    moves.reserve(static_cast<std::size_t>(movesNeeded));
    for (const std::size_t k : order) {
        const DrillBankPlacement& pl = placements[k];
        result.placements.push_back(pl);
        const std::vector<double>& lv = levelsOf(pl.depth);
        const double x = pl.reference.x;
        const double y = pl.reference.y;
        std::uint64_t spindles = 0;
        for (const DrillBankHole& h : pl.holes) {
            spindles |= drillBankSpindleBit(h.spindle);
        }
        const std::size_t strokeFirst = moves.size();
        moves.push_back(Move::rapid({x, y, safeZ}));
        for (std::size_t i = 0; i < lv.size(); ++i) {
            const double z = lv[i];
            moves.push_back(Move::plunge({x, y, z}));
            if (params.dwellTime > 0.0) {
                moves.push_back(Move::dwell({x, y, z}, params.dwellTime));
            }
            if (i + 1 < lv.size()) {
                const double above = params.peckRetract == PeckRetract::AboveStart ? startZ : z;
                moves.push_back(Move::retract({x, y, std::min(above + params.retractGap, safeZ)}));
            }
        }
        moves.push_back(Move::retract({x, y, safeZ}));
        for (std::size_t i = strokeFirst; i < moves.size(); ++i) {
            moves[i].bank = spindles;
        }
    }
    return result;
}

}  // namespace carvetto::toolpath
