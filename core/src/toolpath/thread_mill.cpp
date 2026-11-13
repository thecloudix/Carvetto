#include "carvetto/toolpath/thread_mill.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <utility>

#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/toolpath/box_index.hpp"
#include "carvetto/toolpath/drill.hpp"
#include "carvetto/toolpath/move.hpp"

namespace carvetto::toolpath {

namespace {

using geometry::Contour;
using geometry::Vec2;

constexpr double kTwoPi = 6.283185307179586;
constexpr double kMinChordal = 1e-4;
constexpr double kMaxSize = 1000.0;       // diameter, pitch bound (mm)
constexpr double kMaxLength = 100000.0;   // depths and lengths (mm)
constexpr double kMaxTurns = 100000.0;    // revolutions per thread
constexpr double kMaxMoves = 20000000.0;  // moves per toolpath
constexpr double kMinPathRadius = 1e-3;   // mm
constexpr double kThreadHeightFactor = 0.8660254037844386;  // H = sqrt(3)/2 P
constexpr double kTurnEpsilon = 1e-9;

struct IsoSize {
    double diameter;
    double pitch;
};
constexpr std::array<IsoSize, 23> kIsoCoarse{{{1.0, 0.25},  {1.2, 0.25},  {1.4, 0.3},  {1.6, 0.35}, {2.0, 0.4},
                                               {2.5, 0.45}, {3.0, 0.5},   {4.0, 0.7},  {5.0, 0.8},  {6.0, 1.0},
                                               {8.0, 1.25}, {10.0, 1.5},  {12.0, 1.75}, {14.0, 2.0}, {16.0, 2.0},
                                               {20.0, 2.5}, {24.0, 3.0},  {30.0, 3.5}, {36.0, 4.0}, {42.0, 4.5},
                                               {48.0, 5.0}, {56.0, 5.5},  {64.0, 6.0}}};
struct UnifiedSize {
    double inches;
    double tpi;
};
constexpr std::array<UnifiedSize, 10> kUnc{{{0.25, 20.0},
                                            {0.3125, 18.0},
                                            {0.375, 16.0},
                                            {0.4375, 14.0},
                                            {0.5, 13.0},
                                            {0.5625, 12.0},
                                            {0.625, 11.0},
                                            {0.75, 10.0},
                                            {0.875, 9.0},
                                            {1.0, 8.0}}};

std::string num(double v) {
    std::array<char, 64> buf{};
    const auto res = std::to_chars(buf.data(), buf.data() + buf.size(), v);
    return std::string(buf.data(), res.ptr);
}

std::string vecName(std::size_t i) {
    return "vector " + std::to_string(i);
}

struct Plan {
    double pitch{0.0};
    double radius{0.0};   // path radius of the tool axis
    double zStart{0.0};
    double zEnd{0.0};
    double turns{0.0};
    bool ccw{false};
    bool up{false};
    double leadLength{0.0};  // external tangent line
};

/// Checks the parameters and tool and fills `out`; an empty string when valid. `issue` says
/// which diagnostic the problem is.
std::string plan(const tooling::Tool& tool, const ThreadMillParams& p, Plan& out, ToolpathIssue& issue) {
    issue = ToolpathIssue::InvalidParameters;
    const double values[] = {p.threadedLength, p.pitch,       p.diameter,   p.fitTolerance,
                             p.startDepth,     p.threadLength, p.maxDepth,    p.safeZ,
                             p.startPosition.x, p.startPosition.y, p.tolerance.distance, p.tolerance.chordal,
                             p.tolerance.angle};
    for (const double v : values) {
        if (!std::isfinite(v)) {
            return "a parameter is not a finite number";
        }
    }
    const std::optional<double> pitch = threadPitch(p);
    if (!pitch) {
        return p.pitchPreset == ThreadPitchPreset::Custom
                   ? "the pitch must be greater than zero"
                   : "the pitch preset has no entry for a diameter of " + num(p.diameter) + " mm";
    }
    const double P = *pitch;
    if (!(P >= 1e-3) || P > kMaxSize) {
        return "the pitch must be between 0.001 and " + num(kMaxSize) + " mm";
    }
    if (!(p.diameter >= 1e-3) || p.diameter > kMaxSize) {
        return "the thread diameter must be between 0.001 and " + num(kMaxSize) + " mm";
    }
    if (std::fabs(p.fitTolerance) > P) {
        return "the fit tolerance must not exceed the pitch";
    }
    if (p.startDepth < 0.0 || p.startDepth > kMaxLength) {
        return "start depth must be between 0 and " + num(kMaxLength) + " mm";
    }
    if (!(p.threadLength >= P * (1.0 - 1e-12)) || p.threadLength > kMaxLength) {
        return "the thread length must be at least one pitch and at most " + num(kMaxLength) + " mm";
    }
    if (p.maxDepth < 0.0) {
        return "max depth must not be negative";
    }
    if (p.maxDepth > 0.0 && p.startDepth + p.threadLength > p.maxDepth * (1.0 + 1e-12)) {
        return "the thread reaches " + num(p.startDepth + p.threadLength) + " mm deep; the maximum depth is " +
               num(p.maxDepth) + " mm";
    }
    const geometry::Tolerance& t = p.tolerance;
    if (t.chordal < kMinChordal || t.chordal > 1.0 || !(t.distance > 0.0) || t.distance > t.chordal) {
        return "invalid tolerance: chordal must be between 0.0001 and 1 mm, distance > 0 and <= chordal";
    }
    if (!threadMillToolAllowed(tool.type)) {
        issue = ToolpathIssue::InvalidTool;
        return "tool '" + tool.name + "' must be a thread mill or an end mill for thread milling";
    }
    if (const std::optional<std::string> bad = tooling::validate(tool)) {
        issue = ToolpathIssue::InvalidTool;
        return "tool '" + tool.name + "': " + *bad;
    }
    // A thread mill tool states its own teeth (ADR-0056): single point when its threaded length is
    // 0, else multi point over that length; the form's cutter fields then do not apply.
    const bool toolTeeth = tool.type == tooling::ToolType::ThreadMill;
    const bool multi = toolTeeth ? tool.threadedLength > 0.0 : p.cutter == ThreadCutter::MultiPoint;
    const double toothedLength = toolTeeth ? tool.threadedLength : p.threadedLength;
    double teeth = P;
    if (multi) {
        teeth = toothedLength;
        if (!(teeth >= P * (1.0 - 1e-12))) {
            return "the threaded length of the cutter must be at least one pitch";
        }
        if (teeth > p.threadLength * (1.0 + 1e-12)) {
            return "the threaded length of the cutter (" + num(teeth) + " mm) is longer than the thread (" +
                   num(p.threadLength) + " mm)";
        }
    }
    const double d = tool.diameter;
    if (p.type == ThreadType::Internal && !(d < threadBoreDiameter(p.diameter, P))) {
        issue = ToolpathIssue::ToolTooLarge;
        return "the tool (" + num(d) + " mm) does not fit the bore of the thread (" +
               num(threadBoreDiameter(p.diameter, P)) + " mm)";
    }
    const std::optional<double> radius = threadPathRadius(p, d);
    if (!radius) {
        issue = ToolpathIssue::ToolTooLarge;
        return p.type == ThreadType::Internal
                   ? "the tool is too large for this thread: its axis would not clear the centre"
                   : "the thread is too thin: its root diameter is not above zero";
    }
    if (!std::isfinite(*radius * 2.0)) {
        return "the thread is too large";
    }
    // Tool axis Z (tip): the teeth span [tip, tip + teeth]; the helix advances (L - teeth) + P so
    // every groove is cut and the extra pitch lies on the open side of the thread.
    const double top = -p.startDepth;
    const double bottom = -(p.startDepth + p.threadLength);
    const double travel = p.threadLength - teeth + P;
    out.up = p.cutDirection == ThreadCutDirection::Up;
    out.zStart = out.up ? bottom : top - teeth + P;
    out.zEnd = out.up ? top - teeth + P : bottom;
    out.pitch = P;
    out.radius = *radius;
    out.turns = travel / P;
    out.ccw = (p.hand == ThreadHand::Right) == out.up;
    out.leadLength = d;
    if (!(out.turns <= kMaxTurns)) {
        issue = ToolpathIssue::WorkLimitExceeded;
        return "more than " + num(kMaxTurns) + " revolutions per thread (pitch too small for the thread length)";
    }
    if (!(p.safeZ > std::max(out.zStart, out.zEnd)) || !(p.safeZ <= kMaxLength)) {
        return "safe Z must be above the highest cutting position (" + num(std::max(out.zStart, out.zEnd)) + " mm)";
    }
    return {};
}

bool centreInside(const Contour& c, Vec2 centre, const geometry::Tolerance& tol) {
    switch (geometry::locatePoint(c, centre, geometry::FillRule::EvenOdd, tol)) {
        case geometry::PointLocation::Inside: return true;
        case geometry::PointLocation::Outside: return false;
        case geometry::PointLocation::OnBoundary: break;
    }
    return geometry::windingNumber(c, centre, tol) % 2 != 0;
}

/// Appends the moves of one thread at `c`.
void emitThread(std::vector<Move>& moves, Vec2 c, const Plan& pl, double safeZ, bool internal) {
    const double r = pl.radius;
    const double sign = pl.up ? 1.0 : -1.0;
    const auto arc = [&](Vec3 to, Vec2 centre) {
        return pl.ccw ? Move::arcCCW(to, centre) : Move::arcCW(to, centre);
    };
    // Full revolutions, then the fraction.
    const double whole = std::floor(pl.turns + kTurnEpsilon);
    double fraction = pl.turns - whole;
    if (fraction < kTurnEpsilon) {
        fraction = 0.0;
    }
    const auto fullTurns = static_cast<std::size_t>(whole);
    const Vec2 p0{c.x + r, c.y};  // helix start and the point of each revolution
    // Direction of travel at angle `a` round the axis.
    const auto tangentAt = [&](double a) {
        return pl.ccw ? Vec2{-std::sin(a), std::cos(a)} : Vec2{std::sin(a), -std::cos(a)};
    };
    const double endAngle = (pl.ccw ? 1.0 : -1.0) * fraction * kTwoPi;
    const Vec2 end = fraction > 0.0 ? Vec2{c.x + r * std::cos(endAngle), c.y + r * std::sin(endAngle)} : p0;

    if (internal) {
        moves.push_back(Move::rapid({c.x, c.y, safeZ}));
        moves.push_back(Move::plunge({c.x, c.y, pl.zStart}));
        moves.push_back(arc({p0.x, p0.y, pl.zStart}, {c.x + 0.5 * r, c.y}));
    } else {
        const Vec2 in = tangentAt(0.0);
        const Vec2 s{p0.x - in.x * pl.leadLength, p0.y - in.y * pl.leadLength};
        moves.push_back(Move::rapid({s.x, s.y, safeZ}));
        moves.push_back(Move::plunge({s.x, s.y, pl.zStart}));
        moves.push_back(Move::feed({p0.x, p0.y, pl.zStart}));
    }
    for (std::size_t i = 0; i < fullTurns; ++i) {
        const bool last = i + 1 == fullTurns && fraction == 0.0;
        const double z = last ? pl.zEnd : pl.zStart + sign * pl.pitch * static_cast<double>(i + 1);
        moves.push_back(arc({p0.x, p0.y, z}, c));
    }
    if (fraction > 0.0) {
        moves.push_back(arc({end.x, end.y, pl.zEnd}, c));
    }
    if (internal) {
        // A half circle back to the centre, in the direction of travel.
        moves.push_back(arc({c.x, c.y, pl.zEnd}, {0.5 * (c.x + end.x), 0.5 * (c.y + end.y)}));
        moves.push_back(Move::retract({c.x, c.y, safeZ}));
    } else {
        const Vec2 t = tangentAt(endAngle);
        const Vec2 out{end.x + t.x * pl.leadLength, end.y + t.y * pl.leadLength};
        moves.push_back(Move::feed({out.x, out.y, pl.zEnd}));
        moves.push_back(Move::retract({out.x, out.y, safeZ}));
    }
}

}  // namespace

bool threadMillToolAllowed(tooling::ToolType type) {
    return type == tooling::ToolType::EndMill || type == tooling::ToolType::ThreadMill;
}

std::optional<double> threadPresetPitch(ThreadPitchPreset preset, double diameter) {
    if (!std::isfinite(diameter)) {
        return std::nullopt;
    }
    switch (preset) {
        case ThreadPitchPreset::Custom: return std::nullopt;
        case ThreadPitchPreset::IsoMetricCoarse:
            for (const IsoSize& s : kIsoCoarse) {
                if (std::fabs(diameter - s.diameter) <= 0.005) {
                    return s.pitch;
                }
            }
            return std::nullopt;
        case ThreadPitchPreset::UnifiedCoarse:
            for (const UnifiedSize& s : kUnc) {
                if (std::fabs(diameter - 25.4 * s.inches) <= 0.005) {
                    return 25.4 / s.tpi;
                }
            }
            return std::nullopt;
    }
    return std::nullopt;
}

std::optional<double> threadPitch(const ThreadMillParams& params) {
    if (params.pitchPreset != ThreadPitchPreset::Custom) {
        return threadPresetPitch(params.pitchPreset, params.diameter);
    }
    if (!std::isfinite(params.pitch) || !(params.pitch > 0.0)) {
        return std::nullopt;
    }
    return params.pitch;
}

double threadBoreDiameter(double diameter, double pitch) {
    if (!std::isfinite(diameter) || !std::isfinite(pitch) || !(diameter > 0.0) || !(pitch > 0.0)) {
        return 0.0;
    }
    // D - 2 H1, H1 = 5/8 H.
    const double bore = diameter - 2.0 * 0.625 * kThreadHeightFactor * pitch;
    return bore > 0.0 ? bore : 0.0;
}

double threadExternalDepth(double pitch) {
    return 17.0 / 24.0 * kThreadHeightFactor * pitch;
}

std::optional<double> threadPathRadius(const ThreadMillParams& params, double toolDiameter) {
    const std::optional<double> pitch = threadPitch(params);
    if (!pitch || !std::isfinite(toolDiameter) || !(toolDiameter > 0.0) || !std::isfinite(params.diameter) ||
        !std::isfinite(params.fitTolerance)) {
        return std::nullopt;
    }
    const double d = 0.5 * toolDiameter;
    const double r = params.type == ThreadType::Internal
                         ? 0.5 * (params.diameter + params.fitTolerance) - d
                         : 0.5 * (params.diameter - 2.0 * threadExternalDepth(*pitch) - params.fitTolerance) + d;
    if (params.type == ThreadType::External &&
        !(0.5 * (params.diameter - 2.0 * threadExternalDepth(*pitch) - params.fitTolerance) > 0.0)) {
        return std::nullopt;
    }
    if (!std::isfinite(r) || !(r >= kMinPathRadius)) {
        return std::nullopt;
    }
    return r;
}

std::vector<Contour> threadBoreCircles(const ThreadMillParams& params, const std::vector<Vec2>& centres) {
    std::vector<Contour> out;
    const std::optional<double> pitch = threadPitch(params);
    if (params.type != ThreadType::Internal || !pitch) {
        return out;
    }
    const double bore = threadBoreDiameter(params.diameter, *pitch);
    if (!(bore > 0.0)) {
        return out;
    }
    out.reserve(centres.size());
    for (const Vec2 c : centres) {
        out.push_back(Contour::circle(c, 0.5 * bore));
    }
    return out;
}

ThreadMillResult threadMillToolpath(const std::vector<Contour>& contours, const tooling::Tool& tool,
                                    const std::optional<tooling::CuttingData>& cuttingData,
                                    const ThreadMillParams& params) {
    ThreadMillResult result;
    result.toolpath.name = params.name;
    auto say = [&result](ToolpathIssue issue, std::optional<std::size_t> contour, std::string message) {
        result.diagnostics.push_back(ToolpathDiagnostic{issue, contour, std::move(message), std::nullopt});
    };
    Plan pl;
    ToolpathIssue issue{};
    if (const std::string bad = plan(tool, params, pl, issue); !bad.empty()) {
        say(issue, std::nullopt, bad);
        return result;
    }
    const geometry::Tolerance& tol = params.tolerance;
    const PassTool pt = passTool(tool, effectiveCuttingData(tool, cuttingData), params.startDepth + params.threadLength);
    result.toolpath.tool = pt.snapshot;
    if (!std::isfinite(pt.snapshot.plungeRate) || !(pt.snapshot.plungeRate > 0.0) ||
        !std::isfinite(pt.snapshot.feedRate) || !(pt.snapshot.feedRate > 0.0)) {
        say(ToolpathIssue::InvalidTool, std::nullopt,
            "tool '" + tool.name + "': the feed and plunge rates must be greater than zero");
        return result;
    }

    struct Candidate {
        Vec2 centre;
        std::size_t contour;
    };
    std::vector<Candidate> candidates;
    std::size_t openCount = 0;
    std::optional<std::size_t> firstOpen;
    for (std::size_t i = 0; i < contours.size(); ++i) {
        const Contour& c = contours[i];
        if (c.isEmpty()) {
            say(ToolpathIssue::EmptyContour, i, vecName(i) + " has no spans; skipped");
            continue;
        }
        if (!c.closed) {
            ++openCount;
            if (!firstOpen) {
                firstOpen = i;
            }
            continue;
        }
        if (!c.isContinuous(tol)) {
            say(ToolpathIssue::DiscontinuousContour, i, vecName(i) + " has gaps between spans; skipped");
            continue;
        }
        const std::optional<Vec2> centre = drillCentre(c, tol);
        if (!centre) {
            say(ToolpathIssue::NoRegion, i, vecName(i) + " encloses no area, so it has no centre; skipped");
            continue;
        }
        if (!centreInside(c, *centre, tol)) {
            say(ToolpathIssue::CentreOutsideVector, i,
                vecName(i) + ": its centre is not inside it; no thread made");
            continue;
        }
        candidates.push_back({*centre, i});
    }
    if (openCount > 0) {
        say(ToolpathIssue::OpenContourIgnored, firstOpen,
            std::to_string(openCount) + " open vector(s) ignored: thread milling needs closed vectors");
    }

    std::vector<geometry::Box2> boxes;
    boxes.reserve(candidates.size());
    for (const Candidate& h : candidates) {
        boxes.push_back(geometry::Box2{h.centre, h.centre});
    }
    const BoxIndex index(std::move(boxes));
    std::vector<bool> kept(candidates.size(), false);
    std::vector<OrderItem> items;
    std::vector<std::size_t> itemCandidate;
    for (std::size_t i = 0; i < candidates.size(); ++i) {
        const Vec2 c = candidates[i].centre;
        const Vec2 d{tol.distance, tol.distance};
        std::optional<std::size_t> same;
        for (const std::size_t j : index.query(geometry::Box2{c - d, c + d})) {
            if (j < i && kept[j] && geometry::distance(candidates[j].centre, c) <= tol.distance) {
                same = j;
                break;
            }
        }
        if (same) {
            say(ToolpathIssue::DuplicateHole, candidates[i].contour,
                vecName(candidates[i].contour) + " has the same centre as " + vecName(candidates[*same].contour) +
                    "; the thread is made once");
            continue;
        }
        kept[i] = true;
        OrderItem item;
        item.path = contours[candidates[i].contour];
        item.path.closed = false;
        item.selectionIndex = candidates[i].contour;
        item.start = c;
        item.end = c;
        items.push_back(std::move(item));
        itemCandidate.push_back(i);
    }
    if (items.empty()) {
        if (result.diagnostics.empty()) {
            say(ToolpathIssue::NoRegion, std::nullopt, "no vectors to thread");
        }
        return result;
    }
    const double perThread = std::ceil(pl.turns) + 8.0;
    if (!(perThread * static_cast<double>(items.size()) <= kMaxMoves)) {
        say(ToolpathIssue::WorkLimitExceeded, std::nullopt,
            "the toolpath would have more than " + num(kMaxMoves) + " moves (too many threads or revolutions)");
        return result;
    }

    const std::vector<std::size_t> order = orderCuts(items, params.order, params.startPosition, tol);
    std::vector<Move>& moves = result.toolpath.moves;
    moves.reserve(static_cast<std::size_t>(perThread) * order.size());
    const bool internal = params.type == ThreadType::Internal;
    for (const std::size_t k : order) {
        const Candidate& cand = candidates[itemCandidate[k]];
        result.threads.push_back({cand.centre, cand.contour});
        emitThread(moves, cand.centre, pl, params.safeZ, internal);
    }
    return result;
}

}  // namespace carvetto::toolpath
