#include "carvetto/simulate/simulator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <new>
#include <utility>

#include "carvetto/toolpath/time.hpp"
#include "det_math.hpp"
#include "sweep.hpp"

namespace carvetto::simulate {

using geometry::Vec3;
using toolpath::Move;
using toolpath::MoveKind;

namespace {

constexpr double kMaxCoordinate = 1e7;  // mm; every product and sum of coordinates stays finite
constexpr double kArcRadiusSlack = 0.005;    // mm; largest accepted start/end radius mismatch of an arc ...
constexpr double kArcRadiusRelative = 1e-6;  // ... or this fraction of the start radius, whichever is larger
constexpr double kMinRadius = 1e-9;     // mm; smaller arc radii / distances count as zero

bool coordOk(double v) {
    return std::isfinite(v) && std::fabs(v) <= kMaxCoordinate;
}

struct Run {
    std::size_t first{0};
    std::size_t count{0};
    ToolShape tool;
    ToolShape shank;
    std::uint32_t id{0};
    int toolNumber{0};
    std::optional<Vec3> from;
};

void rotateBy(double ax, double ay, int dir, double phi, double& rx, double& ry) {
    double qd = std::floor(phi / detail::kHalfPi);
    qd = qd < 0.0 ? 0.0 : (qd > 4.0 ? 4.0 : qd);
    const int quarters = static_cast<int>(qd);
    double rem = phi - qd * detail::kHalfPi;
    rem = rem < 0.0 ? 0.0 : (rem > detail::kHalfPi ? detail::kHalfPi : rem);
    double x = ax;
    double y = ay;
    for (int q = 0; q < quarters; ++q) {
        const double nx = dir > 0 ? -y : y;
        const double ny = dir > 0 ? x : -x;
        x = nx;
        y = ny;
    }
    double s = 0.0;
    double c = 1.0;
    detail::sinCosQuarter(rem, s, c);
    const double d = static_cast<double>(dir);
    rx = x * c - d * y * s;
    ry = d * x * s + y * c;
}

unsigned gFailMask = 0;

/// Consumes the test-hook failure `bit`, throwing as a failed allocation would.
void maybeFail(unsigned bit) {
    if ((gFailMask & bit) != 0) {
        gFailMask &= ~bit;
        throw std::bad_alloc();
    }
}

}  // namespace

namespace detail {
void failNextAllocation(unsigned mask) {
    gFailMask = mask;
}
}  // namespace detail

struct Simulator::Impl {
    Stock stock;
    SimOptions options;
    SimError error;
    StockCheck check;
    Heightfield field;
    Sweeper sweeper;

    std::vector<Move> moves;
    std::vector<Run> runs;
    std::vector<double> endTimes;
    std::optional<Vec3> queueEnd;
    double queueTime{0.0};

    std::vector<SimDiagnostic> diagnostics;
    std::size_t dropped{0};
    std::size_t done{0};
    std::size_t curRun{0};
    std::optional<Vec3> pos;
    double partial{0.0};
    bool allocated{false};
    /// ToolTooDeep verdict of the move at `probedMove`, probed before any partial cut of that move.
    std::optional<std::size_t> probedMove;
    bool probedTooDeep{false};

    Impl(const Stock& s, const SimOptions& o) : stock(s), options(o), check(checkStock(s)), sweeper(field) {
        if (!check.ok()) {
            error.status = check.issue == StockIssue::TooManyCells ? SimStatus::TooManyCells : SimStatus::InvalidStock;
            error.message = check.message;
            return;
        }
        // The byte count is cells * 9 at most; refuse what a size_t cannot hold (32-bit builds).
        if (check.cells() > std::numeric_limits<std::size_t>::max() / 9) {
            error.status = SimStatus::OutOfMemory;
            error.message = "the stock does not fit in the address space";
            return;
        }
        try {
            maybeFail(1);
            field = Heightfield(stock, check);
        } catch (const std::bad_alloc&) {
            field = Heightfield();
            error.status = SimStatus::OutOfMemory;
            error.message = "not enough memory for " + std::to_string(check.cells()) + " cells";
            return;
        }
        allocated = true;
        sweeper.setProbeTolerance(options.collisionTolerance);
    }

    void addDiagnostic(DiagnosticKind kind, std::size_t idx, std::uint32_t id, Vec3 at, const char* text) {
        if (diagnostics.size() >= options.maxDiagnostics) {
            ++dropped;
            return;
        }
        diagnostics.push_back(SimDiagnostic{kind, idx, id, at, text});
    }

    double moveSeconds(const Move& m, const Vec3& from, bool known, const toolpath::ToolSnapshot& snap) const {
        if (m.kind == MoveKind::Dwell) {
            return std::isfinite(m.dwellSeconds) && m.dwellSeconds > 0.0 ? m.dwellSeconds : 0.0;
        }
        const double dist = known ? toolpath::moveLength(m, from) : 0.0;
        double rate = options.rapidRate;
        if (m.kind != MoveKind::Rapid) {
            const bool plungeLike = m.kind == MoveKind::Plunge || m.kind == MoveKind::Retract;
            rate = m.feedOverride ? *m.feedOverride : (plungeLike ? snap.plungeRate : snap.feedRate);
        }
        const double t = toolpath::moveTime(dist, rate, options.acceleration);
        return std::isfinite(t) && t > 0.0 ? t : 0.0;
    }

    /// Arc geometry for a move starting at `from`; false when the arc is a stationary or stamp case.
    bool makeArc(const Move& m, const Vec3& from, ArcGeom& g) const {
        g.cx = m.centre.x;
        g.cy = m.centre.y;
        g.ax = from.x - g.cx;
        g.ay = from.y - g.cy;
        g.rho = std::sqrt(g.ax * g.ax + g.ay * g.ay);
        if (!(g.rho > kMinRadius)) {
            return false;
        }
        const double ex = m.to.x - g.cx;
        const double ey = m.to.y - g.cy;
        const double el = std::sqrt(ex * ex + ey * ey);
        if (!(el > kMinRadius)) {
            return false;
        }
        g.dir = m.kind == MoveKind::ArcCW ? -1 : 1;
        const double cr = g.ax * ey - g.ay * ex;
        const double dt = g.ax * ex + g.ay * ey;
        g.full = cr == 0.0 && dt > 0.0;
        if (g.full) {
            g.bx = g.ax;
            g.by = g.ay;
        } else {
            g.bx = ex * g.rho / el;
            g.by = ey * g.rho / el;
        }
        g.za = from.z;
        g.zb = m.to.z;
        return true;
    }

    /// Sweeps a move's path with `tool` in `mode`; fraction < 1 sweeps only that prefix of it.
    std::size_t sweepMove(const ToolShape& tool, const Move& m, const Vec3& from, SweepMode mode, double fraction) {
        if (isArc(m.kind)) {
            ArcGeom g;
            if (makeArc(m, from, g)) {
                if (fraction < 1.0) {
                    double theta = detail::kTwoPi;
                    if (!g.full) {
                        const double cr = g.ax * g.by - g.ay * g.bx;
                        const double dt = g.ax * g.bx + g.ay * g.by;
                        theta = detail::atan2Det(static_cast<double>(g.dir) * cr, dt);
                        if (theta <= 0.0) {
                            theta += detail::kTwoPi;
                        }
                    }
                    rotateBy(g.ax, g.ay, g.dir, theta * fraction, g.bx, g.by);
                    g.full = false;
                    g.zb = g.za + (g.zb - g.za) * fraction;
                }
                return sweeper.arc(tool, g, mode);
            }
            // Stationary arc (zero radius): the tool only moves in Z.
            return sweeper.segment(tool, from.x, from.y, from.z, from.x, from.y,
                                   from.z + (m.to.z - from.z) * fraction, mode);
        }
        const double ex = fraction < 1.0 ? from.x + (m.to.x - from.x) * fraction : m.to.x;
        const double ey = fraction < 1.0 ? from.y + (m.to.y - from.y) * fraction : m.to.y;
        const double ez = fraction < 1.0 ? from.z + (m.to.z - from.z) * fraction : m.to.z;
        return sweeper.segment(tool, from.x, from.y, from.z, ex, ey, ez, mode);
    }

    void beginRun(std::size_t idx) {
        const Run& run = runs[curRun];
        if (curRun > 0 && pos) {
            const Run& prev = runs[curRun - 1];
            if ((prev.tool != run.tool || prev.toolNumber != run.toolNumber) &&
                pos->z < field.topZ() - options.collisionTolerance) {
                addDiagnostic(DiagnosticKind::ToolChangeWithoutRetract, idx, run.id, *pos,
                              "tool change while the previous tool is still inside the stock");
            }
        }
        if (run.from) {
            pos = run.from;
        }
    }

    /// True when the shank above the flutes would meet material on this move (probe, no cut).
    bool shankHits(const Run& run, const Move& m, const Vec3& from) {
        const double flute = run.tool.fluteLength();
        const double zlow = std::min(from.z, m.to.z);
        return m.kind != MoveKind::Dwell && std::isfinite(flute) && zlow + flute < field.topZ() &&
               sweepMove(run.shank, m, from, SweepMode::Probe, 1.0) > 0;
    }

    bool processMove(std::size_t idx) {
        while (idx >= runs[curRun].first + runs[curRun].count) {
            ++curRun;
        }
        const Run& run = runs[curRun];
        if (idx == run.first) {
            beginRun(idx);
        }
        sweeper.setPassId(run.id);
        Move m = moves[idx];
        const bool known = pos.has_value();
        const Vec3 from = known ? *pos : m.to;
        if (!known && isArc(m.kind)) {
            m.kind = MoveKind::Feed;  // no start point: the tool is stamped at the end point
        }
        const double zlow = std::min(from.z, m.to.z);
        if (m.kind != MoveKind::Dwell) {
            // The shank above the flutes must not meet material: probed against the heights before
            // this move cuts (what the flutes have to remove above their own top is the problem).
            // Computed from the state before any partial cut of this move (advanceToTime), so the
            // diagnostics are a function of the job and the move index, not of the playback history.
            const bool deep = (probedMove && *probedMove == idx) ? probedTooDeep : shankHits(run, m, from);
            if (deep) {
                addDiagnostic(DiagnosticKind::ToolTooDeep, idx, run.id, m.to,
                              "tool is deeper in the material than its flute length");
            }
            if (m.kind == MoveKind::Rapid) {
                if (known && zlow < field.topZ() &&
                    sweepMove(run.tool, m, from, SweepMode::Probe, 1.0) > 0) {
                    addDiagnostic(DiagnosticKind::RapidInMaterial, idx, run.id, m.to,
                                  "rapid move through material that has not been cut");
                }
            } else {
                sweepMove(run.tool, m, from, SweepMode::Cut, 1.0);
            }
            if (zlow < field.bottomZ() - options.collisionTolerance) {
                addDiagnostic(DiagnosticKind::BelowTable, idx, run.id, m.to,
                              "tool tip goes below the bottom of the stock (table or clamp)");
            }
        }
        probedMove.reset();
        pos = m.to;
        if (sweeper.work() > options.maxCellWork) {
            error.status = SimStatus::WorkBudgetExceeded;
            error.message = "simulation work budget exceeded";
            error.move = idx;
            return false;
        }
        return true;
    }

    /// False once the stock was refused or the work budget ran out (until reset()).
    bool usable() const { return allocated && error.status != SimStatus::WorkBudgetExceeded; }

    bool advanceTo(std::size_t n) {
        if (!usable()) {
            return false;
        }
        n = std::min(n, moves.size());
        while (done < n) {
            if (!processMove(done)) {
                return false;
            }
            ++done;
            partial = 0.0;
        }
        return true;
    }

    void resetState() {
        if (allocated) {
            field.reset();
        }
        probedMove.reset();
        probedTooDeep = false;
        sweeper.resetWork();
        diagnostics.clear();
        dropped = 0;
        done = 0;
        curRun = 0;
        pos.reset();
        partial = 0.0;
        if (error.status == SimStatus::WorkBudgetExceeded) {
            error = SimError{};
        }
    }

    double currentTime() const {
        const double base = done > 0 ? endTimes[done - 1] : 0.0;
        if (partial > 0.0 && done < endTimes.size()) {
            return base + partial * (endTimes[done] - base);
        }
        return base;
    }
};

Simulator::Simulator(const Stock& stock, const SimOptions& options) : impl_(std::make_unique<Impl>(stock, options)) {}
Simulator::~Simulator() = default;
Simulator::Simulator(Simulator&&) noexcept = default;
Simulator& Simulator::operator=(Simulator&&) noexcept = default;

bool Simulator::ok() const {
    return impl_->error.ok();
}
const SimError& Simulator::error() const {
    return impl_->error;
}

bool Simulator::addToolpath(const toolpath::Toolpath& path, const ToolShape& tool, std::uint32_t id,
                            std::optional<Vec3> from) {
    try {
        return addToolpathImpl(path, tool, id, from);
    } catch (const std::bad_alloc&) {
        impl_->error.status = SimStatus::OutOfMemory;
        impl_->error.message = "not enough memory to queue the toolpath";
        impl_->error.move = std::nullopt;
        return false;
    }
}

bool Simulator::addToolpathImpl(const toolpath::Toolpath& path, const ToolShape& tool, std::uint32_t id,
                                std::optional<Vec3> from) {
    Impl& s = *impl_;
    if (!s.allocated) {
        return false;
    }
    auto refuse = [&](SimStatus st, std::string msg, std::optional<std::size_t> move = std::nullopt) {
        s.error.status = st;
        s.error.message = std::move(msg);
        s.error.move = move;
        return false;
    };
    if (!tool.valid()) {
        return refuse(SimStatus::InvalidTool, "the tool shape is not valid");
    }
    if (path.moves.size() > s.options.maxMoves || s.moves.size() > s.options.maxMoves - path.moves.size()) {
        return refuse(SimStatus::TooManyMoves, "too many moves queued (limit " + std::to_string(s.options.maxMoves) + ")");
    }
    std::optional<Vec3> p = from ? from : s.queueEnd;
    if (from && !(coordOk(from->x) && coordOk(from->y) && coordOk(from->z))) {
        return refuse(SimStatus::InvalidMove, "the start position is not finite or out of range");
    }
    std::vector<double> times;
    times.reserve(path.moves.size());
    double t = s.queueTime;
    for (std::size_t k = 0; k < path.moves.size(); ++k) {
        const Move& m = path.moves[k];
        const std::size_t global = s.moves.size() + k;
        const bool arc = isArc(m.kind);
        if (!coordOk(m.to.x) || !coordOk(m.to.y) || !coordOk(m.to.z) ||
            (arc && (!coordOk(m.centre.x) || !coordOk(m.centre.y)))) {
            return refuse(SimStatus::InvalidMove, "move " + std::to_string(global) + " has non-finite or out-of-range coordinates",
                          global);
        }
        if (arc && p) {
            const double rx = p->x - m.centre.x;
            const double ry = p->y - m.centre.y;
            const double ex = m.to.x - m.centre.x;
            const double ey = m.to.y - m.centre.y;
            const double rho = std::sqrt(rx * rx + ry * ry);
            const double el = std::sqrt(ex * ex + ey * ey);
            const double chord = std::sqrt((m.to.x - p->x) * (m.to.x - p->x) + (m.to.y - p->y) * (m.to.y - p->y));
            if (!(rho > kMinRadius) && chord > kMinRadius) {
                return refuse(SimStatus::InvalidMove, "move " + std::to_string(global) + " is an arc of zero radius", global);
            }
            if (rho > kMinRadius && !(el > kMinRadius)) {
                return refuse(SimStatus::InvalidMove, "move " + std::to_string(global) + " is an arc that ends at its centre",
                              global);
            }
            if (rho > kMinRadius && std::fabs(el - rho) > std::max(kArcRadiusSlack, kArcRadiusRelative * rho)) {
                return refuse(SimStatus::InvalidMove,
                              "move " + std::to_string(global) + " is an arc whose end radius differs from its start radius by " +
                                  std::to_string(std::fabs(el - rho)) + " mm",
                              global);
            }
        }
        t += s.moveSeconds(m, p ? *p : m.to, p.has_value(), path.tool);
        times.push_back(t);
        p = m.to;
    }
    Run run;
    run.first = s.moves.size();
    run.count = path.moves.size();
    run.tool = tool;
    run.shank = tool.shank();
    run.id = id;
    run.toolNumber = path.tool.toolNumber;
    run.from = from;
    const std::size_t oldMoves = s.moves.size();
    const std::size_t oldTimes = s.endTimes.size();
    try {
        maybeFail(2);
        s.moves.insert(s.moves.end(), path.moves.begin(), path.moves.end());
        s.endTimes.insert(s.endTimes.end(), times.begin(), times.end());
        if (run.count > 0) {
            s.runs.push_back(std::move(run));
        }
    } catch (const std::bad_alloc&) {
        s.moves.resize(oldMoves);
        s.endTimes.resize(oldTimes);
        return refuse(SimStatus::OutOfMemory, "not enough memory to queue " + std::to_string(path.moves.size()) + " moves");
    }
    s.queueEnd = p;
    s.queueTime = t;
    if (s.error.status != SimStatus::WorkBudgetExceeded) {
        s.error = SimError{};
    }
    return true;
}

std::size_t Simulator::moveCount() const {
    return impl_->moves.size();
}
std::size_t Simulator::movesDone() const {
    return impl_->done;
}

bool Simulator::advanceToMove(std::size_t n) {
    return impl_->advanceTo(n);
}

bool Simulator::advanceToTime(double seconds) {
    Impl& s = *impl_;
    if (!s.usable()) {
        return false;
    }
    if (!(seconds == seconds)) {
        return true;
    }
    const auto it = std::upper_bound(s.endTimes.begin(), s.endTimes.end(), seconds);
    const std::size_t completed = static_cast<std::size_t>(it - s.endTimes.begin());
    if (!s.advanceTo(completed)) {
        return false;
    }
    if (s.done < s.moves.size() && s.done == completed) {
        const double start = s.done > 0 ? s.endTimes[s.done - 1] : 0.0;
        const double dur = s.endTimes[s.done] - start;
        if (dur > 0.0 && seconds > start) {
            const double f = (seconds - start) / dur;
            if (f > s.partial && f < 1.0) {
                while (s.done >= s.runs[s.curRun].first + s.runs[s.curRun].count) {
                    ++s.curRun;
                }
                const Run& run = s.runs[s.curRun];
                Move m = s.moves[s.done];
                if (m.kind != MoveKind::Rapid && m.kind != MoveKind::Dwell) {
                    // The first move of a toolpath may be the one in progress: its start position
                    // is the toolpath's `from` (when given) before it runs.
                    const std::optional<Vec3> start3 = (s.done == run.first && run.from) ? run.from : s.pos;
                    s.sweeper.setPassId(run.id);
                    const Vec3 from = start3 ? *start3 : m.to;
                    if (!start3 && isArc(m.kind)) {
                        m.kind = MoveKind::Feed;
                    }
                    if (!(s.probedMove && *s.probedMove == s.done)) {
                        s.probedTooDeep = s.shankHits(run, m, from);  // before the first partial cut
                        s.probedMove = s.done;
                    }
                    s.sweepMove(run.tool, m, from, SweepMode::Cut, f);
                }
                s.partial = f;
            }
        }
    }
    return true;
}

bool Simulator::seekMove(std::size_t n) {
    Impl& s = *impl_;
    if (n < s.done || (n == s.done && s.partial > 0.0)) {
        s.resetState();
    }
    return s.advanceTo(n);
}

bool Simulator::seekTime(double seconds) {
    Impl& s = *impl_;
    if (seconds < s.currentTime()) {
        s.resetState();
    }
    return advanceToTime(seconds);
}

void Simulator::reset() {
    impl_->resetState();
}

const std::vector<double>& Simulator::moveEndTimes() const {
    return impl_->endTimes;
}
double Simulator::totalSeconds() const {
    return impl_->queueTime;
}
double Simulator::currentSeconds() const {
    return impl_->currentTime();
}
double Simulator::partialFraction() const {
    return impl_->partial;
}
const Heightfield& Simulator::heightfield() const {
    return impl_->field;
}
const std::vector<SimDiagnostic>& Simulator::diagnostics() const {
    return impl_->diagnostics;
}
std::size_t Simulator::droppedDiagnostics() const {
    return impl_->dropped;
}
double Simulator::cellWork() const {
    return impl_->sweeper.work();
}

}  // namespace carvetto::simulate
