#include "carv_moves.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include "carv_json_reader.hpp"
#include "carvetto/toolpath/cutter_comp.hpp"

namespace carvetto::document::detail {

namespace {

using toolpath::Move;
using toolpath::MoveKind;

constexpr std::array<char, 4> kMagic{'C', 'V', 'T', 'P'};
constexpr std::uint8_t kKindMask = 0x07;
constexpr std::uint8_t kHasFeed = 0x08;
constexpr std::uint8_t kSameX = 0x10;
constexpr std::uint8_t kSameY = 0x20;
constexpr std::uint8_t kSameZ = 0x40;
constexpr std::uint8_t kSameFeed = 0x80;
constexpr int kKindCount = 7;  // Rapid .. Dwell
constexpr std::uint8_t kJetFlag = 0x01;      // version 2: a power follows (or repeats)
constexpr std::uint8_t kJetSame = 0x02;      // version 2: the power equals the last one written
constexpr double kMaxJetPower = 100.0;

std::uint64_t bitsOf(double v) {
    return std::bit_cast<std::uint64_t>(v);
}

void putU64(std::vector<std::uint8_t>& out, std::uint64_t v) {
    for (int i = 0; i < 8; ++i) {
        out.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
    }
}

void putDouble(std::vector<std::uint8_t>& out, double v) {
    putU64(out, bitsOf(v));
}

/// Buffered little-endian reads from a `CharSource`.
class Reader {
   public:
    explicit Reader(CharSource& source) : source_(source) {}

    /// Copies `n` bytes into `dst`; false when the source ends first.
    bool get(std::uint8_t* dst, std::size_t n) {
        while (n > 0) {
            if (pos_ == len_ && !refill()) {
                return false;
            }
            const std::size_t take = std::min(n, len_ - pos_);
            std::memcpy(dst, buf_.data() + pos_, take);
            pos_ += take;
            dst += take;
            n -= take;
        }
        return true;
    }
    bool byte(std::uint8_t& v) {
        if (pos_ == len_ && !refill()) {
            return false;
        }
        v = static_cast<std::uint8_t>(buf_[pos_++]);
        return true;
    }
    bool u64(std::uint64_t& v) {
        std::array<std::uint8_t, 8> b{};
        if (!get(b.data(), b.size())) {
            return false;
        }
        v = 0;
        for (int i = 7; i >= 0; --i) {
            v = (v << 8) | b[static_cast<std::size_t>(i)];
        }
        return true;
    }
    bool real(double& v) {
        std::uint64_t bits = 0;
        if (!u64(bits)) {
            return false;
        }
        v = std::bit_cast<double>(bits);
        return true;
    }
    /// True when the source holds no more bytes.
    bool atEnd() { return pos_ == len_ && !refill(); }

   private:
    bool refill() {
        pos_ = 0;
        len_ = source_.read(buf_.data(), buf_.size());
        return len_ > 0;
    }

    CharSource& source_;
    std::array<char, 1 << 16> buf_{};
    std::size_t pos_{0};
    std::size_t len_{0};
};

}  // namespace

std::string encodeMoves(const std::vector<Move>& moves, std::vector<std::uint8_t>& out) {
    const bool laserMoves = std::any_of(moves.begin(), moves.end(), [](const Move& m) { return m.jetPower != 0.0; });
    const bool compensated = toolpath::usesCutterComp(moves);
    const bool banked = std::any_of(moves.begin(), moves.end(), [](const Move& m) { return m.bank != 0; });
    if (banked && (laserMoves || compensated)) {
        return "a drill bank toolpath has no beam power or cutter compensation";
    }
    // The lowest version that holds the moves; version 3 is version 2's per-move layout plus the runs.
    const std::uint32_t version = banked        ? kMovesFormatVersionDrillBank
                                  : compensated ? kMovesFormatVersionCutterComp
                                  : laserMoves ? kMovesFormatVersionLaser
                                               : kMovesFormatVersion;
    const bool laser = version == kMovesFormatVersionLaser || version == kMovesFormatVersionCutterComp;  // flag byte
    out.insert(out.end(), kMagic.begin(), kMagic.end());
    for (int i = 0; i < 4; ++i) {
        out.push_back(static_cast<std::uint8_t>(version >> (8 * i)));
    }
    putU64(out, moves.size());
    double px = 0.0;
    double py = 0.0;
    double pz = 0.0;
    std::optional<double> lastFeed;
    std::optional<double> lastJet;
    for (std::size_t i = 0; i < moves.size(); ++i) {
        const Move& m = moves[i];
        const int kind = static_cast<int>(m.kind);
        if (kind < 0 || kind >= kKindCount) {
            return "move " + std::to_string(i + 1) + " has an invalid kind";
        }
        const bool arc = toolpath::isArc(m.kind);
        const bool dwell = m.kind == MoveKind::Dwell;
        const bool hasFeed = toolpath::isCutting(m.kind) && m.feedOverride.has_value();
        bool finite = std::isfinite(m.to.x) && std::isfinite(m.to.y) && std::isfinite(m.to.z);
        finite = finite && (!arc || (std::isfinite(m.centre.x) && std::isfinite(m.centre.y)));
        finite = finite && (!dwell || std::isfinite(m.dwellSeconds));
        finite = finite && (!hasFeed || std::isfinite(*m.feedOverride));
        if (!finite) {
            return "move " + std::to_string(i + 1) + " has a non-finite value";
        }
        if (dwell && !(m.dwellSeconds > 0.0)) {
            return "move " + std::to_string(i + 1) + ": dwell seconds must be greater than zero";
        }
        if (hasFeed && !(*m.feedOverride > 0.0)) {
            return "move " + std::to_string(i + 1) + ": feed override must be greater than zero";
        }
        if (m.bank != 0 && !(m.kind == MoveKind::Rapid || m.kind == MoveKind::Plunge || m.kind == MoveKind::Retract ||
                             m.kind == MoveKind::Dwell)) {
            return "move " + std::to_string(i + 1) + ": only a rapid, plunge, retract or dwell can be a drill bank move";
        }
        const bool hasJet = m.jetPower != 0.0;
        if (hasJet && !(std::isfinite(m.jetPower) && m.jetPower > 0.0 && m.jetPower <= kMaxJetPower &&
                        toolpath::isCutting(m.kind))) {
            return "move " + std::to_string(i + 1) +
                   ": beam power must be above zero and at most 100 percent, on a cutting move";
        }
        auto tag = static_cast<std::uint8_t>(kind);
        const bool sameX = bitsOf(m.to.x) == bitsOf(px);
        const bool sameY = bitsOf(m.to.y) == bitsOf(py);
        const bool sameZ = bitsOf(m.to.z) == bitsOf(pz);
        const bool sameFeed = hasFeed && lastFeed && bitsOf(*m.feedOverride) == bitsOf(*lastFeed);
        tag = static_cast<std::uint8_t>(tag | (hasFeed ? kHasFeed : 0) | (sameX ? kSameX : 0) |
                                        (sameY ? kSameY : 0) | (sameZ ? kSameZ : 0) | (sameFeed ? kSameFeed : 0));
        out.push_back(tag);
        std::uint8_t jetByte = 0;
        const bool sameJet = hasJet && lastJet && bitsOf(m.jetPower) == bitsOf(*lastJet);
        if (laser) {
            jetByte = static_cast<std::uint8_t>((hasJet ? kJetFlag : 0) | (sameJet ? kJetSame : 0));
            out.push_back(jetByte);
        }
        if (!sameX) {
            putDouble(out, m.to.x);
        }
        if (!sameY) {
            putDouble(out, m.to.y);
        }
        if (!sameZ) {
            putDouble(out, m.to.z);
        }
        if (arc) {
            putDouble(out, m.centre.x);
            putDouble(out, m.centre.y);
        } else if (dwell) {
            putDouble(out, m.dwellSeconds);
        }
        if (hasFeed) {
            if (!sameFeed) {
                putDouble(out, *m.feedOverride);
            }
            lastFeed = *m.feedOverride;
        }
        if (hasJet) {
            if (!sameJet) {
                putDouble(out, m.jetPower);
            }
            lastJet = m.jetPower;
        }
        px = m.to.x;
        py = m.to.y;
        pz = m.to.z;
    }
    if (compensated) {
        // Runs of equal side (cutter compensation, version 3).
        std::vector<std::uint8_t> runs;
        std::uint64_t runCount = 0;
        std::size_t i = 0;
        while (i < moves.size()) {
            if (moves[i].comp == toolpath::CutterComp::Off) {
                ++i;
                continue;
            }
            const toolpath::CutterComp side = moves[i].comp;
            std::size_t j = i;
            while (j < moves.size() && moves[j].comp == side) {
                ++j;
            }
            putU64(runs, i);
            putU64(runs, j - i);
            runs.push_back(side == toolpath::CutterComp::Left ? 1 : 2);
            ++runCount;
            i = j;
        }
        putU64(out, runCount);
        out.insert(out.end(), runs.begin(), runs.end());
    }
    if (banked) {
        // Runs of equal nonzero spindle sets (drill bank, version 4).
        std::vector<std::uint8_t> runs;
        std::uint64_t runCount = 0;
        std::size_t i = 0;
        while (i < moves.size()) {
            if (moves[i].bank == 0) {
                ++i;
                continue;
            }
            const std::uint64_t set = moves[i].bank;
            std::size_t j = i;
            while (j < moves.size() && moves[j].bank == set) {
                ++j;
            }
            putU64(runs, i);
            putU64(runs, j - i);
            putU64(runs, set);
            ++runCount;
            i = j;
        }
        putU64(out, runCount);
        out.insert(out.end(), runs.begin(), runs.end());
    }
    return {};
}

namespace {

/// Decodes into `out`, or only validates when `out` is null.
std::string decodeImpl(CharSource& source, std::uint64_t expectedCount, std::vector<Move>* out) {
    Reader r(source);
    std::array<std::uint8_t, 4> magic{};
    std::array<std::uint8_t, 4> version{};
    std::uint64_t count = 0;
    if (!r.get(magic.data(), magic.size()) || !r.get(version.data(), version.size()) || !r.u64(count)) {
        return "the move cache is shorter than its header";
    }
    if (std::memcmp(magic.data(), kMagic.data(), kMagic.size()) != 0) {
        return "not a Carvetto move cache (bad signature)";
    }
    const std::uint32_t v = static_cast<std::uint32_t>(version[0]) | (static_cast<std::uint32_t>(version[1]) << 8) |
                            (static_cast<std::uint32_t>(version[2]) << 16) |
                            (static_cast<std::uint32_t>(version[3]) << 24);
    if (v != kMovesFormatVersion && v != kMovesFormatVersionLaser && v != kMovesFormatVersionCutterComp &&
        v != kMovesFormatVersionDrillBank) {
        return "unsupported move cache version " + std::to_string(v);
    }
    if (count != expectedCount) {
        return "holds " + std::to_string(count) + " moves but job.json says " + std::to_string(expectedCount);
    }
    // Exactly what is declared: the count was charged to the move budget (at most one move per
    // byte of the file), so no doubling past it.
    if (out) {
        out->reserve(out->size() + static_cast<std::size_t>(count));
    }
    const std::size_t base = out ? out->size() : 0;
    double px = 0.0;
    double py = 0.0;
    double pz = 0.0;
    std::optional<double> lastFeed;
    std::optional<double> lastJet;
    const bool laser = v == kMovesFormatVersionLaser || v == kMovesFormatVersionCutterComp;  // the flag byte
    for (std::uint64_t i = 0; i < count; ++i) {
        // Moves are numbered from 1, as in the post's errors.
        const auto bad = [i](const char* what) { return "move " + std::to_string(i + 1) + ": " + what; };
        std::uint8_t tag = 0;
        if (!r.byte(tag)) {
            return bad("the move cache ends early");
        }
        const int kind = tag & kKindMask;
        if (kind >= kKindCount) {
            return bad("invalid move kind");
        }
        std::uint8_t jetByte = 0;
        if (laser && !r.byte(jetByte)) {
            return bad("the move cache ends early");
        }
        if ((jetByte & ~(kJetFlag | kJetSame)) != 0 || ((jetByte & kJetSame) != 0 && (jetByte & kJetFlag) == 0)) {
            return bad("invalid laser flags");
        }
        const bool hasJet = (jetByte & kJetFlag) != 0;
        const bool sameJet = (jetByte & kJetSame) != 0;
        Move m;
        m.kind = static_cast<MoveKind>(kind);
        if (hasJet && !toolpath::isCutting(m.kind)) {
            return bad("beam power on a rapid or dwell");
        }
        if (sameJet && !lastJet) {
            return bad("refers to a previous beam power that does not exist");
        }
        const bool hasFeed = (tag & kHasFeed) != 0;
        const bool sameFeed = (tag & kSameFeed) != 0;
        if (hasFeed && !toolpath::isCutting(m.kind)) {
            return bad("a feed override on a rapid or dwell");
        }
        if (sameFeed && (!hasFeed || !lastFeed)) {
            return bad("refers to a previous feed override that does not exist");
        }
        m.to = {px, py, pz};
        bool ok = true;
        if (!(tag & kSameX)) {
            ok = ok && r.real(m.to.x);
        }
        if (!(tag & kSameY)) {
            ok = ok && r.real(m.to.y);
        }
        if (!(tag & kSameZ)) {
            ok = ok && r.real(m.to.z);
        }
        if (toolpath::isArc(m.kind)) {
            ok = ok && r.real(m.centre.x) && r.real(m.centre.y);
        } else if (m.kind == MoveKind::Dwell) {
            ok = ok && r.real(m.dwellSeconds);
        }
        if (hasFeed) {
            double feed = sameFeed ? *lastFeed : 0.0;
            if (!sameFeed) {
                ok = ok && r.real(feed);
            }
            m.feedOverride = feed;
            lastFeed = feed;
        }
        if (hasJet) {
            double power = sameJet ? *lastJet : 0.0;
            if (!sameJet) {
                ok = ok && r.real(power);
            }
            m.jetPower = power;
            lastJet = power;
        }
        if (!ok) {
            return bad("the move cache ends early");
        }
        if (hasJet && !(std::isfinite(m.jetPower) && m.jetPower > 0.0 && m.jetPower <= kMaxJetPower)) {
            return bad("beam power must be above zero and at most 100 percent");
        }
        if (!std::isfinite(m.to.x) || !std::isfinite(m.to.y) || !std::isfinite(m.to.z) ||
            !std::isfinite(m.centre.x) || !std::isfinite(m.centre.y) || !std::isfinite(m.dwellSeconds) ||
            (m.feedOverride && !std::isfinite(*m.feedOverride))) {
            return bad("a non-finite value");
        }
        // A zero or negative pause or feed is never what a toolpath asks for (T-0134).
        if (m.kind == MoveKind::Dwell && !(m.dwellSeconds > 0.0)) {
            return bad("dwell seconds must be greater than zero");
        }
        if (m.feedOverride && !(*m.feedOverride > 0.0)) {
            return bad("feed override must be greater than zero");
        }
        px = m.to.x;
        py = m.to.y;
        pz = m.to.z;
        if (out) {
            out->push_back(m);
        }
    }
    if (v == kMovesFormatVersionCutterComp) {
        std::uint64_t runs = 0;
        if (!r.u64(runs)) {
            return "the move cache ends before its cutter compensation runs";
        }
        if (runs == 0) {
            return "a version 3 move cache without cutter compensation runs";
        }
        std::uint64_t nextFree = 0;  // first move no earlier run has covered
        std::uint8_t lastSide = 0;
        for (std::uint64_t k = 0; k < runs; ++k) {
            std::uint64_t first = 0;
            std::uint64_t length = 0;
            std::uint8_t side = 0;
            if (!r.u64(first) || !r.u64(length) || !r.byte(side)) {
                return "the move cache ends inside its cutter compensation runs";
            }
            const auto badRun = [k](const char* what) {
                return "cutter compensation run " + std::to_string(k + 1) + ": " + what;
            };
            if (side != 1 && side != 2) {
                return badRun("an invalid side");
            }
            if (length == 0 || first < nextFree || first > count || length > count - first) {
                return badRun("outside the moves or overlapping the run before it");
            }
            if (k > 0 && first == nextFree && side == lastSide) {
                return badRun("follows a run of the same side without a gap (runs are written whole)");
            }
            if (out) {
                for (std::uint64_t m = first; m < first + length; ++m) {
                    (*out)[base + static_cast<std::size_t>(m)].comp =
                        side == 1 ? toolpath::CutterComp::Left : toolpath::CutterComp::Right;
                }
            }
            nextFree = first + length;
            lastSide = side;
        }
    }
    if (v == kMovesFormatVersionDrillBank) {
        std::uint64_t runs = 0;
        if (!r.u64(runs)) {
            return "the move cache ends before its drill bank runs";
        }
        if (runs == 0) {
            return "a version 4 move cache without drill bank runs";
        }
        std::uint64_t nextFree = 0;  // first move no earlier run has covered
        std::uint64_t lastSet = 0;
        for (std::uint64_t k = 0; k < runs; ++k) {
            std::uint64_t first = 0;
            std::uint64_t length = 0;
            std::uint64_t set = 0;
            if (!r.u64(first) || !r.u64(length) || !r.u64(set)) {
                return "the move cache ends inside its drill bank runs";
            }
            const auto badRun = [k](const char* what) { return "drill bank run " + std::to_string(k + 1) + ": " + what; };
            if (set == 0) {
                return badRun("an empty spindle set");
            }
            if (length == 0 || first < nextFree || first > count || length > count - first) {
                return badRun("outside the moves or overlapping the run before it");
            }
            if (k > 0 && first == nextFree && set == lastSet) {
                return badRun("follows a run of the same spindle set without a gap (runs are written whole)");
            }
            if (out) {
                for (std::uint64_t m = first; m < first + length; ++m) {
                    Move& target = (*out)[base + static_cast<std::size_t>(m)];
                    if (!(target.kind == MoveKind::Rapid || target.kind == MoveKind::Plunge ||
                          target.kind == MoveKind::Retract || target.kind == MoveKind::Dwell)) {
                        return badRun("covers a move that is not a rapid, plunge, retract or dwell");
                    }
                    target.bank = set;
                }
            }
            nextFree = first + length;
            lastSet = set;
        }
    }
    if (!r.atEnd()) {
        return "data follows the last move";
    }
    return {};
}

}  // namespace

std::string decodeMoves(CharSource& source, std::uint64_t expectedCount, std::vector<Move>& out) {
    return decodeImpl(source, expectedCount, &out);
}

std::string validateMoves(CharSource& source, std::uint64_t expectedCount) {
    return decodeImpl(source, expectedCount, nullptr);
}

}  // namespace carvetto::document::detail
