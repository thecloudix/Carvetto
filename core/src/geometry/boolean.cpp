#include "carvetto/geometry/boolean.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <optional>
#include <tuple>
#include <utility>

#include "boolean_batch.hpp"
#include "cancel_poll.hpp"
#include "clipper_interop.hpp"
#include "offset_batch.hpp"

namespace carvetto::geometry {

namespace {

Clipper2Lib::ClipType toClipType(BooleanOp op) {
    switch (op) {
        case BooleanOp::Union:
            return Clipper2Lib::ClipType::Union;
        case BooleanOp::Difference:
            return Clipper2Lib::ClipType::Difference;
        case BooleanOp::Intersection:
            return Clipper2Lib::ClipType::Intersection;
        case BooleanOp::Xor:
            return Clipper2Lib::ClipType::Xor;
    }
    return Clipper2Lib::ClipType::Union;
}

Clipper2Lib::FillRule toFillRule(FillRule fill) {
    return fill == FillRule::EvenOdd ? Clipper2Lib::FillRule::EvenOdd : Clipper2Lib::FillRule::NonZero;
}

void appendPaths(Clipper2Lib::Paths64& to, Clipper2Lib::Paths64&& from) {
    if (to.empty()) {
        to = std::move(from);
    } else {
        to.insert(to.end(), std::make_move_iterator(from.begin()), std::make_move_iterator(from.end()));
    }
}

// One Clipper2 call; results appended to `closedOut` / `openOut`.
void clipOnce(Clipper2Lib::ClipType op, Clipper2Lib::FillRule fill, bool reverse,
              const Clipper2Lib::Paths64& closedSubjects, const Clipper2Lib::Paths64& openSubjects,
              const Clipper2Lib::Paths64& clips, Clipper2Lib::Paths64& closedOut, Clipper2Lib::Paths64& openOut) {
    Clipper2Lib::Clipper64 clipper;
    clipper.PreserveCollinear(false);
    if (reverse) {
        clipper.ReverseSolution(true);
    }
    clipper.AddSubject(closedSubjects);
    clipper.AddOpenSubject(openSubjects);
    clipper.AddClip(clips);
    Clipper2Lib::Paths64 closedPart;
    Clipper2Lib::Paths64 openPart;
    clipper.Execute(op, fill, closedPart, openPart);
    appendPaths(closedOut, std::move(closedPart));
    appendPaths(openOut, std::move(openPart));
}

// The batches of far-apart paths (indices into `paths`) to clip separately, or nothing when one
// Clipper2 call is cheap enough (fewer than `minDepth` batches side by side) or the input does not
// split. Paths in different batches have bounding boxes that neither overlap nor touch, so no edge
// of one batch meets an edge of another and the winding number of every point comes from one
// batch only: for any operation and fill rule, the union of the batches' results is the result of
// one call.
std::optional<std::vector<std::vector<std::size_t>>> planBooleanBatches(
    const std::vector<const Clipper2Lib::Path64*>& paths, std::size_t minDepth) {
    if (paths.size() < 2 || paths.size() < minDepth) {
        return std::nullopt;
    }
    std::vector<detail::GridBox> boxes(paths.size());
    for (std::size_t i = 0; i < paths.size(); ++i) {
        const Clipper2Lib::Rect64 b = Clipper2Lib::GetBounds(*paths[i]);
        boxes[i] = {b.left, b.top, b.right, b.bottom};
    }
    std::vector<std::vector<std::size_t>> batches = detail::separableBatches(boxes);
    if (batches.size() < 2 || batches.size() < minDepth || detail::batchDepth(boxes, batches) < minDepth) {
        return std::nullopt;
    }
    return batches;
}

// Clipper2 over the given paths, batched as planned by planBooleanBatches (the paths are moved
// into the batches). A batch of clip paths alone contributes only to a Union or Xor.
void clipBatched(Clipper2Lib::ClipType op, Clipper2Lib::FillRule fill, bool reverse,
                 Clipper2Lib::Paths64& closedSubjects, Clipper2Lib::Paths64& openSubjects,
                 Clipper2Lib::Paths64& clips, std::size_t minDepth, Clipper2Lib::Paths64& closedOut,
                 Clipper2Lib::Paths64& openOut) {
    std::vector<const Clipper2Lib::Path64*> all;
    all.reserve(closedSubjects.size() + openSubjects.size() + clips.size());
    for (const auto* set : {&closedSubjects, &openSubjects, &clips}) {
        for (const auto& p : *set) {
            all.push_back(&p);
        }
    }
    const std::optional<std::vector<std::vector<std::size_t>>> batches = planBooleanBatches(all, minDepth);
    if (!batches) {
        clipOnce(op, fill, reverse, closedSubjects, openSubjects, clips, closedOut, openOut);
        return;
    }
    // Batches in the order of their first path, each in Clipper2's own order.
    const std::size_t openStart = closedSubjects.size();
    const std::size_t clipStart = openStart + openSubjects.size();
    const bool clipsAloneCount = op == Clipper2Lib::ClipType::Union || op == Clipper2Lib::ClipType::Xor;
    Clipper2Lib::Paths64 cs;
    Clipper2Lib::Paths64 os;
    Clipper2Lib::Paths64 cl;
    for (const auto& batch : *batches) {
        cs.clear();
        os.clear();
        cl.clear();
        for (const std::size_t i : batch) {
            if (i < openStart) {
                cs.push_back(std::move(closedSubjects[i]));
            } else if (i < clipStart) {
                os.push_back(std::move(openSubjects[i - openStart]));
            } else {
                cl.push_back(std::move(clips[i - clipStart]));
            }
        }
        if (cs.empty() && os.empty() && !clipsAloneCount) {
            continue;
        }
        detail::pollCancel();  // T-0228: between batches (one Clipper2 call is not interruptible)
        clipOnce(op, fill, reverse, cs, os, cl, closedOut, openOut);
    }
}

}  // namespace

namespace detail {

namespace {
BooleanResult booleanOpBatchedImpl(BooleanOp op, const std::vector<Contour>& subject,
                                   const std::vector<Contour>& clip, const Tolerance& tol, FillRule fill,
                                   std::size_t minBatchDepth);
}

BooleanResult booleanOpBatched(BooleanOp op, const std::vector<Contour>& subject, const std::vector<Contour>& clip,
                               const Tolerance& tol, FillRule fill, std::size_t minBatchDepth) {
    try {
        const ClipperCancelGuard guard;
        return booleanOpBatchedImpl(op, subject, clip, tol, fill, minBatchDepth);
    } catch (const GeometryCancelled&) {
        return {};  // T-0228: cancelled; the caller discards it
    }
}

namespace {
BooleanResult booleanOpBatchedImpl(BooleanOp op, const std::vector<Contour>& subject,
                                   const std::vector<Contour>& clip, const Tolerance& tol, FillRule fill,
                                   std::size_t minBatchDepth) {
    std::size_t refused = 0;
    Clipper2Lib::Paths64 closedSubjects;
    Clipper2Lib::Paths64 openSubjects;
    Clipper2Lib::Paths64 clips;
    for (const Contour& c : subject) {
        Clipper2Lib::Path64 p = detail::toClipperPath(c, tol, &refused);
        pollCancel();
        if (p.empty()) {
            continue;
        }
        (c.closed ? closedSubjects : openSubjects).push_back(std::move(p));
    }
    for (const Contour& c : clip) {
        if (!c.closed) {
            continue;
        }
        Clipper2Lib::Path64 p = detail::toClipperPath(c, tol, &refused);
        pollCancel();
        if (!p.empty()) {
            clips.push_back(std::move(p));
        }
    }

    BooleanResult result;
    result.refusedContours = refused;
    if (closedSubjects.empty() && openSubjects.empty()) {
        return result;
    }

    Clipper2Lib::Paths64 closedOut;
    Clipper2Lib::Paths64 openOut;
    clipBatched(toClipType(op), toFillRule(fill), false, closedSubjects, openSubjects, clips, minBatchDepth,
                closedOut, openOut);

    pollCancel();
    result.closed = detail::contoursFromClipper(closedOut, true);
    result.open = detail::contoursFromClipper(openOut, false);
    return result;
}
}  // namespace

Clipper2Lib::Paths64 clipClosedPaths(Clipper2Lib::ClipType op, Clipper2Lib::FillRule fill, bool reverse,
                                     Clipper2Lib::Paths64 subjects, Clipper2Lib::Paths64 clips) {
    Clipper2Lib::Paths64 open;
    Clipper2Lib::Paths64 closedOut;
    Clipper2Lib::Paths64 openOut;
    clipBatched(op, fill, reverse, subjects, open, clips, kOffsetBatchMinDepth, closedOut, openOut);
    return closedOut;
}

}  // namespace detail

BooleanResult booleanOp(BooleanOp op, const std::vector<Contour>& subject, const std::vector<Contour>& clip,
                        const Tolerance& tol, FillRule fill) {
    return detail::booleanOpBatched(op, subject, clip, tol, fill, detail::kOffsetBatchMinDepth);
}

std::vector<Contour> unionContours(const std::vector<Contour>& contours, const Tolerance& tol,
                                   FillRule fill) {
    return booleanOp(BooleanOp::Union, contours, {}, tol, fill).closed;
}

// ---------------------------------------------------------------------------------------------
// Open paths against one fixed clip, cropped to each batch's strip (T-0142).
// ---------------------------------------------------------------------------------------------

namespace detail {

StripIndex::StripIndex(std::vector<GridRing> rings, double acrossX, double acrossY)
    : rings_(std::move(rings)), acrossX_(acrossX), acrossY_(acrossY) {
    struct Edge {
        std::size_t ring;
        std::size_t edge;
        double lo;
        double hi;
    };
    std::vector<Edge> edges;
    for (std::size_t r = 0; r < rings_.size(); ++r) {
        const GridRing& ring = rings_[r];
        for (std::size_t i = 0, n = ring.size(); i < n; ++i) {
            const double a = along(ring[i]);
            const double b = along(ring[(i + 1) % n]);
            edges.push_back({r, i, std::min(a, b), std::max(a, b)});
        }
    }
    std::sort(edges.begin(), edges.end(), [](const Edge& a, const Edge& b) {
        return std::tie(a.lo, a.ring, a.edge) < std::tie(b.lo, b.ring, b.edge);
    });
    const std::size_t n = edges.size();
    ring_.resize(n);
    edge_.resize(n);
    lo_.resize(n);
    hi_.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        ring_[i] = edges[i].ring;
        edge_[i] = edges[i].edge;
        lo_[i] = edges[i].lo;
        hi_[i] = edges[i].hi;
    }
    leaves_ = 1;
    while (leaves_ < n) {
        leaves_ *= 2;
    }
    maxHi_.assign(2 * leaves_, -std::numeric_limits<double>::infinity());
    for (std::size_t i = 0; i < n; ++i) {
        maxHi_[leaves_ + i] = hi_[i];
    }
    for (std::size_t i = leaves_ - 1; i >= 1; --i) {
        maxHi_[i] = std::max(maxHi_[2 * i], maxHi_[2 * i + 1]);
    }
}

std::vector<GridRing> StripIndex::crop(double lo, double hi, double margin) const {
    const double qlo = lo - margin;
    const double qhi = hi + margin;
    // Edges with extent [l, h] meeting [qlo, qhi]: l <= qhi (a prefix of the sorted edges) and
    // h >= qlo (the tree skips nodes whose largest h is below qlo).
    const auto prefix = static_cast<std::size_t>(std::upper_bound(lo_.begin(), lo_.end(), qhi) - lo_.begin());
    std::vector<std::pair<std::size_t, std::size_t>> kept;  // (ring, edge)
    if (prefix > 0) {
        struct Node {
            std::size_t id;
            std::size_t first;  // first leaf covered
            std::size_t width;  // leaves covered
        };
        std::vector<Node> stack{{1, 0, leaves_}};
        while (!stack.empty()) {
            const Node node = stack.back();
            stack.pop_back();
            if (node.first >= prefix || maxHi_[node.id] < qlo) {
                continue;
            }
            if (node.width == 1) {
                kept.emplace_back(ring_[node.first], edge_[node.first]);
                continue;
            }
            const std::size_t half = node.width / 2;
            stack.push_back({2 * node.id + 1, node.first + half, half});
            stack.push_back({2 * node.id, node.first, half});
        }
    }
    std::sort(kept.begin(), kept.end());
    std::vector<GridRing> out;
    for (std::size_t k0 = 0; k0 < kept.size();) {
        const std::size_t r = kept[k0].first;
        std::size_t k1 = k0;
        while (k1 < kept.size() && kept[k1].first == r) {
            ++k1;
        }
        const GridRing& ring = rings_[r];
        const std::size_t n = ring.size();
        if (k1 - k0 == n) {
            out.push_back(ring);
        } else {
            // Each kept edge adds its two end points (a point shared with the previous kept edge
            // once); between kept edges that are not neighbours the ring closes with a straight
            // segment, from the last point back to the first included.
            GridRing cropped;
            for (std::size_t k = k0; k < k1; ++k) {
                const std::size_t e = kept[k].second;
                const GridPoint a = ring[e];
                const GridPoint b = ring[(e + 1) % n];
                if (cropped.empty() || !(cropped.back() == a)) {
                    cropped.push_back(a);
                }
                cropped.push_back(b);
            }
            if (cropped.size() > 1 && cropped.front() == cropped.back()) {
                cropped.pop_back();
            }
            out.push_back(std::move(cropped));
        }
        k0 = k1;
    }
    return out;
}

}  // namespace detail

struct OpenPathClipper::Impl {
    Tolerance tol;
    FillRule fill{FillRule::NonZero};
    Vec2 across;
    std::size_t refused{0};
    Clipper2Lib::Paths64 clips;               // the whole clip
    std::optional<detail::StripIndex> index;  // when cropping
};

OpenPathClipper::OpenPathClipper(const std::vector<Contour>& clip, Vec2 across, const Tolerance& tol, FillRule fill)
    : impl_(std::make_unique<Impl>()) {
    impl_->tol = tol;
    impl_->fill = fill;
    impl_->across = across;
    std::size_t edges = 0;
    for (const Contour& c : clip) {
        if (!c.closed) {
            continue;
        }
        Clipper2Lib::Path64 p = detail::toClipperPath(c, tol, &impl_->refused);
        if (!p.empty()) {
            edges += p.size();
            impl_->clips.push_back(std::move(p));
        }
    }
    if (edges >= kCropMinEdges && std::isfinite(across.x) && std::isfinite(across.y) &&
        (across.x != 0.0 || across.y != 0.0)) {
        std::vector<detail::GridRing> rings;
        rings.reserve(impl_->clips.size());
        for (const auto& p : impl_->clips) {
            detail::GridRing r;
            r.reserve(p.size());
            for (const auto& q : p) {
                r.push_back({q.x, q.y});
            }
            rings.push_back(std::move(r));
        }
        impl_->index.emplace(std::move(rings), across.x, across.y);
    }
}

OpenPathClipper::~OpenPathClipper() = default;
OpenPathClipper::OpenPathClipper(OpenPathClipper&&) noexcept = default;
OpenPathClipper& OpenPathClipper::operator=(OpenPathClipper&&) noexcept = default;

std::size_t OpenPathClipper::refusedContours() const {
    return impl_->refused;
}

std::vector<Contour> OpenPathClipper::clip(BooleanOp op, const std::vector<Contour>& subject) const {
    Clipper2Lib::Paths64 openSubjects;
    for (const Contour& c : subject) {
        if (c.closed) {
            continue;
        }
        Clipper2Lib::Path64 p = detail::toClipperPath(c, impl_->tol, &impl_->refused);
        if (!p.empty()) {
            openSubjects.push_back(std::move(p));
        }
    }
    if (openSubjects.empty()) {
        return {};
    }
    Clipper2Lib::Paths64 closedOut;
    Clipper2Lib::Paths64 openOut;
    if (!impl_->index) {
        clipOnce(toClipType(op), toFillRule(impl_->fill), false, {}, openSubjects, impl_->clips, closedOut, openOut);
        return detail::contoursFromClipper(openOut, false);
    }
    const detail::StripIndex& index = *impl_->index;
    double lo = std::numeric_limits<double>::infinity();
    double hi = -lo;
    double scale = 0.0;
    for (const auto& p : openSubjects) {
        for (const auto& q : p) {
            const double v = index.along({q.x, q.y});
            lo = std::min(lo, v);
            hi = std::max(hi, v);
            scale = std::max({scale, std::abs(static_cast<double>(q.x)), std::abs(static_cast<double>(q.y))});
        }
    }
    // Every subject point lies in [lo, hi] (a strip is convex). The margin, two grid units along
    // `across` plus a relative term far above the double rounding of `along` (a few ulps of the
    // coordinates), keeps every replaced run clear of every subject point.
    const double len = std::hypot(impl_->across.x, impl_->across.y);
    const double margin = len * (2.0 + 1e-12 * scale);
    Clipper2Lib::Paths64 clips;
    for (const detail::GridRing& r : index.crop(lo, hi, margin)) {
        Clipper2Lib::Path64 p;
        p.reserve(r.size());
        for (const detail::GridPoint q : r) {
            p.emplace_back(q.x, q.y);
        }
        clips.push_back(std::move(p));
    }
    clipOnce(toClipType(op), toFillRule(impl_->fill), false, {}, openSubjects, clips, closedOut, openOut);
    return detail::contoursFromClipper(openOut, false);
}

}  // namespace carvetto::geometry
