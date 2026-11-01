/// How `insetBy` tells a genuine inset region from an inside-out phantom, and what it does with
/// a genuine region that overshoots.
///
/// Phantom test (both guards, T-0126): a result region is genuine when some point of it (an outer
/// vertex, or the centroid of its outer ring when that lies in the region) keeps the full
/// distance from the source and lies inside it. Such a point belongs to the true inset, so the
/// region does too. A phantom (Clipper2's inside-out result past the inradius) has no such
/// point: every point of it is closer to the walls than the inset. The test is by depth, not by
/// vertex count, so a large region whose small bulge holds most of its vertices is kept.
enum class InsetGuard {
    /// For the tool-centre region, where a vertex or an edge too close to a wall would be a gouge.
    /// Outer and hole vertices are checked. In a genuine region, failing convex vertices are
    /// pulled in (or removed) so the region only shrinks where it overshot (Clipper2's grid, join
    /// sag, zero-width spikes), and every edge so changed is checked against the walls exactly;
    /// if a failing vertex is reflex, or a changed edge comes too close, the region is inset
    /// further until a valid region results. A region is never dropped for overshoot alone; one
    /// that cannot be kept, or is trimmed by more than the join tolerance, is counted and
    /// reported (T-0125, T-0126; ADR-0017 §4).
    Strict,
    /// Phantoms are rejected and genuine regions kept as they come: used everywhere else (offset
    /// levels, rest material, openings), where a vertex near a cusp of the input that comes out
    /// a little too close costs nothing (T-0121 round 3: a 15 x 6 mm neck of rest material was
    /// dropped whole for one such vertex).
    Majority,
};

/// What the `Strict` guard could not keep exactly.
struct InsetReport {
    /// Genuine regions that could not be made valid (not cut).
    std::size_t dropped{0};
    /// Genuine regions kept only after an extra inset larger than the join tolerance, or losing
    /// more area than such an inset would (material near them may be left).
    std::size_t trimmed{0};
};

double groupArea(const Group& g) {
    double a = ringArea(g.outer);
    for (const Ring& h : g.holes) {
        a += ringArea(h);
    }
    return a;
}

double groupPerimeter(const Group& g) {
    double len = 0.0;
    auto ring = [&](const Ring& r) {
        for (std::size_t i = 0, n = r.size(); i < n; ++i) {
            len += geometry::distance(r[i], r[(i + 1) % n]);
        }
    };
    ring(g.outer);
    for (const Ring& h : g.holes) {
        ring(h);
    }
    return len;
}

/// Insets the region of `cs` by d > 0 (offsetContours with −d) and keeps only genuine result
/// regions, as `guard` describes. Clipper2 1.4.0 turns a polygon inset beyond its inradius
/// inside out (a circle inset by more than its radius comes back as a growing circle), so a
/// valid-looking region can appear where nothing fits; every point of a true inset is at
/// distance >= d from the input, which rejects those. Distances use an edge grid; the inside
/// test is made once per region at a point that keeps the distance. With `Strict`, a genuine
/// region that cannot be repaired, or is trimmed noticeably, is counted in `*report` (never
/// silently lost).
std::vector<Contour> insetBy(const std::vector<Contour>& cs, double d, const Tolerance& tol,
                             InsetGuard guard = InsetGuard::Majority, InsetReport* report = nullptr) {
    const std::vector<Contour> raw = offsetBy(cs, -d, tol);
    if (raw.empty()) {
        return raw;
    }
    std::vector<Ring> base;
    for (const Contour& c : cs) {
        base.push_back(ringOf(c));
    }
    // Where two round joins meet, the vertex lies inside both arcs by up to the join tolerance
    // (tol.chordal), which the pocket's safety margin already budgets for.
    const double slack = tol.chordal + 1e-6;
    const double need = d - slack;
    const EdgeGrid grid(base, need);
    auto passes = [&](Vec2 v) { return !grid.anyCloser(v, need); };
    // Pulls each failing vertex that is convex for the region (a left turn: the region lies on
    // the left of every ring, outers counter-clockwise, holes clockwise) towards the midpoint of
    // its neighbours, just far enough (bisection) that it keeps the distance. This covers both a
    // vertex a hair past the slack (moved by about that hair) and the tip of a zero-width
    // out-and-back spike Clipper2 leaves in disc-union insets (moved back down the spike). The
    // new vertex lies inside the triangle (prev, vertex, next), so the region only shrinks, and
    // by no more than needed. When the midpoint fails too (a spike over several vertices) the
    // vertex is dropped, which cuts that triangle off, and its neighbours follow in later passes
    // (rings keep at least three vertices). A failing reflex vertex is left for the inset repair
    // below. A vertex that keeps the distance does not make its edges keep it (a chord across a
    // wall corner can sag towards it), so every edge with a moved end, and every chord left by a
    // removal, is then checked exactly against the walls (T-0126). Returns true when every
    // vertex and every such edge of every ring keeps the distance.
    auto pullIn = [&](Group& g) {
        bool allOk = true;
        auto ring = [&](Ring& r) {
            std::vector<char> touched(r.size(), 0);  // vertex moved, or a neighbour removed
            for (int pass = 0; pass < 64; ++pass) {
                bool changed = false;
                for (std::size_t i = 0, n = r.size(); i < n; ++i) {
                    if (i >= r.size() || passes(r[i])) {
                        continue;
                    }
                    const Vec2 prev = r[(i + n - 1) % n];
                    const Vec2 next = r[(i + 1) % n];
                    const Vec2 in = r[i] - prev;
                    const Vec2 out = next - r[i];
                    const Vec2 mid = (prev + next) * 0.5;
                    if (in.cross(out) < -1e-12 * in.length() * out.length()) {
                        continue;  // reflex: left for the inset repair
                    }
                    if (!passes(mid)) {
                        // The neighbours fail too (a spike over several vertices): drop this
                        // vertex, cutting the triangle off; the neighbours follow in later passes.
                        if (n > 3) {
                            r.erase(r.begin() + static_cast<std::ptrdiff_t>(i));
                            touched.erase(touched.begin() + static_cast<std::ptrdiff_t>(i));
                            --n;
                            touched[(i + n - 1) % n] = 1;  // the new chord prev -> next
                            touched[i % n] = 1;
                            --i;
                            changed = true;
                        }
                        continue;
                    }
                    double lo = 0.0, hi = 1.0;  // fails at lo, passes at hi
                    for (int it = 0; it < 40; ++it) {
                        const double t = 0.5 * (lo + hi);
                        (passes(r[i] + (mid - r[i]) * t) ? hi : lo) = t;
                    }
                    r[i] = r[i] + (mid - r[i]) * hi;
                    touched[i] = 1;
                    changed = true;
                }
                if (!changed) {
                    break;
                }
            }
            for (std::size_t i = 0, n = r.size(); i < n && allOk; ++i) {
                const std::size_t j = (i + 1) % n;
                allOk = passes(r[i]) && (!(touched[i] || touched[j]) || !grid.anyCloser(r[i], r[j], need));
            }
        };
        ring(g.outer);
        for (Ring& h : g.holes) {
            ring(h);
        }
        return allOk;
    };
    std::vector<Group> kept;
    for (Group& g : groupsOf(raw, tol.distance)) {
        // Phantom test by depth (see InsetGuard): a point of the region at full distance from
        // the walls and inside the source.
        double worst = 0.0;  // largest overshoot past the slack (outer and holes)
        std::optional<Vec2> good;
        for (const Vec2 v : g.outer) {
            const double dist = grid.nearestWithin(v, need);
            if (dist < need) {
                worst = std::max(worst, need - dist);
            } else if (!good && insideRegion(base, v, 0.0)) {
                good = v;
            }
        }
        if (!good && g.outer.size() >= 3) {
            Vec2 c{0.0, 0.0};
            for (const Vec2 v : g.outer) {
                c = c + v;
            }
            c = c * (1.0 / static_cast<double>(g.outer.size()));
            if (passes(c) && insideRegion(ringsOf(g), c, 0.0) && insideRegion(base, c, 0.0)) {
                good = c;
            }
        }
        if (!good) {
            continue;  // phantom (or nothing)
        }
        if (guard == InsetGuard::Majority) {
            kept.push_back(std::move(g));
            continue;
        }
        // Strict: holes count too (a hole vertex too close to its island would be a gouge).
        for (const Ring& h : g.holes) {
            for (const Vec2 v : h) {
                const double dist = grid.nearestWithin(v, need);
                if (dist < need) {
                    worst = std::max(worst, need - dist);
                }
            }
        }
        const double areaBefore = groupArea(g);
        const double perimeter = groupPerimeter(g);
        if (pullIn(g)) {
            kept.push_back(std::move(g));
            continue;
        }
        // Some failing vertex could not be pulled in, or a changed edge comes too close: inset
        // this region further, by twice the worst overshoot (at least 2e-6 mm) and then
        // doubling, until a further inset yields regions whose every vertex (and every edge the
        // pull-in changes) passes. Whether the repair is possible is decided by that result, not
        // by the size of the overshoot; a region that cannot be repaired is reported, never
        // silent, and so is one trimmed by more than the join tolerance.
        bool repaired = false;
        double extra = std::max(2.0 * worst, 2e-6);
        for (int attempt = 0; attempt < 12 && !repaired && extra < d; ++attempt) {
            std::vector<Group> parts = groupsOf(offsetBy(contoursOf(g), -extra, tol), tol.distance);
            repaired = !parts.empty() &&
                       std::all_of(parts.begin(), parts.end(), [&](Group& part) { return pullIn(part); });
            if (repaired) {
                double areaAfter = 0.0;
                for (Group& part : parts) {
                    areaAfter += groupArea(part);
                    kept.push_back(std::move(part));
                }
                const bool large = extra > tol.chordal ||
                                   areaBefore - areaAfter > perimeter * tol.chordal + tol.chordal * tol.chordal;
                if (large && report != nullptr) {
                    ++report->trimmed;
                }
            } else {
                extra *= 2.0;
            }
        }
        if (!repaired && report != nullptr) {
            ++report->dropped;
        }
    }
    return contoursOf(kept);
}

/// Morphological opening: removes slivers narrower than 2·eps (numeric residue of booleans).
