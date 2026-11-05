static Contour poly(const std::vector<Vec2>& pts, bool closed = true) {
    Contour c;
    for (size_t i = 0; i + 1 < pts.size(); ++i) c.append(geometry::LineSpan{pts[i], pts[i + 1]});
    if (closed) c.append(geometry::LineSpan{pts.back(), pts.front()});
    c.closed = closed;
    return c;
}
static Contour translate(Contour c, Vec2 off) {
    for (auto& sp : c.spans) {
        if (auto* l = std::get_if<geometry::LineSpan>(&sp)) { l->start = l->start + off; l->end = l->end + off; }
        else if (auto* a = std::get_if<geometry::ArcSpan>(&sp)) a->centre = a->centre + off;
        else if (auto* b = std::get_if<geometry::BezierSpan>(&sp)) { b->p0 = b->p0 + off; b->p1 = b->p1 + off; b->p2 = b->p2 + off; b->p3 = b->p3 + off; }
    }
    return c;
}
static Contour reversed(const Contour& c) {
    Contour o;
    o.closed = c.closed;
    for (size_t i = c.spans.size(); i-- > 0;) o.append(geometry::reversed(c.spans[i]));
    return o;
}
// gear: n teeth, alternating radii with flat tops
static Contour gear(Vec2 c, double R, int n, double depth) {
    std::vector<Vec2> p;
    for (int i = 0; i < n; ++i) {
        double a0 = 2 * PI * i / n, w = 2 * PI / n;
        double rr[4] = {R - depth, R, R, R - depth};
        double aa[4] = {a0, a0 + w * 0.2, a0 + w * 0.5, a0 + w * 0.7};
        for (int k = 0; k < 4; ++k) p.push_back({c.x + rr[k] * std::cos(aa[k]), c.y + rr[k] * std::sin(aa[k])});
    }
    return poly(p);
}
// letters from orthogonal outlines
static Contour letter(Vec2 o, double s) {
    double t = s * U(0.2, 0.35);
    switch (I(0, 3)) {
        case 0: return poly({{o.x, o.y}, {o.x + s, o.y}, {o.x + s, o.y + t}, {o.x + t, o.y + t}, {o.x + t, o.y + s * 1.4}, {o.x, o.y + s * 1.4}});  // L
        case 1: return poly({{o.x + s / 2 - t / 2, o.y}, {o.x + s / 2 + t / 2, o.y}, {o.x + s / 2 + t / 2, o.y + s - t}, {o.x + s, o.y + s - t}, {o.x + s, o.y + s}, {o.x, o.y + s}, {o.x, o.y + s - t}, {o.x + s / 2 - t / 2, o.y + s - t}});  // T
        case 2: return poly({{o.x, o.y}, {o.x + s, o.y}, {o.x + s, o.y + s}, {o.x + s - t, o.y + s}, {o.x + s - t, o.y + t}, {o.x + t, o.y + t}, {o.x + t, o.y + s}, {o.x, o.y + s}});  // U
        default: return poly({{o.x, o.y}, {o.x + t, o.y}, {o.x + t, o.y + s / 2 - t / 2}, {o.x + s - t, o.y + s / 2 - t / 2}, {o.x + s - t, o.y}, {o.x + s, o.y}, {o.x + s, o.y + s}, {o.x + s - t, o.y + s}, {o.x + s - t, o.y + s / 2 + t / 2}, {o.x + t, o.y + s / 2 + t / 2}, {o.x + t, o.y + s}, {o.x, o.y + s}});  // H
    }
}
// flower: closed loop of alternating convex/concave arcs through points on a circle
static Contour flower(Vec2 c, double R, int n) {
    Contour out;
    out.closed = true;
    std::vector<Vec2> p;
    for (int i = 0; i < n; ++i) p.push_back({c.x + R * std::cos(2 * PI * i / n), c.y + R * std::sin(2 * PI * i / n)});
    for (int i = 0; i < n; ++i) {
        Vec2 a = p[(size_t)i], b = p[(size_t)((i + 1) % n)];
        double L = geometry::distance(a, b);
        bool convex = (i % 2 == 0) || I(0, 1);
        double sag = U(0.1, 0.45) * L;
        double rad = (L * L / 4 + sag * sag) / (2 * sag);
        Vec2 mid = (a + b) * 0.5, dir = (b - a).normalized(), right{dir.y, -dir.x};
        double h = rad - sag;
        Vec2 cen = convex ? mid - right * h : mid + right * h;
        out.append(geometry::ArcSpan::fromCentre(cen, a, b, convex ? geometry::ArcDirection::CCW : geometry::ArcDirection::CW));
    }
    return out;
}
// spikes: star with needle-sharp points
static Contour spikes(Vec2 c, double R, int n) {
    std::vector<Vec2> p;
    for (int i = 0; i < 2 * n; ++i) {
        double a = PI * i / n + (i % 2 ? U(-0.05, 0.05) : 0);
        double rr = i % 2 ? R * U(0.2, 0.35) : R * U(0.8, 1.0);
        p.push_back({c.x + rr * std::cos(a), c.y + rr * std::sin(a)});
    }
    return poly(p);
}
// chamfered rectangle with short edges
static Contour chamfered(Vec2 o, double w, double h, double ch) {
    return poly({{o.x + ch, o.y}, {o.x + w - ch, o.y}, {o.x + w, o.y + ch}, {o.x + w, o.y + h - ch}, {o.x + w - ch, o.y + h}, {o.x + ch, o.y + h}, {o.x, o.y + h - ch}, {o.x, o.y + ch}});
}
static Contour slot(Vec2 c, double L, double rr, double ang) {
    Vec2 d{std::cos(ang), std::sin(ang)}, n{-d.y, d.x};
    Vec2 a = c - d * (L / 2), b = c + d * (L / 2);
    Contour o;
    o.closed = true;
    o.append(geometry::LineSpan{a - n * rr, b - n * rr});
    o.append(geometry::ArcSpan::fromCentre(b, b - n * rr, b + n * rr, geometry::ArcDirection::CCW));
    o.append(geometry::LineSpan{b + n * rr, a + n * rr});
    o.append(geometry::ArcSpan::fromCentre(a, a + n * rr, a - n * rr, geometry::ArcDirection::CCW));
    return o;
}
// D-shape: line + arc
static Contour dshape(Vec2 c, double R) {
    Contour o;
    o.closed = true;
    double a0 = U(-0.6, -0.1) * PI;
    Vec2 s = c + Vec2::fromAngle(a0, R), e = c + Vec2::fromAngle(PI - a0, R);
    o.append(geometry::ArcSpan::fromCentre(c, s, e, geometry::ArcDirection::CCW));
    o.append(geometry::LineSpan{e, s});
    return o;
}
static Contour anyClosed(Vec2 c, double R) {
    switch (I(0, 8)) {
        case 0: return gear(c, R, I(5, 14), R * U(0.1, 0.35));
        case 1: return letter(c - Vec2{R / 2, R / 2}, R * 1.2);
        case 2: return flower(c, R, 2 * I(2, 4));
        case 3: return spikes(c, R, I(3, 7));
        case 4: return chamfered(c - Vec2{R, R * 0.6}, 2 * R, 1.2 * R, U(0.3, R * 0.3));
        case 5: return slot(c, U(0.5, 2) * R, U(0.25, 0.6) * R, U(0, PI));
        case 6: return dshape(c, R);
        case 7: return Contour::circle(c, R * U(0.4, 1.0), I(0, 1) ? geometry::ArcDirection::CCW : geometry::ArcDirection::CW);
        default: return Contour::rectangle(geometry::Box2{{c.x - R, c.y - R * U(0.3, 1)}, {c.x + R * U(0.3, 1), c.y + R}});
    }
}
static Contour anyOpen(Vec2 c, double R) {
    switch (I(0, 3)) {
        case 0: {  // S-curve of two arcs + line
            Contour o;
            double r1 = U(4, R / 2);
            o.append(geometry::ArcSpan{c, r1, PI, 0, geometry::ArcDirection::CW});
            Vec2 e = o.endPoint();
            o.append(geometry::ArcSpan{e + Vec2{r1, 0}, r1, PI, 2 * PI, geometry::ArcDirection::CCW});
            o.append(geometry::LineSpan{o.endPoint(), o.endPoint() + Vec2{U(3, 15), U(-10, 10)}});
            return o;
        }
        case 1: return poly({c, c + Vec2{U(10, 2 * R), U(-5, 5)}}, false);
        case 2: {  // staircase
            std::vector<Vec2> p{c};
            int n = I(2, 5);
            for (int i = 0; i < n; ++i) {
                p.push_back(p.back() + Vec2{U(5, 15), 0});
                p.push_back(p.back() + Vec2{0, U(5, 15) * (I(0, 1) ? 1 : -1)});
            }
            return poly(p, false);
        }
        default: {  // arc-line-arc hook
            Contour o;
            o.append(geometry::LineSpan{c, c + Vec2{R, 0}});
            double rr = U(3, 12);
            o.append(geometry::ArcSpan{c + Vec2{R, rr}, rr, -PI / 2, PI / 2 * U(0.2, 1.8), geometry::ArcDirection::CCW});
            return o;
        }
    }
}

// ---- sample hash: XY quantised to 0.004 mm keeping min z; buckets of 0.5 mm
