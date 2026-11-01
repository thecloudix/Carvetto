static Contour bezBlob(Vec2 c, double R, int n) {
    std::vector<Vec2> p;
    for (int i = 0; i < n; ++i) {
        double a = 2 * PI * i / n + U(-0.2, 0.2) / n;
        double rr = R * U(0.55, 1.0);
        p.push_back({c.x + rr * std::cos(a), c.y + rr * std::sin(a)});
    }
    Contour o;
    o.closed = true;
    for (int i = 0; i < n; ++i) {
        Vec2 p0 = p[(size_t)((i - 1 + n) % n)], p1 = p[(size_t)i], p2 = p[(size_t)((i + 1) % n)], p3 = p[(size_t)((i + 2) % n)];
        o.append(geometry::BezierSpan{p1, p1 + (p2 - p0) * (1.0 / 6), p2 - (p3 - p1) * (1.0 / 6), p2});
    }
    return o;
}
static Contour bezWave(Vec2 c, double L, int n) {
    Contour o;
    Vec2 a = c;
    for (int i = 0; i < n; ++i) {
        Vec2 b = a + Vec2{L / n, U(-L / n, L / n)};
        double h = U(-0.6, 0.6) * L / n;
        o.append(geometry::BezierSpan{a, a + Vec2{L / (3 * n), h}, b - Vec2{L / (3 * n), -h}, b});
        a = b;
    }
    return o;
}
static Contour ellipse(Vec2 c, double a, double b, double rot) {  // 4 Bezier quarter ellipse
    const double k = 0.5522847498;
    Vec2 ux{std::cos(rot), std::sin(rot)}, uy{-ux.y, ux.x};
    auto P = [&](double x, double y) { return c + ux * x + uy * y; };
    Contour o;
    o.closed = true;
    o.append(geometry::BezierSpan{P(a, 0), P(a, k * b), P(k * a, b), P(0, b)});
    o.append(geometry::BezierSpan{P(0, b), P(-k * a, b), P(-a, k * b), P(-a, 0)});
    o.append(geometry::BezierSpan{P(-a, 0), P(-a, -k * b), P(-k * a, -b), P(0, -b)});
    o.append(geometry::BezierSpan{P(0, -b), P(k * a, -b), P(a, -k * b), P(a, 0)});
    return o;
}

static tooling::Tool endMill(double d) {
    tooling::Tool t;
    t.id = "em"; t.name = "EM"; t.type = tooling::ToolType::EndMill; t.diameter = d;
    t.defaultCuttingData.passDepth = 4.0; t.defaultCuttingData.feedRate = 1000.0; t.defaultCuttingData.plungeRate = 300.0;
    t.defaultCuttingData.spindleSpeed = 18000.0; t.defaultCuttingData.toolNumber = 1;
    return t;
}
static tooling::Tool vbit(double d, double deg) { auto t = endMill(d); t.type = tooling::ToolType::VBit; t.includedAngle = deg * PI / 180; return t; }
