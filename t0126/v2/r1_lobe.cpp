// Near-critical diameters: for every lobe peak (exact local max of distance), tools whose
// centre clearance Rr is just below the lobe depth D (Rr = D - delta). The region is then tiny and
// dominated by junction/cusp vertices. Check the peak lies inside the centre region.
#define main scan_main
#include "r1_scan.cpp"
#undef main

int main(int argc, char** argv) {
    int kind = atoi(argv[1]); uint64_t seed0 = strtoull(argv[2], 0, 10); int nseeds = atoi(argv[3]);
    double chordal = argc > 4 ? atof(argv[4]) : 0.01, allowance = argc > 5 ? atof(argv[5]) : 0.0;
    long checks = 0, drops = 0, dropsMsg = 0, gouges = 0;
    std::map<int, long> dropByDelta;
    for (uint64_t seed = seed0; seed < seed0 + nseeds; ++seed) {
        Shape s = makeShape(kind, seed);
        std::vector<Contour> input;
        for (auto& p : s.polys) input.push_back(Contour::fromPolyline(geometry::Polyline{p, true}));
        double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300;
        for (auto& p : s.polys) for (auto v : p) { x0 = std::min(x0, v.x); y0 = std::min(y0, v.y); x1 = std::max(x1, v.x); y1 = std::max(y1, v.y); }
        double ext = std::max(x1 - x0, y1 - y0);
        Edges E; E.build(s.polys, ext / 128);
        auto SD = [&](Vec2 q) { double d = E.ptDist(q); return insideEO(s.polys, q) ? d : -d; };
        double h = ext / 200; int nx = int((x1 - x0) / h) + 2, ny = int((y1 - y0) / h) + 2;
        std::vector<double> sd(size_t(nx) * ny);
        for (int j = 0; j < ny; ++j) for (int i = 0; i < nx; ++i) sd[size_t(j) * nx + i] = SD({x0 + i * h, y0 + j * h});
        std::vector<std::pair<Vec2, double>> peaks;
        for (int j = 1; j + 1 < ny; ++j) for (int i = 1; i + 1 < nx; ++i) {
            double v = sd[size_t(j) * nx + i]; if (v < 0.3) continue;
            bool pk = true;
            for (int dj = -1; dj <= 1; ++dj) for (int di = -1; di <= 1; ++di) if ((di || dj) && sd[size_t(j + dj) * nx + i + di] > v) pk = false;
            if (!pk) continue;
            // refine by pattern search
            Vec2 p{x0 + i * h, y0 + j * h}; double best = v;
            for (double st = h; st > 1e-7; st *= 0.5) {
                bool moved = true;
                while (moved) { moved = false;
                    for (int a = 0; a < 8; ++a) { Vec2 q{p.x + st * std::cos(a * PI / 4), p.y + st * std::sin(a * PI / 4)}; double w = SD(q); if (w > best) { best = w; p = q; moved = true; } } }
            }
            bool dup = false; for (auto& [q, w] : peaks) if (std::hypot(q.x - p.x, q.y - p.y) < 1e-3) dup = true;
            if (!dup) peaks.push_back({p, best});
        }
        toolpath::PocketParams P; P.tolerance.chordal = chordal; P.tolerance.distance = std::min(P.tolerance.distance, chordal); P.allowance = allowance;
        for (auto& [pk, D] : peaks) {
            for (int e = 0; e < 40; ++e) {
                double delta = 1e-4 * std::pow(10.0, 3.3 * e / 39.0);  // 1e-4 .. 0.2
                double Rr = D - delta, r = Rr - allowance - chordal; if (r <= 0.02) continue;
                double d = 2 * r;
                std::vector<std::string> ms;
                auto C = toolpath::pocketCentreRegion(input, d, P, &ms);
                std::vector<Poly> cp; for (auto& c : C) cp.push_back(c.polygonise(geometry::Tolerance::defaults()).points);
                ++checks;
                for (auto& ring : cp) for (size_t i = 0; i < ring.size(); ++i) if (E.segDist(ring[i], ring[(i + 1) % ring.size()], Rr + 1) < r + allowance - 1e-9) { ++gouges; printf("GOUGE seed %llu d %.12g\n", (unsigned long long)seed, d); goto next; }
                next:
                if (!insideEO(cp, pk)) {
                    ++drops; if (!ms.empty()) ++dropsMsg;
                    int bucket = int(std::floor(std::log10(delta)));
                    ++dropByDelta[bucket];
                    if (delta > 2e-3) printf("DROP %s seed %llu peak (%.5f,%.5f) depth %.6f d=%.12g delta=%.5f msgs=%zu %s\n", s.name.c_str(), (unsigned long long)seed, pk.x, pk.y, D, d, delta, ms.size(), ms.empty() ? "" : ms[0].c_str());
                }
            }
        }
    }
    printf("lobe kind %d seeds %llu..+%d chordal %g allow %g: checks %ld drops %ld (with msg %ld) gouges %ld | drops by log10(delta):", kind, (unsigned long long)seed0, nseeds, chordal, allowance, checks, drops, dropsMsg, gouges);
    for (auto& [b, n] : dropByDelta) printf(" [1e%d:%ld]", b, n);
    printf("\n");
}
