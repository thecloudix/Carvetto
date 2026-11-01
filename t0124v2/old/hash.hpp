// ---- sample hash: XY quantised to 0.004 mm keeping min z; buckets of 0.5 mm
struct SampleHash {
    double cell = 0.5;
    std::unordered_map<long long, std::vector<std::array<double, 3>>> b;
    std::unordered_map<long long, size_t> keyIdx;
    std::unordered_map<long long, long long> keyBucket;
    static long long k2(long long i, long long j) { return i * 1000003LL + j; }
    void add(Vec2 q, double z) {
        long long qi = std::llround(q.x / 1e-6), qj = std::llround(q.y / 1e-6);
        long long kq = qi * 1000000007LL + qj * 7919LL;
        long long bi = (long long)std::floor(q.x / cell), bj = (long long)std::floor(q.y / cell);
        long long bk = k2(bi, bj);
        auto& v = b[bk];
        auto it = keyIdx.find(kq);
        if (it != keyIdx.end()) {
            auto& s = b[keyBucket[kq]][it->second];
            if (z < s[2]) s[2] = z;
            return;
        }
        keyIdx[kq] = v.size();
        keyBucket[kq] = bk;
        v.push_back({q.x, q.y, z});
    }
    // min z among samples within rho of X; returns +inf if none. Also returns nearest distance.
    std::pair<double, double> query(Vec2 X, double rho) const {
        double zmin = 1e18, dmin = 1e18;
        long long i0 = (long long)std::floor((X.x - rho) / cell), i1 = (long long)std::floor((X.x + rho) / cell);
        long long j0 = (long long)std::floor((X.y - rho) / cell), j1 = (long long)std::floor((X.y + rho) / cell);
        for (long long i = i0; i <= i1; ++i)
            for (long long j = j0; j <= j1; ++j) {
                auto it = b.find(k2(i, j));
                if (it == b.end()) continue;
                for (auto& s : it->second) {
                    double d = std::hypot(s[0] - X.x, s[1] - X.y);
                    if (d < dmin) dmin = d;
                    if (d <= rho && s[2] < zmin) zmin = s[2];
                }
            }
        return {zmin, dmin};
    }
};

