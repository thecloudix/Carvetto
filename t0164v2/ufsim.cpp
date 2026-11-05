// t0164v2 (c): pure-logic differential test of the cluster loop: main's restart loop (6e537a1) vs the branch's
// worklist loop (cce938c), both copied verbatim except that unionContours/intersect are replaced by a
// deterministic oracle: a cluster's "polygons" are a function of its ordered member list, and the vectors they
// touch (in query order) are a pseudo-random function of that list (own members included, as the index returns them).
// Compares the final roots, member lists, crossing flags and resolved[] (the member list each final root was last
// resolved with) -- everything the output loop reads. usage: ufsim <cases> [seed]
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <random>
#include <set>
#include <vector>
using V = std::vector<std::size_t>;
static std::size_t findRoot(V& parent, std::size_t i) { while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; } return i; }
struct Oracle {
    std::size_t n; std::vector<V> T; unsigned long long salt; double pExtra;
    // polygons of a cluster: 1-3 polys; each poly touches a query-ordered list
    std::vector<V> polys(const V& ms) const {
        unsigned long long h = salt; for (auto m : ms) { h ^= m + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2); }
        std::mt19937_64 r(h); int np = 1 + (int)(r() % 3); std::vector<V> out(np);
        std::vector<std::size_t> cand; for (auto m : ms) { for (auto j : T[m]) cand.push_back(j); cand.push_back(m); }
        if (std::uniform_real_distribution<double>(0, 1)(r) < pExtra) cand.push_back(r() % n);  // set-dependent extra touch
        std::sort(cand.begin(), cand.end()); cand.erase(std::unique(cand.begin(), cand.end()), cand.end());
        std::shuffle(cand.begin(), cand.end(), r);
        for (auto j : cand) out[r() % np].push_back(j);
        return out;
    }
};
struct Res { V parentRoots; std::vector<V> members; std::vector<bool> crossing; std::vector<V> resolvedWith; };
static Res runMain(const Oracle& O, V parent, std::vector<bool> crossing) {
    std::size_t n = O.n; std::vector<V> resolved(n); std::vector<V> stableMembers(n);
    for (bool changed = true; changed;) {
        changed = false; std::vector<V> members(n);
        for (std::size_t k = 0; k < n; ++k) members[findRoot(parent, k)].push_back(k);
        for (std::size_t k = 0; k < n && !changed; ++k) {
            bool cross = false; for (auto m : members[k]) cross = cross || crossing[m];
            if (members[k].empty() || !cross) { resolved[k].clear(); stableMembers[k].clear(); continue; }
            if (members[k] == stableMembers[k]) continue;
            resolved[k] = members[k];
            for (const V& poly : O.polys(members[k])) for (auto j : poly) {
                if (findRoot(parent, j) == k) continue;
                crossing[j] = true; const std::size_t rj = findRoot(parent, j); parent[std::max(k, rj)] = std::min(k, rj); changed = true; }
            if (!changed) stableMembers[k] = members[k];
        }
    }
    std::vector<V> members(n); for (std::size_t k = 0; k < n; ++k) members[findRoot(parent, k)].push_back(k);
    Res r; r.members = members; r.crossing = crossing; r.resolvedWith.resize(n);
    for (std::size_t k = 0; k < n; ++k) { bool cross = false; for (auto m : members[k]) cross = cross || crossing[m]; if (!members[k].empty() && cross) r.resolvedWith[k] = resolved[k]; }
    return r;
}
static long gOvw = 0;
static Res runBranch(const Oracle& O, V parent, std::vector<bool> crossing) {
    std::size_t n = O.n; std::vector<V> resolved(n); std::vector<V> stableMembers(n); std::vector<V> members(n);
    for (std::size_t k = 0; k < n; ++k) members[findRoot(parent, k)].push_back(k);
    auto anyCrossing = [&](const V& ms) { return std::any_of(ms.begin(), ms.end(), [&](std::size_t m) { return crossing[m]; }); };
    std::set<std::size_t> pending;
    for (std::size_t k = 0; k < n; ++k) if (!members[k].empty() && anyCrossing(members[k])) pending.insert(k);
    while (!pending.empty()) {
        const std::size_t k = *pending.begin(); pending.erase(pending.begin());
        if (members[k].empty() || members[k] == stableMembers[k]) continue;
        stableMembers[k].clear();
        resolved[k] = members[k];
        V involved{k};
        for (const V& poly : O.polys(members[k])) for (auto j : poly) {
            if (findRoot(parent, j) == k) continue;
            crossing[j] = true; const std::size_t rj = findRoot(parent, j); if (rj < k && parent[k] != k && parent[k] != rj) ++gOvw; involved.push_back(rj); parent[std::max(k, rj)] = std::min(k, rj); }
        if (involved.size() == 1) { stableMembers[k] = members[k]; continue; }
        V moved; for (auto r : involved) { moved.insert(moved.end(), members[r].begin(), members[r].end()); members[r].clear(); }
        std::sort(moved.begin(), moved.end()); moved.erase(std::unique(moved.begin(), moved.end()), moved.end());
        for (auto v : moved) members[findRoot(parent, v)].push_back(v);
        for (auto v : moved) { const std::size_t r = findRoot(parent, v); if (members[r].front() == v && anyCrossing(members[r])) pending.insert(r); }
    }
    Res r; r.members = members; r.crossing = crossing; r.resolvedWith.resize(n);
    for (std::size_t k = 0; k < n; ++k) { bool cross = false; for (auto m : members[k]) cross = cross || crossing[m]; if (!members[k].empty() && cross) r.resolvedWith[k] = resolved[k]; }
    return r;
}
int main(int argc, char** argv) {
    long cases = argc > 1 ? atol(argv[1]) : 100000; std::mt19937_64 g(argc > 2 ? atoll(argv[2]) : 1);
    long diff = 0, nontriv = 0;
    for (long c = 0; c < cases; ++c) {
        Oracle O; O.n = 2 + g() % 40; O.salt = g(); O.pExtra = (g() % 4) * 0.1; O.T.assign(O.n, {});
        double pt = std::uniform_real_distribution<double>(0, 0.15)(g);
        for (std::size_t m = 0; m < O.n; ++m) for (std::size_t j = 0; j < O.n; ++j) if (j != m && std::uniform_real_distribution<double>(0, 1)(g) < pt) O.T[m].push_back(j);
        V parent(O.n); std::iota(parent.begin(), parent.end(), 0); std::vector<bool> crossing(O.n, false);
        double pc = std::uniform_real_distribution<double>(0, 0.2)(g);
        for (std::size_t k = 0; k < O.n; ++k) { if (std::uniform_real_distribution<double>(0, 1)(g) < pc * 0.3) crossing[k] = true;
            for (std::size_t j = k + 1; j < O.n; ++j) if (std::uniform_real_distribution<double>(0, 1)(g) < pc * 0.2) { crossing[k] = crossing[j] = true; auto ra = findRoot(parent, k), rb = findRoot(parent, j); if (ra != rb) parent[std::max(ra, rb)] = std::min(ra, rb); } }
        Res a = runMain(O, parent, crossing), b = runBranch(O, parent, crossing);
        bool same = a.members == b.members && a.crossing == b.crossing && a.resolvedWith == b.resolvedWith;
        std::size_t nc = 0; for (auto& m : a.members) nc += m.size() > 1; nontriv += nc > 0;
        if (!same) { if (++diff <= 5) { std::printf("DIFF case %ld n=%zu\n", c, O.n); for (std::size_t k = 0; k < O.n; ++k) if (a.members[k] != b.members[k] || a.resolvedWith[k] != b.resolvedWith[k]) { std::printf("  root %zu main:", k); for (auto m : a.members[k]) std::printf(" %zu", m); std::printf(" | br:"); for (auto m : b.members[k]) std::printf(" %zu", m); std::printf("\n"); } } }
    }
    std::printf("ufsim cases=%ld nontrivial=%ld overwrites=%ld diff=%ld\n", cases, nontriv, gOvw, diff);
    return diff != 0;
}
