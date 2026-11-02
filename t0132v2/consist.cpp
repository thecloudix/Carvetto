// T-0132 verifier round 2: save/load consistency probes. Each case: build doc, saveCarv, loadCarv,
// and if loaded, re-save. Prints outcome. A "save OK / load FAIL" line is a consistency bug.
#include <cstdio>
#include <cstring>
#include <iostream>
#include <span>
#include <string>
#include <vector>
#include <random>
#include "carv_zip.hpp"
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/toolpath_record.hpp"
using namespace carvetto;
using namespace carvetto::document;
using carvetto::toolpath::Move;
using Bytes = std::vector<std::uint8_t>;

static void check(const char* label, const Document& doc, const CarvExtras& ex = {}) {
    auto s = saveCarv(doc, ex);
    if (!s.ok()) { std::printf("%-46s save REFUSED (%d): %.160s\n", label, int(s.error().code), s.error().message.c_str()); return; }
    auto l = loadCarv(std::span<const std::uint8_t>(s.value()));
    if (!l.ok()) { std::printf("%-46s save OK (%zu B) / LOAD FAIL (%d): %.200s   <== INCONSISTENT\n", label, s.value().size(), int(l.error().code), l.error().message.c_str()); return; }
    auto s2 = saveCarv(l.value().document, l.value().extras);
    bool same = s2.ok() && s2.value() == s.value();
    std::printf("%-46s save OK (%zu B) / load OK (tp=%zu) / resave %s\n", label, s.value().size(), l.value().document.toolpaths().size(),
                s2.ok() ? (same ? "identical" : "DIFFERENT") : ("REFUSED: " + s2.error().message.substr(0, 120)).c_str());
}

static ToolpathRecord rec(std::vector<Move> mv, bool withMoves = true) {
    ToolpathRecord r; r.name = "t"; r.selection = VectorSelection::allVectors();
    r.tool.name = "tool"; r.tool.diameter = 6; r.tool.feedRate = 1000; r.tool.plungeRate = 300; r.tool.spindleSpeed = 18000;
    if (withMoves) r.moves = shareMoves(std::move(mv));
    return r;
}
static std::vector<Move> randomMoves(std::size_t n, unsigned seed) {
    std::mt19937_64 g(seed); std::uniform_real_distribution<double> d(-500, 500);
    std::vector<Move> v; v.reserve(n);
    for (std::size_t i = 0; i < n; ++i) v.push_back(Move::feed({d(g), d(g), d(g)}));
    return v;
}
static Bytes interimJson(std::size_t nMoves) {
    std::string s = "{\"format\":\"carvetto-toolpath\",\"format_version\":1,\"operation\":\"profile\",\"name\":\"i\",\"params\":{},\"safe_z\":5,"
                    "\"tool\":{\"name\":\"x\",\"tool_number\":1,\"diameter\":6,\"feed_rate\":1000,\"plunge_rate\":300,\"spindle_speed\":1},\"moves\":[";
    for (std::size_t i = 0; i < nMoves; ++i) { if (i) s += ','; s += "[\"rapid\",0,0,5]"; }
    s += "]}";
    return Bytes(s.begin(), s.end());
}

int main(int argc, char** argv) {
    std::string only = argc > 1 ? argv[1] : "";
    auto want = [&](const char* k) { return only.empty() || only == k; };
    const std::vector<Move> one{Move::rapid({1, 2, 30})};
    if (want("count")) {
        for (std::size_t n : {9997u, 9998u, 9999u, 10000u}) {
            Document d; for (std::size_t i = 0; i < n; ++i) d.insertToolpath(rec(randomMoves(3, unsigned(i))));
            check(("computed tp " + std::to_string(n)).c_str(), d);
            CarvExtras e1 = CarvExtras{}.withMember("extra/a.bin", Bytes(100, 7));
            check(("computed tp " + std::to_string(n) + " + 1 extra").c_str(), d, e1);
            CarvExtras e2 = e1.withMember("extra/b.bin", Bytes(100, 8));
            check(("computed tp " + std::to_string(n) + " + 2 extras").c_str(), d, e2);
        }
    }
    if (want("nomoves")) {
        for (std::size_t n : {10000u, 10001u, 12000u}) {
            Document d; for (std::size_t i = 0; i < n; ++i) d.insertToolpath(rec({}, false));
            check(("uncomputed tp " + std::to_string(n)).c_str(), d);
        }
        // 9998 computed + 3000 uncomputed after
        Document d; for (std::size_t i = 0; i < 9998; ++i) d.insertToolpath(rec(randomMoves(2, unsigned(i))));
        for (std::size_t i = 0; i < 3000; ++i) d.insertToolpath(rec({}, false));
        check("9998 computed + 3000 uncomputed", d);
    }
    if (want("interimextra")) {
        // API path: interim member in extras
        Document d; for (std::size_t i = 0; i < 10000; ++i) d.insertToolpath(rec({}, false));
        CarvExtras e = CarvExtras{}.withMember("toolpaths/0001.json", interimJson(3));
        check("10000 uncomputed + interim extra", d, e);
        Document d2; for (std::size_t i = 0; i < 9998; ++i) d2.insertToolpath(rec(randomMoves(2, unsigned(i))));
        CarvExtras e2 = CarvExtras{}.withMember("toolpaths/0001.json", interimJson(3)); // replaces nothing (bin vs json)
        check("9998 computed + interim extra (9999th)", d2, e2);
        Document d3; d3.insertToolpath(rec(randomMoves(1000, 1)));
        CarvExtras e3 = CarvExtras{}.withMember("toolpaths/0002.json", interimJson(60000));
        check("1000 native + 60000-move interim extra", d3, e3);
    }
    if (want("strings")) {
        Document d; auto r = rec(one); r.name = std::string(1100000, 'n'); d.insertToolpath(r);
        check("toolpath name 1.1 MB", d);
        Document d2; auto r2 = rec(one); r2.tool.name = std::string(1100000, 'n'); d2.insertToolpath(r2);
        check("tool name 1.1 MB", d2);
        Document d3; auto r3 = rec(one); r3.toolSource.toolId = std::string(1100000, 'n'); d3.insertToolpath(r3);
        check("toolSource id 1.1 MB", d3);
        Document d4; d4.layers().size(); Layer L; L.name = std::string(1100000, 'n'); d4.insertLayer(L);
        check("layer name 1.1 MB (pre-existing class)", d4);
    }
    if (want("dupsel")) {
        for (std::size_t n : {400000u, 1000000u, 13000000u}) {
            Document d; Layer L; L.name = "a"; const LayerId lid = d.insertLayer(L);
            ToolpathRecord r = rec(one); r.selection = {};
            r.selection.layers.assign(n, lid);
            bool ok = d.execute(std::make_unique<AddToolpathCommand>(r));
            std::printf("dup layer ids %zu accepted by AddToolpathCommand: %d, integrity issues %zu\n", n, ok, d.checkIntegrity().size());
            check(("selection " + std::to_string(n) + " dup layer ids").c_str(), d);
        }
        // distinct deleted ids
        Document d; std::vector<ObjectId> ids;
        { Layer m; m.id = LayerId{20000000}; d.insertLayer(m); d.takeLayer(m.id); }
        for (std::uint64_t i = 0; i < 13000000; ++i) ids.push_back(ObjectId{100 + i});
        ToolpathRecord r = rec(one); r.selection = {}; r.selection.objects = ids;
        bool ok = d.execute(std::make_unique<AddToolpathCommand>(r));
        std::printf("13M distinct unused object ids accepted: %d\n", ok);
        check("selection 13M distinct unused ids", d);
    }
    if (want("movebudget")) {
        // identical moves + padding: sweep around the threshold; any saved file must load
        int bad = 0, saved = 0, refused = 0;
        for (std::size_t n = 2000; n <= 400000; n = n * 11 / 10) {
            for (std::size_t pad : {0u, 1000u, 50000u}) {
                Document d; std::vector<Move> mv(n, Move::rapid({1, 2, 3})); d.insertToolpath(rec(mv));
                d.insertToolpath(rec(randomMoves(pad / 20 + 1, 3)));
                auto s = saveCarv(d);
                if (!s.ok()) { ++refused; continue; }
                ++saved;
                auto l = loadCarv(std::span<const std::uint8_t>(s.value()));
                if (!l.ok()) { ++bad; std::printf("  n=%zu pad=%zu size=%zu LOAD FAIL %s\n", n, pad, s.value().size(), l.error().message.c_str()); }
            }
        }
        std::printf("movebudget sweep: saved %d refused %d inconsistent %d\n", saved, refused, bad);
        // exact boundary search: one identical-move toolpath + random toolpath of fixed size
        const auto baseMoves = randomMoves(3000, 9);
        auto mk = [&](std::size_t k, Document& d) { d.insertToolpath(rec(baseMoves)); d.insertToolpath(rec(std::vector<Move>(k, Move::rapid({1, 2, 3})))); };
        std::size_t lo = 1, hi = 400000;
        while (lo + 1 < hi) {
            std::size_t mid = (lo + hi) / 2; Document d; mk(mid, d); 
            auto s = saveCarv(d); if (s.ok()) lo = mid; else hi = mid;
        }
        Document d; mk(lo, d);
        auto s = saveCarv(d);
        std::printf("largest saving count %zu: total moves %zu file %zu B\n", lo, lo + 3000, s.ok() ? s.value().size() : 0);
        check("boundary largest", d);
        Document d2; mk(lo + 1, d2);
        check("boundary largest+1", d2);
    }
    if (want("cap")) {
        // per-member cap: distinct arcs with feed override: 49 B each; 5.5M -> 270 MB > 256 MiB
        std::size_t n = 5'500'000; std::mt19937_64 g(5); std::uniform_real_distribution<double> u(-500, 500);
        std::vector<Move> v; v.reserve(n);
        for (std::size_t i = 0; i < n; ++i) { Move m = Move::arcCW({u(g), u(g), u(g)}, {u(g), u(g)}, u(g) + 1000); v.push_back(m); }
        Document d; d.insertToolpath(rec(std::move(v)));
        check("5.5M distinct arcs+feed (270 MB cache)", d);
    }
    if (want("zero")) {
        Document d; d.insertToolpath(rec({})); check("record with empty (non-null) moves", d);
        Document d2; d2.insertToolpath(rec({}, false)); check("record without moves", d2);
        Document d3; d3.insertToolpath(rec({Move::dwell({0, 0, 0}, -0.0)})); check("dwell -0.0", d3);
    }
    return 0;
}
