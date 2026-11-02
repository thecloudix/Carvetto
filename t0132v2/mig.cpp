// T-0132 verifier round 2: migrated (main-CLI interim) records vs native (branch CLI) records.
#include <cstdio>
#include <span>
#include <variant>
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
using namespace carvetto::document;
int main(int, char** argv) {
    auto a = loadCarv(std::filesystem::path(argv[1])); auto b = loadCarv(std::filesystem::path(argv[2]));
    if (!a.ok() || !b.ok()) { std::printf("load failed\n"); return 1; }
    const auto& ta = a.value().document.toolpaths(); const auto& tb = b.value().document.toolpaths();
    std::printf("migrated %zu native %zu; extras migrated empty=%d\n", ta.size(), tb.size(), a.value().extras.empty());
    int bad = 0;
    for (std::size_t i = 0; i < ta.size() && i < tb.size(); ++i) {
        const auto& x = ta[i]; const auto& y = tb[i];
        std::printf("  %zu: id %llu/%llu name %d params %d sel %d (layers:", i, (unsigned long long)x.id.value, (unsigned long long)y.id.value, x.name == y.name, x.params == y.params, x.selection == y.selection);
        for (auto l : x.selection.layers) std::printf(" %llu", (unsigned long long)l.value);
        std::printf(" /");
        for (auto l : y.selection.layers) std::printf(" %llu", (unsigned long long)l.value);
        std::printf(") all %d/%d tool %d src %d moves %d  whole==%d\n", x.selection.all, y.selection.all, x.tool == y.tool, x.toolSource == y.toolSource, *x.moves == *y.moves, x == y);
        bad += !(x == y);
    }
    // normalisation: migrated doc saved and reloaded equals itself; and native file == migrated resave
    for (auto* L : {&a.value(), &b.value()}) {
        auto s = saveCarv(L->document, L->extras); auto r = loadCarv(std::span<const std::uint8_t>(s.value()));
        bool eq = r.ok() && r.value().document.toolpaths() == L->document.toolpaths();
        auto s2 = saveCarv(r.value().document, r.value().extras);
        std::printf("save/load records equal: %d, resave byte-identical: %d, size %zu\n", eq, s2.ok() && s2.value() == s.value(), s.value().size());
    }
    auto sa = saveCarv(a.value().document, a.value().extras); auto sb = saveCarv(b.value().document, b.value().extras);
    std::printf("migrated-resave bytes == native-file-resave bytes: %d\n", sa.value() == sb.value());
    return bad;
}
