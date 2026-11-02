// T-0136 verifier: edge probes for (b) (c) (f).
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>
#include <span>
#include <string>
#include <vector>
#include "carv_zip.hpp"
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/toolpath_record.hpp"
using namespace carvetto;
using namespace carvetto::document;
using toolpath::Move;
using Bytes = std::vector<std::uint8_t>;
static ToolpathRecord rec(bool computed = true) {
    ToolpathRecord r; r.name = "tp"; r.selection = VectorSelection::allVectors();
    r.tool.name = "tool"; r.tool.diameter = 6; r.tool.feedRate = 1000; r.tool.plungeRate = 300; r.tool.spindleSpeed = 18000;
    if (computed) r.moves = shareMoves({Move::rapid({0, 0, 5}), Move::feed({1, 1, -1})});
    return r;
}
static int inv = 0;
static void check(const char* what, const Document& d, const CarvExtras& ex = {}) {
    auto s = saveCarv(d, ex);
    if (!s.ok()) { std::printf("%-40s SAVE REFUSED [%d]: %.300s\n", what, int(s.error().code), s.error().message.c_str()); return; }
    auto l = loadCarv(std::span<const std::uint8_t>(s.value()));
    if (!l.ok()) { ++inv; std::printf("%-40s INVARIANT BROKEN: %.300s\n", what, l.error().message.c_str()); return; }
    bool eq = l.value().document.toolpaths() == d.toolpaths();
    auto s2 = saveCarv(l.value().document, l.value().extras);
    std::printf("%-40s saved+loaded eq=%d resave=%s\n", what, eq, s2.ok() ? (s2.value() == s.value() ? "identical" : "DIFFERENT") : s2.error().message.c_str());
}
int main() {
    const double nan = std::numeric_limits<double>::quiet_NaN(), inf = std::numeric_limits<double>::infinity();
    // (f) negative NaN, signalling NaN, denormals, max double.
    for (double v : {-nan, std::numeric_limits<double>::signaling_NaN(), std::numeric_limits<double>::denorm_min(), std::numeric_limits<double>::max(), -0.0}) {
        Document d; auto r = rec(); std::get<toolpath::ProfileParams>(r.params).safeZ = v; d.insertToolpath(r);
        char b[64]; std::snprintf(b, sizeof b, "profile safe_z=%g", v); check(b, d);
    }
    { Document d; auto r = rec(false); toolpath::PocketParams p; p.tolerance.angle = inf; r.params = p; d.insertToolpath(r); check("uncomputed pocket tol.angle inf", d); }
    { Document d; auto r = rec(); std::get<toolpath::ProfileParams>(r.params).cuttingData = tooling::CuttingData{}; std::get<toolpath::ProfileParams>(r.params).cuttingData->feedRate = nan;
      bool ok = d.execute(std::make_unique<AddToolpathCommand>(r)); std::printf("add with NaN cuttingData -> %d\n", ok); check("NaN cuttingData via command", d); }
    // (b)
    { Document d; d.insertToolpath(rec()); for (int i = 1; i < 10000; ++i) d.insertToolpath(rec(false)); d.insertToolpath(rec()); check("computed at 10001 (10001 records)", d); }
    { Document d; for (int i = 0; i < 9999; ++i) d.insertToolpath(rec(false)); d.insertToolpath(rec()); check("only computed at 10000", d); }
    { Document d; for (int i = 0; i < 9998; ++i) d.insertToolpath(rec()); check("9998 computed", d); }
    { Document d; for (int i = 0; i < 9998; ++i) d.insertToolpath(rec()); check("9998 computed + 1 extra", d, CarvExtras{}.withMember("a.txt", Bytes{'x'})); }
    { Document d; for (int i = 0; i < 9997; ++i) d.insertToolpath(rec()); check("9997 computed + 1 extra", d, CarvExtras{}.withMember("a.txt", Bytes{'x'})); }
    { Document d; for (int i = 0; i < 9997; ++i) d.insertToolpath(rec()); CarvExtras ex; for (int k = 0; k < 5; ++k) ex = ex.withMember("m" + std::to_string(k) + ".txt", Bytes{'x'}); check("9997 computed + 5 extras", d, ex); }
    { Document d; d.insertToolpath(rec()); CarvExtras ex; for (int k = 0; k < 9999; ++k) ex = ex.withMember("m" + std::to_string(k) + ".txt", Bytes{'x'}); check("1 computed + 9999 extras", d, ex); }
    { Document d; d.insertToolpath(rec(false)); CarvExtras ex; for (int k = 0; k < 9999; ++k) ex = ex.withMember("m" + std::to_string(k) + ".txt", Bytes{'x'}); check("0 computed + 9999 extras", d, ex); }
    // (c)
    for (std::size_t n : {std::size_t(1048576), std::size_t(1048577)}) {
        std::mt19937 g(1); std::string s(n, ' '); for (char& c : s) c = char('a' + g() % 26);
        Document d; auto r = rec(); r.name = s; d.insertToolpath(r); check(n == 1048576 ? "random name 1 MiB exact" : "random name 1 MiB + 1", d);
    }
    for (std::size_t n : {std::size_t(200000), std::size_t(260000), std::size_t(300000), std::size_t(500000), std::size_t(1048576)}) {
        Document d; auto r = rec(); r.name = std::string(n, 'a'); d.insertToolpath(r); char b[64]; std::snprintf(b, sizeof b, "repetitive name %zu", n); check(b, d);
    }
    { Document d; for (int i = 0; i < 20; ++i) { auto r = rec(); r.name = std::string(60000, 'a'); d.insertToolpath(r); } check("20 x 60k repetitive names", d); }
    { Document d; auto r = rec(); r.name = std::string(1100000, 'a'); r.toolSource.toolId = std::string(1200000, 'b'); d.insertToolpath(r); check("name 1.1M + toolId 1.2M", d); }
    { Document d; Layer l; l.name = std::string(800000, 'x'); d.insertLayer(l); auto r = rec(); r.name = std::string(1100000, 'q'); d.insertToolpath(r); check("layer 800k + tp name 1.1M", d); }
    { Document d; d.variables().set(std::string(70, 'v'), 1.0); auto r = rec(); r.name = std::string(1100000, 'z'); d.insertToolpath(r); check("var key 70 + name 1.1M", d); }
    std::printf("INVARIANT BROKEN: %d\n", inv);
}
