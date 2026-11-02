// T-0132 verifier round 3: adversarial save/load invariant probes via the raw API.
// For each case: save; if OK must load, toolpaths equal, resave identical. Refusal message printed.
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <limits>
#include <random>
#include <span>
#include <string>
#include <vector>
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/toolpath_record.hpp"
using namespace carvetto;
using namespace carvetto::document;
using carvetto::toolpath::Move;
using Bytes = std::vector<std::uint8_t>;
static int inconsistent = 0;
static double cpu() { timespec ts; clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts); return ts.tv_sec + ts.tv_nsec * 1e-9; }
static void check(const std::string& label, const Document& doc, const CarvExtras& ex = {}) {
    double t0 = cpu();
    auto s = saveCarv(doc, ex);
    double t1 = cpu();
    if (!s.ok()) { std::printf("%-52s REFUSED code=%d (%.2fs): %.220s\n", label.c_str(), int(s.error().code), t1 - t0, s.error().message.c_str()); return; }
    auto l = loadCarv(std::span<const std::uint8_t>(s.value()));
    if (!l.ok()) { ++inconsistent; std::printf("%-52s save OK / LOAD FAIL: %s  <== INCONSISTENT\n", label.c_str(), l.error().message.c_str()); return; }
    const Document& d2 = l.value().document;
    bool tpEq = d2.toolpaths() == doc.toolpaths();
    bool shape = d2.toolpaths().size() == doc.toolpaths().size() && d2.layers().size() == doc.layers().size() && d2.objects().size() == doc.objects().size();
    auto s2 = saveCarv(d2, l.value().extras);
    bool same = s2.ok() && s2.value() == s.value();
    if (!shape) ++inconsistent;
    std::printf("%-52s OK %zu B; tp %zu->%zu records-equal %d shape %d resave %s\n", label.c_str(), s.value().size(), doc.toolpaths().size(), d2.toolpaths().size(), tpEq, shape,
                s2.ok() ? (same ? "identical" : "DIFFERENT") : ("REFUSED " + s2.error().message.substr(0, 100)).c_str());
}
static ToolpathRecord rec(std::vector<Move> mv, bool withMoves = true) {
    ToolpathRecord r; r.name = "t"; r.selection = VectorSelection::allVectors();
    r.tool.name = "tool"; r.tool.diameter = 6; r.tool.feedRate = 1000; r.tool.plungeRate = 300; r.tool.spindleSpeed = 18000;
    if (withMoves) r.moves = shareMoves(std::move(mv));
    return r;
}
static const std::vector<Move> one{Move::rapid({1, 2, 30})};
int main(int argc, char** argv) {
    std::string only = argc > 1 ? argv[1] : "";
    auto want = [&](const char* k) { return only.empty() || only == k; };
    const double nan = std::numeric_limits<double>::quiet_NaN(), inf = std::numeric_limits<double>::infinity();
    if (want("nan")) {
        { Document d; auto r = rec(one); std::get<toolpath::ProfileParams>(r.params).allowance = nan; d.insertToolpath(r); check("profile allowance NaN", d); }
        { Document d; auto r = rec(one); std::get<toolpath::ProfileParams>(r.params).safeZ = inf; d.insertToolpath(r); check("profile safeZ +inf", d); }
        { Document d; auto r = rec(one); std::get<toolpath::ProfileParams>(r.params).safeZ = -0.0; d.insertToolpath(r); check("profile safeZ -0.0", d); }
        { Document d; auto r = rec(one); std::get<toolpath::ProfileParams>(r.params).safeZ = 1e308; d.insertToolpath(r); check("profile safeZ 1e308", d); }
        { Document d; auto r = rec(one); std::get<toolpath::ProfileParams>(r.params).safeZ = 4.9e-324; d.insertToolpath(r); check("profile safeZ denormal", d); }
        { Document d; auto r = rec(one); r.tool.diameter = nan; d.insertToolpath(r); check("tool diameter NaN", d); }
        { Document d; auto r = rec(one); r.tool.feedRate = -inf; d.insertToolpath(r); check("tool feed -inf", d); }
        { Document d; auto r = rec(one); r.tool.toolNumber = std::numeric_limits<int>::min(); d.insertToolpath(r); check("tool number INT_MIN", d); }
        { Document d; auto r = rec(one); toolpath::PocketParams p; p.safeZ = nan; r.params = p; d.insertToolpath(r); check("pocket safeZ NaN", d); }
        { Document d; auto r = rec({Move::feed({nan, 0, 0})}); d.insertToolpath(r); check("move x NaN", d); }
        { Document d; auto r = rec({Move::feed({1e308, -1e308, 4.9e-324})}); d.insertToolpath(r); check("move extremes", d); }
    }
    if (want("strings")) {
        { Document d; auto r = rec(one); r.name = "\xff\xfe bad utf8"; d.insertToolpath(r); check("name invalid UTF-8", d); }
        { Document d; auto r = rec(one); r.name = std::string("a\0b", 3); d.insertToolpath(r); check("name embedded NUL", d); }
        { Document d; auto r = rec(one); r.name = "\x01\x1b[31m\n\t\"\\"; d.insertToolpath(r); check("name control chars", d); }
        { Document d; auto r = rec(one); r.name = ""; d.insertToolpath(r); check("name empty", d); }
        { Document d; auto r = rec(one); r.toolSource.machine = "m"; d.insertToolpath(r); check("toolSource machine w/o material", d); }
        { Document d; auto r = rec(one); r.name = std::string((1u << 20), 'q'); d.insertToolpath(r); check("name exactly 1 MiB", d); }
        { Document d; auto r = rec(one); r.name = std::string((1u << 20) + 1, 'q'); d.insertToolpath(r); check("name 1 MiB + 1", d); }
        { Document d; bool ok = d.variables().set("", 1.0); std::printf("set empty var: %d\n", ok); check("variable empty name", d); }
        { Document d; d.variables().set(std::string(2u << 20, (char)118), nan); check("variable 2MiB name NaN value", d); }
    }
    if (want("ids")) {
        { Document d; auto r = rec(one); r.id = ToolpathId{IdGenerator::kMaxId - 1}; d.insertToolpath(r); check("tp id kMaxId-1 (generator exhausted)", d); }
        { Document d; auto r = rec(one); r.id = ToolpathId{IdGenerator::kMaxId + 5}; d.insertToolpath(r); check("tp id kMaxId+5", d); }
        { Document d; auto r = rec(one); r.id = ToolpathId{d.defaultLayer().value}; d.insertToolpath(r); check("tp id == default layer id", d); }
        { Document d; auto a = rec(one); ToolpathId t = d.insertToolpath(a); auto b = rec(one); b.selection = {}; b.selection.objects = {ObjectId{t.value}}; d.insertToolpath(b); check("selection object = toolpath id", d); }
        { Document d; auto r = rec(one); r.selection = {}; r.selection.layers = {LayerId{0}}; d.insertToolpath(r); check("selection layer 0", d); }
        { Document d; auto r = rec(one); r.selection.layers = {d.defaultLayer()}; d.insertToolpath(r); check("all + layers", d); }
        { Document d; auto r = rec(one); r.selection = {}; r.selection.layers = {d.defaultLayer()}; d.insertToolpath(r);
          d.takeLayer(d.defaultLayer()); Layer L; L.name="n"; d.insertLayer(L); check("selection of deleted layer", d); }
    }
    if (want("positions")) {
        { Document d; for (int i = 0; i < 9998; ++i) d.insertToolpath(rec({}, false)); d.insertToolpath(rec(one)); check("computed at position 9999 only", d); }
        { Document d; for (int i = 0; i < 9999; ++i) d.insertToolpath(rec({}, false)); d.insertToolpath(rec(one)); check("computed at position 10000 only", d); }
        { Document d; for (int i = 0; i < 10000; ++i) d.insertToolpath(i % 2 ? rec(one) : rec({}, false)); check("10000 alternating computed", d); }
        { Document d; d.insertToolpath(rec(one)); CarvExtras e; for (int i = 0; i < 9997; ++i) e = e.withMember("x/" + std::to_string(i) + ".bin", Bytes(1, 1)); check("1 computed + 9997 extras (10000 members)", d, e);
          e = e.withMember("x/last.bin", Bytes(1, 1)); check("1 computed + 9998 extras (10001 members)", d, e); }
    }
    if (want("names")) {
        for (const char* n : {"toolpaths/0001.JSON", "toolpaths/1.json", "toolpaths/00001.json", "toolpaths/0000.json", "toolpaths/0000.bin", "toolpaths/abcd.bin",
                              "toolpaths/0001.bin.x", "toolpaths/0001.json/x", "Toolpaths/0001.json", "toolpaths//0001.json", "toolpaths/10000.bin", "toolpaths/-001.json", "toolpaths/0001.jsonx"}) {
            try {
                CarvExtras e = CarvExtras{}.withMember(n, std::string(n).ends_with(".json") || std::string(n).ends_with(".JSON") ? Bytes{'{', '}'} : Bytes{1, 2, 3});
                Document d; d.insertToolpath(rec(one)); check(std::string("extra member '") + n + "'", d, e);
            } catch (const std::exception& ex) { std::printf("%-52s withMember threw: %.120s\n", (std::string("extra member '") + n + "'").c_str(), ex.what()); }
        }
    }
    if (want("bigsel")) {
        Document d; { Layer m; m.id = LayerId{20000000}; d.insertLayer(m); d.takeLayer(m.id); }
        ToolpathRecord r = rec(one); r.selection = {}; for (std::uint64_t i = 0; i < 13000000; ++i) r.selection.objects.push_back(ObjectId{100 + i});
        d.insertToolpath(r); check("selection 13M distinct unused ids", d);
    }
    if (want("layers")) {
        Document d; for (int i = 0; i < 100000; ++i) { Layer L; L.name = "L" + std::to_string(i); d.insertLayer(L); }
        ToolpathRecord r = rec(one); r.selection = {}; for (auto& L : d.layers()) r.selection.layers.push_back(L.id);
        d.execute(std::make_unique<AddToolpathCommand>(r)); check("100k layers all selected", d);
    }
    std::printf("INCONSISTENT: %d\n", inconsistent);
    return inconsistent;
}
