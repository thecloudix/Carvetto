// T-0132 verifier round 3: SetToolpathCommand dedupe + undo/redo; integrity for repeated object ids.
#include <cstdio>
#include <memory>
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
using namespace carvetto; using namespace carvetto::document; using carvetto::toolpath::Move;
int main() {
    Document d; Layer a; a.name = "a"; LayerId la = d.insertLayer(a); Layer b; b.name = "b"; LayerId lb = d.insertLayer(b);
    ToolpathRecord r; r.name = "t"; r.selection = {}; r.selection.layers = {la}; r.moves = shareMoves({Move::rapid({0,0,5})});
    d.execute(std::make_unique<AddToolpathCommand>(r)); ToolpathRecord cur = d.toolpaths()[0];
    ToolpathRecord e = cur; e.selection.layers = {lb, la, lb, la, lb};
    bool ok = d.execute(std::make_unique<SetToolpathCommand>(e));
    auto& L = d.toolpaths()[0].selection.layers;
    std::printf("set ok %d stored %zu ids [%llu %llu] integrity %zu\n", ok, L.size(), (unsigned long long)L[0].value, (unsigned long long)(L.size()>1?L[1].value:0), d.checkIntegrity().size());
    d.undo(); std::printf("undo -> %zu ids (%d)\n", d.toolpaths()[0].selection.layers.size(), d.toolpaths()[0] == cur);
    d.redo(); std::printf("redo -> %zu ids\n", d.toolpaths()[0].selection.layers.size());
    ToolpathRecord same = d.toolpaths()[0]; same.selection.layers.push_back(lb);
    std::printf("set differing only by a repeat: %d (expect 0, no-op)\n", d.execute(std::make_unique<SetToolpathCommand>(same)));
    ToolpathRecord raw = cur; raw.id = {}; raw.selection = {}; ObjectId o{999}; (void)o;
    Object v = Object::vector(geometry::Contour::rectangle(geometry::Box2{{0,0},{1,1}}), d.defaultLayer(), d.activeSheet()); ObjectId oid = d.insertObject(v);
    raw.selection.objects = {oid, oid, oid}; d.insertToolpath(raw);
    auto is = d.checkIntegrity(); std::printf("raw repeated object: %zu issue(s): %s\n", is.size(), is.empty() ? "" : is[0].message.c_str());
}
