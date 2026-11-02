#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
using namespace carvetto::document; using carvetto::toolpath::Move;
int main(int, char** argv){ Document doc; ToolpathRecord r; r.name="tp"; r.selection=VectorSelection::allVectors(); r.moves=shareMoves({Move::rapid({0,0,5})}); doc.insertToolpath(r); return saveCarv(doc, std::filesystem::path(argv[1])).ok()?0:1; }
