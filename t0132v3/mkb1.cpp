#include <fstream>
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
using namespace carvetto::document;
int main() { Document doc; ToolpathRecord r; r.name = "tp0"; r.selection = VectorSelection::allVectors(); r.moves = shareMoves({carvetto::toolpath::Move::rapid({0, 0, 5})}); doc.insertToolpath(r);
  auto s = saveCarv(doc).value(); std::ofstream o("b1.carv", std::ios::binary); o.write((const char*)s.data(), (std::streamsize)s.size()); }
