#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour.hpp"
using namespace carvetto; using namespace carvetto::document; using carvetto::toolpath::Move;
int main(int, char** argv){ Document doc;
 doc.insertObject(Object::vector(geometry::Contour::rectangle(geometry::Box2{{0,0},{1,1}}), doc.defaultLayer(), doc.activeSheet()));
 ToolpathRecord r; r.name="tp"; r.selection=VectorSelection::allVectors(); r.moves=shareMoves({Move::rapid({0,0,5})}); doc.insertToolpath(r); return saveCarv(doc, std::filesystem::path(argv[1])).ok()?0:1; }
