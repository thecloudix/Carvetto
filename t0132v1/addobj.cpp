#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour.hpp"
#include <cstdio>
#include <span>
using namespace carvetto; using namespace carvetto::document;
int main() { Document d; Object o = Object::vector(geometry::Contour::rectangle(geometry::Box2{{0,0},{1,1}}), LayerId{777}, d.activeSheet());
 bool ok = d.execute(std::make_unique<AddObjectCommand>(o)); auto s = saveCarv(d); auto l = loadCarv(std::span<const std::uint8_t>(s.value()));
 std::printf("AddObject with missing layer: executed=%d issues=%zu save=%d load=%s\n", ok, d.checkIntegrity().size(), s.ok(), l.ok() ? "ok" : l.error().message.c_str()); }
