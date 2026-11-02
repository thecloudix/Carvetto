#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
#include <cstdio>
#include <limits>
#include <span>
using namespace carvetto::document; using carvetto::toolpath::Move;
int main(int, char** argv) { Document d3; ToolpathRecord t; t.name = "x\n\x01\xff\xfe"; std::get<carvetto::toolpath::ProfileParams>(t.params).safeZ = std::numeric_limits<double>::quiet_NaN(); t.tool.diameter = std::numeric_limits<double>::infinity();
 d3.insertToolpath(t); auto s = saveCarv(d3, std::filesystem::path(argv[1])); auto l = loadCarv(std::filesystem::path(argv[1]));
 if (!l.ok()) { std::printf("load: %s\n", l.ok() ? "ok" : l.error().message.c_str()); return 0; }
 auto& r = l.value().document.toolpaths()[0]; std::printf("loaded safeZ=%g diameter=%g name=%zu bytes\n", r.safeZ(), r.tool.diameter, r.name.size()); }
