#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
#include <iostream>
using namespace carvetto::document;
int main(int argc, char** argv) {
    auto a = loadCarv(std::filesystem::path(argv[1]));
    if (!a.ok()) { std::cout << "LOADERR " << int(a.error().code) << " " << a.error().message << "\n"; return 2; }
    auto s = saveCarv(a.value().document, std::filesystem::path(argv[2]), a.value().extras);
    if (!s.ok()) { std::cout << "SAVEERR " << s.error().message << "\n"; return 3; }
    auto b = loadCarv(std::filesystem::path(argv[2]));
    if (!b.ok()) { std::cout << "RELOADERR " << b.error().message << "\n"; return 4; }
    bool eq = a.value().document.toolpaths() == b.value().document.toolpaths();
    auto s2 = saveCarv(b.value().document, b.value().extras);
    auto s1 = saveCarv(a.value().document, a.value().extras);
    std::cout << "OK records_equal=" << eq << " resave_identical=" << (s1.value() == s2.value()) << " n=" << a.value().document.toolpaths().size() << "\n";
    return eq ? 0 : 5;
}
