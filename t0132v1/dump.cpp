#include "carv_zip.hpp"
#include <fstream>
#include <iostream>
#include <iterator>
int main(int, char** argv) {
    std::ifstream f(argv[1], std::ios::binary);
    std::vector<std::uint8_t> b((std::istreambuf_iterator<char>(f)), {});
    auto r = carvetto::document::detail::readZip(b);
    if (!r.ok) { std::cerr << r.error << "\n"; return 1; }
    for (auto& m : r.members) {
        std::cout << "== " << m.name << " (" << m.data.size() << " B)\n";
        if (m.name.size() > 5 && m.name.substr(m.name.size() - 5) == ".json") std::cout << std::string(m.data.begin(), m.data.end()) << "\n";
    }
}
