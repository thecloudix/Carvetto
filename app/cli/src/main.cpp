// carvetto-cli entry point (T-0122). All logic lives in cli.cpp so tests can drive it.
#include <iostream>
#include <string>
#include <vector>

#include "cli.hpp"

int main(int argc, char** argv) {
    std::vector<std::string> args;
    for (int i = 1; i < argc; ++i) {
        args.emplace_back(argv[i]);
    }
    return carvetto::cli::run(args, std::cout, std::cerr);
}
