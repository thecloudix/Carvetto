#pragma once

#include <ostream>
#include <string>
#include <vector>

// carvetto-cli (T-0122): the headless command-line front end. Everything is in this library so
// tests drive it in-process; main.cpp only forwards argv.

namespace carvetto::cli {

/// Process exit codes.
enum ExitCode : int {
    kExitOk = 0,       ///< The command succeeded.
    kExitFailure = 1,  ///< The command ran but failed: unreadable or invalid input, nothing produced.
    kExitUsage = 2,    ///< The command line itself is wrong: unknown command or option, missing arguments.
};

/// Runs one command. `args` are the arguments after the program name. Normal output goes to
/// `out`, warnings and errors to `err`. Never throws; returns an `ExitCode`.
int run(const std::vector<std::string>& args, std::ostream& out, std::ostream& err);

/// The top-level help text (`carvetto-cli --help`), documenting every command.
std::string helpText();

}  // namespace carvetto::cli
