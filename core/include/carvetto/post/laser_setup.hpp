#pragma once

#include <string>

#include "carvetto/post/post_processor.hpp"
#include "carvetto/tooling/machine.hpp"

// Whether a machine and a post fit together for laser output (T-0325, ADR-0107).

namespace carvetto::post {

/// Empty when a laser job may go to `post` on `machine`; otherwise a sentence saying what is missing. The post must
/// declare LASER_SUPPORT = "YES" and have the JET_TOOL_ON and JET_TOOL_OFF blocks; the machine, when given, must be
/// flagged `laser`. (A cutting-only job needs neither: this is asked for jobs with a laser toolpath.)
std::string laserSetupProblem(const tooling::Machine* machine, const PostProcessor& post);

}  // namespace carvetto::post
