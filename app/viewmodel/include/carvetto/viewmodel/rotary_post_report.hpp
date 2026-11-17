#pragma once

#include <string>
#include <vector>

#include "carvetto/document/job.hpp"
#include "carvetto/viewmodel/save_toolpaths.hpp"

// The rotary refusals and warnings of the post (T-0320, ADR-0103) picked out of a `SaveOutput` and given
// a plain explanation, for the rotary post dialog (T-0334). Qt-free.

namespace carvetto::viewmodel {

struct RotaryPostIssue {
    /// True: the post refused to write (an error); false: a warning, the program is written.
    bool refusal{false};
    /// The post's own message, as reported.
    std::string message;
    /// What the user can do about it.
    std::string advice;
};

struct RotaryPostReport {
    std::vector<RotaryPostIssue> issues;
    bool refused() const;
    bool empty() const { return issues.empty(); }
};

/// The rotary-related lines of `output` (its error and warnings) for a rotary `job`; empty for any other job.
/// A line is rotary-related when it mentions the cylinder or a rotary axis.
RotaryPostReport rotaryPostReport(const SaveOutput& output, const document::Job& job);

}  // namespace carvetto::viewmodel
