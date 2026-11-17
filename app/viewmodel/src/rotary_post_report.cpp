#include "carvetto/viewmodel/rotary_post_report.hpp"

#include <string_view>

namespace carvetto::viewmodel {

namespace {

bool has(const std::string& text, std::string_view part) {
    return text.find(part) != std::string::npos;
}

bool rotaryRelated(const std::string& text) {
    return has(text, "cylinder") || has(text, "rotary");
}

std::string adviceFor(const std::string& text) {
    if (has(text, "does not wrap")) {
        return "Choose a post that wraps the circumference onto a rotary axis (a ROTARY_WRAP_X or ROTARY_WRAP_Y "
               "setting), or the program moves the circumference as a flat distance.";
    }
    if (has(text, "lies along") || has(text, "must wrap")) {
        return "Pick a post whose wrapped axis is the circumference, or turn the job's rotary axis in Job Setup.";
    }
    if (has(text, "at or below the cylinder axis")) {
        return "A wrapped program cannot cut past the cylinder axis: reduce the cut depth or the model height, "
               "or recalculate the toolpath.";
    }
    if (has(text, "wider than the cylinder radius")) {
        return "Use a smaller tool, or a bigger cylinder: a flat toolpath wrapped on a small cylinder is cut "
               "inaccurately.";
    }
    if (has(text, "more than once round")) {
        return "Split the toolpath so each one goes at most once round, or check the vectors fit the circumference.";
    }
    if (has(text, "no rotary diameter") || has(text, "rotary diameter")) {
        return "Set the cylinder diameter in Job Setup or Material Setup.";
    }
    if (has(text, "rotary axis must be")) {
        return "Set the rotary axis to X or Y in Job Setup.";
    }
    return "Check the job's rotary settings and the post.";
}

}  // namespace

bool RotaryPostReport::refused() const {
    for (const RotaryPostIssue& i : issues) {
        if (i.refusal) {
            return true;
        }
    }
    return false;
}

RotaryPostReport rotaryPostReport(const SaveOutput& output, const document::Job& job) {
    RotaryPostReport report;
    if (job.type != document::JobType::Rotary) {
        return report;
    }
    if (!output.ok && rotaryRelated(output.error)) {
        report.issues.push_back({true, output.error, adviceFor(output.error)});
    }
    for (const std::string& w : output.warnings) {
        if (rotaryRelated(w)) {
            report.issues.push_back({false, w, adviceFor(w)});
        }
    }
    return report;
}

}  // namespace carvetto::viewmodel
