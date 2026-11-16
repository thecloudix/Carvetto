#include "carvetto/post/laser_setup.hpp"

namespace carvetto::post {

std::string laserSetupProblem(const tooling::Machine* machine, const PostProcessor& post) {
    if (machine != nullptr && !machine->laser) {
        return "the machine \"" + machine->name + "\" is not marked as a laser machine";
    }
    if (!post.settings().laserSupport) {
        return "the post does not declare LASER_SUPPORT = \"YES\"";
    }
    for (const char* name : {"JET_TOOL_ON", "JET_TOOL_OFF"}) {
        if (!post.hasBlock(name)) {
            return std::string("the post has no ") + name + " block";
        }
    }
    return {};
}

}  // namespace carvetto::post
