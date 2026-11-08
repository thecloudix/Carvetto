#include "carvetto/viewmodel/toolpath_runner.hpp"

#include <exception>
#include <string>
#include <utility>

namespace carvetto::viewmodel {

ToolpathRunner::~ToolpathRunner() {
    cancel();
    if (worker_.joinable()) {
        worker_.join();
    }
}

bool ToolpathRunner::start(std::vector<ToolpathJob> jobs, std::function<void()> onFinished) {
    if (state_ != nullptr) {
        return false;
    }
    auto state = std::make_shared<State>();
    state->jobs = std::move(jobs);
    state->onFinished = std::move(onFinished);
    try {
        worker_ = std::thread([state]() {
            state->results.reserve(state->jobs.size());
            for (const ToolpathJob& job : state->jobs) {
                if (state->cancel.load()) {
                    ToolpathCalculation skipped;
                    skipped.cancelled = true;
                    skipped.definition = job.definition;
                    skipped.target = job.target;
                    state->results.push_back(std::move(skipped));
                    continue;
                }
                try {
                    state->results.push_back(job.run(&state->cancel));
                } catch (const std::exception& e) {
                    ToolpathCalculation failed;
                    failed.definition = job.definition;
                    failed.target = job.target;
                    ToolpathMessage m;
                    m.severity = MessageSeverity::Error;
                    m.text = std::string("calculation failed: ") + e.what();
                    failed.messages.push_back(std::move(m));
                    state->results.push_back(std::move(failed));
                }
            }
            state->done.store(true);
            if (state->onFinished) {
                state->onFinished();
            }
        });
    } catch (const std::exception&) {
        return false;
    }
    state_ = std::move(state);
    return true;
}

void ToolpathRunner::cancel() {
    if (state_ != nullptr) {
        state_->cancel.store(true);
    }
}

bool ToolpathRunner::finished() const {
    return state_ != nullptr && state_->done.load();
}

std::optional<std::vector<ToolpathCalculation>> ToolpathRunner::take() {
    if (state_ == nullptr) {
        return std::nullopt;
    }
    if (worker_.joinable()) {
        worker_.join();
    }
    std::vector<ToolpathCalculation> out = std::move(state_->results);
    state_.reset();
    return out;
}

}  // namespace carvetto::viewmodel
