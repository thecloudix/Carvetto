#include "carvetto/geometry/cancel.hpp"

namespace carvetto::geometry {

namespace {
thread_local const std::atomic<bool>* tlFlag = nullptr;
}

CancelScope::CancelScope(const std::atomic<bool>* flag) : previous_(tlFlag) {
    if (flag != nullptr) {
        tlFlag = flag;
    }
}

CancelScope::~CancelScope() {
    tlFlag = previous_;
}

bool cancelRequested() {
    return tlFlag != nullptr && tlFlag->load(std::memory_order_relaxed);
}

}  // namespace carvetto::geometry
