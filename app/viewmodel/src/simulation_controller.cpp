#include "carvetto/viewmodel/simulation_controller.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <new>

namespace carvetto::viewmodel {

double SimProgress::fraction() const {
    if (state == SimState::Ready || state == SimState::Idle) {
        return 1.0;
    }
    if (target <= runBase) {
        return 1.0;
    }
    const std::size_t done = std::clamp(movesDone, runBase, target);
    return static_cast<double>(done - runBase) / static_cast<double>(target - runBase);
}

struct SimulationShared {
    std::mutex m;
    std::condition_variable cv;
    SimulationSetup setup;
    ReliefOptions relief;
    bool reliefDirty{false};
    std::size_t target{0};
    bool targetPending{true};
    bool quit{false};
    std::atomic<bool> cancelRun{false};
    std::atomic<int> chunkDelayMs{0};

    SimState state{SimState::Preparing};
    std::size_t movesDone{0};
    std::size_t runBase{0};
    std::size_t moveCount{0};
    bool cancelled{false};
    std::string error;
    std::shared_ptr<const SimulationMeta> meta;
    std::shared_ptr<const SimulationFrame> frame;
    std::uint64_t serial{0};
};

namespace {

using Clock = std::chrono::steady_clock;

double secondsSince(Clock::time_point t0) {
    return std::chrono::duration<double>(Clock::now() - t0).count();
}

void fail(SimulationShared& sh, std::string message) {
    std::lock_guard<std::mutex> lock(sh.m);
    sh.state = SimState::Failed;
    sh.error = std::move(message);
}

void worker(std::shared_ptr<SimulationShared> shp) {
    SimulationShared& sh = *shp;
    try {
        simulate::Simulator sim(sh.setup.stock, sh.setup.options);
        if (!sim.ok()) {
            fail(sh, sim.error().message);
            return;
        }
        for (std::size_t i = 0; i < sh.setup.toolpaths.size(); ++i) {
            const SimulationToolpath& tp = sh.setup.toolpaths[i];
            {
                std::lock_guard<std::mutex> lock(sh.m);
                if (sh.quit) {
                    return;
                }
            }
            toolpath::Toolpath path;
            path.name = tp.name;
            path.tool = tp.tool;
            path.moves = *tp.moves;  // transient copy: the simulator keeps its own queue
            if (!sim.addToolpath(path, tp.shape, static_cast<std::uint32_t>(i + 1))) {
                fail(sh, sim.error().message.empty() ? std::string("The simulator refused toolpath \"") + tp.name + "\"."
                                                      : sim.error().message);
                return;
            }
        }
        {
            auto meta = std::make_shared<SimulationMeta>();
            meta->moveCount = sim.moveCount();
            meta->nx = sim.heightfield().nx();
            meta->ny = sim.heightfield().ny();
            meta->cellSize = sim.heightfield().cellSize();
            meta->totalSeconds = sim.totalSeconds();
            meta->heightfieldBytes = sim.heightfield().bytes();
            meta->moveEndTimes = std::make_shared<const std::vector<double>>(sim.moveEndTimes());
            std::lock_guard<std::mutex> lock(sh.m);
            sh.meta = std::move(meta);
            sh.moveCount = sim.moveCount();
            sh.state = SimState::Working;
        }

        std::size_t chunk = 256;
        for (;;) {
            std::size_t target = 0;
            ReliefOptions relief;
            {
                std::unique_lock<std::mutex> lock(sh.m);
                sh.cv.wait(lock, [&] { return sh.quit || sh.targetPending || sh.reliefDirty; });
                if (sh.quit) {
                    return;
                }
                target = std::min(sh.target, sim.moveCount());
                sh.targetPending = false;
                sh.reliefDirty = false;
                relief = sh.relief;
                sh.cancelRun.store(false);
                sh.cancelled = false;
                if (target != sim.movesDone()) {
                    sh.state = SimState::Working;
                }
            }
            const auto t0 = Clock::now();
            if (target < sim.movesDone()) {
                sim.reset();
                std::lock_guard<std::mutex> lock(sh.m);
                sh.movesDone = 0;
            }
            {
                std::lock_guard<std::mutex> lock(sh.m);
                sh.runBase = sim.movesDone();
            }
            bool superseded = false;
            bool aborted = false;
            while (sim.movesDone() < target) {
                const auto c0 = Clock::now();
                const std::size_t step = std::min(target, sim.movesDone() + chunk);
                if (!sim.advanceToMove(step)) {
                    fail(sh, sim.error().message.empty() ? std::string("The simulation failed.") : sim.error().message);
                    return;
                }
                const double took = secondsSince(c0);
                if (took < 0.02) {
                    chunk = std::min<std::size_t>(chunk * 2, 1 << 16);
                } else if (took > 0.08) {
                    chunk = std::max<std::size_t>(chunk / 2, 1);
                }
                const int delay = sh.chunkDelayMs.load();
                if (delay > 0) {
                    chunk = std::min<std::size_t>(chunk, 16);  // test hook: many short, slow chunks
                    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
                }
                std::lock_guard<std::mutex> lock(sh.m);
                sh.movesDone = sim.movesDone();
                if (sh.quit) {
                    return;
                }
                if (sh.targetPending) {
                    superseded = true;
                    break;
                }
                if (sh.cancelRun.load()) {
                    aborted = true;
                    sh.cancelled = true;
                    sh.target = sim.movesDone();
                    break;
                }
            }
            if (superseded) {
                continue;  // the newer request is taken at the top of the loop
            }
            (void)aborted;
            auto frame = std::make_shared<SimulationFrame>();
            frame->move = sim.movesDone();
            frame->seconds = sim.currentSeconds();
            frame->cutCells = sim.heightfield().cutCells();
            frame->relief = std::make_shared<const Raster>(renderRelief(sim.heightfield(), relief));
            frame->diagnostics = sim.diagnostics();
            frame->droppedDiagnostics = sim.droppedDiagnostics();
            frame->workSeconds = secondsSince(t0);
            for (const SimulationToolpath& tp : sh.setup.toolpaths) {
                frame->toolpathIds.push_back(tp.id.value);
            }
            std::lock_guard<std::mutex> lock(sh.m);
            frame->serial = ++sh.serial;
            sh.frame = std::move(frame);
            sh.movesDone = sim.movesDone();
            if (!sh.targetPending && !sh.reliefDirty) {
                sh.state = SimState::Ready;
            }
        }
    } catch (const std::bad_alloc&) {
        fail(sh, "Out of memory while simulating. Choose a lower quality.");
    } catch (const std::exception& e) {
        fail(sh, std::string("The simulation failed: ") + e.what());
    } catch (...) {
        fail(sh, "The simulation failed.");
    }
}

}  // namespace

SimulationController::~SimulationController() {
    stop();
}

bool SimulationController::start(SimulationSetup setup, const ReliefOptions& options) {
    stop();
    if (!setup.ok) {
        return false;
    }
    setup_ = setup;
    auto sh = std::make_shared<SimulationShared>();
    sh->setup = std::move(setup);
    sh->relief = options;
    shared_ = sh;
    worker_ = std::thread(worker, sh);
    return true;
}

void SimulationController::stop() {
    if (shared_) {
        {
            std::lock_guard<std::mutex> lock(shared_->m);
            shared_->quit = true;
            shared_->cancelRun.store(true);
        }
        shared_->cv.notify_all();
    }
    if (worker_.joinable()) {
        worker_.join();
    }
    shared_.reset();
    setup_ = SimulationSetup{};
}

void SimulationController::seek(std::size_t move) {
    if (!shared_) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(shared_->m);
        if (shared_->state == SimState::Failed) {
            return;
        }
        shared_->target = move;
        shared_->targetPending = true;
        if (shared_->state == SimState::Ready) {
            shared_->state = SimState::Working;
        }
    }
    shared_->cv.notify_all();
}

void SimulationController::setReliefOptions(const ReliefOptions& options) {
    if (!shared_) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(shared_->m);
        shared_->relief = options;
        shared_->reliefDirty = true;
        if (shared_->state == SimState::Ready) {
            shared_->state = SimState::Working;
        }
    }
    shared_->cv.notify_all();
}

void SimulationController::cancel() {
    if (!shared_) {
        return;
    }
    std::lock_guard<std::mutex> lock(shared_->m);
    if (shared_->state == SimState::Working || shared_->state == SimState::Preparing) {
        shared_->cancelRun.store(true);
    }
}

SimProgress SimulationController::progress() const {
    SimProgress p;
    if (!shared_) {
        return p;
    }
    std::lock_guard<std::mutex> lock(shared_->m);
    p.state = shared_->state;
    p.movesDone = shared_->movesDone;
    p.runBase = shared_->runBase;
    p.target = shared_->target;
    p.moveCount = shared_->moveCount;
    p.cancelled = shared_->cancelled;
    if (shared_->meta) {
        p.target = std::min(p.target, shared_->moveCount);
    }
    return p;
}

std::string SimulationController::error() const {
    if (!shared_) {
        return {};
    }
    std::lock_guard<std::mutex> lock(shared_->m);
    return shared_->error;
}

std::shared_ptr<const SimulationMeta> SimulationController::meta() const {
    if (!shared_) {
        return nullptr;
    }
    std::lock_guard<std::mutex> lock(shared_->m);
    return shared_->meta;
}

std::shared_ptr<const SimulationFrame> SimulationController::frame() const {
    if (!shared_) {
        return nullptr;
    }
    std::lock_guard<std::mutex> lock(shared_->m);
    return shared_->frame;
}

void SimulationController::setChunkDelayForTest(int milliseconds) {
    if (shared_) {
        shared_->chunkDelayMs.store(milliseconds);
    }
}

}  // namespace carvetto::viewmodel
