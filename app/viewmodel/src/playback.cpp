#include "carvetto/viewmodel/playback.hpp"

#include <algorithm>
#include <cmath>

namespace carvetto::viewmodel {

void PlaybackModel::reset(std::shared_ptr<const std::vector<double>> endTimes) {
    endTimes_ = std::move(endTimes);
    position_ = 0;
    clock_ = 0.0;
    playing_ = false;
}

double PlaybackModel::totalSeconds() const {
    if (!endTimes_ || endTimes_->empty()) {
        return 0.0;
    }
    const double t = endTimes_->back();
    return std::isfinite(t) ? t : 0.0;
}

double PlaybackModel::seconds() const {
    if (!endTimes_ || position_ == 0) {
        return 0.0;
    }
    const double t = (*endTimes_)[std::min(position_, endTimes_->size()) - 1];
    return std::isfinite(t) ? t : 0.0;
}

void PlaybackModel::setPosition(std::size_t move) {
    position_ = std::min(move, moveCount());
    clock_ = seconds();
    if (atEnd()) {
        playing_ = false;
    }
}

double PlaybackModel::fraction() const {
    const std::size_t n = moveCount();
    if (n == 0) {
        return 0.0;
    }
    const double total = totalSeconds();
    if (total > 0.0) {
        return std::clamp(seconds() / total, 0.0, 1.0);
    }
    return static_cast<double>(position_) / static_cast<double>(n);
}

void PlaybackModel::setFraction(double fraction) {
    const std::size_t n = moveCount();
    if (n == 0 || !std::isfinite(fraction)) {
        return;
    }
    fraction = std::clamp(fraction, 0.0, 1.0);
    const double total = totalSeconds();
    if (total > 0.0) {
        setPosition(positionForTime(fraction * total));
    } else {
        setPosition(static_cast<std::size_t>(std::llround(fraction * static_cast<double>(n))));
    }
}

void PlaybackModel::stepForward(std::size_t moves) {
    setPosition(position_ + std::min(moves, moveCount() - position_));
}

void PlaybackModel::stepBack(std::size_t moves) {
    setPosition(position_ - std::min(moves, position_));
}

void PlaybackModel::play() {
    if (moveCount() == 0) {
        return;
    }
    if (atEnd()) {
        setPosition(0);
    }
    playing_ = true;
}

void PlaybackModel::setSpeed(double multiplier) {
    if (std::isfinite(multiplier)) {
        speed_ = std::clamp(multiplier, 0.1, 1000.0);
    }
}

std::size_t PlaybackModel::positionForTime(double t) const {
    if (!endTimes_) {
        return 0;
    }
    // Number of moves that are finished by time t (infinite end times never finish).
    const auto it = std::upper_bound(endTimes_->begin(), endTimes_->end(), t);
    return static_cast<std::size_t>(it - endTimes_->begin());
}

bool PlaybackModel::tick(double realSeconds) {
    if (!playing_ || !(realSeconds > 0.0) || !std::isfinite(realSeconds) || moveCount() == 0) {
        return false;
    }
    const std::size_t before = position_;
    if (totalSeconds() > 0.0) {
        clock_ += realSeconds * speed_;
        const std::size_t next = positionForTime(clock_);
        position_ = std::max(position_, std::min(next, moveCount()));
    } else {
        // No usable times (rates of 0): 50 moves per second at 1x.
        const double moves = std::max(1.0, std::floor(realSeconds * speed_ * 50.0));
        position_ = std::min(moveCount(), position_ + static_cast<std::size_t>(std::min(moves, 1e9)));
    }
    if (atEnd()) {
        playing_ = false;
    }
    return position_ != before;
}

}  // namespace carvetto::viewmodel
