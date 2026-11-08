#pragma once

#include <cstddef>
#include <memory>
#include <vector>

// Playback state of the simulation (T-0223): the scrubber position, run / pause / step and speed.
// Qt-free and clock-free: the owner calls `tick(realSeconds)` from a timer. The position is a move
// count (0 = nothing cut, N = everything), so scrubbing maps to `Simulator::seekMove`, which is
// byte-exact (ADR-0032 section 9); play advances a machine-time clock and converts it to a move
// count with the cumulative move end times.

namespace carvetto::viewmodel {

class PlaybackModel {
   public:
    /// Speeds as multiples of machine time (1 = real time).
    static constexpr double kSpeeds[] = {1.0, 2.0, 5.0, 10.0, 25.0, 50.0, 100.0, 250.0, 500.0};

    /// New job: `endTimes` = cumulative machine seconds at the end of each move (non-decreasing;
    /// may contain infinity when a rate is 0). Position 0, paused.
    void reset(std::shared_ptr<const std::vector<double>> endTimes);

    std::size_t moveCount() const { return endTimes_ ? endTimes_->size() : 0; }
    std::size_t position() const { return position_; }
    /// Machine time at the position (0 before the first move; infinity is reported as 0).
    double seconds() const;
    double totalSeconds() const;

    /// Jumps to move count `move` (clamped); stops playing when it reaches the end.
    void setPosition(std::size_t move);
    /// Position as a fraction of the machine time (0..1) / of the move count when time is unknown.
    void setFraction(double fraction);
    double fraction() const;
    void stepForward(std::size_t moves = 1);
    void stepBack(std::size_t moves = 1);

    bool playing() const { return playing_; }
    void play();  ///< at the end, restarts from 0
    void pause() { playing_ = false; }
    void toggle() { playing_ ? pause() : play(); }
    bool atEnd() const { return position_ >= moveCount(); }

    double speed() const { return speed_; }
    /// Clamped to [0.1, 1000].
    void setSpeed(double multiplier);

    /// Advances by `realSeconds` of wall time at the current speed while playing; returns true when
    /// the position changed. Reaching the end pauses.
    bool tick(double realSeconds);

   private:
    std::size_t positionForTime(double t) const;

    std::shared_ptr<const std::vector<double>> endTimes_;
    std::size_t position_{0};
    double clock_{0.0};
    bool playing_{false};
    double speed_{10.0};
};

}  // namespace carvetto::viewmodel
