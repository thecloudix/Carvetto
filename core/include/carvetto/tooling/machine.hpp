#pragma once

#include <string>
#include <vector>

namespace carvetto::tooling {

/// A stock material. Cutting data is keyed by (machine id, material id).
struct Material {
    /// Stable unique id.
    std::string id;
    std::string name;

    bool operator==(const Material&) const = default;
};

/// A CNC machine. Accelerations are used only for time estimation (mm/s² and rad/s²).
struct Machine {
    /// Stable unique id.
    std::string id;
    std::string name;
    /// Linear acceleration per axis, mm/s² (X, Y, Z).
    double accelerationX{0.0};
    double accelerationY{0.0};
    double accelerationZ{0.0};
    /// Rotary-axis acceleration, rad/s².
    double accelerationAngular{0.0};
    /// Names of the post-processors usable with this machine, in user order.
    std::vector<std::string> associatedPosts;
    /// The post selected by default; empty = none. Expected to be one of `associatedPosts`.
    std::string defaultPost;
    /// Rapid travel rate, mm/min; 0 = not set (rapid moves are then left out of time estimates).
    double rapidRate{0.0};
    /// Work area (X, Y, Z travel), mm; 0 = not set.
    double workAreaX{0.0};
    double workAreaY{0.0};
    double workAreaZ{0.0};
    /// Spindle speed limit, rev/min; 0 = not set.
    double maxSpindleSpeed{0.0};
    /// Feed rate limit, mm/min; 0 = not set.
    double maxFeedRate{0.0};

    bool operator==(const Machine&) const = default;
};

}  // namespace carvetto::tooling
