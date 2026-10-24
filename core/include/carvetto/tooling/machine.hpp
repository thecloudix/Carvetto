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

    bool operator==(const Machine&) const = default;
};

}  // namespace carvetto::tooling
