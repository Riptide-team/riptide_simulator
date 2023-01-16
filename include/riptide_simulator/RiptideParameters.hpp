#pragma once

#include <eigen3/Eigen/Dense>

namespace riptide_simulator {
    struct RiptideParameters {
        // Inertia matrix
        static const Eigen::Matrix3d I;

        // Geometry matrix
        static const Eigen::Matrix3d B;

        // Thrust coefficient
        static const double thrust;

        // Fluid friction coefficient
        static const double f;
    };
} // riptide_simulator
    