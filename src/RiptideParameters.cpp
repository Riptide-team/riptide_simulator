#include "riptide_simulator/RiptideParameters.hpp"

namespace riptide_simulator {
    // Inertia matrix
    const Eigen::Matrix3d RiptideParameters::I = Eigen::DiagonalMatrix<double, 3>(5, 10, 10);

    // Geometry matrix
    const Eigen::Matrix3d RiptideParameters::B = (Eigen::Matrix3d() << -1, -1, -1,
            0, std::sin(2*M_PI/3), -std::sin(2*M_PI/3),
            1, std::cos(2*M_PI/3), std::cos(2*M_PI/3)).finished();

    // Thrust parameter
    const double RiptideParameters::thrust = 50 / std::pow(120 * M_PI, 2);

    // Fluid friction coefficient
    const double RiptideParameters::f = 5;
} // riptide_simulator