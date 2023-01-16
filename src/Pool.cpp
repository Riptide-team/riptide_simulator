#include "riptide_simulator/Pool.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace riptide_simulator {
    Pool::Ptr Pool::Create(Pool::Coordinates walls, double depth) {
        return Pool::Ptr(new Pool(walls, depth));
    }

    Pool::Ptr Pool::Create(std::vector<double> walls_x, std::vector<double> walls_y, double depth) {
        unsigned int n = std::min(walls_x.size(), walls_y.size());
        Pool::Coordinates walls(0);
        std::transform(walls_x.begin(), walls_x.begin()+n, walls_y.begin(), std::back_inserter(walls),
            [](const auto &x, const auto &y){ return std::make_pair(x, y); }
        );
        return Pool::Ptr(new Pool(walls, depth));
    }

    std::pair<double, double> Pool::Center() const {
        const auto p = std::minmax_element(walls_.begin(), walls_.end());
        return std::make_pair<double,double>((p.first->first+p.second->first) / 2., (p.first->second+p.second->second) / 2.);
    }

    Pool::Coordinates Pool::BoundingBox() const {
        const auto p = std::minmax_element(walls_.begin(), walls_.end());
        Pool::Coordinates c;
        c.emplace_back(p.first->first, p.second->first);
        c.emplace_back(p.first->second, p.second->second);
        return c;
    }
} // riptide_simulator