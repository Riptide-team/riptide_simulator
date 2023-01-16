#pragma once

#include <memory>
#include <utility>
#include <vector>

namespace riptide_simulator {
    class Pool {
        public:

            using Ptr = std::shared_ptr<Pool>;
            using Coordinates = std::vector<std::pair<double, double>>;

            // Pointer to a Pool Creation
            static Ptr Create(Coordinates walls, double depth);
            static Ptr Create(std::vector<double> walls_x, std::vector<double> walls_y, double depth);

            // Depth getter
            inline double Depth() const { return depth_; };

            // Walls getter
            inline Coordinates Walls() const { return walls_; };

            // Number of points getter
            inline unsigned int N() const { return walls_.size(); };

            // Center getter
            std::pair<double, double> Center() const;

            // Bounding box getter
            Coordinates BoundingBox() const;

        private:
            // Constructor
            Pool(Coordinates walls, double depth) : depth_(depth), walls_(walls) {};

            // Piscine depth
            double depth_;

            // Walls coordinates (xi, yi)
            Coordinates walls_;
    };
} // riptide_simulator