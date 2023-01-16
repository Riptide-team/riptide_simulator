#include "riptide_simulator/Simulator.hpp"
#include "riptide_simulator/Riptide.hpp"

#include <iostream>


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<riptide_simulator::Simulator>());
    rclcpp::shutdown();
    return 0;
}