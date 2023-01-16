#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include "spawnmsg.pb.h"


namespace riptide_simulator {
    class RiptideSpawner : public rclcpp::Node {
        public:
            RiptideSpawner() : Node("riptide_spawner") {
                this->declare_parameter("riptide_name", "");
                this->declare_parameter("position", std::vector<double>({0., 0., 0.}));
                this->declare_parameter("orientation", std::vector<double>({0., 0., 0., 1.}));
                this->declare_parameter("robot_description", "");
                CallSpawnService();
            };

        private:
            void CallSpawnService() {
                std::vector<double> position = this->get_parameter("position").as_double_array();
                std::vector<double> orientation = this->get_parameter("orientation").as_double_array();
                std::string robot_description = this->get_parameter("robot_description").as_string();

                if (robot_description.empty()) {
                    RCLCPP_FATAL(this->get_logger(), "The `robot_description` should be non-empty");
                }

                std::copy(position.begin(), position.end(), std::ostream_iterator<double>(std::cout, " "));
                std::copy(orientation.begin(), orientation.end(), std::ostream_iterator<double>(std::cout, " "));

                // Create a transport node.
                ignition::transport::Node node;

                // Request message
                riptide::msgs::SpawnMsg req;

                // Setting the name
                req.set_name((this->get_parameter("riptide_name")).value_to_string());

                // Setting the position
                riptide::msgs::Vector3* p = req.mutable_position();
                p->set_x(position[0]);
                p->set_y(position[1]);
                p->set_z(position[2]);

                // Setting the orientation
                riptide::msgs::Quaternion* o = req.mutable_orientation();
                o->set_x(orientation[0]);
                o->set_y(orientation[1]);
                o->set_z(orientation[2]);
                o->set_w(orientation[3]);

                // Setting robot description
                req.set_robot_description(robot_description);

                // Request the "/riptide_simulator/spawner" service.
                ignition::msgs::Boolean rep;
                bool result;
                unsigned int timeout = 5000;

                bool executed = node.Request("/riptide_simulator/spawner", req, timeout, rep, result);
                if (executed) {
                    if (result)
                        RCLCPP_INFO(this->get_logger(), "Riptide spawned!");
                    else
                        RCLCPP_FATAL(this->get_logger(), "Service call failed");
                }
                else
                    RCLCPP_FATAL(this->get_logger(), "Service call timed out");
            };
    };
} // riptide_simulator

int main(int argc, char * argv[]) {
        rclcpp::init(argc, argv);
        riptide_simulator::RiptideSpawner();
        rclcpp::shutdown();
        return 0;
    }