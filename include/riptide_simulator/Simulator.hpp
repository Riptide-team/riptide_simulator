#pragma once
#include "riptide_simulator/Riptide.hpp"
#include "riptide_simulator/Pool.hpp"

#include "rclcpp/rclcpp.hpp"
#include "spawnmsg.pb.h"

#include <ignition/msgs/boolean.pb.h>
#include <ignition/transport.hh>

#include <visualization_msgs/msg/marker.hpp>


namespace riptide_simulator {
    class Simulator : public rclcpp::Node {
        public:
            Simulator();

            void init_ignition();

            void update();

        private:
            // Update timer
            rclcpp::TimerBase::SharedPtr update_timer_;

            // Time step
            std::chrono::duration<float> dt_ = std::chrono::milliseconds(5);

            // Create a transport node.
            ignition::transport::Node node;

            // Service spawner name
            std::string service_spawner = "/riptide_simulator/spawner";

            bool spawner_cb(const riptide::msgs::SpawnMsg &_req, ignition::msgs::Boolean &_rep);

            // Riptide vector mutex
            std::mutex riptides_mutex_;

            // Riptides Pointer
            std::vector<Riptide<RiptideParameters>::Ptr> riptides_;

            // Pool pointer
            Pool::Ptr pool_ptr;

            // Show pool method
            void show_pool();

            // Marker publisher
            rclcpp::TimerBase::SharedPtr pool_timer_;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pool_marker_publisher_;
    };
} // riptide_simulator