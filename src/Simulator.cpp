#include "riptide_simulator/Simulator.hpp"
#include "riptide_simulator/Pool.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <functional>
#include <algorithm>

#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include "spawnmsg.pb.h"
#include <ignition/msgs/boolean.pb.h>

#include <Eigen/Geometry>

using namespace std::chrono_literals;

namespace riptide_simulator {
    Simulator::Simulator() : Node("riptide_simulator") {
        //  Declaring parameters
        this->declare_parameter("walls_x", std::vector<double>({-10., 10., 10., -10.}));
        this->declare_parameter("walls_y", std::vector<double>({-6., -6., 6., 6.}));
        this->declare_parameter("depth", 6.);
        this->declare_parameter("show_pool", true);

        // Getting parameters
        double pool_depth_ = this->get_parameter("depth").as_double();
        std::vector<double> pool_walls_x = this->get_parameter("walls_x").as_double_array();
        std::vector<double> pool_walls_y = this->get_parameter("walls_y").as_double_array();

        // Checking parameters
        if (pool_depth_ < 0.) {
            RCLCPP_FATAL(this->get_logger(), "The pool's depth should be positive! Got %f", pool_depth_);
        }

        if (pool_walls_x.size() != pool_walls_y.size()) {
            RCLCPP_FATAL(this->get_logger(), "Pool wall's coordinate should have the same size! walls_x.size()=%lu, walls_y.size()=%lu",
            pool_walls_x.size(), pool_walls_y.size());
        }

        // Creating a pool pointer
        pool_ptr = Pool::Create(pool_walls_x, pool_walls_y, pool_depth_);

        // Init ignition
        init_ignition();

        // Create callback to update the simulation
        update_timer_ = this->create_wall_timer(dt_, std::bind(&Simulator::update, this));
        
        bool show_pool_ = this->get_parameter("show_pool").as_bool();
        if (show_pool_) {
            pool_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("~/pool/marker", 10);
            pool_timer_ = this->create_wall_timer(50ms, std::bind(&Simulator::show_pool, this));
        }
    };

    void Simulator::init_ignition() {
        // Advertise a service call.
        if (!node.Advertise(service_spawner, &Simulator::spawner_cb, this)) {
            RCLCPP_FATAL(this->get_logger(), "Error advertising service [%s]", service_spawner.c_str());
        }
    }

    bool Simulator::spawner_cb(const riptide::msgs::SpawnMsg &_req, ignition::msgs::Boolean &_rep) {
        std::string spawn_name = _req.name();

        // Checking that the provided name is not already used by another riptide
        std::vector<std::string> names;
        std::transform(
            riptides_.cbegin(), riptides_.cend(), std::back_inserter(names),
            [](const Riptide<RiptideParameters>::Ptr r) { return r->get_name(); }
        );
        auto itr = find(names.begin(), names.end(), spawn_name);
        if(itr != names.end()) {
            RCLCPP_FATAL(this->get_logger(), "Riptide already named: %s", spawn_name.c_str());
            return false;
        }

        // Getting position
        Eigen::Vector3d p;
        p << _req.position().x(), _req.position().y(), _req.position().z();
        
        // Getting orientation
        Eigen::Quaterniond q;
        q.x() = _req.orientation().x();
        q.y() = _req.orientation().y();
        q.z() = _req.orientation().z();
        q.w() = _req.orientation().w();

        // Getting robot description
        std::string robot_description = _req.robot_description();

        // Create a Riptide with the provided name
        Riptide<RiptideParameters>::Ptr r_ptr = Riptide<RiptideParameters>::Create(spawn_name, p, q, robot_description, pool_ptr);
        std::lock_guard<std::mutex> lock_(riptides_mutex_);

        RCLCPP_INFO(this->get_logger(), "Riptide Spawned!");

        riptides_.push_back(r_ptr);

        // Getting spawn name
        _rep.set_data(true);
        return true;
    }

    void Simulator::show_pool() {
        visualization_msgs::msg::Marker m;
        m.id = 0;
        m.ns = "pool";
        m.header.frame_id = "world";
        m.header.stamp = this->now();
        m.action = visualization_msgs::msg::Marker::ADD;
        m.type = visualization_msgs::msg::Marker::CUBE;

        // Color
        std_msgs::msg::ColorRGBA color;
        color.a = .25;
        color.r = 65./255.;
        color.g = 105./255.;
        color.b = 1.;
        m.color = color;

        // Shape
        geometry_msgs::msg::Pose pose;
        std::pair<double,double> center = pool_ptr->Center();
        pose.position.x = center.first;
        pose.position.y = center.second;
        pose.position.z = -pool_ptr->Depth()/2.;
        m.pose = pose;

        Pool::Coordinates bb = pool_ptr->BoundingBox();
        m.scale.x = std::abs(bb[0].first) + std::abs(bb[0].second);
        m.scale.y = std::abs(bb[1].first) + std::abs(bb[1].second);
        m.scale.z = pool_ptr->Depth();

        for (const auto &p: pool_ptr->Walls()) {
            geometry_msgs::msg::Point pt;
            pt.x = p.first;
            pt.y = p.second;
            m.points.push_back(pt);
            m.colors.push_back(color);
        }

        m.mesh_use_embedded_materials = true;
        pool_marker_publisher_->publish(m);
    }

    void Simulator::update() {
        std::lock_guard<std::mutex> lock_(riptides_mutex_);
        for (auto &ptr_r: riptides_) {
            ptr_r->update(dt_.count());
        }
    }
}