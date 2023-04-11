#pragma once

#include "riptide_simulator/Actuator.hpp"
#include "riptide_simulator/RiptideParameters.hpp"
#include "riptide_simulator/Pool.hpp"

#include <eigen3/Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

#include <cmath>
#include <string>
#include <memory>
#include <mutex>
#include <variant>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <ignition/transport/NodeOptions.hh>

#include "controlmsg.pb.h"
#include "multijointmsg.pb.h"
#include "pressuremsg.pb.h"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <kdl_parser/kdl_parser.hpp>

#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/string.hpp>


namespace riptide_simulator {

    /// Returns the 3D cross product Skew symmetric matrix of a given 3D vector.
    template<class Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 3, 3> Skew(const Eigen::MatrixBase<Derived> & vec) {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
        return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0, -vec[2], vec[1],
            vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0).finished();
    }

    template <typename param = RiptideParameters>
    class Riptide : public rclcpp::Node {
        public:

            // Pointer to a Riptide
            using Ptr = std::shared_ptr<Riptide<param>>;

            // Create interface to create a Riptide
            static Ptr Create(std::string name, Eigen::Vector3d position, Eigen::Quaterniond quaternion, std::string robot_description, Pool::Ptr pool);

            // Update method
            void update(const double dt);

            // Name getter
            inline std::string get_name() const { return name_; };

        private:
            // Constructor, to construct use Riptide::Create interface and manipulate Riptide::Ptr
            Riptide(std::string name, Eigen::Vector3d position, Eigen::Quaterniond quaternion, std::string robot_description, Pool::Ptr pool);

            // name of the riptide
            std::string name_;

            // Position
            Eigen::Vector3d p_;

            // Velocity
            Eigen::Vector3d v_;

            // Acceleration
            Eigen::Vector3d a_;

            // Rotation matrix
            Eigen::Matrix3d R_;

            // Requested angular velocity
            Eigen::Vector3d wr_;

            // Control vector
            Eigen::Vector4d u_;

            // Fluid density
            double rho_ = 1000.;

            // Fluid temperature
            double temperature_ = 15.;

            // Gravity
            double g_ = 9.81;

            // Atmospheric pressure in Pa
            double P0_ = 101300.0;

            // Echosounder measured distance
            double d_;

            // Battery tension
            double voltage_ = 16.;

            // Current currently consumed
            double current_ = .5;

            // Mutex
            std::mutex mutex_;

            // Create a transport node.
            std::shared_ptr<ignition::transport::Node> node;

            // Imu service name
            std::string imu_service = "imu";

            // Imu service callback
            bool imu_cb(ignition::msgs::IMU &_rep);

            // EchoSounder service name
            std::string echosounder_service = "echosounder/range";

            // EchoSounder service callback
            bool echosounder_cb(ignition::msgs::Float &_rep);

            // Joint state name
            std::string joint_state_service = "joint_state";

            // Joint state service callback
            bool joint_state_cb(riptide::msgs::MultiJointMsg &_rep);

            // Control service name
            std::string control_service = "control";

            // Imu service callback
            void control_cb(const riptide::msgs::ControlMsg &_req);
            
            // Pressure name
            std::string pressure_service = "pressure";

            // Pressure callback
            bool pressure_cb(riptide::msgs::PressureMsg &_rep);

            // Battery name
            std::string battery_service = "battery";

            // Battery service callback
            bool battery_cb(ignition::msgs::BatteryState &_rep);

            // Ignition transport initializer
            void init_ignition();

            // ROS2 publishers
            void init_ros2();

            // Static tf publisher
            std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
            void publish_static_tf();

            // Joint tf publisher
            std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;
            void publish_dynamic_tf();

            // Parameters
            std::string robot_description_;

            // KDL tree of the robot
            KDL::Tree kdl_tree_;

            // Robot description publisher
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_description_publisher_;

            rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr echosounder_publisher_;

            // Actuators
            Fin::Ptr d_fin_;
            Fin::Ptr p_fin_;
            Fin::Ptr s_fin_;
            Thruster::Ptr thruster_;

            // Pool pointer
            Pool::Ptr pool_;

    };

    // Implementation
    template <typename param>
    Riptide<param>::Riptide(std::string name, Eigen::Vector3d position, Eigen::Quaterniond quaternion, std::string robot_description, Pool::Ptr pool) : Node(name), R_(quaternion.normalized().toRotationMatrix()) {
        // Non-empty name check
        if (name.empty())
            RCLCPP_FATAL(this->get_logger(), "Unable to spawn a Riptide without name");
        else
            name_ = name;

        // Robot description
        robot_description_ = robot_description;
        if (!kdl_parser::treeFromString(robot_description_, kdl_tree_)){
            RCLCPP_FATAL(this->get_logger(), "Failed to construct kdl tree");
        }

        // Position, velocity, acceleration
        p_ = position;
        v_ = Eigen::Vector3d::Zero();
        a_ = Eigen::Vector3d::Zero();

        // Control vector
        u_ = Eigen::Vector4d::Zero();

        // Actuators simulation
        d_fin_ = Actuator<Fin>::Create();
        p_fin_ = Actuator<Fin>::Create();
        s_fin_ = Actuator<Fin>::Create();

        thruster_ = Actuator<Thruster>::Create();

        // Pool
        pool_ = pool;

        // Ignition init
        init_ignition();

        // Ros2 init
        init_ros2();
    }

    template <typename param>
    typename Riptide<param>::Ptr Riptide<param>::Create(std::string name, Eigen::Vector3d position, Eigen::Quaterniond quaternion, std::string robot_description, Pool::Ptr pool) {
        return Ptr(new Riptide(name, position, quaternion, robot_description, pool));
    }

    template <typename param>
    void Riptide<param>::init_ros2() {
        // static tf publisher
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

        publish_static_tf();

        // tf publisher
        dynamic_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // Robot description publisher
        robot_description_publisher_ = this->create_publisher<std_msgs::msg::String>("~/robot_description", rclcpp::QoS(1).transient_local());
        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = robot_description_;
        robot_description_publisher_->publish(std::move(msg));

        // Echosounder range publisher
        echosounder_publisher_ = this->create_publisher<sensor_msgs::msg::Range>("~/echosounder/range", 10);

        RCLCPP_INFO(this->get_logger(), "Sucessfully initialized ros2!");
    }

    template <typename param>
    void Riptide<param>::init_ignition() {
        // Node init
        ignition::transport::NodeOptions node_option;
        node_option.SetNameSpace(name_);
        node = std::make_shared<ignition::transport::Node>(node_option);

        // Advertise IMU service call.
        if (!node->Advertise(imu_service, &Riptide::imu_cb, this)) {
            std::cout << "Error advertising service [" << imu_service << "]\n";
        }

        // Advertise EchouSounder service call.
        if (!node->Advertise(echosounder_service, &Riptide::echosounder_cb, this)) {
            std::cout << "Error advertising service [" << echosounder_service << "]\n";
        }

        // Advertise control service call.
        if (!node->Advertise(control_service, &Riptide::control_cb, this)) {
            std::cout << "Error advertising service [" << control_service << "]\n";
        }

        // Advertise joint_state service call.
        if (!node->Advertise(joint_state_service, &Riptide::joint_state_cb, this)) {
            std::cout << "Error advertising service [" << joint_state_service << "]\n";
        }

        // Advertise pressure service call.
        if (!node->Advertise(pressure_service, &Riptide::pressure_cb, this)) {
            std::cout << "Error advertising service [" << pressure_service << "]\n";
        }

        // Advertise battery service call.
        if (!node->Advertise(battery_service, &Riptide::battery_cb, this)) {
            std::cout << "Error advertising service [" << battery_service << "]\n";
        }

        RCLCPP_INFO(this->get_logger(), "Sucessfully initialized ignition!");
    }

    template <typename param>
    void Riptide<param>::update(const double dt) {
        // Lock guard
        std::lock_guard<std::mutex> lock_(mutex_);

        // Joint simulation
        thruster_->update(dt);
        d_fin_->update(dt);
        p_fin_->update(dt);
        s_fin_->update(dt);

        // Riptide simulation
        Eigen::Vector3d u_r = {d_fin_->get_position(), p_fin_->get_position(), s_fin_->get_position()};
        wr_ = v_(0) * param::I * param::B * u_r;
        a_ = param::thrust * (Eigen::Vector3d() << thruster_->get_velocity() * std::abs(thruster_->get_velocity()), 0, 0).finished() - param::f * v_.cwiseProduct(v_.cwiseAbs());
        v_ += dt * a_;
        p_ += dt * R_ * v_;
        R_ = R_ * (Skew(dt * wr_)).exp();

        // Publishing dynamic tf
        publish_dynamic_tf();
    }

    template <typename param>
    bool Riptide<param>::imu_cb(ignition::msgs::IMU &_rep) {
        _rep.set_entity_name(name_);

        // Lock guard
        std::lock_guard<std::mutex> lock_(mutex_);

        // Linear acceleration
        Eigen::Vector3d am = a_ + wr_.cross(v_) - R_.transpose() * (Eigen::Vector3d() << 0, 0, -9.8).finished();
        ignition::msgs::Vector3d acceleration_msg;
        acceleration_msg.set_x(am[0]);
        acceleration_msg.set_y(am[1]);
        acceleration_msg.set_z(am[2]);
        _rep.mutable_linear_acceleration()->CopyFrom(acceleration_msg);

        // Angular velocity
        ignition::msgs::Vector3d velocity_msg;
        velocity_msg.set_x(wr_[0]);
        velocity_msg.set_y(wr_[1]);
        velocity_msg.set_z(wr_[2]);
        _rep.mutable_angular_velocity()->CopyFrom(velocity_msg);

        // Quaternion
        ignition::msgs::Quaternion quaternion_msg;
        Eigen::Quaterniond q(R_);
        quaternion_msg.set_x(q.x());
        quaternion_msg.set_y(q.y());
        quaternion_msg.set_z(q.z());
        quaternion_msg.set_w(q.w());
        _rep.mutable_orientation()->CopyFrom(quaternion_msg);

        return true;
    }

    template <typename param>
    bool Riptide<param>::pressure_cb(riptide::msgs::PressureMsg &_rep) {
        // Lock guard
        std::lock_guard<std::mutex> lock_(mutex_);

        _rep.set_pressure(-rho_ * g_ * p_(2) + P0_);
        _rep.set_temperature(temperature_);
        _rep.set_depth(-p_(2));
        _rep.set_altitude(std::numeric_limits<double>::quiet_NaN());

        return true;
    }

    template <typename param>
    bool Riptide<param>::battery_cb(ignition::msgs::BatteryState &_rep) {
        // Lock guard
        std::lock_guard<std::mutex> lock_(mutex_);

        _rep.set_voltage(voltage_);
        _rep.set_current(current_);

        return true;
    }

    template <typename param>
    bool Riptide<param>::echosounder_cb(ignition::msgs::Float &_rep) {
        // Lock guard
        std::lock_guard<std::mutex> lock_(mutex_);

        // Echosounder simulation
        Eigen::Vector3d u = - R_ * Eigen::Vector3d::UnitZ();

        double epsilon = 0.1;
        if (u.dot(-Eigen::Vector3d::UnitZ()) > 1 - epsilon) {
            _rep.set_data(p_(2) + pool_->Depth());
            return true;
        }
        else if (u.dot(Eigen::Vector3d::UnitZ()) > 1 - epsilon) {
            _rep.set_data(-p_(2));
            return true;
        }
        else if (std::abs(u.dot(Eigen::Vector3d::UnitZ())) < epsilon) {
            unsigned int n = pool_->N();
            Pool::Coordinates walls = pool_->Walls();

            // Second point determinant
            Eigen::Matrix2d A;
            A(0, 1) = u(0);
            A(1, 1) = u(1);
            
            // Second point determinant
            Eigen::Matrix2d B;
            B(0, 1) = u(0);
            B(1, 1) = u(1);

            d_ = std::numeric_limits<double>::infinity();

            for (unsigned int i=0; i<n; ++i) {
                // First point determinant
                A(0, 1) = walls[i].first - p_(0);
                A(1, 1) = walls[i].second - p_(1);

                // Finishing second point determinant
                B(0, 1) = walls[(i+1)%n].first - p_(0);
                B(1, 1) = walls[(i+1)%n].second - p_(1);

                if (A.determinant() * B.determinant() <= 0) {
                    Eigen::Matrix2d aA {
                        {walls[i].first - p_(0), walls[(i+1)%n].first - walls[i].first},
                        {walls[i].second - p_(1), walls[(i+1)%n].second - walls[i].second}
                    };

                    Eigen::Matrix2d aB {
                        {u(0), walls[(i+1)%n].first - walls[i].first},  
                        {u(1), walls[(i+1)%n].second - walls[i].second}
                    };

                    double alpha = aA.determinant() / aB.determinant();
                    if (alpha >= 0.) {
                        d_ = std::min(d_, alpha);
                    }
                }
            }
            _rep.set_data(d_);
            return true;
        }
        return false;
    }

    template <typename param>
    void Riptide<param>::control_cb(const riptide::msgs::ControlMsg &_req) {
        // Lock guard
        std::lock_guard<std::mutex> lock_(mutex_);

        // Setting command to actuators
        if (!std::isnan(_req.thrust()))
            thruster_->set_command(_req.thrust());
        if (!std::isnan(_req.d()))
            d_fin_->set_command(-_req.d()); // To stick to the riptide to be verified / unified
        if (!std::isnan(_req.p()))
            p_fin_->set_command(-_req.p()); // To stick to the riptide to be verified / unified
        if (!std::isnan(_req.s()))
            s_fin_->set_command(-_req.s()); // To stick to the riptide to be verified / unified
    }

    template <typename param>
    bool Riptide<param>::joint_state_cb(riptide::msgs::MultiJointMsg &_rep) {
        // Lock guard
        std::lock_guard<std::mutex> lock_(mutex_);

        // Filling the MultiJoint msg
        riptide::msgs::JointMsg* thruster_joint = _rep.add_joint();
        thruster_joint->set_name("thruster");
        thruster_joint->set_position(thruster_->get_position());
        thruster_joint->set_velocity(thruster_->get_velocity());

        riptide::msgs::JointMsg* d_joint = _rep.add_joint();
        d_joint->set_name("d");
        d_joint->set_position(d_fin_->get_position());
        d_joint->set_velocity(d_fin_->get_velocity());

        riptide::msgs::JointMsg* p_joint = _rep.add_joint();
        p_joint->set_name("p");
        p_joint->set_position(p_fin_->get_position());
        p_joint->set_velocity(p_fin_->get_velocity());

        riptide::msgs::JointMsg* s_joint = _rep.add_joint();
        s_joint->set_name("s");
        s_joint->set_position(s_fin_->get_position());
        s_joint->set_velocity(s_fin_->get_velocity());

        return true;
    }

    template <typename param>
    void Riptide<param>::publish_dynamic_tf() {
        std::vector<geometry_msgs::msg::TransformStamped> tf_transforms;

        rclcpp::Time now = this->now();
        
        // tf between world and the riptide
        geometry_msgs::msg::TransformStamped tf_riptide;
        tf_riptide.header.stamp = now;
        tf_riptide.header.frame_id = "world";
        tf_riptide.child_frame_id = name_;

        tf_riptide.transform.translation.x = p_(0);
        tf_riptide.transform.translation.y = p_(1);
        tf_riptide.transform.translation.z = p_(2);

        Eigen::Quaterniond q(R_);
        tf_riptide.transform.rotation.x = q.x();
        tf_riptide.transform.rotation.y = q.y();
        tf_riptide.transform.rotation.z = q.z();
        tf_riptide.transform.rotation.w = q.w();

        tf_transforms.push_back(tf_riptide);

        // tf for each actuators;
        Fin::Ptr d_fin_ptr = d_fin_;
        Fin::Ptr p_fin_ptr = p_fin_;
        Fin::Ptr s_fin_ptr = s_fin_;
        Thruster::Ptr thruster_ptr = thruster_;
        std::vector<std::pair<std::string, std::variant<Fin::Ptr, Thruster::Ptr>>> frames = {
            std::make_pair<std::string, Fin::Ptr>("d_fin", std::move(d_fin_ptr)),
            std::make_pair<std::string, Fin::Ptr>("p_fin", std::move(p_fin_ptr)),
            std::make_pair<std::string, Fin::Ptr>("s_fin", std::move(s_fin_ptr)),
            std::make_pair<std::string, Thruster::Ptr>("thruster", std::move(thruster_ptr))
        };

        for (const auto &f: frames) {
            double position;
            std::visit([&](auto arg){ position = arg->get_position(); }, f.second);
            KDL::Frame frame = kdl_tree_.getSegment(name_ + "_" + f.first)->second.segment.pose(position);
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = now;
            tf.header.frame_id = name_;
            tf.child_frame_id = name_ + "_" + f.first;

            tf.transform.translation.x = frame.p[0];
            tf.transform.translation.y = frame.p[1];
            tf.transform.translation.z = frame.p[2];
            frame.M.GetQuaternion(tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w);
            tf_transforms.push_back(tf);
        }

        dynamic_tf_broadcaster_->sendTransform(tf_transforms);
    }

    template <typename param>
    void Riptide<param>::publish_static_tf() {
        std::vector<geometry_msgs::msg::TransformStamped> tf_transforms;
        rclcpp::Time now = this->now();
        
        // tf between world and the riptide
        geometry_msgs::msg::TransformStamped tf_echosounder;
        tf_echosounder.header.frame_id = name_;
        tf_echosounder.child_frame_id = name_ + "_echosounder";

        KDL::Frame frame = kdl_tree_.getSegment(name_ + "_echosounder")->second.segment.pose(0.);
        tf_echosounder.transform.translation.x = frame.p[0];
        tf_echosounder.transform.translation.y = frame.p[1];
        tf_echosounder.transform.translation.z = frame.p[2];
        frame.M.GetQuaternion(tf_echosounder.transform.rotation.x, tf_echosounder.transform.rotation.y, tf_echosounder.transform.rotation.z, tf_echosounder.transform.rotation.w);
        tf_transforms.push_back(tf_echosounder);

        static_tf_broadcaster_->sendTransform(tf_transforms);
    }

} // riptide_simulator