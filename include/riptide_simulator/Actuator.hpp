#pragma once

#include <algorithm>
#include <cmath>
#include <memory>


namespace riptide_simulator {
    template <typename T>
    class Actuator {
        public:
            using Ptr = std::shared_ptr<Actuator<T>>;

            static Ptr Create(double position=0, double velocity=0, double command=0) {
                return Ptr(new Actuator<T>(position, velocity, command));
            };

            // Update method
            void update(const double dt) { static_cast<T*>(this)->update_impl(dt, command_); };

            // Command setter
            void set_command(const double u) { command_ = u; };

            // Position getter
            double get_position() const { return position_; };

            // Velocity getter
            double get_velocity() const { return velocity_; };

        protected:
            // Constructor, to construct use Riptide::Create interface and manipulate Riptide::Ptr
            Actuator(double position, double velocity, double command) : position_(position), velocity_(velocity), command_(command) {};
            
            // Position
            double position_;

            // Velocity
            double velocity_;

            // Command
            double command_;
    };

    struct Fin : Actuator<Fin> {
        using Ptr = std::shared_ptr<Actuator<Fin>>;

        void update_impl(const double dt, double u) {
            velocity_ = M_PI / 0.36 * std::atan(u - position_);
            position_ = std::clamp(position_ + dt * velocity_, - M_PI/4, M_PI/4);
        }
    };

    struct Thruster : Actuator<Thruster> {
        using Ptr = std::shared_ptr<Actuator<Thruster>>;

        void update_impl(const double dt, double u) {
            velocity_ = std::clamp(u, -120 * M_PI, 120 * M_PI);
            position_ += dt * velocity_;
        }
    };
} // riptide_simulator