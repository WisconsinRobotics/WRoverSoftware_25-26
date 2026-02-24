#include <memory>
#include <cmath>
#include <functional>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "ctre/phoenix6/Pigeon2.hpp"

using namespace ctre::phoenix6;
using namespace std::chrono_literals;

class CompassDataPublisher : public rclcpp::Node {
    public:
        CompassDataPublisher()
        : Node("compass"),
          pigeon2imu(10, "can0")
        {
            // Publisher set to "imu/data"
            publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
            
            // 50Hz Timer
            timer_ = this->create_wall_timer(
                20ms, std::bind(&CompassDataPublisher::timer_callback, this));
        }

    private:
        void timer_callback()
        {
            auto message = sensor_msgs::msg::Imu();
            
            message.header.stamp = this->get_clock()->now();
            
            //Get Signals
            auto &q_w = pigeon2imu.GetQuatW();
            auto &q_x = pigeon2imu.GetQuatX();
            auto &q_y = pigeon2imu.GetQuatY();
            auto &q_z = pigeon2imu.GetQuatZ();
            
            auto &gyro_x = pigeon2imu.GetAngularVelocityXWorld();
            auto &gyro_y = pigeon2imu.GetAngularVelocityYWorld();
            auto &gyro_z = pigeon2imu.GetAngularVelocityZWorld();

            auto &accel_x = pigeon2imu.GetAccelerationX();
            auto &accel_y = pigeon2imu.GetAccelerationY();
            auto &accel_z = pigeon2imu.GetAccelerationZ();

            // Synchronous Refresh
            BaseStatusSignal::RefreshAll(
                q_w, q_x, q_y, q_z, 
                gyro_x, gyro_y, gyro_z, 
                accel_x, accel_y, accel_z
            );

            // Populate Orientation
            message.orientation.w = q_w.GetValue().value();
            message.orientation.x = q_x.GetValue().value();
            message.orientation.y = q_y.GetValue().value();
            message.orientation.z = q_z.GetValue().value();

            // Populate Angular Velocity (Deg -> Rad)
            double deg_to_rad = M_PI / 180.0;
            message.angular_velocity.x = gyro_x.GetValue().value() * deg_to_rad;
            message.angular_velocity.y = gyro_y.GetValue().value() * deg_to_rad;
            message.angular_velocity.z = gyro_z.GetValue().value() * deg_to_rad;

            // Populate Linear Acceleration (G -> m/s^2)
            double g_to_mps2 = 9.80665;
            message.linear_acceleration.x = accel_x.GetValue().value() * g_to_mps2;
            message.linear_acceleration.y = accel_y.GetValue().value() * g_to_mps2;
            message.linear_acceleration.z = accel_z.GetValue().value() * g_to_mps2;

            // Covariance
            message.orientation_covariance = {0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001};
            message.angular_velocity_covariance = {0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02};
            message.linear_acceleration_covariance = {0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04};

            publisher_->publish(message);
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
        hardware::Pigeon2 pigeon2imu;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CompassDataPublisher>());
    rclcpp::shutdown();
    return 0;
}