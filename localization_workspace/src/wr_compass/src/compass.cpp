#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <functional> // Include this for std::bind4
#include <chrono>
#include <functional>
#include <memory>
#include "ctre/phoenix6/Pigeon2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <numbers>
#include <cmath>


using namespace ctre::phoenix6;
using namespace std::chrono_literals;


class CompassDataPublisher : public rclcpp::Node {
    public:
        CompassDataPublisher()
        :
            Node("compass"),
            pigeon2imu(10, "can0")
        {
            publisher_quat = this->create_publisher<sensor_msgs::msg::Imu>("imu_quat_data", 10);
	    publisher_euler = this->create_publisher<sensor_msgs::msg::Imu>("imu_euler_data", 10);
            timer_ = this->create_wall_timer(
                500ms, std::bind(&CompassDataPublisher::timer_callback, this));
        }

    private:
        void timer_callback()
        {
            sensor_msgs::msg::Imu quat_message;

	    double qx = pigeon2imu.GetQuatX().GetValue().value();
            double qy = pigeon2imu.GetQuatY().GetValue().value();
            double qz = pigeon2imu.GetQuatZ().GetValue().value();
            double qw = pigeon2imu.GetQuatW().GetValue().value();
	    //normalizing values
	    double n = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
	    if (n > 0.0) {
           	 qx /= n; qy /= n; qz /= n; qw /= n;
	    }

            double gx = pigeon2imu.GetAngularVelocityX().GetValue().value();
            double gy = pigeon2imu.GetAngularVelocityY().GetValue().value();
            double gz = pigeon2imu.GetAngularVelocityZ().GetValue().value();

            constexpr double deg2rad = std::numbers::pi / 180.0;
            gx *= deg2rad;
            gy *= deg2rad;
            gz *= deg2rad;

            double ax = pigeon2imu.GetAccelerationX().GetValue().value();
            double ay = pigeon2imu.GetAccelerationY().GetValue().value();
            double az = pigeon2imu.GetAccelerationZ().GetValue().value();

	    quat_message.orientation.x = qx;
            quat_message.orientation.y = qy;
            quat_message.orientation.z = qz;
            quat_message.orientation.w = qw;

            quat_message.angular_velocity.x = gx;
            quat_message.angular_velocity.y = gy;
            quat_message.angular_velocity.z = gz;

	    quat_message.linear_acceleration.x = ax;
            quat_message.linear_acceleration.y = ay;
            quat_message.linear_acceleration.z = az;

            publisher_quat->publish(quat_message);

            //euler
            sensor_msgs::msg::Imu euler_message;

            euler_message.orientation.x = pigeon2imu.GetRoll().GetValue().value() * deg2rad;
            euler_message.orientation.y = pigeon2imu.GetPitch().GetValue().value() * deg2rad;
            euler_message.orientation.z = pigeon2imu.GetYaw().GetValue().value() * deg2rad;

            euler_message.angular_velocity = quat_message.angular_velocity;
            euler_message.linear_acceleration = quat_message.linear_acceleration;

            publisher_euler->publish(euler_message);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_quat;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_euler;
        hardware::Pigeon2 pigeon2imu;
};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CompassDataPublisher>());
    rclcpp::shutdown();
    return 0;
}
