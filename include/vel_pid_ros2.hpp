#ifndef VEL_CONTROL_ROS2_HPP_
#define VEL_CONTROL_ROS2_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "pid_utils.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace vel_pid_ros2
{
    class VelPidROS2 :public rclcpp::Node
    {
        public:
        explicit VelPidROS2(const rclcpp::NodeOptions & node_options=rclcpp::NodeOptions());

        private:
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void control_callback();

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        geometry_msgs::msg::Twist::SharedPtr target_velocity_;
        geometry_msgs::msg::Twist::SharedPtr now_velocity_;

        float p_gain_, i_gain_, d_gain_, delta_time_;
        int control_freqency_;
        PID x_pid_;
        PID y_pid_;
        PID rotation_pid_;

        bool imu_flag_, cmd_flag_;
    };
}

#endif