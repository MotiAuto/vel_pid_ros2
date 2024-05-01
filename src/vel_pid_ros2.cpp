#include "vel_pid_ros2.hpp"

namespace vel_pid_ros2
{
    VelPidROS2::VelPidROS2(const rclcpp::NodeOptions & node_options)
    :Node("vel_control_ros2", node_options)
    {
        cmd_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            0,
            std::bind(&VelPidROS2::cmd_callback, this, _1));
        
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu",
            0,
            std::bind(&VelPidROS2::imu_callback, this, _1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/output", 0);

        timer_ = this->create_wall_timer(get_milli(control_freqency_), std::bind(&VelPidROS2::control_callback, this));

        this->declare_parameter("p_gain", 0.1);
        this->declare_parameter("i_gain", 0.1);
        this->declare_parameter("d_gain", 0.1);

        this->get_parameter("p_gain", p_gain_);
        this->get_parameter("i_gain", i_gain_);
        this->get_parameter("d_gain", d_gain_);

        this->declare_parameter("control_freqency", 10);

        x_pid_.set_gain(p_gain_, i_gain_, d_gain_);
        y_pid_.set_gain(p_gain_, i_gain_, d_gain_);
        rotation_pid_.set_gain(p_gain_, i_gain_, d_gain_);

        delta_time_ = 1.0 / (float)(control_freqency_);

        RCLCPP_INFO(this->get_logger(), "Start VelContollerROS2. control_freqency:%d", control_freqency_);
    }

    std::chrono::milliseconds VelPidROS2::get_milli(int freq)
    {
        return std::chrono::milliseconds(freq);
    }

    void VelPidROS2::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        now_velocity_->linear.x += msg->linear_acceleration.x * delta_time_;
        now_velocity_->linear.y += msg->linear_acceleration.y * delta_time_;
        now_velocity_->angular.z = msg->angular_velocity.z;
    }

    void VelPidROS2::cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        target_velocity_ = msg;
    }

    void VelPidROS2::control_callback()
    {
        auto result = geometry_msgs::msg::Twist();

        result.linear.x = x_pid_.calc_pid(target_velocity_->linear.x, now_velocity_->linear.x, delta_time_);
        result.linear.y = y_pid_.calc_pid(target_velocity_->linear.y, now_velocity_->linear.y, delta_time_);
        result.angular.z = rotation_pid_.calc_pid(target_velocity_->angular.z, now_velocity_->angular.z, delta_time_);

        publisher_->publish(result);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(vel_pid_ros2::VelPidROS2);