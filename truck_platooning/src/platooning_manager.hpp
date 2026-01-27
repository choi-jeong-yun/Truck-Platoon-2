#ifndef PLATOONING_MANAGER_HPP
#define PLATOONING_MANAGER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include <string>
#include <optional>
#include <algorithm>
#include <cmath>

class PlatooningManager : public rclcpp::Node
{
public:
    PlatooningManager(const std::string& namespace_)
    : Node("platooning_manager_" + namespace_), namespace_(namespace_)
    {
        throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/" + namespace_ + "/throttle_control", 10);

        target_distance_ = 10.0f;
        safe_distance_ = 8.0f;
        kp_ = 0.3f;
        ki_ = 0.01f;
        kd_ = 1.8f;

        integral_ = 0.0f;
        prev_error_ = 0.0f;
        prev_time_ = this->get_clock()->now();  // ROS 시간 초기화
    }

    void update_distance(float lidar_distance, bool emergency_stop = false)
    {
        lidar_distance_ = lidar_distance;
        control_speed(emergency_stop);
    }

private:
    void control_speed(bool emergency_stop = false)
    {
        if (emergency_stop) {
            RCLCPP_INFO(this->get_logger(), "[%s] 비상 정지", namespace_.c_str());
            std_msgs::msg::Float32 msg;
            msg.data = -1.0f;
            throttle_pub_->publish(msg);
            return;
        }

        if (!lidar_distance_.has_value()) {
            RCLCPP_WARN(this->get_logger(), "[%s] LiDAR 거리 정보 없음, 제어 수행 불가", namespace_.c_str());
            return;
        }

        rclcpp::Time current_time = this->get_clock()->now();
        rclcpp::Duration dt = current_time - prev_time_;
        double dt_sec = dt.seconds();
        if (dt_sec <= 0.0) dt_sec = 1.0;

        float error = lidar_distance_.value() - target_distance_;
        integral_ += error * dt_sec;
        float derivative = (error - prev_error_) / dt_sec;

        float pid_output = kp_ * error + ki_ * integral_ + kd_ * derivative;

        if (lidar_distance_.value() < safe_distance_) {
            pid_output = -1.0f;
        }

        pid_output = std::clamp(pid_output, -1.0f, 1.0f);

        std_msgs::msg::Float32 msg;
        msg.data = pid_output;
        throttle_pub_->publish(msg);

        prev_error_ = error;
        prev_time_ = current_time;
    }

    std::string namespace_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;

    std::optional<float> lidar_distance_;
    float target_distance_;
    float safe_distance_;
    float kp_, ki_, kd_;
    float integral_;
    float prev_error_;
    rclcpp::Time prev_time_;
};

#endif  // PLATOONING_MANAGER_HPP
