#ifndef COMMAND_PUBLISHER_HPP
#define COMMAND_PUBLISHER_HPP

#include <vector>
#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

void publish_commands(
    const std::vector<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr>& throttle_publishers,
    const std::vector<float>& current_velocities,
    float target_velocity,
    const std::vector<float>& last_steering,
    bool emergency_stop = false
) {
    for (size_t truck_id = 0; truck_id < throttle_publishers.size(); ++truck_id) {
        std_msgs::msg::Float32 msg;

        if (truck_id == 0 && emergency_stop) {
            msg.data = -1.0f;  // throttle = 0
            throttle_publishers[truck_id]->publish(msg);
            continue;
        }

        float throttle_value = 0.8f;
        if (current_velocities[truck_id] > target_velocity) {
            throttle_value = 0.0f;
        }
        if (std::abs(last_steering[truck_id]) > 3.0f) {
            throttle_value *= 0.8f;
        }

        msg.data = throttle_value;
        throttle_publishers[truck_id]->publish(msg);
    }
}

#endif // COMMAND_PUBLISHER_HPP
