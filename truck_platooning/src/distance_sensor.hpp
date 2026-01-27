#ifndef DISTANCE_SENSOR_HPP
#define DISTANCE_SENSOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <optional>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>

class DistanceSensor {
public:
    DistanceSensor(std::shared_ptr<rclcpp::Node> node, const std::string& ns)
        : node_(node), namespace_(ns), lidar_distance_(std::nullopt),
          y_filter_min_(-1.0f), y_filter_max_(1.0f)
    {
        subscription_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/" + namespace_ + "/front_lidar", rclcpp::SensorDataQoS(),
            std::bind(&DistanceSensor::lidar_callback, this, std::placeholders::_1));
    }

    void update_lane_bounds(float y_min, float y_max) {
        y_filter_min_ = y_min;
        y_filter_max_ = y_max;
    }

    std::optional<float> get_distance() const {
        return lidar_distance_;
    }

private:
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        std::vector<float> valid_x;

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            float x = *iter_x;
            float y = -*iter_y;
            float z = *iter_z;

            if (x <= 0) continue;
            if (y < y_filter_min_ || y > y_filter_max_) continue;
            if (z <= -1.0f) continue;

            float angle = std::atan2(y, x);
            if (std::abs(angle) < M_PI / 180.0f * 25.0f) {
                valid_x.push_back(x);
            }
        }

        if (!valid_x.empty()) {
            lidar_distance_ = *std::min_element(valid_x.begin(), valid_x.end());
        } else {
            lidar_distance_ = std::nullopt;
        }
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::string namespace_;
    std::optional<float> lidar_distance_;
    float y_filter_min_, y_filter_max_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

#endif // DISTANCE_SENSOR_HPP
