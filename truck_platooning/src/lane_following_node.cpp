// lane_following_node.cpp - Final Version (C++) with Fixes
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include "pid_controller.hpp"
#include "lane_detect.hpp"
#include "command_publisher.hpp"
#include "distance_sensor.hpp"
#include "platooning_manager.hpp"

#include <unordered_map>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <functional>
#include "rclcpp/qos.hpp"  // 반드시 포함

using std::placeholders::_1;


class LaneFollowingNode : public rclcpp::Node {
public:
    LaneFollowingNode()
    : Node("lane_following_node"),
      emergency_stop_(false),
      target_velocity_(14.5f),
      leader_steering_(0.0f),
      setup_done_(false) {

        // QoS를 depth 10으로 하고, RELIABILITY를 BEST_EFFORT로 설정하려면:
        auto qos_camera = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data))
                            .best_effort();


        for (int i = 0; i < 3; ++i) {
            std::string ns = "truck" + std::to_string(i);

            // Publishers
            steer_publishers_[i] = this->create_publisher<std_msgs::msg::Float32>("/" + ns + "/steer_control", 10);
            throttle_publishers_[i] = this->create_publisher<std_msgs::msg::Float32>("/" + ns + "/throttle_control", 10);

            // Subscriptions
            velocity_subscribers_[i] = this->create_subscription<std_msgs::msg::Float32>(
                "/" + ns + "/velocity", 10,
                [this, i](const std_msgs::msg::Float32::SharedPtr msg) {
                    current_velocities_[i] = msg->data;
                });

            camera_subscribers_[i] = this->create_subscription<sensor_msgs::msg::Image>(
                "/" + ns + "/front_camera", qos_camera,
                [this, i](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
                    this->process_camera(msg, i);
                });

            pid_controllers_[i] = std::make_shared<PIDController>(0.8f, 0.01f, 0.2f);
        }

        setup_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                if (!this->setup_done_) {
                    for (int i = 0; i < 3; ++i) {
                        std::string ns = "truck" + std::to_string(i);
                        distance_sensors_[i] = std::make_shared<DistanceSensor>(this->shared_from_this(), ns);
                        if (i != 0)
                            platooning_managers_[i] = std::make_shared<PlatooningManager>(ns);
                    }
                    this->setup_done_ = true;
                }
            });

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&LaneFollowingNode::publish_commands, this));
    }


private:
    void process_camera(const sensor_msgs::msg::Image::ConstSharedPtr msg, int truck_id) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat bev = applyBirdsEyeView(cv_ptr->image);
        auto [lane_img, lane_coords] = detectLane(bev, truck_id, {}, {});

        float steer = calculateSteering(
            lane_coords.first,
            lane_coords.second,
            bev.cols,
            [this, truck_id](float error) {
                return pid_controllers_[truck_id]->compute(error);
            }
        );

        std_msgs::msg::Float32 steer_msg;
        steer_msg.data = steer;
        steer_publishers_[truck_id]->publish(steer_msg);

        if (truck_id == 0) leader_steering_ = steer;
    }

    void publish_commands() {
        if (!setup_done_) return; // Sensor/manager가 아직 초기화되지 않은 경우

        auto front_dist_opt = distance_sensors_[0]->get_distance();
        emergency_stop_ = false;
        float front_distance = front_dist_opt.value_or(100.0f);

        if (front_distance < 5.0f) {
            emergency_stop_ = true;
            RCLCPP_WARN(this->get_logger(), "[Truck 0] Emergency Stop! Distance: %.2f", front_distance);
        }

        for (int i = 0; i < 3; ++i) {
            std_msgs::msg::Float32 msg;

            if (i == 0) {
                float target_vel = target_velocity_;
                if (emergency_stop_) {
                    msg.data = -1.0f;
                    throttle_publishers_[i]->publish(msg);
                    continue;
                }

                if (front_distance <= 10.0f) target_vel = 0.0f;
                else if (front_distance < 30.0f)
                    target_vel = (front_distance - 10.0f) / 20.0f * (target_velocity_ - 3.0f);

                msg.data = target_vel;
                throttle_publishers_[i]->publish(msg);
            } else {
                auto lidar_dist = distance_sensors_[i]->get_distance();
                if (lidar_dist.has_value()) {
                    if (emergency_stop_) {
                        float leader_vel = current_velocities_[0];
                        if (std::abs(leader_steering_) > 20.0f) leader_vel *= 0.5f;
                        msg.data = leader_vel;
                        throttle_publishers_[i]->publish(msg);
                    } else {
                        platooning_managers_[i]->update_distance(lidar_dist.value());
                    }
                }
            }
        }
    }

    bool emergency_stop_;
    bool setup_done_ = false;
    float target_velocity_;
    float leader_steering_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr setup_timer_;

    std::unordered_map<int, float> current_velocities_;
    std::unordered_map<int, std::shared_ptr<PIDController>> pid_controllers_;
    std::unordered_map<int, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> steer_publishers_;
    std::unordered_map<int, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> throttle_publishers_;
    std::unordered_map<int, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> camera_subscribers_;
    std::unordered_map<int, rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> velocity_subscribers_;
    std::unordered_map<int, std::shared_ptr<DistanceSensor>> distance_sensors_;
    std::unordered_map<int, std::shared_ptr<PlatooningManager>> platooning_managers_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneFollowingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
