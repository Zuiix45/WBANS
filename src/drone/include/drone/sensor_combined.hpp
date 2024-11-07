#pragma once

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>

using namespace std::chrono_literals;

/**
 * @brief Sensor Combined uORB topic data callback
 */
class SensorCombinedListener : public rclcpp::Node {
    public:
        explicit SensorCombinedListener();
    
        bool shouldExit();

        void exit();
    private:
        bool running;

        rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr subscription_;
};
