#pragma once

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/debug_vect.hpp>

using namespace std::chrono_literals;

class DebugVectAdvertiser : public rclcpp::Node {
    public:
        explicit DebugVectAdvertiser();

        bool shouldExit();

        void exit();
    private:
        bool running;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<px4_msgs::msg::DebugVect>::SharedPtr publisher_;
};