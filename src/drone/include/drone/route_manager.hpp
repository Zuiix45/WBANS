#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * @class RouteManager
 * @brief Manages the route for a drone by handling waypoints and position updates.
 * 
 * This class inherits from rclcpp::Node and is responsible for managing the drone's route.
 * It parses waypoints, checks if the drone is on a point, and handles position updates.
 */
class RouteManager : public rclcpp::Node {
    public:
        /**
         * @brief Constructor for RouteManager.
         */
        explicit RouteManager();

        /**
         * @brief Checks if the drone is on the current waypoint.
         * 
         * @return True if the drone is on the current waypoint, false otherwise.
         */
        bool isOnPoint();

        bool shouldExit();

        void exit();

    private:
        void routeTimer_callback();
        void pos_callback(const std_msgs::msg::String::SharedPtr msg);

        bool running; ///< Flag for running the route.

        std::vector<std::string> waypoints_; ///< List of waypoints defining the route.
        uint nextWaypoint; ///< Index of the next waypoint.

        std::vector<double> currentPos; ///< Current position of the drone.
        std::vector<double> toleranceFactors_; ///< Tolerance factors for position checking.

        rclcpp::TimerBase::SharedPtr routeTimer; ///< Timer for managing the route.
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr routePublisher; ///< Publisher for route updates.

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr posSubscriber; ///< Subscriber for position updates.
};
