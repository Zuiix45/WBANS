#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <px4_msgs/msg/position_setpoint.hpp>
#include <px4_msgs/msg/position_setpoint_triplet.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

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
        void waypointPub_callback();
        void currentPosSub_callback(const px4_msgs::msg::VehicleGlobalPosition msg);

        bool running; ///< Flag for running the route.

        std::vector<px4_msgs::msg::PositionSetpoint> waypoints; ///< List of waypoints defining the route.
        int currentWaypoint; ///< Index of the current waypoint.

        px4_msgs::msg::VehicleGlobalPosition currentPos; ///< Current position of the drone.
        std::vector<double> toleranceFactors_; ///< Tolerance factors for position checking.

        rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr currentPosSubscriber; ///< Subscriber for position updates.

        rclcpp::TimerBase::SharedPtr waypointPubTimer;
        rclcpp::Publisher<px4_msgs::msg::PositionSetpointTriplet>::SharedPtr waypointPublisher;
};
