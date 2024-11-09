#include "drone/route_manager.hpp"

namespace {
  void createWaypoint(std::string waypointStr, px4_msgs::msg::PositionSetpoint &waypoint) {
    double lat = std::stod(waypointStr.substr(0, waypointStr.find(",")));
    waypointStr.erase(0, waypointStr.find(",") + 1);
    double lon = std::stod(waypointStr.substr(0, waypointStr.find(",")));
    waypointStr.erase(0, waypointStr.find(",") + 1);
    double alt = std::stod(waypointStr);

    waypoint.valid = true;
    waypoint.lat = lat;
    waypoint.lon = lon;
    waypoint.alt = alt;
  }
}

RouteManager::RouteManager()
    : Node("route_manager"), running(true), currentWaypoint(0) {

      std::vector<std::string> waypoints_;

      this->declare_parameter("waypoints", rclcpp::PARAMETER_STRING_ARRAY);
      this->declare_parameter("tolerance_factors", rclcpp::PARAMETER_DOUBLE_ARRAY);

      this->get_parameter("waypoints", waypoints_);
      this->get_parameter("tolerance_factors", toleranceFactors_);

      if (waypoints_.size() == 0) throw std::runtime_error("No waypoints found");

      for (auto waypointStr : waypoints_) {
        px4_msgs::msg::PositionSetpoint waypoint;
        createWaypoint(waypointStr, waypoint);
        waypoints.push_back(waypoint);
      }

      this->waypointPublisher = this->create_publisher<px4_msgs::msg::PositionSetpointTriplet>(
      "rt/fmu/out/position_setpoint_triplet", 10);

      this->waypointPubTimer = this->create_wall_timer(
      500ms, [this]{this->waypointPub_callback();});

      this->currentPosSubscriber = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
      "/fmu/in/aux_global_position", 10, [this](const px4_msgs::msg::VehicleGlobalPosition msg) {
        this->currentPosSub_callback(msg);
      });
    }

bool RouteManager::isOnPoint() {
  auto isClose = [](double tolerance, double aim, double current) {
    return ((aim - tolerance < current) && (aim + tolerance > current));
  };

  auto waypoint = waypoints[currentWaypoint];

  // Check if the drone is on the current waypoint
  return (isClose(toleranceFactors_[0], waypoint.lat, currentPos.lat) &&
          isClose(toleranceFactors_[1], waypoint.lon, currentPos.lon) &&
          isClose(toleranceFactors_[2], waypoint.alt, currentPos.alt));
}

bool RouteManager::shouldExit() {
  return !running;
}

void RouteManager::exit() {
  running = false;
}

void RouteManager::waypointPub_callback() {
  px4_msgs::msg::PositionSetpointTriplet msg;
  RCLCPP_INFO(this->get_logger(), "Current position: lat: %f, lon: %f, alt: %f", currentPos.lat, currentPos.lon, currentPos.alt);

  if (currentWaypoint == 0) {
    msg.current = waypoints[currentWaypoint];
    msg.next = waypoints[currentWaypoint + 1];

    msg.previous.valid = false;
  } 
  
  else if (currentWaypoint == (int)waypoints.size() - 1) {
    msg.previous = waypoints[currentWaypoint - 1];
    msg.current = waypoints[currentWaypoint];

    msg.next.valid = false;
  }

  else {
    msg.previous = waypoints[currentWaypoint - 1];
    msg.current = waypoints[currentWaypoint];
    msg.next = waypoints[currentWaypoint + 1];
  }
}

void RouteManager::currentPosSub_callback(const px4_msgs::msg::VehicleGlobalPosition msg) {
  currentPos = msg;
  RCLCPP_INFO(this->get_logger(), "Current position: lat: %f, lon: %f, alt: %f", currentPos.lat, currentPos.lon, currentPos.alt);
  if (isOnPoint()) {
    currentWaypoint++;
    if (currentWaypoint >= (int)waypoints.size()) {
      exit();
    }
  }
}

/* Main Function */

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RouteManager>();

  std::thread node_thread([node]() {
    while (rclcpp::ok() && !node->shouldExit()) {
      rclcpp::spin_some(node);
    }
  });

  node_thread.join();
  rclcpp::shutdown();
  return 0;
}