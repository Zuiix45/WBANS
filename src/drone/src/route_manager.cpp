#include "drone/route_manager.hpp"

#define getTimeStamp() std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count()

namespace {
  void createWaypoint(std::string waypointStr, px4_msgs::msg::PositionSetpoint &waypoint) {
    double lat = std::stod(waypointStr.substr(0, waypointStr.find(",")));
    waypointStr.erase(0, waypointStr.find(",") + 1);
    double lon = std::stod(waypointStr.substr(0, waypointStr.find(",")));
    waypointStr.erase(0, waypointStr.find(",") + 1);
    double alt = std::stod(waypointStr);

    waypoint.valid = true;
    waypoint.type = 0; // Position Setpoint type

    waypoint.vx = 0.0;
    waypoint.vy = 0.0;
    waypoint.vz = 0.0;

    waypoint.lat = lat;
    waypoint.lon = lon;
    waypoint.alt = alt;
    waypoint.yaw = 0.0;

    waypoint.loiter_radius = 0.0;
    waypoint.loiter_minor_radius = 0.0;
    waypoint.loiter_direction_counter_clockwise = false;
    waypoint.loiter_orientation = 0.0;
    waypoint.loiter_pattern = 0;

    waypoint.acceptance_radius = 0.0;
    waypoint.alt_acceptance_radius = 0.0;
    
    waypoint.cruising_speed = 0.0;
    waypoint.gliding_enabled = false;
    waypoint.cruising_throttle = 0.0;
  }
}

RouteManager::RouteManager()
    : Node("route_manager"), running(true), currentWaypoint(0) {

      rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

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
      "/fmu/out/position_setpoint_triplet", 10);

      this->waypointPubTimer = this->create_wall_timer(
      500ms, [this]{this->waypointPub_callback();});

      this->currentPosSubscriber = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
      "/fmu/in/vehicle_global_position", qos, [this](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
        this->currentPosSub_callback(msg);
      });
    }

bool RouteManager::isOnPoint(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
  auto isClose = [](double tolerance, double aim, double current) {
    return ((aim - tolerance < current) && (aim + tolerance > current));
  };

  auto waypoint = waypoints[currentWaypoint];

  // Check if the drone is on the current waypoint
  return (isClose(toleranceFactors_[0], waypoint.lat, msg->lat) &&
          isClose(toleranceFactors_[1], waypoint.lon, msg->lon) &&
          isClose(toleranceFactors_[2], waypoint.alt, msg->alt));
}

bool RouteManager::shouldExit() {
  return !running;
}

void RouteManager::exit() {
  running = false;
}

void RouteManager::waypointPub_callback() {
  px4_msgs::msg::PositionSetpointTriplet msg;

  if (currentWaypoint == 0) {
    px4_msgs::msg::PositionSetpoint previous_; // Create a filler waypoint for the previous point
    createWaypoint("0, 0, 0", previous_);

    msg.previous = previous_;
    msg.current = waypoints[currentWaypoint];
    msg.next = waypoints[currentWaypoint + 1];

    msg.previous.valid = false;

    msg.previous.timestamp = getTimeStamp();
    msg.current.timestamp = getTimeStamp();
    msg.next.timestamp = getTimeStamp();
  } 
  
  else if (currentWaypoint == (int)waypoints.size() - 1) {
    px4_msgs::msg::PositionSetpoint next_; // Create a filler waypoint for the next point
    createWaypoint("0, 0, 0", next_);

    msg.previous = waypoints[currentWaypoint - 1];
    msg.current = waypoints[currentWaypoint];
    msg.next = next_;

    msg.next.valid = false;

    msg.previous.timestamp = getTimeStamp();
    msg.current.timestamp = getTimeStamp();
    msg.next.timestamp = getTimeStamp();
  }

  else {
    msg.previous = waypoints[currentWaypoint - 1];
    msg.current = waypoints[currentWaypoint];
    msg.next = waypoints[currentWaypoint + 1];

    msg.previous.timestamp = getTimeStamp();
    msg.current.timestamp = getTimeStamp();
    msg.next.timestamp = getTimeStamp();
  }

  this->waypointPublisher->publish(msg);
}

void RouteManager::currentPosSub_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Current position: lat: %f, lon: %f, alt: %f", msg->lat, msg->lon, msg->alt);
  if (this->isOnPoint(msg)) {
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