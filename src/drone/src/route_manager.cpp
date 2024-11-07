#include "drone/route_manager.hpp"

namespace {
  void parseCoordsStr(std::string coordsStr, std::vector<double> &coords) {
    std::string delimiter = ",";
    size_t pos = 0;
    std::string token;
    int i = 0;
    while ((pos = coordsStr.find(delimiter)) != std::string::npos) {
      token = coordsStr.substr(0, pos);
      coords[i] = std::stod(token);
      coordsStr.erase(0, pos + delimiter.length());
      i++;
    }
    coords[i] = std::stod(coordsStr);
  }
}

RouteManager::RouteManager()
    : Node("route_manager"), running(true), nextWaypoint(0), currentPos({0.0, 0.0, 0.0}) {

      int bufferSize_;

      this->declare_parameter("waypoints", rclcpp::PARAMETER_STRING_ARRAY);
      this->declare_parameter("tolerance_factors", rclcpp::PARAMETER_DOUBLE_ARRAY);
      this->declare_parameter("buffer_size", 10);

      this->get_parameter("waypoints", waypoints_);
      this->get_parameter("tolerance_factors", toleranceFactors_);
      this->get_parameter("buffer_size", bufferSize_);

      if (this->waypoints_.size() == 0) throw std::runtime_error("No waypoints found");

      this->routePublisher = this->create_publisher<std_msgs::msg::String>("waypoint_transmit", bufferSize_);
      this->routeTimer = this->create_wall_timer(
      500ms, std::bind(&RouteManager::routeTimer_callback, this));

      this->posSubscriber = this->create_subscription<std_msgs::msg::String>(
      "pos_transmit", bufferSize_, std::bind(&RouteManager::pos_callback, this, _1));
    }

bool RouteManager::isOnPoint() {
  auto isClose = [](double tolerance, double aim, double current) {
    return ((aim - tolerance < current) && (aim + tolerance > current));
  };

  std::string waypointStr = this->waypoints_[this->nextWaypoint];
  std::vector<double> waypoint(3);
  parseCoordsStr(waypointStr, waypoint);

  bool result = (isClose(toleranceFactors_[0], waypoint[0], currentPos[0]) &&
            isClose(toleranceFactors_[1], waypoint[1], currentPos[1]) &&
            isClose(toleranceFactors_[2], waypoint[2], currentPos[2]));

  waypoint.clear();
  waypoint.shrink_to_fit();

  return result;
}

bool RouteManager::shouldExit() {
  return !running;
}

void RouteManager::exit() {
  running = false;
}

void RouteManager::routeTimer_callback() {
  auto message = std_msgs::msg::String();
  message.data = waypoints_[nextWaypoint];
  RCLCPP_INFO(this->get_logger(), message.data.c_str());
  this->routePublisher->publish(message);
}

void RouteManager::pos_callback(const std_msgs::msg::String::SharedPtr msg) {
  // Parse the received position message and update the current position
  parseCoordsStr(msg->data, this->currentPos);

  if (isOnPoint()) {
    nextWaypoint++;

    if (nextWaypoint == waypoints_.size()) {
      RCLCPP_INFO(this->get_logger(), "Route completed");
      this->exit();
    }

    RCLCPP_INFO(this->get_logger(), "Next waypoint: %s", waypoints_[nextWaypoint].c_str());
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