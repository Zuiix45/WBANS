#include "drone/sensor_combined.hpp"

SensorCombinedListener::SensorCombinedListener() : Node("sensor_combined_listener"), running(true) {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    subscription_ = this->create_subscription<px4_msgs::msg::SensorCombined>("/fmu/out/sensor_combined", qos,
    [this](const px4_msgs::msg::SensorCombined::UniquePtr msg) {
        std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
        std::cout << "RECEIVED SENSOR COMBINED DATA"   << std::endl;
        std::cout << "============================="   << std::endl;
        std::cout << "ts: "          << msg->timestamp    << std::endl;
        std::cout << "gyro_rad[0]: " << msg->gyro_rad[0]  << std::endl;
        std::cout << "gyro_rad[1]: " << msg->gyro_rad[1]  << std::endl;
        std::cout << "gyro_rad[2]: " << msg->gyro_rad[2]  << std::endl;
        std::cout << "gyro_integral_dt: " << msg->gyro_integral_dt << std::endl;
        std::cout << "accelerometer_timestamp_relative: " << msg->accelerometer_timestamp_relative << std::endl;
        std::cout << "accelerometer_m_s2[0]: " << msg->accelerometer_m_s2[0] << std::endl;
        std::cout << "accelerometer_m_s2[1]: " << msg->accelerometer_m_s2[1] << std::endl;
        std::cout << "accelerometer_m_s2[2]: " << msg->accelerometer_m_s2[2] << std::endl;
        std::cout << "accelerometer_integral_dt: " << msg->accelerometer_integral_dt << std::endl;
    });
}

bool SensorCombinedListener::shouldExit() {
    return !running;
}

void SensorCombinedListener::exit() {
    running = false;
}

int main(int argc, char *argv[]) {
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

    auto node = std::make_shared<SensorCombinedListener>();

    std::thread node_thread([node]() {
        while (rclcpp::ok() && !node->shouldExit()) {
            rclcpp::spin_some(node);
        }
    });

    node_thread.join();
	rclcpp::shutdown();
	return 0;
}