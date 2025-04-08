#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_patrol_srv/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class DirectionClient : public rclcpp::Node {
public:
  DirectionClient()
      : Node("direction_client"), service_done_(false), service_called_(false) {
    client_ = this->create_client<robot_patrol_srv::srv::GetDirection>(
        "/direction_service");
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&DirectionClient::send_async_request, this));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&DirectionClient::scan_callback, this,
                  std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Test Service Client Ready ");
  }

  bool is_service_done() const { return service_done_; }

  bool has_laser_data() const { return !laser_data_.ranges.empty(); }

  void send_async_request() {
    if (service_called_) {
      return;
    }

    timer_->cancel();
    service_called_ = true;

    if (!client_) {
      RCLCPP_ERROR(this->get_logger(), "Service client not initialized.");
      return;
    }

    if (!client_->wait_for_service(1s)) {
      RCLCPP_ERROR(this->get_logger(), "Service not available after waiting.");
      service_done_ = true;
      return;
    }

    auto request =
        std::make_shared<robot_patrol_srv::srv::GetDirection::Request>();
    request->laser_data = laser_data_;
    RCLCPP_INFO(this->get_logger(), "Test Service Requested");

    auto result_future = client_->async_send_request(
        request, std::bind(&DirectionClient::response_callback, this,
                           std::placeholders::_1));
  }

private:
  rclcpp::Client<robot_patrol_srv::srv::GetDirection>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool service_done_;
  bool service_called_;
  sensor_msgs::msg::LaserScan laser_data_;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (msg->ranges.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty laser scan data.");
      return;
    }
    laser_data_ = *msg;
  }

  void response_callback(
      rclcpp::Client<robot_patrol_srv::srv::GetDirection>::SharedFuture
          future) {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Test Service Response sent");
    service_done_ = true;
    service_called_ = false;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto service_client = std::make_shared<DirectionClient>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(service_client);

  while (!service_client->is_service_done()) {
    executor.spin_some();
  }

  rclcpp::shutdown();
  return 0;
}
