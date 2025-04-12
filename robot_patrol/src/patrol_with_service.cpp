#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_patrol_srv/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("robot_patrol_node"), direction_(""), service_called_(false) {
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&Patrol::scan_callback, this, _1));
    client_ = this->create_client<robot_patrol_srv::srv::GetDirection>(
        "/direction_service");
    pub_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&Patrol::patrol_execute, this));
    move_cmd_.linear.x = 0.0;
    move_cmd_.angular.z = 0.0;
    RCLCPP_INFO(this->get_logger(), "Service Client Ready");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (msg->ranges.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty laser scan data.");
      return;
    }
    laser_data_ = *msg;
  }

  void patrol_execute() {
    if (laser_data_.ranges.empty()) {
      RCLCPP_WARN(this->get_logger(), "Laser data not received yet.");
      return;
    }

    // Get laser measurements for the forward direction
    float laser_forward = get_min_distance(laser_data_.ranges, 280, 400);

    if (laser_forward > 0.35) {
      // No obstacle in front, move forward
      move_cmd_.angular.z = 0.0;
      service_called_ = false;
    } else {
      // Call the service to get the next patrol direction
      if (!service_called_) {
        call_direction_service();
        service_called_ = true;
      }
    }
    move_cmd_.linear.x = 0.1;
    cmd_vel_pub_->publish(move_cmd_);
  }

  void call_direction_service() {
    if (!client_->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(),
                  "Service Client is not ready, cannot send request.");
      return;
    }

    auto request =
        std::make_shared<robot_patrol_srv::srv::GetDirection::Request>();
    request->laser_data = laser_data_;
    RCLCPP_INFO(this->get_logger(), "Patrol direction Service Requested");

    auto result_future = client_->async_send_request(
        request,
        std::bind(&Patrol::response_callback, this, std::placeholders::_1));
  }

  void response_callback(
      rclcpp::Client<robot_patrol_srv::srv::GetDirection>::SharedFuture
          future) {
    auto response = future.get();

    direction_ = response->direction;
    RCLCPP_INFO(this->get_logger(), "Service Response - Direction: %s",
                direction_.c_str());

    if (direction_ == "front") {
      move_cmd_.angular.z = 0.0;
    } else if (direction_ == "left") {
      move_cmd_.angular.z = .8;
    } else if (direction_ == "right") {
      move_cmd_.angular.z = -.8;
    }
  }

  float get_min_distance(const std::vector<float> &ranges, size_t start_idx,
                         size_t end_idx) {
    auto it =
        std::min_element(ranges.begin() + start_idx, ranges.begin() + end_idx,
                         [](float a, float b) { return a > 0.0f && a < b; });

    return (it != ranges.begin() + end_idx)
               ? *it
               : std::numeric_limits<float>::infinity();
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Client<robot_patrol_srv::srv::GetDirection>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  sensor_msgs::msg::LaserScan laser_data_;
  geometry_msgs::msg::Twist move_cmd_;
  std::string direction_;
  bool service_called_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}
