#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("robot_patrol_node"), direction_(0.0) {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&Patrol::scan_callback, this, std::placeholders::_1));
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    if (msg->ranges.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty laser scan data.");
      return;
    }

    // Get laser measurements for the left, right, and forward directions
    this->laser_right = get_min_distance(msg->ranges, 180, 190);
    this->laser_forward = get_min_distance(msg->ranges, 300, 420);
    this->laser_left = get_min_distance(msg->ranges, 530, 540);

    RCLCPP_INFO(this->get_logger(), "[LEFT] = '%f'", this->laser_left);
    RCLCPP_INFO(this->get_logger(), "[RIGHT] = '%f'", this->laser_right);
    RCLCPP_INFO(this->get_logger(), "[FORWARD] = '%f'", this->laser_forward);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  geometry_msgs::msg::Twist move_cmd_;
  float laser_left;
  float laser_right;
  float laser_forward;
  float direction_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}
