#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("robot_patrol_node") {
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&Patrol::scan_callback, this, std::placeholders::_1));
    pub_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&Patrol::patrol_execute, this));
    move_cmd_.linear.x = 0.0;
    move_cmd_.angular.z = 0.0;
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    if (msg->ranges.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty laser scan data.");
      return;
    }
    laser_ranges_ = msg->ranges;
  }

  void patrol_execute() {
    if (laser_ranges_.empty()) {
      RCLCPP_WARN(this->get_logger(), "Laser data not received yet.");
      return;
    }

    // Get laser measurements for the forward direction
    float laser_forward = get_min_distance(laser_ranges_, 280, 400);

    if (laser_forward > 0.35) {
      // No obstacle in front, move forward
      move_cmd_.angular.z = 0.0;
    } else {
      // Find the empty space and turn
      float direction_ = find_patrol_direction(laser_ranges_);
      move_cmd_.angular.z = direction_ / 2;
    }
    move_cmd_.linear.x = 0.1;
    cmd_vel_pub_->publish(move_cmd_);
  }

  float find_patrol_direction(const std::vector<float> &ranges) {
    size_t start_idx = 180;
    size_t end_idx = 540;

    // Find the maximum distance in the range
    int max_idx = get_max_dist_Idx(ranges, start_idx, end_idx);

    // Corresponding angle ref to front [rad]
    int angle_deg = (max_idx - 360) / 2;
    return angle_deg * M_PI / 180;
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

  int get_max_dist_Idx(const std::vector<float> &vec, size_t start_idx,
                       size_t end_idx) {
    float max_value = -std::numeric_limits<float>::infinity();
    int max_index = -1;

    // Ensure the range is valid
    if (start_idx >= vec.size() || end_idx >= vec.size() ||
        start_idx > end_idx) {
      std::cerr << "Invalid vector size!" << std::endl;
      return -1;
    }

    for (size_t i = start_idx; i <= end_idx; ++i) {
      if (!std::isinf(vec[i]) && vec[i] > max_value) {
        max_value = vec[i];
        max_index = i;
      }
    }

    return max_index;
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  geometry_msgs::msg::Twist move_cmd_;
  std::vector<float> laser_ranges_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}