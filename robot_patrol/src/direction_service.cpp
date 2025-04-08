#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_patrol_srv/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("direction_server") {
    service_ = this->create_service<robot_patrol_srv::srv::GetDirection>(
        "/direction_service",
        std::bind(&DirectionService::direction_callback, this, _1, _2));

    RCLCPP_INFO(this->get_logger(),
                "Service Server Ready to find direction for patrol.");
  }

private:
  void direction_callback(
      const std::shared_ptr<robot_patrol_srv::srv::GetDirection::Request>
          request,
      std::shared_ptr<robot_patrol_srv::srv::GetDirection::Response> response) {
    RCLCPP_INFO(this->get_logger(),
                "Service Requested - Finding direction for patrolling...");
    const std::vector<float> ranges = request->laser_data.ranges;

    size_t right_start_idx = 180;
    size_t front_start_idx = 301;
    size_t left_start_idx = 420;
    size_t left_end_idx = 540;

    auto is_valid = [](float value) {
      return !std::isinf(value) && value <= 30.0f;
    };

    float total_dist_sec_right = std::accumulate(
        ranges.begin() + right_start_idx, ranges.begin() + front_start_idx - 1,
        0.0f, [is_valid](float sum, float value) {
          return is_valid(value) ? sum + value : sum;
        });
    float total_dist_sec_front = std::accumulate(
        ranges.begin() + front_start_idx, ranges.begin() + left_start_idx - 1,
        0.0f, [is_valid](float sum, float value) {
          return is_valid(value) ? sum + value : sum;
        });
    float total_dist_sec_left = std::accumulate(
        ranges.begin() + left_start_idx, ranges.begin() + left_end_idx, 0.0f,
        [is_valid](float sum, float value) {
          return is_valid(value) ? sum + value : sum;
        });

    // Print the sums
    // std::cout << "Right sum: " << total_dist_sec_right << std::endl;
    // std::cout << "Front sum: " << total_dist_sec_front << std::endl;
    // std::cout << "Left sum: " << total_dist_sec_left << std::endl;

    float max_sum = std::max(
        {total_dist_sec_right, total_dist_sec_front, total_dist_sec_left});

    std::string direction;
    if (max_sum == total_dist_sec_right) {
      direction = "right";
    } else if (max_sum == total_dist_sec_front) {
      direction = "front";
    } else if (max_sum == total_dist_sec_left) {
      direction = "left";
    }

    response->direction = direction;
    RCLCPP_INFO(this->get_logger(), "Service Completed - Direction: %s",
                direction.c_str());
  }

  rclcpp::Service<robot_patrol_srv::srv::GetDirection>::SharedPtr service_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}