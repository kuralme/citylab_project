#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_patrol/action/go_to_pose.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>

using std::placeholders::_1;
using std::placeholders::_2;

class GoToPose : public rclcpp::Node {
public:
  using GoToPoseActionType = robot_patrol::action::GoToPose;
  using GoalHandleGoToPose =
      rclcpp_action::ServerGoalHandle<GoToPoseActionType>;

  explicit GoToPose(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("robot_patrol_action_server", options) {
    this->action_server_ = rclcpp_action::create_server<GoToPoseActionType>(
        this, "go_to_pose", std::bind(&GoToPose::handle_goal, this, _1, _2),
        std::bind(&GoToPose::handle_cancel, this, _1),
        std::bind(&GoToPose::handle_accepted, this, _1));
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&GoToPose::odom_callback, this, _1));

    RCLCPP_INFO(this->get_logger(),
                "GoToPose Action Service Ready. Waiting for a goal...");
  }

private:
  rclcpp_action::Server<GoToPoseActionType>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  geometry_msgs::msg::Pose2D robot_pose2d_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_pose2d_.x = msg->pose.pose.position.x;
    robot_pose2d_.y = msg->pose.pose.position.y;

    tf2::Quaternion quat(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    robot_pose2d_.theta = yaw;
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPoseActionType::Goal> goal) {
    RCLCPP_INFO(this->get_logger(),
                "Action Called - Goal position: x = %.2f, y = "
                "%.2f, theta = %.2f",
                goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);

    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GoToPoseActionType::Feedback>();
    auto &current_pos = feedback->current_pos;
    auto result = std::make_shared<GoToPoseActionType::Result>();

    geometry_msgs::msg::Twist move;
    rclcpp::Rate loop_rate(10);

    double goal_x = goal->goal_pos.x;
    double goal_y = goal->goal_pos.y;
    double goal_theta = goal->goal_pos.theta;

    while (rclcpp::ok()) {
      // Check if the goal has been canceled
      if (goal_handle->is_canceling()) {
        result->status = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // Calculate the error between current and goal positions
      double delta_x = goal_x - robot_pose2d_.x;
      double delta_y = goal_y - robot_pose2d_.y;
      double distance = std::sqrt(delta_x * delta_x + delta_y * delta_y);

      double angle_to_goal = std::atan2(delta_y, delta_x);
      double delta_theta = angle_to_goal - robot_pose2d_.theta;

      if (delta_theta > M_PI) {
        delta_theta -= 2 * M_PI;
      } else if (delta_theta < -M_PI) {
        delta_theta += 2 * M_PI;
      }

      // Proportional control for linear and angular velocity
      double lin_vel = 0.2;
      double ang_vel = 1.5 * delta_theta;

      // Stop the robot if reached the goal
      if (distance < 0.1) {
        lin_vel = 0.0;

        // Compute orientation error
        double theta_error = goal_theta - robot_pose2d_.theta;
        if (theta_error > M_PI)
          theta_error -= 2 * M_PI;
        if (theta_error < -M_PI)
          theta_error += 2 * M_PI;

        if (std::abs(theta_error) > 0.05) {
          // Still need to rotate
          ang_vel = 0.8 * theta_error;
        } else {
          // Both position and orientation are good -> finish
          ang_vel = 0.0;
          result->status = true;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(),
                      "Action Completed - Goal and orientation reached.");
          return;
        }
      }

      // Update current position for feedback
      current_pos.x = robot_pose2d_.x;
      current_pos.y = robot_pose2d_.y;
      current_pos.theta = robot_pose2d_.theta;
      goal_handle->publish_feedback(feedback);

      // Publish velocities
      move.linear.x = lin_vel;
      move.angular.z = ang_vel;
      cmd_vel_pub_->publish(move);

      loop_rate.sleep();
    }
  }
}; // class GoToPoseAction

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto action_server = std::make_shared<GoToPose>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
