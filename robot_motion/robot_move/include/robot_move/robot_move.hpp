#pragma once

#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_navigation/action/go_to_xy_phi.hpp"
#include "play_motion2_msgs/action/play_motion2.hpp"

class RobotMove
{
public:
  explicit RobotMove(const std::string & node_name = "robot_move");
  ~RobotMove();

  // Move the robot base to (x, y) with orientation phi (degrees in map frame).
  // Blocks until navigation completes. Returns true on success.
  bool move_base(double x, double y, double phi);

  // Execute a named arm motion via play_motion2.
  // Blocks until the motion completes. Returns true on success.
  bool move_arm(const std::string & motion_name);

private:
  using GoToXYPhi   = base_navigation::action::GoToXYPhi;
  using PlayMotion2 = play_motion2_msgs::action::PlayMotion2;

  std::shared_ptr<rclcpp::Node>                   node_;
  rclcpp_action::Client<GoToXYPhi>::SharedPtr     nav_client_;
  rclcpp_action::Client<PlayMotion2>::SharedPtr   pm2_client_;

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
};
