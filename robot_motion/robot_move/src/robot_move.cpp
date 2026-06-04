#include "robot_move/robot_move.hpp"

#include <chrono>
#include <future>
#include <mutex>

using namespace std::chrono_literals;

// ─── Constructor / Destructor ────────────────────────────────────────────────

RobotMove::RobotMove(const std::string & node_name)
: node_(std::make_shared<rclcpp::Node>(node_name))
{
  nav_client_ = rclcpp_action::create_client<GoToXYPhi>(node_, "go_to_xy_phi");
  pm2_client_ = rclcpp_action::create_client<PlayMotion2>(node_, "/play_motion2");

  executor_.add_node(node_);
  spin_thread_ = std::thread([this]() { executor_.spin(); });
}

RobotMove::~RobotMove()
{
  executor_.cancel();
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

// ─── move_base ───────────────────────────────────────────────────────────────

bool RobotMove::move_base(double x, double y, double phi)
{
  if (!nav_client_->wait_for_action_server(10s)) {
    RCLCPP_ERROR(node_->get_logger(), "go_to_xy_phi action server not available");
    return false;
  }

  GoToXYPhi::Goal goal;
  goal.x   = x;
  goal.y   = y;
  goal.phi = phi;

  std::promise<bool> done;
  auto fut = done.get_future();
  std::once_flag settled;

  auto opts = rclcpp_action::Client<GoToXYPhi>::SendGoalOptions();

  opts.goal_response_callback =
    [&](const rclcpp_action::ClientGoalHandle<GoToXYPhi>::SharedPtr & handle) {
      if (!handle) {
        std::call_once(settled, [&]() { done.set_value(false); });
        RCLCPP_ERROR(node_->get_logger(), "move_base: goal rejected by server");
      }
    };

  opts.feedback_callback =
    [this](rclcpp_action::ClientGoalHandle<GoToXYPhi>::SharedPtr,
           const std::shared_ptr<const GoToXYPhi::Feedback> fb) {
      RCLCPP_INFO(node_->get_logger(),
        "move_base: distance remaining %.2f m", fb->distance_remaining);
    };

  opts.result_callback =
    [&](const rclcpp_action::ClientGoalHandle<GoToXYPhi>::WrappedResult & result) {
      bool success = (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                     && result.result->success;
      std::call_once(settled, [&]() { done.set_value(success); });
      if (success) {
        RCLCPP_INFO(node_->get_logger(), "move_base: succeeded");
      } else {
        RCLCPP_ERROR(node_->get_logger(), "move_base: failed — %s",
          result.result->message.c_str());
      }
    };

  nav_client_->async_send_goal(goal, opts);
  return fut.get();
}

// ─── move_arm ────────────────────────────────────────────────────────────────

bool RobotMove::move_arm(const std::string & motion_name)
{
  if (!pm2_client_->wait_for_action_server(10s)) {
    RCLCPP_ERROR(node_->get_logger(), "play_motion2 action server not available");
    return false;
  }

  PlayMotion2::Goal goal;
  goal.motion_name   = motion_name;
  goal.skip_planning = false;

  std::promise<bool> done;
  auto fut = done.get_future();
  std::once_flag settled;

  auto opts = rclcpp_action::Client<PlayMotion2>::SendGoalOptions();

  opts.goal_response_callback =
    [&](const rclcpp_action::ClientGoalHandle<PlayMotion2>::SharedPtr & handle) {
      if (!handle) {
        std::call_once(settled, [&]() { done.set_value(false); });
        RCLCPP_ERROR(node_->get_logger(),
          "move_arm: goal '%s' rejected by server", motion_name.c_str());
      }
    };

  opts.result_callback =
    [&](const rclcpp_action::ClientGoalHandle<PlayMotion2>::WrappedResult & result) {
      bool success = (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                     && result.result->success;
      std::call_once(settled, [&]() { done.set_value(success); });
      if (success) {
        RCLCPP_INFO(node_->get_logger(),
          "move_arm: motion '%s' succeeded", motion_name.c_str());
      } else {
        RCLCPP_ERROR(node_->get_logger(),
          "move_arm: motion '%s' failed — %s",
          motion_name.c_str(), result.result->error.c_str());
      }
    };

  pm2_client_->async_send_goal(goal, opts);
  return fut.get();
}
