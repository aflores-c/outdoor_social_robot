#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <base_navigation/action/go_to_xy_phi.hpp>
#include <cmath>
#include <memory>

class NavigateToPoseServer : public rclcpp::Node
{
public:
    using GoToXYPhi     = base_navigation::action::GoToXYPhi;
    using GoalHandleXY  = rclcpp_action::ServerGoalHandle<GoToXYPhi>;
    using Nav2Action    = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<Nav2Action>;

    NavigateToPoseServer() : Node("navigate_to_pose_server")
    {
        nav2_client_ = rclcpp_action::create_client<Nav2Action>(this, "navigate_to_pose");

        action_server_ = rclcpp_action::create_server<GoToXYPhi>(
            this,
            "go_to_xy_phi",
            std::bind(&NavigateToPoseServer::handle_goal,   this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&NavigateToPoseServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&NavigateToPoseServer::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "GoToXYPhi action server ready");
    }

private:
    rclcpp_action::Server<GoToXYPhi>::SharedPtr action_server_;
    rclcpp_action::Client<Nav2Action>::SharedPtr nav2_client_;

    // --- Server callbacks ---

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const GoToXYPhi::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(),
            "Received goal: x=%.3f  y=%.3f  phi=%.1f deg", goal->x, goal->y, goal->phi);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleXY> /*goal_handle*/)
    {
        RCLCPP_INFO(this->get_logger(), "Cancel request received");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleXY> goal_handle)
    {
        std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
    }

    // --- Execution ---

    void execute(const std::shared_ptr<GoalHandleXY> goal_handle)
    {
        const auto & goal = goal_handle->get_goal();

        if (!nav2_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "nav2 NavigateToPose server not available");
            auto result = std::make_shared<GoToXYPhi::Result>();
            result->success = false;
            result->message = "NavigateToPose server unavailable";
            goal_handle->abort(result);
            return;
        }

        // Build nav2 goal
        double phi_rad = goal->phi * M_PI / 180.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, phi_rad);

        Nav2Action::Goal nav2_goal;
        nav2_goal.pose.header.frame_id    = "map";
        nav2_goal.pose.header.stamp       = this->now();
        nav2_goal.pose.pose.position.x    = goal->x;
        nav2_goal.pose.pose.position.y    = goal->y;
        nav2_goal.pose.pose.position.z    = 0.0;
        nav2_goal.pose.pose.orientation   = tf2::toMsg(q);

        // Shared state for synchronisation between callbacks and this thread
        bool nav2_done     = false;
        bool nav2_success  = false;
        std::mutex mtx;
        std::condition_variable cv;

        auto send_opts = rclcpp_action::Client<Nav2Action>::SendGoalOptions();

        send_opts.feedback_callback =
            [this, goal_handle, &mtx](
                GoalHandleNav::SharedPtr,
                const std::shared_ptr<const Nav2Action::Feedback> fb)
            {
                std::lock_guard<std::mutex> lock(mtx);
                auto feedback = std::make_shared<GoToXYPhi::Feedback>();
                feedback->distance_remaining = fb->distance_remaining;
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(),
                    "Distance remaining: %.2f m", fb->distance_remaining);
            };

        send_opts.result_callback =
            [&](const GoalHandleNav::WrappedResult & result)
            {
                std::unique_lock<std::mutex> lock(mtx);
                nav2_success = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
                nav2_done    = true;
                cv.notify_one();
            };

        // Cancel nav2 goal if our goal is cancelled
        std::shared_ptr<GoalHandleNav> nav2_goal_handle;
        auto goal_response_future = nav2_client_->async_send_goal(nav2_goal, send_opts);

        // Wait for goal to be accepted
        if (rclcpp::spin_until_future_complete(
                this->get_node_base_interface(), goal_response_future)
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send goal to nav2");
            auto result = std::make_shared<GoToXYPhi::Result>();
            result->success = false;
            result->message = "Failed to send goal to NavigateToPose";
            goal_handle->abort(result);
            return;
        }

        nav2_goal_handle = goal_response_future.get();
        if (!nav2_goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "nav2 rejected the goal");
            auto result = std::make_shared<GoToXYPhi::Result>();
            result->success = false;
            result->message = "Goal rejected by NavigateToPose";
            goal_handle->abort(result);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "nav2 accepted goal — navigating...");

        // Wait for nav2 to finish (or our goal to be cancelled)
        {
            std::unique_lock<std::mutex> lock(mtx);
            cv.wait(lock, [&] {
                if (goal_handle->is_canceling()) {
                    nav2_client_->async_cancel_goal(nav2_goal_handle);
                }
                return nav2_done;
            });
        }

        auto result = std::make_shared<GoToXYPhi::Result>();
        if (goal_handle->is_canceling()) {
            result->success = false;
            result->message = "Navigation canceled";
            goal_handle->canceled(result);
            RCLCPP_WARN(this->get_logger(), "Navigation canceled");
        } else if (nav2_success) {
            result->success = true;
            result->message = "Navigation succeeded";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
        } else {
            result->success = false;
            result->message = "Navigation failed";
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Navigation failed");
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigateToPoseServer>());
    rclcpp::shutdown();
    return 0;
}
