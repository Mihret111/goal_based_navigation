#include "nav_system/nav_action_server_component.hpp"  // Including the header file 

#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <utility>

using namespace std::chrono_literals;

namespace nav_system   // Namespace declaration
{
    
    // Constructor - initializes the action server
    NavActionServerComponent::NavActionServerComponent(const rclcpp::NodeOptions & options)
    : Node("nav_action_server_component", options)   // Because it takes NodeOptions, it can be loaded into a component container
    {
    action_server_ = rclcpp_action::create_server<NavigateToPose>(     // create server that handles goal requests, cancellations, and feedback 
        this,
        "navigate_to_pose",     // binding the action server to the "navigate_to_pose" custom action name
        std::bind(&NavActionServerComponent::handle_goal, this, std::placeholders::_1, std::placeholders::_2),       // binding the handle_goal callback method to the action server
        std::bind(&NavActionServerComponent::handle_cancel, this, std::placeholders::_1),  // binding the handle_cancel method to the action server
        std::bind(&NavActionServerComponent::handle_accepted, this, std::placeholders::_1)   // binding the handle_accepted method to the action server
    );

    RCLCPP_INFO(this->get_logger(), "NavActionServerComponent started.");
    }

    rclcpp_action::GoalResponse NavActionServerComponent::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal)
    {
        (void)uuid;

        RCLCPP_INFO(
            this->get_logger(),
            "Received goal request: x=%.3f, y=%.3f, theta=%.3f",
            goal->x, goal->y, goal->theta);

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse NavActionServerComponent::handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_WARN(this->get_logger(), "Received request to cancel goal.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void NavActionServerComponent::handle_accepted(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
    {
        // Create a new thread to execute the goal in the background 
                      // because execution is long-running, we would not want the acceptance callback itself to block.
        std::thread(
            std::bind(&NavActionServerComponent::execute, this, std::placeholders::_1),  // bind the execute method to the action server
            goal_handle
        ).detach();   // detach the thread to execute in the background
    }

    void NavActionServerComponent::execute(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
    {
        // Log the start of goal execution
        RCLCPP_INFO(this->get_logger(), "Executing goal...");

        auto feedback = std::make_shared<NavigateToPose::Feedback>();
        auto result = std::make_shared<NavigateToPose::Result>();

        // Dummy execution loop to test the action implementation
        for (int i = 0; i < 5; ++i) {

            // check if cancel request has arrived
            if (goal_handle->is_canceling()) {
            result->success = false;
            result->final_x = 0.0;
            result->final_y = 0.0;
            result->final_theta = 0.0;
            result->message = "Goal canceled during skeleton execution.";

            // if so, mark the goal as canceled and return
            goal_handle->canceled(result);
            RCLCPP_WARN(this->get_logger(), "Goal canceled.");   
            return;
            }

            // dummy feedback values to simulate progress toward the goal
            feedback->current_x = 0.1 * i;
            feedback->current_y = 0.0;
            feedback->current_theta = 0.0;
            feedback->distance_error = 1.0 - 0.2 * i;
            feedback->heading_error = 0.0;
            feedback->phase = "SKELETON_EXECUTION";

            // publish feedback to the client
            goal_handle->publish_feedback(feedback);

            // log the feedback
            RCLCPP_INFO(
            this->get_logger(),
            "Publishing feedback: current_x=%.2f, distance_error=%.2f, phase=%s",
            feedback->current_x,
            feedback->distance_error,
            feedback->phase.c_str());

            std::this_thread::sleep_for(1s);
        }

        result->success = true;
        result->final_x = goal_handle->get_goal()->x;
        result->final_y = goal_handle->get_goal()->y;
        result->final_theta = goal_handle->get_goal()->theta;
        result->message = "Skeleton execution completed successfully.";

        goal_handle->succeed(result);      // if no cancel request has arrived and goal is completed, the goal is marked as succeeded

        RCLCPP_INFO(this->get_logger(), "Goal succeeded.");
    }

}  // namespace nav_system

RCLCPP_COMPONENTS_REGISTER_NODE(nav_system::NavActionServerComponent)     // This line registers the class as a loadable ROS2 component