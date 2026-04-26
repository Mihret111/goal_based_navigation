#include "nav_system/nav_action_server_component.hpp"  // Including the header file 

#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <utility>

#include <cmath>
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/utils.h"    // for receiving yaw from quaternion of odom message
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace
{
    // wrap angle to meaningful range for interpretation (pi to -pi)
    double wrap_to_pi(double angle)
    {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
    }

    // clamp value to the [min, max] range
    double clamp_value(double value, double min_value, double max_value)
    {
    return std::max(min_value, std::min(value, max_value));
    }

}
namespace nav_system   // Namespace declaration
{
    
    // Constructor - initializes the action server
    NavActionServerComponent::NavActionServerComponent(const rclcpp::NodeOptions & options)
    : Node("nav_action_server_component", options),   // Because it takes NodeOptions, it can be loaded into a component container
    current_x_(0.0),
    current_y_(0.0),
    current_theta_(0.0),
    odom_received_(false),
    first_odom_logged_(false)
    {
        // publisher for velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10);
        
        // subscriber for odometry data
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        10,
        std::bind(&NavActionServerComponent::odom_callback, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<NavigateToPose>(     // create server that handles goal requests, cancellations, and feedback 
        this,
        "navigate_to_pose",     // binding the action server to the "navigate_to_pose" custom action name
        std::bind(&NavActionServerComponent::handle_goal, this, std::placeholders::_1, std::placeholders::_2),       // binding the handle_goal callback method to the action server
        std::bind(&NavActionServerComponent::handle_cancel, this, std::placeholders::_1),  // binding the handle_cancel method to the action server
        std::bind(&NavActionServerComponent::handle_accepted, this, std::placeholders::_1)   // binding the handle_accepted method to the action server
    );


    RCLCPP_INFO(this->get_logger(), "NavActionServerComponent started.");
    }
    // callback method for odometry data
    void NavActionServerComponent::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);

        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_theta_ = tf2::getYaw(msg->pose.pose.orientation);
        odom_received_ = true;

        if (!first_odom_logged_) {
            first_odom_logged_ = true;
            RCLCPP_INFO(
            this->get_logger(),
            "First odom received: x=%.3f, y=%.3f, yaw=%.3f",
            current_x_, current_y_, current_theta_);
        }
    }
    // get current pose of the robot
    void NavActionServerComponent::get_current_pose(
        double & x, double & y, double & yaw)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        x = current_x_;
        y = current_y_;
        yaw = current_theta_;
    }
    // publish stop command to the robot
    void NavActionServerComponent::publish_stop()
    {
        publish_cmd_vel(0.0, 0.0);
    }   

    // publish velocity commands to the robot
    void NavActionServerComponent::publish_cmd_vel(
        double linear_x,
        double angular_z)
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = linear_x;
        cmd.angular.z = angular_z;
        cmd_vel_pub_->publish(cmd);
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
        // check if odometry data is received
        if (!odom_received_) {
            result->success = false;
            result->final_x = 0.0;
            result->final_y = 0.0;
            result->final_theta = 0.0;
            result->message = "No odometry received yet. Cannot execute navigation.";
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Aborting goal: no odometry available.");
            return;
        }
        // Define controller gains and tolerances
        const double k_omega = 1.5;
        const double max_omega = 1.0;
        const double yaw_tolerance = 0.005;

        rclcpp::Rate loop_rate(10.0);    // create a 10 Hz control loop

        // Main control loop for rotation to goal heading
        while (rclcpp::ok()) {    // loop continues as long as ROS is running

            // Check if the goal was canceled
            if (goal_handle->is_canceling()) {
                double x, y, yaw;
                get_current_pose(x, y, yaw);

                publish_stop();  // Publish stop command

                // Set the result to canceled and publish the result
                result->success = false;
                result->final_x = x;
                result->final_y = y;
                result->final_theta = yaw;
                result->message = "Goal canceled during rotation phase.";

                goal_handle->canceled(result);
                RCLCPP_WARN(this->get_logger(), "Goal canceled.");
                return;
            }

            // If not canceled, get the current pose of the robot
            double x, y, yaw;
            get_current_pose(x, y, yaw);  // store current pose in x, y, yaw

            // Calculate the distance error and heading error
            const double dx = goal_handle->get_goal()->x - x;
            const double dy = goal_handle->get_goal()->y - y;
            const double distance_error = std::hypot(dx, dy);

            const double heading_to_goal = std::atan2(dy, dx);
            const double heading_error = wrap_to_pi(heading_to_goal - yaw);

            // Prepare the feedback with the current pose and errors
            feedback->current_x = x;
            feedback->current_y = y;
            feedback->current_theta = yaw;
            feedback->distance_error = distance_error;
            feedback->heading_error = heading_error;
            feedback->phase = "ROTATING_TO_GOAL";

            // Publish the feedback
            goal_handle->publish_feedback(feedback);

            // If the heading error is within the tolerance, stop the robot: Goal Reached
            if (std::abs(heading_error) < yaw_tolerance) {
                publish_stop();

                // Set the result to succeeded and publish the result
                result->success = false;
                result->final_x = x;
                result->final_y = y;
                result->final_theta = yaw;
                result->message =
                    "Rotation phase completed successfully.";

                goal_handle->abort(result);
                RCLCPP_WARN(
                    this->get_logger(),
                    "Rotation phase complete. (partially done)");
                return;
            }

            // If not yet at the goal heading (heading error > tolerance), rotate the robot to the goal heading
            const double omega_cmd =
                clamp_value(k_omega * heading_error, -max_omega, max_omega); // proportional controller with saturation (clamping control effort)

            publish_cmd_vel(0.0, omega_cmd);  // Publish angular velocity command

            RCLCPP_INFO(
                this->get_logger(),
                "Rotating: yaw=%.3f, heading_to_goal=%.3f, heading_error=%.3f, omega_cmd=%.3f",
                yaw, heading_to_goal, heading_error, omega_cmd);

            loop_rate.sleep();  // wait for the next control cycle
        }

        publish_stop();   // unexpected termination

        result->success = false;
        result->final_x = 0.0;
        result->final_y = 0.0;
        result->final_theta = 0.0;
        result->message = "Execution loop ended unexpectedly.";
        goal_handle->abort(result);
    }

}  // namespace nav_system

RCLCPP_COMPONENTS_REGISTER_NODE(nav_system::NavActionServerComponent)     // This line registers the class as a loadable ROS2 component