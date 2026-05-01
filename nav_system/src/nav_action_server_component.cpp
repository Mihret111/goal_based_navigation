#include "nav_system/nav_action_server_component.hpp"  // Including the header file 

#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <utility>
#include <string>

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
    while (angle > M_PI) {   // M_PI is defined in <cmath> header file == 3.1415...
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

        // create a diagnostics publisher
        diagnostics_pub_ = this->create_publisher<nav_interfaces::msg::NavDiagnostics>(
        "/nav/diagnostics",
        10);             // Will be subscribed to by the ui to display diagnostics information in the visual window

        // defining a configuration parameter that allows to switch between staged and simultaneous implementations
        this->declare_parameter<std::string>("controller_mode", "staged");
        controller_mode_ = this->get_parameter("controller_mode").as_string();

        RCLCPP_INFO(
        this->get_logger(),
        "Controller mode: %s",
        controller_mode_.c_str());


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

    void NavActionServerComponent::publish_diagnostics(
    double current_x, double current_y, double current_theta,
    double target_x, double target_y, double target_theta,
    double distance_error, double heading_error,
    double linear_cmd, double angular_cmd,
    const std::string & phase, bool goal_active)
    {
    nav_interfaces::msg::NavDiagnostics msg;
    msg.current_x = current_x;
    msg.current_y = current_y;
    msg.current_theta = current_theta;

    msg.target_x = target_x;
    msg.target_y = target_y;
    msg.target_theta = target_theta;

    msg.distance_error = distance_error;
    msg.heading_error = heading_error;

    msg.linear_cmd = linear_cmd;
    msg.angular_cmd = angular_cmd;

    msg.phase = phase;
    msg.controller_mode = controller_mode_;
    msg.goal_active = goal_active;

    diagnostics_pub_->publish(msg);
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
            //if not received, abort the goal: cannot be executed
            result->success = false;
            result->final_x = 0.0;
            result->final_y = 0.0;
            result->final_theta = 0.0;
            result->message = "No odometry received yet. Cannot execute navigation.";
            
            
            publish_diagnostics( 0.0, 0.0, 0.0, goal_handle->get_goal()->x, goal_handle->get_goal()->y,
                                 goal_handle->get_goal()->theta, 0.0, 0.0, 
                                 0.0, 0.0, "NO_ODOMETRY", false);

            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Aborting goal: no odometry available.");
            return;
        }
        // Define controller gains and tolerances
        const double k_omega = 1.5;
        const double max_omega = 1.0;
        const double yaw_tolerance = 0.05;   // tolerance for yaw angle error in radians

        const double k_v = 0.5;
        const double max_v = 0.25;                 // maximum allowable linear velocity in meters/second  (physically relatable to motor speed limits)
        const double position_tolerance = 0.05;    // defines how close enough is enough in meters

        const double final_yaw_tolerance = 0.05;   // tolerance for final orientation alignment
        const double k_omega_final = 1.5;          // gain for final orientation alignment

        // saftey: (check result though....if heading error gets too large while moving, stop translating and rotate again.
        const double heading_realign_threshold = 0.30;

        // MODE-Simultaneous: parameters(gains) for the simultaneous implementation of rotation and translation
        const double k_rho = 0.6;
        const double k_alpha = 1.8;
        const double k_beta = -0.5;
        // const double max_v = 0.25;
        // const double max_omega = 1.0;
        // const double position_tolerance = 0.05;
        // const double final_yaw_tolerance = 0.05;

        // create a 10 Hz control loop
        rclcpp::Rate loop_rate(10.0);   

        enum class Phase
        {  // phased implemetation : subject to change for simultaneous rotation and translation
        ROTATING_TO_GOAL,
        MOVING_TO_POSITION,
        ALIGNING_FINAL_THETA
        };

        Phase phase = Phase::ROTATING_TO_GOAL;   // Initializing phase to ROTATING_TO_GOAL

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


                publish_diagnostics(x, y, yaw, goal_handle->get_goal()->x,goal_handle->get_goal()->y,
                                    goal_handle->get_goal()->theta, 0.0, 0.0, 
                                    0.0, 0.0, "GOAL_CANCELED", false);


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
            const double final_theta_error = wrap_to_pi(goal_handle->get_goal()->theta - yaw);
            
            // MODE-Simultaneous: parameters for simultaneous rotation and translation
            const double rho = std::hypot(dx, dy);   // error in distance
            const double alpha = wrap_to_pi(std::atan2(dy, dx) - yaw);   // error in heading to goal
            const double beta = wrap_to_pi(goal_handle->get_goal()->theta - yaw - alpha);   // error in final orientation
            // const double final_theta_error = wrap_to_pi(goal_handle->get_goal()->theta - yaw);


        // MODE-Simultaneous
        if (controller_mode_ == "simultaneous") {
            feedback->current_x = x;
            feedback->current_y = y;
            feedback->current_theta = yaw;
            feedback->distance_error = rho;
            feedback->heading_error = alpha;
            feedback->phase = "SIMULTANEOUS_CONTROL";

            goal_handle->publish_feedback(feedback);

            // Success condition: If the distance error and heading error are within the tolerance, stop the robot: Goal Reached
            if (rho < position_tolerance &&
                std::abs(final_theta_error) < final_yaw_tolerance)
            {
                publish_stop();

                result->success = true;
                result->final_x = x;
                result->final_y = y;
                result->final_theta = yaw;
                result->message = "Navigation completed successfully with simultaneous controller.";

                publish_diagnostics( x, y, yaw, goal_handle->get_goal()->x,goal_handle->get_goal()->y,
                                    goal_handle->get_goal()->theta, distance_error, final_theta_error, 
                                    0.0, 0.0, "GOAL_FINISHED", false);

                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Navigation goal succeeded (simultaneous mode).");
                return;
            }
            // If not, implement the movement and rotation towards goal
            const double v_cmd =
                clamp_value(k_rho * rho * std::max(0.0, std::cos(alpha)), 0.0, max_v);
                //If the robot has bad heading error, cos(alpha) becomes small or negative, 
                //so v_cmd becomes small or zero, preventing unwanted arc movement.
                //If low heading error, large forward speed; if high heading error, low forward speed
            
            const double omega_cmd =
                clamp_value(k_alpha * alpha + k_beta * beta, -max_omega, max_omega);
                //If beta is large and negative (wrong side), omega will be large positive (turn left).
                //If beta is large and positive (other wrong side), omega will be large negative (turn right).
                //If beta is near zero, omega will be small, letting the robot drive straight.

            publish_cmd_vel(v_cmd, omega_cmd);

            RCLCPP_INFO(
                this->get_logger(),
                "SIMULTANEOUS | x=%.3f, y=%.3f, yaw=%.3f, rho=%.3f, alpha=%.3f, beta=%.3f, v=%.3f, omega=%.3f",
                x, y, yaw, rho, alpha, beta, v_cmd, omega_cmd);

            loop_rate.sleep();  // Wait 0.1 seconds before next control loop iteration
            publish_diagnostics( x, y,  yaw, goal_handle->get_goal()->x, goal_handle->get_goal()->y,
                                    goal_handle->get_goal()->theta, distance_error,feedback->heading_error,
                                    v_cmd,omega_cmd,feedback->phase,true);
            continue;
        }

        //MODE-Staged
        else if (controller_mode_ == "staged") {
            // Prepare the feedback with the current pose and errors
            feedback->current_x = x;
            feedback->current_y = y;
            feedback->current_theta = yaw;
            feedback->distance_error = distance_error;
            // set heading error based on phase
            if (phase == Phase::ALIGNING_FINAL_THETA) {    
                feedback->heading_error = final_theta_error;
            } else {
                feedback->heading_error = heading_error;
            }

            // Publish the feedback
            goal_handle->publish_feedback(feedback);

            // If the heading error is within the tolerance, stop the robot: Goal Reached
            if (phase == Phase::ROTATING_TO_GOAL &&
                std::abs(heading_error) < yaw_tolerance)
            {
                publish_stop();
                feedback->phase = "ROTATING_TO_GOAL";
                phase = Phase::MOVING_TO_POSITION;
                RCLCPP_INFO(this->get_logger(), "Rotation phase complete. Switching to move phase.");
                loop_rate.sleep();
                continue;
            }

            // if position tolerance is reached, abort the goal
            if (phase == Phase::MOVING_TO_POSITION &&
                distance_error < position_tolerance)
            {
                publish_stop();
                phase = Phase::ALIGNING_FINAL_THETA;
                RCLCPP_INFO(
                    this->get_logger(),
                    "Position phase complete. Switching to final-theta alignment phase.");
                loop_rate.sleep();
                continue;
            }

            if (phase == Phase::ALIGNING_FINAL_THETA &&
            std::abs(final_theta_error) < final_yaw_tolerance)
            {
                publish_stop();

                result->success = true;
                result->final_x = x;
                result->final_y = y;
                result->final_theta = yaw;
                result->message = "Navigation completed successfully.";

                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Navigation goal succeeded.");
                return;
            }
        
            // Implement the movement and rotation towards goal
            double v_cmd = 0.0;
            double omega_cmd = 0.0;

            if (phase == Phase::ROTATING_TO_GOAL) {
            v_cmd = 0.0;
            omega_cmd = clamp_value(k_omega * heading_error, -max_omega, max_omega);
            feedback->phase = "ROTATING_TO_GOAL";
            }
            else if (phase == Phase::MOVING_TO_POSITION) {
            // If the heading error becomes too large while moving, stop forward
            // motion and go back to rotation phase for stability.
            if (std::abs(heading_error) > heading_realign_threshold) {
                v_cmd = 0.0;
                omega_cmd = clamp_value(k_omega * heading_error, -max_omega, max_omega);
                feedback->phase = "REALIGNING_DURING_MOVE";
            } else {
                v_cmd = clamp_value(k_v * distance_error, 0.0, max_v);
                omega_cmd = clamp_value(k_omega * heading_error, -max_omega, max_omega);
                feedback->phase = "MOVING_TO_POSITION";
            }
            }
            else if (phase == Phase::ALIGNING_FINAL_THETA) {
                v_cmd = 0.0;
                omega_cmd = clamp_value(k_omega_final * final_theta_error, -max_omega, max_omega);
                feedback->phase = "ALIGNING_FINAL_THETA";
            }

            publish_cmd_vel(v_cmd, omega_cmd);

            publish_diagnostics( x, y, yaw, goal_handle->get_goal()->x,goal_handle->get_goal()->y,
                                 goal_handle->get_goal()->theta, distance_error, heading_error, 
                                 v_cmd, omega_cmd, feedback->phase, true);

            RCLCPP_INFO(
            this->get_logger(),
            "Phase=%s | x=%.3f, y=%.3f, yaw=%.3f, dist=%.3f, heading_err=%.3f, v_cmd=%.3f, omega_cmd=%.3f",
            feedback->phase.c_str(),
            x, y, yaw, distance_error, heading_error, v_cmd, omega_cmd);
            
            loop_rate.sleep();  // wait for the next control cycle
        }
    }
    
        publish_stop();   // unexpected termination

        result->success = false;
        double final_x, final_y, final_yaw;
        get_current_pose(final_x, final_y, final_yaw);
        result->final_x = final_x;
        result->final_y = final_y;
        result->final_theta = final_yaw;
        result->message = "Execution loop ended unexpectedly.";
        publish_diagnostics( final_x, final_y, final_yaw, goal_handle->get_goal()->x, goal_handle->get_goal()->y,
                                 goal_handle->get_goal()->theta, 0.0, 0.0, 
                                 0.0, 0.0, "GOAL_ABORTED", false);
        goal_handle->abort(result);
    }

}  // namespace nav_system

RCLCPP_COMPONENTS_REGISTER_NODE(nav_system::NavActionServerComponent)     // This line registers the class as a loadable ROS2 component