#include "nav_system/nav_action_client_component.hpp"

#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp_components/register_node_macro.hpp"

namespace nav_system
{

NavActionClientComponent::NavActionClientComponent(
  const rclcpp::NodeOptions & options)
: Node("nav_action_client_component", options)    // Constructor
{
  action_client_ = rclcpp_action::create_client<NavigateToPose>(
    this,
    "navigate_to_pose");     // Creates an action client for the action server named "navigate_to_pose"

  target_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
    "/ui/target_pose",
    10,
    std::bind(&NavActionClientComponent::target_pose_callback, this, std::placeholders::_1));

  cancel_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "/ui/cancel_goal",
    10,
    std::bind(&NavActionClientComponent::cancel_callback, this, std::placeholders::_1));

  status_pub_ = this->create_publisher<std_msgs::msg::String>("/ui/nav_status", 10);
  result_pub_ = this->create_publisher<std_msgs::msg::String>("/ui/nav_result", 10);

  RCLCPP_INFO(this->get_logger(), "NavActionClientComponent started.");
}

void NavActionClientComponent::target_pose_callback(
  const geometry_msgs::msg::Pose2D::SharedPtr msg)
{  // callback function that is called when a new goal is received from the UI 
  RCLCPP_INFO(
    this->get_logger(),
    "UI requested goal: x=%.3f, y=%.3f, theta=%.3f",   // Prints the received goal
    msg->x, msg->y, msg->theta);

  publish_status("Received UI goal request.");         
  send_goal(msg->x, msg->y, msg->theta);              // Sends the goal to the action server
}

void NavActionClientComponent::cancel_callback(
  const std_msgs::msg::Empty::SharedPtr msg)
{  // callback function that is called when a cancel request is received from the UI 
  (void)msg;   // Discards the message argument to prevent unused parameter warnings 

  if (!active_goal_handle_) {    // checks if there is an active goal 
    publish_status("No active goal to cancel.");
    RCLCPP_WARN(this->get_logger(), "Cancel requested, but no active goal exists.");
    return;
  }

  publish_status("Cancel requested by UI.");
  RCLCPP_WARN(this->get_logger(), "Sending cancel request...");

  auto cancel_future = action_client_->async_cancel_goal(active_goal_handle_);    // Sends the cancel request to the action server
  (void)cancel_future;     // Discards to prevent unused variable warnings
}

void NavActionClientComponent::send_goal(double x, double y, double theta)
{  //Sends the goal to the action server

   // Waits for the action server to be available 
  if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
    publish_status("Action server not available.");
    RCLCPP_ERROR(this->get_logger(), "Action server not available.");
    return;
  }

  // Build the goal message to be sent to the action server
  NavigateToPose::Goal goal_msg;
  goal_msg.x = x;
  goal_msg.y = y;
  goal_msg.theta = theta;

  publish_status("Sending goal to action server...");

  // Configure the options for sending the goal to the action server
  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.goal_response_callback =
    std::bind(&NavActionClientComponent::goal_response_callback, this, std::placeholders::_1);     // when the goal is accepted/rejected → goal_response_callback
  options.feedback_callback =
    std::bind(&NavActionClientComponent::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);     // when the action server sends feedback → feedback_callback
  options.result_callback =
    std::bind(&NavActionClientComponent::result_callback, this, std::placeholders::_1);     // when the action server finishes → result_callback

  action_client_->async_send_goal(goal_msg, options);     // Sends the goal to the action server
}

void NavActionClientComponent::goal_response_callback(
  GoalHandleNavigateToPose::SharedPtr goal_handle)
{  //Callback function that is called when the action server accepts or rejects the goal
  if (!goal_handle) {
    publish_status("Goal rejected by server.");
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server.");
    return;
  }

  // if accepted, save the goal handle and publish the status
  active_goal_handle_ = goal_handle;
  publish_status("Goal accepted by server.");
  RCLCPP_INFO(this->get_logger(), "Goal accepted by server.");
}

void NavActionClientComponent::feedback_callback(
  GoalHandleNavigateToPose::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{  // callback function that is called when the action server sends feedback 
  std::ostringstream oss;
  oss << "Feedback: phase=" << feedback->phase     // current phase of the navigation
      << ", current=(" << feedback->current_x      // current x-coordinate of the robot 
      << ", " << feedback->current_y             // current y-coordinate of the robot
      << ", " << feedback->current_theta           // current orientation of the robot
      << "), distance_error=" << feedback->distance_error    // distance error from the goal
      << ", heading_error=" << feedback->heading_error;    // heading error from the goal

  publish_status(oss.str());        // publish the feedback to the UI
  RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());     // print the feedback to the console
}

void NavActionClientComponent::result_callback(
  const GoalHandleNavigateToPose::WrappedResult & result)
{ // callback function that is called when the action server finishes 
  std::ostringstream oss;

  // inteprete the result code and append the result status to the oss string 
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      oss << "Result: SUCCEEDED";
      break;
    case rclcpp_action::ResultCode::ABORTED:
      oss << "Result: ABORTED";
      break;
    case rclcpp_action::ResultCode::CANCELED:
      oss << "Result: CANCELED";
      break;
    default:
      oss << "Result: UNKNOWN";
      break;
  }

  // Appends the final pose and message to the oss string
  oss << " | success=" << (result.result->success ? "true" : "false")
      << " | final_pose=("
      << result.result->final_x << ", "
      << result.result->final_y << ", "
      << result.result->final_theta << ")"
      << " | message=" << result.result->message;

  publish_result_text(oss.str());
  publish_status("Action finished.");
  RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

  active_goal_handle_.reset();
}

void NavActionClientComponent::publish_status(const std::string & text)
{ //Publishes the status of the action server to the UI 
  std_msgs::msg::String msg;
  msg.data = text;
  status_pub_->publish(msg);
}

void NavActionClientComponent::publish_result_text(const std::string & text)
{ //Publishes the result of the action server to the UI 
  std_msgs::msg::String msg;
  msg.data = text;
  result_pub_->publish(msg);
}

}  // namespace nav_system

RCLCPP_COMPONENTS_REGISTER_NODE(nav_system::NavActionClientComponent)