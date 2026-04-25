#ifndef NAV_SYSTEM__NAV_ACTION_CLIENT_COMPONENT_HPP_
#define NAV_SYSTEM__NAV_ACTION_CLIENT_COMPONENT_HPP_

#include <memory>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav_interfaces/action/navigate_to_pose.hpp"

namespace nav_system
{

class NavActionClientComponent : public rclcpp::Node
{
public:
  using NavigateToPose = nav_interfaces::action::NavigateToPose;
  using GoalHandleNavigateToPose =
    rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit NavActionClientComponent(const rclcpp::NodeOptions & options);    // constructor that takes NodeOptions, so it can be loaded into a component container  

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;   // action client that handles goal requests, cancellations, and feedback 

  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr target_pose_sub_;   // listens to the UI goal topic (/ui/target_pose)
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;   // listens to the UI cancel topic (/ui/cancel_goal)

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;   // publishes the status of the navigation to the UI topic (/ui/nav_status)
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;   // publishes the result of the navigation to the UI topic (/ui/nav_result)

  GoalHandleNavigateToPose::SharedPtr active_goal_handle_;  // pointer to the currently active goal handle

  void target_pose_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg);  
  void cancel_callback(const std_msgs::msg::Empty::SharedPtr msg);

  void send_goal(double x, double y, double theta);

  void goal_response_callback(GoalHandleNavigateToPose::SharedPtr goal_handle);
  void feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void result_callback(const GoalHandleNavigateToPose::WrappedResult & result);

  void publish_status(const std::string & text);
  void publish_result_text(const std::string & text);
};

}  // namespace nav_system

#endif  // NAV_SYSTEM__NAV_ACTION_CLIENT_COMPONENT_HPP_