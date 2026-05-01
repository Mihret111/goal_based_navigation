#ifndef nav_system__NAV_ACTION_SERVER_COMPONENT_HPP_   
#define nav_system__NAV_ACTION_SERVER_COMPONENT_HPP_

#include <memory>
#include <string>
#include <mutex>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nav_interfaces/action/navigate_to_pose.hpp"
#include "nav_interfaces/msg/nav_diagnostics.hpp"

namespace nav_system
{
// A component = basically a node packaged as a loadable plugin
class NavActionServerComponent : public rclcpp::Node           
{
public:
  using NavigateToPose = nav_interfaces::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;   // type for a specific accepted goal.

  explicit NavActionServerComponent(const rclcpp::NodeOptions & options);   // Constructor

private:                     // Private variables
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;  // shared pointer to the action server 
  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;   // Subscriber to the odometry topic 
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;   // Publisher to the velocity topic
  rclcpp::Publisher<nav_interfaces::msg::NavDiagnostics>::SharedPtr diagnostics_pub_;  // Publisher to the diagnostics topic

  std::string controller_mode_;  // mode variable to chooose from staged vs simultaneous movement 

  std::mutex pose_mutex_;                                       // protects shared pose variables  
  double current_x_, current_y_, current_theta_;                   // Current position and orientation of the robot
  bool odom_received_;                                             // flag to indicate if odometry has been received
  bool first_odom_logged_;                                         // flag to indicate if the first odometry has been logged

  // Called when a new goal request arrives
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal);
 
  // Called when user requests cancellation
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);

  // Called when goal is accepted
  void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);

  // Contains actual navigation logic
  void execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);

  // Callback functions
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void get_current_pose(double & x, double & y, double & theta);
  void publish_cmd_vel(double linear_x, double angular_z);    // publish velocity commands to the robot
  void publish_stop();  // publish stop command to the robot

  // publishes the navigation diagnostics to the diagnostics topic
  void publish_diagnostics(
  double current_x,
  double current_y,
  double current_theta,
  double target_x,
  double target_y,
  double target_theta,
  double distance_error,
  double heading_error,
  double linear_cmd,
  double angular_cmd,
  const std::string & phase,
  bool goal_active);
};



}  // namespace nav_system

#endif  // NAV_SYSTEM__NAV_ACTION_SERVER_COMPONENT_HPP_