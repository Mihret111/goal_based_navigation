#ifndef nav_system__NAV_ACTION_SERVER_COMPONENT_HPP_
#define nav_system__NAV_ACTION_SERVER_COMPONENT_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "nav_interfaces/action/navigate_to_pose.hpp"

namespace nav_system
{
// A component: basically a node packaged as a loadable plugin
class NavActionServerComponent : public rclcpp::Node           
{
public:
  using NavigateToPose = nav_interfaces::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  explicit NavActionServerComponent(const rclcpp::NodeOptions & options);

private:
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;   // shared pointer to the action server 

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);

  void execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);
};

}  // namespace nav_system

#endif  // NAV_SYSTEM__NAV_ACTION_SERVER_COMPONENT_HPP_