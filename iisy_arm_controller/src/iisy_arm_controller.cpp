#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char* argv[]){
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "iisy_arm_controller",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("iisy_arm_controller");
  RCLCPP_INFO(logger, "Hello iisy_arm_controller!");

  // MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;;
  auto move_group_interface = MoveGroupInterface(node, "arm_controller");

  geometry_msgs::msg::Pose msg;
  auto const target_pose = [&msg]
  {
    msg.orientation.x = 0.964;
    msg.orientation.y = -0.201;
    msg.orientation.z = 0.075;
    msg.orientation.w = 0.159;
    msg.position.x = 0.133;
    msg.position.y = 0.05;
    msg.position.z = 1.216;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Plan to the target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan if successful
  if (success){
    RCLCPP_INFO(logger, "Plan found, executing...");
    move_group_interface.execute(plan);
    RCLCPP_INFO(logger, "Motion complete.");
  } else {
    RCLCPP_ERROR(logger, "Planning failed.");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}