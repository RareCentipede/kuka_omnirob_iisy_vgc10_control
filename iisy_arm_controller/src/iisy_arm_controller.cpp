#include <memory>
#include <cstdio>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include "mpnp_interfaces/srv/move_arm.hpp"
#include "mpnp_interfaces/msg/vector4.hpp"

using moveit::planning_interface::MoveGroupInterface;
using mpnp_interfaces::msg::Vector4;
using mpnp_interfaces::srv::MoveArm;

class ArmController: public rclcpp::Node{
  public:
    ArmController(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> &move_group_interface): Node("iisy_arm_controller") {
      RCLCPP_INFO(logger, "Hello iisy_arm_controller!");
      move_arm_service_ = this->create_service<mpnp_interfaces::srv::MoveArm>("iisy/move_arm",
                        std::bind(&ArmController::move_arm_service, this, std::placeholders::_1, std::placeholders::_2));
      move_group_interface_ = move_group_interface;
    }

    private:
    bool success = false;
    rclcpp::Logger logger = this->get_logger();
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    rclcpp::Service<MoveArm>::SharedPtr move_arm_service_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;

    void move_arm_service(const std::shared_ptr<mpnp_interfaces::srv::MoveArm::Request> request,
                          std::shared_ptr<mpnp_interfaces::srv::MoveArm::Response> response){
      geometry_msgs::msg::Pose target_pose;
      Vector4 target_position = request->target_position;

      target_pose = [&target_position, &target_pose]{
        target_pose.position.x = target_position.x;
        target_pose.position.y = target_position.y;
        target_pose.position.z = target_position.z;
        target_pose.orientation.w = target_position.w;
        return target_pose;
      }();
      move_group_interface_->setPoseTarget(target_pose);

      success = static_cast<bool>(move_group_interface_->plan(plan));

      // Execute the plan if successful
      if (success){
        RCLCPP_INFO(logger, "Plan found, executing...");
        move_group_interface_->execute(plan);
        RCLCPP_INFO(logger, "Motion complete.");
        response->message = "Motion complete.";
      } else {
        RCLCPP_ERROR(logger, "Planning failed.");
        response->message = "Planning failed.";
      }

      response->success = success;
    }
};

int main(int argc, char* argv[]){
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    rclcpp::Node::make_shared("temp_node"), "arm_controller");
  auto const node = std::make_shared<ArmController>(move_group);
  rclcpp::spin(node);

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}