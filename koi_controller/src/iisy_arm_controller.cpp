#include <memory>
#include <cstdio>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

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

      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
    }

  private:
    // TF variables
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool success = false;
    rclcpp::Logger logger = this->get_logger();
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    rclcpp::Service<MoveArm>::SharedPtr move_arm_service_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;

    void move_arm_service(const std::shared_ptr<mpnp_interfaces::srv::MoveArm::Request> request,
                          std::shared_ptr<mpnp_interfaces::srv::MoveArm::Response> response){
      geometry_msgs::msg::Pose target_pose;
      geometry_msgs::msg::PoseStamped target_pose_stamped;
      Vector4 target_position = request->target_position;
      geometry_msgs::msg::PoseStamped target_pose_stamped_transformed;

      geometry_msgs::msg::TransformStamped transform_base_link_to_gripper = tf_buffer_->lookupTransform(
        "arm_flange",
        "box/base_link",
        this->now(),
        tf2::durationFromSec(1.0));

      target_pose_stamped.header.frame_id = "box/base_link";
      target_pose_stamped.pose.position.x = target_position.x;
      target_pose_stamped.pose.position.y = target_position.y;
      target_pose_stamped.pose.position.z = target_position.z;
      target_pose_stamped.pose.orientation.w = target_position.w;

      target_pose_stamped.header.stamp = transform_base_link_to_gripper.header.stamp;

      tf2::doTransform(target_pose_stamped, target_pose_stamped_transformed, transform_base_link_to_gripper);

      RCLCPP_INFO(logger, "Transformed target pose: [%.2f, %.2f, %.2f, %.2f]",
                  target_pose_stamped_transformed.pose.position.x,
                  target_pose_stamped_transformed.pose.position.y,
                  target_pose_stamped_transformed.pose.position.z,
                  target_pose_stamped_transformed.pose.orientation.w);

      target_pose = [&target_pose_stamped_transformed, &target_pose]{
        target_pose.position.x = target_pose_stamped_transformed.pose.position.x;
        target_pose.position.y = target_pose_stamped_transformed.pose.position.y;
        target_pose.position.z = target_pose_stamped_transformed.pose.position.z;
        target_pose.orientation.w = target_pose_stamped_transformed.pose.orientation.w;
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