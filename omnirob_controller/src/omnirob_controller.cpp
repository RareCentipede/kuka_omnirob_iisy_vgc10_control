#include "omnirob_controller.hpp"

OmnirobController::OmnirobController() : Node("omnirob_controller") {
  RCLCPP_INFO(this->get_logger(), "hello world omnirob_controller package");
  subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "/omnirob/pose", 10, std::bind(&OmnirobController::pose_callback, this, std::placeholders::_1)
  );
}

void OmnirobController::pose_callback(const geometry_msgs::msg::Pose &pose_msg) const {
      RCLCPP_INFO(this->get_logger(), "Received pose: position(%f, %f, %f), orientation(%f, %f, %f, %f)",
      pose_msg.position.x, pose_msg.position.y, pose_msg.position.z,
      pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w);
}

//TODO: Create service to move the robot to a desired pose, and create a client to call that service from a test node.
//TODO: Create an arm control interface, check to use service or action server, and create a client to call that
//TODO: service/action server from a test node.
//TODO: Create a scene to pick and place blocks.

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OmnirobController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
