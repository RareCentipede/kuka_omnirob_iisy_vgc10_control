#include "omnirob_controller.hpp"

OmnirobController::OmnirobController() : Node("omnirob_controller") {
    RCLCPP_INFO(this->get_logger(), "hello world omnirob_controller package");
    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/omnirob/pose", 10, std::bind(&OmnirobController::pose_callback, this, std::placeholders::_1)
    );

    twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/omnirob_controller/reference", 10);

    move_base_service_ = this->create_service<mpnp_interfaces::srv::MoveBase>(
      "/omnirob_controller/move_base", std::bind(&OmnirobController::move_base_service, this,
        std::placeholders::_1, std::placeholders::_2)
    );
}

void OmnirobController::pose_callback(const geometry_msgs::msg::Pose &pose_msg) {
      RCLCPP_INFO(this->get_logger(), "Received pose: position(%f, %f, %f), orientation(%f, %f, %f, %f)",
      pose_msg.position.x, pose_msg.position.y, pose_msg.position.z,
      pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w);

      // Update the current position
      position[0] = pose_msg.position.x;
      position[1] = pose_msg.position.y;
      position[2] = pose_msg.position.z;
      position[3] = pose_msg.orientation.w; // Assuming we want to track the w component of the orientation as well
}

void OmnirobController::move_base_service(const std::shared_ptr<mpnp_interfaces::srv::MoveBase::Request> request,
                                         std::shared_ptr<mpnp_interfaces::srv::MoveBase::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Received move_base request: target_position(%f, %f, %f, %f)",
      request->target_position.x, request->target_position.y, request->target_position.z, request->target_position.w);

    // Update the target position
    target_position[0] = request->target_position.x;
    target_position[1] = request->target_position.y;
    target_position[2] = request->target_position.z;
    target_position[3] = request->target_position.w;

    while (!isClose(position, target_position, 0.5)){
      ;
    }

    // For now, just respond with success and a message
    response->success = true;
    response->message = "Target position received successfully";
}

bool isClose(const std::array<double, 4> &home, const std::array<double, 4> &target, double tol){
  double euclidean_dist = 0.0;
  for (int i = 0; i < 4; i++){
    euclidean_dist += std::pow(home[i] - target[i], 2);
  }

  euclidean_dist = std::sqrt(euclidean_dist);
  return euclidean_dist < tol;
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
