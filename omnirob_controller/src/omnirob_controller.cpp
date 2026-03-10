#include "omnirob_controller.hpp"

OmnirobController::OmnirobController() : Node("omnirob_controller") {
    RCLCPP_INFO(this->get_logger(), "hello world omnirob_controller package");
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/omnirob_controller/odometry", 10, std::bind(&OmnirobController::odom_calback, this, std::placeholders::_1)
    );

    twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/omnirob_controller/reference", 10);

    move_base_service_ = this->create_service<mpnp_interfaces::srv::MoveBase>(
      "/omnirob_controller/move_base", std::bind(&OmnirobController::move_base_service, this,
        std::placeholders::_1, std::placeholders::_2)
    );

    position = {0.0, 0.0, 0.0, 0.0}; // [x, y, z, w]
    target_position = {0.0, 0.0, 0.0, 0.0}; // [x, y, z, w]
}

void OmnirobController::odom_calback(const nav_msgs::msg::Odometry &odom_msg) {
      // Update the current position
      position[0] = odom_msg.pose.pose.position.x;
      position[1] = odom_msg.pose.pose.position.y;
      position[2] = odom_msg.pose.pose.position.z;
      position[3] = odom_msg.pose.pose.orientation.w; // Assuming we want to track the w component of the orientation as well
}

void OmnirobController::move_base_service(const std::shared_ptr<mpnp_interfaces::srv::MoveBase::Request> request,
                                         std::shared_ptr<mpnp_interfaces::srv::MoveBase::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Received move_base request: target_position(%f, %f, %f, %f)",
      request->target_position.x, request->target_position.y, request->target_position.z, request->target_position.w);

    geometry_msgs::msg::TwistStamped twist_msg;

    // Update the target position
    target_position[0] = request->target_position.x;
    target_position[1] = request->target_position.y;
    target_position[2] = request->target_position.z;
    target_position[3] = request->target_position.w;

    Vector4d vel = target_position - position;
    double dist = vel.norm();
    vel.normalize();
    vel *= request->speed; // Scale by the requested speed
    double est_time = dist / request->speed;

    twist_msg.twist.linear.x = vel[0];
    twist_msg.twist.linear.y = vel[1];
    twist_msg.twist.linear.z = vel[2];
    twist_msg.twist.angular.x = 0.0;
    twist_msg.twist.angular.y = 0.0;
    twist_msg.twist.angular.z = 0.0;

    double start_time = this->now().seconds();
    while ((this->now().seconds() - start_time) < est_time){
      twist_msg.header.stamp = this->now();
      twist_msg.header.frame_id = "omnirob_base_link"; // Assuming the frame of reference is base

      twist_publisher_->publish(twist_msg);
      RCLCPP_INFO(this->get_logger(), "Current position: (%f, %f, %f, %f), Target position: (%f, %f, %f, %f)",
        position[0], position[1], position[2], position[3],
        target_position[0], target_position[1], target_position[2], target_position[3]);
    }

    // Stop the robot
    twist_msg.twist.linear.x = 0.0;
    twist_msg.twist.linear.y = 0.0;
    twist_msg.twist.linear.z = 0.0;
    twist_publisher_->publish(twist_msg);

    // For now, just respond with success and a message
    response->success = true;
    response->message = "Target position reached!";
}

bool isClose(const Vector4d &home, const Vector4d &target, double tol){
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
