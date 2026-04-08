#include "koi_controller/omnirob_controller.hpp"

OmnirobController::OmnirobController() : Node("omnirob_controller") {
    RCLCPP_INFO(this->get_logger(), "hello world omnirob_controller package");

    rclcpp::CallbackGroup::SharedPtr cb_group = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant
    );

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = cb_group;

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/omnirob_controller/odometry", 10, std::bind(&OmnirobController::odom_calback, this, std::placeholders::_1),
      subscription_options
    );

    twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/omnirob_controller/reference", 10);

    move_base_service_ = this->create_service<mpnp_interfaces::srv::MoveBase>(
      "/omnirob_controller/move_base", std::bind(&OmnirobController::move_base_service, this,
        std::placeholders::_1, std::placeholders::_2),
        rclcpp::QoS(10),
        cb_group
    );

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    auto static_transform = geometry_msgs::msg::TransformStamped();
    static_transform.header.stamp = this->now();
    static_transform.header.frame_id = "world";
    static_transform.child_frame_id = "odom";
    static_transform.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(static_transform);

    pose = {0.0, 0.0, 0.0, 0.0}; // [x, y, z, w]
    target_pose = {0.0, 0.0, 0.0, 0.0}; // [x, y, z, w]
}

void OmnirobController::odom_calback(const nav_msgs::msg::Odometry &odom_msg) {
  // Update the current position
  // Convert quaternion to yaw angle for simplicity, assuming planar movement
  tf2::Quaternion q(
    odom_msg.pose.pose.orientation.x,
    odom_msg.pose.pose.orientation.y,
    odom_msg.pose.pose.orientation.z,
    odom_msg.pose.pose.orientation.w
  );
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  pose[0] = odom_msg.pose.pose.position.x;
  pose[1] = odom_msg.pose.pose.position.y;
  pose[2] = odom_msg.pose.pose.position.z;
  pose[3] = yaw;

  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = this->now();
  transformStamped.header.frame_id = "odom";
  transformStamped.child_frame_id = "platform_base_link";
  transformStamped.transform.translation.x = odom_msg.pose.pose.position.x;
  transformStamped.transform.translation.y = odom_msg.pose.pose.position.y;
  transformStamped.transform.translation.z = odom_msg.pose.pose.position.z;

  transformStamped.transform.rotation.x = odom_msg.pose.pose.orientation.x;
  transformStamped.transform.rotation.y = odom_msg.pose.pose.orientation.y;
  transformStamped.transform.rotation.z = odom_msg.pose.pose.orientation.z;
  transformStamped.transform.rotation.w = odom_msg.pose.pose.orientation.w;

  tf_broadcaster_->sendTransform(transformStamped);
}

void OmnirobController::move_base_service(const std::shared_ptr<mpnp_interfaces::srv::MoveBase::Request> request,
                                         std::shared_ptr<mpnp_interfaces::srv::MoveBase::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received move_base request: target_position(%f, %f, %f)",
    request->target_position.x, request->target_position.y, request->target_position.z);

  geometry_msgs::msg::TwistStamped twist_msg;

  // Update the target position
  target_pose[0] = request->target_position.x;
  target_pose[1] = request->target_position.y;
  target_pose[2] = 0.0;

  double yaw_rate;
  double target_yaw;
  target_yaw = std::atan2(target_pose[1] - pose[1], target_pose[0] - pose[0]);

  while(abs((pose[3] - target_yaw)) > 1e-3){
    yaw_rate = target_yaw - pose[3];
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = "platform_base_link"; // Assuming the frame of reference is base
    twist_msg.twist.angular.z = yaw_rate;
    twist_publisher_->publish(twist_msg);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Current heading: (%f), Target heading: (%f), Error: %f",
                          pose[3], target_yaw, abs((pose[3] - target_yaw)));
  }

  geometry_msgs::msg::TwistStamped zero_twist_msg;
  while (!isClose(pose.head<3>(), target_pose.head<3>(), (1e-3)))
  {
    geometry_msgs::msg::PoseStamped target_pose_msg;
    target_pose_msg.header.stamp = this->now() - rclcpp::Duration(0, 100000000); // Subtract 100ms to ensure the transform is available
    target_pose_msg.header.frame_id = "world";
    target_pose_msg.pose.position.x = target_pose[0];
    target_pose_msg.pose.position.y = target_pose[1];
    target_pose_msg.pose.position.z = target_pose[2];

    target_pose_msg = tf_buffer_->transform(target_pose_msg, "platform_base_link");

    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = "world"; // Assuming the frame of reference is base
    twist_msg.twist.linear.x = target_pose_msg.pose.position.x;
    twist_msg.twist.linear.y = target_pose_msg.pose.position.y;
    twist_msg.twist.linear.z = target_pose_msg.pose.position.z;
    twist_publisher_->publish(twist_msg);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Current position: (%f, %f, %f), Target position: (%f, %f, %f), Error: %f",
                          pose[0], pose[1], pose[2], target_pose[0], target_pose[1], target_pose[2],
                          (pose.head<3>() - target_pose.head<3>()).norm());
  }

  // Stop the robot
  twist_publisher_->publish(zero_twist_msg);

  gz::msgs::Pose teleport_req;
  gz::msgs::Boolean teleport_response;
  bool result;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, target_yaw);
  teleport_req.set_name("omnirob_iisy_vgc10");
  teleport_req.mutable_position()->set_x(target_pose[0]);
  teleport_req.mutable_position()->set_y(target_pose[1]);
  teleport_req.mutable_position()->set_z(target_pose[2]);
  teleport_req.mutable_orientation()->set_x(q.x());
  teleport_req.mutable_orientation()->set_y(q.y());
  teleport_req.mutable_orientation()->set_z(q.z());
  teleport_req.mutable_orientation()->set_w(q.w());
  bool executed = gz_node_.Request(
      "/world/empty/set_pose",
      teleport_req,
      5000,
      teleport_response,
      result
  );

  if (!executed || !result || !teleport_response.data()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to teleport the robot to the target position!");
  } else {
    RCLCPP_INFO(this->get_logger(), "Successfully teleported the robot to the target position!");
  }

  // For now, just respond with success and a message
  response->success = true;
  response->message = "Target position reached! Final position: (" + std::to_string(pose[0]) + ", " + \
                       std::to_string(pose[1]) + ", " + std::to_string(pose[2]) + ", " + std::to_string(pose[3]) + ")"+ \
                       "Final error: " + std::to_string((pose.head<3>() - target_pose.head<3>()).norm());
}

bool isClose(const Vector3d &home, const Vector3d &target, double tol){
  double euclidean_dist = (home - target).norm();
  return euclidean_dist < tol;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OmnirobController>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
