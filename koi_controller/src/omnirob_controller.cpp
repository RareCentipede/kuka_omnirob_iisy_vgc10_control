#include "koi_controller/omnirob_controller.hpp"

OmnirobController::OmnirobController() : Node("omnirob_position_controller") {
    RCLCPP_INFO(this->get_logger(), "hello world omnirob_controller package");

    rclcpp::CallbackGroup::SharedPtr cb_group = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant
    );

    rclcpp::CallbackGroup::SharedPtr cb_group2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive
    );
    rclcpp::SubscriptionOptions subscription_options;
    rclcpp::SubscriptionOptions subscription_options2;
    subscription_options.callback_group = cb_group;
    subscription_options2.callback_group = cb_group2;

    robot_pos_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/omnirob_iisy_vgc10/pose", 10, std::bind(&OmnirobController::robot_pose_callback, this, std::placeholders::_1),
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
    // static_tf_broadcaster_->sendTransform(static_transform);

    pose = {0.0, 0.0, 0.0, 0.0}; // [x, y, z, w]
    target_pose = {0.0, 0.0, 0.0, 0.0}; // [x, y, z, w]
}

void OmnirobController::robot_pose_callback(const geometry_msgs::msg::Pose &pose_msg) {
  tf2::Quaternion q(
    pose_msg.orientation.x,
    pose_msg.orientation.y,
    pose_msg.orientation.z,
    pose_msg.orientation.w
  );
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  pose[0] = pose_msg.position.x;
  pose[1] = pose_msg.position.y;
  pose[2] = 0.0;
  pose[3] = yaw;
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
    target_pose_msg.header.stamp = this->now() - rclcpp::Duration(0, 500000000); // Subtract 100ms to ensure the transform is available
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
