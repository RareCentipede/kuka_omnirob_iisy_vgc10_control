#ifndef OMNIROB_CONTROLLER_HPP_
#define OMNIROB_CONTROLLER_HPP_

#include <cstdio>
#include <memory>
#include <cmath>
#include <array>
#include <eigen3/Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/model.pb.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>

#include "mpnp_interfaces/srv/move_base.hpp"

using Eigen::Vector3d;
using Eigen::Vector4d;

class OmnirobController: public rclcpp::Node{
  public:
    OmnirobController();
    geometry_msgs::msg::Pose pose_msg;
    Vector4d pose;        // [x, y, z, w]
    Vector4d target_pose; // [x, y, z, w]

  private:
    gz::transport::Node gz_node_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
    rclcpp::Service<mpnp_interfaces::srv::MoveBase>::SharedPtr move_base_service_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    void odom_calback(const nav_msgs::msg::Odometry &odom_msg);
    void move_base_service(const std::shared_ptr<mpnp_interfaces::srv::MoveBase::Request> request,
                           std::shared_ptr<mpnp_interfaces::srv::MoveBase::Response> response);
};

bool isClose(const Vector3d &home, const Vector3d &target, double tol);

#endif  // OMNIROB_CONTROLLER_HPP_