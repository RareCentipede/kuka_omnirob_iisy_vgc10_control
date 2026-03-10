#ifndef OMNIROB_CONTROLLER_HPP_
#define OMNIROB_CONTROLLER_HPP_

#include <cstdio>
#include <memory>
#include <cmath>
#include <array>
#include <eigen3/Eigen/Dense>
#include <mutex>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mpnp_interfaces/srv/move_base.hpp"
#include "rclcpp/rclcpp.hpp"

using Eigen::Vector4d;

class OmnirobController: public rclcpp::Node{
  public:
    OmnirobController();
    Vector4d position; // [x, y, z, w]
    Vector4d target_position; // [x, y, z, w]
    std::mutex position_mutex; // Mutex to protect access to position

  private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
    rclcpp::Service<mpnp_interfaces::srv::MoveBase>::SharedPtr move_base_service_;

    void odom_calback(const nav_msgs::msg::Odometry &odom_msg);
    void move_base_service(const std::shared_ptr<mpnp_interfaces::srv::MoveBase::Request> request,
                           std::shared_ptr<mpnp_interfaces::srv::MoveBase::Response> response);
};

bool isClose(const Vector4d &home, const Vector4d &target, double tol);

#endif  // OMNIROB_CONTROLLER_HPP_