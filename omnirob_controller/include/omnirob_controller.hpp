#ifndef OMNIROB_CONTROLLER_HPP_
#define OMNIROB_CONTROLLER_HPP_

#include <cstdio>
#include <memory>
#include <cmath>
#include <array>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mpnp_interfaces/srv/move_base.hpp"
#include "rclcpp/rclcpp.hpp"

class OmnirobController: public rclcpp::Node{
  public:
    OmnirobController();
    std::array<double, 4> position = {0.0, 0.0, 0.0, 0.0}; // [x, y, z, w]
    std::array<double, 4> target_position = {0.0, 0.0, 0.0, 0.0}; // [x, y, z, w]

  private:
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
    rclcpp::Service<mpnp_interfaces::srv::MoveBase>::SharedPtr move_base_service_;

    void pose_callback(const geometry_msgs::msg::Pose &pose_msg);
    void move_base_service(const std::shared_ptr<mpnp_interfaces::srv::MoveBase::Request> request,
                           std::shared_ptr<mpnp_interfaces::srv::MoveBase::Response> response);
};

bool isClose(const std::array<double, 4> &home, const std::array<double, 4> &target, double tol);

#endif  // OMNIROB_CONTROLLER_HPP_