#ifndef OMNIROB_CONTROLLER_HPP_
#define OMNIROB_CONTROLLER_HPP_

#include <cstdio>
#include <memory>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

class OmnirobController: public rclcpp::Node{
  public:
    OmnirobController();

  private:
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;

    void pose_callback(const geometry_msgs::msg::Pose &pose_msg) const;
};

#endif  // OMNIROB_CONTROLLER_HPP_