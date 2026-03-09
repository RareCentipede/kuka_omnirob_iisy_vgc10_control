#include <cstdio>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

class OmnirobController: public rclcpp::Node{
  public:
    OmnirobController(): Node("omnirob_controller"){
      RCLCPP_INFO(this->get_logger(), "hello world omnirob_controller package");
      auto subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "omnirob/pose", 10, std::bind(&OmnirobController::pose_callback, this, std::placeholders::_1)
      );
    }
  private:
    void pose_callback(const geometry_msgs::msg::Pose &pose_msg) const {
      RCLCPP_INFO(this->get_logger(), "Received pose: position(%f, %f, %f), orientation(%f, %f, %f, %f)",
      pose_msg.position.x, pose_msg.position.y, pose_msg.position.z,
      pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w);
    }
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OmnirobController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}