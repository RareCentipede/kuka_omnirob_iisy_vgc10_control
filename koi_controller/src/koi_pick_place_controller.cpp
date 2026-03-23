#include "koi_controller/koi_pick_place_controller.hpp"

KOIPickPlaceController::KOIPickPlaceController(const rclcpp::NodeOptions &options)
: Node("koi_pick_place_controller", options)
{
  RCLCPP_INFO(this->get_logger(), "Hello KOIPickPlaceController!");

  cb_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant
  );
  subscription_options_.callback_group = cb_group_;

  pick_service_ = this->create_service<mpnp_interfaces::srv::Pick>(
    "/koi_pick_place_controller/pick", std::bind(&KOIPickPlaceController::pick_service, this,
      std::placeholders::_1, std::placeholders::_2),
      rclcpp::QoS(10),
      cb_group_
  );

  place_service_ = this->create_service<mpnp_interfaces::srv::Place>(
    "/koi_pick_place_controller/place", std::bind(&KOIPickPlaceController::place_service, this,
      std::placeholders::_1, std::placeholders::_2),
      rclcpp::QoS(10),
      cb_group_
  );

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr KOIPickPlaceController::getNodeBaseInterface(){
  return this->get_node_base_interface();
}

void KOIPickPlaceController::pick_service(const std::shared_ptr<mpnp_interfaces::srv::Pick::Request> request,
                                    std::shared_ptr<mpnp_interfaces::srv::Pick::Response> response){
  RCLCPP_INFO(this->get_logger(), "Received pick request for object: %s", request->object.name.c_str());
  // Execute the planned pick task here (not implemented)
}

void KOIPickPlaceController::place_service(const std::shared_ptr<mpnp_interfaces::srv::Place::Request> request,
                                     std::shared_ptr<mpnp_interfaces::srv::Place::Response> response){
  RCLCPP_INFO(this->get_logger(), "Received place request for object: %s", request->object.name.c_str());
  // Execute the planned place task here (not implemented)
}

int main(int argc, char **argv){
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto controller_node = std::make_shared<KOIPickPlaceController>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &controller_node](){
    executor.add_node(controller_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(controller_node->getNodeBaseInterface());
  });

  spin_thread->join();
  rclcpp::shutdown();
}