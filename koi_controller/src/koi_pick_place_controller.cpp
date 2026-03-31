#include "koi_controller/koi_pick_place_controller.hpp"
#include <format>

KOIPickPlaceController::KOIPickPlaceController(const rclcpp::NodeOptions &options)
: Node("koi_pick_place_controller", options){
  RCLCPP_INFO(this->get_logger(), "Hello KOIPickPlaceController!");

  // Callback group
  cb_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant
  );
  subscription_options_.callback_group = cb_group_;

  // Services
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

  // TF2 setup
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

  // Planners
  interpolation_planner_ = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  cartesian_planner_ = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner_->setMaxVelocityScalingFactor(1.0);
  cartesian_planner_->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner_->setStepSize(.01);

  // Set up grasp pose orientation and frame
  grasp_pose_.header.frame_id = "world";
  grasp_pose_.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 1.0, 0, 0)); // Points the gripper downwards
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr KOIPickPlaceController::getNodeBaseInterface(){
  return this->get_node_base_interface();
}

void KOIPickPlaceController::setupPlanningScene(const std::string &object_name,
                                                const geometry_msgs::msg::Pose &pose,
                                                const char *frame_id){
  moveit_msgs::msg::CollisionObject collision_obj;
  collision_obj.id = object_name;
  collision_obj.header.frame_id = frame_id;
  collision_obj.primitives.resize(1);
  collision_obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  collision_obj.primitives[0].dimensions = {0.1, 0.1, 0.1}; // Example dimensions, adjust as needed
  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0] = pose;
  planning_scene_interface_.applyCollisionObject(collision_obj);
}

std::optional<geometry_msgs::msg::Pose> KOIPickPlaceController::compute_target_pose(const std::string &object_name,
                                                                                      const std::string &obj_frame_name){
  geometry_msgs::msg::TransformStamped box_tf;
  geometry_msgs::msg::Pose target_pose;

  try{
    // World to object/base_link transformation gives the object's pose in the world frame
    box_tf = tf_buffer_->lookupTransform(
      "world",
      obj_frame_name,
      this->now(),
      tf2::durationFromSec(1.0)
    );
  }
  catch (tf2::TransformException &ex){
    RCLCPP_ERROR(this->get_logger(), "Could not get transform for object %s: %s", object_name.c_str(), ex.what());
    return std::nullopt; // Return empty optional to indicate failure
  }

  target_pose.position = [&box_tf](){
    geometry_msgs::msg::Point p;
    p.x = box_tf.transform.translation.x;
    p.y = box_tf.transform.translation.y;
    p.z = box_tf.transform.translation.z+0.25;
    return p;
  }();
  target_pose.orientation = [&box_tf](){
    geometry_msgs::msg::Quaternion q;
    q.x = box_tf.transform.rotation.x;
    q.y = box_tf.transform.rotation.y;
    q.z = box_tf.transform.rotation.z;
    q.w = box_tf.transform.rotation.w;
    return q;
  }();

  return target_pose; // No exception, pose assigned successfully
}

void KOIPickPlaceController::pick_service(const std::shared_ptr<mpnp_interfaces::srv::Pick::Request> request,
                                          std::shared_ptr<mpnp_interfaces::srv::Pick::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received pick request for object: %s", request->object_name.c_str());
  const std::string box_link = request->object_name + "/base_link";

  std::optional<geometry_msgs::msg::Pose> target_pose = this->compute_target_pose(request->object_name, box_link);
  if (!target_pose.has_value()) {
    response->success = false;
    response->message = std::format("Failed to get object pose for {0}", request->object_name);
    return;
  }

  current_obj_.name = request->object_name;
  current_obj_.pose = target_pose.value();
  this->setupPlanningScene(current_obj_.name, current_obj_.pose, request->frame_id.c_str());

  bool success = this->doPickTask();
  response->success = success;
  response->message = success ? "Pick task executed successfully" : "Failed to execute pick task";
}

void KOIPickPlaceController::place_service(const std::shared_ptr<mpnp_interfaces::srv::Place::Request> request,
                                           std::shared_ptr<mpnp_interfaces::srv::Place::Response> response){
  RCLCPP_INFO(this->get_logger(), "Received place request for object: %s to :%s",
              current_obj_.name.c_str(), request->target_name.c_str());
  const std::string target_link = request->target_name + "/base_link";

  std::optional<geometry_msgs::msg::Pose> target_pose = this->compute_target_pose(request->target_name, target_link);
  if (!target_pose.has_value()) {
    response->success = false;
    response->message = std::format("Failed to get pose for {0}", request->target_name);
    return;
  }

  geometry_msgs::msg::PoseStamped target_pose_stamped;
  target_pose_stamped.header.frame_id = request->frame_id;
  target_pose_stamped.header.stamp = this->now();
  target_pose_stamped.pose = target_pose.value();
  target_pose_stamped.pose.position.z += 0.1; // Add offset to ensure the place pose is above the target

  bool success = this->doPlaceTask(target_pose_stamped);
  if (success){
    planning_scene_interface_.removeCollisionObjects({current_obj_.name}); // Remove object from planning scene after detaching
    current_obj_ = Object(); // Clear current object information
  }

  response->success = success;
  response->message = success ? "Place task executed successfully" : "Failed to execute place task";
}

KOIPickPlaceController::~KOIPickPlaceController(){
  RCLCPP_INFO(this->get_logger(), "Shutting down KOIPickPlaceController");

  pick_task_.clear();
  place_task_.clear();

  sampling_planner_.reset();
  interpolation_planner_.reset();
  cartesian_planner_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
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