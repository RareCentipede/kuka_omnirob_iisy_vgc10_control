#include "koi_controller/koi_pick_place_controller.hpp"
#include <format>

using pick_response = mpnp_interfaces::srv::Pick::Response;
using place_response = mpnp_interfaces::srv::Place::Response;

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

  grasp_client_ = this->create_client<mpnp_interfaces::srv::Trigger>("/vacuum_tool/onrobot_vgc10/grasp");
  release_client_ = this->create_client<mpnp_interfaces::srv::Trigger>("/vacuum_tool/onrobot_vgc10/release");
  gz_node_.Subscribe(
    "/world/empty/dynamic_pose/info",
    &KOIPickPlaceController::dynamic_tf_callback,
    this
  );

  // Wait for grasp and release services to be available
  while (!grasp_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the grasp service. Exiting.");
      return;
    }
    RCLCPP_WARN(this->get_logger(), "Waiting for grasp service to be available...");
  }
  RCLCPP_INFO(this->get_logger(), "Grasp service is now available.");

  while (!release_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the release service. Exiting.");
      return;
    }
    RCLCPP_WARN(this->get_logger(), "Waiting for release service to be available...");
  }
  RCLCPP_INFO(this->get_logger(), "Release service is now available.");

  // TF2 setup
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

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

void KOIPickPlaceController::dynamic_tf_callback(const gz::msgs::Pose_V &posev_msg){
  for (const auto &pose_msg : posev_msg.pose()){
    if (std::find(object_names_.begin(), object_names_.end(), pose_msg.name()) != object_names_.end()){
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = this->now();
      tf_msg.header.frame_id = "world";
      tf_msg.child_frame_id = pose_msg.name() + "/base_link";

      tf_msg.transform.translation.x = pose_msg.position().x();
      tf_msg.transform.translation.y = pose_msg.position().y();
      tf_msg.transform.translation.z = pose_msg.position().z();
  
      tf_msg.transform.rotation.x = pose_msg.orientation().x();
      tf_msg.transform.rotation.y = pose_msg.orientation().y();
      tf_msg.transform.rotation.z = pose_msg.orientation().z();
      tf_msg.transform.rotation.w = pose_msg.orientation().w();
  
      tf_broadcaster_->sendTransform(tf_msg);

      current_obj_.pose.position.x = pose_msg.position().x();
      current_obj_.pose.position.y = pose_msg.position().y();
      current_obj_.pose.position.z = pose_msg.position().z();
      current_obj_.pose.orientation.x = pose_msg.orientation().x();
      current_obj_.pose.orientation.y = pose_msg.orientation().y();
      current_obj_.pose.orientation.z = pose_msg.orientation().z();
      current_obj_.pose.orientation.w = pose_msg.orientation().w();
    }
  }
}

void KOIPickPlaceController::setupPlanningScene(const std::string &object_name,
                                                const geometry_msgs::msg::Pose &pose,
                                                const char *frame_id){
  moveit_msgs::msg::CollisionObject collision_obj;
  collision_obj.id = object_name;
  collision_obj.header.frame_id = frame_id;
  collision_obj.primitives.resize(1);
  collision_obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  collision_obj.primitives[0].dimensions = {0.3, 0.3, 0.3}; // Example dimensions, adjust as needed
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
    p.z = box_tf.transform.translation.z;
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
  if (current_obj_.name != ""){
    response->success = false;
    response->result = pick_response::GRIPPER_OCCUPIED;
    response->message = std::format("Another object ({0}) is currently held. Place it before picking a new one.", 
                                    current_obj_.name);
    return;
  }
  const std::string box_link = request->object_name + "/base_link";

  std::optional<geometry_msgs::msg::Pose> target_pose = this->compute_target_pose(request->object_name, box_link);
  if (!target_pose.has_value()) {
    response->success = false;
    response->result = pick_response::TF_FAILED;
    response->message = std::format("Failed to get object pose for {0}", request->object_name);
    return;
  }

  current_obj_.name = request->object_name;
  current_obj_.pose = target_pose.value();
  this->setupPlanningScene(current_obj_.name, current_obj_.pose, request->frame_id.c_str());

  bool move_to_pick_success = this->doMoveToPickTask();

  if (not move_to_pick_success){
    response->success = false;
    response->result = pick_response::IK_FAILED;
    response->message = "Failed to compute path to pick pose";
  
    // Clear object from planning scene if pick task fails
    planning_scene_interface_.removeCollisionObjects({current_obj_.name});
    current_obj_ = Object();
    return;
  }
  rclcpp::sleep_for(std::chrono::seconds(2)); // Sleep to ensure the robot has reached the pick pose before sending grasp command
  mpnp_interfaces::srv::Trigger::Request::SharedPtr grasp_req;
  grasp_req = std::make_shared<mpnp_interfaces::srv::Trigger::Request>();
  grasp_req->target_obj = current_obj_.name;

  int max_attempts = 5;
  int attempt = 0;
  bool grasp_success = false;
  auto grasp_result = grasp_client_->async_send_request(grasp_req);

  while (attempt < max_attempts){
    if (grasp_result.wait_for(std::chrono::seconds(5)) == std::future_status::ready){
      if (grasp_result.get()->success){
        RCLCPP_INFO(this->get_logger(), "Grasp successful for object: %s", current_obj_.name.c_str());
        grasp_success = true;
        break;
      }
      else{
        RCLCPP_WARN(this->get_logger(), "Grasp attempt %d failed for object: %s. Retrying...", attempt+1, current_obj_.name.c_str());
        attempt++;
        grasp_result = grasp_client_->async_send_request(grasp_req);
      }
    }
  }

  if (!grasp_success){
    response->success = false;
    response->result = pick_response::GRASP_FAILED;
    response->message = grasp_result.get()->message;

    // Clear object from planning scene if pick task fails
    RCLCPP_ERROR(this->get_logger(), "Grasp failed for object: %s after %d attempts", current_obj_.name.c_str(), max_attempts);
    planning_scene_interface_.removeCollisionObjects({current_obj_.name});
    current_obj_ = Object();
    return;
  }

  bool retreat_success = this->doRetreatFromPickTask();
  if (not retreat_success){
    response->success = false;
    response->result = pick_response::GRASP_FAILED;
    response->message = "Failed to retreat after picking object";
  
    // Clear object from planning scene if pick task fails
    planning_scene_interface_.removeCollisionObjects({current_obj_.name});
    current_obj_ = Object();
    return;
  }
  bool success = move_to_pick_success && retreat_success;
  response->success = success;
  response->result = success ? pick_response::SUCCESS : pick_response::IK_FAILED;
  response->message = success ? "Pick task executed successfully" : "Failed to execute pick task";
}

void KOIPickPlaceController::place_service(const std::shared_ptr<mpnp_interfaces::srv::Place::Request> request,
                                           std::shared_ptr<mpnp_interfaces::srv::Place::Response> response){
  RCLCPP_INFO(this->get_logger(), "Received place request for object: %s to :%s",
              current_obj_.name.c_str(), request->target_name.c_str());
  if (current_obj_.name == ""){
    response->success = false;
    response->result = place_response::GRIPPER_EMPTY;
    response->message = "No object currently held. Pick an object before placing.";
    return;
  }
  const std::string target_link = request->target_name + "/base_link";

  std::optional<geometry_msgs::msg::Pose> target_pose = this->compute_target_pose(request->target_name, target_link);
  if (!target_pose.has_value()) {
    response->success = false;
    response->result = place_response::TF_FAILED;
    response->message = std::format("Failed to get pose for {0}", request->target_name);
    return;
  }

  geometry_msgs::msg::PoseStamped target_pose_stamped;
  target_pose_stamped.header.frame_id = request->frame_id;
  target_pose_stamped.header.stamp = this->now();
  target_pose_stamped.pose = target_pose.value();

  bool move_to_place_success = this->doMoveToPlaceTask(target_pose_stamped);
  if (!move_to_place_success){
    response->success = false;
    response->result = place_response::IK_FAILED;
    response->message = "Failed to compute path to place pose";
    return;
  }
  rclcpp::sleep_for(std::chrono::seconds(2)); // Sleep to ensure the robot has reached the place pose before sending release command
  mpnp_interfaces::srv::Trigger::Request::SharedPtr release_req;
  release_req = std::make_shared<mpnp_interfaces::srv::Trigger::Request>();
  release_req->target_obj = current_obj_.name;

  auto release_result = release_client_->async_send_request(release_req);
  if (release_result.wait_for(std::chrono::seconds(5)) == std::future_status::ready){
    if (release_result.get()->success){
      RCLCPP_INFO(this->get_logger(), "Release successful for object: %s", current_obj_.name.c_str());
    }
    else{
      RCLCPP_ERROR(this->get_logger(), "Release failed for object: %s. Message: %s", current_obj_.name.c_str(), release_result.get()->message.c_str());
      response->success = false;
      response->result = place_response::RELEASE_FAILED;
      response->message = release_result.get()->message;
      return;
    }
  }

  planning_scene_interface_.removeCollisionObjects({current_obj_.name});
  current_obj_ = Object();

  bool retreat_success = this->doReturnHomeTask();
  if (!retreat_success){
    response->success = false;
    response->result = place_response::RETURN_HOME_FAILED;
    response->message = "Failed to return home after placing object";
    return;
  }

  bool success = move_to_place_success && retreat_success;
  response->success = success;
  response->result = success ? place_response::SUCCESS : place_response::IK_FAILED;
  response->message = success ? "Place task executed successfully" : "Failed to execute place task";
}

KOIPickPlaceController::~KOIPickPlaceController(){
  RCLCPP_INFO(this->get_logger(), "Shutting down KOIPickPlaceController");

  pick_task_.clear();
  move_to_place_task.clear();

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