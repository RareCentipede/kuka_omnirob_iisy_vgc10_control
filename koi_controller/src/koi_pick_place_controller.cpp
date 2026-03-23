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

  sampling_planner_ = std::make_shared<mtc::solvers::PipelinePlanner>(this->shared_from_this());
  interpolation_planner_ = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  cartesian_planner_ = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner_->setMaxVelocityScalingFactor(1.0);
  cartesian_planner_->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner_->setStepSize(.01);

  attach_object_stage_ = nullptr;
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr KOIPickPlaceController::getNodeBaseInterface(){
  return this->get_node_base_interface();
}

void KOIPickPlaceController::setupPlanningScene(const mpnp_interfaces::msg::Object &object,
                                                const geometry_msgs::msg::Pose &pose,
                                                const char *frame_id){
  moveit_msgs::msg::CollisionObject collision_obj;
  collision_obj.id = object.name;
  collision_obj.header.frame_id = frame_id;
  collision_obj.primitives.resize(1);
  collision_obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  collision_obj.primitives[0].dimensions = {0.1, 0.1, 0.1}; // Example dimensions, adjust as needed
  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0] = pose;
  planning_scene_interface_.applyCollisionObject(collision_obj);
}

void KOIPickPlaceController::pick_service(const std::shared_ptr<mpnp_interfaces::srv::Pick::Request> request,
                                    std::shared_ptr<mpnp_interfaces::srv::Pick::Response> response){
  RCLCPP_INFO(this->get_logger(), "Received pick request for object: %s", request->object.name.c_str());
  current_obj_ = request->object;
  this->setupPlanningScene(request->object, request->object.pose, request->frame_id.c_str());

  bool success = this->doPickTask();
  response->success = success;
  response->message = success ? "Pick task executed successfully" : "Failed to execute pick task";
}

void KOIPickPlaceController::place_service(const std::shared_ptr<mpnp_interfaces::srv::Place::Request> request,
                                     std::shared_ptr<mpnp_interfaces::srv::Place::Response> response){
  RCLCPP_INFO(this->get_logger(), "Received place request for object: %s", request->object.name.c_str());
  // Execute the planned place task here (not implemented)
}

bool KOIPickPlaceController::doPickTask(){
  pick_task_ = this->createPickTask();

  try{
    pick_task_.init();
  }
  catch (mtc::InitStageException &e){
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize pick task: %s", e.what());
    return false;
  }

  if (!pick_task_.plan(5)){
    RCLCPP_ERROR(this->get_logger(), "Failed to plan pick task");
    return false;
  }
  pick_task_.introspection().publishSolution(*pick_task_.solutions().front());

  auto result = pick_task_.execute(*pick_task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS){
    RCLCPP_ERROR(this->get_logger(), "Failed to execute pick task");
    return false;
  }

  return true;
}

mtc::Task KOIPickPlaceController::createPickTask(){
  mtc::Task pick_task;
  pick_task.stages()->setName("pick task");
  pick_task.loadRobotModel(this->shared_from_this());

  // Set task properties
  pick_task.setProperty("group", arm_group_name_);
  pick_task.setProperty("hand_group", hand_group_name_);
  pick_task.setProperty("hand_frame", hand_frame_);

  mtc::Stage *current_state_ptr = nullptr; // Forward current_state on to grasp pose generator
  auto stage_state_current = std::make_unique<stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  pick_task.add(std::move(stage_state_current));

  attach_object_stage_ = nullptr; // Will be set in addAttachObjectStage

  // Stage move to pick
  this->addMoveToPickStage(pick_task);
  this->addApproachObjectStage(pick_task);
  this->addSampleGraspStage(pick_task, current_state_ptr);
  this->addAttachObjectStage(pick_task);
  this->addLiftObjectStage(pick_task);

  return pick_task;
}

void KOIPickPlaceController::addMoveToPickStage(mtc::Task &pick_task){
  // Stage move to pick
  auto stage_move_to_pick = std::make_unique<stages::Connect>(
    "move to pick",
    stages::Connect::GroupPlannerVector{{
        arm_group_name_,
        sampling_planner_
    }}
  );

  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  pick_task.add(std::move(stage_move_to_pick));
}

void KOIPickPlaceController::addApproachObjectStage(mtc::Task &pick_task){
  // Approach object
  auto stage_approach = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner_);
  stage_approach->properties().set("marker_ns", "approach_object");
  stage_approach->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
  stage_approach->setMinMaxDistance(0.1, 0.3);

  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = hand_frame_;
  vec.vector.z = 1.0;
  stage_approach->setDirection(vec);
  pick_task.add(std::move(stage_approach));
}

void KOIPickPlaceController::addSampleGraspStage(mtc::Task &pick_task, mtc::Stage *current_state_ptr){
   // Sample grasp pose
  auto stage_sample_grasp = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
  stage_sample_grasp->properties().configureInitFrom(mtc::Stage::PARENT);
  stage_sample_grasp->properties().set("marker_ns", "grasp_pose");
  stage_sample_grasp->setObject(current_obj_.name);
  stage_sample_grasp->setAngleDelta(M_PI / 12);
  stage_sample_grasp->setMonitoredStage(current_state_ptr); // Hook into current state

  const std::string box_link = current_obj_.name + '/' + "base_link";
  geometry_msgs::msg::TransformStamped grasp_frame_tf = tf_buffer_->lookupTransform(
      hand_frame_,
      box_link,
      this->now(),
      tf2::durationFromSec(1.0));

  Eigen::Isometry3d grasp_frame_transform = convert_geometry_tf_to_eigen(grasp_frame_tf.transform);

  auto ik_wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage_sample_grasp));
  ik_wrapper->setMaxIKSolutions(8);
  ik_wrapper->setMinSolutionDistance(1.0);
  ik_wrapper->setIKFrame(grasp_frame_transform, hand_frame_);
  ik_wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
  ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
  pick_task.add(std::move(ik_wrapper));
}

void KOIPickPlaceController::addAttachObjectStage(mtc::Task &pick_task){
  auto stage_modify_scene = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand, obj)");
  stage_modify_scene->allowCollisions(
    current_obj_.name,
    pick_task.getRobotModel()
        ->getJointModelGroup(hand_group_name_)
        ->getLinkModelNamesWithCollisionGeometry(),
        true
  );
  pick_task.add(std::move(stage_modify_scene));

  auto stage_attach = std::make_unique<stages::ModifyPlanningScene>("attach obj");
  stage_attach->attachObject(current_obj_.name, hand_frame_);
  attach_object_stage_ = stage_attach.get();
  pick_task.add(std::move(stage_attach));
}

void KOIPickPlaceController::addLiftObjectStage(mtc::Task &pick_task){
  auto stage_lift = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner_);
  stage_lift->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_lift->setMinMaxDistance(0.1, 0.3);
  stage_lift->setIKFrame(hand_frame_);
  stage_lift->properties().set("marker_ns", "lift_object");

  // Set upward direction
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "world";
  vec.vector.z = 1.0;
  stage_lift->setDirection(vec);
  pick_task.add(std::move(stage_lift));
}

mtc::Task KOIPickPlaceController::createPlaceTask(){
  mtc::Task place_task;
  place_task.stages()->setName("place task");
  place_task.loadRobotModel(this->shared_from_this());
  return place_task;
}

Eigen::Isometry3d convert_geometry_tf_to_eigen(const geometry_msgs::msg::Transform &transform_msg){
  Eigen::Isometry3d grasp_frame_transform;
  Eigen::Quaterniond quat(
    transform_msg.rotation.x,
    transform_msg.rotation.y,
    transform_msg.rotation.z,
    transform_msg.rotation.w);

  grasp_frame_transform.translation().x() = transform_msg.translation.x;
  grasp_frame_transform.translation().y() = transform_msg.translation.y;
  grasp_frame_transform.translation().z() = transform_msg.translation.z;
  grasp_frame_transform.linear() = quat.toRotationMatrix();

  return grasp_frame_transform;
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