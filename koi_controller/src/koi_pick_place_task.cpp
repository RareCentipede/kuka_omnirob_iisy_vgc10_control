#include "koi_controller/koi_pick_place_controller.hpp"

bool KOIPickPlaceController::doMoveToPickTask(){
  auto move_to_pick_task = this->createMoveToPickTask();

  try{
    move_to_pick_task.init();
  }
  catch (mtc::InitStageException &e){
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to initialize move to pick task: " << e.what());
    RCLCPP_ERROR_STREAM(this->get_logger(), e);
    return false;
  }

  if (!move_to_pick_task.plan(5)){
    RCLCPP_ERROR(this->get_logger(), "Failed to plan move to pick task");
    return false;
  }
  move_to_pick_task.introspection().publishSolution(*move_to_pick_task.solutions().front());

  auto result = move_to_pick_task.execute(*move_to_pick_task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS){
    RCLCPP_ERROR(this->get_logger(), "Failed to execute move to pick task");
    return false;
  }

  return true;
}

bool KOIPickPlaceController::doRetreatFromPickTask(){
  auto retreat_from_pick_task = this->createRetreatFromPickTask();

  try{
    retreat_from_pick_task.init();
  }
  catch (mtc::InitStageException &e){
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to initialize retreat from pick task: " << e.what());
    RCLCPP_ERROR_STREAM(this->get_logger(), e);
    return false;
  }

  if (!retreat_from_pick_task.plan(5)){
    RCLCPP_ERROR(this->get_logger(), "Failed to plan retreat from pick task");
    return false;
  }
  retreat_from_pick_task.introspection().publishSolution(*retreat_from_pick_task.solutions().front());

  auto result = retreat_from_pick_task.execute(*retreat_from_pick_task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS){
    RCLCPP_ERROR(this->get_logger(), "Failed to execute retreat from pick task");
    return false;
  }

  return true;
}

mtc::Task KOIPickPlaceController::createMoveToPickTask(){
  sampling_planner_ = std::make_shared<mtc::solvers::PipelinePlanner>(this->shared_from_this());
  mtc::Task move_to_pick_task;
  move_to_pick_task.stages()->setName("pick task");
  move_to_pick_task.loadRobotModel(this->shared_from_this());

  // Set task properties
  move_to_pick_task.setProperty("group", arm_group_name_);
  move_to_pick_task.setProperty("eef", "vacuum");
  move_to_pick_task.setProperty("ik_frame", hand_frame_);

  mtc::Stage *current_state_ptr = nullptr; // Forward current_state on to grasp pose generator
  auto stage_state_current = std::make_unique<stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  move_to_pick_task.add(std::move(stage_state_current));

  // Stages
  this->addMoveToPickStage(move_to_pick_task);
  this->addApproachObjectStage(move_to_pick_task);
  this->addSampleGraspStage(move_to_pick_task, current_state_ptr);

  return move_to_pick_task;
}

mtc::Task KOIPickPlaceController::createRetreatFromPickTask(){
  mtc::Task retreat_from_pick_task;
  retreat_from_pick_task.stages()->setName("retreat from pick task");
  retreat_from_pick_task.loadRobotModel(this->shared_from_this());

  // Set task properties
  retreat_from_pick_task.setProperty("group", arm_group_name_);
  retreat_from_pick_task.setProperty("eef", "vacuum");
  retreat_from_pick_task.setProperty("ik_frame", hand_frame_);

  auto stage_state_current = std::make_unique<stages::CurrentState>("current");
  retreat_from_pick_task.add(std::move(stage_state_current));

  // Stages
  // this->addAttachObjectStage(retreat_from_pick_task);
  this->addLiftObjectStage(retreat_from_pick_task);
  this->addRetreatStage(retreat_from_pick_task);

  return retreat_from_pick_task;
}

bool KOIPickPlaceController::doPlaceTask(geometry_msgs::msg::PoseStamped &target_pose_stamped){
  place_task_ = this->createPlaceTask(target_pose_stamped);

  try{
    place_task_.init();
  }
  catch (mtc::InitStageException &e){
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to initialize place task: " << e.what());
    RCLCPP_ERROR_STREAM(this->get_logger(), e);
    return false;
  }

  if (!place_task_.plan(5)){
    RCLCPP_ERROR(this->get_logger(), "Failed to plan place task");
    return false;
  }
  place_task_.introspection().publishSolution(*place_task_.solutions().front());

  auto result = place_task_.execute(*place_task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS){
    RCLCPP_ERROR(this->get_logger(), "Failed to execute place task");
    return false;
  }

  planning_scene_interface_.removeCollisionObjects({current_obj_.name});
  return true;
}

mtc::Task KOIPickPlaceController::createPlaceTask(const geometry_msgs::msg::PoseStamped &target_pose_stamped){
  sampling_planner_ = std::make_shared<mtc::solvers::PipelinePlanner>(this->shared_from_this());
  mtc::Task place_task;
  place_task.stages()->setName("place task");
  place_task.loadRobotModel(this->shared_from_this());

  // Set task properties
  place_task.setProperty("group", arm_group_name_);
  place_task.setProperty("eef", "vacuum");
  place_task.setProperty("ik_frame", hand_frame_);

  mtc::Stage *current_state_ptr = nullptr; // Forward current_state on to grasp pose generator
  auto stage_state_current = std::make_unique<stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  place_task.add(std::move(stage_state_current));

  this->addMoveToPlaceStage(place_task);
  this->addSamplePlacePoseStage(place_task, current_state_ptr, target_pose_stamped);
  this->addDetachObjectStage(place_task);
  this->addRetreatStage(place_task);
  this->addReturnHomeStage(place_task);

  return place_task;
}