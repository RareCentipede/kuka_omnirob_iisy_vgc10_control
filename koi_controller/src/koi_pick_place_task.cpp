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
  this->addAttachObjectStage(retreat_from_pick_task);
  this->addLiftObjectStage(retreat_from_pick_task);
  this->addRetreatStage(retreat_from_pick_task);

  return retreat_from_pick_task;
}

bool KOIPickPlaceController::doMoveToPlaceTask(const geometry_msgs::msg::PoseStamped &target_pose_stamped){
  auto move_to_place_task = this->createMoveToPlaceTask(target_pose_stamped);

  try{
    move_to_place_task.init();
  }
  catch (mtc::InitStageException &e){
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to initialize move to place task: " << e.what());
    RCLCPP_ERROR_STREAM(this->get_logger(), e);
    return false;
  }

  if (!move_to_place_task.plan(5)){
    RCLCPP_ERROR(this->get_logger(), "Failed to plan move to place task");
    return false;
  }
  move_to_place_task.introspection().publishSolution(*move_to_place_task.solutions().front());

  auto result = move_to_place_task.execute(*move_to_place_task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS){
    RCLCPP_ERROR(this->get_logger(), "Failed to execute move to place task");
    return false;
  }

  planning_scene_interface_.removeCollisionObjects({current_obj_.name});
  return true;
}

bool KOIPickPlaceController::doReturnHomeTask(){
  auto return_home_task = this->createReturnHomeTask();

  try{
    return_home_task.init();
  }
  catch (mtc::InitStageException &e){
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to initialize move to place task: " << e.what());
    RCLCPP_ERROR_STREAM(this->get_logger(), e);
    return false;
  }

  if (!return_home_task.plan(5)){
    RCLCPP_ERROR(this->get_logger(), "Failed to plan move to place task");
    return false;
  }
  return_home_task.introspection().publishSolution(*return_home_task.solutions().front());

  auto result = return_home_task.execute(*return_home_task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS){
    RCLCPP_ERROR(this->get_logger(), "Failed to execute move to place task");
    return false;
  }

  planning_scene_interface_.removeCollisionObjects({current_obj_.name});
  return true;
}

mtc::Task KOIPickPlaceController::createMoveToPlaceTask(const geometry_msgs::msg::PoseStamped &target_pose_stamped){
  sampling_planner_ = std::make_shared<mtc::solvers::PipelinePlanner>(this->shared_from_this());
  mtc::Task place_task;
  place_task.stages()->setName("move to place task");
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
  return place_task;
}

mtc::Task KOIPickPlaceController::createReturnHomeTask(){
  mtc::Task return_home_task;
  return_home_task.stages()->setName("return home task");
  return_home_task.loadRobotModel(this->shared_from_this());

  // Set task properties
  return_home_task.setProperty("group", arm_group_name_);
  return_home_task.setProperty("eef", "vacuum");
  return_home_task.setProperty("ik_frame", hand_frame_);

  auto stage_state_current = std::make_unique<stages::CurrentState>("current");
  return_home_task.add(std::move(stage_state_current));

  this->addDetachObjectStage(return_home_task);
  this->addRetreatStage(return_home_task);
  this->addReturnHomeStage(return_home_task);

  return return_home_task;
}