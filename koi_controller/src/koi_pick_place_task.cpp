#include "koi_controller/koi_pick_place_controller.hpp"

bool KOIPickPlaceController::doPickTask(){
  pick_task_ = this->createPickTask();

  try{
    pick_task_.init();
  }
  catch (mtc::InitStageException &e){
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to initialize pick task: " << e.what());
    RCLCPP_ERROR_STREAM(this->get_logger(), e);
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
  sampling_planner_ = std::make_shared<mtc::solvers::PipelinePlanner>(this->shared_from_this());
  mtc::Task pick_task;
  pick_task.stages()->setName("pick task");
  pick_task.loadRobotModel(this->shared_from_this());

  // Set task properties
  pick_task.setProperty("group", arm_group_name_);
  pick_task.setProperty("eef", "vacuum");
  pick_task.setProperty("ik_frame", hand_frame_);

  mtc::Stage *current_state_ptr = nullptr; // Forward current_state on to grasp pose generator
  auto stage_state_current = std::make_unique<stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  pick_task.add(std::move(stage_state_current));

  // Stages
  this->addMoveToPickStage(pick_task);
  this->addApproachObjectStage(pick_task);
  this->addSampleGraspStage(pick_task, current_state_ptr);
  this->addAttachObjectStage(pick_task);
  this->addLiftObjectStage(pick_task);

  return pick_task;
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