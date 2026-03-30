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

  attach_object_stage_ = nullptr; // Will be set in addAttachObjectStage

  // Stages
  this->addMoveToPickStage(pick_task);
  this->addApproachObjectStage(pick_task);
  this->addSampleGraspStage(pick_task, current_state_ptr);
  this->addAttachObjectStage(pick_task);
  this->addLiftObjectStage(pick_task);

  return pick_task;
}