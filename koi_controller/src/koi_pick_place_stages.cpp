#include "koi_controller/koi_pick_place_controller.hpp"

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
  stage_approach->setIKFrame(hand_frame_);

  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = hand_frame_;
  vec.vector.z = 1.0;
  stage_approach->setDirection(vec);
  pick_task.add(std::move(stage_approach));
}

void KOIPickPlaceController::addSampleGraspStage(mtc::Task &pick_task, mtc::Stage *current_state_ptr){
   // Sample grasp pose
  auto stage_sample_grasp = std::make_unique<stages::GeneratePose>("generate grasp pose");
  stage_sample_grasp->properties().configureInitFrom(mtc::Stage::PARENT);
  stage_sample_grasp->properties().set("marker_ns", "grasp_pose");

  grasp_pose_.pose.position = current_obj_.pose.position;
  grasp_pose_.pose.position.z += 0.1; // Ensure the grasp pose is above the object

  stage_sample_grasp->setPose(grasp_pose_);
  stage_sample_grasp->setMonitoredStage(current_state_ptr); // Forward current state to grasp pose generator for informed sampling

  auto ik_wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage_sample_grasp));
  ik_wrapper->setMaxIKSolutions(8);
  ik_wrapper->setMinSolutionDistance(1.0);

  ik_wrapper->setIKFrame(hand_frame_);
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