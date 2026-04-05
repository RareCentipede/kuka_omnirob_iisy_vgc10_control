#include "koi_controller/koi_pick_place_controller.hpp"

  /****************************************************
---- *                Picking stages                 *
    ***************************************************/

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
  grasp_pose_.pose.position.z += 0.18; // Ensure the grasp pose is above the object

  stage_sample_grasp->setPose(grasp_pose_);
  stage_sample_grasp->setMonitoredStage(current_state_ptr); // Forward current state to grasp pose generator for informed sampling

  auto ik_wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage_sample_grasp));
  ik_wrapper->setMaxIKSolutions(8);
  ik_wrapper->setMinSolutionDistance(0.1);

  ik_wrapper->setIKFrame(hand_frame_);
  ik_wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
  ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
  pick_task.add(std::move(ik_wrapper));
}

void KOIPickPlaceController::addAttachObjectStage(mtc::Task &pick_task){
  auto stage_attach = std::make_unique<stages::ModifyPlanningScene>("attach obj");
  stage_attach->attachObject(current_obj_.name, hand_frame_);
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

  /****************************************************
---- *                Placing stages                 *
    ***************************************************/

void KOIPickPlaceController::addMoveToPlaceStage(mtc::Task &place_task){
  auto stage_move_to_place = std::make_unique<stages::Connect>(
      "move to place",
      stages::Connect::GroupPlannerVector{
      {arm_group_name_, sampling_planner_},
      {hand_group_name_, interpolation_planner_}});

  stage_move_to_place->setTimeout(5.0);
  stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
  place_task.add(std::move(stage_move_to_place));
}

void KOIPickPlaceController::addSamplePlacePoseStage(mtc::Task &place_task, mtc::Stage *attach_object_stage,
                                                     const geometry_msgs::msg::PoseStamped &target_pose_stamped){
  auto stage_generate_place_pose = std::make_unique<stages::GeneratePlacePose>("generate place pose");
  stage_generate_place_pose->properties().configureInitFrom(mtc::Stage::PARENT);
  stage_generate_place_pose->properties().set("marker_ns", "place_pose");
  stage_generate_place_pose->setObject(current_obj_.name);

  grasp_pose_.pose.position = target_pose_stamped.pose.position;
  grasp_pose_.pose.position.z += 0.18; // Ensure the place pose is above the target position

  stage_generate_place_pose->setPose(grasp_pose_);
  stage_generate_place_pose->setMonitoredStage(attach_object_stage); // Forward attach object stage state

  auto ik_wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage_generate_place_pose));
  ik_wrapper->setMaxIKSolutions(8);
  ik_wrapper->setMinSolutionDistance(0.1);
  ik_wrapper->setIKFrame(hand_frame_);
  ik_wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
  ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
  place_task.add(std::move(ik_wrapper));
}

void KOIPickPlaceController::addDetachObjectStage(mtc::Task &place_task){
  auto stage_modify_scene = std::make_unique<stages::ModifyPlanningScene>("disallow collision (hand, obj)");
  stage_modify_scene->allowCollisions(
    current_obj_.name,
    place_task.getRobotModel()
        ->getJointModelGroup(hand_group_name_)
        ->getLinkModelNamesWithCollisionGeometry(),
        false
  );
  place_task.add(std::move(stage_modify_scene));

  auto stage_detach = std::make_unique<stages::ModifyPlanningScene>("detach object");
  stage_detach->detachObject(current_obj_.name, hand_frame_);
  place_task.add(std::move(stage_detach));
}

void KOIPickPlaceController::addRetreatStage(mtc::Task &place_task){
  auto stage_retreat = std::make_unique<stages::MoveRelative>("retreat", cartesian_planner_);
  stage_retreat->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_retreat->setMinMaxDistance(0.1, 0.3);
  stage_retreat->setIKFrame(hand_frame_);
  stage_retreat->properties().set("marker_ns", "retreat");

  // Set backward direction
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = hand_frame_;
  // vec.vector.x = 1.0;
  vec.vector.z = -1.0;
  stage_retreat->setDirection(vec);
  place_task.add(std::move(stage_retreat));
}

void KOIPickPlaceController::addReturnHomeStage(mtc::Task &place_task){
  auto stage_return_home = std::make_unique<stages::MoveTo>("return home", interpolation_planner_);
  stage_return_home->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_return_home->properties().set("marker_ns", "return_home");
  stage_return_home->setGoal("home");
  place_task.add(std::move(stage_return_home));
}