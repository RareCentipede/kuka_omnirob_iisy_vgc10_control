#ifndef KOI_PICK_PLACE_CONTROLLER_HPP_
#define KOI_PICK_PLACE_CONTROLLER_HPP_

#include <cstdio>
#include <memory>
#include <cmath>
#include <array>
#include <eigen3/Eigen/Dense>
#include <format>
#include <string>
#include <optional>
#include <format>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mpnp_interfaces/srv/pick.hpp"
#include "mpnp_interfaces/srv/place.hpp"
#include "mpnp_interfaces/msg/object.hpp"

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace mtc = moveit::task_constructor;
namespace stages = mtc::stages;

struct Object{
    std::string name;
    geometry_msgs::msg::Pose pose;
};

class KOIPickPlaceController: public rclcpp::Node{
    public:
        KOIPickPlaceController(const rclcpp::NodeOptions &options);
        ~KOIPickPlaceController();
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

        bool doPickTask();
        bool doPlaceTask(geometry_msgs::msg::PoseStamped &target_pose);
        void setupPlanningScene(const std::string &object, const geometry_msgs::msg::Pose &pose, const char *frame_id);

    private:
        const std::string arm_group_name_ = "arm_controller";
        const std::string hand_group_name_ = "vacuum_controller";
        const std::string hand_frame_ = "suction";

        Object current_obj_;
        geometry_msgs::msg::PoseStamped grasp_pose_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::CallbackGroup::SharedPtr cb_group_;
        rclcpp::SubscriptionOptions subscription_options_;

        rclcpp::Service<mpnp_interfaces::srv::Pick>::SharedPtr pick_service_;
        rclcpp::Service<mpnp_interfaces::srv::Place>::SharedPtr place_service_;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

        void pick_service(const std::shared_ptr<mpnp_interfaces::srv::Pick::Request> request,
            std::shared_ptr<mpnp_interfaces::srv::Pick::Response> response);
        void place_service(const std::shared_ptr<mpnp_interfaces::srv::Place::Request> request,
                std::shared_ptr<mpnp_interfaces::srv::Place::Response> response);

        std::optional<geometry_msgs::msg::Pose> compute_target_pose(const std::string &object_name,
                                                                       const std::string &obj_frame_name);

        mtc::Task createPickTask();
        void addMoveToPickStage(mtc::Task &pick_task);
        void addApproachObjectStage(mtc::Task &pick_task);
        void addSampleGraspStage(mtc::Task &pick_task, mtc::Stage *current_state_ptr);
        void addAttachObjectStage(mtc::Task &pick_task);
        void addLiftObjectStage(mtc::Task &pick_task);

        mtc::Task createPlaceTask(const geometry_msgs::msg::PoseStamped &target_pose);
        void addMoveToPlaceStage(mtc::Task &place_task);
        void addSamplePlacePoseStage(mtc::Task &place_task, mtc::Stage *attach_object_stage,
                                     const geometry_msgs::msg::PoseStamped &target_pose);
        void addDetachObjectStage(mtc::Task &place_task);
        void addRetreatStage(mtc::Task &place_task);
        void addReturnHomeStage(mtc::Task &place_task);

        mtc::solvers::PipelinePlannerPtr sampling_planner_;
        mtc::solvers::JointInterpolationPlannerPtr interpolation_planner_;
        mtc::solvers::CartesianPathPtr cartesian_planner_;

        mtc::Task pick_task_;
        mtc::Task place_task_;
};

#endif  // KOI_PICK_PLACE_CONTROLLER_HPP_