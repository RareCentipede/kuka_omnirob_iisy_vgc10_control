#ifndef KOI_PICK_PLACE_CONTROLLER_HPP_
#define KOI_PICK_PLACE_CONTROLLER_HPP_

#include <cstdio>
#include <memory>
#include <cmath>
#include <array>
#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mpnp_interfaces/srv/pick.hpp"
#include "mpnp_interfaces/srv/place.hpp"

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace mtc = moveit::task_constructor;
namespace stages = moveit::task_constructor::stages;

class KOIPickPlaceController: public rclcpp::Node{
    public:
        KOIPickPlaceController(const rclcpp::NodeOptions &options);
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

        void doPickTask();
        void doPlaceTask();
        void setupPlanningScene();

    private:
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

        mtc::Task createPickTask();
        mtc::Task createPlaceTask();
        mtc::Task pick_task_;
        mtc::Task place_task_;
};

#endif  // KOI_PICK_PLACE_CONTROLLER_HPP_