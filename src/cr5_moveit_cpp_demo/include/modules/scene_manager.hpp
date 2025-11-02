#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <custom_messages/msg/map.hpp>
#include <vector>

namespace cr5_demo {

class SceneManager {
public:
    SceneManager(std::shared_ptr<rclcpp::Node> node);
    
    /**
     * @brief Adds obstacles from the map to the planning scene
     * @param move_group MoveIt interface for the robot
     * @param planning_scene_interface Interface to modify planning scene
     * @param map The map containing obstacles to add
     */
    void addObstaclesToScene(moveit::planning_interface::MoveGroupInterface& move_group,
                           moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                           const custom_messages::msg::Map& map);

private:
    std::shared_ptr<rclcpp::Node> node_;
};

} // namespace cr5_demo