#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <custom_messages/msg/map.hpp>
#include <vector>
#include <array>

namespace cr5_demo {

struct PlantWithCenter {
    moveit_msgs::msg::CollisionObject plant_shape;
    geometry_msgs::msg::Point center;
};

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
    
    /**
     * @brief Sets up the basic floor obstacle for safety
     * @param move_group MoveIt interface for the robot
     * @param planning_scene_interface Interface to modify planning scene
     */
    void setupFloorObstacle(moveit::planning_interface::MoveGroupInterface& move_group,
                          moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);

private:
    std::shared_ptr<rclcpp::Node> node_;
    
    /**
     * @brief Calculates dimensions of a plant from position boundaries
     * @param high_pos_x X position boundaries
     * @param high_pos_y Y position boundaries  
     * @param high_pos_z Z position boundaries
     * @return Array with [width, depth, height]
     */
    std::array<float, 3> calculateDimPlant(std::array<float, 2> high_pos_x,
                                          std::array<float, 2> high_pos_y,
                                          std::array<float, 2> high_pos_z);
    
    /**
     * @brief Calculates center point of an obstacle
     * @param high_pos_x X position boundaries
     * @param high_pos_y Y position boundaries
     * @param high_pos_z Z position boundaries
     * @return Center point of the obstacle
     */
    geometry_msgs::msg::Point calculateObstacleCenter(std::array<float, 2> high_pos_x,
                                                     std::array<float, 2> high_pos_y,
                                                     std::array<float, 2> high_pos_z);
    
    /**
     * @brief Creates a collision object for a plant
     * @param name Name identifier for the plant
     * @param dim Dimensions of the plant [width, depth, height]
     * @param center Center position of the plant
     * @param planning_scene_interface Interface to add the object
     * @param move_group MoveIt interface for the robot
     * @return PlantWithCenter structure containing the plant data
     */
    PlantWithCenter addObstacle(std::string name,
                               std::array<float, 3> dim,
                               geometry_msgs::msg::Point center,
                               moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                               moveit::planning_interface::MoveGroupInterface& move_group);
};

} // namespace cr5_demo