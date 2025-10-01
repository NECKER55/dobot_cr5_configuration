#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <custom_messages/msg/map.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "modules/scene_manager.hpp"
#include "modules/plant_scanner.hpp"

namespace cr5_demo {

class PlantProcessor {
public:
    PlantProcessor(std::shared_ptr<rclcpp::Node> node);
    
    /**
     * @brief Callback function for processing received maps
     * @param msg Map message from reworked_map
     * @param move_group MoveIt interface for the robot
     * @param planning_scene_interface Interface to modify planning scene
     */
    void mapCallback(const custom_messages::msg::Map::SharedPtr msg,
                    moveit::planning_interface::MoveGroupInterface& move_group,
                    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);
    
    /**
     * @brief Processes all target plants in the map
     * @param move_group MoveIt interface for the robot
     */
    void processTargetPlants(moveit::planning_interface::MoveGroupInterface& move_group);

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<SceneManager> scene_manager_;
    std::shared_ptr<PlantScanner> plant_scanner_;
    
    // Current map being processed
    custom_messages::msg::Map current_map_;
    
    /**
     * @brief Calculates the center point for plant scanning
     * @param object The object containing shape information
     * @param use_first_scan Whether this is the first scan (different height)
     * @return Center point for camera positioning
     */
    geometry_msgs::msg::Point calculateScanCenter(const custom_messages::msg::Object& object,
                                                 bool use_first_scan);
};

} // namespace cr5_demo