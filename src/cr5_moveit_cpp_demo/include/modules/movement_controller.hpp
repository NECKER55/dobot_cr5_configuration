#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>

using namespace std::chrono_literals;

namespace cr5_demo {

struct ScanPoint {
    geometry_msgs::msg::Pose pose;
    bool is_boundary;
    bool covered;
    double optimality;  // Store the original optimality value
    int index;          // Store the original index
};

class MovementController {
public:
    MovementController(std::shared_ptr<rclcpp::Node> node);
    
    /**
     * @brief Attempts to plan and execute movement to a specific scan point
     * @param scan_point The point to reach
     * @param move_group MoveIt interface for the robot
     * @param point_index Index of the point for logging
     * @param node ROS2 node for logging
     * @return true if movement was successful, false otherwise
     */
    bool attemptToReachPoint(ScanPoint& scan_point,
                           moveit::planning_interface::MoveGroupInterface& move_group,
                           int point_index,
                           std::shared_ptr<rclcpp::Node> node);

private:
    std::shared_ptr<rclcpp::Node> node_;
    
    /**
     * @brief Plans movement to a target pose
     * @param target_pose The pose to reach
     * @param move_group MoveIt interface
     * @return true if planning was successful
     */
    bool planMovement(const geometry_msgs::msg::Pose& target_pose,
                     moveit::planning_interface::MoveGroupInterface& move_group);
    
    /**
     * @brief Executes a planned movement
     * @param move_group MoveIt interface
     * @return true if execution was successful
     */
    bool executeMovement(moveit::planning_interface::MoveGroupInterface& move_group);
    
    /**
     * @brief Marks points as covered by camera FOV around current position
     * @param scan_points Vector of all scan points
     * @param current_position Current robot position
     * @param camera_fov_radius Camera field of view radius
     * @return Number of points marked as covered
     */
    int markPointsAsCovered(std::vector<ScanPoint>& scan_points,
                           const geometry_msgs::msg::Point& current_position,
                           double camera_fov_radius = 0.15);
};

} // namespace cr5_demo