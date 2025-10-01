#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <custom_messages/msg/optimal_point.hpp>
#include <custom_messages/msg/circumference.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include "modules/movement_controller.hpp"

namespace cr5_demo {

class PlantScanner {
public:
    PlantScanner(std::shared_ptr<rclcpp::Node> node);
    
    /**
     * @brief Processes the circumference around a target plant
     * @param target_to_watch Center point of the plant to watch
     * @param circumference Vector of optimal points around the plant
     * @param move_group MoveIt interface for the robot
     * @param trajectory_index Current trajectory index for logging
     */
    void processCircumference(geometry_msgs::msg::Point& target_to_watch,
                             const custom_messages::msg::Circumference& circumference,
                             moveit::planning_interface::MoveGroupInterface& move_group,
                             [[maybe_unused]] int trajectory_index);

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MovementController> movement_controller_;
    
    // Camera FOV constants
    static constexpr double CAMERA_FOV_HORIZONTAL = 0.15;
    
    /**
     * @brief Identifies boundary points that should be prioritized for scanning
     * @param scan_points Vector of all scan points to analyze
     */
    void identifyBoundaryPoints(std::vector<ScanPoint>& scan_points);
    
    /**
     * @brief Scans remaining uncovered points after boundary scan
     * @param scan_points Vector of all scan points
     * @param target_to_watch Center point of the plant
     * @param move_group MoveIt interface for the robot
     */
    void scanUncoveredPoints(std::vector<ScanPoint>& scan_points,
                            [[maybe_unused]] geometry_msgs::msg::Point& target_to_watch,
                            moveit::planning_interface::MoveGroupInterface& move_group);
    
    /**
     * @brief Creates a pose that points towards the plant center
     * @param point The optimal point position
     * @param center The center of the plant to point towards
     * @return Pose with position and orientation
     */
    geometry_msgs::msg::Pose createPosePointingToCenter(
        const custom_messages::msg::OptimalPoint& point,
        const geometry_msgs::msg::Point& center);
    
    /**
     * @brief Calculates distance between two points
     * @param p1 First point
     * @param p2 Second point
     * @return Distance between points
     */
    double calculateDistance(const custom_messages::msg::OptimalPoint& p1, 
                           const custom_messages::msg::OptimalPoint& p2);
    
    /**
     * @brief Marks points as covered based on camera FOV from current position
     * @param current_point The current scan point position
     * @param all_points Vector of all scan points
     * @return Number of points marked as covered
     */
    int markPointsAsCovered(ScanPoint& current_point, std::vector<ScanPoint>& all_points);
    
    /**
     * @brief Converts optimal points to scan points structure
     * @param optimal_points Input optimal points
     * @param target_center Center of the plant for orientation calculation
     * @return Vector of scan points
     */
    std::vector<ScanPoint> convertToScanPoints(
        const std::vector<custom_messages::msg::OptimalPoint>& optimal_points,
        const geometry_msgs::msg::Point& target_center);
};

} // namespace cr5_demo