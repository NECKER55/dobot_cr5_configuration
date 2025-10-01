#include "modules/movement_controller.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

namespace cr5_demo {

// Constants
const double CAMERA_FOV_HORIZONTAL = 0.30; // Camera FOV radius in meters

MovementController::MovementController(std::shared_ptr<rclcpp::Node> node) 
    : node_(node) {
}

bool MovementController::attemptToReachPoint(ScanPoint& scan_point,
                                           moveit::planning_interface::MoveGroupInterface& move_group,
                                           int point_index,
                                           std::shared_ptr<rclcpp::Node> node) {
    if (scan_point.covered) {
        return false;
    }

    // Try to plan and execute movement
    RCLCPP_INFO(node->get_logger(), "[MOVEMENT] >> Planning movement to point %d...", point_index);
    
    move_group.setPoseTarget(scan_point.pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group.plan(plan));
    
    if (success) {
        RCLCPP_INFO(node->get_logger(), "[MOVEMENT] >> ✓ Planning successful for point %d", point_index);
        
        // Execute the plan
        RCLCPP_INFO(node->get_logger(), "[MOVEMENT] >> Executing movement...");
        moveit::core::MoveItErrorCode execution_result = move_group.execute(plan);
        
        if (execution_result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "[MOVEMENT] >> ✓ Movement completed successfully!");
            
            // Mark this point as covered
            scan_point.covered = true;
            
            RCLCPP_INFO(node->get_logger(), "[SCANNING] >> Waiting for scanning (3 seconds)...");
            rclcpp::sleep_for(3000ms); // Allow time for scanning
            return true;
        } else {
            RCLCPP_ERROR(node->get_logger(), "[MOVEMENT] >> ✗ Execution ERROR - code: %d", 
                        execution_result.val);
        }
    } else {
        RCLCPP_WARN(node->get_logger(), "[MOVEMENT] >> ✗ Planning failed for point %d", point_index);
    }
    
    return false;
}

bool MovementController::planMovement(const geometry_msgs::msg::Pose& target_pose,
                                     moveit::planning_interface::MoveGroupInterface& move_group) {
    move_group.setPoseTarget(target_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    return static_cast<bool>(move_group.plan(plan));
}

bool MovementController::executeMovement(moveit::planning_interface::MoveGroupInterface& move_group) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group.plan(plan));
    
    if (success) {
        moveit::core::MoveItErrorCode execution_result = move_group.execute(plan);
        return execution_result == moveit::core::MoveItErrorCode::SUCCESS;
    }
    
    return false;
}

int MovementController::markPointsAsCovered(std::vector<ScanPoint>& scan_points,
                                          const geometry_msgs::msg::Point& current_position,
                                          double camera_fov_radius) {
    int covered_count = 0;
    
    // Mark nearby points as covered based on camera FOV
    for (auto& point : scan_points) {
        if (!point.covered) {
            double dx = current_position.x - point.pose.position.x;
            double dy = current_position.y - point.pose.position.y;
            double dz = current_position.z - point.pose.position.z;
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            // Simple FOV model: if point is within FOV range, mark as covered
            if (dist < camera_fov_radius) {
                point.covered = true;
                covered_count++;
                RCLCPP_DEBUG(node_->get_logger(), "        Covered point at distance %.3f", dist);
            }
        }
    }
    
    return covered_count;
}

} // namespace cr5_demo