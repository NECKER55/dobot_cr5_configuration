#include "modules/movement_controller.hpp"

namespace cr5_demo {

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

            RCLCPP_INFO(node->get_logger(), "[SCANNING] >> Waiting for scanning (11 seconds)...");
            rclcpp::sleep_for(11000ms); // Allow time for scanning
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

} // namespace cr5_demo