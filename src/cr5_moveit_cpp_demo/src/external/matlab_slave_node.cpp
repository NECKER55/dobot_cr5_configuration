#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace cr5_demo {

class MatlabSlaveNode : public rclcpp::Node
{
public:
    MatlabSlaveNode() : Node("matlab_slave_node")
    {
        // Initialize MoveIt interfaces
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "cr5_arm");
        
        // Set planning parameters
        move_group_->setPlanningTime(10.0);
        move_group_->setNumPlanningAttempts(5);
        move_group_->setMaxVelocityScalingFactor(0.3);
        move_group_->setMaxAccelerationScalingFactor(0.3);
        
        // Subscribe to matlab_slave topic
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "matlab_slave", 10,
            std::bind(&MatlabSlaveNode::pose_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Matlab Slave Node initialized");
        RCLCPP_INFO(this->get_logger(), "Waiting for poses on 'matlab_slave' topic...");
        
        // Log current robot state
        log_current_pose();
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), 
            "Received target pose: position(%.3f, %.3f, %.3f), orientation(%.3f, %.3f, %.3f, %.3f)",
            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
            msg->pose.orientation.x, msg->pose.orientation.y, 
            msg->pose.orientation.z, msg->pose.orientation.w);
        
        // Validate the pose
        if (!is_pose_valid(msg->pose)) {
            RCLCPP_ERROR(this->get_logger(), "Received invalid pose, skipping movement");
            return;
        }
        
        // Attempt to move to the target pose
        move_to_pose(msg->pose);
    }
    
    bool is_pose_valid(const geometry_msgs::msg::Pose& pose)
    {
        // Check if position is within reasonable bounds (in meters)
        const double max_reach = 1.0;  // Adjust based on CR5 workspace
        const double min_height = 0.1; // Minimum safe height
        
        double distance = std::sqrt(
            pose.position.x * pose.position.x + 
            pose.position.y * pose.position.y);
        
        if (distance > max_reach) {
            RCLCPP_WARN(this->get_logger(), 
                "Target position too far from robot base: %.3f m (max: %.3f m)", 
                distance, max_reach);
            return false;
        }
        
        if (pose.position.z < min_height) {
            RCLCPP_WARN(this->get_logger(), 
                "Target height too low: %.3f m (min: %.3f m)", 
                pose.position.z, min_height);
            return false;
        }
        
        // Check if quaternion is normalized (approximately)
        double quat_norm = std::sqrt(
            pose.orientation.x * pose.orientation.x +
            pose.orientation.y * pose.orientation.y +
            pose.orientation.z * pose.orientation.z +
            pose.orientation.w * pose.orientation.w);
        
        if (std::abs(quat_norm - 1.0) > 0.1) {
            RCLCPP_WARN(this->get_logger(), 
                "Quaternion not normalized: norm = %.3f", quat_norm);
            return false;
        }
        
        return true;
    }
    
    void move_to_pose(const geometry_msgs::msg::Pose& target_pose)
    {
        try {
            RCLCPP_INFO(this->get_logger(), "Planning movement to target pose...");
            
            // Set the target pose
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = move_group_->getPlanningFrame();
            pose_stamped.header.stamp = this->now();
            pose_stamped.pose = target_pose;
            
            move_group_->setPoseTarget(pose_stamped);
            
            // Plan the trajectory
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            
            if (success) {
                RCLCPP_INFO(this->get_logger(), "Planning successful! Executing movement...");
                
                // Execute the planned trajectory
                auto result = move_group_->execute(plan);
                
                if (result == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(this->get_logger(), "Movement executed successfully!");
                    log_current_pose();
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to execute movement");
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Planning failed for target pose");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception during movement: %s", e.what());
        }
    }
    
    void log_current_pose()
    {
        try {
            geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();
            RCLCPP_INFO(this->get_logger(), 
                "Current robot pose: position(%.3f, %.3f, %.3f), orientation(%.3f, %.3f, %.3f, %.3f)",
                current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
                current_pose.pose.orientation.x, current_pose.pose.orientation.y, 
                current_pose.pose.orientation.z, current_pose.pose.orientation.w);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Could not get current pose: %s", e.what());
        }
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};

} // namespace cr5_demo

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<cr5_demo::MatlabSlaveNode>();
    
    RCLCPP_INFO(node->get_logger(), "Starting Matlab Slave Node...");
    
    // Use MultiThreadedExecutor for better performance
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    try {
        executor.spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in executor: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}