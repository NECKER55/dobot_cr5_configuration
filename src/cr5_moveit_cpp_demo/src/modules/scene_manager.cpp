#include "modules/scene_manager.hpp"
#include <chrono>
#include <cmath>
#include <iostream>

using namespace std::chrono_literals;

namespace cr5_demo {

SceneManager::SceneManager(std::shared_ptr<rclcpp::Node> node) : node_(node) {
}

void SceneManager::addObstaclesToScene(moveit::planning_interface::MoveGroupInterface& move_group,
                                     moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                                     const custom_messages::msg::Map& map) {
    RCLCPP_INFO(node_->get_logger(), "[OBSTACLES] Adding obstacles to planning scene...");
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    
    // Add workspace boundaries as collision object
    RCLCPP_INFO(node_->get_logger(), "[OBSTACLES] > Creating workspace floor...");
    moveit_msgs::msg::CollisionObject workspace;
    workspace.header.frame_id = move_group.getPlanningFrame();
    workspace.id = "workspace_boundaries";
    
    // Add floor
    shape_msgs::msg::SolidPrimitive floor_primitive;
    floor_primitive.type = floor_primitive.BOX;
    floor_primitive.dimensions = {2.0, 2.0, 0.01};
    
    geometry_msgs::msg::Pose floor_pose;
    floor_pose.position.x = 0;
    floor_pose.position.y = 0;
    floor_pose.position.z = -0.005;
    floor_pose.orientation.w = 1.0;
    
    workspace.primitives.push_back(floor_primitive);
    workspace.primitive_poses.push_back(floor_pose);
    workspace.operation = workspace.ADD;
    collision_objects.push_back(workspace);
    
    RCLCPP_INFO(node_->get_logger(), "[OBSTACLES] > ✓ Floor added at z=%.3f", floor_pose.position.z);
    
    // Add all objects from map as collision objects
    RCLCPP_INFO(node_->get_logger(), "[OBSTACLES] > Adding objects from map...");
    for (size_t i = 0; i < map.objects.size(); ++i) {
        const auto& object = map.objects[i];
        moveit_msgs::msg::CollisionObject collision_obj;
        collision_obj.header.frame_id = move_group.getPlanningFrame();
        collision_obj.id = "object_" + std::to_string(i);
        
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        
        double size_x = object.shape.top_right.x - object.shape.low_left.x + 0.01; // Add small margin
        double size_y = object.shape.top_right.y - object.shape.low_left.y + 0.01; // Add small margin
        double size_z = object.shape.top_right.z - object.shape.low_left.z + 0.01; // Add small margin
        primitive.dimensions = {size_x, size_y, size_z};
        
        geometry_msgs::msg::Pose box_pose;
        box_pose.position.x = object.shape.low_left.x + size_x / 2.0;
        box_pose.position.y = object.shape.low_left.y + size_y / 2.0;
        box_pose.position.z = object.shape.low_left.z + size_z / 2.0;
        box_pose.orientation.w = 1.0;
        
        collision_obj.primitives.push_back(primitive);
        collision_obj.primitive_poses.push_back(box_pose);
        collision_obj.operation = collision_obj.ADD;
        collision_objects.push_back(collision_obj);
        
        std::string obj_type = object.target ? "TARGET" : "OBSTACLE";
        RCLCPP_INFO(node_->get_logger(), "[OBSTACLES] > ✓ %s_%zu: size=(%.3fx%.3fx%.3f) center=(%.3f,%.3f,%.3f)",
                   obj_type.c_str(), i, size_x, size_y, size_z,
                   box_pose.position.x, box_pose.position.y, box_pose.position.z);
    }
    
    planning_scene_interface.applyCollisionObjects(collision_objects);
    rclcpp::sleep_for(1s);
    RCLCPP_INFO(node_->get_logger(), "[OBSTACLES] ✓ Added %zu collision objects to scene", collision_objects.size());
}

void SceneManager::setupFloorObstacle(moveit::planning_interface::MoveGroupInterface& move_group,
                                     moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
    // Setup basic floor obstacle for safety testing
    moveit_msgs::msg::CollisionObject obstacle;
    obstacle.header.frame_id = move_group.getPlanningFrame();
    obstacle.id = "floor";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {2, 2, 0.5};

    geometry_msgs::msg::Point obstacle_center = calculateObstacleCenter({-1,1}, {-1,1}, {0, -0.5});
    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = obstacle_center.x;
    box_pose.position.y = obstacle_center.y;
    box_pose.position.z = obstacle_center.z;
    box_pose.orientation.w = 1.0;

    obstacle.primitives.push_back(primitive);
    obstacle.primitive_poses.push_back(box_pose);
    obstacle.operation = obstacle.ADD;

    planning_scene_interface.applyCollisionObjects({obstacle});
    rclcpp::sleep_for(1s);
}

std::array<float, 3> SceneManager::calculateDimPlant(std::array<float, 2> high_pos_x,
                                                     std::array<float, 2> high_pos_y,
                                                     std::array<float, 2> high_pos_z) {
    float length = abs(high_pos_x[0] - high_pos_x[1]); // x
    float depth = abs(high_pos_y[0] - high_pos_y[1]); // y
    float height = abs(high_pos_z[0] - high_pos_z[1]); // z
    
    // Calculate diagonal (used elsewhere in original code)
    double diagonal = std::sqrt(std::pow(length, 2) + std::pow(depth, 2) + std::pow(height, 2))/2;
    std::cout << diagonal << std::endl;

    std::array<float, 3> dim = {length, depth, height};
    return dim;
}

geometry_msgs::msg::Point SceneManager::calculateObstacleCenter(std::array<float, 2> high_pos_x,
                                                               std::array<float, 2> high_pos_y,
                                                               std::array<float, 2> high_pos_z) {
    geometry_msgs::msg::Point center;
    
    center.x = high_pos_x[0] + (high_pos_x[1] - high_pos_x[0])/2;
    center.y = high_pos_y[0] + (high_pos_y[1] - high_pos_y[0])/2;
    center.z = high_pos_z[0] + (high_pos_z[1] - high_pos_z[0])/2;

    return center;
}

PlantWithCenter SceneManager::addObstacle(std::string name,
                                         std::array<float, 3> dim,
                                         geometry_msgs::msg::Point center,
                                         moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                                         moveit::planning_interface::MoveGroupInterface& move_group) {
    // Create collision object
    moveit_msgs::msg::CollisionObject obstacle;
    obstacle.header.frame_id = move_group.getPlanningFrame();
    obstacle.id = name;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {dim[0], dim[1], dim[2]};

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = center.x;
    box_pose.position.y = center.y;
    box_pose.position.z = center.z;
    box_pose.orientation.w = 1.0;

    obstacle.primitives.push_back(primitive);
    obstacle.primitive_poses.push_back(box_pose);
    obstacle.operation = obstacle.ADD;

    planning_scene_interface.applyCollisionObjects({obstacle});
    rclcpp::sleep_for(1s);

    // Create and return PlantWithCenter
    PlantWithCenter plant;
    plant.plant_shape = obstacle;
    plant.center = center;
    
    return plant;
}

} // namespace cr5_demo