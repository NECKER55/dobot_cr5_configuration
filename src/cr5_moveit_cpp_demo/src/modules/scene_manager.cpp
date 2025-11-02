#include "modules/scene_manager.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace cr5_demo {

SceneManager::SceneManager(std::shared_ptr<rclcpp::Node> node) : node_(node) {
}

void SceneManager::addObstaclesToScene(moveit::planning_interface::MoveGroupInterface& move_group,
                                     moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                                     const custom_messages::msg::Map& map) {
    RCLCPP_INFO(node_->get_logger(), "[OBSTACLES] Adding obstacles to planning scene...");
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    
    // Add workspace boundaries as collision object, for now is not used, it could be usefull in the future
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

} // namespace cr5_demo