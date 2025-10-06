
from launch import LaunchDescription
import os

# per caricare moveit (moveitrobot_state_publisher,move_group,planning_scene_monitor,kinematics + ompl_planning,static_transform_publisher)
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
#from moveit_configs_utils.launches import generate_move_group_launch
from ament_index_python.packages import get_package_share_directory



moveit_config = (
    MoveItConfigsBuilder("cr5_robot", package_name="cr5_moveit")
    .robot_description(file_path="config/cr5_robot.urdf.xacro")  # puoi tenerlo così anche se non usi ros2_control
    .robot_description_semantic(file_path="config/cr5_robot.srdf")
    .robot_description_kinematics(file_path="config/kinematics.yaml")
    .planning_scene_monitor(
        publish_robot_description=True,
        publish_robot_description_semantic=True,
        publish_planning_scene=True,
        publish_geometry_updates=True,
        publish_state_updates=True,
        publish_transforms_updates=True,
    )
    .trajectory_execution(file_path="config/moveit_controllers.yaml")  #    questo è fondamentale per usare il fake controller
    .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
    .to_moveit_configs()
)


run_move_group_node = Node(
    package="moveit_ros_move_group",
    executable="move_group",
    output="screen",
    parameters=[moveit_config.to_dict()],
)

# static transform publisher
static_tf = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="static_transform_publisher",
    output="log",
    arguments=[
        "0", "0", "0",      # xyz
        "0", "0", "0",      # rpy
        "world", "dummy_link" # -parent -child
    ]
)


# Publish TF
robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    name="robot_state_publisher",
    output="both",
    parameters=[moveit_config.robot_description],
)


translater_node = Node( #nodo che traduce i messaggi di moveit in quelli per le api del dobot
    package='dobot_bringup_v3', 
    executable='translater_node',  
    name='translater_node',
    output='screen'
)

listener_node = Node( # Nodo che ascolta i messaggi dalla zed
    package='dobot_bringup_v3', 
    executable='listener_node',  
    name='listener_node',
    output='screen'
)

joint_states_bridge_node = Node(
    package='dobot_bringup_v3', 
    executable='joint_states_bridge',  
    name='joint_states_bridge_node',
    output='screen'
)




rviz_node = Node(
    package="rviz2",  # Nome del pacchetto che contiene l'eseguibile di RViz.  (STANDARD NON PACCHETTO IN UNA REPO)
    executable="rviz2",  # Nome dell'eseguibile di RViz. In ROS2, il nome dell'eseguibile è "rviz2".
    name="rviz2",  # Nome del nodo che verrà creato in ROS2. Qui viene semplicemente chiamato "rviz2".
    output="screen",  # Specifica che l'output del nodo (log, errori, etc.) sarà visibile sulla console/schermo.
    arguments=['-d', os.path.join(  # Opzioni da passare all'eseguibile di RViz. Qui stiamo passando il file di configurazione RViz.
        get_package_share_directory("cr5_moveit"),
        "config",  
        "moveit.rviz"  #  file YAML che definisce come deve essere visualizzato il robot.
    )],
    parameters=[moveit_config.to_dict()]  # Parametri per configurare il nodo. Qui stiamo passando i parametri definiti da MoveIt! per configurare RViz.
)



def generate_launch_description():

    return LaunchDescription(
        [
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            #joint_states_bridge_node,
            translater_node,
            listener_node,
            rviz_node,
        ]
    )

