# Dobot CR5 ROS 2 Workspace

This repository contains the ROS 2 project for controlling a Dobot CR5 collaborative robot arm. The project focuses on automated path planning for scanning objects within a defined workspace, utilizing MoveIt! for motion planning and a custom perception system for environmental awareness.

## Project Structure

This workspace is structured to separate the main project logic from its external dependencies. It includes:
* A custom package for path planning and object scanning.
* A simulated camera node (`fake_zed`) to generate a map of the environment.
* A processing node (`reworked_map`) to evaluate optimal scanning positions.
* A Git submodule for the modified Dobot robot package.

### Key Components

* **`move_cr5_node`**: The core MoveIt! node responsible for orchestrating the robot's movements. It communicates with the planning scene and executes planned trajectories.
* **`fake_zed`**: A simulation node that publishes a `Map` message, defining the workspace boundaries and the location of various objects, including obstacles and a specific target plant to be scanned.
* **`reworked_map`**: A node that subscribes to the `Map` topic. It processes the map by calculating potential camera positions (circumferences) around the target plant. Each position is assigned an "optimality" score based on two criteria:
    1.  The point's position relative to the defined workspace boundaries.
    2.  The point's proximity to other obstacles.
* **`DOBOT_6Axis_ROS2_V3_mod` (Git Submodule)**: This directory contains the ROS 2 package for the Dobot CR5 robot arm. It has been included as a submodule because it was cloned from an external source and then modified to suit the needs of this project.

## Getting Started

### Prerequisites

* A Linux system with ROS 2 Humble installed.
* Git for version control.

### Installation and Setup

1.  **Clone the main repository:**
    Start by cloning this repository.

    ```bash
    git clone [https://github.com/NECKER55/dobot_cr5_configuration.git](https://github.com/NECKER55/dobot_cr5_configuration.git)
    cd dobot_cr5_configuration
    ```

    > **Note on Submodules:**
    > This repository includes the `DOBOT_6Axis_ROS2_V3` package as a Git submodule. When you clone the main repository, the submodule's content is not downloaded automatically. You must run a separate command to retrieve its files.

2.  **Initialize and Update the Submodule:**
    After cloning the main repository, run the following command to download and set up the submodule:

    ```bash
    git submodule update --init --recursive
    ```
    This command downloads the content of the submodule into the `src/DOBOT_6Axis_ROS2_V3` directory.

    **If you are setting up the submodule from a fresh repository (i.e., you have the code locally but the submodule is not yet linked):**
    First, ensure that the submodule's content is pushed to its own dedicated GitHub repository. Then, from the root of your main project, use the `git submodule add` command:

    ```bash
    # This command adds a remote repository as a submodule at the specified local path.
    # Replace the URL with the correct one for your modified Dobot package.
    git submodule add [https://github.com/NECKER55/DOBOT_6Axis_ROS2_V3_mod.git](https://github.com/NECKER55/DOBOT_6Axis_ROS2_V3_mod.git) src/DOBOT_6Axis_ROS2_V3
    ```
    After running `git submodule add`, you will need to commit this change to your main repository's history:
    ```bash
    git commit -m "Added Dobot package as a submodule"
    ```

3.  **Build the ROS 2 Workspace:**
    Build the entire workspace, including all custom packages and the submodule.

    ```bash
    colcon build --symlink-install
    ```

4.  **Source the Workspace:**
    Source your workspace to make the new packages available in your environment.

    ```bash
    source install/setup.bash
    ```

5.  **Launch the Simulation:**
    Run the main launch file to start Gazebo, MoveIt!, and all other necessary nodes. (Replace `your_launch_file.launch.py` with the actual launch file name).

    ```bash
    ros2 launch your_package_name your_launch_file.launch.py
    ```

## Troubleshooting

### `use_sim_time` Issues

If you encounter errors related to `Didn't receive robot state` or planning failures, it's highly likely that the `use_sim_time` parameter is not consistently set to `true` across all nodes in the simulation.

To fix this, ensure that your launch file explicitly sets `use_sim_time` to `true` for all relevant nodes, especially `move_group`, `robot_state_publisher`, and your custom nodes.

```python
# Example in a launch file
Node(
    package='moveit_ros_move_group',
    executable='move_group',
    parameters=[{'use_sim_time': True}],
    # ...
)
