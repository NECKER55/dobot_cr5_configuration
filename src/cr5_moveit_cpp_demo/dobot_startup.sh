#!/bin/bash

# Dobot CR5 Automated Startup Script
# This script automates the entire robot startup process with error checking

set -e  # Exit on any error

# Configuration variables
ROBOT_IP="192.168.5.1"
LOCAL_IP="192.168.5.100"
NETWORK_INTERFACE=""
SUBNET="192.168.5.0/24"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to detect network interface
detect_network_interface() {
    log_info "Detecting available network interfaces..."
    
    # Get all network interfaces except loopback
    interfaces=$(ip link show | grep -E "^[0-9]+:" | grep -v lo | awk -F': ' '{print $2}' | cut -d'@' -f1)
    
    if [ -z "$interfaces" ]; then
        log_error "No network interfaces found!"
        exit 1
    fi
    
    echo "Available network interfaces:"
    select interface in $interfaces "Manual Input"; do
        case $interface in
            "Manual Input")
                read -p "Enter interface name manually: " NETWORK_INTERFACE
                break
                ;;
            *)
                if [ -n "$interface" ]; then
                    NETWORK_INTERFACE="$interface"
                    break
                fi
                echo "Invalid selection. Please try again."
                ;;
        esac
    done
    
    log_success "Selected network interface: $NETWORK_INTERFACE"
}

# Function to configure network
configure_network() {
    log_info "Configuring network interface $NETWORK_INTERFACE..."
    
    # Check if interface exists
    if ! ip link show "$NETWORK_INTERFACE" &>/dev/null; then
        log_error "Network interface $NETWORK_INTERFACE does not exist!"
        return 1
    fi
    
    # Disable NetworkManager management to avoid conflicts
    log_info "Disabling NetworkManager management for $NETWORK_INTERFACE..."
    if command -v nmcli &>/dev/null; then
        sudo nmcli dev set "$NETWORK_INTERFACE" managed no 2>/dev/null || {
            log_warning "Failed to disable NetworkManager management (continuing anyway)"
        }
        log_info "Restarting NetworkManager..."
        sudo systemctl restart NetworkManager 2>/dev/null || {
            log_warning "Failed to restart NetworkManager (continuing anyway)"
        }
        sleep 2  # Wait for NetworkManager to restart
    else
        log_info "NetworkManager not found, skipping management disable"
    fi
    
    # Check if IP is already configured
    if ip addr show "$NETWORK_INTERFACE" | grep -q "$LOCAL_IP"; then
        log_warning "IP $LOCAL_IP already configured on $NETWORK_INTERFACE"
        return 0
    fi
    
    # Configure IP address
    log_info "Adding IP $LOCAL_IP to interface $NETWORK_INTERFACE..."
    if sudo ip addr add "$LOCAL_IP/24" dev "$NETWORK_INTERFACE" 2>/dev/null; then
        log_success "IP address configured successfully"
    else
        log_warning "IP address might already be configured or failed to configure"
    fi
    
    # Bring interface up
    log_info "Bringing interface $NETWORK_INTERFACE up..."
    sudo ip link set "$NETWORK_INTERFACE" up
    
    return 0
}

# Function to test robot connectivity
test_robot_connectivity() {
    log_info "Testing connectivity to robot at $ROBOT_IP..."
    
    # Ping test with timeout
    if ping -c 3 -W 2 "$ROBOT_IP" &>/dev/null; then
        log_success "Robot is reachable at $ROBOT_IP"
        return 0
    else
        log_error "Cannot reach robot at $ROBOT_IP"
        log_info "Please check:"
        echo "  - Robot is powered on"
        echo "  - Robot IP is set to $ROBOT_IP"
        echo "  - Network cable is connected"
        echo "  - Robot and PC are on same network"
        return 1
    fi
}

# Function to check ROS2 environment
check_ros2_environment() {
    log_info "Checking ROS2 environment..."
    
    if [ -z "$ROS_DISTRO" ]; then
        log_error "ROS2 environment not sourced!"
        log_info "Please source ROS2 setup: source /opt/ros/humble/setup.bash"
        return 1
    fi
    
    log_success "ROS2 environment ready (Distribution: $ROS_DISTRO)"
    
    # Source workspace if it exists
    if [ -f "$HOME/dobot_v3/install/setup.bash" ]; then
        log_info "Sourcing workspace..."
        source "$HOME/dobot_v3/install/setup.bash"
        log_success "Workspace sourced successfully"
    else
        log_warning "Workspace setup.bash not found at $HOME/dobot_v3/install/"
    fi
    
    return 0
}

# Function to start robot drivers
start_robot_drivers() {
    log_info "Starting Dobot CR5 drivers..."
    
    # Check if drivers are already running
    if pgrep -f "dobot_bringup_ros2.launch.py" &>/dev/null; then
        log_warning "Robot drivers appear to be already running"
        return 0
    fi
    
    # Start drivers in background
    log_info "Launching robot drivers..."
    ros2 launch dobot_bringup_v3 dobot_bringup_ros2.launch.py &
    DRIVER_PID=$!
    
    # Wait for drivers to initialize
    log_info "Waiting for drivers to initialize (30 seconds)..."
    sleep 30
    
    # Check if process is still running
    if kill -0 $DRIVER_PID 2>/dev/null; then
        log_success "Robot drivers started successfully (PID: $DRIVER_PID)"
        echo $DRIVER_PID > /tmp/dobot_driver.pid
        return 0
    else
        log_error "Robot drivers failed to start or crashed"
        return 1
    fi
}

# Function to start MoveIt and full system
start_moveit_system() {
    log_info "Starting MoveIt and full robot system..."
    
    # Check if MoveIt is already running
    if pgrep -f "full_bringup.launch.py" &>/dev/null; then
        log_warning "MoveIt system appears to be already running"
        return 0
    fi
    
    # Start MoveIt system in background
    log_info "Launching MoveIt planning system..."
    ros2 launch cr5_moveit full_bringup.launch.py &
    MOVEIT_PID=$!
    
    # Wait for MoveIt to initialize
    log_info "Waiting for MoveIt to initialize (45 seconds)..."
    sleep 45
    
    # Check if process is still running
    if kill -0 $MOVEIT_PID 2>/dev/null; then
        log_success "MoveIt system started successfully (PID: $MOVEIT_PID)"
        echo $MOVEIT_PID > /tmp/dobot_moveit.pid
        return 0
    else
        log_error "MoveIt system failed to start or crashed"
        return 1
    fi
}

# Function to verify system readiness
verify_system_readiness() {
    log_info "Verifying system readiness..."
    
    # Check for required topics
    log_info "Checking for robot topics..."
    timeout 10 ros2 topic list | grep -q "/joint_states" || {
        log_error "Robot joint_states topic not found!"
        return 1
    }
    
    log_info "Checking for MoveIt planning service..."
    timeout 10 ros2 service list | grep -q "plan_kinematic_path" || {
        log_error "MoveIt planning service not found!"
        return 1
    }
    
    log_success "System verification completed successfully"
    return 0
}

# Function to start the movement node
start_movement_node() {
    log_info "Starting CR5 movement node..."
    
    log_info "You can now run your movement script with:"
    echo "ros2 run cr5_moveit_cpp_demo move_cr5_node_modular"
    echo ""
    log_info "Or the original monolithic version:"
    echo "ros2 run cr5_moveit_cpp_demo move_cr5_node"
    
    # Ask user if they want to start the movement node now
    read -p "Start movement node now? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        log_info "Starting modular movement node..."
        ros2 run cr5_moveit_cpp_demo move_cr5_node_modular
    fi
}

# Function to cleanup on exit
cleanup() {
    log_info "Cleaning up..."
    
    # Kill background processes if they exist
    if [ -f /tmp/dobot_driver.pid ]; then
        DRIVER_PID=$(cat /tmp/dobot_driver.pid)
        if kill -0 $DRIVER_PID 2>/dev/null; then
            log_info "Stopping robot drivers..."
            kill $DRIVER_PID
        fi
        rm -f /tmp/dobot_driver.pid
    fi
    
    if [ -f /tmp/dobot_moveit.pid ]; then
        MOVEIT_PID=$(cat /tmp/dobot_moveit.pid)
        if kill -0 $MOVEIT_PID 2>/dev/null; then
            log_info "Stopping MoveIt system..."
            kill $MOVEIT_PID
        fi
        rm -f /tmp/dobot_moveit.pid
    fi
}

# Set up cleanup trap
trap cleanup EXIT

# Main execution flow
main() {
    log_info "========================================"
    log_info "    Dobot CR5 Automated Startup"
    log_info "========================================"
    echo
    
    # Step 1: Detect and configure network
    detect_network_interface || exit 1
    configure_network || exit 1
    echo
    
    # Step 2: Test robot connectivity
    test_robot_connectivity || exit 1
    echo
    
    # Step 3: Check ROS2 environment
    check_ros2_environment || exit 1
    echo
    
    # Step 4: Start robot drivers
    start_robot_drivers || exit 1
    echo
    
    # Step 5: Start MoveIt system
    start_moveit_system || exit 1
    echo
    
    # Step 6: Verify system readiness
    verify_system_readiness || exit 1
    echo
    
    # Step 7: Ready to run movement node
    log_success "========================================"
    log_success "    DOBOT CR5 SYSTEM READY!"
    log_success "========================================"
    echo
    
    start_movement_node
}

# Run main function
main "$@"