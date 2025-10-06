#!/bin/bash

# Dobot CR5 Cleanup Script
# Stops all Dobot-related processes safely

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

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

log_info "========================================"
log_info "    Dobot CR5 System Cleanup"
log_info "========================================"

# Stop movement nodes
log_info "Stopping movement nodes..."
if pgrep -f "move_cr5_node" &>/dev/null; then
    pkill -TERM -f "move_cr5_node"
    sleep 2
    # Force kill if still running
    pkill -KILL -f "move_cr5_node" 2>/dev/null || true
    log_success "Movement nodes stopped"
else
    log_info "No movement nodes running"
fi

# Stop RViz specifically
log_info "Stopping RViz..."
if pgrep -f "rviz2" &>/dev/null; then
    pkill -TERM -f "rviz2"
    sleep 2
    pkill -KILL -f "rviz2" 2>/dev/null || true
    log_success "RViz stopped"
else
    log_info "RViz not running"
fi

# Stop MoveIt processes (including move_group and reworked_map_node)
log_info "Stopping MoveIt system..."
if pgrep -f "full_bringup.launch.py\|move_group\|reworked_map_node" &>/dev/null; then
    pkill -TERM -f "full_bringup.launch.py"
    pkill -TERM -f "move_group"
    pkill -TERM -f "reworked_map_node"
    sleep 3
    # Force kill if still running
    pkill -KILL -f "move_group" 2>/dev/null || true
    pkill -KILL -f "reworked_map_node" 2>/dev/null || true
    pkill -KILL -f "full_bringup.launch.py" 2>/dev/null || true
    log_success "MoveIt system stopped"
else
    log_info "MoveIt system not running"
fi

# Stop driver processes (including dobot_bringup and translater_node)
log_info "Stopping robot drivers..."
if pgrep -f "dobot_bringup_ros2.launch.py\|dobot_bringup\|translater_node" &>/dev/null; then
    pkill -TERM -f "dobot_bringup_ros2.launch.py"
    pkill -TERM -f "dobot_bringup"
    pkill -TERM -f "translater_node"
    sleep 3
    # Force kill if still running
    pkill -KILL -f "dobot_bringup" 2>/dev/null || true
    pkill -KILL -f "translater_node" 2>/dev/null || true
    pkill -KILL -f "dobot_bringup_ros2.launch.py" 2>/dev/null || true
    log_success "Robot drivers stopped"
else
    log_info "Robot drivers not running"
fi

# Additional cleanup for any remaining launcher processes
log_info "Stopping any remaining launcher processes..."
if pgrep -f "launch\|launcher" &>/dev/null; then
    pkill -TERM -f "launch"
    sleep 2
    pkill -KILL -f "launch" 2>/dev/null || true
    log_success "Launcher processes stopped"
fi

# Clean up PID files
log_info "Cleaning up temporary files..."
rm -f /tmp/dobot_driver.pid
rm -f /tmp/dobot_moveit.pid
log_success "Temporary files cleaned"

# Wait a moment for processes to fully terminate
sleep 2

# Verify cleanup
log_info "Verifying cleanup..."
remaining_processes=$(pgrep -f "dobot\|moveit\|move_cr5\|rviz2\|reworked_map\|translater" | wc -l)
if [ "$remaining_processes" -eq 0 ]; then
    log_success "All Dobot processes stopped successfully"
else
    log_warning "$remaining_processes Dobot-related processes still running"
    log_info "Attempting final aggressive cleanup..."
    
    # Get the specific PIDs and kill them
    for pid in $(pgrep -f "dobot_bringup\|move_group\|translater_node\|reworked_map"); do
        log_info "Killing process $pid"
        kill -KILL $pid 2>/dev/null || true
    done
    
    sleep 2
    final_check=$(pgrep -f "dobot\|moveit\|move_cr5\|rviz2\|reworked_map\|translater" | wc -l)
    if [ "$final_check" -eq 0 ]; then
        log_success "Final cleanup successful"
    else
        log_error "Some processes may require manual termination"
        log_info "Remaining processes:"
        ps aux | grep -E "(dobot|moveit|rviz|move_cr5|reworked_map|translater)" | grep -v grep | awk '{print "  PID:"$2" "$11" "$12" "$13}'
        log_info "You can kill them manually with: kill -KILL <PID>"
    fi
fi

log_success "========================================"
log_success "    Cleanup completed!"
log_success "========================================"