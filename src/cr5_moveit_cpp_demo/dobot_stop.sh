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
    pkill -f "move_cr5_node"
    log_success "Movement nodes stopped"
else
    log_info "No movement nodes running"
fi

# Stop MoveIt processes
log_info "Stopping MoveIt system..."
if pgrep -f "full_bringup.launch.py" &>/dev/null; then
    pkill -f "full_bringup.launch.py"
    sleep 3
    pkill -f "move_group" 2>/dev/null || true
    log_success "MoveIt system stopped"
else
    log_info "MoveIt system not running"
fi

# Stop driver processes
log_info "Stopping robot drivers..."
if pgrep -f "dobot_bringup_ros2.launch.py" &>/dev/null; then
    pkill -f "dobot_bringup_ros2.launch.py"
    sleep 3
    pkill -f "dobot_driver" 2>/dev/null || true
    log_success "Robot drivers stopped"
else
    log_info "Robot drivers not running"
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
remaining_processes=$(pgrep -f "dobot\|moveit\|move_cr5" | wc -l)
if [ "$remaining_processes" -eq 0 ]; then
    log_success "All Dobot processes stopped successfully"
else
    log_warning "$remaining_processes Dobot-related processes still running"
    log_info "You may need to kill them manually with:"
    echo "  pkill -f dobot"
    echo "  pkill -f moveit"
fi

log_success "========================================"
log_success "    Cleanup completed!"
log_success "========================================"