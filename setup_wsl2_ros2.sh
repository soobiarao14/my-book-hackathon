#!/bin/bash
#
# WSL2 + ROS 2 Humble Setup Script
# Automated installation for Humanoid Robotics Capstone Project
#
# Usage: bash setup_wsl2_ros2.sh
#
# This script will:
# 1. Update Ubuntu 22.04 system
# 2. Install ROS 2 Humble Desktop
# 3. Install build tools (colcon, rosdep)
# 4. Install project dependencies
# 5. Build the ROS 2 workspace
# 6. Configure environment variables
#

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Progress indicator
print_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running on Ubuntu 22.04
print_step "Checking Ubuntu version..."
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$VERSION_ID" != "22.04" ]; then
        print_error "This script requires Ubuntu 22.04. Detected: $VERSION_ID"
        exit 1
    fi
    print_success "Ubuntu 22.04 detected"
else
    print_error "Cannot detect OS version"
    exit 1
fi

# Update system
print_step "Updating system packages..."
sudo apt update && sudo apt upgrade -y
print_success "System updated"

# Set locale
print_step "Configuring locale..."
sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
print_success "Locale configured"

# Check if ROS 2 is already installed
if [ -f /opt/ros/humble/setup.bash ]; then
    print_warning "ROS 2 Humble already installed, skipping installation"
else
    print_step "Installing ROS 2 Humble..."

    # Add ROS 2 apt repository
    sudo apt install software-properties-common curl -y
    sudo add-apt-repository universe -y

    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    # Install ROS 2 Humble Desktop
    sudo apt update
    sudo apt install ros-humble-desktop -y

    print_success "ROS 2 Humble installed"
fi

# Install build tools
print_step "Installing ROS 2 build tools..."
sudo apt install python3-colcon-common-extensions python3-rosdep python3-pip -y

# Initialize rosdep if not already done
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    print_step "Initializing rosdep..."
    sudo rosdep init
fi
rosdep update
print_success "Build tools installed"

# Install ROS 2 packages for the project
print_step "Installing ROS 2 packages..."
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-moveit \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-vision-msgs \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    gazebo \
    python3-opencv \
    python3-numpy
print_success "ROS 2 packages installed"

# Install Python dependencies
print_step "Installing Python dependencies..."
pip3 install --upgrade pip
pip3 install \
    openai \
    groq \
    pyaudio \
    sounddevice \
    numpy \
    opencv-python \
    pyyaml \
    python-dotenv
print_success "Python dependencies installed"

# Navigate to workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR/ros2_ws"

if [ ! -d "$WORKSPACE_DIR" ]; then
    print_error "Workspace directory not found: $WORKSPACE_DIR"
    exit 1
fi

cd "$WORKSPACE_DIR"
print_step "Working in: $WORKSPACE_DIR"

# Source ROS 2
source /opt/ros/humble/setup.bash

# Install workspace dependencies
print_step "Installing workspace dependencies..."
rosdep install --from-paths src --ignore-src -r -y || print_warning "Some dependencies may be missing (this is OK for stubs)"

# Build workspace
print_step "Building ROS 2 workspace..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
print_success "Workspace built successfully"

# Source workspace
source install/setup.bash

# Create .env file if it doesn't exist
ENV_FILE="$SCRIPT_DIR/.env"
if [ ! -f "$ENV_FILE" ]; then
    print_step "Creating .env file..."
    cat > "$ENV_FILE" << 'EOF'
# OpenAI API Key (for Whisper ASR and TTS)
# Get your key from: https://platform.openai.com/api-keys
OPENAI_API_KEY=your_openai_key_here

# Groq API Key (for Llama 3 8B LLM)
# Get your key from: https://console.groq.com/keys
GROQ_API_KEY=your_groq_key_here

# ROS Domain ID (for multi-machine setup)
ROS_DOMAIN_ID=42
EOF
    print_warning "Created .env file. Please edit it with your actual API keys:"
    print_warning "  nano $ENV_FILE"
else
    print_success ".env file already exists"
fi

# Add to ~/.bashrc
print_step "Configuring ~/.bashrc..."
BASHRC_ADDITIONS="
# ROS 2 Humble setup
source /opt/ros/humble/setup.bash

# Project workspace setup
if [ -f $WORKSPACE_DIR/install/setup.bash ]; then
    source $WORKSPACE_DIR/install/setup.bash
fi

# Load environment variables
if [ -f $ENV_FILE ]; then
    export \$(cat $ENV_FILE | grep -v '^#' | xargs)
fi

# Useful aliases
alias ros2_build='cd $WORKSPACE_DIR && colcon build --symlink-install'
alias ros2_clean='cd $WORKSPACE_DIR && rm -rf build install log'
alias ros2_ws='cd $WORKSPACE_DIR'
"

# Check if already added
if ! grep -q "# ROS 2 Humble setup" ~/.bashrc; then
    echo "$BASHRC_ADDITIONS" >> ~/.bashrc
    print_success "Added ROS 2 configuration to ~/.bashrc"
else
    print_warning "ROS 2 configuration already in ~/.bashrc"
fi

# Test installation
print_step "Testing ROS 2 installation..."
source /opt/ros/humble/setup.bash
ros2 doctor || print_warning "ros2 doctor found some issues (this may be OK)"

# List installed packages
print_step "Verifying project packages..."
PACKAGES=$(ros2 pkg list | grep -E "vla_planning|perception|navigation|manipulation|hw_interface|custom_interfaces" || true)
if [ -z "$PACKAGES" ]; then
    print_error "Project packages not found. Build may have failed."
else
    print_success "Found project packages:"
    echo "$PACKAGES"
fi

# Final summary
echo ""
echo -e "${GREEN}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║         ROS 2 Humble Setup Complete!                      ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${BLUE}Next steps:${NC}"
echo "1. Edit API keys in .env file:"
echo "   ${YELLOW}nano $ENV_FILE${NC}"
echo ""
echo "2. Reload your shell or run:"
echo "   ${YELLOW}source ~/.bashrc${NC}"
echo ""
echo "3. Test the installation:"
echo "   ${YELLOW}ros2 pkg list | grep custom_interfaces${NC}"
echo ""
echo "4. Start implementing nodes (currently all are stubs):"
echo "   ${YELLOW}cd $WORKSPACE_DIR/src/vla_planning/vla_planning${NC}"
echo ""
echo "5. Rebuild after changes:"
echo "   ${YELLOW}ros2_build${NC}"
echo ""
echo -e "${BLUE}Useful commands:${NC}"
echo "  ${YELLOW}ros2_ws${NC}        - Navigate to workspace"
echo "  ${YELLOW}ros2_build${NC}     - Build workspace"
echo "  ${YELLOW}ros2_clean${NC}     - Clean build artifacts"
echo ""
echo -e "${GREEN}Happy coding! 🤖${NC}"
