# Setup Guide - Humanoid Robotics Capstone Project

This guide will help you set up the development environment for the ROS 2 humanoid robotics project.

## Table of Contents

1. [Windows Users (WSL2)](#windows-users-wsl2)
2. [Linux Users (Native Ubuntu)](#linux-users-native-ubuntu)
3. [Verify Installation](#verify-installation)
4. [Configure API Keys](#configure-api-keys)
5. [Troubleshooting](#troubleshooting)

---

## Windows Users (WSL2)

### Prerequisites

- Windows 10 version 2004+ or Windows 11
- Administrator access

### Quick Setup (Automated)

**Step 1: Install WSL2 with Ubuntu 22.04**

Open PowerShell as Administrator and run:

```powershell
wsl --install -d Ubuntu-22.04
```

Restart your computer when prompted.

**Step 2: Launch Ubuntu and Run Setup Script**

1. Open "Ubuntu 22.04" from Start menu
2. Create a username and password when prompted
3. Navigate to the project directory:

```bash
cd /mnt/c/Users/"Since Tech"/my-book
```

4. Run the automated setup script:

```bash
bash setup_wsl2_ros2.sh
```

This will take 15-30 minutes. The script will:
- ✓ Update Ubuntu system
- ✓ Install ROS 2 Humble Desktop
- ✓ Install build tools (colcon, rosdep)
- ✓ Install project dependencies
- ✓ Build the ROS 2 workspace
- ✓ Configure environment variables

**Step 3: Configure API Keys**

After setup completes, edit the `.env` file:

```bash
nano .env
```

Add your API keys:
```env
OPENAI_API_KEY=sk-...  # Get from https://platform.openai.com/api-keys
GROQ_API_KEY=gsk_...   # Get from https://console.groq.com/keys
```

Save with `Ctrl+O`, `Enter`, then exit with `Ctrl+X`.

**Step 4: Reload Environment**

```bash
source ~/.bashrc
```

Done! Skip to [Verify Installation](#verify-installation).

---

## Linux Users (Native Ubuntu)

### Prerequisites

- Ubuntu 22.04 LTS (required for ROS 2 Humble)
- Sudo access

### Quick Setup

**Step 1: Clone/Navigate to Project**

```bash
cd /path/to/my-book
```

**Step 2: Run Setup Script**

```bash
bash setup_wsl2_ros2.sh
```

The script works on both WSL2 and native Ubuntu.

**Step 3: Configure API Keys**

```bash
nano .env
```

Add your keys, save, and exit.

**Step 4: Reload Environment**

```bash
source ~/.bashrc
```

---

## Verify Installation

Test that everything is working:

```bash
# Check ROS 2 installation
ros2 doctor

# List project packages
ros2 pkg list | grep -E "vla_planning|perception|navigation|manipulation|hw_interface|custom_interfaces"
```

Expected output (6 packages):
```
custom_interfaces
hw_interface
manipulation
navigation
perception
vla_planning
```

**Test building:**
```bash
cd ros2_ws
colcon build --symlink-install
```

Should complete without errors (may show warnings about stub implementations).

**Run a stub node:**
```bash
source install/setup.bash
ros2 run vla_planning audio_capture_node
```

Should output: `Audio Capture Node initialized (STUB)`

---

## Configure API Keys

### OpenAI API Key (Whisper ASR + TTS)

1. Go to https://platform.openai.com/api-keys
2. Create a new API key
3. Add to `.env`:
   ```
   OPENAI_API_KEY=sk-proj-...
   ```

### Groq API Key (Llama 3 8B LLM)

1. Go to https://console.groq.com/keys
2. Create a new API key
3. Add to `.env`:
   ```
   GROQ_API_KEY=gsk_...
   ```

### Load Environment Variables

After editing `.env`:
```bash
source ~/.bashrc
```

Or manually:
```bash
export $(cat .env | grep -v '^#' | xargs)
```

---

## Troubleshooting

### Issue: "ros2: command not found"

**Solution:**
```bash
source /opt/ros/humble/setup.bash
source ~/my-book/ros2_ws/install/setup.bash
```

Or reload bashrc:
```bash
source ~/.bashrc
```

### Issue: "Package 'X' not found"

**Solution:** Rebuild workspace:
```bash
cd ~/my-book/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Issue: "Permission denied" when running script

**Solution:** Make script executable:
```bash
chmod +x setup_wsl2_ros2.sh
```

### Issue: GUI apps (Gazebo) don't show on WSL2

**Solutions:**

1. **Windows 11 or Windows 10 with WSLg:** GUI should work automatically.

2. **Windows 10 without WSLg:** Install VcXsrv:
   - Download from https://sourceforge.net/projects/vcxsrv/
   - Launch XLaunch, select "Multiple windows", "Start no client", check "Disable access control"
   - Add to `~/.bashrc`:
     ```bash
     export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
     ```

### Issue: Build fails with "Could not find a package configuration file"

**Solution:** Install missing dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Issue: Python module not found (openai, groq, etc.)

**Solution:** Reinstall Python dependencies:
```bash
pip3 install openai groq pyaudio sounddevice python-dotenv
```

---

## Useful Commands

After setup, these aliases are available:

```bash
ros2_ws        # Navigate to workspace
ros2_build     # Build workspace
ros2_clean     # Clean build artifacts
```

**Manual commands:**

```bash
# Navigate to workspace
cd ~/my-book/ros2_ws

# Build all packages
colcon build --symlink-install

# Build specific package
colcon build --packages-select vla_planning

# Clean build
rm -rf build install log

# List all nodes in a package
ros2 pkg executables vla_planning

# Run a node
ros2 run vla_planning audio_capture_node

# Launch a system
ros2 launch vla_planning vla_planning.launch.py
```

---

## Next Steps

1. **Implement actual functionality** - Currently all nodes are stubs with TODO comments
2. **Start with Phase 0 tasks** - Environment setup, Gazebo models
3. **Follow spec.md** - Read `specs/001-humanoid-robotics-capstone/spec.md` for requirements
4. **Check tasks.md** - Follow task breakdown if available

For detailed architecture, see:
- `README.md` - Project overview
- `specs/001-humanoid-robotics-capstone/plan.md` - Architecture decisions

---

## Hardware Setup (Physical Robot)

For deployment on physical robot:

**Jetson Orin NX setup:**
1. Flash JetPack 5.1.1 (includes Ubuntu 22.04)
2. Install ROS 2 Humble using same script
3. Run: `ros2 launch full_system_jetson.launch.py`

**Workstation setup:**
1. Ensure GPU drivers installed (NVIDIA RTX 4070 Ti)
2. Run: `ros2 launch full_system_workstation.launch.py`
3. Set matching `ROS_DOMAIN_ID` on both machines

---

## Support

For issues:
1. Check this troubleshooting section
2. Review ROS 2 Humble documentation: https://docs.ros.org/en/humble/
3. Check project README.md

Happy building! 🤖
