# Appendix A: Hardware Guide

**Complete Hardware Specifications for Physical AI & Humanoid Robotics**

---

## Overview

This book is designed for **flexibility**. You can follow along using:
1. **Simulation Only** (minimum hardware)
2. **High-Performance Workstation** (recommended)
3. **Edge AI Kit** (for physical deployment)
4. **Full Robot Platform** (advanced, optional)

This appendix provides detailed specifications, purchasing guides, and setup instructions for each option.

---

## Option 1: Simulation Only (Cloud or Modest Laptop)

**Best for**: Students without access to high-end hardware, online learners

### Minimum Local Requirements

- **Laptop/Desktop**: Any modern computer
- **RAM**: 16GB (32GB recommended)
- **OS**: Ubuntu 22.04 LTS (WSL2 on Windows works with limitations)
- **Storage**: 50GB free space
- **Internet**: Stable connection for cloud instances

### Cloud Instances (AWS/Azure/GCP)

**Recommended**: AWS g5.2xlarge (A10G GPU, 24GB VRAM)

#### AWS Configuration

```bash
# Instance Type: g5.2xlarge
# GPU: NVIDIA A10G (24GB VRAM)
# CPU: 8 vCPUs (AMD EPYC 7R13)
# RAM: 32GB
# Storage: 200GB EBS (gp3)
# OS: Ubuntu 22.04 LTS (Deep Learning AMI)
```

**Cost Breakdown**:
- **On-Demand**: ~$1.21/hour
- **Spot Instance**: ~$0.40-0.60/hour (66% savings, subject to availability)
- **Reserved (1-year)**: ~$0.75/hour

**Quarterly Cost** (10 hours/week × 13 weeks):
- On-Demand: 130 hours × $1.21 = **$157**
- Spot: 130 hours × $0.50 = **$65**
- Plus storage: ~$20
- **Total**: **$85-180/quarter**

#### Setup Script

```bash
# Launch g5.2xlarge with Deep Learning AMI
# SSH into instance

# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS 2 Humble
source /opt/ros/humble/setup.bash  # Already installed in DL AMI

# Install Gazebo
sudo apt install -y ros-humble-gazebo-ros-pkgs

# Install Isaac Sim (requires NVIDIA account)
# Follow: https://developer.nvidia.com/isaac-sim

# Clone course repository
git clone https://github.com/sincetech/physical-ai-robotics-book
cd physical-ai-robotics-book
colcon build
```

**Limitations**:
- Cannot deploy to physical robots (no local Jetson)
- Network latency for remote control
- Ongoing costs (vs one-time hardware purchase)

---

## Option 2: High-Performance Workstation (Recommended)

**Best for**: Serious students, researchers, professionals

This is the **primary recommended setup** for this book. It provides the best balance of performance, cost, and versatility.

### Complete Workstation Build

#### Component List

| Component | Specification | Price (USD) | Notes |
|-----------|--------------|-------------|-------|
| **GPU** | NVIDIA RTX 4070 Ti (12GB VRAM) | $800 | Minimum for Isaac Sim |
| **Alternative** | NVIDIA RTX 4080 (16GB VRAM) | $1,100 | Better for large models |
| **Premium** | NVIDIA RTX 4090 (24GB VRAM) | $1,600 | Ideal for research |
| **CPU** | Intel Core i7-13700K (16 cores) | $400 | Physics simulation |
| **Alternative** | AMD Ryzen 9 7900X (12 cores) | $450 | Comparable performance |
| **RAM** | 64GB DDR5-5600 (2×32GB) | $220 | Critical for Isaac Sim |
| **Minimum** | 32GB DDR5 (2×16GB) | $110 | Works but limits scenarios |
| **Storage (NVMe)** | 1TB Gen4 NVMe SSD | $120 | OS + ROS workspace |
| **Storage (SATA)** | 2TB SATA SSD (Optional) | $100 | Datasets, recordings |
| **Motherboard** | Z790 or X670 (PCIe 4.0+) | $250 | Must support RTX 40-series |
| **Power Supply** | 850W 80+ Gold Modular | $120 | RTX 4090 needs 850W+ |
| **Case** | Mid-Tower with good airflow | $100 | NZXT H510, Fractal Meshify |
| **Cooling** | AIO Liquid Cooler (240mm) | $100 | CPU runs hot during sims |
| **Monitor** | 27" 1440p IPS (Optional) | $300 | Better for RViz/Gazebo |
| | | | |
| **TOTAL (Recommended)** | RTX 4070 Ti Build | **~$2,200** | Balanced performance |
| **TOTAL (Premium)** | RTX 4090 Build | **~$3,000** | Research-grade |

#### Detailed Specifications

**GPU: NVIDIA RTX 4070 Ti (12GB)**
- **CUDA Cores**: 7,680
- **Tensor Cores**: 240 (Gen 4)
- **RT Cores**: 60 (Gen 3)
- **Memory**: 12GB GDDR6X
- **Memory Bandwidth**: 504 GB/s
- **TDP**: 285W
- **Why**: Minimum for NVIDIA Isaac Sim. Can run Whisper, LLMs, and vision models simultaneously.

**Upgrade Path**: RTX 4080/4090
- **RTX 4080** (16GB): $1,100 - Better for large LLMs (Llama 70B)
- **RTX 4090** (24GB): $1,600 - Can run Isaac Sim + Llama 3 70B concurrently

**CPU: Intel Core i7-13700K**
- **Cores**: 16 (8P + 8E)
- **Threads**: 24
- **Base Clock**: 3.4 GHz
- **Boost Clock**: 5.4 GHz
- **Cache**: 30MB L3
- **TDP**: 125W (253W max)
- **Why**: Physics simulation (Gazebo, Isaac) is CPU-intensive. High single-thread performance critical for Nav2.

**RAM: 64GB DDR5-5600**
- **Why 64GB?**:
  - Isaac Sim: 10-15GB (loading USD scenes)
  - Gazebo: 4-8GB (complex worlds with multiple robots)
  - ROS 2 nodes: 2-4GB
  - LLM (Llama 3 8B): 8-10GB
  - OS + background: 4GB
  - **Total**: 28-41GB (64GB provides headroom)
- **32GB works** for single-robot scenarios, but may crash with complex Isaac scenes

**Storage: 1TB NVMe SSD**
- **Gen 4 NVMe** (7,000 MB/s read): Fast loading for Isaac Sim assets
- **Usage**:
  - Ubuntu 22.04: 25GB
  - ROS 2 + packages: 15GB
  - Isaac Sim: 100GB
  - Gazebo models: 10GB
  - ROS bags (recordings): 50GB
  - Code + datasets: 100GB
  - **Total**: ~300GB (1TB provides room for growth)

#### Verified Build Example

**"The Budget Beast" ($2,200)**

```
GPU: NVIDIA RTX 4070 Ti 12GB (MSI Gaming X Trio)       $800
CPU: Intel Core i7-13700K                               $400
RAM: G.Skill Trident Z5 64GB DDR5-5600                  $220
SSD: Samsung 990 PRO 1TB NVMe                           $120
Motherboard: MSI Z790 Gaming Plus WiFi                  $250
PSU: Corsair RM850x 850W 80+ Gold                       $120
Case: NZXT H510 Flow                                    $100
Cooling: NZXT Kraken X53 240mm AIO                      $100
Fans: 3× Arctic P14 PWM (already included in case)       $0
Thermal Paste: Arctic MX-5 (included with cooler)        $0
                                                     --------
                                                 TOTAL: $2,210
```

**Performance**:
- Isaac Sim: 60-90 FPS (Carter warehouse scene)
- Gazebo: 100+ FPS (single humanoid, moderate complexity)
- Whisper (base): `<50ms latency>`

- Llama 3 8B: `<100ms` inference
- Nav2 + VSLAM: Real-time (30Hz)

---

### OS Installation

**Ubuntu 22.04 LTS (Recommended)**

#### Why Ubuntu 22.04?

- **ROS 2 Humble**: Officially supported (LTS until 2027)
- **Isaac ROS**: Requires Ubuntu 22.04
- **NVIDIA Drivers**: Best support on Ubuntu
- **Community**: Largest robotics community uses Ubuntu

#### Installation Steps

1. **Download Ubuntu 22.04 LTS**:
   ```
   https://ubuntu.com/download/desktop
   ```

2. **Create Bootable USB** (Windows):
   - Download Rufus: https://rufus.ie/
   - Insert USB (8GB+)
   - Select Ubuntu ISO, write to USB

3. **Install Ubuntu**:
   - Boot from USB (press F12/F2 during startup)
   - Select "Install Ubuntu"
   - Choose "Minimal Installation" (faster, cleaner)
   - Enable "Install third-party software" (NVIDIA drivers)
   - Partition: Use entire disk (or dual-boot if desired)
   - Username: `robotics`, Password: (your choice)

4. **Post-Installation Setup**:
   ```bash
   # Update system
   sudo apt update && sudo apt upgrade -y

   # Install essential tools
   sudo apt install -y build-essential git curl wget vim

   # Install NVIDIA drivers (if not auto-installed)
   sudo ubuntu-drivers autoinstall
   sudo reboot

   # Verify GPU
   nvidia-smi
   # Should show RTX 4070 Ti with driver version 535+
   ```

---

## Option 3: Edge AI Kit (Physical Deployment)

**Best for**: Students wanting to deploy to physical hardware without buying a full robot

This kit provides the "brain" (Jetson), "eyes" (camera), and "ears" (microphone) needed for Physical AI, without the expensive mechanical body.

### The "Economy Jetson Student Kit"

**Total Cost**: ~$700

| Component | Model | Price | Purpose |
|-----------|-------|-------|---------|
| **Brain** | NVIDIA Jetson Orin Nano Super Dev Kit (8GB) | $249 | AI inference, ROS 2 nodes |
| **Eyes** | Intel RealSense D435i (RGB-D + IMU) | $349 | Vision, depth, SLAM |
| **Ears** | ReSpeaker USB Mic Array v2.0 | $69 | Voice commands, far-field ASR |
| **Storage** | Samsung EVO Select 128GB microSD (A2) | $18 | OS + ROS workspace |
| **Power** | Official Jetson Power Supply (19V/4.74A) | $25 | Included with Dev Kit |
| **Networking** | Wi-Fi 6E + Bluetooth 5.2 | $0 | Included in "Super" kit |
| **Cables** | USB-C to USB-A (for RealSense), HDMI | $10 | Connectivity |
| **Cooling** | Noctua NF-A4x10 FLX Fan (Optional) | $15 | Better thermals |
| | | | |
| **TOTAL** | | **$697** | Complete edge AI system |

---

### Component Details

#### 1. NVIDIA Jetson Orin Nano Super Dev Kit ($249)

**Specifications**:
- **GPU**: 1024-core NVIDIA Ampere (40 TOPS AI performance)
- **CPU**: 6-core Arm Cortex-A78AE @ 1.5 GHz
- **RAM**: 8GB LPDDR5 (128-bit, 102 GB/s bandwidth)
- **Storage**: microSD slot (supports UHS-I)
- **Power**: 7W / 15W power modes
- **Connectivity**:
  - Wi-Fi 6E (802.11ax)
  - Bluetooth 5.2
  - Gigabit Ethernet
  - 4× USB 3.2 (Type-A)
  - 1× USB-C (device mode)
  - DisplayPort
  - MIPI CSI camera connectors (2×)

**Why Jetson Orin Nano?**:
- **Isaac ROS Native**: Runs Isaac ROS GEMs (VSLAM, DNN inference) with GPU acceleration
- **ROS 2 Compatible**: Full ROS 2 Humble support
- **Power Efficient**: 7-15W (vs 350W for RTX 4070 Ti)
- **Compact**: Fits on robot chassis
- **Price Drop**: $249 (was $499) as of November 2024

**Performance**:
- **Whisper (base, FP16)**: ~500ms for 3s audio
- **YOLO v8 (medium)**: ~30 FPS @ 640x640
- **Isaac Visual SLAM**: Real-time @ 30Hz
- **Llama 3 8B (INT4 quantized)**: ~1-2 tokens/sec (slow, prefer cloud)

---

#### 2. Intel RealSense D435i ($349)

**Specifications**:
- **RGB Camera**: 1920×1080 @ 30 FPS
- **Depth Resolution**: 1280×720 @ 90 FPS
- **Depth Technology**: Stereo vision (active IR)
- **Range**: 0.3m to 10m (optimal: 0.5-3m)
- **Accuracy**: `<2%` error at 2m
- **Field of View**: 87° × 58° (depth), 69° × 42° (RGB)
- **IMU**: BMI055 (6-DOF: accel + gyro)
- **Interface**: USB 3.1 Gen 1 (Type-C)
- **Dimensions**: 90mm × 25mm × 25mm
- **Weight**: 72g

**Why D435i over D435?**:
- **IMU Included**: Essential for VSLAM (Isaac Visual SLAM, RTAB-Map)
- **Better SLAM**: IMU + vision = more robust localization
- **Same Price**: Only $50 more than D435 (worth it)

**ROS 2 Integration**:
```bash
sudo apt install ros-humble-realsense2-camera
ros2 launch realsense2_camera rs_launch.py enable_depth:=true enable_infra:=true
```

**Use Cases in This Book**:
- **Module 2**: RGB-D data for Gazebo sensor simulation
- **Module 3**: Visual SLAM (Chapter 9)
- **Module 4**: Object detection for VLA pipeline

---

#### 3. ReSpeaker USB Mic Array v2.0 ($69)

**Specifications**:
- **Microphones**: 4× omnidirectional
- **Array Diameter**: 70mm (circular)
- **Frequency Response**: 100Hz - 10kHz
- **SNR**: 63dB
- **Max SPL**: 120dB
- **Interface**: USB 2.0 (UAC 1.0, plug-and-play)
- **LEDs**: 12× RGB (indicates direction of sound source)
- **Beamforming**: Yes (4-mic spatial filtering)
- **AEC**: Acoustic Echo Cancellation (optional)

**Why ReSpeaker over Basic USB Mic?**:
- **Far-Field**: Works up to 3 meters (vs 1m for laptop mics)
- **Noise Reduction**: 4-mic beamforming filters background noise
- **Direction of Arrival (DOA)**: Indicates who is speaking
- **Plug-and-Play**: No driver installation needed

**ROS 2 Integration**:
```bash
# Install audio tools
sudo apt install -y libasound2-dev portaudio19-dev

# Test microphone
arecord -D hw:2,0 -f S16_LE -r 16000 -c 1 test.wav
# Speak for 5 seconds, then Ctrl+C

# Play back
aplay test.wav
```

**Use Cases**:
- **Chapter 11**: Voice Activity Detection (VAD)
- **Chapter 11**: Whisper ASR for voice commands
- **Chapter 13**: Capstone project voice interface

---

#### 4. Storage: Samsung EVO Select 128GB microSD ($18)

**Why microSD?**:
- Jetson Orin Nano uses microSD for OS storage
- 128GB is minimum; 256GB recommended if recording ROS bags

**Requirements**:
- **Speed Class**: A2 (Application Performance Class 2)
- **Read Speed**: 100 MB/s+
- **Write Speed**: 60 MB/s+

**Setup**:
```bash
# Flash JetPack 5.1.2 to microSD
# Download from: https://developer.nvidia.com/embedded/jetpack

# Use balenaEtcher (cross-platform)
# Write JetPack image to microSD
# Insert into Jetson, power on
```

---

### Edge AI Kit Assembly

**Step 1: Flash Jetson**
1. Download JetPack 5.1.2 image
2. Flash to 128GB microSD with balenaEtcher
3. Insert microSD into Jetson
4. Connect HDMI, keyboard, mouse, power
5. Power on, complete Ubuntu setup

**Step 2: Connect Peripherals**
```
Jetson Orin Nano
├─ USB 3.2 Port 1 → Intel RealSense D435i (USB-C to USB-A cable)
├─ USB 3.2 Port 2 → ReSpeaker Mic Array (USB 2.0)
├─ USB 3.2 Port 3 → Keyboard (during setup)
├─ USB 3.2 Port 4 → Mouse (during setup)
├─ HDMI → Monitor
├─ Ethernet → Router (or use Wi-Fi)
└─ Power → 19V/4.74A adapter
```

**Step 3: Install ROS 2 Humble**
```bash
# SSH into Jetson (or use direct terminal)
ssh robotics@jetson-orin.local

# Install ROS 2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop

# Install RealSense ROS wrapper
sudo apt install ros-humble-realsense2-camera

# Install Isaac ROS (Docker-based)
# Follow: https://nvidia-isaac-ros.github.io/getting_started/index.html
```

**Step 4: Test Hardware**
```bash
# Test RealSense
ros2 launch realsense2_camera rs_launch.py

# In another terminal, view images
ros2 run rqt_image_view rqt_image_view

# Test microphone
arecord -D hw:2,0 -r 16000 -f S16_LE test.wav
# Speak, then Ctrl+C
aplay test.wav
```

---

## Option 4: Full Robot Platform (Advanced)

**Best for**: Labs, research groups, well-funded capstone teams

### Budget Option: Hiwonder TonyPi Pro ($600)

**Specifications**:
- **Height**: 38cm (tabletop humanoid)
- **Weight**: 1.2kg
- **DOF**: 16 (head: 2, arms: 6×2, legs: 4×2)
- **Processor**: Raspberry Pi 4B (4GB)
- **Camera**: USB camera (640×480)
- **Power**: 7.4V Li-Po battery (2500mAh)
- **Programming**: Python, ROS (unofficial)

**Limitations**:
- **Raspberry Pi**: Cannot run Isaac ROS (needs Jetson)
- **Kinematics Only**: Focus on walking/manipulation, not perception
- **Low Battery Life**: ~30 minutes continuous operation

**Use Case**: Learn humanoid kinematics, then swap brain with Jetson kit.

---

### Mid-Range: Unitree Go2 Edu ($1,800-$3,000)

**Specifications**:
- **Type**: Quadruped (4-legged, not bipedal)
- **Weight**: 15kg
- **Payload**: 5kg
- **Speed**: 5 m/s max
- **Battery**: 4 hours continuous
- **Processor**: NVIDIA Jetson (varies by model)
- **Sensors**: LIDAR, 5× RGB cameras, IMU
- **ROS 2**: Official support

**Why Quadruped Instead of Humanoid?**:
- **Stability**: 4 legs = easier balance than 2
- **Durability**: Can handle falls better
- **Cost**: 1/5 the price of humanoid
- **ROS 2 Support**: Excellent documentation

**Limitation**: Not humanoid (bipedal locomotion different)

---

### Premium: Unitree G1 Humanoid (~$16,000)

**Specifications**:
- **Height**: 1.3m (half-scale humanoid)
- **Weight**: 35kg
- **DOF**: 23 (head: 2, arms: 7×2, torso: 3, legs: 6×2)
- **Processor**: NVIDIA Jetson Orin NX (16GB)
- **Sensors**: Intel RealSense D435i, LIDAR, IMU
- **Battery**: 2 hours continuous
- **Walking Speed**: 2 m/s
- **ROS 2**: Official SDK

**Why G1?**:
- **Affordable Humanoid**: 1/5 the price of Boston Dynamics Atlas
- **Open SDK**: ROS 2 control interfaces
- **Dynamic Walking**: Can handle uneven terrain
- **Research-Grade**: Used by universities worldwide

**Use Case**: Final capstone project deployment (shared lab resource)

---

## Comparison Table

| Option | Cost | GPU | AI Performance | Physical Robot | Best For |
|--------|------|-----|----------------|----------------|----------|
| **Simulation Only (Cloud)** | $85-180/quarter | A10G (24GB) | Excellent | ❌ No | Online learners |
| **Workstation (RTX 4070 Ti)** | $2,200 one-time | RTX 4070 Ti (12GB) | Excellent | ❌ No | Most students |
| **Workstation (RTX 4090)** | $3,000 one-time | RTX 4090 (24GB) | Outstanding | ❌ No | Researchers |
| **Edge AI Kit** | $700 one-time | Jetson Orin (40 TOPS) | Good | ✅ Sensors only | Budget physical AI |
| **Workstation + Edge Kit** | $2,900 one-time | Both | Best of both | ✅ Sensors only | Recommended combo |
| **Workstation + Go2** | $4,000 one-time | Both | Excellent | ✅ Quadruped | Lab setups |
| **Workstation + G1** | $18,000 one-time | Both | Excellent | ✅ Humanoid | Research labs |

---

## Recommended Setup by Budget

### $500-1,000: Simulation + Edge Kit
- Cloud instance (spot): $65/quarter
- Jetson Orin Nano kit: $700
- **Total**: $765 first quarter, $65/quarter after

**What you can do**:
- Complete all chapters in simulation
- Deploy voice commands to Jetson
- Test perception pipelines on real camera

---

### $2,000-2,500: Workstation Only
- RTX 4070 Ti build: $2,200
- **Total**: $2,200 one-time

**What you can do**:
- Complete all chapters at high performance
- Run Isaac Sim smoothly
- Train custom models
- No physical deployment (simulation only)

---

### $3,000-3,500: Workstation + Edge Kit (Recommended)
- RTX 4070 Ti build: $2,200
- Jetson Orin Nano kit: $700
- **Total**: $2,900 one-time

**What you can do**:
- ✅ Complete all chapters
- ✅ Deploy to physical edge hardware
- ✅ Run Isaac Sim + Gazebo
- ✅ Test perception on real sensors
- ✅ Voice-commanded navigation (simulated body, real brain/sensors)

**This is the recommended setup for serious students.**

---

### $5,000+: Workstation + Edge Kit + Robot
- RTX 4090 build: $3,000
- Jetson Orin Nano kit: $700
- Unitree Go2 Edu: $1,800
- **Total**: $5,500 one-time

**What you can do**:
- ✅ Everything above
- ✅ Deploy to physical quadruped robot
- ✅ Complete sim-to-real transfer
- ✅ Record research-grade demonstrations

**Best for**: Lab environments, research teams, funded capstone projects

---

## FAQ

### Can I use a Mac or Windows?

**Short Answer**: Not ideal. Use Ubuntu 22.04 (native or dual-boot).

**Long Answer**:
- **macOS**:
  - ❌ No NVIDIA GPU support (M1/M2/M3 are ARM, not CUDA)
  - ❌ Isaac Sim does not run on macOS
  - ⚠️ ROS 2 works via Homebrew but with limitations
  - ✅ Can use SSH to remote Ubuntu machine/cloud instance

- **Windows**:
  - ⚠️ WSL2 works for ROS 2 but GPU passthrough is limited
  - ❌ Isaac Sim requires native Ubuntu (not WSL2)
  - ✅ Dual-boot Ubuntu 22.04 recommended
  - ✅ Can use Windows for Unity (Module 2)

**Recommendation**: Dual-boot Ubuntu 22.04 + Windows 11 for best compatibility.

---

### Can I use an AMD GPU?

**Short Answer**: No, you need NVIDIA for Isaac Sim and Isaac ROS.

**Why NVIDIA Only**:
- **Isaac Sim**: Requires RTX GPUs (ray-tracing cores)
- **Isaac ROS**: Requires CUDA for TensorRT inference
- **ROS 2**: Works with AMD but loses GPU acceleration

**AMD Alternative**: Use ROCm for general PyTorch, but cannot run Isaac stack.

---

### What if I can't afford a workstation?

**Options**:
1. **Cloud Instances**: $65-180/quarter (affordable for 13 weeks)
2. **University Lab**: Many universities have RTX workstations
3. **Group Purchase**: Split cost with teammates
4. **Used Hardware**: RTX 3090 (used) ~$600, works well
5. **Simulation-Only**: Complete book without physical deployment

---

## Summary

- **Minimum**: Cloud instance + modest laptop (~$200 total)
- **Recommended**: RTX 4070 Ti workstation + Jetson Orin kit (~$2,900)
- **Premium**: RTX 4090 + Jetson + Robot (~$5,500+)

Choose based on your budget, learning goals, and access to physical robots.

---

**Next**: [Appendix B: Installation & Troubleshooting →](appendix-b-installation.md)

---

*Hardware recommendations validated as of December 2025. Prices subject to change.*
