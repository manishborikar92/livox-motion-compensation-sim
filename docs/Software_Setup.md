# Software Setup Guide

## Prerequisites

### System Requirements
- **Operating System**: Ubuntu 20.04 LTS (recommended) or Windows 10/11 Pro
- **CPU**: Intel i7 8-core or AMD Ryzen 7 equivalent
- **RAM**: 32GB minimum (64GB recommended for large datasets)
- **Storage**: 2TB NVMe SSD for high-speed data logging
- **Network**: Gigabit Ethernet interface

### Python Environment
```bash
# Python 3.8 or higher required
python3 --version

# Install pip if not available
sudo apt update
sudo apt install python3-pip python3-venv
```

## Core Software Installation

### 1. Livox SDK Installation (Critical)

**⚠️ Important**: Use the original Livox SDK, NOT SDK2 for Mid-70 compatibility.

```bash
# Install dependencies
sudo apt install build-essential cmake git

# Clone the correct SDK for Mid-70
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK

# Build the SDK
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install

# Verify installation
./sample/lidar_sample/lidar_sample
```

### 2. Python Dependencies

Create a virtual environment and install required packages:

```bash
# Create virtual environment
python3 -m venv livox_env
source livox_env/bin/activate

# Install core dependencies
pip install numpy scipy matplotlib pandas
pip install utm laspy open3d

# Optional: Install additional packages
pip install opencv-python scikit-learn
```

### 3. Point Cloud Library (PCL)

```bash
# Install PCL development libraries
sudo apt install libpcl-dev pcl-tools

# Verify installation
pcl_viewer --version
```

## ROS Integration (Optional)

### ROS1 (Noetic) Setup

```bash
# Install ROS Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full

# Initialize rosdep
sudo rosdep init
rosdep update

# Setup environment
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Create catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Install correct Livox ROS driver for Mid-70
cd ~/catkin_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver.git
cd ~/catkin_ws
catkin_make
```

### ROS2 (Galactic/Humble) Setup

```bash
# Install ROS2 Galactic (Ubuntu 20.04)
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-galactic-desktop

# Setup environment
echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Create colcon workspace
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src

# Install correct Livox ROS2 driver for Mid-70
git clone https://github.com/Livox-SDK/livox_ros2_driver.git
cd ~/colcon_ws
colcon build
echo "source ~/colcon_ws/install/setup.bash" >> ~/.bashrc
```

## Network Configuration

### Firewall Setup
```bash
# Allow Livox UDP ports
sudo ufw allow 65000:65002/udp
sudo ufw allow from 192.168.1.0/24

# Verify firewall status
sudo ufw status
```

### Network Interface Configuration
```bash
# Configure static IP for LiDAR communication
sudo nano /etc/netplan/01-network-manager-all.yaml

# Add configuration:
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 192.168.1.100/24
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]

# Apply configuration
sudo netplan apply
```

## Development Environment

### IDE Setup

**Visual Studio Code (Recommended)**
```bash
# Install VS Code
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
sudo apt update
sudo apt install code

# Install useful extensions
code --install-extension ms-python.python
code --install-extension ms-vscode.cpptools
code --install-extension ms-iot.vscode-ros
```

### Git Configuration
```bash
# Configure Git
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"

# Clone the simulation repository
git clone <repository-url>
cd livox-motion-compensation-sim
```

## Simulation Framework Setup

### Install Simulation Dependencies
```bash
# Activate virtual environment
source livox_env/bin/activate

# Install simulation-specific packages
pip install -r requirements.txt

# Verify installation
python -c "import numpy, scipy, matplotlib; print('Dependencies OK')"
```

### Configuration Files

Create configuration directory:
```bash
mkdir -p ~/.config/livox_sim
cp config/default_config.yaml ~/.config/livox_sim/
```

### Test Installation

```bash
# Run basic connectivity test
python test_installation.py

# Expected output:
# ✅ Python environment: OK
# ✅ NumPy/SciPy: OK
# ✅ Matplotlib: OK
# ✅ Network configuration: OK
# ✅ Livox SDK: OK
```

## GNSS/INS Integration

### NovAtel Integration
```bash
# Install NovAtel Connect SDK
wget https://docs.novatel.com/OEM7/Content/Software/NovAtel_Connect.htm
# Follow manufacturer installation instructions

# Configure network interface for GNSS receiver
sudo ip addr add 192.168.1.100/24 dev eth1
```

### VectorNav Integration
```bash
# Download and install VectorNav library
wget https://www.vectornav.com/docs/default-source/downloads/linux-libraries/vnproglib-1.1.5.0.tar.gz
tar -xzf vnproglib-1.1.5.0.tar.gz
cd vnproglib-1.1.5.0
make
sudo make install
```

## Time Synchronization

### Chrony Setup for PPS
```bash
# Install chrony
sudo apt install chrony

# Configure PPS input (if hardware available)
echo 'pps-gpio' | sudo tee -a /etc/modules
echo 'dtoverlay=pps-gpio,gpiopin=18' | sudo tee -a /boot/config.txt

# Configure chrony
sudo nano /etc/chrony/chrony.conf
# Add: refclock PPS /dev/pps0 trust lock NMEA

sudo systemctl restart chrony
```

### NTP Synchronization
```bash
# Install NTP client
sudo apt install ntp

# Configure NTP servers
sudo nano /etc/ntp.conf
# Add preferred time servers

sudo systemctl restart ntp
```

## Verification and Testing

### Hardware Connectivity Test
```bash
# Test LiDAR connectivity
ping 192.168.1.12x  # Replace x with your LiDAR's last IP digit

# Test UDP port connectivity
nc -u 192.168.1.12x 65000
```

### Software Functionality Test
```bash
# Run Livox SDK sample
cd Livox-SDK/build
./sample/lidar_sample/lidar_sample

# Expected output:
# [INFO] Livox lidar detected successfully
# [INFO] Device S/N: 3GGDJ6K00200101
# [INFO] Firmware: 03.08.0000
```

### ROS Integration Test (if installed)
```bash
# Source ROS environment
source ~/catkin_ws/devel/setup.bash  # ROS1
# or
source ~/colcon_ws/install/setup.bash  # ROS2

# Launch Livox driver
roslaunch livox_ros_driver livox_lidar_msg.launch  # ROS1
# or
ros2 launch livox_ros2_driver msg_MID360_launch.py  # ROS2

# Verify topics
rostopic list | grep livox  # ROS1
# or
ros2 topic list | grep livox  # ROS2
```

## Troubleshooting

### Common Issues

**1. SDK Compilation Errors**
```bash
# Install missing dependencies
sudo apt install build-essential cmake git
sudo apt install libeigen3-dev libpcl-dev

# Clean and rebuild
cd Livox-SDK/build
make clean
cmake ..
make -j$(nproc)
```

**2. Network Connectivity Issues**
```bash
# Check network interface
ip addr show

# Test connectivity
ping 192.168.1.12x
telnet 192.168.1.12x 65000

# Check firewall
sudo ufw status
sudo ufw allow 65000:65002/udp
```

**3. Permission Issues**
```bash
# Add user to dialout group (for serial devices)
sudo usermod -a -G dialout $USER

# Set network capabilities
sudo setcap cap_net_raw+ep /usr/bin/python3
```

**4. ROS Build Errors**
```bash
# Update rosdep
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Clean build
rm -rf build/ devel/  # ROS1
# or
rm -rf build/ install/ log/  # ROS2

# Rebuild
catkin_make  # ROS1
# or
colcon build  # ROS2
```

## Performance Optimization

### System Tuning
```bash
# Increase network buffer sizes
echo 'net.core.rmem_max = 134217728' | sudo tee -a /etc/sysctl.conf
echo 'net.core.wmem_max = 134217728' | sudo tee -a /etc/sysctl.conf
sudo sysctl -p

# Set CPU governor to performance
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

### Memory Configuration
```bash
# Increase shared memory
echo 'kernel.shmmax = 68719476736' | sudo tee -a /etc/sysctl.conf
echo 'kernel.shmall = 4294967296' | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

## Next Steps

After completing the software setup:

1. **Hardware Connection**: Connect Livox Mid-70 and GNSS/INS systems
2. **Calibration**: Perform extrinsic calibration between sensors
3. **Testing**: Run simulation framework and validate outputs
4. **Integration**: Integrate with your specific application

Refer to the [Usage Guide](Usage_Guide.md) for detailed instructions on running simulations and processing data.