# Setup Guide

This guide walks through setting up the Helios2 ToF camera with ROS 2 Humble from scratch.
**Total time: ~1 hour if everything goes smoothly.**

It assumes:
- Ubuntu 22.04 LTS on the host machine
- An Ethernet port available on the host (built-in or USB Ethernet adapter)
- The Helios2 camera with its power supply (12V DC barrel jack or PoE injector)
- An Ethernet cable (CAT5e or better)

If any of these are missing, stop and acquire them before starting.

---

## Step 1: Install ROS 2 Humble

Skip if already installed. Verify with `ros2 --version`.

Follow the official guide: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

Install the desktop version (includes RViz):
```bash
sudo apt update
sudo apt install ros-humble-desktop
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

Install build tools:
```bash
sudo apt install python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
```

---

## Step 2: Install Arena SDK

The Arena SDK is **proprietary** and must be downloaded from LUCID's website:
https://thinklucid.com/downloads-hub/

Get the **ArenaSDK Linux x64** package (tarball, not deb). At time of writing it's `ArenaSDK_v0.1.93_Linux_x64.tar.gz` or similar.

```bash
# Extract to your home directory (or any path you prefer)
cd ~
tar xzf ~/Downloads/ArenaSDK_*_Linux_x64.tar.gz
cd ArenaSDK_Linux_x64

# Run the installer (it just sets up library paths in /etc/ld.so.conf.d/)
sudo sh Arena_SDK_Linux_x64.conf

# Verify
ldconfig -p | grep arena
# Should show libarena.so, libsave.so, libgentl.so
```

> **Why this path matters:** the file `/etc/ld.so.conf.d/Arena_SDK.conf` is what our package's `Findarena_sdk.cmake` reads to locate the SDK. Don't move the SDK directory after installation.

---

## Step 3: Set up the camera network

GigE Vision cameras need to be on the same L2 broadcast domain as the host. The cleanest way is **direct Ethernet, host to camera, no router**.

### Why not just plug it into your home router?

Because home routers usually split wired and wireless clients into different subnets that don't bridge to each other. We tried this. It looked like it should work and didn't — see [ARCHITECTURE.md](ARCHITECTURE.md#why-direct-ethernet-not-through-a-router) for the diagnosis.

### Network setup

The Helios2's default IP is `192.168.100.40` (this is set at the factory; persistent across power cycles). We'll put the host on `192.168.100.1/24` to be in the same subnet.

**One-time setup:**
```bash
# Identify your Ethernet interface
ip link show
# Note the name — could be enp0s31f6, eth0, etc. Below uses enp0s31f6.

# Create a persistent NetworkManager profile for the camera link
sudo nmcli connection add type ethernet ifname enp0s31f6 con-name helios2-cam \
    ipv4.method manual \
    ipv4.addresses 192.168.100.1/24 \
    ipv4.never-default yes \
    ipv4.dns "" \
    autoconnect yes

# Delete any default "Wired connection 1" so it doesn't fight us
sudo nmcli connection delete "Wired connection 1" 2>/dev/null || true

# Activate
sudo nmcli connection up helios2-cam
```

> **Why `ipv4.never-default yes`:** without this flag, if the camera link somehow grabs a default gateway from DHCP (which shouldn't happen but does), it will steal your internet routing. The flag locks this interface as "camera-only, never internet."

> **Why a NetworkManager profile, not raw `ip addr add`:** NetworkManager will wipe raw `ip` commands every time you sleep/wake the laptop or reboot. A proper profile survives reboots.

### How to get internet at the same time

You need a **second** network interface for internet — WiFi, or a USB Ethernet adapter. Don't try to share the Ethernet between the camera and the home network. We tried, it doesn't work.

If you use a USB Ethernet adapter for internet:
```bash
# Plug it in. NetworkManager should auto-configure it for DHCP.
# If it doesn't get an IP, force DHCP:
sudo dhclient -v <usb-ethernet-interface>
```

### Physical connection

1. Plug one end of an Ethernet cable into the camera.
2. Plug the other end into the host's Ethernet port.
3. Power the camera (12V DC or PoE injector).
4. Wait ~10 seconds for the camera to boot.

### Verify the camera is reachable

```bash
ping -c 3 192.168.100.40
```

You should see sub-millisecond replies. If not, see [TROUBLESHOOTING.md](TROUBLESHOOTING.md#ping-fails).

---

## Step 4: Network tuning for GigE Vision

ToF cameras dump a lot of UDP data. Default Linux socket buffers (~208 KB) are too small for sustained 30 Hz streaming. Bump them:

```bash
# Immediate effect
sudo sysctl -w net.core.rmem_max=33554432
sudo sysctl -w net.core.rmem_default=33554432
sudo sysctl -w net.core.wmem_max=33554432

# Persistent across reboots
sudo tee -a /etc/sysctl.conf <<EOF
# GigE Vision tuning for Helios2 ToF camera
net.core.rmem_max=33554432
net.core.rmem_default=33554432
net.core.wmem_max=33554432
EOF
```

> **Why this matters:** if the kernel's UDP receive buffer fills up, packets get dropped silently. The Arena SDK then logs "Incomplete frame" and may eventually throw a TimeoutException. 32 MB is plenty of headroom.

---

## Step 5: Create a colcon workspace

Skip if you already have one.

```bash
mkdir -p ~/cobot_ws/src
cd ~/cobot_ws
colcon build
source install/setup.bash
echo 'source ~/cobot_ws/install/setup.bash' >> ~/.bashrc
```

---

## Step 6: Clone this repository

```bash
cd ~/cobot_ws/src
git clone https://github.com/YOUR_USER/helios2_ros2.git
cd helios2_ros2
```

> If you don't have git access, copy the package directory directly.

---

## Step 7: Install ROS dependencies

```bash
cd ~/cobot_ws
rosdep install --from-paths src --ignore-src -r -y
```

Also install OpenCV (Arena SDK has an undocumented dependency on it — see [ARCHITECTURE.md](ARCHITECTURE.md#opencv-quirk)):
```bash
sudo apt install libopencv-dev
```

---

## Step 8: Build the driver

```bash
cd ~/cobot_ws
colcon build --packages-select helios2_node --symlink-install
source install/setup.bash
```

The first build may produce a warning about `arena_sdk_installation_root` — this is informational, not an error. The build is successful if it says "1 package finished".

---

## Step 9: Run

```bash
ros2 launch helios2_node helios2.launch.py
```

You should see:
```
[INFO] Opening device: HTP003S-001  S/N=233300002  IP=192.168.100.40
[INFO] Setting PixelFormat = Coord3D_ABCY16
[INFO] Setting Scan3dOperatingMode = Distance3000mmSingleFreq
[INFO] ExposureTimeSelector = Exp1000Us
[INFO] Coord scales (mm/LSB): X=0.250000 Y=0.250000 Z=0.250000 | offsets (mm): X=-8192.000 Y=-8192.000 Z=0.000
[INFO] Stream started.
[INFO] helios2_node up. Streaming on 'helios2/points'
[INFO] Capture thread started.
```

In a second terminal:
```bash
source ~/cobot_ws/install/setup.bash
ros2 topic hz /helios2/points
```

Expected: ~30 Hz.

---

## Step 10: View in RViz

```bash
# Terminal 2 — static TF
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world helios2_optical_frame

# Terminal 3 — RViz
rviz2
```

In RViz:
1. **Fixed Frame** → `world`
2. **Add → By topic → /helios2/points → PointCloud2**
3. **Critical:** Set **Reliability Policy** to **Best Effort** (in the display's QoS section). RViz defaults to Reliable which silently won't match.
4. **Color Transformer** → `AxisColor`, **Axis** → `Z`
5. **Size (m)** → 0.01 for crisper rendering

Wave your hand in front of the camera. You should see a 3D point cloud reconstruction.

---

## Step 11 (optional): Add to startup

If you use this camera regularly, add the workspace to your bashrc:
```bash
echo 'source ~/cobot_ws/install/setup.bash' >> ~/.bashrc
```

NetworkManager profile `helios2-cam` already activates on boot since we set `autoconnect yes`. The sysctl changes are persistent. So a fresh boot should give you a camera-ready laptop without any manual steps.

---

## What's next

- **URDF**: Create a URDF for the Helios2 so it has a proper TF frame, with mount geometry relative to the robot.
- **Calibration**: Extrinsic calibration to know where the camera is in the robot's world.
- **Filtering**: Voxel grid downsampling, statistical outlier removal, ROI cropping — all standard PCL stuff that runs as separate nodes downstream of `helios2_node`.
- **Application**: Whatever the cobot is supposed to do with the depth data.
