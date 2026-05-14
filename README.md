# helios2_ros2

ROS 2 Humble driver and integration for the **LUCID Vision Helios2 / Helios2+** time-of-flight (ToF) camera.

Publishes:
- `helios2/points` — `sensor_msgs/PointCloud2` (XYZI, float32, **meters**, organized)
- `helios2/intensity` — `sensor_msgs/Image` (mono16)

Tested on:
- Ubuntu 22.04 LTS
- ROS 2 Humble
- LUCID Arena SDK (Linux x64)
- Helios2+ (model HTP003S-001)

---

## Quick start

If you already have ROS 2 Humble and the Arena SDK installed:

```bash
# 1. Clone into your workspace
cd ~/cobot_ws/src
git clone https://github.com/Dinoopdas/helios2_ros2.git
cd helios2_ros2

# 2. Set up the camera network (see SETUP.md for details)
./scripts/setup_network.sh

# 3. Build
cd ~/cobot_ws
colcon build --packages-select helios2_node --symlink-install
source install/setup.bash

# 4. Run
ros2 launch helios2_node helios2.launch.py
```

If you don't yet have ROS 2 + Arena SDK installed, or this is your first time setting up the camera, **read [SETUP.md](docs/SETUP.md) first**. It walks you through everything start to finish.

---

## Documentation

- **[SETUP.md](docs/SETUP.md)** — Full installation guide, start to finish. Read this first.
- **[ARCHITECTURE.md](docs/ARCHITECTURE.md)** — How the driver works and why it's built this way.
- **[TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)** — Every error we hit, how to fix it.
- **[LEARNING.md](docs/LEARNING.md)** — Background concepts. Read this if you've never worked with GigE Vision or industrial cameras.

---

## Visualizing in RViz

```bash
# Terminal 1 — driver
ros2 launch helios2_node helios2.launch.py

# Terminal 2 — static TF (until URDF exists)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world helios2_optical_frame

# Terminal 3 — RViz
rviz2
```

In RViz:
1. **Global Options → Fixed Frame** = `world`
2. **Add → By topic → /helios2/points → PointCloud2**
3. **Important:** Set the PointCloud2 display's **Reliability Policy** to **Best Effort**.
   (The driver publishes BEST_EFFORT QoS; RViz defaults to RELIABLE which won't match.)
4. **Color Transformer** → `AxisColor`, Axis → `Z`

---

## Status

- ✅ Camera discovery and connection
- ✅ Coord3D_ABCY16 decode → PointCloud2
- ✅ Intensity image publishing
- ✅ ~30 Hz steady-state streaming
- 🔴 URDF / camera mount frame
- 🔴 Point cloud filtering / downsampling

---

## License

Apache 2.0. Arena SDK is proprietary to LUCID Vision Labs and must be obtained from them directly.
