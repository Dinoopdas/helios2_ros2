# helios2_node

ROS 2 Humble driver for the **LUCID Vision Helios2** time-of-flight camera.

Publishes:
- `helios2/points`    — `sensor_msgs/PointCloud2` (organized, XYZI, float32, **meters**)
- `helios2/intensity` — `sensor_msgs/Image` (mono16 intensity)

Coordinate frame: standard ROS camera optical convention
(Z forward into scene, X right, Y down). Frame id defaults to `helios2_optical_frame`.

---

## Install

1. Copy this package into your workspace `src/`:

   ```bash
   cp -r helios2_node ~/cobot_ws/src/
   ```

2. Copy the Arena SDK find-module from the existing `arena_camera_node` package:

   ```bash
   cp ~/cobot_ws/src/arena_camera_node/cmake/Findarena_sdk.cmake \
      ~/cobot_ws/src/helios2_node/cmake/
   ```

3. Build:

   ```bash
   cd ~/cobot_ws
   colcon build --packages-select helios2_node --symlink-install
   source install/setup.bash
   ```

---

## Run

```bash
ros2 launch helios2_node helios2.launch.py
# or directly:
ros2 run helios2_node helios2_node
```

### Useful parameters

| Param                    | Default                         | Notes                                  |
|--------------------------|---------------------------------|----------------------------------------|
| `operating_mode`         | `Distance3000mmSingleFreq`      | Range/freq preset. See Helios2 docs.   |
| `exposure_time_selector` | `Exp1000Us`                     | `Exp62_5Us` / `Exp250Us` / `Exp1000Us` |
| `frame_id`               | `helios2_optical_frame`         | Frame attached to cloud + image        |
| `publish_intensity`      | `true`                          | Disable to save bandwidth/CPU          |
| `invalid_as_nan`         | `true`                          | Emit NaN for no-return pixels          |
| `serial_number`          | `""`                            | Pick a specific camera                 |

Example:

```bash
ros2 launch helios2_node helios2.launch.py \
    operating_mode:=Distance6000mmSingleFreq \
    exposure_time_selector:=Exp250Us
```

---

## Verify

```bash
# Topic check
ros2 topic list
ros2 topic info /helios2/points --verbose
ros2 topic hz   /helios2/points         # expect ~30 Hz
ros2 topic echo /helios2/points --field header --once

# Visualize
rviz2
#   1. Set Fixed Frame to "helios2_optical_frame" (or add a static transform)
#   2. Add -> PointCloud2 -> /helios2/points
#   3. Add -> Image       -> /helios2/intensity
```

If RViz won't display the cloud and Fixed Frame is correct, check QoS:
the cloud is published as `BEST_EFFORT`. In RViz's PointCloud2 display,
set Reliability Policy to "Best Effort".

---

## Static transform for RViz

Until you put this in a URDF, publish a static TF so RViz has somewhere to anchor:

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world helios2_optical_frame
```

---

## Troubleshooting

- **TimeoutException on GetImage**: GigE link issue. Verify `ping` to the camera,
  check MTU (1500 by default, jumbo only if every link supports it), and confirm
  `StreamAutoNegotiatePacketSize=true` (the driver sets this automatically).
- **"Incomplete frame" warnings**: packet loss. Try a shorter / better cable,
  bump `net.core.rmem_max`, or disable other traffic on the NIC.
- **All points at the origin**: bad pixel format. Confirm in the startup log
  that PixelFormat was set to `Coord3D_ABCY16`.
- **Cloud appears upside-down or mirrored in RViz**: that's the optical frame
  convention. Add a URDF/TF that puts a proper `helios2_link` (REP-103: X
  forward, Y left, Z up) and use that for downstream nodes.
