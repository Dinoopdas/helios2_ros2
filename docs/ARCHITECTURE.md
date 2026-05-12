# Architecture

This document explains **why** the driver is built the way it is. If you want the *how*, see [SETUP.md](SETUP.md). If you're just trying to fix a problem, see [TROUBLESHOOTING.md](TROUBLESHOOTING.md).

---

## The big picture

```
+--------------------------+         +------------------------+
|   Helios2 ToF Camera     |  GigE   |  Host (Ubuntu 22.04)   |
|   192.168.100.40         |<------->|  192.168.100.1         |
|                          |  cable  |                        |
|   - Sends Coord3D_ABCY16 |         |  - helios2_node        |
|     UDP packets at 30 Hz |         |  - Arena SDK           |
+--------------------------+         |  - ROS 2 Humble        |
                                     |                        |
                                     |  Publishes:            |
                                     |  /helios2/points       |
                                     |  /helios2/intensity    |
                                     +------------------------+
                                              |
                                              | DDS
                                              v
                                     +------------------------+
                                     |  Downstream consumers  |
                                     |  - RViz2               |
                                     |  - PCL filters         |
                                     |  - Object detection    |
                                     |  - Motion planning     |
                                     +------------------------+
```

The driver is intentionally **thin** — it does the minimum work to get camera data into ROS-standard message types. Everything else (filtering, perception, calibration) is a separate downstream node.

---

## Why direct Ethernet, not through a router?

This was the single biggest time sink during the initial setup. Here's what we learned.

### What we tried first

Camera plugged into a home router, laptop on the router's WiFi. Both "on the same network."

### What actually happens

The router's WiFi clients got addresses in `192.168.100.x` (where the camera lives). Wired clients (which we didn't have) got `192.168.1.x`. The router does **not** bridge between these — they're effectively separate L2 segments.

But the laptop, with both WiFi and Ethernet active, saw the camera through *two* paths:
1. WiFi → router → camera (via the router's internal bridging on `192.168.100.x`)
2. Ethernet → ... → nowhere (the router didn't carry our camera subnet on its wired side either)

GigE Vision uses UDP broadcast discovery. When the SDK enumerated, it found the camera **twice** (once per interface) and didn't know which "device" was real. It picked the wrong one and timed out trying to stream.

Turning WiFi off entirely was the diagnostic that proved this.

### Why direct Ethernet is the right answer

Industrial vision practice is **one dedicated NIC per camera, direct cable, no switches if possible**. This eliminates:
- Subnet bridging issues
- Switch latency variability
- Multi-camera interference
- Any router doing something weird with QoS or multicast

For the host's internet access, use a **second** NIC (USB Ethernet adapter or WiFi). They're physically and logically isolated.

### How this works in production

In a real cobot cell:
- Camera NIC is a dedicated Intel i210/i350 PCIe card or a quality USB 3.0 Ethernet adapter
- Camera and host share a managed switch with jumbo frames, multicast filtering, and IGMP snooping disabled
- Cabling is industrial-grade with locking M12 connectors
- The "internet" NIC is on a completely separate network (often air-gapped on factory floors)

---

## Why a custom driver and not an existing one?

We started with `arena_camera_node` (a generic ROS 2 wrapper around the Arena SDK that already existed in the workspace). It worked, in the sense that the node ran and published topics. But:

- It only publishes `sensor_msgs/Image`, not `PointCloud2`.
- Its `pixelformat_translation.h` only maps **2D image** formats: Mono8, RGB8, Bayer*, YUV422. There are zero entries for any Coord3D format.
- The Helios2 was therefore being treated as a 2D camera — outputting the intensity channel only, throwing away the X, Y, Z coordinate data.

You can't fix that with parameters. The driver fundamentally doesn't understand 3D pixel formats. The fix would have been a substantial rewrite of `arena_camera_node` itself, which would risk breaking its use with regular 2D cameras.

The correct architectural choice: **write a separate driver specialized for the Helios2 ToF pipeline**.

This keeps responsibilities clean:
- `arena_camera_node`: 2D Arena-SDK cameras (industrial inspection, etc.)
- `helios2_node`: 3D Arena-SDK ToF cameras

If someone later wants to support the LUCID Triton (high-res 2D), they extend `arena_camera_node`. If they want a different ToF camera (Pico Zense, Photoneo), they fork `helios2_node`. No one driver tries to do everything.

---

## How the data pipeline works

### Coord3D_ABCY16 pixel format

The Helios2 outputs one of several pixel formats. We use `Coord3D_ABCY16`. Each pixel is **8 bytes**:

| Bytes | Channel | Type     | Meaning                |
|-------|---------|----------|------------------------|
| 0–1   | A       | int16_t  | X coordinate (raw)     |
| 2–3   | B       | int16_t  | Y coordinate (raw)     |
| 4–5   | C       | int16_t  | Z (depth) coordinate   |
| 6–7   | Y       | uint16_t | Intensity (confidence) |

The raw values aren't in meters — they're integer LSBs that need to be multiplied by per-axis scale factors and shifted by per-axis offsets.

### Scale and offset

These come from the camera's GenICam nodemap:

```cpp
SetNode("Scan3dCoordinateSelector", "CoordinateA");  // or B, C
scale  = GetNode<double>("Scan3dCoordinateScale");   // mm per LSB
offset = GetNode<double>("Scan3dCoordinateOffset");  // mm
```

For our Helios2+:
- `scale_x = scale_y = scale_z = 0.25` mm/LSB
- `offset_x = offset_y = -8192` mm  (re-centers the origin)
- `offset_z = 0` mm

So the conversion per pixel is:
```
x_meters = (raw_int16 * 0.25 + (-8192)) * 0.001
y_meters = (raw_int16 * 0.25 + (-8192)) * 0.001
z_meters = (raw_int16 * 0.25 +   0   ) * 0.001
```

> **Gotcha:** People sometimes forget the offset and get a point cloud that's "translated to the side" by ~2 meters. Don't do that. Read both scale AND offset, per axis.

### Invalid pixels

When the camera can't get a depth return (out of range, no reflective surface, multipath interference), all three coordinate channels are 0. We detect this and emit NaN in the point cloud so that downstream PCL/RViz filters automatically drop them.

```cpp
const bool invalid = (a == 0 && b == 0 && c == 0);
if (invalid) {
  dst[0] = dst[1] = dst[2] = NaN;
}
```

We also set `is_dense = false` on the PointCloud2 message to signal that NaNs may be present.

### Frame conventions

The published cloud is in the **camera optical frame** (REP-103 convention for camera data):
- **+X** right
- **+Y** down
- **+Z** forward (into the scene)

This is the standard for camera frames in ROS. If you want REP-103 robot convention (X forward, Y left, Z up), publish a static TF that rotates the optical frame appropriately. We'll do this properly via URDF later.

### Threading

The Arena SDK's `GetImage()` is **blocking**. If we ran it on the rclcpp executor thread, the node couldn't respond to parameter changes, service calls, or shutdown signals during the blocking call. So:

- `main()` thread: rclcpp::spin (handles ROS callbacks)
- Capture thread: dedicated `std::thread` running `captureLoop()`, doing `GetImage()` → decode → publish in a tight loop

The capture thread is started in the constructor and joined in the destructor. The `running_` atomic flag is the clean-shutdown signal.

### QoS choice

We publish with `BEST_EFFORT` + `VOLATILE` + `KeepLast(5)`. This is the standard "sensor data" QoS profile:
- `BEST_EFFORT`: tolerates packet loss between publisher and subscriber. We're at 30 Hz; if a single message is dropped, the next one is 33 ms away.
- `VOLATILE`: late-joining subscribers don't get old data.
- `KeepLast(5)`: short history.

**This is why RViz needs to be configured for "Best Effort" reliability** — by default it asks for RELIABLE, which won't match a BEST_EFFORT publisher, and you'll see "incompatible QoS" warnings and no data. This trips up everyone the first time.

---

## OpenCV quirk

The Arena SDK's `libarena.so` has an **undeclared** dependency on OpenCV's `calib3d` module (specifically `cv::projectPoints`). LUCID compiled their SDK against OpenCV but the .so doesn't list calib3d in its `DT_NEEDED` entries.

Result: linking against `libarena.so` alone produces:
```
undefined reference to `cv::projectPoints(...)`
```

Fix: in `CMakeLists.txt`, find_package OpenCV and link `${OpenCV_LIBS}` (plus an explicit `opencv_calib3d`) into any executable that uses Arena.

```cmake
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)
target_link_libraries(helios2_node ${arena_sdk_LIBRARIES} ${OpenCV_LIBS} opencv_calib3d)
```

How to find this kind of thing in the future:
```bash
nm -D /path/to/libvendor.so | grep " U "      # 'U' = undefined symbols
ldd /path/to/libvendor.so                     # what it actually links
```

---

## Things we considered but didn't do (yet)

### Image transport plugins
We publish raw `sensor_msgs/Image` for intensity. Adding `image_transport` would enable compressed_image variants for bandwidth-constrained scenarios. Not needed yet; can add later.

### ros2_control hardware interface
If the camera ever needs to participate in a `ros2_control` pipeline (e.g., as a feedback sensor), it would need a `hardware_interface::SensorInterface` adapter. Premature for now.

### Time synchronization
We stamp messages with `node.now()` at publish time. The camera also exposes its own timestamp registers via GenICam — using those would give sub-millisecond accuracy across multi-camera setups. Worth adding when we have multiple synchronized sensors.

### Calibration / camera_info
We don't publish `sensor_msgs/CameraInfo`. The Helios2 is factory-calibrated and the X/Y/Z values are already metric, so there's no need to publish intrinsics separately. If we ever wanted to project the depth onto a separate RGB camera, that would change.

### Multi-camera support
The driver opens a single device. Adding multi-camera support is a few hours of work (parameterize the serial, instantiate N nodes via a launch file). Not needed yet.

---

## Mistakes we made (so you don't have to)

The full chronological list is in [TROUBLESHOOTING.md](TROUBLESHOOTING.md). Highlights:

1. **Tried to use the home router.** Wasted hours. Doesn't work.
2. **Set MTU to 9000 before testing.** Camera didn't negotiate jumbo frames, every packet got dropped, ping failed. Always start at MTU 1500.
3. **Tried to extend the existing arena_camera_node for 3D.** Wrong architecture; write a separate driver.
4. **Forgot the OpenCV calib3d link.** Twice. Just always add it.
5. **Used raw `ip addr add` instead of NetworkManager profile.** Got wiped on sleep/reboot.
6. **Left WiFi on with overlapping subnets.** Caused dual enumeration of the camera.
7. **Didn't bump kernel UDP buffers.** Got "incomplete frame" warnings under load.
8. **Didn't catch GenICam exceptions in the capture loop.** A single cable wiggle crashed the entire node.
