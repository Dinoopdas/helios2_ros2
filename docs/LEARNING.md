# Learning Guide

This is for someone who is new to industrial vision and wants to understand **why** things work the way they do.
If you're an experienced robotics engineer, you can skip this. If you came from web/app development or general embedded work, **read this first** — it will save you days of confusion.

Topics:
1. GigE Vision and GenICam (what they are, why they matter)
2. ToF (time-of-flight) depth sensing — how a Helios2 actually sees in 3D
3. Pixel formats (Coord3D_ABCY16 specifically)
4. ROS 2 sensor data conventions
5. QoS in DDS — the silent killer of "why am I not seeing data"

---

## 1. GigE Vision and GenICam

These are the **two industry standards** that make industrial cameras interoperable. Every serious machine vision camera supports them. Understanding them turns "magic black box" into "well-defined network device."

### GigE Vision

A streaming protocol on top of UDP. Specifies:
- How a host discovers cameras on a network (UDP broadcast on port 3956)
- How a host reserves a camera ("control channel")
- How the camera sends image data ("stream channel")
- Error recovery, packet resending, etc.

Why UDP and not TCP? **Latency.** TCP retransmits on packet loss, which on a real-time link adds unpredictable delay. UDP just sends frames; if some pixels are lost, you reject that frame and use the next one (33 ms later for 30 Hz).

**Implication:** Your network has to be clean. No retransmits, no buffering, no NAT, no firewalls in the path. This is why we direct-connect the camera.

### GenICam

A **vendor-neutral API** for configuring cameras. Specifies:
- A standard "nodemap" of settings (exposure, gain, pixel format, etc.) accessed by name
- A standard for advertising vendor-specific features
- The XML schema that cameras use to describe their own configuration tree
- C++ libraries (`GenApi`, `GCBase`) that parse the XML and let you read/write nodes by name

When you write:
```cpp
Arena::SetNodeValue<gcstring>(nm, "PixelFormat", "Coord3D_ABCY16");
```
You're using GenICam to write to a standard node ("PixelFormat") that's defined in the SFNC (Standard Features Naming Convention). Any GenICam-compliant camera has this same node. The Arena SDK is LUCID's wrapper that adds GigE Vision streaming on top of GenICam configuration.

**Implication:** If you understand the GenICam SFNC, you can configure any industrial camera. The node names are stable across vendors. Look up "GenICam SFNC PDF" for the full spec.

### How they fit together

```
+----------------+    GigE Vision (UDP streaming)     +-----------+
|  Camera        |<--------------------------------- >|  Host     |
|  ----------    |                                    |  -------  |
|  GenApi XML    |    GenICam (TCP/UDP config writes) |  Arena    |
|  (describes    |<-----------------------------------|  SDK      |
|   features)    |                                    |  (LUCID)  |
+----------------+                                    +-----------+
```

The Arena SDK = LUCID's implementation of GigE Vision + GenICam, packaged as a C++ API.

---

## 2. ToF (time-of-flight) depth sensing

Three main 3D camera technologies, all very different:

| Tech              | How it works                                | Used in              |
|-------------------|---------------------------------------------|----------------------|
| Stereo            | Two cameras, triangulate matched features   | ZED, Intel Realsense D4xx |
| Structured Light  | Project a known pattern, see deformation    | Kinect v1, Photoneo  |
| **Time-of-Flight**| Modulate IR light, measure phase delay      | **Helios2**, Kinect v2 |

### How Helios2 works specifically

The Helios2 emits infrared light modulated at a known frequency (e.g., 100 MHz). The light bounces off objects in the scene and returns to a sensor with thousands of pixels, each measuring the phase delay of the returning light.

```
phase_delay = (4π × distance × frequency) / c
distance    = (phase_delay × c) / (4π × frequency)
```

Each pixel independently calculates its own depth. The result is a depth map at the camera's native resolution (typically 640×480 for the Helios2).

### Why this matters operationally

- **Range is hard-bounded by modulation frequency.** The Helios2 has multiple "operating modes" — `Distance1250mmSingleFreq`, `Distance3000mmSingleFreq`, `Distance6000mmSingleFreq`, etc. Higher range = lower frequency = coarser depth resolution. Trade-off.
- **Highly reflective or absorbing surfaces fail.** Mirrors, black velvet, polished metal — these confuse phase measurement and produce garbage.
- **Multipath interference.** Light bounces twice off a corner before returning, giving a fake depth. Corners and concave shapes have known artifacts.
- **Exposure time vs. motion blur.** Longer exposure = more SNR but more blur. The `ExposureTimeSelector` controls this. `Exp1000Us` is a good default.

### Why does it output XYZ, not just depth?

Each pixel knows its own depth (Z) AND its angular position on the sensor. The camera applies the lens model (factory calibrated) to project depth into 3D (X, Y, Z). So you're getting a **fully calibrated point cloud**, not a raw depth image. This is why you don't need to publish camera_info or do any projection on the host.

---

## 3. Pixel formats: what is Coord3D_ABCY16?

A pixel format is just **how bytes are laid out per pixel** in the buffer the camera sends. Industrial cameras support many formats — each is optimized for different use cases.

| Format            | Layout per pixel        | Bytes | Use case                |
|-------------------|------------------------|-------|-------------------------|
| Mono8             | grayscale uint8        | 1     | Standard 2D B&W         |
| Mono16            | grayscale uint16       | 2     | High-dynamic-range B&W  |
| RGB8              | R, G, B as uint8       | 3     | Standard 2D color       |
| BayerRG8          | raw Bayer pattern      | 1     | Pre-debayer color       |
| **Coord3D_ABCY16**| X16, Y16, Z16, I16     | **8** | **3D + intensity**      |
| Coord3D_ABC16     | X16, Y16, Z16 only     | 6     | 3D without intensity    |
| Coord3D_C16       | Z16 only (depth map)   | 2     | Depth as 2D image       |

The "ABCY" naming comes from GenICam:
- A = first coordinate (X)
- B = second coordinate (Y)
- C = third coordinate (Z)
- Y = intensity (Y here means "luminance," not the Y axis — confusing, blame GenICam)

The "16" means each channel is 16-bit. Coord3D_ABCY16 = four 16-bit channels = 8 bytes per pixel.

### Why not just float meters?

You'd have to send 16 bytes per pixel (four float32s) instead of 8 — doubling the bandwidth on the GigE link. At 640×480 × 30 Hz, that's the difference between 75 MB/s and 150 MB/s, which is the difference between "works on GigE" and "doesn't fit on GigE." So the camera sends int16 LSBs and the host scales to meters.

### What about the offsets being -8192?

For the Helios2, raw X and Y values are unsigned (0 to 65535) and span the full sensor field. The center of the sensor maps to raw value 32768. To put the origin at the optical center, the camera reports an offset of -8192 mm... wait, that doesn't quite add up either.

Honestly, the per-axis offset semantics are camera-specific and you should just **trust the camera's reported values and apply the formula** `world = raw * scale + offset`. The camera's calibration encodes everything. If you've done the math right, a flat wall will appear as a flat plane in the point cloud, perpendicular to Z.

---

## 4. ROS 2 sensor data conventions

ROS has established conventions for sensor data. Follow them or downstream tools won't understand your output.

### REP-103: Coordinate frames

- **Robot body** (REP-103): X forward, Y left, Z up. Right-handed.
- **Camera optical** (REP-103): Z forward (into scene), X right, Y down. Right-handed.

Camera frames are rotated 90° relative to robot body frames. This is historical (came from computer vision before it merged with robotics). You'll see TF chains like `base_link` (body) → `camera_link` (body convention, optional) → `camera_optical_frame` (camera convention).

### REP-105: Units

- **Distance**: meters
- **Angle**: radians
- **Time**: seconds

Always. The Helios2 reports millimeters internally; we convert to meters before publishing. If you forget, your point cloud is 1000× too big and RViz won't render it.

### Standard message types

| Sensor data    | Message type                  |
|----------------|-------------------------------|
| 2D image       | `sensor_msgs/Image`           |
| Camera params  | `sensor_msgs/CameraInfo`      |
| Point cloud    | `sensor_msgs/PointCloud2`     |
| LaserScan      | `sensor_msgs/LaserScan`       |
| IMU            | `sensor_msgs/Imu`             |

Use these. Don't invent custom types. Every existing ROS tool consumes these.

### PointCloud2 specifics

```
height × width × point_step = total bytes
```

For an **organized** point cloud (one point per camera pixel, preserving image structure):
- `height` = image rows (e.g., 480)
- `width` = image columns (e.g., 640)
- `point_step` = 16 bytes (4 float32 fields: x, y, z, intensity)
- `is_dense = false` (we have NaN values for invalid pixels)

For an **unorganized** point cloud (just a flat list of points):
- `height = 1`
- `width` = number of points
- `is_dense = true` (no NaNs allowed)

We publish organized clouds because the structure is useful downstream (you can re-extract the depth image, do 2D filters in image space, etc.).

---

## 5. QoS in DDS — the silent killer

ROS 2 uses DDS (Data Distribution Service) for messaging. DDS has very rich Quality of Service settings that can prevent publishers and subscribers from talking to each other.

### The key QoS policies

| Policy        | Values                          | What it controls                        |
|---------------|--------------------------------|------------------------------------------|
| Reliability   | RELIABLE / BEST_EFFORT          | Should DDS retry on packet loss?         |
| Durability    | VOLATILE / TRANSIENT_LOCAL      | Late subscribers get old messages?       |
| History       | KEEP_LAST(N) / KEEP_ALL         | How many messages to buffer              |
| Lifespan      | duration                        | Messages expire after this long          |
| Deadline      | duration                        | Promise of regular publication rate      |

### The QoS compatibility rule

A subscriber will only receive messages from a publisher whose QoS is **at least as strict** as the subscriber's requirements.

| Publisher       | Subscriber wants | Match? |
|----------------|------------------|--------|
| RELIABLE       | RELIABLE         | Yes    |
| RELIABLE       | BEST_EFFORT      | Yes    |
| BEST_EFFORT    | RELIABLE         | **NO** |
| BEST_EFFORT    | BEST_EFFORT      | Yes    |

The driver publishes BEST_EFFORT (correct for sensor data — we don't want retransmits of stale frames). RViz defaults to RELIABLE. They don't match. The topic appears in `ros2 topic list` but the subscriber gets nothing.

**Lesson:** when a topic exists but you see no data, **check QoS first**. The driver's startup log shows its QoS; RViz's display options expose the subscriber's QoS.

### Sensor data QoS profile

The convention for high-frequency sensors:
```cpp
rclcpp::QoS sensor_qos(rclcpp::KeepLast(5));
sensor_qos.best_effort();
sensor_qos.durability_volatile();
```

There's a shortcut in `rclcpp` for this exact profile: `rclcpp::SensorDataQoS()`. We don't use it because it gives KEEP_LAST(5) which is fine, but being explicit makes the intent clear.

---

## Recommended further reading

- **GenICam SFNC**: https://www.emva.org/standards-technology/genicam/ (download the PDF, search for "Scan3d")
- **GigE Vision standard**: requires AIA membership, but plenty of online summaries exist
- **REP-103**: https://www.ros.org/reps/rep-0103.html
- **REP-105**: https://www.ros.org/reps/rep-0105.html
- **ROS 2 QoS overview**: https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html
- **Arena SDK examples**: `/opt/ArenaSDK/.../Examples/Arena/Cpp_Helios_*` — read these, they're the canonical reference for how to talk to a Helios.
- **PCL tutorials**: https://pcl.readthedocs.io/ — once you have a PointCloud2, PCL is how you filter, segment, register, etc.
