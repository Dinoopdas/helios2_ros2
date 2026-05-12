# Troubleshooting

Every error message we hit during initial setup, what it actually means, and how to fix it.
Organized by where the failure shows up.

---

## Network problems

### `ping 192.168.100.40` fails with "Destination Host Unreachable"

**Meaning:** Your laptop is trying to ARP for the camera (Layer 2) and getting no reply.

**Causes & fixes:**

1. **MTU mismatch.** If you set jumbo frames (MTU 9000) on the host but the camera/cable doesn't support them, every packet gets silently dropped.
   ```bash
   sudo ip link set enp0s31f6 mtu 1500
   ```

2. **Wrong subnet.** Confirm both your laptop and the camera are in 192.168.100.0/24.
   ```bash
   ip addr show enp0s31f6
   # Should show 192.168.100.X/24
   ```

3. **Physical link not up.**
   ```bash
   cat /sys/class/net/enp0s31f6/carrier   # should be 1
   ip link show enp0s31f6                  # should say "state UP" and "LOWER_UP"
   ```
   If carrier is 0, check both ends of the cable, cable LEDs, and confirm the camera has power.

4. **Cable is bad.** Swap with a known-good cable.

### `ping` works but Arena SDK can't find the camera

**Meaning:** L3 is fine but GigE Vision discovery (UDP broadcast on port 3956) is blocked.

**Causes & fixes:**

1. **Firewall is blocking UDP.**
   ```bash
   sudo ufw status
   # If active:
   sudo ufw allow in on enp0s31f6
   ```

2. **Stuck GenICam session from a previous crash.** GenICam reserves the camera on connection. If the previous process crashed, the reservation can persist for 30+ seconds.
   - Wait 60 seconds and retry.
   - If that doesn't work, **power-cycle the camera** (unplug power for 5 seconds).

3. **WiFi is on a conflicting subnet.** If your WiFi is also handing out IPs in 192.168.100.x, the SDK enumerates the camera twice and confuses itself.
   ```bash
   nmcli device disconnect <wifi-interface>
   ```

### Camera reachable on WiFi but not on direct Ethernet

**Meaning:** The router is doing internal NAT/bridging that exposes the camera on WiFi, but the wired side is on a different subnet.

**Fix:** Don't use the router. Direct-connect the camera to the laptop's Ethernet. See [SETUP.md](SETUP.md#step-3-set-up-the-camera-network).

### NetworkManager keeps wiping my static IP

**Meaning:** You set the IP with `sudo ip addr add ...` but NetworkManager doesn't know about it, so it overrides.

**Fix:** Create a real NetworkManager profile:
```bash
sudo nmcli connection add type ethernet ifname enp0s31f6 con-name helios2-cam \
    ipv4.method manual \
    ipv4.addresses 192.168.100.1/24 \
    ipv4.never-default yes \
    autoconnect yes
sudo nmcli connection up helios2-cam
```

### Internet stops working when I plug in the camera

**Meaning:** The camera's NIC grabbed a default route.

**Fix:** The `ipv4.never-default yes` flag in the NM profile (above) prevents this. If you already have a profile without it:
```bash
sudo nmcli connection modify helios2-cam ipv4.never-default yes
sudo nmcli connection up helios2-cam
```

### USB Ethernet has an IP but no internet

**Meaning:** DHCP gave an address but no default gateway, or got cached wrong.

**Fix:** Force a DHCP renew:
```bash
sudo dhclient -r <interface>
sudo dhclient -v <interface>
# Check it got a gateway
ip route | grep default
```

---

## Build problems

### `undefined reference to 'cv::projectPoints(...)'`

**Meaning:** Arena SDK's `libarena.so` calls into OpenCV's calib3d but doesn't link it.

**Fix:** In your `CMakeLists.txt`:
```cmake
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)
target_link_libraries(your_target ${arena_sdk_LIBRARIES} ${OpenCV_LIBS} opencv_calib3d)
```

This is annoying but consistent. **Every** Linux executable that links Arena SDK needs OpenCV explicitly.

### CMake says `Could not find arena_sdk`

**Meaning:** The `Findarena_sdk.cmake` module isn't in the cmake search path, or `/etc/ld.so.conf.d/Arena_SDK.conf` doesn't exist.

**Fixes:**

1. **Confirm Arena SDK is installed:**
   ```bash
   ls /etc/ld.so.conf.d/Arena_SDK.conf
   # If missing, re-run the installer from the Arena SDK directory:
   cd /path/to/ArenaSDK_Linux_x64
   sudo sh Arena_SDK_Linux_x64.conf
   ```

2. **Confirm the find module is in your package's cmake/:**
   ```bash
   ls ~/cobot_ws/src/helios2_node/cmake/Findarena_sdk.cmake
   # If missing, copy from the source repo or from arena_camera_node
   ```

3. **Confirm CMakeLists.txt adds the path:**
   ```cmake
   set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")
   ```

### Build error about `True` not being defined (in arena_camera_node)

**Meaning:** The arena_camera_node source has `True` (Python style) instead of `true` (C++) on lines 429-430 of ArenaCameraNode.cpp.

**Fix:** Edit ArenaCameraNode.cpp, change `True` to `true`. Already done in our checked-out copy.

---

## Runtime problems

### Driver says "No Arena devices found on the network"

See [Network problems → ping works but Arena SDK can't find the camera](#ping-works-but-arena-sdk-cant-find-the-camera) above.

Most common cause: previous process crashed and the GenICam session is still reserved. Wait 60 seconds, or power-cycle the camera.

### `terminate called after throwing 'GenICam::TimeoutException'`

**Meaning:** A blocking call (usually `GetImage()`) timed out and the exception escaped a catch block, crashing the node.

**Causes:**

1. **Packet loss on the GigE link** — usually a flaky cable.
2. **Receive buffer overflow** — kernel UDP buffer too small for the bitrate.
3. **Camera firmware glitch** — rare but happens with ToF cameras under sustained load.

**Fixes:**

1. **Bump kernel UDP buffers (one-time):**
   ```bash
   sudo sysctl -w net.core.rmem_max=33554432
   sudo sysctl -w net.core.rmem_default=33554432
   ```

2. **Check the cable.** If the error correlates with moving the laptop, the cable is mechanically failing. Swap it.

3. **Watch dmesg for link drops:**
   ```bash
   sudo dmesg -w | grep -i 'link\|enp0s31f6'
   ```
   Any "link down" event during streaming = physical issue.

4. **Catch the exception cleanly in code.** The driver's capture loop should catch `GenICam::GenericException` (not just `std::exception`) and keep running. The current version of `helios2_node.cpp` does this.

### "Incomplete frame received" warnings spam the console

**Meaning:** Packets are being lost. Some frames arrive partial and get rejected.

**Common causes (in order):**

1. **Receive buffer too small.** Bump `net.core.rmem_max` to 32 MB.
2. **Cable issue.** Wiggle the cable at both connectors — if the rate of incomplete frames changes, the cable is bad.
3. **NIC interrupt overload.** Try setting an explicit IRQ affinity:
   ```bash
   # Find which IRQ is the NIC
   grep enp0s31f6 /proc/interrupts
   # Pin to CPU0 (example)
   echo 1 | sudo tee /proc/irq/<IRQ_NUM>/smp_affinity
   ```
4. **Other heavy traffic on the NIC.** Make sure no other process is using this interface.

### RViz shows "No tf data" or the cloud doesn't appear

**Meaning:** Either the Fixed Frame doesn't have a transform to the cloud's frame, or QoS doesn't match.

**Fixes:**

1. **Publish a static transform:**
   ```bash
   ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world helios2_optical_frame
   ```

2. **Set Fixed Frame to `world` or `helios2_optical_frame`** in RViz's Global Options.

3. **Set the PointCloud2 display's Reliability Policy to Best Effort.** This is the #1 cause of "the topic exists but I see nothing." The driver publishes BEST_EFFORT; RViz defaults to RELIABLE; they don't match; no data flows.

### Cloud appears but everything is at the origin or shifted by ~2 meters

**Meaning:** The per-axis offset (or scale) wasn't applied correctly.

**Fix:** Check the driver's startup log for "Coord scales (mm/LSB)" — confirm the offsets are non-zero for X and Y. Our code applies them as `raw * scale + offset`. If you modified the code, double-check that.

### Cloud rate is much lower than 30 Hz

**Meaning:** Either the camera's frame rate is set low, or there's a CPU/network bottleneck.

**Diagnostics:**
```bash
# What's the actual publish rate?
ros2 topic hz /helios2/points

# Is the driver CPU-bound?
top -p $(pgrep -f helios2_node)

# Is there packet loss?
# (Look for "Incomplete frame" warnings in the driver console)
```

If CPU is high (>80% on a single core), the per-pixel decode loop might benefit from SIMD optimization — but at 320×240 = 76800 pixels at 30 Hz, this should be well under 5% on a modern laptop.

---

## QoS / DDS problems

### "incompatible QoS" warnings on the driver console

**Meaning:** A subscriber (usually RViz) is asking for stricter QoS than the publisher offers.

**Fix:** Change the subscriber's QoS, not the publisher's. The publisher's BEST_EFFORT QoS is correct for sensor data — don't change it to RELIABLE.

In RViz: PointCloud2 display → Topic → Reliability Policy → Best Effort.

In code: when subscribing,
```cpp
rclcpp::QoS qos(5);
qos.best_effort();
auto sub = create_subscription<sensor_msgs::msg::PointCloud2>("/helios2/points", qos, ...);
```

---

## Debugging toolbox

### Confirm the camera is alive at every layer

```bash
# L1 (physical): is the cable carrier present?
cat /sys/class/net/enp0s31f6/carrier

# L2 (data link): is the host's interface up?
ip link show enp0s31f6

# L3 (network): can we reach the camera IP?
ping -c 3 192.168.100.40

# L4+ (GigE Vision): does the SDK enumerate it?
# Run any Arena SDK example, or our driver. If it sees the camera, GigE Vision is fine.
```

### Sniff what the camera is actually sending

```bash
# Watch all traffic on the camera interface
sudo tcpdump -i enp0s31f6 -nn

# Just GigE Vision discovery (UDP 3956)
sudo tcpdump -i enp0s31f6 -nn udp port 3956

# Just streaming data from the camera (UDP from camera IP)
sudo tcpdump -i enp0s31f6 -nn src 192.168.100.40
```

A working camera streaming at 30 Hz will produce a lot of packets — you'll see continuous output.

### Check kernel-level packet drops

```bash
# Per-interface stats
ip -s link show enp0s31f6
# Watch the "dropped" and "missed" columns

# Network-wide stats
netstat -su | grep -i drop
```

If "dropped" climbs while streaming, the kernel can't keep up — bump buffers.

### Run an Arena SDK example as a sanity check

If our driver fails but you're not sure if it's the driver or the camera:

```bash
cd /opt/ArenaSDK/ArenaSDK_Linux_x64/Examples/Arena/Cpp_Helios_HeatMap
# Build the example per its instructions
# Run it. If it captures one frame and saves a heatmap, the camera + SDK + network are all fine.
```

If the example works and our driver doesn't, the bug is in our driver. If neither works, the bug is below the driver (network, SDK install, camera).
