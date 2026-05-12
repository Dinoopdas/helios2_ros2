// =============================================================================
//  helios2_node.cpp
//  ROS 2 Humble driver for LUCID Vision Helios2 ToF camera.
//  Configures the camera for Coord3D_ABCY16 output and publishes:
//    - sensor_msgs/PointCloud2 (organized, XYZI, float32, meters)
//    - sensor_msgs/Image       (mono16 intensity)
//
//  Built on top of LUCID's Arena SDK (C++).
//  Reference: /opt/ArenaSDK/.../Examples/Arena/Cpp_Helios_HeatMap
// =============================================================================

#include <atomic>
#include <chrono>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include "ArenaApi.h"

using namespace std::chrono_literals;
namespace senc = sensor_msgs::image_encodings;

// -----------------------------------------------------------------------------
// Per-pixel layout for Coord3D_ABCY16:
//   bytes 0..1 : A (X)   int16_t
//   bytes 2..3 : B (Y)   int16_t
//   bytes 4..5 : C (Z)   int16_t   <- depth, multiply by scale_z to get mm
//   bytes 6..7 : Y       uint16_t  <- intensity
//   total      : 8 bytes per pixel
// -----------------------------------------------------------------------------
static constexpr size_t HELIOS_BYTES_PER_PIXEL = 8;

class Helios2Node : public rclcpp::Node
{
public:
  Helios2Node()
  : Node("helios2_node"),
    running_(false),
    system_(nullptr),
    device_(nullptr)
  {
    // -------- Parameters --------
    serial_number_       = declare_parameter<std::string>("serial_number", "");
    frame_id_            = declare_parameter<std::string>("frame_id", "helios2_optical_frame");
    operating_mode_      = declare_parameter<std::string>("operating_mode", "Distance3000mmSingleFreq");
    exposure_selector_   = declare_parameter<std::string>("exposure_time_selector", "Exp1000Us");
    publish_intensity_   = declare_parameter<bool>("publish_intensity", true);
    invalid_as_nan_      = declare_parameter<bool>("invalid_as_nan", true);
    image_timeout_ms_    = declare_parameter<int>("image_timeout_ms", 2000);

    pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "helios2/points");
    intensity_topic_  = declare_parameter<std::string>("intensity_topic",  "helios2/intensity");

    // -------- QoS: sensor data --------
    rclcpp::QoS sensor_qos(rclcpp::KeepLast(5));
    sensor_qos.best_effort().durability_volatile();

    pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_, sensor_qos);
    if (publish_intensity_) {
      img_pub_ = create_publisher<sensor_msgs::msg::Image>(intensity_topic_, sensor_qos);
    }

    // -------- Init camera --------
    try {
      initCamera();
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "Camera init failed: %s", e.what());
      throw;
    }

    // -------- Spawn capture thread --------
    running_ = true;
    capture_thread_ = std::thread(&Helios2Node::captureLoop, this);

    RCLCPP_INFO(get_logger(), "helios2_node up. Streaming on '%s'", pointcloud_topic_.c_str());
  }

  ~Helios2Node() override
  {
    running_ = false;
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    shutdownCamera();
  }

private:
  // ---------------------------------------------------------------------------
  // Camera setup — locate device, set pixel format, read coordinate scales.
  // ---------------------------------------------------------------------------
  void initCamera()
  {
    system_ = Arena::OpenSystem();
    system_->UpdateDevices(100);
    auto infos = system_->GetDevices();

    if (infos.empty()) {
      throw std::runtime_error("No Arena devices found on the network.");
    }

    // Select by serial if provided, else first device
    Arena::DeviceInfo selected = infos[0];
    if (!serial_number_.empty()) {
      bool found = false;
      for (auto & info : infos) {
        if (std::string(info.SerialNumber().c_str()) == serial_number_) {
          selected = info;
          found = true;
          break;
        }
      }
      if (!found) {
        throw std::runtime_error("Camera with serial " + serial_number_ + " not found.");
      }
    }

    RCLCPP_INFO(get_logger(), "Opening device: %s  S/N=%s  IP=%s",
                selected.ModelName().c_str(),
                selected.SerialNumber().c_str(),
                selected.IpAddressStr().c_str());

    device_ = system_->CreateDevice(selected);
    auto nm  = device_->GetNodeMap();
    auto tlm = device_->GetTLStreamNodeMap();

    // Confirm this is actually a Helios (model name starts with HLT or HT)
    auto model = Arena::GetNodeValue<GenICam::gcstring>(nm, "DeviceModelName");
    std::string model_s = model.c_str();
    if (model_s.rfind("HLT", 0) != 0 && model_s.rfind("HT", 0) != 0) {
      RCLCPP_WARN(get_logger(),
        "Device model '%s' does not look like a Helios. 3D config may fail.", model_s.c_str());
    }

    // PixelFormat -> Coord3D_ABCY16
    RCLCPP_INFO(get_logger(), "Setting PixelFormat = Coord3D_ABCY16");
    Arena::SetNodeValue<GenICam::gcstring>(nm, "PixelFormat", "Coord3D_ABCY16");

    // Operating mode (range / frequency)
    try {
      RCLCPP_INFO(get_logger(), "Setting Scan3dOperatingMode = %s", operating_mode_.c_str());
      Arena::SetNodeValue<GenICam::gcstring>(nm, "Scan3dOperatingMode", operating_mode_.c_str());
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(),
        "Could not set operating mode '%s': %s. Continuing with current camera value.",
        operating_mode_.c_str(), e.what());
    }

    // Exposure selector (preset)
    try {
      Arena::SetNodeValue<GenICam::gcstring>(nm, "ExposureTimeSelector", exposure_selector_.c_str());
      RCLCPP_INFO(get_logger(), "ExposureTimeSelector = %s", exposure_selector_.c_str());
    } catch (...) {
      RCLCPP_WARN(get_logger(), "ExposureTimeSelector '%s' not accepted, leaving default.",
                  exposure_selector_.c_str());
    }

    // Read coordinate scales & offsets (per axis A, B, C)
    auto read_scale_offset = [&](const char * which, float & scale, float & offset) {
      Arena::SetNodeValue<GenICam::gcstring>(nm, "Scan3dCoordinateSelector", which);
      scale  = static_cast<float>(Arena::GetNodeValue<double>(nm, "Scan3dCoordinateScale"));
      try {
        offset = static_cast<float>(Arena::GetNodeValue<double>(nm, "Scan3dCoordinateOffset"));
      } catch (...) {
        offset = 0.0f;
      }
    };

    read_scale_offset("CoordinateA", scale_x_, offset_x_);
    read_scale_offset("CoordinateB", scale_y_, offset_y_);
    read_scale_offset("CoordinateC", scale_z_, offset_z_);

    RCLCPP_INFO(get_logger(),
      "Coord scales (mm/LSB): X=%.6f Y=%.6f Z=%.6f | offsets (mm): X=%.3f Y=%.3f Z=%.3f",
      scale_x_, scale_y_, scale_z_, offset_x_, offset_y_, offset_z_);

    // Stream settings — critical for GigE Vision reliability
    Arena::SetNodeValue<bool>(tlm, "StreamAutoNegotiatePacketSize", true);
    Arena::SetNodeValue<bool>(tlm, "StreamPacketResendEnable",      true);

    // Start streaming
    device_->StartStream();
    RCLCPP_INFO(get_logger(), "Stream started.");
  }

  // ---------------------------------------------------------------------------
  // Capture loop — runs in its own thread. GetImage() blocks until a frame or
  // timeout, which is why we don't want this on the rclcpp executor.
  // ---------------------------------------------------------------------------
  void captureLoop()
  {
    RCLCPP_INFO(get_logger(), "Capture thread started.");

    while (running_ && rclcpp::ok()) {
      Arena::IImage * img = nullptr;
     try {
        img = device_->GetImage(static_cast<uint64_t>(image_timeout_ms_));
      } catch (GenICam::GenericException & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "GetImage GenICam error: %s", e.GetDescription());
        std::this_thread::sleep_for(100ms);
        continue;
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "GetImage error: %s", e.what());
        std::this_thread::sleep_for(100ms);
        continue;
      } catch (...) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "GetImage threw unknown exception");
        std::this_thread::sleep_for(100ms);
        continue;
      }

      if (img == nullptr) {
        continue;
      }

      if (img->IsIncomplete()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Incomplete frame received. Check StreamAutoNegotiatePacketSize, MTU, and link quality.");
        device_->RequeueBuffer(img);
        continue;
      }

      try {
        publishFrame(img);
      } catch (GenICam::GenericException & e) {
        RCLCPP_ERROR(get_logger(), "publishFrame GenICam: %s", e.GetDescription());
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "publishFrame error: %s", e.what());
      } catch (...) {
        RCLCPP_ERROR(get_logger(), "publishFrame unknown exception");
      }

      device_->RequeueBuffer(img);
    }

    RCLCPP_INFO(get_logger(), "Capture thread exiting.");
  }

  // ---------------------------------------------------------------------------
  // Convert one Arena frame -> PointCloud2 (+ optional intensity Image).
  // ---------------------------------------------------------------------------
  void publishFrame(Arena::IImage * img)
  {
    const size_t width  = img->GetWidth();
    const size_t height = img->GetHeight();
    const size_t npx    = width * height;
    const uint8_t * src = img->GetData();

    // Sanity check: 8 bytes/pixel for Coord3D_ABCY16
    const size_t src_bpp_bytes = img->GetBitsPerPixel() / 8;
    if (src_bpp_bytes != HELIOS_BYTES_PER_PIXEL) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Unexpected bytes/pixel: %zu (want %zu). Wrong PixelFormat?",
        src_bpp_bytes, HELIOS_BYTES_PER_PIXEL);
      return;
    }

    const rclcpp::Time stamp = now();

    // ---- Build PointCloud2 (organized: height=image_h, width=image_w) ----
    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    cloud->header.stamp    = stamp;
    cloud->header.frame_id = frame_id_;
    cloud->height          = static_cast<uint32_t>(height);
    cloud->width           = static_cast<uint32_t>(width);
    cloud->is_bigendian    = false;
    cloud->is_dense        = false;  // we may emit NaNs for invalid pixels

    // Fields: x, y, z, intensity — all float32, tightly packed (16 bytes/point)
    cloud->fields.resize(4);
    auto set_field = [](sensor_msgs::msg::PointField & f,
                        const std::string & name, uint32_t offset)
    {
      f.name     = name;
      f.offset   = offset;
      f.datatype = sensor_msgs::msg::PointField::FLOAT32;
      f.count    = 1;
    };
    set_field(cloud->fields[0], "x",         0);
    set_field(cloud->fields[1], "y",         4);
    set_field(cloud->fields[2], "z",         8);
    set_field(cloud->fields[3], "intensity", 12);

    cloud->point_step = 16;
    cloud->row_step   = cloud->point_step * static_cast<uint32_t>(width);
    cloud->data.resize(cloud->row_step * height);

    float * dst = reinterpret_cast<float *>(cloud->data.data());

    // Scale factor: mm -> m (ROS REP-103 uses meters)
    constexpr float MM_TO_M = 0.001f;
    const float kx = scale_x_ * MM_TO_M;
    const float ky = scale_y_ * MM_TO_M;
    const float kz = scale_z_ * MM_TO_M;
    const float ox = offset_x_ * MM_TO_M;
    const float oy = offset_y_ * MM_TO_M;
    const float oz = offset_z_ * MM_TO_M;

    // Optional: intensity image buffer (mono16)
    std::unique_ptr<sensor_msgs::msg::Image> img_msg;
    uint16_t * inten_dst = nullptr;
    if (publish_intensity_ && img_pub_) {
      img_msg = std::make_unique<sensor_msgs::msg::Image>();
      img_msg->header.stamp    = stamp;
      img_msg->header.frame_id = frame_id_;
      img_msg->height          = static_cast<uint32_t>(height);
      img_msg->width           = static_cast<uint32_t>(width);
      img_msg->encoding        = senc::MONO16;
      img_msg->is_bigendian    = 0;
      img_msg->step            = static_cast<uint32_t>(width * sizeof(uint16_t));
      img_msg->data.resize(img_msg->step * height);
      inten_dst = reinterpret_cast<uint16_t *>(img_msg->data.data());
    }

    // -------- Unpack ABCY pixels --------
    const uint8_t * p = src;
    for (size_t i = 0; i < npx; ++i) {
      const int16_t  a = *reinterpret_cast<const int16_t  *>(p + 0);
      const int16_t  b = *reinterpret_cast<const int16_t  *>(p + 2);
      const int16_t  c = *reinterpret_cast<const int16_t  *>(p + 4);
      const uint16_t y = *reinterpret_cast<const uint16_t *>(p + 6);

      // Invalid-pixel heuristic: Helios marks no-return as (0,0,0) for the
      // coordinate channels. The HeatMap example's filterPoints=true does
      // the same. We surface invalids as NaN so RViz / PCL filter them.
      const bool invalid = (a == 0 && b == 0 && c == 0);

      if (invalid && invalid_as_nan_) {
        dst[0] = std::numeric_limits<float>::quiet_NaN();
        dst[1] = std::numeric_limits<float>::quiet_NaN();
        dst[2] = std::numeric_limits<float>::quiet_NaN();
      } else {
        dst[0] = static_cast<float>(a) * kx + ox;
        dst[1] = static_cast<float>(b) * ky + oy;
        dst[2] = static_cast<float>(c) * kz + oz;
      }
      dst[3] = static_cast<float>(y);  // intensity as float for PointCloud2

      if (inten_dst) {
        *inten_dst++ = y;
      }

      p   += HELIOS_BYTES_PER_PIXEL;
      dst += 4;
    }

    pc_pub_->publish(std::move(cloud));
    if (img_msg) {
      img_pub_->publish(std::move(img_msg));
    }
  }

  // ---------------------------------------------------------------------------
  // Tear down.
  // ---------------------------------------------------------------------------
  void shutdownCamera()
  {
    try {
      if (device_) {
        device_->StopStream();
        system_->DestroyDevice(device_);
        device_ = nullptr;
      }
      if (system_) {
        Arena::CloseSystem(system_);
        system_ = nullptr;
      }
      RCLCPP_INFO(get_logger(), "Camera shut down cleanly.");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Shutdown error: %s", e.what());
    }
  }

  // ---- Parameters ----
  std::string serial_number_;
  std::string frame_id_;
  std::string operating_mode_;
  std::string exposure_selector_;
  std::string pointcloud_topic_;
  std::string intensity_topic_;
  bool        publish_intensity_;
  bool        invalid_as_nan_;
  int         image_timeout_ms_;

  // ---- Camera state ----
  std::atomic<bool>  running_;
  std::thread        capture_thread_;
  Arena::ISystem *   system_;
  Arena::IDevice *   device_;

  float scale_x_{0.0f}, scale_y_{0.0f}, scale_z_{0.0f};
  float offset_x_{0.0f}, offset_y_{0.0f}, offset_z_{0.0f};

  // ---- ROS pubs ----
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr       img_pub_;
};

// =============================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<Helios2Node>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    fprintf(stderr, "Fatal: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
