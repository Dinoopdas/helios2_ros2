// =============================================================================
//  helios2_node.cpp  (v2 — tuned for accuracy)
//
//  ROS 2 Humble driver for the LUCID Vision Helios2+ ToF camera.
//
//  v2 changes vs v1:
//    - Exposes every tunable Helios feature as a ROS parameter
//    - Defaults tuned for STATIC, CLOSE-RANGE (300-1250mm) ACCURACY
//    - Uses hardware-side Scan3dImageAccumulation (1..32 frames averaged ON SENSOR)
//    - Enables spatial filter and flying-pixel removal by default
//    - Uses High Conversion Gain (HCG) for low-light close-range
//    - Adds optional XYZ ROI cropping (publishes NaN for out-of-ROI pixels)
//    - Adds confidence-based filtering using the intensity channel
//    - Adds explicit GenICam exception catches in capture loop
//    - Logs achieved frame rate
//
//  Publishes:
//    helios2/points    sensor_msgs/PointCloud2  (organized, XYZI, float32, meters)
//    helios2/intensity sensor_msgs/Image        (mono16 intensity / confidence)
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

static constexpr size_t HELIOS_BYTES_PER_PIXEL = 8;  // Coord3D_ABCY16

class Helios2Node : public rclcpp::Node
{
public:
  Helios2Node()
  : Node("helios2_node"),
    running_(false),
    system_(nullptr),
    device_(nullptr)
  {
    // -------- Parameters (with sensible accuracy-mode defaults) --------
    serial_number_       = declare_parameter<std::string>("serial_number", "");
    frame_id_            = declare_parameter<std::string>("frame_id", "helios2_optical_frame");
    pointcloud_topic_    = declare_parameter<std::string>("pointcloud_topic", "helios2/points");
    intensity_topic_     = declare_parameter<std::string>("intensity_topic",  "helios2/intensity");
    publish_intensity_   = declare_parameter<bool>("publish_intensity", true);
    image_timeout_ms_    = declare_parameter<int>("image_timeout_ms", 5000);   // higher for averaging

    // --- Helios-specific tuning ---
    operating_mode_      = declare_parameter<std::string>("operating_mode",   "Distance1250mmSingleFreq");
    exposure_selector_   = declare_parameter<std::string>("exposure_selector","Exp1000Us");
    conversion_gain_     = declare_parameter<std::string>("conversion_gain",  "High");   // HCG for close range
    image_accumulation_  = declare_parameter<int>   ("image_accumulation", 16);          // 1..32; 16 is good middle
    spatial_filter_      = declare_parameter<bool>  ("spatial_filter", true);
    flying_pixels_remove_= declare_parameter<bool>  ("flying_pixels_remove", true);
    confidence_threshold_= declare_parameter<int>   ("confidence_threshold", 500);       // 0..65535
    amplitude_gain_      = declare_parameter<double>("amplitude_gain", 1.0);              // 0..128

    // --- Driver-side filtering ---
    min_intensity_       = declare_parameter<int>("min_intensity", 0);   // drop pixels below
    invalid_as_nan_      = declare_parameter<bool>("invalid_as_nan", true);

    // --- ROI cropping (meters) — defaults pass everything ---
    roi_min_x_ = declare_parameter<double>("roi_min_x", -10.0);
    roi_max_x_ = declare_parameter<double>("roi_max_x",  10.0);
    roi_min_y_ = declare_parameter<double>("roi_min_y", -10.0);
    roi_max_y_ = declare_parameter<double>("roi_max_y",  10.0);
    roi_min_z_ = declare_parameter<double>("roi_min_z",  0.0);
    roi_max_z_ = declare_parameter<double>("roi_max_z",  10.0);

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

    running_ = true;
    capture_thread_ = std::thread(&Helios2Node::captureLoop, this);

    RCLCPP_INFO(get_logger(), "helios2_node v2 up. Publishing on '%s' and '%s'",
                pointcloud_topic_.c_str(), intensity_topic_.c_str());
  }

  ~Helios2Node() override
  {
    running_ = false;
    if (capture_thread_.joinable()) capture_thread_.join();
    shutdownCamera();
  }

private:
  // -------------------------------------------------------------------------
  // Helpers to set GenICam nodes defensively. Many firmwares throw if you
  // set a node that's not currently writable (e.g., depends on mode).
  // -------------------------------------------------------------------------
  template<typename T>
  bool trySetNode(GenApi::INodeMap * nm, const char * name, const T & value, const char * desc = nullptr)
  {
    try {
      Arena::SetNodeValue<T>(nm, name, value);
      RCLCPP_INFO(get_logger(), "  [OK]  %s = %s%s",
                  name, std::to_string(value).c_str(),
                  desc ? (std::string("  -- ") + desc).c_str() : "");
      return true;
    } catch (GenICam::GenericException & e) {
      RCLCPP_WARN(get_logger(), "  [SKIP] %s: %s", name, e.GetDescription());
      return false;
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "  [SKIP] %s: %s", name, e.what());
      return false;
    } catch (...) {
      RCLCPP_WARN(get_logger(), "  [SKIP] %s: unknown error", name);
      return false;
    }
  }

  bool trySetEnum(GenApi::INodeMap * nm, const char * name, const std::string & value, const char * desc = nullptr)
  {
    try {
      Arena::SetNodeValue<GenICam::gcstring>(nm, name, value.c_str());
      RCLCPP_INFO(get_logger(), "  [OK]  %s = %s%s",
                  name, value.c_str(),
                  desc ? (std::string("  -- ") + desc).c_str() : "");
      return true;
    } catch (GenICam::GenericException & e) {
      RCLCPP_WARN(get_logger(), "  [SKIP] %s = '%s': %s", name, value.c_str(), e.GetDescription());
      return false;
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "  [SKIP] %s = '%s': %s", name, value.c_str(), e.what());
      return false;
    } catch (...) {
      RCLCPP_WARN(get_logger(), "  [SKIP] %s: unknown error", name);
      return false;
    }
  }

  // -------------------------------------------------------------------------
  void initCamera()
  {
    system_ = Arena::OpenSystem();
    system_->UpdateDevices(100);
    auto infos = system_->GetDevices();

    if (infos.empty()) {
      throw std::runtime_error("No Arena devices found on the network.");
    }

    Arena::DeviceInfo selected = infos[0];
    if (!serial_number_.empty()) {
      bool found = false;
      for (auto & info : infos) {
        if (std::string(info.SerialNumber().c_str()) == serial_number_) {
          selected = info; found = true; break;
        }
      }
      if (!found) throw std::runtime_error("Camera with serial " + serial_number_ + " not found.");
    }

    RCLCPP_INFO(get_logger(), "Opening device: %s  S/N=%s  IP=%s",
                selected.ModelName().c_str(),
                selected.SerialNumber().c_str(),
                selected.IpAddressStr().c_str());

    device_ = system_->CreateDevice(selected);
    auto nm  = device_->GetNodeMap();
    auto tlm = device_->GetTLStreamNodeMap();

    auto model = Arena::GetNodeValue<GenICam::gcstring>(nm, "DeviceModelName");
    std::string model_s = model.c_str();
    RCLCPP_INFO(get_logger(), "Model: %s", model_s.c_str());

    try {
      auto fw = Arena::GetNodeValue<GenICam::gcstring>(nm, "DeviceFirmwareVersion");
      RCLCPP_INFO(get_logger(), "Firmware: %s", fw.c_str());
    } catch (...) {}

    // ====== Pixel format MUST be set first (other settings depend on it) ======
    RCLCPP_INFO(get_logger(), "Configuring Helios2 for ACCURACY mode:");
    trySetEnum(nm, "PixelFormat", "Coord3D_ABCY16", "3D + intensity");

    // ====== Operating mode (range/precision) ======
    trySetEnum(nm, "Scan3dOperatingMode", operating_mode_,
               "Close-range high-precision mode");

    // ====== Exposure preset ======
    trySetEnum(nm, "ExposureTimeSelector", exposure_selector_,
               "Longest = best SNR");

    // ====== Conversion gain (HCG = better close-range SNR) ======
    trySetEnum(nm, "ConversionGain", conversion_gain_,
               "High Conversion Gain for low-reflectance close-range");

    // ====== Amplitude gain ======
    trySetNode<double>(nm, "Scan3dAmplitudeGain", amplitude_gain_,
                      "Multiplier on intensity output");

    // ====== Hardware temporal averaging — the big SNR win ======
    int accum = std::max(1, std::min(32, image_accumulation_));
    trySetNode<int64_t>(nm, "Scan3dImageAccumulation", accum,
                       "Frames averaged on the sensor (huge SNR boost)");

    // ====== Spatial filter ======
    trySetNode<bool>(nm, "Scan3dSpatialFilterEnable", spatial_filter_,
                    "Smooths noise across neighboring pixels");

    // ====== Flying pixel removal ======
    trySetNode<bool>(nm, "Scan3dFlyingPixelsRemovalEnable", flying_pixels_remove_,
                    "Removes edge-bleed artifacts");

    // ====== Confidence threshold ======
    trySetNode<bool>(nm, "Scan3dConfidenceThresholdEnable", true,
                    "Reject low-confidence pixels");
    trySetNode<int64_t>(nm, "Scan3dConfidenceThresholdMin", confidence_threshold_,
                       "Minimum confidence value (0..65535)");

    // ====== Read back the coordinate scale and offset (essential for decode) ======
    auto read_scale_offset = [&](const char * which, float & scale, float & offset) {
      Arena::SetNodeValue<GenICam::gcstring>(nm, "Scan3dCoordinateSelector", which);
      scale  = static_cast<float>(Arena::GetNodeValue<double>(nm, "Scan3dCoordinateScale"));
      try {
        offset = static_cast<float>(Arena::GetNodeValue<double>(nm, "Scan3dCoordinateOffset"));
      } catch (...) { offset = 0.0f; }
    };
    read_scale_offset("CoordinateA", scale_x_, offset_x_);
    read_scale_offset("CoordinateB", scale_y_, offset_y_);
    read_scale_offset("CoordinateC", scale_z_, offset_z_);

    RCLCPP_INFO(get_logger(),
      "Coord scales (mm/LSB): X=%.4f Y=%.4f Z=%.4f | offsets (mm): X=%.2f Y=%.2f Z=%.2f",
      scale_x_, scale_y_, scale_z_, offset_x_, offset_y_, offset_z_);

    // ====== Report achieved frame rate ======
    try {
      double rate = Arena::GetNodeValue<double>(nm, "AcquisitionFrameRate");
      RCLCPP_INFO(get_logger(), "Achieved frame rate: %.2f Hz (with accumulation=%d)", rate, accum);
    } catch (...) {}

    // ====== Stream settings ======
    Arena::SetNodeValue<bool>(tlm, "StreamAutoNegotiatePacketSize", true);
    Arena::SetNodeValue<bool>(tlm, "StreamPacketResendEnable",      true);

    device_->StartStream();
    RCLCPP_INFO(get_logger(), "Stream started.");
  }

  // -------------------------------------------------------------------------
  void captureLoop()
  {
    RCLCPP_INFO(get_logger(), "Capture thread started.");
    auto last_log = now();
    int frame_count = 0;

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
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "GetImage unknown exception");
        std::this_thread::sleep_for(100ms);
        continue;
      }

      if (!img) continue;

      if (img->IsIncomplete()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Incomplete frame received.");
        device_->RequeueBuffer(img);
        continue;
      }

      try {
        publishFrame(img);
        frame_count++;
      } catch (GenICam::GenericException & e) {
        RCLCPP_ERROR(get_logger(), "publishFrame GenICam: %s", e.GetDescription());
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "publishFrame error: %s", e.what());
      } catch (...) {
        RCLCPP_ERROR(get_logger(), "publishFrame unknown exception");
      }

      device_->RequeueBuffer(img);

      // Periodic rate report (every 5 seconds)
      auto t = now();
      double dt = (t - last_log).seconds();
      if (dt >= 5.0) {
        RCLCPP_INFO(get_logger(), "Publishing at %.2f Hz", frame_count / dt);
        last_log = t;
        frame_count = 0;
      }
    }
    RCLCPP_INFO(get_logger(), "Capture thread exiting.");
  }

  // -------------------------------------------------------------------------
  void publishFrame(Arena::IImage * img)
  {
    const size_t width  = img->GetWidth();
    const size_t height = img->GetHeight();
    const size_t npx    = width * height;
    const uint8_t * src = img->GetData();

    if ((img->GetBitsPerPixel() / 8) != HELIOS_BYTES_PER_PIXEL) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Wrong bytes/pixel %zu (expected %zu). Wrong PixelFormat?",
        img->GetBitsPerPixel() / 8, HELIOS_BYTES_PER_PIXEL);
      return;
    }

    const rclcpp::Time stamp = now();

    // ---- PointCloud2 ----
    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    cloud->header.stamp    = stamp;
    cloud->header.frame_id = frame_id_;
    cloud->height          = static_cast<uint32_t>(height);
    cloud->width           = static_cast<uint32_t>(width);
    cloud->is_bigendian    = false;
    cloud->is_dense        = false;

    cloud->fields.resize(4);
    auto set_field = [](sensor_msgs::msg::PointField & f, const std::string & name, uint32_t offset) {
      f.name = name; f.offset = offset;
      f.datatype = sensor_msgs::msg::PointField::FLOAT32; f.count = 1;
    };
    set_field(cloud->fields[0], "x",         0);
    set_field(cloud->fields[1], "y",         4);
    set_field(cloud->fields[2], "z",         8);
    set_field(cloud->fields[3], "intensity", 12);

    cloud->point_step = 16;
    cloud->row_step   = cloud->point_step * static_cast<uint32_t>(width);
    cloud->data.resize(cloud->row_step * height);

    float * dst = reinterpret_cast<float *>(cloud->data.data());

    constexpr float MM_TO_M = 0.001f;
    const float kx = scale_x_ * MM_TO_M, ox = offset_x_ * MM_TO_M;
    const float ky = scale_y_ * MM_TO_M, oy = offset_y_ * MM_TO_M;
    const float kz = scale_z_ * MM_TO_M, oz = offset_z_ * MM_TO_M;

    // ROI in meters (cast once)
    const float rx0 = static_cast<float>(roi_min_x_), rx1 = static_cast<float>(roi_max_x_);
    const float ry0 = static_cast<float>(roi_min_y_), ry1 = static_cast<float>(roi_max_y_);
    const float rz0 = static_cast<float>(roi_min_z_), rz1 = static_cast<float>(roi_max_z_);

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

    const uint16_t min_i = static_cast<uint16_t>(std::max(0, std::min(65535, min_intensity_)));
    const float NaN = std::numeric_limits<float>::quiet_NaN();

    const uint8_t * p = src;
    for (size_t i = 0; i < npx; ++i) {
      const int16_t  a = *reinterpret_cast<const int16_t  *>(p + 0);
      const int16_t  b = *reinterpret_cast<const int16_t  *>(p + 2);
      const int16_t  c = *reinterpret_cast<const int16_t  *>(p + 4);
      const uint16_t y = *reinterpret_cast<const uint16_t *>(p + 6);

      const bool zero_coord = (a == 0 && b == 0 && c == 0);
      const bool low_intens = (y < min_i);

      if ((zero_coord && invalid_as_nan_) || low_intens) {
        dst[0] = dst[1] = dst[2] = NaN;
      } else {
        float xm = static_cast<float>(a) * kx + ox;
        float ym = static_cast<float>(b) * ky + oy;
        float zm = static_cast<float>(c) * kz + oz;

        // ROI test
        if (xm < rx0 || xm > rx1 || ym < ry0 || ym > ry1 || zm < rz0 || zm > rz1) {
          dst[0] = dst[1] = dst[2] = NaN;
        } else {
          dst[0] = xm; dst[1] = ym; dst[2] = zm;
        }
      }
      dst[3] = static_cast<float>(y);

      if (inten_dst) *inten_dst++ = y;
      p   += HELIOS_BYTES_PER_PIXEL;
      dst += 4;
    }

    pc_pub_->publish(std::move(cloud));
    if (img_msg) img_pub_->publish(std::move(img_msg));
  }

  // -------------------------------------------------------------------------
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
  std::string serial_number_, frame_id_, pointcloud_topic_, intensity_topic_;
  bool publish_intensity_;
  int image_timeout_ms_;

  std::string operating_mode_, exposure_selector_, conversion_gain_;
  int image_accumulation_;
  bool spatial_filter_, flying_pixels_remove_;
  int confidence_threshold_;
  double amplitude_gain_;

  int min_intensity_;
  bool invalid_as_nan_;
  double roi_min_x_, roi_max_x_, roi_min_y_, roi_max_y_, roi_min_z_, roi_max_z_;

  // ---- Runtime state ----
  std::atomic<bool> running_;
  std::thread       capture_thread_;
  Arena::ISystem *  system_;
  Arena::IDevice *  device_;
  float scale_x_{0}, scale_y_{0}, scale_z_{0};
  float offset_x_{0}, offset_y_{0}, offset_z_{0};

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
