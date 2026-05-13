#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include "ArenaApi.h"

struct Probe { const char * name; };

static const std::vector<Probe> kFeatures = {
  {"DeviceModelName"}, {"DeviceFirmwareVersion"},
  {"Scan3dOperatingMode"}, {"ExposureTimeSelector"}, {"ExposureTime"},
  {"ExposureAuto"},
  {"Scan3dConfidenceThresholdEnable"}, {"Scan3dConfidenceThresholdMin"},
  {"Scan3dSpatialFilterEnable"},
  {"Scan3dFlyingPixelsRemovalEnable"}, {"Scan3dFlyingPixelsDistanceThreshold"},
  {"Scan3dDistanceFilterEnable"}, {"Scan3dDistanceMin"}, {"Scan3dDistanceMax"},
  {"Scan3dAmplitudeGain"}, {"ConversionGain"},
  {"AcquisitionFrameRate"}, {"AcquisitionFrameRateEnable"},
  {"Scan3dImageAccumulation"},
};

static const char * status_str(GenApi::INode * node) {
  if (!node || !GenApi::IsImplemented(node)) return "NOT IMPLEMENTED";
  if (!GenApi::IsAvailable(node))   return "NOT AVAILABLE";
  if (!GenApi::IsReadable(node))    return "NOT READABLE";
  if (!GenApi::IsWritable(node))    return "READ-ONLY";
  return "WRITABLE";
}

static std::string current_value(GenApi::INode * node) {
  if (!node || !GenApi::IsReadable(node)) return "-";
  try {
    GenApi::CValuePtr v(node);
    if (v.IsValid()) return std::string(v->ToString().c_str());
  } catch (...) {}
  return "-";
}

static std::string range_info(GenApi::INode * node) {
  if (!node) return "";
  try {
    GenApi::CFloatPtr fp(node);
    if (fp.IsValid() && GenApi::IsReadable(fp)) {
      char b[128]; snprintf(b, sizeof(b), "  [range: %.3f .. %.3f]", fp->GetMin(), fp->GetMax());
      return b;
    }
  } catch (...) {}
  try {
    GenApi::CIntegerPtr ip(node);
    if (ip.IsValid() && GenApi::IsReadable(ip)) {
      char b[128]; snprintf(b, sizeof(b), "  [range: %ld .. %ld]", (long)ip->GetMin(), (long)ip->GetMax());
      return b;
    }
  } catch (...) {}
  try {
    GenApi::CEnumerationPtr ep(node);
    if (ep.IsValid() && GenApi::IsReadable(ep)) {
      GenApi::NodeList_t entries;
      ep->GetEntries(entries);
      std::string s = "  [values:";
      for (auto & e : entries) {
        GenApi::CEnumEntryPtr eep(e);
        if (eep.IsValid() && GenApi::IsAvailable(eep)) s += " " + std::string(eep->GetSymbolic().c_str());
      }
      s += "]"; return s;
    }
  } catch (...) {}
  return "";
}

int main() {
  try {
    auto sys = Arena::OpenSystem();
    sys->UpdateDevices(100);
    auto infos = sys->GetDevices();
    if (infos.empty()) { std::cerr << "No devices found\n"; return 1; }
    auto dev = sys->CreateDevice(infos[0]);
    auto nm = dev->GetNodeMap();

    std::cout << "\n=== Helios2 Feature Probe ===\n\n";
    std::cout << std::left << std::setw(42) << "Feature" << std::setw(20) << "Status" << "Value / range\n";
    std::cout << std::string(100, '-') << "\n";

    for (const auto & p : kFeatures) {
      try {
        GenApi::CNodePtr node = nm->GetNode(p.name);
        std::cout << std::left << std::setw(42) << p.name
                  << std::setw(20) << status_str(node)
                  << current_value(node) << range_info(node) << "\n";
      } catch (const std::exception & e) {
        std::cout << std::left << std::setw(42) << p.name
                  << std::setw(20) << "ERROR"
                  << "(" << e.what() << ")\n";
      } catch (...) {
        std::cout << std::left << std::setw(42) << p.name
                  << std::setw(20) << "ERROR"
                  << "(unknown exception)\n";
      }
    }
    sys->DestroyDevice(dev);
    Arena::CloseSystem(sys);
    return 0;
  } catch (GenICam::GenericException & e) {
    std::cerr << "GenICam: " << e.GetDescription() << "\n"; return 1;
  } catch (std::exception & e) {
    std::cerr << "Error: " << e.what() << "\n"; return 1;
  }
}
