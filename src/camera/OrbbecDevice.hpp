#pragma once

#include "config.hpp"
#include <chrono>
#include <libobsensor/ObSensor.hpp>
#include <memory>
#include <optional>
#include <string>

namespace camera {

class OrbbecDevice {
  public:
    OrbbecDevice(std::shared_ptr<ob::Device> device);
    ~OrbbecDevice();

    const std::string &getSerialNumber() const;
    std::chrono::system_clock::time_point getCreateTime() const;
    std::string getCreateTimeString() const;
    std::optional<std::pair<std::unique_ptr<Image>, std::unique_ptr<Image>>>
    getOneFrame();

  private:
    std::shared_ptr<ob::Device> device_;
    std::shared_ptr<ob::Config> config_;
    std::unique_ptr<ob::Pipeline> pipeline_;
    std::string serialNumber_;
    std::chrono::system_clock::time_point createTime_;
    bool depth_enabled_;
};

} // namespace camera
