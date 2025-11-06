#pragma once

#include "CameraBackend.hpp"
#include "camera.grpc.pb.h"
#include <grpcpp/grpcpp.h>

namespace camera {

class CameraServer final : public CameraService::Service {
  private:
    CameraBackend cameraBackend;

  public:
    CameraServer();
    ~CameraServer();

    // Get list of connected device serial numbers
    grpc::Status GetConnectedDeviceSerialNumbers(
        grpc::ServerContext *context, const camera::EmptyRequest *request,
        camera::DeviceListResponse *response) override;

    // Get image from specified camera
    grpc::Status GetImage(grpc::ServerContext *context,
                          const camera::GetImageRequest *request,
                          camera::ImageResponse *response) override;
};

} // namespace camera
