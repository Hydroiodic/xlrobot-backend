#pragma once

#include <cstdint>

namespace camera {

constexpr int IMAGE_WIDTH = 640;
constexpr int IMAGE_HEIGHT = 480;
constexpr int IMAGE_CHANNELS = 3;
constexpr int IMAGE_SIZE = IMAGE_WIDTH * IMAGE_HEIGHT * IMAGE_CHANNELS;
constexpr int STREAM_FPS = 30;

struct ImageHeader {
    uint32_t width;
    uint32_t height;
    uint32_t channels;
};

struct Image {
    uint8_t data[IMAGE_SIZE];
};

constexpr ImageHeader DEFAULT_IMAGE_HEADER = ImageHeader{
    .width = static_cast<uint32_t>(IMAGE_WIDTH),
    .height = static_cast<uint32_t>(IMAGE_HEIGHT),
    .channels = static_cast<uint32_t>(IMAGE_CHANNELS),
};

} // namespace camera
