include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(
    CAMERA_SRCS
    CameraBackend.cpp
    CameraServer.cpp
    OrbbecDevice.cpp
)

add_library(
    camera
    STATIC
    ${CAMERA_SRCS}
)

target_include_directories(
    camera
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(
    camera
    PUBLIC
    proto
    ob::OrbbecSDK
)
