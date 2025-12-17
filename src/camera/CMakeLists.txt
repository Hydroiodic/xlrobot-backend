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
)

if(CAMERA_TYPE_INT EQUAL 0)
    target_link_libraries(
        camera
        PUBLIC
        OrbbecSDK::OrbbecSDK
    )

    install(
        FILES ${OrbbecSDK_DLL_FILES}
        DESTINATION lib64
    )
endif()
