set(OrbbecSDK_DIR ${CMAKE_SOURCE_DIR}/3rdparty/OrbbecSDK)

find_package(absl REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Threads REQUIRED)

if(CAMERA_TYPE_INT EQUAL 0)
    find_package(OrbbecSDK REQUIRED)
endif()

add_subdirectory(proto)

if(ARMS_TYPE_INT GREATER_EQUAL 0)
    add_subdirectory(arms)
endif()
if(CAMERA_TYPE_INT GREATER_EQUAL 0)
    add_subdirectory(camera)
endif()

# Add executable target
add_executable(
    XLRobotBackend
    main.cpp
)

target_compile_definitions(
    XLRobotBackend
    PRIVATE
    ARMS_TYPE=${ARMS_TYPE_INT}
    CAMERA_TYPE=${CAMERA_TYPE_INT}
)

# Link OrbbecSDK and other necessary libraries
target_link_libraries(
    XLRobotBackend
    PRIVATE
    proto
)

if(ARMS_TYPE_INT GREATER_EQUAL 0)
    target_link_libraries(
        XLRobotBackend
        PRIVATE
        arms
    )
endif()

if(CAMERA_TYPE_INT GREATER_EQUAL 0)
    target_link_libraries(
        XLRobotBackend
        PRIVATE
        camera
    )
endif()
