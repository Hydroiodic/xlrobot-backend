set(OrbbecSDK_DIR ${CMAKE_SOURCE_DIR}/lib)

find_package(absl REQUIRED)
find_package(OrbbecSDK REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Threads REQUIRED)

add_subdirectory(proto)
add_subdirectory(arms)
add_subdirectory(camera)

if(ARMS_TYPE STREQUAL "tj")
    add_subdirectory(tj_sdk)
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
)

# Link OrbbecSDK and other necessary libraries
target_link_libraries(
    XLRobotBackend
    PRIVATE
    arms
    camera
    proto
)

if(ARMS_TYPE STREQUAL "tj")
    target_link_libraries(
        XLRobotBackend
        PRIVATE
        Kine
        MarvinSDK
    )
endif()
