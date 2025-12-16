cmake_minimum_required(VERSION 3.10)
project(XLRobotBackend)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(ARMS_TYPE "${ARMS_TYPE}" CACHE STRING "Arms type")
if(NOT ARMS_TYPE)
    set(ARMS_TYPE "")
endif()

set(CAMERA_TYPE "${CAMERA_TYPE}" CACHE STRING "Camera type")
if(NOT CAMERA_TYPE)
    set(CAMERA_TYPE "")
endif()

if(ARMS_TYPE STREQUAL "jaka")
    set(ARMS_TYPE_INT 0)
elseif(ARMS_TYPE STREQUAL "tj")
    set(ARMS_TYPE_INT 1)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/TJ_FX_ROBOT_CONTRL_SDK/include)
else()
    message(STATUS "Arms type not specified or unrecognized, arms functionality will be disabled.")
    set(ARMS_TYPE_INT -1)
endif()

if(CAMERA_TYPE STREQUAL "orbbec")
    set(CAMERA_TYPE_INT 0)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/OrbbecSDK_v2/include)
else()
    message(STATUS "Camera type not specified or unrecognized, camera functionality will be disabled.")
    set(CAMERA_TYPE_INT -1)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_subdirectory(patch)
add_subdirectory(3rdparty)
add_subdirectory(src)

install(
    TARGETS XLRobotBackend
    RUNTIME DESTINATION home
)

if(ARMS_TYPE_INT EQUAL 0)
    install(
        FILES ${CMAKE_SOURCE_DIR}/lib/libjakaAPI_2_3_0_12.so
        DESTINATION lib64
    )
    install(
        FILES ${CMAKE_SOURCE_DIR}/lib/libjakaAPI.so
        DESTINATION lib64
    )
endif()
if(ARMS_TYPE_INT EQUAL 1)
    install(
        FILES ${CMAKE_SOURCE_DIR}/assets/ccs_m6.MvKDCfg
        DESTINATION home
    )
endif()
