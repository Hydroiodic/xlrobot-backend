cmake_minimum_required(VERSION 3.10)
project(XLRobotBackend)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(ARMS_TYPE "${ARMS_TYPE}" CACHE STRING "Arms type (jaka or tj)")
if(NOT ARMS_TYPE)
    set(ARMS_TYPE "jaka")
endif()

set(ARMS_TYPE_INT 0)
if(ARMS_TYPE STREQUAL "jaka")
    set(ARMS_TYPE_INT 0)
elseif(ARMS_TYPE STREQUAL "tj")
    set(ARMS_TYPE_INT 1)
else()
    message(FATAL_ERROR "Unknown ARMS_TYPE='${ARMS_TYPE}', expected 'jaka' or 'tj'")
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_subdirectory(src)
