include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(
    ARMS_SRCS
    ErrorParser.cpp
    timespec.cpp
)

if(ARMS_TYPE_INT EQUAL 0)
    list(APPEND ARMS_SRCS JakaServer.cpp)
elseif(ARMS_TYPE_INT EQUAL 1)
    list(APPEND ARMS_SRCS TjServer.cpp)
else()
    message(FATAL_ERROR "Unknown ARMS_TYPE='${ARMS_TYPE}', expected 'jaka' or 'tj'")
endif()

if(ARMS_TYPE_INT EQUAL 1)
    include(${CMAKE_SOURCE_DIR}/3rdparty/OrbbecSDK_v2/3rdparty/cmrc/src/cmake_resource.cmake)
    cmrc_add_resource_library(
        tj_config_rc
        NAMESPACE tj_config
        WHENCE ${CMAKE_SOURCE_DIR}/assets
        ${CMAKE_SOURCE_DIR}/assets/ccs_m6.MvKDCfg
    )
endif()

add_library(
    arms
    STATIC
    ${ARMS_SRCS}
)

target_include_directories(
    arms
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(
    arms
    PUBLIC
    proto
)

if(ARMS_TYPE_INT EQUAL 0)
    target_link_libraries(
        arms
        PUBLIC
        ${CMAKE_SOURCE_DIR}/lib/libjakaAPI.so
    )
endif()
if(ARMS_TYPE_INT EQUAL 1)
    target_link_libraries(arms PUBLIC tj_config_rc)
endif()
