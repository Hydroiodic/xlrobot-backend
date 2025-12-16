find_package(absl REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Threads REQUIRED)

find_program(GRPC_CPP_PLUGIN grpc_cpp_plugin HINTS /usr/bin /usr/local/bin)
if(NOT GRPC_CPP_PLUGIN)
    message(
        FATAL_ERROR
        "grpc_cpp_plugin not found. Install gRPC tools or set GRPC_CPP_PLUGIN path."
    )
endif()

file(GLOB_RECURSE PROTO_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.proto")
if(NOT PROTO_FILES)
    message(FATAL_ERROR "No .proto files found in ${CMAKE_CURRENT_SOURCE_DIR}")
endif()

set(PROTO_SRCS "")
set(PROTO_HDRS "")
set(GRPC_SRCS "")
set(GRPC_HDRS "")

foreach(_PROTO_FILE ${PROTO_FILES})
    get_filename_component(_PROTO_NAME ${_PROTO_FILE} NAME_WE)
    get_filename_component(_PROTO_DIR ${_PROTO_FILE} DIRECTORY)

    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${_PROTO_DIR})

    set(_PB_SRC ${CMAKE_CURRENT_BINARY_DIR}/${_PROTO_DIR}/${_PROTO_NAME}.pb.cc)
    set(_PB_HDR ${CMAKE_CURRENT_BINARY_DIR}/${_PROTO_DIR}/${_PROTO_NAME}.pb.h)
    set(_GRPC_SRC ${CMAKE_CURRENT_BINARY_DIR}/${_PROTO_DIR}/${_PROTO_NAME}.grpc.pb.cc)
    set(_GRPC_HDR ${CMAKE_CURRENT_BINARY_DIR}/${_PROTO_DIR}/${_PROTO_NAME}.grpc.pb.h)

    add_custom_command(
        OUTPUT ${_PB_SRC} ${_PB_HDR}
        COMMAND ${Protobuf_PROTOC_EXECUTABLE}
        --cpp_out=${CMAKE_CURRENT_BINARY_DIR}
        -I ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/${_PROTO_FILE}
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${_PROTO_FILE}
        VERBATIM
    )

    add_custom_command(
        OUTPUT ${_GRPC_SRC} ${_GRPC_HDR}
        COMMAND ${Protobuf_PROTOC_EXECUTABLE}
        --grpc_out=${CMAKE_CURRENT_BINARY_DIR}
        --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}
        -I ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/${_PROTO_FILE}
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${_PROTO_FILE}
        VERBATIM
    )

    list(APPEND PROTO_SRCS ${_PB_SRC})
    list(APPEND PROTO_HDRS ${_PB_HDR})
    list(APPEND GRPC_SRCS ${_GRPC_SRC})
    list(APPEND GRPC_HDRS ${_GRPC_HDR})
endforeach()

# NOTE: grpc and gpr may not be necessary
find_package(gRPC CONFIG QUIET)
if(gRPC_FOUND)
    message(STATUS "Found gRPC CMake targets; using gRPC::grpc++")
    set(GRPC_LINK_TARGETS gRPC::grpc++ gRPC::grpc gRPC::gpr)
else()
    message(STATUS "gRPC CMake targets not found; falling back to explicit libs")
    set(GRPC_LINK_TARGETS grpc++ grpc gpr)
endif()

add_library(
    proto
    STATIC
    ${PROTO_SRCS}
    ${GRPC_SRCS}
)

target_include_directories(
    proto
    PUBLIC
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(
    proto
    PUBLIC
    ${GRPC_LINK_TARGETS}
    absl::synchronization
    protobuf::libprotobuf
    Threads::Threads
)

if(Protobuf_VERSION VERSION_GREATER_EQUAL 4)
    target_link_libraries(
        proto
        PUBLIC
        absl::cord
        absl::log_internal_check_op
    )
endif()
