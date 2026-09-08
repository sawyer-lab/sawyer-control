cmake_minimum_required(VERSION 3.20)
project(sawyer_control_cpp_demo LANGUAGES CXX)

find_package(Protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)

set(PROTO ${CMAKE_CURRENT_LIST_DIR}/../../proto/sawyer_control/v1/control.proto)
set(GENERATED ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(PROTO_OUTPUT ${GENERATED}/sawyer_control/v1)
file(MAKE_DIRECTORY ${PROTO_OUTPUT})

add_custom_command(
    OUTPUT ${PROTO_OUTPUT}/control.pb.cc ${PROTO_OUTPUT}/control.grpc.pb.cc
    COMMAND protobuf::protoc
    ARGS --proto_path=${CMAKE_CURRENT_LIST_DIR}/../../proto
         --cpp_out=${GENERATED}
         --grpc_out=${GENERATED}
         --plugin=protoc-gen-grpc=$<TARGET_FILE:gRPC::grpc_cpp_plugin>
         ${PROTO}
    DEPENDS ${PROTO})

add_executable(sawyer_status main.cc ${PROTO_OUTPUT}/control.pb.cc ${PROTO_OUTPUT}/control.grpc.pb.cc)
target_include_directories(sawyer_status PRIVATE ${GENERATED})
target_link_libraries(sawyer_status PRIVATE gRPC::grpc++ protobuf::libprotobuf)
