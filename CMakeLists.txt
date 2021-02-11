cmake_minimum_required(VERSION 3.5.1)

project(rgd C CXX)

## set up test
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -std=c++14 -fPIC")


# Proto file
get_filename_component(rgd_proto "rgd.proto" ABSOLUTE)
get_filename_component(rgd_proto_path "${rgd_proto}" PATH)

# Generated sources
add_custom_target(proto ALL DEPENDS ${rgd_proto_srcs} ${rgd_proto_hdrs})
set(rgd_proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/rgd.pb.cc")
set(rgd_proto_hdrs "${CMAKE_CURRENT_BINARY_DIR}/rgd.pb.h")
add_custom_command(
  OUTPUT "${rgd_proto_srcs}" "${rgd_proto_hdrs}"
  COMMAND protoc
  ARGS  --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
    -I "${rgd_proto_path}"
    "${rgd_proto}"
  DEPENDS "${rgd_proto}")


include_directories("${CMAKE_CURRENT_BINARY_DIR}")


add_executable(z3test
  z3test.cc
  util.cc
  z3solver.cc
  ${rgd_proto_srcs}
)
target_link_libraries(z3test
  z3
  protobuf
  pthread
	boost_system
	boost_filesystem)
