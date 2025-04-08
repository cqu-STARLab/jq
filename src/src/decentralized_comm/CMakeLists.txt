cmake_minimum_required(VERSION 3.0.2)
project(decentralized_comm)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(Protobuf REQUIRED)  # ✅ 添加 Protobuf
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZeroMQ REQUIRED libzmq)  # ✅ 添加 ZeroMQ

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES decentralized_comm
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

## 设置 proto 文件路径
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")

## 自动编译 .proto 文件生成 .pb.cc 和 .pb.h
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${ProtoFiles})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${ZeroMQ_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)


link_directories(
  ${ZeroMQ_LIBRARY_DIRS}
)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto/robot_message.proto)

add_executable(robot_sender_node src/robot_sender_node.cpp ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(
  robot_sender_node 
  ${catkin_LIBRARIES} 
  ${Protobuf_LIBRARIES} 
  ${ZeroMQ_LIBRARIES}
)

add_executable(robot_receiver_node src/robot_receiver_node.cpp ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(
  robot_receiver_node 
  ${catkin_LIBRARIES} 
  ${Protobuf_LIBRARIES} 
  ${ZeroMQ_LIBRARIES}
)

add_executable(navigation_sender_node src/robot_navigation_sender_node.cpp ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(
  navigation_sender_node 
  ${catkin_LIBRARIES} 
  ${Protobuf_LIBRARIES} 
  ${ZeroMQ_LIBRARIES}
)

add_executable(navigation_receiver_node src/robot_navigation_receiver_node.cpp ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(
  navigation_receiver_node 
  ${catkin_LIBRARIES} 
  ${Protobuf_LIBRARIES} 
  ${ZeroMQ_LIBRARIES}
)