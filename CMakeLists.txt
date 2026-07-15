cmake_minimum_required(VERSION 3.16..3.28)

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

project(fast_lio)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")

# OpenMP funciona en aarch64/Jetson. No debe limitarse a x86.
find_package(OpenMP REQUIRED COMPONENTS CXX)

# ROS 2 dependencies.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  pcl_ros
  pcl_conversions
  livox_ros_driver2
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Third-party dependencies.
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

message(STATUS "Eigen include: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "PCL includes: ${PCL_INCLUDE_DIRS}")
message(STATUS "OpenMP CXX flags: ${OpenMP_CXX_FLAGS}")

set(msg_files
  "msg/Pose6D.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(
  fastlio_mapping
  src/laserMapping.cpp
  include/ikd-Tree/ikd_Tree.cpp
  src/preprocess.cpp
)

target_include_directories(
  fastlio_mapping
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)

target_compile_definitions(
  fastlio_mapping
  PRIVATE
    MP_EN
    BOOST_BIND_GLOBAL_PLACEHOLDERS
)

target_compile_options(
  fastlio_mapping
  PRIVATE
    $<$<CONFIG:Release>:-O3>
)

ament_target_dependencies(
  fastlio_mapping
  ${dependencies}
)

# Debe utilizarse la firma simple porque
# ament_target_dependencies() también la utiliza internamente.
target_link_libraries(
  fastlio_mapping
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
)

list(APPEND EOL_LIST
  "foxy"
  "galactic"
  "eloquent"
  "dashing"
  "crystal"
)

if($ENV{ROS_DISTRO} IN_LIST EOL_LIST)
  rosidl_target_interfaces(
    fastlio_mapping
    ${PROJECT_NAME}
    "rosidl_typesupport_cpp"
  )
else()
  rosidl_get_typesupport_target(
    cpp_typesupport_target
    ${PROJECT_NAME}
    "rosidl_typesupport_cpp"
  )

  target_link_libraries(
    fastlio_mapping
    ${cpp_typesupport_target}
  )
endif()

install(
  TARGETS fastlio_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()