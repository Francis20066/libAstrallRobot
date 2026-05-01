cmake_minimum_required(VERSION 3.20)
project(astrall_sim)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

if(NOT TARGET astrall_core)
  set(ASTRALL_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
  set(ASTRALL_BUILD_DEMOS OFF CACHE BOOL "" FORCE)
  set(ASTRALL_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(ASTRALL_ENABLE_SDK OFF CACHE BOOL "" FORCE)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../astrall_core ${CMAKE_CURRENT_BINARY_DIR}/astrall_core)
endif()

add_executable(astrall_sim_node src/astrall_sim_node.cpp)
target_link_libraries(astrall_sim_node
  PRIVATE
    astrall_core
  PUBLIC
    ${diagnostic_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs_library
    tf2_ros::tf2_ros
)

install(TARGETS astrall_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
