cmake_minimum_required(VERSION 3.5)
project(mujoco_ros2_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)
find_package(joint_limits REQUIRED)
find_package(urdf REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Local IXWebSocket - build from source
add_subdirectory(IXWebSocket)

# System nlohmann_json
find_package(nlohmann_json REQUIRED)

set(THIS_PACKAGE_DEPENDS
  ament_cmake
  rclcpp
  hardware_interface
  pluginlib
  controller_manager
  joint_limits
  urdf
  glfw3
  control_toolbox
  nav_msgs
)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# MuJoCo no longer required - robot description comes from WebSocket server
message(STATUS "Building without MuJoCo dependency - using WebSocket robot description")

add_library(mujoco_system_plugins SHARED src/mujoco_system.cpp)
ament_target_dependencies(mujoco_system_plugins ${THIS_PACKAGE_DEPENDS})
target_link_libraries(mujoco_system_plugins
    ixwebsocket
    nlohmann_json::nlohmann_json
)
target_include_directories(mujoco_system_plugins
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR})

install(TARGETS
  mujoco_system_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# TODO: make it simple
add_executable(mujoco_ros2_control
  src/mujoco_ros2_control_node.cpp
  src/mujoco_ros2_control.cpp
)
ament_target_dependencies(mujoco_ros2_control ${THIS_PACKAGE_DEPENDS})
target_link_libraries(mujoco_ros2_control
    glfw
    cv_bridge::cv_bridge
    ixwebsocket
    nlohmann_json::nlohmann_json
)
target_include_directories(mujoco_ros2_control
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR})

install(TARGETS
  mujoco_ros2_control
  DESTINATION lib/${PROJECT_NAME})

pluginlib_export_plugin_description_file(mujoco_ros2_control mujoco_system_plugins.xml)
ament_package()
