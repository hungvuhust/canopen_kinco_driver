cmake_minimum_required(VERSION 3.8)
project(canopen_kinco_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wpedantic -Wextra -Wno-unused-parameter)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

set(dependencies
  canopen_base_driver
  canopen_core
  canopen_interfaces
  rclcpp
  lely_core_libraries
  rclcpp_components
  sensor_msgs
  std_srvs
)

foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

# ==================================================================================================

add_library(lely_motion_controller_bridge_kinco SHARED
  src/motor.cpp
  src/command.cpp
  src/state.cpp
  src/default_homing_mode.cpp
)

target_compile_features(lely_motion_controller_bridge_kinco PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
target_compile_options(lely_motion_controller_bridge_kinco PUBLIC -Wl,--no-undefined)
target_include_directories(lely_motion_controller_bridge_kinco PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies( lely_motion_controller_bridge_kinco
  ${dependencies}
)

# ==================================================================================================




# ==================================================================================================

add_library(node_canopen_kinco_driver SHARED
  src/node_interfaces/node_canopen_kinco_driver.cpp
)
target_compile_features(node_canopen_kinco_driver PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
target_compile_options(node_canopen_kinco_driver PUBLIC -Wl,--no-undefined)
target_include_directories(node_canopen_kinco_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(node_canopen_kinco_driver
  lely_motion_controller_bridge_kinco
)
ament_target_dependencies(
  node_canopen_kinco_driver
  ${dependencies}
)

# ==================================================================================================




# ==================================================================================================

add_library(kinco_driver SHARED
  src/kinco_driver.cpp
)
target_compile_features(kinco_driver PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
target_compile_options(kinco_driver PUBLIC -Wl,--no-undefined)
target_include_directories(kinco_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(kinco_driver PUBLIC
  node_canopen_kinco_driver
  lely_motion_controller_bridge_kinco
)
ament_target_dependencies(
  kinco_driver PUBLIC
  ${dependencies}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(kinco_driver PUBLIC "CANOPEN_KINCO_DRIVER_BUILDING_LIBRARY")

rclcpp_components_register_nodes(kinco_driver "ros2_canopen::KincoDriver")
set(node_plugins "${node_plugins}ros2_canopen::KincoDriver;$<TARGET_FILE:kinco_driver>\n")

# ==================================================================================================


install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS kinco_driver node_canopen_kinco_driver lely_motion_controller_bridge_kinco
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  kinco_driver
  node_canopen_kinco_driver
  lely_motion_controller_bridge_kinco
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(
  ${dependencies}
)

ament_package()
