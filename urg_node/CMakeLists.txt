cmake_minimum_required(VERSION 3.5)
project(urg_node)

find_package(ament_cmake REQUIRED)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Build the URG (Hokuyo laser) driver. Kept as a permanent option so
# downstream users/CI can selectively build only the driver(s) they need.
# Targets are added in a follow-up change.
option(BUILD_URG "Build the URG (Hokuyo laser) driver" ON)

# Build the UAM driver. Kept as a permanent option so downstream
# users/CI can selectively build only the driver(s) they need.
# Targets are added in a follow-up change.
option(BUILD_UAM "Build the UAM driver" OFF)

if(BUILD_URG)
  # we currently still rely on boost for its `crc` and `shared_array` components
  find_package(Boost REQUIRED)
  find_package(diagnostic_updater REQUIRED)
  find_package(laser_proc REQUIRED)
  find_package(locus_cpp REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(urg_c REQUIRED)
  find_package(urg_node_msgs REQUIRED)

  add_library(urg_node_driver SHARED
    src/urg_c_wrapper.cpp
    src/urg_node_driver.cpp
    src/uam/workers/sensing_data/ar00_worker.cpp
  )
  target_include_directories(urg_node_driver
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(urg_node_driver PUBLIC
    Boost
    diagnostic_updater
    laser_proc
    locus_cpp
    rclcpp
    rclcpp_components
    sensor_msgs
    std_srvs
    urg_c
    urg_node_msgs
  )
  rclcpp_components_register_node(urg_node_driver
    PLUGIN "urg_node::UrgNode"
    EXECUTABLE urg_node)

  add_executable(getID src/getID.cpp)
  target_link_libraries(getID urg_node_driver)

  install(TARGETS urg_node_driver getID
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY include/urg_node include/uam
    DESTINATION include
  )

  install(PROGRAMS scripts/set_urg_ip.py
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY urdf meshes
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(BUILD_UAM)
  # UAM targets are added in a follow-up change.
endif()

if(BUILD_TESTING AND BUILD_URG)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

if(BUILD_URG)
  ament_export_dependencies(urg_node_msgs)
  ament_export_dependencies(
    diagnostic_updater
    laser_proc
    locus_cpp
    rclcpp
    sensor_msgs
    std_srvs
    urg_c
  )
  ament_export_include_directories(include)
  ament_export_libraries(urg_node_driver)
endif()

ament_package()
