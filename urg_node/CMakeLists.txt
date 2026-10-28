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
option(BUILD_URG "Build the URG (Hokuyo laser) driver" ON)

# Build the UAM driver. Kept as a permanent option so downstream
# users/CI can selectively build only the driver(s) they need.
option(BUILD_UAM "Build the UAM driver" ON)

find_package(Boost REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(urg_node_msgs REQUIRED)

if(BUILD_URG)
  find_package(diagnostic_updater REQUIRED)
  find_package(laser_proc REQUIRED)
  find_package(urg_c REQUIRED)

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
  find_package(visualization_msgs REQUIRED)
  # uam_get_configuration_id's command-line parsing and JSON output needs
  # these; they are not needed by the rest of the UAM driver.
  find_package(Boost REQUIRED COMPONENTS program_options filesystem)
  find_package(nlohmann_json REQUIRED)

  # Low-level UAM protocol workers and TCP transport. ar00_worker.cpp is also
  # compiled into urg_node_driver above (URG needs it too, for its AR00 status
  # query) so that BUILD_URG does not require BUILD_UAM; the small amount of
  # duplicated compilation when both options are enabled is an acceptable
  # trade-off for keeping the two options independent.
  add_library(uam_workers SHARED
    src/uam/workers/sensing_data/ar00_worker.cpp
    src/uam/workers/sensing_data/ar01_worker.cpp
    src/uam/workers/sensing_data/ar06_worker.cpp
    src/uam/workers/vr00_worker.cpp
    src/uam/workers/xr00_worker.cpp
    src/uam/workers/id00_worker.cpp
    src/uam/workers/uam_command_worker.cpp
    src/uam/io/tcp_client.cpp
  )
  target_include_directories(uam_workers
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(uam_workers PUBLIC Boost rclcpp)

  add_library(uam_driver SHARED src/uam/uam_driver.cpp)
  target_link_libraries(uam_driver PUBLIC uam_workers)
  ament_target_dependencies(uam_driver PUBLIC sensor_msgs urg_node_msgs)

  add_library(uam_driver_ros SHARED
    src/uam/uam_driver_ros.cpp
    src/uam/uam_driver_ros_params.cpp
  )
  target_link_libraries(uam_driver_ros PUBLIC uam_driver)
  ament_target_dependencies(uam_driver_ros PUBLIC
    rclcpp
    rclcpp_components
    std_srvs
    visualization_msgs
  )
  rclcpp_components_register_node(uam_driver_ros
    PLUGIN "uam::UamROS"
    EXECUTABLE uam_driver_ros_node
    EXECUTOR MultiThreadedExecutor)
  # A MultiThreadedExecutor (not the default SingleThreadedExecutor) is
  # required here: UamROS::lidarHardReset() blocks a timer callback while
  # waiting for a service response, and that response is only delivered if
  # another executor thread is free to process it concurrently (via the
  # node's dedicated reentrant callback group).

  add_executable(uam_get_configuration_id src/uam/uam_get_safety_configuration_id.cpp)
  target_link_libraries(uam_get_configuration_id
    uam_driver
    Boost::program_options
    Boost::filesystem
    nlohmann_json::nlohmann_json
  )

  add_executable(uam_status_aggregator_node src/uam/uam_status_aggregator_node.cpp)
  ament_target_dependencies(uam_status_aggregator_node PUBLIC rclcpp urg_node_msgs)

  install(TARGETS
      uam_workers
      uam_driver
      uam_driver_ros
      uam_get_configuration_id
      uam_status_aggregator_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  if(NOT BUILD_URG)
    install(DIRECTORY include/urg_node include/uam
      DESTINATION include
    )
  endif()
endif()

if(BUILD_TESTING AND (BUILD_URG OR BUILD_UAM))
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

if(BUILD_URG OR BUILD_UAM)
  ament_export_dependencies(
    Boost
    rclcpp
    rclcpp_components
    sensor_msgs
    std_srvs
    urg_node_msgs
  )
  ament_export_include_directories(include)
endif()

if(BUILD_URG)
  ament_export_dependencies(
    diagnostic_updater
    laser_proc
    urg_c
  )
  ament_export_libraries(urg_node_driver)
endif()

if(BUILD_UAM)
  ament_export_dependencies(visualization_msgs)
  ament_export_libraries(uam_workers uam_driver uam_driver_ros)
endif()

ament_package()
