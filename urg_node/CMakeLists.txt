cmake_minimum_required(VERSION 3.5.0)
project(urg_node)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater dynamic_reconfigure laser_proc
  nodelet rosconsole roscpp sensor_msgs std_msgs std_srvs tf urg_c
  json_transport
  urg_node_msgs
  visualization_msgs
)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/URG.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uam_workers uam_driver urg_c_wrapper urg_node_driver
  CATKIN_DEPENDS dynamic_reconfigure laser_proc nodelet rosconsole
  roscpp sensor_msgs std_msgs std_srvs urg_c visualization_msgs
  DEPENDS
)


include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(uam_workers 
  src/uam/workers/sensing_data/ar00_worker.cpp
  src/uam/workers/sensing_data/ar01_worker.cpp
  src/uam/workers/sensing_data/ar06_worker.cpp
  src/uam/workers/vr00_worker.cpp
  src/uam/workers/xr00_worker.cpp
  src/uam/workers/id00_worker.cpp
  src/uam/workers/uam_command_worker.cpp
  src/uam/io/tcp_client.cpp
  )
target_link_libraries(uam_workers ${catkin_LIBRARIES})

add_library(uam_driver
 src/uam/uam_driver.cpp)
target_link_libraries(uam_driver uam_workers ${catkin_LIBRARIES})

add_library(urg_c_wrapper src/urg_c_wrapper.cpp)
target_link_libraries(urg_c_wrapper uam_workers ${catkin_LIBRARIES})

# Declare the cpp as a lib also
add_library(urg_node_driver src/urg_node_driver.cpp)
add_dependencies(urg_node_driver ${PROJECT_NAME}_gencfg)
target_link_libraries(urg_node_driver urg_c_wrapper ${catkin_LIBRARIES})


## Declare a cpp executable
add_executable(urg_node src/urg_node.cpp)
target_link_libraries(urg_node urg_node_driver ${catkin_LIBRARIES})

## Declare getID tool
add_executable(getID src/getID.cpp)
target_link_libraries(getID urg_c_wrapper ${catkin_LIBRARIES})

add_executable (uam_driver_ros_node 
 src/uam/uam_driver_ros_params.cpp
 src/uam/uam_driver_ros_node.cpp
 src/uam/uam_driver_ros.cpp
 )
add_dependencies(uam_driver_ros_node ${PROJECT_NAME}_gencfg )
target_link_libraries(uam_driver_ros_node uam_driver ${catkin_LIBRARIES})
 
add_executable (uam_get_configuration_id 
 src/uam/uam_get_safety_configuration_id.cpp
 )
add_dependencies(uam_get_configuration_id ${PROJECT_NAME}_gencfg)
target_link_libraries(uam_get_configuration_id uam_driver ${catkin_LIBRARIES})


add_executable (uam_status_aggregator_node 
 src/uam/uam_status_aggregator_node.cpp
 )
target_link_libraries(uam_status_aggregator_node ${catkin_LIBRARIES})




## Mark executables and/or libraries for installation
install(TARGETS uam_workers uam_driver urg_c_wrapper urg_node urg_node_driver getID 
                uam_driver_ros_node uam_get_configuration_id uam_status_aggregator_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY include/uam/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Install script to set IP addresses
catkin_install_python(PROGRAMS
   scripts/set_urg_ip.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY urdf meshes launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint)
  # Ignore long -> int32_t warning, urg_c driver uses
  # long, etc. Wrapper must also use these
  set(ROSLINT_CPP_OPTS "--filter=-build/c++11,-runtime/references")
  roslint_cpp()
  roslint_add_test()

  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/urg_lidar.launch)
endif()
