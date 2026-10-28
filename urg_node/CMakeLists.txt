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
option(BUILD_URG "Build the URG (Hokuyo laser) driver" OFF)

# Build the UAM driver. Kept as a permanent option so downstream
# users/CI can selectively build only the driver(s) they need.
# Targets are added in a follow-up change.
option(BUILD_UAM "Build the UAM driver" OFF)

if(BUILD_URG)
  # URG targets are added in a follow-up change.
endif()

if(BUILD_UAM)
  # UAM targets are added in a follow-up change.
endif()

ament_package()
