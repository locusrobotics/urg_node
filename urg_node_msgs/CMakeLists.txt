cmake_minimum_required(VERSION 3.0.2)
project(urg_node_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  DIRECTORY
    msg
  FILES
  Status.msg
  StatusArray.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
