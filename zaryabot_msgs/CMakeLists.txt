cmake_minimum_required(VERSION 2.8.3)
project(zaryabot_msgs)


find_package(catkin REQUIRED std_msgs message_generation)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES NumPunchState.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)


catkin_package(
  CATKIN_DEPENDS
  std_msgs
  message_runtime
)
