cmake_minimum_required(VERSION 3.5)

project(rochu_gripper_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(
  DIRECTORY 
    msg
  FILES
    GripperMode.msg
    GripperRequest.msg
    GripperState.msg
)

generate_messages(DEPENDENCIES)

catkin_package(CATKIN_DEPENDS message_runtime)
