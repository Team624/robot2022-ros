cmake_minimum_required(VERSION 2.8.3)
project(diff_drive)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  Linear.msg
  Angular.msg
  Constants.msg
  Goal.msg
  GoalPath.msg
  BoolArray.msg
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
#  add_rostest(tests/your_first_rostest.test)
#  add_rostest(tests/your_second_rostest.test)
endif()


catkin_python_setup()

generate_messages (
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs message_runtime
)
