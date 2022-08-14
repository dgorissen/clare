cmake_minimum_required(VERSION 3.0.2)
project(clare_head)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
   FILES
   FaceMessage.msg
   EarsMessage.msg
   EvoMessage.msg
   NoseMessage.msg
   IRMessage.msg
   ExpressionList.msg
)

add_service_files(
  FILES
  ListExpressions.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  message_runtime
)

catkin_install_python(PROGRAMS
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
