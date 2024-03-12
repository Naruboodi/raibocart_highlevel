cmake_minimum_required(VERSION 3.0.2)
project(raibo_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  interlock_fb.msg
  qr_location.msg
  speed_fb.msg
  speed_sp.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
 CATKIN_DEPENDS std_msgs message_runtime
)

###########
## Build ##
###########

#############
## Install ##
#############

#############
## Testing ##
#############
