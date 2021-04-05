cmake_minimum_required(VERSION 3.0.2)
project(l298n)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  message_generation
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
set (wiringPi_include "/use/local/include")

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MotorCommand.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetMotorDriver.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES l298n
  CATKIN_DEPENDS roscpp geometry_msgs message_runtime message_generation std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${wiringPi_include}
)

add_executable(l298n_node src/l298n.cpp)
add_dependencies(l298n_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(l298n_node ${catkin_LIBRARIES} wiringPi -lrt)
