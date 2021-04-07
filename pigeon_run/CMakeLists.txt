cmake_minimum_required(VERSION 3.0.2)
project(pigeon_run)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  motor_driver_msgs
)

find_package(Threads)

add_library(screen
  src/ftxui/screen/box.cpp
  src/ftxui/screen/screen.cpp
  src/ftxui/screen/string.cpp
  src/ftxui/screen/terminal.cpp
  src/ftxui/screen/wcwidth.cpp
  include/ftxui/screen/box.hpp
  include/ftxui/screen/color.hpp
  include/ftxui/screen/screen.hpp
  include/ftxui/screen/string.hpp
)

add_library(dom
  include/ftxui/dom/elements.hpp
  include/ftxui/dom/node.hpp
  include/ftxui/dom/requirement.hpp
  include/ftxui/dom/take_any_args.hpp
  src/ftxui/dom/blink.cpp
  src/ftxui/dom/bold.cpp
  src/ftxui/dom/border.cpp
  src/ftxui/dom/color.cpp
  src/ftxui/dom/composite_decorator.cpp
  src/ftxui/dom/dbox.cpp
  src/ftxui/dom/dim.cpp
  src/ftxui/dom/flex.cpp
  src/ftxui/dom/frame.cpp
  src/ftxui/dom/gauge.cpp
  src/ftxui/dom/graph.cpp
  src/ftxui/dom/hbox.cpp
  src/ftxui/dom/hflow.cpp
  src/ftxui/dom/inverted.cpp
  src/ftxui/dom/node.cpp
  src/ftxui/dom/node_decorator.cpp
  src/ftxui/dom/paragraph.cpp
  src/ftxui/dom/separator.cpp
  src/ftxui/dom/size.cpp
  src/ftxui/dom/spinner.cpp
  src/ftxui/dom/text.cpp
  src/ftxui/dom/underlined.cpp
  src/ftxui/dom/util.cpp
  src/ftxui/dom/vbox.cpp
)

add_library(component
  src/ftxui/component/checkbox.cpp
  src/ftxui/component/component.cpp
  src/ftxui/component/container.cpp
  src/ftxui/component/event.cpp
  src/ftxui/component/input.cpp
  src/ftxui/component/menu.cpp
  src/ftxui/component/radiobox.cpp
  src/ftxui/component/radiobox.cpp
  src/ftxui/component/screen_interactive.cpp
  src/ftxui/component/toggle.cpp
  include/ftxui/component/checkbox.hpp
  include/ftxui/component/component.hpp
  include/ftxui/component/container.hpp
  include/ftxui/component/event.hpp
  include/ftxui/component/input.hpp
  include/ftxui/component/menu.hpp
  include/ftxui/component/radiobox.hpp
  include/ftxui/component/screen_interactive.hpp
  include/ftxui/component/receiver.hpp
  include/ftxui/component/toggle.hpp
)

#add_library(pigeon_robot_steering
#    src/pigeon_robot_steering.cpp
#    include/pigeon_tui/pigeon_robot_steering.h
#    )

add_library(pigeon_terminal
    include/pigeon_terminal/pigeon_terminal.h
    src/pigeon_terminal/pigeon_terminal.cpp
    )

#add_library(pigeon::pigeon_terminal ALIAS pigeon_terminal)

add_library(ftxui::screen ALIAS screen)
add_library(ftxui::dom ALIAS dom)
add_library(ftxui::component ALIAS component)

target_link_libraries(dom PUBLIC screen)
target_link_libraries(component PUBLIC dom Threads::Threads)

foreach(lib screen dom component)
  target_include_directories(${lib}
    PUBLIC
      $<INSTALL_INTERFACE:include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
      src
  )

# C++17 is used. We requires fold expression at least.
set_property(TARGET ${lib} PROPERTY CXX_STANDARD 17)

# Force Microsoft Visual Studio to decode sources files in UTF-8. This applies
# to the library and the library users.
if (MSVC)
  target_compile_options(${lib} PUBLIC "/utf-8")
endif()

# Add as many warning as possible:
if (WIN32)
  if (MSVC)
    target_compile_options(${lib} PRIVATE "/W4")
    target_compile_options(${lib} PRIVATE "/WX")
    target_compile_options(${lib} PRIVATE "/wd4244")
    target_compile_options(${lib} PRIVATE "/wd4267")
  endif()
  # Force Win32 to UNICODE
  target_compile_definitions(${lib} PRIVATE UNICODE _UNICODE)
else()
  target_compile_options(${lib} PRIVATE "-Wall")
  target_compile_options(${lib} PRIVATE "-Wextra")
  target_compile_options(${lib} PRIVATE "-pedantic")
  target_compile_options(${lib} PRIVATE "-Werror")
  target_compile_options(${lib} PRIVATE "-Wno-sign-compare")
endif()

endforeach()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MotorCommand.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

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

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pigeon_tui
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(motor_driver_tester_node src/motor_driver_tester.cpp)
target_link_libraries(motor_driver_tester_node PRIVATE pigeon_terminal PRIVATE screen PRIVATE component PRIVATE gtest PRIVATE gmock PRIVATE gtest_main ${catkin_LIBRARIES})


