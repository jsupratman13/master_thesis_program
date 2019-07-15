cmake_minimum_required(VERSION 2.8.3)
project(master_thesis_program)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  rviz
  std_msgs
  message_generation
  #leptrino_force_torque
)

add_message_files(
  FILES
  Answers.msg
  Info.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS roscpp rospy message_runtime)

set(CMAKE_AUTOMOC ON)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add_library(${PROJECT_NAME} src/panel.cpp src/demo.cpp src/motion_creator.cpp src/cartesian_control.cpp src/joint_control.cpp)
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES})

#add_executable(user_interface src/gui.cpp src/demo.cpp src/motion_creator.cpp src/cartesian_control.cpp src/joint_control.cpp)
#target_link_libraries(user_interface ${catkin_LIBRARIES} ${QT_LIBRARIES})

add_executable(subjectB src/subjectB.cpp)
target_link_libraries(subjectB ${catkin_LIBRARIES} ${QT_LIBRARIES})

add_executable(subjectA src/subjectA.cpp)
target_link_libraries(subjectA ${catkin_LIBRARIES} ${QT_LIBRARIES})
