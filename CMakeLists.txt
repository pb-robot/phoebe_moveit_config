cmake_minimum_required(VERSION 2.8.3)
project(phoebe_moveit_config)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  control_msgs
  std_msgs
  dynamixel_controllers
)

catkin_package()

add_executable(dynamixel_actionserver src/dynamixel_actionserver.cpp)

target_link_libraries(dynamixel_actionserver
  ${catkin_LIBRARIES}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

