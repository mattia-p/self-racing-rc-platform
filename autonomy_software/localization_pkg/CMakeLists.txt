cmake_minimum_required(VERSION 3.13.3)
project(localization_pkg)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  self_racing_car_msgs
  dynamic_reconfigure_pkg
  dynamic_reconfigure
)

# pkg-config to find GeographicLib
find_package(PkgConfig REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_pkg_INCLUDE_DIRS}
  /usr/include/GeographicLib
)

add_executable(vehicle_state_publisher_cpp_node src/vehicle_state_publisher.cpp)

target_link_libraries(vehicle_state_publisher_cpp_node
  ${catkin_LIBRARIES}
  Geographic
)

catkin_install_python(PROGRAMS
  src/rtk_gps_serial_reader.py
  src/vehicle_state_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
