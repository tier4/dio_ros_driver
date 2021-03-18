cmake_minimum_required(VERSION 3.5)
project(dio_ros_driver)

# Compile as C++14, supported in ROS Melodic and newer.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find depended packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(Boost REQUIRED
  COMPONENTS system
)

#include_directories(
#  ${catkin_INCLUDE_DIRS}
  #include
 #)

# generate original message.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DIODebug.msg"
  "msg/DIOPort.msg"
  DEPENDENCIES builtin_interfaces
)

# build dio_ros_driver
#catkin_package()

#set(SOURCE_FILES src/dio_ros_driver_node.cpp
#                 src/dio_ros_driver.cpp
#                 src/dio_accessor_base.cpp
#                 src/din_accessor.cpp
#                 src/dout_accessor.cpp
#                 src/dio_diagnostic_updater.cpp
#                 src/dio_accessor_diagnosis.cpp)
#add_executable(${PROJECT_NAME}_node ${SOURCE_FILES})

#add_dependencies(${PROJECT_NAME}_node
# ${${PROJECT_NAME}_EXPORTED_TARGETS}
# ${catkin_EXPORTED_TARGETS}
#)

# use gpiod as link library.
#target_link_libraries(${PROJECT_NAME}_node
#  ${catkin_LIBRARIES}
#  gpiod
#)

# install node.
#install(TARGETS ${PROJECT_NAME}_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

# copy launch files to install/ directory.
#install(
#  DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#  PATTERN ".svn" EXCLUDE
#  PATTERN ".git" EXCLUDE
#)

# copy config files to install/ directory.
#install (
#  DIRECTORY config/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
#  PATTERN ".svn" EXCLUDE
#  PATTERN ".git" EXCLUDE
#)
