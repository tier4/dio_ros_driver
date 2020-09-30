cmake_minimum_required(VERSION 3.0.2)
project(dio_ros_driver)

# Compile as C++14, supported in ROS Melodic and newer.
# add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roslint
  roscpp
  std_msgs
  message_generation
  diagnostic_msgs
)
find_package(Boost REQUIRED
  COMPONENTS system
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  include
 )


add_message_files(
  FILES
  DIOPort.msg
)

generate_messages (
  DEPENDENCIES
  std_msgs
)
catkin_package()

set(SOURCE_FILES src/dio_ros_driver_node.cpp
                 src/dio_ros_driver.cpp
                 src/dio_accessor_base.cpp
                 src/din_accessor.cpp
                 src/dout_accessor.cpp
		 src/dio_diagnostic_updater.cpp
		 src/dio_accessor_diagnosis.cpp)
add_executable(${PROJECT_NAME}_node ${SOURCE_FILES})

add_dependencies(${PROJECT_NAME}_node
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  gpiod
)

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
  PATTERN ".git" EXCLUDE
)

install (
  DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
  PATTERN ".git" EXCLUDE
)
