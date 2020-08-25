cmake_minimum_required(VERSION 3.0.2)
project(dio_ros_driver)

# Compile as C++14, supported in ROS Melodic and newer.
# add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roslint
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
 )

set(SOURCE_FILES src/dio_ros_driver_node.cpp
                 src/dio_ros_driver.cpp
                 src/dio_user_handler.c)
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
